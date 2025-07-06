# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import asyncio
import time
import traceback

import zmq

import carb
import omni.graph.core as og
import omni.replicator.core as rep
import omni.usd
from isaacsim.core.api.sensors import BaseSensor
from isaacsim.core.prims import XFormPrim
from isaacsim.sensors.camera import Camera
from omni.kit.viewport.utility import get_active_viewport_window
from omni.replicator.core.scripts.utils import viewport_manager
from omni.syntheticdata import SyntheticData

from .. import EXT_NAME

# The omni.__proto__ namespace is created by this extention
# read more at core.proto_util.py
from omni.__proto__ import client_stream_message_pb2


class ZMQAnnotator:
    """
    Captures camera data and streams it via ZMQ.

    This class creates annotators for RGB, depth, and bounding box data from a camera,
    and streams this data to external applications via ZMQ. It can operate in two modes:
    - OGN node mode (C++ implementation)
    - Python mode (fallback implementation)
    """

    def __init__(
        self,
        camera: str,
        resolution: tuple,
        use_ogn_nodes: bool = False,
        server_ip: str = "localhost",
        port: int = 5555,
        rgb: bool = True,
        depth: bool = True,
        bbox: bool = True,
    ):
        """
        Initializes a ZMQAnnotator object.

        Args:
            camera (str): The path to the camera prim in the scene
            resolution (tuple): The resolution of the camera's render product (width, height)
            use_ogn_nodes (bool): Whether to use OGN nodes (C++ implementation)
            server_ip (str): The IP address of the ZMQ server
            port (int): The port to use for ZMQ communication
        """
        self.use_ogn_nodes = use_ogn_nodes
        self.server_ip = server_ip
        self.port = port
        self.resolution = resolution

        # Get stage and synthetic data interface
        self.stage = omni.usd.get_context().get_stage()
        self.sdg_iface = SyntheticData.Get()

        # Set device based on mode (CUDA for OGN nodes, CPU for Python)
        device = "cuda" if self.use_ogn_nodes else "cpu"

        # Create render product for the camera
        name = f"{camera.split('/')[-1]}_{port}_rp"
        self._rp = viewport_manager.get_render_product(camera, resolution, False, name)
        self.rp = self._rp.hydra_texture.get_render_product_path()

        # Create annotators for different data types
        self.annotators = {}
        if rgb:
            self.rgb_annot = rep.AnnotatorRegistry.get_annotator("rgb", device=device)
            self.rgb_annot.attach(self.rp)
            self.annotators["rgb"] = self.rgb_annot
        if depth:
            self.distance_to_camera_annot = rep.AnnotatorRegistry.get_annotator("distance_to_camera", device=device)
            self.distance_to_camera_annot.attach(self.rp)
            self.annotators["distance_to_camera"] = self.distance_to_camera_annot
        if bbox:
            # Note: bbox is not supported on GPU, so always use CPU
            self.bbox2d_annot = rep.AnnotatorRegistry.get_annotator("bounding_box_2d_tight_fast")
            self.bbox2d_annot.attach(self.rp)
            self.annotators["bounding_box_2d_tight_fast"] = self.bbox2d_annot

        # in C++ mode we stream using the omniGraph nodes
        # in Python mode we stream in .stream() method in this class
        if self.use_ogn_nodes:
            self.build_graph(name, camera)
        else:
            # Attache to Camera Prim for Python mode
            self.camera_xform = XFormPrim(camera)
            self.camera = Camera(
                prim_path=camera,
                render_product_path=self.rp,
                resolution=resolution,
            )
            BaseSensor.initialize(self.camera)
            # Not calling full initialization (self.camera.initialize()),
            # since we've taken care of that above
            self.last_error_time = 0  # for throttling error rate

        print(f"[{EXT_NAME}] [Port: {self.port}] Constructed annotator.")

    def build_graph(self, rp_name: str, camera_path: str) -> None:
        """
        Build the OGN graph for streaming camera data.

        This method creates the OGN nodes needed for streaming of camera data

        Args:
            rp_name (str): The name of the render product
            camera_path (str): The path to the camera prim
        """
        # we are extanding the already existing synthetic data data graph
        _graph_path = SyntheticData._graphPathRoot + "/" + SyntheticData._postProcessGraphPath
        self.graph = og.Controller.graph(_graph_path)

        # get the graph dispatcher node
        dispacher_node = self.graph.get_node(_graph_path + "/PostProcessDispatcher")

        # create/assign physics step, sync and time nodes
        _physics_nodes = {
            "physics_step": {"node_type": "isaacsim.core.nodes.OnPhysicsStep", "node": None},
            "sync": {"node_type": "omni.graph.action.RationalTimeSyncGate", "node": None},
            "sim_time": {"node_type": "isaacsim.core.nodes.IsaacReadSimulationTime", "node": None},
            "sys_time": {"node_type": "isaacsim.core.nodes.IsaacReadSystemTime", "node": None},
        }
        for node_name, _ in _physics_nodes.items():
            node = self.graph.get_node(_graph_path + f"/{node_name}")
            if not node:
                node = self.graph.create_node(_graph_path + f"/{node_name}", _["node_type"], True)
            else:
                carb.log_warn(f"{node_name} node already exists")
            _["node"] = node

        # assign to vars for clarity
        sync_node = _physics_nodes["sync"]["node"]
        p_step = _physics_nodes["physics_step"]["node"]
        sim_time = _physics_nodes["sim_time"]["node"]
        sys_time = _physics_nodes["sys_time"]["node"]

        # connect dispacth to sync node
        dispacher_node.get_attribute("outputs:referenceTimeDenominator").connect(
            sync_node.get_attribute("inputs:rationalTimeDenominator"), True
        )
        dispacher_node.get_attribute("outputs:referenceTimeNumerator").connect(
            sync_node.get_attribute("inputs:rationalTimeNumerator"), True
        )

        # create zmq node
        zmq_ = self.graph.create_node(
            _graph_path + "/zmq{}".format(self.port), "isaacsim.zmq.bridge.OgnIsaacBridgeZMQNode", True
        )
        zmq_.get_attribute("inputs:port").set(self.port)
        zmq_.get_attribute("inputs:ip").set(self.server_ip)

        # create camera info node
        camera_ = self.graph.create_node(
            _graph_path + "/camera{}".format(self.port), "isaacsim.zmq.bridge.OgnIsaacBridgeZMQCamera", True
        )
        camera_.get_attribute("inputs:cameraPrimPath").set(camera_path)
        camera_.get_attribute("inputs:width").set(self.resolution[0])
        camera_.get_attribute("inputs:height").set(self.resolution[1])
        dispacher_node.get_attribute("outputs:exec").connect(camera_.get_attribute("inputs:execIn"), True)

        # connect camera info node to zmq node
        camera_.get_attribute("outputs:cameraViewTransform").connect(
            zmq_.get_attribute("inputs:cameraViewTransform"), True
        )
        camera_.get_attribute("outputs:cameraIntrinsics").connect(zmq_.get_attribute("inputs:cameraIntrinsics"), True)
        camera_.get_attribute("outputs:cameraWorldScale").connect(zmq_.get_attribute("inputs:cameraWorldScale"), True)

        # get the annotator nodes and connect them to the zmq node
        pfx = f"/{rp_name}_"
        sufx = "buffPtr"
        annot_var_mapping = {}
        if self.annotators.get("rgb"):
            annot_var_mapping["rgb"] = {
                "node_name": f"{pfx}LdrColorSD{sufx}",
                "attr_suffix": "Color",
                "attrs": ["bufferSize", "dataPtr"],
            }
        if self.annotators.get("distance_to_camera"):
            annot_var_mapping["distance_to_camera"] = {
                "node_name": f"{pfx}DistanceToCameraSD{sufx}",
                "attr_suffix": "Depth",
                "attrs": ["bufferSize", "dataPtr"],
            }
        if self.annotators.get("bounding_box_2d_tight_fast"):
            annot_var_mapping["bounding_box_2d_tight_fast"] = {
                "node_name": f"{pfx}bounding_box_2d_tight_fast",
                "attr_suffix": "BBox2d",
                "attrs": ["bboxIds", "bufferSize", "data", "height", "width", "primPaths", "labels", "ids"],
            }
        for an, _params in annot_var_mapping.items():
            ptr_node = self.graph.get_node(_graph_path + _params["node_name"])
            ptr_node.get_attribute("outputs:exec").connect(sync_node.get_attribute("inputs:execIn"), True)
            for p in _params["attrs"]:
                target_attr = zmq_.get_attribute(f"inputs:{p}{_params['attr_suffix']}")
                ptr_node.get_attribute(f"outputs:{p}").connect(target_attr, True)

        # connect physics and clock nodes to zmq node
        p_step.get_attribute("outputs:deltaSimulationTime").connect(
            zmq_.get_attribute("inputs:deltaSimulationTime"), True
        )
        p_step.get_attribute("outputs:deltaSystemTime").connect(zmq_.get_attribute("inputs:deltaSystemTime"), True)

        p_step.get_attribute("outputs:step").connect(sync_node.get_attribute("inputs:execIn"), True)
        sim_time.get_attribute("outputs:simulationTime").connect(zmq_.get_attribute("inputs:simulationTime"), True)
        sys_time.get_attribute("outputs:systemTime").connect(zmq_.get_attribute("inputs:systemTime"), True)

        # connect sync node to zmq node to trigger the stream
        sync_node.get_attribute("outputs:execOut").connect(zmq_.get_attribute("inputs:execIn"), True)

        self.nodes = [p_step, sync_node, sim_time, sys_time, zmq_, camera_]

    def destroy(self) -> None:
        """
        Clean up resources used by the annotator.

        This method detaches all annotators from the render product,
        destroys OGN nodes if they were created, and destroys the render product.
        """
        if self.use_ogn_nodes:
            for node in self.nodes:
                try:
                    _p = node.get_prim_path()
                    print(f"[{EXT_NAME}] Removing node: {_p}")
                    self.graph.destroy_node(_p, True)
                except:
                    carb.log_warn("Node {} not found".format(node))
            self.nodes = []

        self.rgb_annot.detach(self.rp)
        self.bbox2d_annot.detach(self.rp)
        self.distance_to_camera_annot.detach(self.rp)

        # Causes instability, not recommended when kit is controlling the main thread
        # self._rp.destroy()

        print(f"[{EXT_NAME}] [port {self.port}] Annotators destroyed.")

    def stream(self, dt: float, sim_time: float) -> float:
        """
        ** Used in Python mode Only **

        Capture and stream data from the camera and annotators.

        This method is called each step (or rate-limited) to capture RGB, depth, and bounding box data
        from the camera, package it into a protobuf message, and send it via ZMQ.

        Args:
            dt (float): The time difference since the last call
            sim_time (float): The current simulation time

        Returns:
            float: Execution time in seconds

        Note:
            The protobuf message is defined in
            ClientStreamMessage @ proto/client_stream_message.proto

        """
        start_time = time.monotonic()
        # https://docs.omniverse.nvidia.com/extensions/latest/ext_replicator/annotators_details.html#bounding-box-2d-tight

        # Create protobuf message
        client_stream = client_stream_message_pb2.ClientStreamMessage()

        # Get bounding box data (performance intensive operation)
        bbox2d_data = self.bbox2d_annot.get_data()

        # Fill BBox2D information
        bbox2d_info = client_stream_message_pb2.BBox2DInfo()
        bbox2d_info.bboxIds.extend(bbox2d_data["info"]["bboxIds"].tolist())
        for key, value in bbox2d_data["info"]["idToLabels"].items():
            bbox2d_info.idToLabels[str(key)] = f"class:{next(iter(value.values()))}"
        client_stream.bbox2d.info.CopyFrom(bbox2d_info)

        # Fill BBox2D data
        for data in bbox2d_data["data"]:
            bbox2d_type = client_stream.bbox2d.data.add()
            bbox2d_type.semanticId = data[0]
            bbox2d_type.xMin = data[1]
            bbox2d_type.yMin = data[2]
            bbox2d_type.xMax = data[3]
            bbox2d_type.yMax = data[4]
            bbox2d_type.occlusionRatio = data[5]

        # Fill Camera information
        camera = client_stream_message_pb2.Camera()
        view_matrix = self.camera.get_view_matrix_ros()
        camera.view_matrix_ros.extend(view_matrix.flatten().tolist())
        try:
            intrinsics_matrix = self.camera.get_intrinsics_matrix()
            camera.intrinsics_matrix.extend(intrinsics_matrix.flatten().tolist())
        except:
            # Camera.get_intrinsics_matrix() will throw exception for non pinhole cameras
            # I this case, we will not stream camera data
            carb.log_verbose(traceback.format_exc())
        camera.camera_scale.extend(self.camera_xform.get_world_scales()[0].tolist())
        client_stream.camera.CopyFrom(camera)

        # Fill Clock information
        clock = client_stream_message_pb2.Clock()
        clock.sim_dt = dt
        clock.sys_dt = 0  # not simply accessible via python
        clock.sim_time = sim_time
        clock.sys_time = time.time()
        client_stream.clock.CopyFrom(clock)

        # Fill RGB image data
        client_stream.color_image = self.rgb_annot.get_data().tobytes()

        # Fill Depth image data
        client_stream.depth_image = self.distance_to_camera_annot.get_data().tobytes()

        # Serialize and send the message
        message = client_stream.SerializeToString()

        # send message with error throttling if not connected to a server
        async def graceful_send():
            try:
                await self.sock.send(message)
            except zmq.Again:
                if sim_time - self.last_error_time > 5.0:
                    carb.log_warn("Failed to send message (no server available)")
                    self.last_error_time = sim_time
            except asyncio.exceptions.CancelledError:
                pass  # expected error when user stops the streaming
            except:
                # unexpected error
                carb.log_warn(traceback.format_exc())

        asyncio.ensure_future(graceful_send())

        # Calculate and return execution time
        exec_time = time.monotonic() - start_time
        return exec_time
