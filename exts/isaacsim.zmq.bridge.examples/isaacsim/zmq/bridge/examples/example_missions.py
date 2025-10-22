# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import asyncio
import time


import carb
import numpy as np
import omni.usd
from isaacsim.core.api.robots import Robot
from isaacsim.core.prims import XFormPrim
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.core.utils.types import ArticulationAction
from isaacsim.robot.manipulators.examples.franka import Franka
from isaacsim.robot.manipulators.examples.franka.controllers.rmpflow_controller import (
    RMPFlowController,
)
import isaacsim.core.utils.stage as stage_utils
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path
from isaacsim.util.debug_draw import _debug_draw
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade

from . import EXT_NAME, ZMQAnnotator
from .mission import Mission

# The omni.__proto__ namespace is created by this extention
# read more at coreproto_util.py
from omni.__proto__ import server_control_message_pb2

class FrankaVisionMission(Mission):
    """Mission that demonstrates a Franka robot with vision capabilities.

    This mission sets up a Franka robot with a camera and enables control via ZMQ.
    It streams camera data and allows external control of the robot's end effector.
    """

    name = "FrankaVisionMission"
    world_usd_path = "franka_world.usda"

    def __init__(self, server_ip: str = "localhost"):
        Mission.__init__(self, server_ip=server_ip)
        self.server_ip = server_ip

        # Scene setup parameters
        self.scene_root = "/World"
        self._camera_path = None
        self._camera_prim = None

        self.draw = _debug_draw.acquire_debug_draw_interface()

        self.cur_focal_length = 20

        # Simulation parameters
        self.physics_dt = 60.0  # Rate of the physics simulation
        self.camera_hz = 60.0  # Do not go above physics_dt!
        self.dimension_x = 720
        self.dimension_y = 720

        # Camera setup
        self.camera_annotator = None
        self.camera_annotators = []

        self.use_ogn_nodes = True  # True > use OGN C++ node, False > use Python

        # Target position randomization
        self.last_trigger_time = 0
        _seed = 1234
        self.rng = np.random.default_rng(_seed)

    def start_mission(self) -> None:
        """Start the mission by setting up ZMQ communication and camera streaming.

        This method initializes ZMQ sockets, sets up the camera annotator, and starts
        the command reception loops for various control channels.
        """
        # Define communication ports for different data streams
        self.ports = {
            "camera_annotator": 5561,
            "camera_control_command": 5557,
            "settings": 5559,
            "franka": 5560,
        }

        # Set up ZMQ sockets for receiving commands
        self.camera_control_socket = self.zmq_client.get_pull_socket(self.ports["camera_control_command"])
        self.settings_socket = self.zmq_client.get_pull_socket(self.ports["settings"])
        self.franka_socket = self.zmq_client.get_pull_socket(self.ports["franka"])

        # Set up camera for streaming
        self._camera_path = "/World/camera/y_link/Camera"
        stage = omni.usd.get_context().get_stage()
        self._camera_prim = stage.GetPrimAtPath(self._camera_path)

        # Enable command reception
        self.receive_commands = True

        # Create camera annotator for streaming camera data
        self.camera_annotator = ZMQAnnotator(
            self._camera_path,
            (self.dimension_x, self.dimension_y),
            use_ogn_nodes=self.use_ogn_nodes,
            server_ip=self.server_ip,
            port=self.ports["camera_annotator"],
        )
        self.camera_annotators.append(self.camera_annotator)

        # If not using OGN nodes, set up Python-based streaming
        if not self.use_ogn_nodes:
            print(f"[{EXT_NAME}] Using Python-based streaming")
            self.camera_annot_sock_pub = self.zmq_client.get_push_socket(self.ports["camera_annotator"])
            self.camera_annotator.sock = self.camera_annot_sock_pub
            self.zmq_client.add_physx_step_callback(
                "camera_annotator", 1 / self.camera_hz, self.camera_annotator.stream
            )

        # Set up async receive loops for all command channels
        self.subscribe_to_protobuf_in_loop(
            self.camera_control_socket,
            server_control_message_pb2.ServerControlMessage,
            self.camera_control_sub_loop,
        )
        self.subscribe_to_protobuf_in_loop(
            self.settings_socket,
            server_control_message_pb2.ServerControlMessage,
            self.settings_sub_loop,
        )
        self.subscribe_to_protobuf_in_loop(
            self.franka_socket,
            server_control_message_pb2.ServerControlMessage,
            self.franka_sub_loop
        )

    async def stop_mission_async(self) -> None:
        """Stop the mission and clean up resources.

        This method stops the simulation, disconnects ZMQ sockets, and destroys annotators.
        """
        if not hasattr(self, "world"):
            carb.log_warn(f"[{EXT_NAME}] world was not initialized.")
            return

        await self.world.stop_async()
        self.receive_commands = False
        self.zmq_client.remove_physx_callbacks()
        # must wait for all callbacks to finish before disconnecting from the server
        await asyncio.sleep(0.5)
        await self.zmq_client.disconnect_all()
        # must wait for all client to disconnect before destroying the annotators
        await asyncio.sleep(0.5)
        if self.world.is_stopped():
            for annotator in self.camera_annotators:
                # annotator._rp.hydra_texture.set_updates_enabled(False)
                annotator.destroy()
                # annotator._rp.hydra_texture.set_updates_enabled(True)
        else:
            carb.log_warn(f"[{EXT_NAME}] Cant destory annotators while simulation is running!")

    def stop_mission(self) -> None:
        asyncio.ensure_future(self.stop_mission_async())

    def camera_control_sub_loop(self, proto_msg: server_control_message_pb2.ServerControlMessage) -> None:
        """Handle camera control commands received.

        Processes camera mount joints velocities and focal length adjustments from the incoming message.
        Applies joint velocities to the camera mount and updates the focal length if changed.

        Args:
            proto_msg: ServerControlMessage containing a CameraControlCommand
        """
        new_velocities = (0, 0, 0)
        if proto_msg.HasField("camera_control_command"):
            joints_vel = proto_msg.camera_control_command.joints_vel
            new_velocities = (joints_vel.x, joints_vel.y, joints_vel.z)
            focal_length = proto_msg.camera_control_command.focal_length

            if focal_length != self.cur_focal_length:
                try:
                    focalLength_attr = self._camera_prim.GetAttribute("focalLength")
                    focalLength_attr.Set(focal_length)
                    self.cur_focal_length = focal_length
                except:
                    carb.log_warn(f"[{EXT_NAME}] Failed to set focal length")
                    pass

        if self.world.is_playing():
            try:
                self.camera_controller.apply_action(
                    ArticulationAction(
                        joint_positions=None,
                        joint_efforts=None,
                        joint_velocities=[new_velocities[0], new_velocities[1], new_velocities[2]],
                    )
                )
            except:
                print(traceback.format_exc())
                print(new_velocities)
                carb.log_warn(f"[{EXT_NAME}] unable to apply action to camera")

    def settings_sub_loop(self, proto_msg: server_control_message_pb2.ServerControlMessage) -> None:
        """General purpose control loop to tweak parameters of the simulator

        Args:
            proto_msg: ServerControlMessage containing a ControlCommand
        """
        if proto_msg.HasField("settings_command"):
            self.zmq_client.adaptive_rate = proto_msg.settings_command.adaptive_rate

    def franka_sub_loop(self, proto_msg: server_control_message_pb2.ServerControlMessage) -> None:
        """Handle Franka robot commands received via ZMQ.

        Controls the Franka robot's end effector position using RMPFlow and
        randomizes the target position every 5 seconds.

        Args:
            proto_msg: ServerControlMessage containing a FrankaCommand
        """
        # Default position if no command is received
        new_effector_pos = [0, 0, 0]
        self.draw.clear_points()

        if proto_msg.HasField("franka_command"):
            effector_pos = proto_msg.franka_command.effector_pos
            new_effector_pos = [effector_pos.x, effector_pos.y, effector_pos.z]

        if self.world.is_playing():
            try:
                # Move end effector to target position:
                # Position is computed from the server
                # Orientation is computed from ground truth
                rot_gt = self.target.get_world_poses()[1][0]
                actions = self.rmpf_controller.forward(
                    target_end_effector_position=np.array(new_effector_pos),
                    target_end_effector_orientation=rot_gt,
                )
                self.franka_articulation_controller.apply_action(actions)
                if proto_msg.franka_command.show_marker:
                    self.draw.draw_points([new_effector_pos], [(0, 0, 1, 1)], [10])
            except Exception as e:
                carb.log_warn(f"[{EXT_NAME}] Error applying action: {e}")

        # randomize the target position every 8 seconds :)
        current_time = time.time()
        if current_time - self.last_trigger_time > 8:
            lower_bounds = np.array([0.2, -0.2, 0.1])
            upper_bounds = np.array([0.6, 0.2, 0.5])
            random_array = self.rng.uniform(lower_bounds, upper_bounds)
            self.target.set_world_poses(positions=np.array([random_array]))
            self.last_trigger_time = current_time

    def reset_franka_mission(self) -> None:
        """Reset the Franka robot and its controller."""
        self.franka = Franka(prim_path="/World/Franka")
        self.franka.initialize()
        self.rmpf_controller = RMPFlowController(name="target_follower_controller", robot_articulation=self.franka)
        self.franka_articulation_controller = self.franka.get_articulation_controller()
        self.target = XFormPrim(prim_paths_expr="/World/Target")
        rot = euler_angles_to_quat((-180, 0, -180), degrees=True)
        self.target.set_world_poses(orientations=np.array([rot]))

    def before_reset_world(self) -> None:
        """Prepare the world for reset.

        This method is called before resetting the world to set up the camera robot.
        """
        self.draw.clear_points()
        self.camera_robot = Robot(prim_path=f"/World/camera", name="robot")
        self.world.scene.add(self.camera_robot)

    def after_reset_world(self) -> None:
        """Execute operations after the world has been reset.

        This method is called after resetting the world to set up controllers and the Franka robot.
        """
        self.zmq_client.simulation_start_timecode = time.time()
        self.camera_controller = self.camera_robot.get_articulation_controller()
        self.meters_per_unit = self.world.scene.stage.GetMetadata(UsdGeom.Tokens.metersPerUnit)
        self.reset_franka_mission()

    @classmethod
    def add_franka(cls) -> None:
        """Add a Franka robot to the scene as reference"""
        root = get_assets_root_path()
        franka_usd = root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        stage_utils.add_reference_to_stage(
            usd_path=franka_usd,
            prim_path="/World/Franka"
        )

    @classmethod
    async def _async_load(cls) -> None:
        """Load the mission asynchronously."""
        await Mission._async_load(cls.mission_usd_path())
        cls.add_franka()
        await asyncio.sleep(0.5)
        omni.kit.selection.SelectNoneCommand().do()

    @classmethod
    def load_mission(cls) -> None:
        """Load the mission synchronously."""
        Mission.load_mission(cls.mission_usd_path())
        cls.add_franka()
        omni.kit.selection.SelectNoneCommand().do()


class FrankaMultiVisionMission(FrankaVisionMission):
    """FrankaVisionMission mission with a second camera attached to the gripper.

    This mission extends FrankaVisionMission by adding a second camera
    attached to the Franka robot's gripper.
    """

    name = "FrankaMultiVisionMission"
    world_usd_path = "franka_multi_cam_world.usda"

    @classmethod
    def add_franka(cls) -> None:
        """Add a Franka robot with an additional gripper camera to the scene."""
        super().add_franka()
        cls.gripper_camera_prim_path = "/World/Franka/panda_hand/gripper_camera"
        gripper_camera = Camera(prim_path=cls.gripper_camera_prim_path)
        gripper_camera.set_clipping_range(0.01, 10000)
        gripper_camera.set_visibility(False)

        gripper_camera.set_lens_distortion_model("OmniLensDistortionFthetaAPI")

        pos = (0.1, 0.0, 0)
        rot = euler_angles_to_quat((190, 0, 0), degrees=True)
        gripper_camera_xform = XFormPrim(prim_paths_expr="/World/Franka/panda_hand/gripper_camera")
        gripper_camera_xform.set_local_poses(translations=np.array([pos]), orientations=np.array([rot]))



    def start_mission(self) -> None:
        """Start the mission with multiple cameras.

        This method extends the parent class implementation by adding
        a second camera annotator for the gripper camera.
        """
        super().start_mission()

        self.ports["gripper_annotator"] = 5591

        self.gripper_annotator = ZMQAnnotator(
            self.gripper_camera_prim_path,
            (self.dimension_x, self.dimension_y),
            use_ogn_nodes=self.use_ogn_nodes,
            server_ip=self.server_ip,
            port=self.ports["gripper_annotator"],
        )
        self.camera_annotators.append(self.gripper_annotator)

        # If not using OGN nodes, set up Python-based streaming
        if not self.use_ogn_nodes:
            print(f"[{EXT_NAME}] Using Python-based streaming")
            self.gripper_annot_sock_pub = self.zmq_client.get_push_socket(self.ports["gripper_annotator"])
            self.gripper_annotator.sock = self.gripper_annot_sock_pub
            self.zmq_client.add_physx_step_callback(
                "gripper_annotator", 1 / self.camera_hz, self.gripper_annotator.stream
            )
