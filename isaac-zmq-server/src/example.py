# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import argparse
import math
import sys
import time
import traceback
from pprint import pprint

import dearpygui.dearpygui as dpg
import numpy as np

from isaac_zmq_server.cv import (
    CameraToWorldSpaceTransform,
    colorize_depth,
    draw_bounding_boxes,
)
from isaac_zmq_server.server import ZMQServer
from isaac_zmq_server.ui import App

import client_stream_message_pb2
import server_control_message_pb2

parser = argparse.ArgumentParser(description="Isaac Sim ZMQ Client Example")
parser.add_argument("--port", type=int, default=5561, help="Port to subscribe data on")
parser.add_argument("--subscribe_only", type=int, default=0, help="1 to only subscribe to data, 0 to publish and subscribe")
parser.add_argument("--resolution_x", type=int, default=720, help="Image resolution x")
parser.add_argument("--resolution_y", type=int, default=720, help="Image resolution y")
args = parser.parse_args()

# Set up configuration based on arguments
SUBSCRIBE_ONLY = bool(args.subscribe_only)
PORT = args.port
RESOLUTION_X = args.resolution_x
RESOLUTION_Y = args.resolution_y

if SUBSCRIBE_ONLY:
    print("Server is in subscribe only mode at port: {}, resolution: {}x{}".format(PORT, RESOLUTION_X, RESOLUTION_Y))
else:
    print("Server is in publish and subscribe mode at port: {}, resolution: {}x{}".format(PORT, RESOLUTION_X, RESOLUTION_Y))


class FrankaVisionMission(App):
    """
    GUI application for visualizing data from the Franka robot Mission.

    This class extends the App base class to create a specific application for the
    Franka robot mission. It handles receiving camera data, depth information, and
    bounding box detections (ground truth) from Isaac Sim, and provides controls to send commands
    back to the simulation, forming a SIL (software in the loop).
    """

    def __init__(self):

        App.__init__(self)

        # UI configuration
        self.dimmention = (RESOLUTION_X, RESOLUTION_Y)
        self.expected_size = self.dimmention[0] * self.dimmention[1] * 4
        self.hz = 60  # Target refresh rate

        self.window_name = f"Isaac Sim ZMQ Server {RESOLUTION_X}x{RESOLUTION_Y}@{PORT}"
        if SUBSCRIBE_ONLY:
            self.window_name += " (Subscribe Only)"

        self.window_width = self.dimmention[0]
        self.window_height = self.dimmention[1] + 80

        # Camera parameters
        self.camera_range = [20, 200]
        self.current_camera_f = 20
        self.active_camera = "/World/base_link/y_link/Camera"

        # Timing and performance tracking
        self.app_start_time = None
        self.app_time = 0
        self.sim_time_interval = 0
        self.sim_time_accumulated = 0
        self.mesure_interval = 1  # seconds
        self.num_receive_annotations = 0
        self.actual_rate = 10  # fps - will get updated from the client

        # Data storage
        self.texture_data = np.zeros((self.dimmention[1], self.dimmention[0], 4), dtype=np.float32)
        self.depth_data = np.zeros((self.dimmention[1], self.dimmention[0], 4), dtype=np.uint8)
        self.current_camera_command = [0, 0, 0]

        self.camera_to_world = CameraToWorldSpaceTransform((self.dimmention[0], self.dimmention[1]))

        self.debug_start_time = None

    def create_app_body(self):
        with dpg.texture_registry(show=False):
            dpg.add_raw_texture(
                self.dimmention[0],  # width
                self.dimmention[1],  # height
                self.texture_data,
                tag="image_stream",
                format=dpg.mvFormat_Float_rgba,
            )

        with dpg.window(tag="Main Window"):
            dpg.add_image("image_stream")

            times = [
                ("sim_time", "sim_time_label", "Sim Time", "Simulated pyhsical time"),
                ("sim_hz", "sim_hz_label", "Sim Rate", "Simulation stream rate"),
                ("actual_hz", "actual_hz_label", "Actual Rate", "Actual incoming rate"),
            ]
            for index, (tag, label_tag, label, tip) in enumerate(times):
                with dpg.value_registry():
                    dpg.add_string_value(tag=tag, default_value="0.0")
                dpg.add_text(label, pos=(10, 10 + (20 * index)))
                dpg.add_text(source=tag, label=tag, pos=(120, 10 + (20 * index)), tag=label_tag)
                with dpg.tooltip(label_tag):
                    dpg.add_text(tip)

            dpg.add_separator()

            with dpg.group():
                dpg.add_text("Control Camera with arrows", show=not SUBSCRIBE_ONLY)

                with dpg.group(horizontal=True):
                    dpg.add_text("Adeptive rate", show=not SUBSCRIBE_ONLY)
                    dpg.add_checkbox(default_value=True, tag="adeptive_rate", show=not SUBSCRIBE_ONLY)
                    dpg.add_text("Ground Truth")
                    dpg.add_combo(
                        items=["RGB", "BBOX2D", "DEPTH"],
                        default_value="BBOX2D",
                        width=100,
                        tag="ground_truth_mode",
                    )
                    dpg.add_text("Focal Length", show=not SUBSCRIBE_ONLY)
                    dpg.add_slider_float(
                        tag="zoom",
                        default_value=20,
                        min_value=self.camera_range[0],
                        max_value=self.camera_range[1],
                        width=80,
                        show=not SUBSCRIBE_ONLY,
                    )
                    dpg.add_text("Draw Detection on World", show=not SUBSCRIBE_ONLY)
                    dpg.add_checkbox(default_value=True, tag="draw_detection_on_world", show=not SUBSCRIBE_ONLY)

                    with dpg.tooltip("adeptive_rate"):
                        dpg.add_text("Adjust stream rate to real time")
                    with dpg.tooltip("draw_detection_on_world"):
                        dpg.add_text("Find world position of detection center and draw it on the scene")

        with dpg.handler_registry():
            dpg.add_key_down_handler(callback=self.key_press_evnet)
            dpg.add_key_release_handler(callback=self.key_depress_evnet)
            dpg.add_mouse_wheel_handler(callback=self.mouse_wheel_evnet)

        dpg.set_primary_window("Main Window", True)

    def create_network_iface(self) -> None:
        """
        Creates a network interface for the mission.

        Initializes the ports for camera annotator, camera link, and focal length.
        Sets up the ZMQ server to receive annotations from the camera annotator port.
        Sets up the ZMQ server to send commands to the focal length and camera link ports.
        """
        self.app_time = time.monotonic()

        # Define ports for different communication channels
        self.ports = {
            "camera_annotator": PORT,
            "camera_control_command": 5557,
            "settings": 5559,
            "franka": 5560,
        }

        # Create ZMQ server instance
        self.zmq_server = ZMQServer()

        # Set up receiver for camera data
        self.zmq_server.subscribe_to_socket_in_loop(
            "camera_annotator",
            self.ports["camera_annotator"],
            self.process_annotations,
        )

        # Set up senders for various commands (if not in receive-only mode)
        if not SUBSCRIBE_ONLY:
            self.zmq_server.publish_protobuf_in_loop(
                "camera_control_command",
                self.ports["camera_control_command"],
                self.hz,
                self.camera_control_command,
            )

            self.zmq_server.publish_protobuf_in_loop(
                "settings",
                self.ports["settings"],
                self.hz,
                self.settings_command,
            )

            self.zmq_server.publish_protobuf_in_loop(
                "franka",
                self.ports["franka"],
                self.hz,
                self.franka_command,
            )

    def proto_bbox_data_to_dict(self, bbox2d_data) -> dict:
        """
        Convert protobuf bounding box data to a Python dictionary.
        source: isaac_bridge_zmq_stream_pb2.BBox2D

        Args:
            bbox2d_data: Protobuf BBox2D message

        Returns:
            dict: Dictionary containing bounding box data
        """
        # Extract the list of bounding boxes
        json_bbox_data = []
        for bbox in bbox2d_data.data:
            bbox_dict = {
                "semanticId": str(bbox.semanticId),
                "xMin": bbox.xMin,
                "yMin": bbox.yMin,
                "xMax": bbox.xMax,
                "yMax": bbox.yMax,
                "occlusionRatio": bbox.occlusionRatio,
            }
            json_bbox_data.append(bbox_dict)

        # Extract idToLabels map and bounding box IDs
        id_to_labels = dict(bbox2d_data.info.idToLabels)
        json_bbox_ids = list(bbox2d_data.info.bboxIds)

        # Combine into final dictionary structure
        bbox2d_info = {"idToLabels": id_to_labels, "bboxIds": json_bbox_ids}
        bbox2d_data = {"data": json_bbox_data, "info": bbox2d_info}
        return bbox2d_data

    def proto_camera_data_to_dict(self, camera_data) -> dict:
        """
        Convert protobuf camera data to a Python dictionary.
        source: isaac_bridge_zmq_stream_pb2.Camera
        Args:
            camera_data: Protobuf Camera message

        Returns:
            dict: Dictionary containing camera matrices and parameters
        """
        # Convert view matrix from flat array to 4x4 matrix
        view_matrix_ros_flat = camera_data.view_matrix_ros
        view_matrix_list = [view_matrix_ros_flat[i : i + 4] for i in range(0, 16, 4)]

        # Get camera scale
        scale_list = list(camera_data.camera_scale)

        # Convert intrinsics matrix from flat array to 3x3 matrix
        intrinsics_flat = camera_data.intrinsics_matrix
        intrinsics_list = [intrinsics_flat[i : i + 3] for i in range(0, 9, 3)]

        # Combine into final dictionary structure
        camera_data = {
            "view_matrix_ros": view_matrix_list,
            "camera_scale": scale_list,
            "intrinsics_matrix": intrinsics_list,
        }
        return camera_data

    def process_annotations(self, message: str) -> None:
        """
        Receive and process annotations from the message.

        Extracts image data, bounding box data, depth data, camera data, and delta time from the message.
        Updates `texture_data` with the image data.
        Calculates the center of the bounding box in world coordinates.

        message:
            client_stream_message_pb2.ClientStreamMessage @ proto/client_stream_message.proto

        """
        if not self.debug_start_time:
            self.debug_start_time = time.monotonic()

        client_stream = client_stream_message_pb2.ClientStreamMessage()

        # Deserialize the message
        client_stream.ParseFromString(message)

        dt = client_stream.clock.sim_dt
        sim_time = client_stream.clock.sim_time
        timecode = client_stream.clock.sys_time

        img_data = client_stream.color_image  # bytes
        depth_data = client_stream.depth_image  # bytes

        bbox2d_data = self.proto_bbox_data_to_dict(client_stream.bbox2d)
        camera_data = self.proto_camera_data_to_dict(client_stream.camera)

        self.rates_debug(sim_time, timecode)

        if len(img_data) != self.expected_size:
            print(f"Received image data of size {len(img_data)}, expected {self.expected_size}")
            return

        if dpg.get_value("ground_truth_mode") in ["BBOX2D", "RGB"]:
            # Reshape to height x width x channels
            img_array = np.frombuffer(img_data, dtype=np.uint8).reshape(self.dimmention[1], self.dimmention[0], 4)

            if dpg.get_value("ground_truth_mode") == "BBOX2D":
                try:
                    img_array = draw_bounding_boxes(img_array, bbox2d_data)
                except:
                    print(traceback.format_exc())

        elif dpg.get_value("ground_truth_mode") == "DEPTH":
            # Reshape to height x width (rows x cols)
            img_array = np.frombuffer(depth_data, dtype=np.float32).reshape(self.dimmention[1], self.dimmention[0], 1)
            try:
                img_array = colorize_depth(img_array)
            except:
                print(traceback.format_exc())

        np.divide(img_array, 255.0, out=self.texture_data)

        if not SUBSCRIBE_ONLY:
            interseting_bbox = self.get_interseting_bbox(bbox2d_data)
            self.camera_to_world.get_bbox_center_in_world_coords(interseting_bbox, depth_data, camera_data, device="cuda")

    def get_interseting_bbox(self, bbox_data: dict) -> None:
        index = -1
        interseting = ["class:object"]
        if bbox_data["info"].get("idToLabels"):
            for k, v in bbox_data["info"]["idToLabels"].items():
                if v in interseting:
                    index = k
                    break

            for box in bbox_data["data"]:
                if box["semanticId"] == index:
                    return {"data": box}

        return {"data": []}

    def rates_debug(self, sim_time: float, timecode: float) -> None:
        # print("[isaac-zmq-server] Message transit time: {:.4f}".format(time.time() - timecode))

        try:
            self.sim_time_interval = sim_time - self.sim_time_accumulated
            self.num_receive_annotations += 1
            sim_rate = self.num_receive_annotations / self.sim_time_interval
        except ZeroDivisionError:
            sim_rate = 0
            print("[isaac-zmq-server] Sim time interval is zero.")

        dpg.set_value("sim_time", str("{:.2f}".format(sim_time)))

        current_time = time.monotonic()

        if current_time - self.app_time > self.mesure_interval:
            self.app_time = current_time
            self.actual_rate = self.num_receive_annotations / self.mesure_interval

            self.num_receive_annotations = 0
            self.sim_time_interval = 0
            self.sim_time_accumulated = sim_time

            dpg.set_value("sim_hz", str("{:.2f}".format(sim_rate)))
            dpg.set_value("actual_hz", str("{:.2f}".format(self.actual_rate)))

    def camera_control_command(self) -> server_control_message_pb2.ServerControlMessage:
        """
        Creates camera control commands with mount joint velocities and focal length.

        Scales camera movement based on zoom level and smoothly adjusts focal length
        toward the target value.

        Returns:
            server_control_message_pb2.ServerControlMessage: A protobuf message with camera control parameters
        """
        factor = np.interp(dpg.get_value("zoom"), self.camera_range, [0.3, 1.2])
        command_x = self.current_camera_command[0] * factor
        command_y = self.current_camera_command[1] * factor
        command_z = self.current_camera_command[2] * factor

        # Create a ServerControlMessage with a RobotCommand
        message = server_control_message_pb2.ServerControlMessage()

        # Set camera_link vector
        message.camera_control_command.joints_vel.x = command_x
        message.camera_control_command.joints_vel.y = command_y
        message.camera_control_command.joints_vel.z = command_z

        def smooth_step(current: float, target: float, min_step=0.5, max_step=4, smoothness=0.1):
            diff = abs(target - current)
            # Use a sigmoid function for smooth interpolation
            factor = 1 / (1 + math.exp(-diff / smoothness))
            step = min_step + (max_step - min_step) * factor
            return step

        target_zoom = dpg.get_value("zoom")
        if self.current_camera_f != target_zoom:
            step = smooth_step(self.current_camera_f, target_zoom)
            if self.current_camera_f < target_zoom:
                self.current_camera_f = min(self.current_camera_f + step, target_zoom)
            else:
                self.current_camera_f = max(self.current_camera_f - step, target_zoom)

        message.camera_control_command.focal_length = self.current_camera_f

        return message

    def settings_command(self) -> server_control_message_pb2.ServerControlMessage:
        """
        Generates a control command with debug position, adaptive rate, and active camera information.

        Returns:
            server_control_message_pb2.ServerControlMessage: A protobuf message containing control parameters.
        """
        # Create a ServerControlMessage with a ControlCommand
        message = server_control_message_pb2.ServerControlMessage()
        # Set other control parameters
        message.settings_command.adaptive_rate = dpg.get_value("adeptive_rate")

        return message

    def franka_command(self) -> server_control_message_pb2.ServerControlMessage:
        """
        Generates a command for the Franka robot's effector position.

        Returns:
            server_control_message_pb2.ServerControlMessage: A protobuf message containing the effector position.
        """
        # Create a ServerControlMessage with a FrankaCommand
        message = server_control_message_pb2.ServerControlMessage()

        # Set effector_pos vector
        message.franka_command.effector_pos.x = self.camera_to_world.detection_world_pos[0]
        message.franka_command.effector_pos.y = self.camera_to_world.detection_world_pos[1]
        message.franka_command.effector_pos.z = self.camera_to_world.detection_world_pos[2]

        # Set show_marker
        if dpg.get_value("draw_detection_on_world"):
            message.franka_command.show_marker = True
        else:
            message.franka_command.show_marker = False

        return message

    def mouse_wheel_evnet(self, sender: int, app_data: int) -> None:
        new_value = dpg.get_value("zoom") + (app_data * 5)
        new_value = max(min(new_value, 200), 20)
        dpg.set_value("zoom", new_value)

    def key_press_evnet(self, sender, app_data) -> None:
        if dpg.is_key_down(dpg.mvKey_Up):
            self.current_camera_command = [0, 0, 1]
        elif dpg.is_key_down(dpg.mvKey_Down):
            self.current_camera_command = [0, 0, -1]
        elif dpg.is_key_down(dpg.mvKey_Left):
            self.current_camera_command = [0, -1, 0]
        elif dpg.is_key_down(dpg.mvKey_Right):
            self.current_camera_command = [0, 1, 0]

    def key_depress_evnet(self, sender: int, app_data: int) -> None:
        self.current_camera_command = [0, 0, 0]

    def _cleanup(self) -> None:
        super()._cleanup()


FrankaVisionMission.run_app()
