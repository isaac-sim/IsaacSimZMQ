# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import asyncio
import time

import carb
import isaacsim.core.experimental.utils.app as app_utils
import isaacsim.core.utils.stage as stage_utils
import isaacsim.robot_motion.experimental.motion_generation as mg
import numpy as np
import omni.timeline
import omni.usd
import warp as wp
from isaacsim.core.experimental.prims import Articulation, GeomPrim, XformPrim
from isaacsim.core.simulation_manager import SimulationManager
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.robot_motion.cumotion import (
    CumotionWorldInterface,
    RmpFlowController,
    load_cumotion_supported_robot,
)
from isaacsim.sensors.camera import Camera
from isaacsim.storage.native import get_assets_root_path
from isaacsim.util.debug_draw import _debug_draw

# The omni.__proto__ namespace is created by this extention
# read more at coreproto_util.py
from omni.__proto__ import server_control_message_pb2
from pxr import Gf, Sdf, Tf, Usd, UsdGeom, UsdPhysics, UsdShade

from . import EXT_NAME, ZMQAnnotator
from .mission import Mission


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

        # cuMotion robot config (constant across resets)
        _cumotion_robot = load_cumotion_supported_robot("franka")
        self._cumotion_robot = _cumotion_robot
        self._site_space = _cumotion_robot.robot_description.tool_frame_names()
        self._tool_frame = self._site_space[0]

        # cuMotion per-command sync gates; both transforms are committed once
        self._static_robot = True  # Set to False for a mobile base
        self._static_scene = True  # Set to True when obstacles never move

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
            self.zmq_client.add_physics_step_callback(
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
        # Coalesce franka commands at physics-step rate:
        # ZMQ stores the latest message;
        # _franka_physics_step consumes it once per step.
        self._latest_franka_msg = None
        self.subscribe_to_protobuf_in_loop(
            self.franka_socket, server_control_message_pb2.ServerControlMessage, self._franka_store_latest
        )
        self.zmq_client.add_physics_step_callback("franka_control", 1 / self.physics_dt, self._franka_physics_step)

    async def stop_mission_async(self) -> None:
        """Stop the mission and clean up resources.

        This method stops the simulation, disconnects ZMQ sockets, and destroys annotators.
        """
        omni.timeline.get_timeline_interface().stop()
        self.receive_commands = False
        self.zmq_client.remove_physics_callbacks()
        # must wait for all callbacks to finish before disconnecting from the server
        await asyncio.sleep(0.5)
        await self.zmq_client.disconnect_all()
        # must wait for all client to disconnect before destroying the annotators
        await asyncio.sleep(0.5)
        if app_utils.is_stopped():
            for annotator in self.camera_annotators:
                annotator.destroy()
        else:
            carb.log_warn(f"[{EXT_NAME}] Cant destroy annotators while simulation is running!")

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

        if app_utils.is_playing():
            try:
                # Ideally physics ops are coalesced onto a physics-step callback
                # Negligible for this 3-DOF write.
                self.camera_robot.set_dof_velocity_targets(
                    np.array([new_velocities[0], new_velocities[1], new_velocities[2]])
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

    def _franka_store_latest(self, proto_msg: server_control_message_pb2.ServerControlMessage) -> None:
        """ZMQ-loop sink: stash the latest franka command for the physics step."""
        self._latest_franka_msg = proto_msg

    def _franka_physics_step(self, dt: float, sim_time: float) -> None:
        """Apply the most recent franka command on the physics step (drops older)."""
        msg = self._latest_franka_msg
        if msg is None:
            return
        self._latest_franka_msg = None
        self.franka_sub_loop(msg)

    def franka_sub_loop(self, proto_msg: server_control_message_pb2.ServerControlMessage) -> None:
        """Apply a franka command (called from _franka_physics_step, not the ZMQ loop).

        Controls the Franka end effector via cuMotion RMPFlow and randomizes the
        target every 8 seconds.
        Args:
            proto_msg: ServerControlMessage containing a FrankaCommand
        """
        # Default position if no command is received
        new_effector_pos = [0, 0, 0]
        self.draw.clear_points()

        if proto_msg.HasField("franka_command"):
            effector_pos = proto_msg.franka_command.effector_pos
            new_effector_pos = [effector_pos.x, effector_pos.y, effector_pos.z]

        if app_utils.is_playing():
            try:
                # Move end effector to target position:
                # Position is computed from the server
                # Orientation is computed from ground truth
                _, target_orientations = self.target.get_world_poses()
                target_positions = wp.array([new_effector_pos], dtype=wp.float32)
                setpoint = mg.RobotState(
                    sites=mg.SpatialState.from_name(
                        spatial_space=[self._tool_frame],
                        positions=([self._tool_frame], target_positions),
                        orientations=([self._tool_frame], target_orientations),
                    ),
                )
                names = self._dof_names
                estimated = mg.RobotState(
                    joints=mg.JointState.from_name(
                        robot_joint_space=names,
                        positions=(names, self.franka.get_dof_positions()),
                        velocities=(names, self.franka.get_dof_velocities()),
                    )
                )
                t = SimulationManager.get_simulation_time()
                if self._rmpflow_reset_needed:
                    if not self.rmpf_controller.reset(estimated, setpoint, t=t):
                        return
                    self._rmpflow_reset_needed = False
                if not self._static_robot:
                    self._world_binding.get_world_interface().update_world_to_robot_root_transforms(
                        self.franka.get_world_poses()
                    )
                if not self._static_scene:
                    self._world_binding.synchronize_transforms()
                desired = self.rmpf_controller.forward(estimated, setpoint, t)
                if desired is not None and desired.joints.positions is not None:
                    self.franka.set_dof_position_targets(
                        positions=desired.joints.positions,
                        dof_indices=desired.joints.position_indices,
                    )
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
        self.franka = Articulation("/World/Franka")
        self._rmpflow_reset_needed = True

        # dof_names is constant for an articulation's lifetime; cache it once.
        self._dof_names = self.franka.dof_names

        robot_pos, robot_ori = self.franka.get_world_poses()
        objects = mg.SceneQuery().get_prims_in_aabb(
            search_box_origin=robot_pos.numpy()[0],
            search_box_minimum=[-10.0, -10.0, -10.0],
            search_box_maximum=[10.0, 10.0, 10.0],
            tracked_api=mg.TrackableApi.PHYSICS_COLLISION,
            exclude_prim_paths=["/World/Franka", "/World/Target"],
        )
        self._world_binding = mg.WorldBinding(
            world_interface=CumotionWorldInterface(),
            obstacle_strategy=mg.ObstacleStrategy(),
            tracked_prims=objects,
            tracked_collision_api=mg.TrackableApi.PHYSICS_COLLISION,
        )
        self._world_binding.initialize()
        # Initial cuMotion world sync; franka_sub_loop honors the static flags after this.
        self._world_binding.get_world_interface().update_world_to_robot_root_transforms(poses=(robot_pos, robot_ori))
        self._world_binding.synchronize_transforms()

        self.rmpf_controller = RmpFlowController(
            cumotion_robot=self._cumotion_robot,
            cumotion_world_interface=self._world_binding.get_world_interface(),
            robot_joint_space=self.franka.dof_names,
            robot_site_space=self._site_space,
            tool_frame=self._tool_frame,
        )

        self.target = GeomPrim("/World/Target")
        rot = euler_angles_to_quat((-180, 0, -180), degrees=True)
        self.target.set_world_poses(orientations=np.array([rot]))

    def before_reset_world(self) -> None:
        """Prepare the world for reset.

        This method is called before resetting the world to set up the camera robot.
        """
        self.draw.clear_points()
        self.camera_robot = Articulation("/World/camera")

    def after_reset_world(self) -> None:
        """Execute operations after the world has been reset.

        This method is called after resetting the world to set up controllers and the Franka robot.
        """
        self.zmq_client.simulation_start_timecode = time.time()
        self.meters_per_unit = omni.usd.get_context().get_stage().GetMetadata(UsdGeom.Tokens.metersPerUnit)
        self.reset_franka_mission()

    @classmethod
    def add_franka(cls) -> None:
        """Add a Franka robot to the scene as reference"""
        root = get_assets_root_path()
        franka_usd = root + "/Isaac/Robots/FrankaRobotics/FrankaPanda/franka.usd"
        stage_utils.add_reference_to_stage(usd_path=franka_usd, prim_path="/World/Franka")

    @classmethod
    async def _async_load(cls, mission_instance) -> None:
        """Load the mission asynchronously."""
        await Mission._async_load_stage(cls.mission_usd_path(), mission_instance)
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
    gripper_camera_prim_path = "/World/Franka/panda_hand/gripper_camera"
    gripper_camera_pos = (0.1, 0.0, -0.1)
    gripper_camera_rot = euler_angles_to_quat((180, 15, 0), degrees=True)

    @classmethod
    def add_franka(cls) -> None:
        """Add a Franka robot with an additional gripper camera to the scene."""
        super().add_franka()
        gripper_camera = Camera(prim_path=cls.gripper_camera_prim_path)
        gripper_camera.set_clipping_range(0.01, 10000)
        gripper_camera.set_visibility(False)

        gripper_camera.set_lens_distortion_model("OmniLensDistortionOpenCvFisheyeAPI")

        gripper_camera_xform = XformPrim(cls.gripper_camera_prim_path)
        gripper_camera_xform.set_local_poses(
            translations=np.array([cls.gripper_camera_pos]), orientations=np.array([cls.gripper_camera_rot])
        )

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
            self.zmq_client.add_physics_step_callback(
                "gripper_annotator", 1 / self.camera_hz, self.gripper_annotator.stream
            )

    def reset_franka_mission(self) -> None:
        super().reset_franka_mission()
        gripper_camera_xform = XformPrim(self.gripper_camera_prim_path)
        gripper_camera_xform.set_local_poses(
            translations=np.array([self.gripper_camera_pos]), orientations=np.array([self.gripper_camera_rot])
        )
