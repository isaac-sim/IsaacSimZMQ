# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import asyncio
import time
from pathlib import Path
import zmq.asyncio

import omni
import carb

import isaacsim.core.utils.stage as stage_utils
from isaacsim.core.api.world import World

from . import EXT_NAME, ZMQClient


class Mission:
    """Base class for all ZMQ bridge missions.

    This class provides the foundation for creating missions that communicate with external
    applications via ZeroMQ. Derived classes should implement the mission-specific logic
    by overriding methods like before_reset_world, after_reset_world, start_mission, etc.
    """

    name = "Mission"
    world_usd_path = None  # Should be set by derived classes to point to the USD world file

    def __init__(self, server_ip: str = "localhost"):
        self.zmq_client = ZMQClient(server_ip=server_ip)

        self.receive_commands = False

    def before_reset_world(self) -> None:
        """
        Prepare the world for reset
        """
        carb.log_warn(f"[{EXT_NAME}] before reset world: NOT IMPLEMENTED")

    def after_reset_world(self) -> None:
        """
        Execute any operation that requried a clean world.
        """
        carb.log_warn(f"[{EXT_NAME}] after reset world: NOT IMPLEMENTED")

    def start_mission(self) -> None:
        """
        Starts the mission by initializing the necessary sockets, annotators, and callbacks.
        """
        carb.log_warn(f"[{EXT_NAME}] start mission: NOT IMPLEMENTED")

    def reset_world(self) -> None:
        self.world = World(physics_dt=1.0 / self.physics_dt)
        # Clear only the registry to maintain stage structure
        self.world.scene.clear(registry_only=True)
        self.before_reset_world()
        self.world.reset()
        self.after_reset_world()

    def stop_mission(self) -> None:
        """
        Stops the current mission by setting the `receive_commands` flag to False and removing all callbacks from the ZMQ client.
        It also ensures that all connections are disconnected asynchronously.

        """
        carb.log_warn(f"[{EXT_NAME}] stop mission: NOT IMPLEMENTED")

    def subscribe_to_protobuf_in_loop(
        self, socket: zmq.asyncio.Socket, proto_class, fn: callable, *args, **kwargs
    ) -> None:
        """
        Runs a loop that continuously receives protobuf messages from a ZeroMQ socket and calls a given function for each received message.

        Args:
            socket (zmq.asyncio.Socket): The ZeroMQ socket to receive data from.
            proto_class: The protobuf message class to parse the received data.
            fn (callable): The function to call for each received protobuf message.
        """

        async def _async_executor() -> None:
            # Continue receiving data as long as receive_commands flag is True
            while self.receive_commands:
                proto_msg = await self.zmq_client.receive_protobuf(socket, proto_class)
                fn(proto_msg, *args, **kwargs)
            carb.log_info(f"[{socket}] Stopped listening for protobuf messages.")

        asyncio.ensure_future(_async_executor())

    @classmethod
    def mission_usd_path(cls) -> str:
        """
        For the headless example, we need to import the stage progrematically.
        """
        # Get the extension path to locate assets
        manager = omni.kit.app.get_app().get_extension_manager()
        extension_path = manager.get_extension_path_by_module("isaacsim.zmq.bridge.examples")
        data_path = Path(extension_path).joinpath("data")
        assets_path = data_path.parent.parent.parent / "assets"
        source_usd = str(assets_path / cls.world_usd_path)

        return source_usd

    @classmethod
    def load_mission(cls, source_usd: str) -> None:
        """
        Loads the mission by opening the stage file.
        """
        print(f"[{EXT_NAME}] loading mission")
        stage_utils.open_stage(source_usd)

    def reset_world_async(self) -> None:
        carb.log_warn(f"[{EXT_NAME}] reset world async: NOT IMPLEMENTED")

    @classmethod
    async def _async_load(cls, source_usd: str) -> None:
        await stage_utils.open_stage_async(source_usd)

    @classmethod
    def load_mission_async(cls) -> None:
        print(f"[{EXT_NAME}] loading mission")
        asyncio.ensure_future(cls._async_load())

    async def stop_mission_async(self) -> None:
        """
        Stop the mission and clean up resources.
        """
        carb.log_warn(f"[{EXT_NAME}] stop mission: NOT IMPLEMENTED")

    async def _reset(self) -> None:
        """
        similar to reset_world() but async
        """
        await self.stop_mission_async()
        self.world = World(physics_dt=1.0 / self.physics_dt)
        # Clear only the registry to maintain stage structure
        self.world.scene.clear(registry_only=True)
        # Initialize simulation context asynchronously
        await self.world.initialize_simulation_context_async()
        self.before_reset_world()
        await self.world.reset_async()
        self.after_reset_world()
        self.start_mission()

    def reset_world_async(self) -> None:
        asyncio.ensure_future(self._reset())
