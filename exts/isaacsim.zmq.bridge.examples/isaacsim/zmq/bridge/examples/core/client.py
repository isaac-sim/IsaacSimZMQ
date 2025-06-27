# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import asyncio
import traceback
import zmq
import zmq.asyncio

import carb
import omni

from isaacsim.core.api.world import World

from .rate_limiter import RateLimitedCallback
from .. import EXT_NAME

class ZMQClient:
    """
    The ZMQClient class provides a singleton instance which handles the creation and management of ZMQ sockets.

    This class is responsible for:
    - Creating and managing ZMQ sockets for communication with external applications
    - Handling connection and disconnection of sockets
    - Managing physics callbacks for rate-limited data streaming (Python-only mode)
    - Providing methods for sending and receiving data through ZMQ
    """

    _instance = None

    def __new__(cls, *args, **kwargs):
        # Implement singleton pattern
        if not cls._instance:
            cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self, server_ip: str = "localhost"):
        self.server_ip = server_ip
        self.push_sockets = {}
        self.pull_sockets = {}
        self.phyx_callbacks = {}
        self.annotators = {}

        # ZMQ context
        self._context = None

        # Timing and rate control
        self.start_time = 0
        self._adeptive_rate = True

    def context(self) -> zmq.asyncio.Context:
        """
        Returns the ZMQ context if it has not been initialized yet.
        Initializes and returns the ZMQ context if it has not been initialized.

        Returns:
            zmq.asyncio.Context: The ZMQ context.
        """
        if not self._context:
            self._context = zmq.asyncio.Context()
        return self._context

    def get_pull_socket(self, port: int) -> zmq.Socket:
        """
        Creates and returns a ZeroMQ PULL socket connected to the specified port.

        This socket type is used to receive data from a remote PUSH socket.

        Args:
            port (int): The port number to connect the socket to.

        Returns:
            zmq.Socket: The created PULL socket.
        """
        addr = f"tcp://{self.server_ip}:{port}"
        sock = self.context().socket(zmq.PULL)
        sock.set_hwm(1)  # High water mark: only buffer 1 message
        sock.connect(addr)
        self.pull_sockets[addr] = sock
        return sock

    async def receive_protobuf(self, sock: zmq.asyncio.Socket, proto_class) -> object:
        """
        Asynchronously receives a protobuf message from a ZeroMQ socket.

        Args:
            sock (zmq.asyncio.Socket): The ZeroMQ socket to receive data from.
            proto_class: The protobuf message class to parse the received data.

        Returns:
            object: The received data as a protobuf message.
        """
        message_bytes = await sock.recv()
        proto_msg = proto_class()
        proto_msg.ParseFromString(message_bytes)
        return proto_msg

    async def disconnect_all(self) -> None:
        """
        Disconnects all ZeroMQ sockets and terminates the ZeroMQ context.

        This method iterates over all push and pull sockets, disconnects them, and closes them.
        It then clears the socket dictionaries and terminates the ZeroMQ context.
        """
        # Disconnect and close all push sockets
        for addr, sock in self.push_sockets.items():
            await asyncio.sleep(0.1)
            try:
                sock.setsockopt(zmq.LINGER, 0)  # Don't wait for pending messages
                sock.disconnect(addr)
            except asyncio.CancelledError:
                pass
            except zmq.error.ZMQError:
                carb.log_warn(f"[{EXT_NAME}] zmq error - non critical\n {traceback.format_exc()}")
            except Exception:
                carb.log_error(traceback.format_exc())

            sock.close()

        # Disconnect and close all pull sockets
        for addr, sock in self.pull_sockets.items():
            await asyncio.sleep(0.1)
            try:
                sock.disconnect(addr)
            except asyncio.CancelledError:
                pass
            except zmq.error.ZMQError:
                carb.log_warn(f"[{EXT_NAME}] zmq error - non critical\n {traceback.format_exc()}")
            except Exception:
                carb.log_error(traceback.format_exc())

            sock.close()

        # Clear socket dictionaries
        self.pull_sockets = {}
        self.push_sockets = {}

        # Terminate ZMQ context
        if self._context:
            self._context.term()
            self._context = None

    ######################################################################################
    # The following methods are used only when the C++ node modes is not in use
    # They provide Python-based alternatives for streaming data
    ######################################################################################

    def get_push_socket(self, port: int) -> zmq.Socket:
        """
        Creates and returns a ZeroMQ PUSH socket connected to the specified port.

        This socket type is used to send data to a remote PULL socket.

        Args:
            port (int): The port number to connect the socket to.

        Returns:
            zmq.Socket: The created PUSH socket.
        """
        addr = f"tcp://{self.server_ip}:{port}"
        sock = self.context().socket(zmq.PUSH)
        sock.set_hwm(1)  # High water mark: only buffer 1 message
        sock.setsockopt(zmq.SNDTIMEO, 1000)  # 1 sec timeout for sending
        sock.connect(addr)
        self.push_sockets[addr] = sock
        return sock

    def add_physx_step_callback(self, name: str, hz: float, fn: callable) -> None:
        """
        Adds a callback function to be executed at a specified simulation steps frequency.

        This method creates a rate-limited callback that will be executed during physics
        simulation steps at the specified frequency.

        Args:
            name (str): The name of the callback.
            hz (float): The frequency at which the callback is executed.
            fn (callable): The callback function to be executed.
        """
        self.world = World.instance()
        rate_limited_callback = RateLimitedCallback(name, hz, fn, self.start_time)
        self.world.add_physics_callback(name, rate_limited_callback.rate_limit)
        self.phyx_callbacks[name] = rate_limited_callback
        return

    def remove_physx_callbacks(self) -> None:
        """
        Removes all registered physics callbacks.

        This method iterates over the `phyx_callbacks` dictionary and unsubscribes each callback
        from the physics simulation.
        """
        for name, cb in self.phyx_callbacks.items():
            self.world.remove_physics_callback(name)
            del cb

    @property
    def adeptive_rate(self) -> bool:
        return self._adeptive_rate

    @adeptive_rate.setter
    def adeptive_rate(self, value: bool) -> None:
        """
        Set the adaptive rate setting and update all callbacks.

        When adaptive rate is enabled, the system will automatically adjust
        the callback frequency to match the desired rate, accounting for
        execution time of the callbacks.

        Args:
            value (bool): True to enable adaptive rate, False to disable it.
        """
        if value != self._adeptive_rate:
            self._adeptive_rate = value
            for cb in self.phyx_callbacks.values():
                cb.adeptive_rate = value
