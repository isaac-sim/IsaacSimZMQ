# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import importlib
import sys
import types

import carb
import omni


def register_proto_modules():
    """Register Protobuf modules in a persistent namespace to prevent hot-reload issues.

    Problem: Protobuf modules raise exceptions when hot-reloaded during extension reloads.
    Solution: Store imported modules in a persistent namespace (omni.__proto__) that survives
              extension reloads, preventing the need to reimport the modules.
    """
    namespace = "__proto__"

    # Create persistent namespace
    if not hasattr(omni, namespace):
        omni.__proto__ = types.ModuleType(f"omni.{namespace}")
        sys.modules[f"omni.{namespace}"] = omni.__proto__

    # List of protobuf modules to register
    proto_modules = [
        "server_control_message_pb2",
        "client_stream_message_pb2",
    ]

    # Import each module if not already registered
    for module_name in proto_modules:
        if not hasattr(omni.__proto__, module_name):
            try:
                imported_module = importlib.import_module(f".{module_name}", package=__package__)
                setattr(omni.__proto__, module_name, imported_module)
            except ImportError as e:
                carb.log_warn(f"Warning: Failed to import {module_name}: {e}")
