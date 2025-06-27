# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import asyncio
import os
from pathlib import Path

import carb
import omni.ext
import omni.ui as ui
import omni.usd
from omni.kit.notification_manager import NotificationStatus, post_notification
from omni.kit.widget.toolbar import WidgetGroup

from . import EXT_NAME
from .mission import Mission


def get_data_path() -> Path:
    manager = omni.kit.app.get_app().get_extension_manager()
    extension_path = manager.get_extension_path_by_module("isaacsim.zmq.bridge.examples")
    return Path(extension_path).joinpath("data")


class ZMQClientButtonGroup(WidgetGroup):
    """
    UI widget group that provides buttons to start/stop streaming and reset the world.
    """

    def __init__(self):
        WidgetGroup.__init__(self)
        self._is_streaming = False
        self.mission = None

    def set_mission(self, mission: Mission) -> None:
        """
        Set the active mission for this button group.

        Args:
            mission (Mission): The mission to control with this button group
        """
        self.mission = mission

    def clean(self) -> None:
        """Clean up resources when the widget is destroyed."""
        super().clean()
        # self._start_stop_button = None
        self._reset_button = None

    def get_style(self) -> dict:
        return {}

    def on_reset_click(self) -> None:
        """
        Handle click on the reset world button.

        This method resets the world for the current mission.
        If no mission is set, it displays a popup notification.
        """
        if not self.mission:
            post_notification(
                f"[{EXT_NAME}] Please load a mission - Menu > Create > Isaac ZMQ Examples",
                duration=4,
                status=NotificationStatus.WARNING,
            )
            return

        self._reset_button.checked = False
        self.mission.reset_world_async()

    def create(self, default_size) -> None:
        """
        Create the UI buttons for the widget group.
        """
        # Create reset world button
        self._reset_button = ui.ToolButton(
            image_url="${glyphs}/menu_refresh.svg",
            name="resert_world",
            tooltip=f"Reset World",
            width=default_size,
            height=default_size,
            clicked_fn=self.on_reset_click,
        )
        self.set_visiblity(False)

    def set_visiblity(self, visible: bool) -> None:
        """
        Set the visibility of the buttons.

        Args:
            visible (bool): Whether the buttons should be visible
        """
        if hasattr(self, "_reset_button"):
            self._reset_button.visible = visible
