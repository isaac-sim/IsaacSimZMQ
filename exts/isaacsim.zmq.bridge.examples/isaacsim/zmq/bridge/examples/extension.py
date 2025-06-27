# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT


from functools import partial

import carb
import omni.ext
import omni.ui as ui
import omni.usd
from omni.kit.menu.utils import MenuItemDescription, add_menu_items, remove_menu_items
from omni.kit.notification_manager import post_notification
from omni.kit.widget.toolbar import get_instance
import omni.timeline

from . import EXT_NAME
from .example_missions import FrankaMultiVisionMission, FrankaVisionMission
from .ui import ZMQClientButtonGroup


class IsaacSimZMQBridgeExamples(omni.ext.IExt):
    """Extension for demonstrating ZMQ bridge functionality in Isaac Sim.

    This extension provides UI elements and mission management for ZMQ bridge examples,
    allowing users to load different example missions and control their execution.
    """

    server_ip = "localhost"

    def on_startup(self, ext_id) -> None:
        self.mission = None

        # Append example buttons to the main isaac sim toolbar
        self.toolbar = get_instance()
        self.button_group = ZMQClientButtonGroup()
        self.toolbar.add_widget(self.button_group, 100, self.toolbar.get_context())

        # Append example menu items to the main isaac sim menu
        self._franka_mission_menu = MenuItemDescription(
            name="Franka RMPFlow",
            glyph="plug.svg",
            onclick_fn=FrankaVisionMission.load_mission_async,
        )
        self._franka_multi_mission_menu = MenuItemDescription(
            name="Franka RMPFlow (Multi Camera)",
            glyph="plug.svg",
            onclick_fn=FrankaMultiVisionMission.load_mission_async,
        )

        self._menu_items = [
            MenuItemDescription(
                name="Isaac ZMQ Examples",
                glyph="plug.svg",
                sub_menu=[self._franka_mission_menu, self._franka_multi_mission_menu],
            )
        ]
        add_menu_items(self._menu_items, "Create")

        # Subscribe to stage events to detect when a new stage is loaded / Cleared
        self.stage_load_sub = (
            omni.usd.get_context()
            .get_stage_event_stream()
            .create_subscription_to_pop(self.stage_event, name="event_stage_loaded")
        )
        self.check_stage()

        # Subscribe to timeline events to detect when the timeline is stopped
        self.timeline = omni.timeline.get_timeline_interface()
        self.timeline_sub = (
            self.timeline
            .get_timeline_event_stream()
            .create_subscription_to_pop(self.timeline_event,
                                        name="timeline_event")
        )

    def timeline_event(self, event) -> None:
        if event.type == int(omni.timeline.TimelineEventType.STOP):
            if self.mission:
                self.mission.stop_mission()

    def stage_event(self, event) -> None:
        """Handle stage events.

        When a stage is opened, check if it matches any of our example missions.
        """
        if event.type == int(omni.usd.StageEventType.OPENED):
            self.check_stage()

    def check_stage(self) -> None:
        """Check the current stage and set the appropriate mission.

        This method examines the loaded USD file and sets the corresponding mission
        if it matches one of our example missions.
        """
        # set the mission based on the loaded stage
        stage = omni.usd.get_context().get_stage()
        if not stage:
            return
        usd_path = stage.GetRootLayer().realPath
        if usd_path.endswith(FrankaVisionMission.world_usd_path):
            self._set_mission(FrankaVisionMission, "FrankaVisionMission")
        elif usd_path.endswith(FrankaMultiVisionMission.world_usd_path):
            self._set_mission(FrankaMultiVisionMission, "FrankaMultiVisionMission")
        else:
            self._clear_mission()
        return

    def _set_mission(self, mission_class, mission_name) -> None:
        """Set the active mission.

        Creates an instance of the specified mission class and assigns it to the button group.

        Args:
            mission_class: The mission class to instantiate
            mission_name: The name of the mission for logging
        """
        print(f"[{EXT_NAME}] Setting {mission_name}.")
        post_notification(f"[{EXT_NAME}] Setting {mission_name}.", duration=2)
        self.mission = mission_class(server_ip=self.server_ip)
        self.button_group.set_mission(self.mission)
        self.button_group.set_visiblity(True)

    def _clear_mission(self) -> None:
        """Clear the active mission.

        Called when the loaded stage doesn't match any of our example missions.
        """
        carb.log_warn(f"[{EXT_NAME}] Stage is empty - Setting mission to None")
        post_notification(f"[{EXT_NAME}] Setting mission to None", duration=2)
        self.mission = None
        self.button_group.set_mission(None)
        self.button_group.set_visiblity(False)

    def on_shutdown(self) -> None:
        """Clean up resources when the extension is disabled.

        Stops any active mission and removes UI elements.
        """
        if self.mission:
            self.mission.stop_mission()

        self.toolbar.remove_widget(self.button_group)
        self.button_group = None

        remove_menu_items(self._menu_items, "Create")

        self.stage_load_sub = None
        self.timeline_sub = None
