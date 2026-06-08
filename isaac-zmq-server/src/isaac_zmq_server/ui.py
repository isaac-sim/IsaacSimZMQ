# SPDX-FileCopyrightText: Copyright (c) 2024 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: MIT

import os

import dearpygui.dearpygui as dpg


def _gnome_monitor_scale() -> float | None:
    """Read the active monitor scale from GNOME's ~/.config/monitors.xml."""
    import xml.etree.ElementTree as ET

    path = os.path.expanduser("~/.config/monitors.xml")
    try:
        root = ET.parse(path).getroot()
        for config in root.findall("configuration"):
            for monitor in config.findall("logicalmonitor"):
                scale = monitor.findtext("scale")
                if scale:
                    return float(scale)
    except Exception:
        pass
    return None


def detect_dpi_scale() -> float:
    """Return the logical-to-physical pixel scale for the active display.

    Precedence: ISAAC_ZMQ_DPI_SCALE env, GNOME monitors.xml, Xft.dpi (xrdb),
    GDK_DPI_SCALE / QT_SCALE_FACTOR, then 1.0.
    """
    val = os.environ.get("ISAAC_ZMQ_DPI_SCALE")
    if val:
        try:
            return float(val)
        except ValueError:
            pass
    gnome_scale = _gnome_monitor_scale()
    if gnome_scale and gnome_scale > 1.05:
        return gnome_scale
    try:
        import subprocess

        out = subprocess.check_output(["/usr/bin/xrdb", "-query"], stderr=subprocess.DEVNULL, timeout=1).decode()
        for line in out.splitlines():
            if line.startswith("Xft.dpi"):
                dpi = float(line.split()[-1])
                return dpi / 96.0
    except Exception:
        pass
    for var in ("GDK_DPI_SCALE", "QT_SCALE_FACTOR"):
        val = os.environ.get(var)
        if val:
            try:
                return float(val)
            except ValueError:
                pass
    return 1.0


class App:
    """
    Base class for creating the GUI with DearPyGUI.

    This class provides the basic structure for creating a window, setting up
    the application, and handling the main loop. Derived classes should implement
    the create_app_body and create_network_iface methods.
    """

    def __init__(self):
        """Initialize the application with default window settings."""
        self.window_name = "DearPyGUI App"
        self.window_width = 800
        self.window_height = 600
        self.resizeable = False

        # HiDPI scales
        self.dpi_scale = detect_dpi_scale()
        ui_override = os.environ.get("ISAAC_ZMQ_UI_SCALE")
        try:
            self.ui_scale = float(ui_override) if ui_override else self.dpi_scale * 1.5
        except ValueError:
            self.ui_scale = self.dpi_scale * 1.5
        print(f"[ui] dpi_scale={self.dpi_scale} ui_scale={self.ui_scale}")

    def create_app_body(self):
        """
        Create the body of the application.

        This method should be implemented by derived classes to define the
        UI elements of the application.

        Raises:
            NotImplementedError: If the derived class does not implement this method.
        """
        raise NotImplementedError

    def create_network_iface(self):
        """
        Create the network interface for the application.

        This method should be implemented by derived classes to set up
        network communication.

        Raises:
            NotImplementedError: If the derived class does not implement this method.
        """
        raise NotImplementedError

    def _create_app(self) -> None:
        """
        Create the application window.
        """
        # Initialize DearPyGUI
        dpg.create_context()
        dpg.create_viewport(
            title=self.window_name,
            width=self.window_width,
            height=self.window_height,
            resizable=self.resizeable,
        )
        dpg.setup_dearpygui()

        # Set up fonts (4x super-sample, on-screen size driven by ui_scale)
        self.font_scale = 4
        with dpg.font_registry():
            font_medium = dpg.add_font("./isaac_zmq_server/fonts/Inter-Medium.ttf", 16 * self.font_scale)

        dpg.set_global_font_scale(self.ui_scale / self.font_scale)
        dpg.bind_font(font_medium)

        # Scale widget paddings/spacings to match the scaled font
        dpg.bind_theme(self._build_hidpi_theme(self.ui_scale))

        # Create the application body
        self.create_app_body()

        # Show the viewport
        dpg.show_viewport()

    @staticmethod
    def _build_hidpi_theme(scale: float):
        """Return a DPG theme scaling widget paddings/spacings/grab sizes by ``scale``."""
        with dpg.theme() as theme:
            with dpg.theme_component(dpg.mvAll):
                dpg.add_theme_style(dpg.mvStyleVar_FramePadding, 4 * scale, 3 * scale)
                dpg.add_theme_style(dpg.mvStyleVar_ItemSpacing, 8 * scale, 4 * scale)
                dpg.add_theme_style(dpg.mvStyleVar_ItemInnerSpacing, 4 * scale, 4 * scale)
                dpg.add_theme_style(dpg.mvStyleVar_WindowPadding, 8 * scale, 8 * scale)
                dpg.add_theme_style(dpg.mvStyleVar_FrameRounding, 2 * scale)
                dpg.add_theme_style(dpg.mvStyleVar_GrabMinSize, 10 * scale)
                dpg.add_theme_style(dpg.mvStyleVar_ScrollbarSize, 14 * scale)
                dpg.add_theme_style(dpg.mvStyleVar_IndentSpacing, 21 * scale)
        return theme

    def _run(self) -> None:
        """
        Run the main application loop.
        """
        while dpg.is_dearpygui_running():
            dpg.render_dearpygui_frame()

    def _cleanup(self) -> None:
        """
        Clean up resources when the application is closed.

        This method cleans up the ZMQ server and destroys the DearPyGUI context.
        """
        self.zmq_server.cleanup()
        dpg.destroy_context()

    @classmethod
    def run_app(cls) -> None:
        """
        Static method to run the application.
        """
        app = cls()
        app._create_app()
        app.create_network_iface()
        app._run()
        app._cleanup()
