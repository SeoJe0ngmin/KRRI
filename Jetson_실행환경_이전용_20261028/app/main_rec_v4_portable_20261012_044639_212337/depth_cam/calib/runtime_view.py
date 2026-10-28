"""Pure UI rendering shared by previews and the isolated presentation process."""
import sys
import importlib
import cv2
import numpy as np
from .hud import draw_panel
from .camera_overlay import draw_camera_overlay
from .performance import profiled

@profiled('ui.compose', 'host_work')
def compose_runtime_view(
    vis, fsm, diagram_drawer, camera_display_scale,
    interface_lines=None, cmd_status=None,
    interface_panel_width=640, fsm_panel_width=900,
    prominent_yaw_deg=None, prominent_z_distance_m=None,
):
    """Compose three isolated columns: camera | interface | FSM diagram."""
    scale = float(camera_display_scale)
    camera_view = vis
    if abs(scale - 1.0) > 1e-6:
        height, width = vis.shape[:2]
        camera_view = cv2.resize(
            vis,
            (max(1, round(width * scale)), max(1, round(height * scale))),
            interpolation=cv2.INTER_LINEAR,
        )
    panel_height = camera_view.shape[0]
    interface_bg = np.array((18, 21, 25), dtype=np.uint8)
    interface_panel = np.full(
        (panel_height, max(320, int(interface_panel_width)), 3),
        interface_bg, dtype=np.uint8,
    )
    panel_lines = [
        ("RUNTIME / MODEL INFERENCE", (240, 240, 240)),
    ]
    panel_lines.extend(list(interface_lines or []))
    compact = panel_height < 600
    draw_panel(
        interface_panel, panel_lines, origin=(18, 22), pad=(7, 6),
        line_h=18 if compact else 22,
        font_scale=0.45 if compact else 0.50,
        thickness=1, cmd_status=cmd_status,
        prominent_metrics=(prominent_yaw_deg, prominent_z_distance_m),
    )
    # VideoWriter requires every frame to have exactly the size used when it
    # was opened. Do not crop this panel from live text extents: long status or
    # failure strings would otherwise change the composite width mid-recording
    # and FFmpeg would reject those frames.

    diag = diagram_drawer(
        fsm, panel_size=(panel_height, max(420, int(fsm_panel_width))),
    )
    if diag.shape[0] != camera_view.shape[0]:
        diag = cv2.resize(
            diag, (diag.shape[1], camera_view.shape[0]),
            interpolation=cv2.INTER_AREA,
        )
    separator = np.full((panel_height, 6, 3), (5, 7, 9), dtype=np.uint8)
    return cv2.hconcat([
        camera_view, separator, interface_panel, separator.copy(), diag,
    ])


def screen_work_area_limit():
    """Return a conservative primary-screen drawable size on Windows."""
    if not sys.platform.startswith("win"):
        return None
    try:
        import ctypes

        user32 = ctypes.windll.user32
        screen_w = int(user32.GetSystemMetrics(0))
        screen_h = int(user32.GetSystemMetrics(1))
        if screen_w > 0 and screen_h > 0:
            return max(640, screen_w - 32), max(480, screen_h - 80)
    except Exception:
        pass
    return None


def fit_view_for_display(image, display_limit):
    """Downscale only the visible window; keep recording at full resolution."""
    if display_limit is None:
        return image
    max_w, max_h = display_limit
    height, width = image.shape[:2]
    scale = min(1.0, max_w / max(1, width), max_h / max(1, height))
    if scale >= 0.999:
        return image
    return cv2.resize(
        image,
        (max(1, round(width * scale)), max(1, round(height * scale))),
        interpolation=cv2.INTER_AREA,
    )


def make_renderer(spec):
    module_name = spec['diagram_module']
    if module_name not in ('ui.diagram', 'ui.diagram_v4'):
        raise ValueError('Unsupported UI diagram: ' + module_name)
    diagram = getattr(importlib.import_module(module_name), spec['diagram_name'])
    def render(packet):
        vis = packet['image']
        if packet['overlay'] is not None:
            draw_camera_overlay(vis, **packet['overlay'])
        return compose_runtime_view(vis, packet['fsm'], diagram, spec['camera_display_scale'],
            interface_lines=packet['lines'], cmd_status=packet['fsm'].cmd_status,
            interface_panel_width=spec['interface_panel_width'], fsm_panel_width=spec['fsm_panel_width'],
            prominent_yaw_deg=packet['yaw'], prominent_z_distance_m=packet['z'])
    return render
