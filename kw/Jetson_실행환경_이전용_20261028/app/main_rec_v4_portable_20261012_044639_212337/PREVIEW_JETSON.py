"""Show original model picker and camera-off UI; no hardware or real control."""
import argparse
import os
from pathlib import Path
import sys

root = Path(__file__).resolve().parent
sys.path.insert(0, str(root / "depth_cam"))
os.environ["YOLO_AUTOINSTALL"] = "false"
os.environ["LIFTER_DEMO_MODE"] = "0"
os.environ["PYTHONDONTWRITEBYTECODE"] = "1"
parser = argparse.ArgumentParser(description=__doc__)
parser.add_argument("--model", help="Skip model chooser and use a bundled model filename")
args = parser.parse_args()
from model_catalog import MODEL_ENV, choose_model, resolve_model, validate_pose_model
selected = resolve_model(args.model) if args.model else choose_model()
if selected is None:
    raise SystemExit(0)
os.environ[MODEL_ENV] = str(selected)
print("PREVIEW_MODEL", validate_pose_model(selected), flush=True)
from main_rec_v4 import _load_runtime_components_direct
factory, config, runtime_main, diagram = _load_runtime_components_direct(selected)
from calib import control
control.configure_can_enabled(False)

def blocked_channel(*args, **kwargs):
    raise RuntimeError("UI preview forbids opening a CAN channel")

control.canlib.openChannel = blocked_channel
import cv2
original_imshow = cv2.imshow
original_property = cv2.getWindowProperty

def compatible_property(title, prop):
    try:
        value = original_property(title, prop)
        if prop == cv2.WND_PROP_VISIBLE and value < 0:
            # GTK2 does not implement VISIBLE. AUTOSIZE verifies the window exists.
            return 1.0 if original_property(title, cv2.WND_PROP_AUTOSIZE) >= 0 else 0.0
        return value
    except cv2.error:
        return 0.0

cv2.getWindowProperty = compatible_property
import tkinter as tk
screen = tk.Tk()
screen.withdraw()
display_limit = (screen.winfo_screenwidth() - 100, screen.winfo_screenheight() - 150)
screen.destroy()
from main_rec import _fit_view_for_display
saved = False

def show_and_export(title, image):
    global saved
    display_image = _fit_view_for_display(image, display_limit)
    if not saved:
        cv2.imwrite(str(root / "jetson-software/ui-preview-raw.png"), image)
        cv2.imwrite(str(root / "jetson-software/ui-preview.png"), display_image)
        print("PREVIEW_FRAME_RENDERED", image.shape, "DISPLAY", display_image.shape, flush=True)
        saved = True
    original_imshow(title, display_image)

cv2.imshow = show_and_export
os.chdir(root / "depth_cam")
runtime_main(
    fsm_factory=factory,
    window_title="[UI PREVIEW] FSM v4 | " + selected.name,
    fsm_version="v4",
    diagram_drawer=diagram,
    can_enabled=False,
    camera_enabled=False,
    camera_display_scale=config.CAMERA_DISPLAY_SCALE,
    interface_panel_width=config.INTERFACE_PANEL_WIDTH,
    fsm_panel_width=config.FSM_PANEL_WIDTH,
    demo_mode=False,
)
