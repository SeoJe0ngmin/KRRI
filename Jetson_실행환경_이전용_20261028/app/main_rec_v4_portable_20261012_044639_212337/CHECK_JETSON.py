"""Check imports, bundle integrity and synthetic inference; never start camera/CAN."""
from pathlib import Path
import hashlib
import importlib.util
import json
import os
import sys
import time

root = Path(__file__).resolve().parent
sys.path.insert(0, str(root / "depth_cam"))
os.environ["YOLO_AUTOINSTALL"] = "false"
manifest = json.loads((root / "manifest.json").read_text())
for item in manifest:
    path = (root / item["path"]).resolve()
    assert path.is_relative_to(root) and path.stat().st_size == item["bytes"]
    assert hashlib.sha256(path.read_bytes()).hexdigest() == item["sha256"]
print(f"BUNDLE_INTEGRITY_OK: {len(manifest)} deployed files", flush=True)

from model_catalog import DEFAULT_MODEL, MODEL_ENV, list_models, resolve_model, validate_pose_model
selected = resolve_model(DEFAULT_MODEL)
os.environ[MODEL_ENV] = str(selected)
from calib.fsm_v4.trajectory_response import load_selected_trajectory
for model in list_models():
    identity = validate_pose_model(model)
    response = load_selected_trajectory(root / "depth_cam/calib/fsm_v4/motion_trajectory.selected.json", model)
    assert response.inference_model_sha256_id == identity["sha256"]
    print("MODEL_AND_TRAJECTORY_OK:", model.name, flush=True)

from main_rec_v4 import _load_runtime_components_direct
_, config, _, _ = _load_runtime_components_direct(selected)
config.validate()
print("RUNTIME_IMPORT_AND_CONFIG_OK", flush=True)
print("FLAGS:", "CAN", config.CAN_ENABLED, "CAMERA", config.CAMERA_ENABLED,
      "AUTO_INSERT", config.AUTO_INSERT_ENABLED, flush=True)

import numpy as np
import torch
from calib.perception import Perception
perception = Perception(model_path=str(selected))
for iteration in range(2):
    started = time.monotonic()
    result = perception.infer_front(np.zeros((480, 640, 3), dtype=np.uint8))
    if torch.cuda.is_available():
        torch.cuda.synchronize()
    print("REAL_PERCEPTION_SYNTHETIC_OK:", iteration, perception.device,
          round(time.monotonic() - started, 3), "seconds; detected=", result[0], flush=True)

if config.ROUTE_GPU_VISIBILITY_ENABLED:
    from calib.fsm_v4.gpu_visibility import GPUVisibility
    from calib.fsm_v4.pose import VisualPoseFilter
    from calib.fsm_v4 import planner
    gpu = GPUVisibility()
    try:
        pose = VisualPoseFilter().seed_vehicle(0., 0., 4., 0.)
        result = gpu.forward_limits(pose, None, [(0., pose, None, .2)], True)
        expected = planner._maximum_visible_forward_m(pose, 0., .2, None)[0]
        assert result == [(True, expected)] and gpu.stats['batches'] > 0
        print('PLANNER_GPU_VISIBILITY_OK: CUDA float64 batch; CPU reference matched.', flush=True)
    finally:
        gpu.close()

for name in ("pyrealsense2", "canlib"):
    print("HARDWARE_MODULE:", name, "available" if importlib.util.find_spec(name) else "MISSING", flush=True)
print("CHECK_COMPLETE: no camera pipeline, CAN channel or motion commands started.", flush=True)
