#!/bin/bash
set -euo pipefail
cd -- "$(dirname -- "$0")"
source ./VENDOR_ENV.sh
./app/main_rec_v4_portable_20261012_044639_212337/.venv-jetson/bin/python - <<'PY'
import torch,torchvision,cv2,numpy,scipy,tkinter,pyrealsense2
from canlib import canlib
from ultralytics import YOLO
print('torch',torch.__version__,'CUDA',torch.version.cuda,'cv2',cv2.__version__,'numpy',numpy.__version__)
assert torch.cuda.is_available(),'CUDA unavailable: install matching JetPack CUDA/cuDNN packages'
x=torch.ones((16,16),device='cuda');assert float((x@x)[0,0])==16;torch.cuda.synchronize()
print('GPU test passed. No camera/CAN channel or vehicle runtime opened.')
PY
