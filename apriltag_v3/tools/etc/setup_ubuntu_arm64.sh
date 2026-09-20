#!/bin/bash
# 리눅스 arm64(맥북의 Ubuntu VM, Jetson)에 krri 실행환경을 만든다. 단계별로 돌린다.
#
#     bash tools/etc/setup_ubuntu_arm64.sh apt        # 빌드 도구·라이브러리 (sudo)
#     bash tools/etc/setup_ubuntu_arm64.sh conda      # Miniforge + python 3.11 env "krri"
#     bash tools/etc/setup_ubuntu_arm64.sh realsense  # librealsense 소스 빌드 + pyrealsense2 (30~40분, Jetson Nano 는 1~2시간)
#     bash tools/etc/setup_ubuntu_arm64.sh pip        # requirements.txt (pyrealsense2 는 위에서 빌드했으니 제외)
#     bash tools/etc/setup_ubuntu_arm64.sh can        # Kvaser linuxcan 드라이버 + canlib
#     bash tools/etc/setup_ubuntu_arm64.sh check      # check_setup.py + lsusb
#     bash tools/etc/setup_ubuntu_arm64.sh all
#
# 왜 소스 빌드인가: PyPI 에 리눅스 aarch64 용 pyrealsense2 휠이 없다(requirements.txt 참고).
# RSUSB 백엔드(-DFORCE_RSUSB_BACKEND=ON)라 커널 패치 없이 프레임 메타데이터가 온다.
# 2026-09-07 맥북 M2 의 UTM Ubuntu 24.04.4 VM 에서 전 단계 검증.
set -euo pipefail
STEP="${1:-all}"
RS_VER=v2.58.3            # requirements.txt 의 pyrealsense2==2.58.3.* 와 맞춤
PY=3.11
ENV=krri
CODE_DIR=$(cd "$(dirname "$0")/../.." && pwd)        # 이 스크립트가 든 코드 폴더 (apriltag_v1 또는 apriltag_v2)
REPO_DIR=$(cd "$CODE_DIR/.." && pwd)             # 리포 루트 (requirements.txt 가 있는 곳)
LINUXCAN_URL="https://pim.kvaser.com/var/assets/Product_Resources/7330130980754/5.52.563/linuxcan_5_52_563.tar.gz"
PYBIN="$HOME/miniforge3/envs/$ENV/bin/python"
PIP="$HOME/miniforge3/envs/$ENV/bin/pip"
is_jetson() { [ -f /etc/nv_tegra_release ] || grep -qi tegra /proc/device-tree/compatible 2>/dev/null; }

apt_step() {
  sudo apt update
  sudo DEBIAN_FRONTEND=noninteractive apt install -y \
    build-essential cmake git pkg-config curl wget unzip \
    libusb-1.0-0-dev libssl-dev libudev-dev libgtk-3-dev libglfw3-dev libgl1-mesa-dev libglu1-mesa-dev \
    python3-dev v4l-utils usbutils \
    libxinerama-dev libxcursor-dev libxi-dev libxrandr-dev
}

conda_step() {
  if [ ! -d "$HOME/miniforge3" ]; then
    curl -L -o /tmp/miniforge.sh https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-aarch64.sh
    bash /tmp/miniforge.sh -b -p "$HOME/miniforge3"
  fi
  "$HOME/miniforge3/bin/conda" init bash >/dev/null
  [ -d "$HOME/miniforge3/envs/$ENV" ] || "$HOME/miniforge3/bin/conda" create -y -n "$ENV" python=$PY
  # SSH 로 들어온 터미널에서도 로컬 화면(:0)에 창을 띄울 수 있게 (VM 창 / Jetson 에 모니터가 있을 때)
  if ! grep -q "mutter-Xwaylandauth" "$HOME/.bashrc"; then cat >> "$HOME/.bashrc" <<'RC'

# SSH 로 들어온 터미널에서도 로컬 화면에 창을 띄울 수 있게 (live_pose, run.py --show)
if [ -z "$DISPLAY" ] && [ -S /tmp/.X11-unix/X0 ]; then
    _xa=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* 2>/dev/null | head -1)
    [ -n "$_xa" ] && export DISPLAY=:0 XAUTHORITY="$_xa"
    unset _xa
fi
RC
  fi
}

realsense_step() {
  mkdir -p "$HOME/src" && cd "$HOME/src"
  [ -d librealsense ] || git clone --depth 1 --branch "$RS_VER" https://github.com/realsenseai/librealsense.git
  cd librealsense
  # udev 규칙: 일반 사용자가 카메라를 열 수 있게
  sudo cp config/99-realsense-libusb.rules /etc/udev/rules.d/ && sudo udevadm control --reload-rules && sudo udevadm trigger
  mkdir -p build && cd build
  CUDA=OFF; is_jetson && [ -d /usr/local/cuda ] && CUDA=ON     # Jetson 이면 CUDA 가속 켬(선택)
  cmake .. \
    -DCMAKE_BUILD_TYPE=Release \
    -DFORCE_RSUSB_BACKEND=ON \
    -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE="$PYBIN" \
    -DBUILD_EXAMPLES=ON -DBUILD_GRAPHICAL_EXAMPLES=ON \
    -DBUILD_WITH_CUDA=$CUDA -DBUILD_UNIT_TESTS=OFF
  make -j"$(nproc)"
  sudo make install
  sudo ldconfig
  # cmake 는 시스템 파이썬 경로에 깔므로, env 의 site-packages 에 따로 넣는다
  SITE=$("$PYBIN" -c "import site; print(site.getsitepackages()[0])")
  mkdir -p "$SITE/pyrealsense2"
  cp -f Release/pyrealsense2*.so "$SITE/pyrealsense2/"
  echo "from .pyrealsense2 import *" > "$SITE/pyrealsense2/__init__.py"
  "$PYBIN" -c "import pyrealsense2 as rs; print('pyrealsense2 OK:', rs.__file__)"
}

pip_step() {
  # pyrealsense2 는 위에서 소스 빌드(requirements 의 aarch64 마커로도 빠진다).
  # keyboard 는 리눅스에서 import 에 root 가 필요하지만 설치는 되므로 그대로 둔다(run.py 는 엔터로 폴백).
  grep -v -E "^pyrealsense2" "$REPO_DIR/requirements.txt" > /tmp/req.txt
  "$PIP" install -r /tmp/req.txt
}

can_step() {
  # Kvaser linuxcan (CANlib 드라이버 + 라이브러리). 장치 파일은 드라이버가 0666 으로 만든다.
  if is_jetson; then
    sudo apt install -y nvidia-l4t-kernel-headers || echo "Jetson: 커널 헤더는 JetPack 이 준다. 없으면 /usr/src/linux-headers-* 확인"
  else
    sudo apt install -y "linux-headers-$(uname -r)"
  fi
  mkdir -p "$HOME/src" && cd "$HOME/src"
  [ -f linuxcan.tar.gz ] || curl -L -o linuxcan.tar.gz "$LINUXCAN_URL"
  rm -rf linuxcan && mkdir linuxcan && tar xzf linuxcan.tar.gz -C linuxcan --strip-components=1
  cd linuxcan && make -j"$(nproc)" && sudo make install
  "$PIP" install canlib==1.33.358
  "$PYBIN" -c "from canlib import canlib; print('canlib', canlib.dllversion(), '/ channels', canlib.getNumberOfChannels())"
}

check_step() {
  cd "$CODE_DIR" && "$PYBIN" tools/check_setup.py || true
  lsusb | grep -i -E "intel|kvaser" || echo "(RealSense/Kvaser 가 아직 안 꽂혔거나 VM 에 안 넘어왔다)"
}

case "$STEP" in
  apt) apt_step ;; conda) conda_step ;; realsense) realsense_step ;; pip) pip_step ;; can) can_step ;; check) check_step ;;
  all) apt_step; conda_step; realsense_step; pip_step; can_step; check_step ;;
  *) echo "unknown step: $STEP"; exit 1 ;;
esac
