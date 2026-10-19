#!/usr/bin/env bash
# Optional fallback when existing SDK has a different Python ABI.
# Isolated RSUSB build; no kernel patch, CUDA change, or global SDK replacement.
set -euo pipefail
DOCK_PREFIX="${DOCK_RS_PREFIX:-$HOME/.local/share/krri/realsense}"
DOCK_ENV="${DOCK_V6_ENV:-$HOME/.venvs/krri}"
DOCK_SRC="$DOCK_PREFIX/source"
[[ -x "$DOCK_ENV/bin/python" ]] || { echo 'install_jetson.sh를 먼저 실행하세요.' >&2; exit 2; }
sudo apt-get install -y build-essential cmake git libusb-1.0-0-dev libudev-dev libssl-dev pkg-config python3-dev
mkdir -p "$DOCK_PREFIX"
if [[ ! -d "$DOCK_SRC/.git" ]]; then git clone --branch v2.56.5 --depth 1 https://github.com/realsenseai/librealsense.git "$DOCK_SRC"; fi
[[ "$(git -C "$DOCK_SRC" describe --tags --exact-match)" == v2.56.5 ]] || { echo 'SDK source tag differs' >&2; exit 2; }
[[ -z "$(git -C "$DOCK_SRC" status --porcelain)" ]] || { echo 'SDK source has local changes' >&2; exit 2; }
DOCK_SITE="$("$DOCK_ENV/bin/python" -c 'import sysconfig; print(sysconfig.get_paths()["purelib"])')"
cmake -S "$DOCK_SRC" -B "$DOCK_PREFIX/build" -DCMAKE_BUILD_TYPE=Release \
  -DCMAKE_INSTALL_PREFIX="$DOCK_PREFIX/install" -DFORCE_RSUSB_BACKEND=ON \
  -DBUILD_PYTHON_BINDINGS=ON -DPYTHON_EXECUTABLE="$DOCK_ENV/bin/python" -DPython_EXECUTABLE="$DOCK_ENV/bin/python" \
  -DPYTHON_INSTALL_DIR="$DOCK_SITE" -DBUILD_EXAMPLES=OFF -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_WITH_CUDA=OFF
cmake --build "$DOCK_PREFIX/build" --parallel 1
cmake --install "$DOCK_PREFIX/build"
# Device permission rule only; does not reset USB or network interfaces.
sudo install -m 644 "$DOCK_SRC/config/99-realsense-libusb.rules" /etc/udev/rules.d/99-realsense-libusb.rules
sudo udevadm control --reload-rules
printf '%s\n' 'RealSense 재연결 후 preflight check --sensor-only로 확인하세요.'
