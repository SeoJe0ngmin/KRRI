#!/usr/bin/env bash
# Run ON the Jetson, later. Does not start sensors, CAN, services, or apt upgrade.
set -euo pipefail
DOCK_APP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo 'install_jetson.sh [--mapped] : Ubuntu 24.04 arm64/amd64 ROS Jazzy + pinned OpenVINS build; existing RealSense SDK is retained.'; exit 0
fi
DOCK_MAPPING=0
if [[ "${1:-}" == --mapped ]]; then DOCK_MAPPING=1; shift; fi
(($#==0)) || { echo 'unknown argument' >&2; exit 2; }
[[ -f /etc/os-release ]] || { echo 'Jetson Ubuntu에서 실행하세요.' >&2; exit 2; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || { echo '이 설치 구성은 Ubuntu 24.04 / ROS Jazzy용입니다.' >&2; exit 2; }
DOCK_ARCH="$(dpkg --print-architecture)"
[[ "$DOCK_ARCH" == arm64 || "$DOCK_ARCH" == amd64 ]] || exit 2
DOCK_V6_WS="${DOCK_V6_WS:-$HOME/.local/share/krri-v6/ros_ws}"
DOCK_V6_ENV="${DOCK_V6_ENV:-$HOME/.venvs/docking-v6}"
sudo apt-get update
sudo apt-get install -y curl ca-certificates git build-essential cmake software-properties-common python3-venv python3-dev
if [[ ! -f /opt/ros/jazzy/setup.bash ]]; then
  sudo add-apt-repository -y universe
  DOCK_APT_RELEASE="$(curl -fsSL https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest | /usr/bin/python3 -c 'import json,sys; print(json.load(sys.stdin)["tag_name"])')"
  [[ "$DOCK_APT_RELEASE" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]] || { echo 'Unexpected ROS apt release version' >&2; exit 2; }
  DOCK_DEB="$(mktemp --suffix=.deb)"
  trap 'rm -f "$DOCK_DEB"' EXIT
  curl -fL "https://github.com/ros-infrastructure/ros-apt-source/releases/download/$DOCK_APT_RELEASE/ros2-apt-source_${DOCK_APT_RELEASE}.noble_all.deb" -o "$DOCK_DEB"
  sudo dpkg -i "$DOCK_DEB"
  sudo apt-get update
fi
sudo apt-get install -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev libboost-all-dev libceres-dev libopencv-dev libopencv-contrib-dev \
  python3-numpy python3-scipy python3-opencv python3-yaml python3-pytest \
  ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-tf2-geometry-msgs
if ((DOCK_MAPPING)); then sudo apt-get install -y ros-jazzy-rtabmap-ros; fi
/usr/bin/python3 -m venv --system-site-packages "$DOCK_V6_ENV"
"$DOCK_V6_ENV/bin/python" -m pip install 'pupil-apriltags==1.0.4.post11' 'canlib==1.33.358'
# Do not replace a working RSUSB SDK with a different PyPI wheel.
if ! "$DOCK_V6_ENV/bin/python" -c 'import pyrealsense2' 2>/dev/null; then
  DOCK_OLD_PYTHON="${DOCK_EXISTING_RS_PYTHON:-$HOME/.venvs/krri/bin/python}"
  if [[ -x "$DOCK_OLD_PYTHON" ]]; then
    DOCK_OLD_ABI="$("$DOCK_OLD_PYTHON" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    DOCK_NEW_ABI="$("$DOCK_V6_ENV/bin/python" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    if [[ "$DOCK_OLD_ABI" == "$DOCK_NEW_ABI" ]] && "$DOCK_OLD_PYTHON" -c 'import pyrealsense2' 2>/dev/null; then
      DOCK_RS_MODULE="$("$DOCK_OLD_PYTHON" -c 'import pyrealsense2; print(pyrealsense2.__file__)')"
      "$DOCK_V6_ENV/bin/python" - "$DOCK_RS_MODULE" <<'PY'
import pathlib,sys,sysconfig
source=pathlib.Path(sys.argv[1]).resolve()
# Link only this SDK package/extension, not the entire old environment.
root=source.parent if source.name=='__init__.py' else source
name='pyrealsense2' if root.is_dir() else root.name
dest=pathlib.Path(sysconfig.get_paths()['purelib'])/name
if not dest.exists(): dest.symlink_to(root,target_is_directory=root.is_dir())
PY
    fi
  fi
fi
mkdir -p "$DOCK_V6_WS/src" "$DOCK_V6_WS/upstream"
touch "$DOCK_V6_WS/upstream/COLCON_IGNORE"
DOCK_OV_COMMIT=69488123ed9362dd44b6f28e7f4680abbff1442b
DOCK_OV_SRC="$DOCK_V6_WS/upstream/open_vins"
if [[ ! -d "$DOCK_OV_SRC/.git" ]]; then git clone https://github.com/rpng/open_vins.git "$DOCK_OV_SRC"; fi
if [[ -n "$(git -C "$DOCK_OV_SRC" status --porcelain)" ]]; then echo 'OpenVINS source has local changes; leaving them untouched.' >&2; exit 2; fi
git -C "$DOCK_OV_SRC" fetch origin "$DOCK_OV_COMMIT"
git -C "$DOCK_OV_SRC" checkout --detach "$DOCK_OV_COMMIT"
/usr/bin/python3 "$DOCK_APP/scripts/prepare_openvins.py" "$DOCK_OV_SRC" "$DOCK_V6_WS/src/open_vins"
# Symlinks track subsequent v6 code updates; re-run installer after bridge changes.
for DOCK_PACKAGE in docking_v6_msgs docking_openvins; do
  DOCK_DEST="$DOCK_V6_WS/src/$DOCK_PACKAGE"
  if [[ -e "$DOCK_DEST" && ! -L "$DOCK_DEST" ]]; then echo "Existing directory: $DOCK_DEST" >&2; exit 2; fi
  ln -sfn "$DOCK_APP/ros2/$DOCK_PACKAGE" "$DOCK_DEST"
done
set +u
source /opt/ros/jazzy/setup.bash
set -u
if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
rosdep update
rosdep install --from-paths "$DOCK_V6_WS/src" --ignore-src --rosdistro jazzy -y
cd "$DOCK_V6_WS"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 /usr/bin/colcon build --base-paths "$DOCK_V6_WS/src" --packages-up-to docking_openvins \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$DOCK_APP/scripts/environment.sh"
cd "$DOCK_APP"
"$DOCK_PYTHON" -m unittest discover -s tests -q
"$DOCK_PYTHON" -m docking.preflight check --sensor-only
printf '%s\n' '설치/소프트웨어 검사 완료. 실제 센서 확인: bash scripts/run_nano.sh measure --seconds 90 --view'
