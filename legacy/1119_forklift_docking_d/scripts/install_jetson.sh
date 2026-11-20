#!/usr/bin/env bash
# Run ON the Jetson, later. Does not start sensors, CAN, services, or apt upgrade.
set -euo pipefail
DOCK_APP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "${1:-}" == --help ]]; then
  echo 'install_jetson.sh [--mapped] [--no-apt] : Ubuntu 24.04 arm64/amd64 ROS Jazzy + pinned OpenVINS build; existing RealSense SDK is retained.'
  echo '  --no-apt : skip apt/pip/rosdep (already installed, possibly offline); only lay out and build the workspace.'; exit 0
fi
DOCK_MAPPING=0;DOCK_NO_APT=0
while (($#)); do
  case "$1" in
    --mapped) DOCK_MAPPING=1; shift ;;
    --no-apt) DOCK_NO_APT=1; shift ;;
    *) echo "unknown argument: $1" >&2; exit 2 ;;
  esac
done
[[ -f /etc/os-release ]] || { echo 'Jetson Ubuntu에서 실행하세요.' >&2; exit 2; }
source /etc/os-release
[[ "$ID" == ubuntu && "$VERSION_ID" == 24.04 ]] || { echo '이 설치 구성은 Ubuntu 24.04 / ROS Jazzy용입니다.' >&2; exit 2; }
DOCK_ARCH="$(dpkg --print-architecture)"
[[ "$DOCK_ARCH" == arm64 || "$DOCK_ARCH" == amd64 ]] || exit 2
DOCK_WS="${DOCK_WS:-$HOME/.local/share/krri/ros_ws}"   # one workspace for the project (2026-10-19)
DOCK_VENV="${DOCK_VENV:-$HOME/.venvs/krri}"   # one venv for the project (2026-10-19 user decision)
/usr/bin/python3 "$DOCK_APP/scripts/build_support.py" "$DOCK_APP/ros2"
if ((DOCK_NO_APT)); then
  [[ -f /opt/ros/jazzy/setup.bash && -x /usr/bin/colcon ]] || { echo 'ROS Jazzy/colcon 이 없습니다: --no-apt 없이 실행하세요.' >&2; exit 2; }
  [[ -x "$DOCK_VENV/bin/python" ]] || { echo "가상환경 없음: $DOCK_VENV" >&2; exit 2; }
else
sudo apt-get update
sudo apt-get install --no-remove -y curl ca-certificates git build-essential cmake software-properties-common python3-venv python3-dev
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
# Jetson r39.2 gives NVIDIA's monolithic libopencv-dev 4.8 priority 600,
# above Ubuntu's split 4.6 packages (500). ROS Jazzy/contrib use the latter.
# Select matching versions for this transaction; do not change global apt pins.
DOCK_OPENCV_VERSION="$(LC_ALL=C apt-cache policy libopencv-contrib-dev | awk '$1 == "Candidate:" {print $2}')"
if [[ "$DOCK_OPENCV_VERSION" != 4.6.* ]]; then
  printf 'Ubuntu 24.04 OpenCV 4.6 패키지를 확인할 수 없습니다: %s\n' "$DOCK_OPENCV_VERSION" >&2
  echo 'apt-cache policy libopencv-dev libopencv-contrib-dev python3-opencv 결과를 확인하세요.' >&2
  exit 2
fi
printf 'OpenCV 설치 버전: %s (Ubuntu/ROS Jazzy)\n' "$DOCK_OPENCV_VERSION"
sudo apt-get install --no-remove -y ros-jazzy-ros-base python3-colcon-common-extensions python3-rosdep \
  libeigen3-dev libboost-all-dev libceres-dev \
  "libopencv-dev=$DOCK_OPENCV_VERSION" "libopencv-contrib-dev=$DOCK_OPENCV_VERSION" \
  "python3-opencv=$DOCK_OPENCV_VERSION" python3-numpy python3-scipy python3-yaml python3-pytest \
  ros-jazzy-cv-bridge ros-jazzy-image-transport ros-jazzy-tf2-geometry-msgs
if ((DOCK_MAPPING)); then sudo apt-get install --no-remove -y ros-jazzy-rtabmap-ros; fi
/usr/bin/python3 -m venv --system-site-packages "$DOCK_VENV"
"$DOCK_VENV/bin/python" -m pip install 'pupil-apriltags==1.0.4.post11' 'canlib==1.33.358'
fi
# Do not replace a working RSUSB SDK with a different PyPI wheel.
if ! "$DOCK_VENV/bin/python" -c 'import pyrealsense2' 2>/dev/null; then
  # Another Python that already has a working RSUSB pyrealsense2 (optional, set DOCK_EXISTING_RS_PYTHON); otherwise scripts/install_realsense.sh.
  DOCK_OLD_PYTHON="${DOCK_EXISTING_RS_PYTHON:-}"
  if [[ -n "$DOCK_OLD_PYTHON" && -x "$DOCK_OLD_PYTHON" ]]; then
    DOCK_OLD_ABI="$("$DOCK_OLD_PYTHON" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    DOCK_NEW_ABI="$("$DOCK_VENV/bin/python" -c 'import sys; print(f"{sys.version_info.major}.{sys.version_info.minor}")')"
    if [[ "$DOCK_OLD_ABI" == "$DOCK_NEW_ABI" ]] && "$DOCK_OLD_PYTHON" -c 'import pyrealsense2' 2>/dev/null; then
      DOCK_RS_MODULE="$("$DOCK_OLD_PYTHON" -c 'import pyrealsense2; print(pyrealsense2.__file__)')"
      "$DOCK_VENV/bin/python" - "$DOCK_RS_MODULE" <<'PY'
import pathlib,shutil,sys,sysconfig
source=pathlib.Path(sys.argv[1]).resolve()
# Copy only this SDK package/extension (a copy, not a link: the old environment will be deleted).
root=source.parent if source.name=='__init__.py' else source
name='pyrealsense2' if root.is_dir() else root.name
dest=pathlib.Path(sysconfig.get_paths()['purelib'])/name
if not dest.exists():
    shutil.copytree(root,dest) if root.is_dir() else shutil.copy2(root,dest)
    for extra in root.parent.glob('pyrealsense2*'):   # sibling .so / dist-info next to the module
        target=dest.parent/extra.name
        if extra!=root and not target.exists():
            shutil.copytree(extra,target) if extra.is_dir() else shutil.copy2(extra,target)
PY
    fi
  fi
fi
mkdir -p "$DOCK_WS/src" "$DOCK_WS/upstream"
touch "$DOCK_WS/upstream/COLCON_IGNORE"
DOCK_OV_COMMIT=69488123ed9362dd44b6f28e7f4680abbff1442b
DOCK_OV_SRC="$DOCK_WS/upstream/open_vins"
if [[ ! -d "$DOCK_OV_SRC/.git" ]]; then
  git clone https://github.com/rpng/open_vins.git "$DOCK_OV_SRC"
fi
if [[ -n "$(git -C "$DOCK_OV_SRC" status --porcelain)" ]]; then echo 'OpenVINS source has local changes; leaving them untouched.' >&2; exit 2; fi
if [[ "$(git -C "$DOCK_OV_SRC" rev-parse HEAD)" != "$DOCK_OV_COMMIT" ]]; then
  git -C "$DOCK_OV_SRC" fetch origin "$DOCK_OV_COMMIT"
  git -C "$DOCK_OV_SRC" checkout --detach "$DOCK_OV_COMMIT"
fi
/usr/bin/python3 "$DOCK_APP/scripts/prepare_openvins.py" "$DOCK_OV_SRC" "$DOCK_WS/src/open_vins"
# Symlinks track subsequent code updates; re-run installer after bridge changes.
for DOCK_PACKAGE in docking_msgs docking_openvins; do
  DOCK_DEST="$DOCK_WS/src/$DOCK_PACKAGE"
  if [[ -e "$DOCK_DEST" && ! -L "$DOCK_DEST" ]]; then echo "Existing directory: $DOCK_DEST" >&2; exit 2; fi
  ln -sfn "$DOCK_APP/ros2/$DOCK_PACKAGE" "$DOCK_DEST"
done
set +u
source /opt/ros/jazzy/setup.bash
set -u
if ((!DOCK_NO_APT)); then
  if [[ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]]; then sudo rosdep init; fi
  rosdep update
  rosdep install --from-paths "$DOCK_WS/src" --ignore-src --rosdistro jazzy -y
fi
exec 9>"$DOCK_WS/.docking_rebuild.lock"
flock -n 9 || { echo '다른 브리지 재빌드가 진행 중입니다.' >&2; exit 2; }
touch "$DOCK_WS/.docking_rebuild_pending"
/usr/bin/python3 "$DOCK_APP/scripts/build_support.py" "$DOCK_APP/ros2" --workspace "$DOCK_WS"
cd "$DOCK_WS"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 /usr/bin/colcon build --base-paths "$DOCK_WS/src" --packages-up-to docking_openvins \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$DOCK_APP/scripts/environment.sh"
cd "$DOCK_APP"
"$DOCK_PYTHON" -m docking.preflight check --sensor-only
rm "$DOCK_WS/.docking_rebuild_pending"
"$DOCK_PYTHON" -m unittest discover -s tests -q || echo '코드 검사 실패가 있습니다 (빌드는 완료, 위 출력 확인)' >&2
printf '%s\n' '설치/소프트웨어 검사 완료. 실제 센서 확인: bash scripts/run_nano.sh measure --seconds 90 --view'
