#!/usr/bin/env bash
# Existing Jetson installation only: no apt, downloads, sensors or CAN.
set -euo pipefail
DOCK_APP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$DOCK_APP/scripts/environment.sh"
[[ -f /opt/ros/jazzy/setup.bash && -x /usr/bin/colcon ]] || {
  echo 'ROS Jazzy 설치가 필요합니다: bash scripts/install_jetson.sh' >&2; exit 2;
}
for DOCK_DEP in ov_core ov_init ov_msckf; do
  [[ -f "$DOCK_WS/install/$DOCK_DEP/share/$DOCK_DEP/cmake/${DOCK_DEP}Config.cmake" ]] || {
    echo "기존 OpenVINS 빌드 없음: $DOCK_DEP; bash scripts/install_jetson.sh 필요" >&2; exit 2;
  }
done
exec 9>"$DOCK_WS/.docking_rebuild.lock"
flock -n 9 || { echo '다른 브리지 재빌드가 진행 중입니다.' >&2; exit 2; }
# Remains on failure, so a subsequent run cannot start the old executable.
touch "$DOCK_WS/.docking_rebuild_pending"
rm -rf "$DOCK_WS/src/docking_v6_msgs" "$DOCK_WS/build/docking_v6_msgs" "$DOCK_WS/install/docking_v6_msgs"   # 2026-10-20 rename docking_v6_msgs -> docking_msgs (generated output only)
/usr/bin/python3 "$DOCK_APP/scripts/build_support.py" "$DOCK_APP/ros2" --workspace "$DOCK_WS"
for DOCK_PACKAGE in docking_msgs docking_openvins; do
  DOCK_DEST="$DOCK_WS/src/$DOCK_PACKAGE"
  if [[ -e "$DOCK_DEST" && ! -L "$DOCK_DEST" ]]; then
    echo "기존 디렉터리를 덮어쓰지 않습니다: $DOCK_DEST" >&2; exit 2
  fi
  ln -sfn "$DOCK_APP/ros2/$DOCK_PACKAGE" "$DOCK_DEST"
done
cd "$DOCK_WS"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 /usr/bin/colcon build \
  --base-paths "$DOCK_WS/src" --packages-select docking_msgs \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$DOCK_APP/scripts/environment.sh"
cd "$DOCK_APP"
"$DOCK_PYTHON" -c 'from docking_msgs.msg import VioState; from docking.vio_quality import check_message_schema; check_message_schema(VioState)'
cd "$DOCK_WS"
CMAKE_BUILD_PARALLEL_LEVEL=1 MAKEFLAGS=-j1 /usr/bin/colcon build \
  --base-paths "$DOCK_WS/src" --packages-select docking_openvins \
  --parallel-workers 1 --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF
source "$DOCK_APP/scripts/environment.sh"
cd "$DOCK_APP"
"$DOCK_PYTHON" -m docking.preflight check --sensor-only
rm "$DOCK_WS/.docking_rebuild_pending"
echo '브리지 업데이트 완료. 확인: bash scripts/run_nano.sh measure --seconds 90 --view'
