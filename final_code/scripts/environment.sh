#!/usr/bin/env bash
# Source only. No changes to global shell configuration or older environments.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# 빌드 폴더 · RealSense SDK · 가상환경은 코드 폴더 옆(프로젝트 폴더 안)에 둔다 (2026-11-21 정민님 결정)
DOCK_BASE="$(cd "${DOCK_APP:-$(dirname "${BASH_SOURCE[0]}")/..}/.." && pwd)"
DOCK_RS_LIB="${DOCK_RS_PREFIX:-$DOCK_BASE/realsense}/install/lib"
if [[ -d "$DOCK_RS_LIB" ]]; then export LD_LIBRARY_PATH="$DOCK_RS_LIB:${LD_LIBRARY_PATH:-}"; fi
DOCK_WS="${DOCK_WS:-$DOCK_BASE/ros_ws}"
export DOCK_WS
DOCK_VENV="${DOCK_VENV:-$DOCK_BASE/.venv}"
# ROS-generated scripts may reference unset shell variables.
DOCK_NOUNSET=0
case $- in *u*) DOCK_NOUNSET=1; set +u ;; esac
[[ ! -f /opt/ros/jazzy/setup.bash ]] || source /opt/ros/jazzy/setup.bash
[[ ! -f "$DOCK_WS/install/setup.bash" ]] || source "$DOCK_WS/install/setup.bash"
if ((DOCK_NOUNSET)); then set -u; fi
if [[ -z "${DOCK_PYTHON:-}" ]]; then
  if [[ -x "$DOCK_VENV/bin/python" ]]; then DOCK_PYTHON="$DOCK_VENV/bin/python"; else DOCK_PYTHON=python3; fi
fi
