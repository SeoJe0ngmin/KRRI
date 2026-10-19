#!/usr/bin/env bash
# Source only. No changes to global shell configuration or old v5 environment.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# One project folder, one venv, one workspace (2026-10-19 user decision): ~/.local/share/krri/. The old
# krri-v6 locations are still honoured until they are deleted, so nothing breaks mid-migration.
DOCK_RS_LIB="${DOCK_RS_PREFIX:-$HOME/.local/share/krri/realsense}/install/lib"
[[ -d "$DOCK_RS_LIB" ]] || DOCK_RS_LIB="$HOME/.local/share/krri-v6/realsense/install/lib"
if [[ -d "$DOCK_RS_LIB" ]]; then export LD_LIBRARY_PATH="$DOCK_RS_LIB:${LD_LIBRARY_PATH:-}"; fi
if [[ -z "${DOCK_V6_WS:-}" ]]; then
  DOCK_V6_WS="$HOME/.local/share/krri/ros_ws"
  [[ -d "$DOCK_V6_WS/install" || ! -d "$HOME/.local/share/krri-v6/ros_ws/install" ]] || DOCK_V6_WS="$HOME/.local/share/krri-v6/ros_ws"
fi
export DOCK_V6_WS
DOCK_V6_ENV="${DOCK_V6_ENV:-$HOME/.venvs/krri}"   # one venv for the project (2026-10-19 user decision)
# ROS-generated scripts may reference unset shell variables.
DOCK_NOUNSET=0
case $- in *u*) DOCK_NOUNSET=1; set +u ;; esac
[[ ! -f /opt/ros/jazzy/setup.bash ]] || source /opt/ros/jazzy/setup.bash
[[ ! -f "$DOCK_V6_WS/install/setup.bash" ]] || source "$DOCK_V6_WS/install/setup.bash"
if ((DOCK_NOUNSET)); then set -u; fi
if [[ -z "${DOCK_PYTHON:-}" ]]; then
  if [[ -x "$DOCK_V6_ENV/bin/python" ]]; then DOCK_PYTHON="$DOCK_V6_ENV/bin/python"; else DOCK_PYTHON=python3; fi
fi
