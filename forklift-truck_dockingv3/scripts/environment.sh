#!/usr/bin/env bash
# Source only. No changes to global shell configuration or old v5 environment.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
# One project folder, one venv, one workspace (2026-10-19 user decision): ~/.local/share/krri/.
# (2026-10-20: the krri-v6/docking-v6 fallbacks are gone; the old folders can be deleted.)
DOCK_RS_LIB="${DOCK_RS_PREFIX:-$HOME/.local/share/krri/realsense}/install/lib"
if [[ -d "$DOCK_RS_LIB" ]]; then export LD_LIBRARY_PATH="$DOCK_RS_LIB:${LD_LIBRARY_PATH:-}"; fi
DOCK_WS="${DOCK_WS:-$HOME/.local/share/krri/ros_ws}"
export DOCK_WS
DOCK_VENV="${DOCK_VENV:-$HOME/.venvs/krri}"   # one venv for the project (2026-10-19 user decision)
# ROS-generated scripts may reference unset shell variables.
DOCK_NOUNSET=0
case $- in *u*) DOCK_NOUNSET=1; set +u ;; esac
[[ ! -f /opt/ros/jazzy/setup.bash ]] || source /opt/ros/jazzy/setup.bash
[[ ! -f "$DOCK_WS/install/setup.bash" ]] || source "$DOCK_WS/install/setup.bash"
if ((DOCK_NOUNSET)); then set -u; fi
if [[ -z "${DOCK_PYTHON:-}" ]]; then
  if [[ -x "$DOCK_VENV/bin/python" ]]; then DOCK_PYTHON="$DOCK_VENV/bin/python"; else DOCK_PYTHON=python3; fi
fi
