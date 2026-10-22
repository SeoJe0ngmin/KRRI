#!/usr/bin/env bash
# Source only. Each numerical worker uses one BLAS/OpenMP thread.
export OPENBLAS_NUM_THREADS=1 OMP_NUM_THREADS=1 MKL_NUM_THREADS=1 NUMEXPR_NUM_THREADS=1
if [[ -n "${DOCK_PYTHON:-}" ]]; then
  :
elif [[ -x /home/jetson/.venvs/krri/bin/python ]]; then
  DOCK_PYTHON=/home/jetson/.venvs/krri/bin/python
else
  DOCK_PYTHON=python3
fi
