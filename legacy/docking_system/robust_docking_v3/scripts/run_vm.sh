#!/usr/bin/env bash
# Launch in the Ubuntu guest, not in the macOS host.
set -euo pipefail
RD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(uname -s)" != Linux ]]; then
    echo "Run this inside the Ubuntu VM (VS Code Remote-SSH terminal or ssh ubuntu-vm)." >&2
    exit 2
fi
if [[ -n "${RD_PYTHON:-}" ]]; then
    RD_INTERPRETER="$RD_PYTHON"
elif [[ -x "$HOME/miniforge3/envs/krri/bin/python" ]]; then
    RD_INTERPRETER="$HOME/miniforge3/envs/krri/bin/python"
else
    RD_INTERPRETER=python3
fi
cd "$RD_ROOT"
if [[ $# -eq 0 ]]; then set -- measure --view; fi
RD_MODE="$1"
shift
case "$RD_MODE" in
    doctor|check-devices|calibrate-mount|measure|shadow|drive)
        exec "$RD_INTERPRETER" -m docking "$RD_MODE" --platform vm "$@" ;;
    *)
        exec "$RD_INTERPRETER" -m docking "$RD_MODE" "$@" ;;
esac
