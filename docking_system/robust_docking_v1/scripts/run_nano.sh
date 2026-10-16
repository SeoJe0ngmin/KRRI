#!/usr/bin/env bash
set -euo pipefail
RD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ "$(uname -s)" != Linux ]]; then
    echo "Run this on the Jetson Linux terminal." >&2
    exit 2
fi
RD_INTERPRETER="${RD_PYTHON:-python3}"
cd "$RD_ROOT"
if [[ $# -eq 0 ]]; then set -- measure --view; fi
RD_MODE="$1"
shift
case "$RD_MODE" in
    doctor|check-devices|calibrate-mount|measure|shadow|drive)
        exec "$RD_INTERPRETER" -m docking "$RD_MODE" --platform nano "$@" ;;
    *)
        exec "$RD_INTERPRETER" -m docking "$RD_MODE" "$@" ;;
esac
