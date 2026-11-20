#!/bin/bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
export PYTHONDONTWRITEBYTECODE=1
exec "$ROOT/.venv-jetson/bin/python" -u "$ROOT/PREVIEW_JETSON.py" "$@" > "$ROOT/jetson-software/ui-preview.log" 2>&1
