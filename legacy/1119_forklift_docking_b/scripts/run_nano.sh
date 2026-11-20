#!/usr/bin/env bash
set -euo pipefail
DOCK_APP="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$DOCK_APP/scripts/environment.sh"
cd "$DOCK_APP"
exec "$DOCK_PYTHON" -m docking "$@"
