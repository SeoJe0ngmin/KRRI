#!/usr/bin/env bash
set -euo pipefail
RD_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$RD_ROOT/docking_system/update_jetson.sh" "$@"
