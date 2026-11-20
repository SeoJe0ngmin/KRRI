#!/usr/bin/env bash
set -euo pipefail
RD_SYSTEM="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
exec bash "$RD_SYSTEM/update_jetson.sh" "$@"
