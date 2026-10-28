#!/bin/bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
mkdir -p "$HOME/.cache/lifter-remote"
exec 8> "$HOME/.cache/lifter-remote/runtime.lock"
if ! flock -n 8; then
  echo 'Jetson control is already running. Use the independent monitor or stop it first.'
  exit 2
fi
export PYTHONDONTWRITEBYTECODE=1
export YOLO_AUTOINSTALL=false
"$ROOT/.venv-jetson/bin/python" "$ROOT/CHECK_DEVICES_JETSON.py" --stream
if [ ! -t 0 ]; then
  echo 'Open this launcher in a terminal to start vehicle control.'
  exit 2
fi
echo '이 실행은 카메라와 CAN을 켜고 자동 차량 제어를 시작합니다.'
echo '현장에서 장치 연결과 차량 주변을 확인한 뒤 실행하세요.'
read -r -p '실제 제어를 시작하려면 RUN 입력: ' response
[ "$response" = RUN ] || { echo '실행 취소'; exit 0; }
cd "$ROOT/depth_cam"
exec "$ROOT/.venv-jetson/bin/python" -u main_rec_v4.py "$@"
