#!/bin/bash
set -uo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
echo 'RealSense 카메라·IMU와 Kvaser CAN 연결을 검사합니다.'
echo '카메라 영상만 잠깐 읽으며, 차량 제어 명령은 보내지 않습니다.'
PYTHONDONTWRITEBYTECODE=1 "$ROOT/.venv-jetson/bin/python" "$ROOT/CHECK_DEVICES_JETSON.py" --stream
status=$?
if [ "$status" = 0 ]; then
  echo '장치 검사 통과. 현장 CAN 배선·차량 상태를 확인한 뒤 실행 아이콘을 사용하세요.'
else
  echo '장치 연결 또는 설정 확인이 필요합니다. 위 NOT_READY 메시지를 확인하세요.'
fi
if [ -t 0 ]; then read -r -p '엔터를 누르면 창이 종료됩니다.'; fi
exit "$status"
