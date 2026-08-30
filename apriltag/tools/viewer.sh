#!/bin/bash
# realsense-viewer 를 연다. 카메라가 안 붙어 있으면 먼저 붙인다.
#
#   rsview            그냥 연다
#   rsview --check    붙었는지만 보고 안 연다
set -uo pipefail
HERE="$(cd "$(dirname "$(readlink -f "${BASH_SOURCE[0]}")")" && pwd)"

if ! ls /dev/video* >/dev/null 2>&1; then
    echo "카메라가 안 붙어 있다. 연결한다..."
    "$HERE/wsl_attach_camera.sh" || { echo "연결 실패"; exit 1; }
fi

# 카메라는 한 프로세스만 연다. 누가 잡고 있으면 viewer 가 빈 화면으로 뜬다.
BUSY=$(fuser /dev/video* 2>/dev/null | tr -s ' ' '\n' | grep -v '^$' | sort -u)
if [ -n "$BUSY" ]; then
    echo "경고: 다른 프로세스가 카메라를 잡고 있다 (PID: $(echo $BUSY | tr '\n' ' '))"
    ps -o pid=,cmd= -p $BUSY 2>/dev/null | sed 's/^/      /'
    echo "      viewer 가 화면을 못 받을 수 있다. 그 프로세스를 먼저 끝내라."
fi

[ "${1:-}" = "--check" ] && { echo "카메라 준비됨: $(ls /dev/video* | tr '\n' ' ')"; exit 0; }
exec realsense-viewer "$@"
