#!/bin/bash
# RealSense D435i 를 WSL2 에 붙인다.
#
# WSL2 는 재부팅하거나 USB 를 다시 꽂으면 연결이 풀린다. 그때마다 이 스크립트를 돌리면 된다.
# 하는 일 세 가지:
#   1. 윈도우쪽 usbipd 로 장치를 WSL 에 넘긴다        (interop 으로 powershell.exe 호출)
#   2. uvcvideo 커널 모듈을 올린다                    (wsl -u root 경유. sudo 비번 불필요)
#   3. 장치 노드 권한을 plugdev 로 맞춘다
#
# 3번이 필요한 이유: WSL2 에는 udev 데몬이 없어서 /etc/udev/rules.d 규칙이 자동 적용되지 않는다.
# 그래서 /dev/video* 와 /dev/bus/usb/* 가 root 전용(600) 으로 남는다. 수동으로 열어준다.
set -u
PS=/mnt/c/WINDOWS/System32/WindowsPowerShell/v1.0/powershell.exe
USBIPD="/mnt/c/Program Files/usbipd-win/usbipd.exe"
VID_PID="8086:0b3a"          # D435i. 다른 모델이면 usbipd list 로 확인해서 바꾼다

echo "[1/3] usbipd attach"
BUSID=$("$USBIPD" list 2>/dev/null | tr -d '\r' | awk -v vp="$VID_PID" '$2==vp{print $1; exit}')
if [ -z "${BUSID:-}" ]; then
    echo "      장치를 못 찾음. USB 케이블 확인 후 'usbipd list' 로 직접 확인할 것."
    exit 1
fi
STATE=$("$USBIPD" list 2>/dev/null | tr -d '\r' | awk -v b="$BUSID" '$1==b{print $NF}')
if [ "$STATE" = "Attached" ]; then
    echo "      이미 붙어 있음 (busid $BUSID)"
else
    "$USBIPD" attach --wsl --busid "$BUSID" 2>&1 | tr -d '\r' | sed 's/^/      /'
    sleep 2
fi

echo "[2/3] uvcvideo 로드"
$PS -NoProfile -Command "wsl -u root -e bash -lc \"modprobe uvcvideo && echo loaded\"" 2>&1 | tr -d '\r' | sed 's/^/      /'

echo "[3/3] 권한 설정"
$PS -NoProfile -Command "wsl -u root -e bash -c 'chgrp plugdev /dev/video* 2>/dev/null; chmod g+rw /dev/video* 2>/dev/null; find /dev/bus/usb -type c -exec chgrp plugdev {} + -exec chmod g+rw {} +'" 2>&1 | tr -d '\r' | sed 's/^/      /'

echo
python -c "
import pyrealsense2 as rs
d = rs.context().query_devices()
if len(d)==0:
    print('  실패 - 장치가 안 보인다. src/etc/realsense_check.py 로 진단할 것.'); raise SystemExit(1)
for x in d:
    print(f'  준비 완료 - {x.get_info(rs.camera_info.name)}  '
          f'serial {x.get_info(rs.camera_info.serial_number)}  '
          f'USB {x.get_info(rs.camera_info.usb_type_descriptor)}')
"
