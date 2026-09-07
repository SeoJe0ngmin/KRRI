#!/usr/bin/env bash
# 현장 노트북에 들고 갈 것만 골라서 한 폴더에 담는다.
#
#     ./tools/make_field_bundle.sh /mnt/e/KRRI       (USB 가 E: 인 경우)
#     ./tools/make_field_bundle.sh ~/field_bundle    (일단 홈에 만들고 나중에 복사)
#
# 빼는 것 — 노트북에서 필요 없거나 다시 만들어지는 것:
#     librealsense/   555MB 소스 클론. pyrealsense2 는 pip 로 들어온다
#     work_dirs/      지난 주행 기록
#     legacy/         옛 코드
#     .git/           이력. 현장에서 clone 할 게 아니면 불필요
#     __pycache__/    파이썬 캐시
#
# 넣는 것: 코드 전부 + requirements.txt + setup_windows.bat + offline_wheels/
set -e
cd "$(dirname "$0")/.."

DEST="${1:?쓰는 법: ./tools/make_field_bundle.sh <목적지폴더>   (예: /mnt/e/KRRI)}"
mkdir -p "$DEST/apriltag"

rsync -a --delete \
    --exclude 'librealsense/' \
    --exclude 'work_dirs/' \
    --exclude 'legacy/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    ./ "$DEST/apriltag/"

if [ ! -d offline_wheels ]; then
    echo "!! offline_wheels/ 가 없다 — 인터넷 없는 현장이면 먼저 만들 것:"
    echo "   ./tools/make_offline_bundle.sh"
fi

cat > "$DEST/읽어보기.txt" <<'TXT'
현장 노트북 설치 순서
=====================

이 USB 에 없는 것 두 개를 먼저 설치해야 한다 (인터넷에서 받아 둘 것):

  1. Python 3.11          python.org  — 설치할 때 "Add Python to PATH" 체크
  2. Kvaser Drivers for Windows       — kvaser.com > Downloads
                                        (CAN 통신용. pip 로는 절대 안 받아진다)

그 다음:

  3. 이 폴더를 노트북 하드디스크로 복사   (USB 에서 바로 돌리지 말 것)
  4. apriltag\setup_windows.bat  더블클릭
  5. 끝에 "판정: 준비 완료" 가 뜨는지 확인

리허설 (지게차 없이, 카메라만 꽂고):

  python tools\realsense_check.py       카메라 진단
  python tools\imu_check.py             IMU 부호 확인 (반시계 90도 -> +90 이면 OK)
  python tools\run.py --dry-run --show  알고리즘 전체 (CAN 없이)

현장:

  python tools\run.py --show --record-events    SPACE 시작 / ESC 비상정지
  python tools\analyze_run.py                   끝나고 결과·파라미터 확인

자세한 것은 apriltag\SETUP_WINDOWS.md
TXT

echo "완료: $DEST  ($(du -sh "$DEST" | cut -f1))"
echo "  - apriltag/       코드 + offline_wheels"
echo "  - 읽어보기.txt    노트북에서 할 순서"
