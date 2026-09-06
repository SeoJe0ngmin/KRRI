#!/usr/bin/env bash
# 오프라인 설치 묶음 만들기 — 개발 PC(리눅스/WSL)에서 실행한다.
#
# 현장 노트북(Windows, Python 3.11)에 인터넷이 없어도 설치가 되도록,
# requirements.txt 의 모든 패키지를 윈도우용 휠로 미리 받아 둔다.
#
#     ./tools/make_offline_bundle.sh
#     -> apriltag/offline_wheels/ 에 .whl 들이 쌓임 (gitignore 됨)
#     -> 저장소 폴더째 USB 에 복사하면, 노트북에서 setup_windows.bat 가
#        offline_wheels/ 를 발견하고 인터넷 없이 설치한다
#
# Kvaser 드라이버 설치파일(kvaser_drivers_setup.exe)은 pip 가 아니라서
# 여기 안 들어간다 — kvaser.com 에서 따로 받아 USB 에 같이 넣을 것.
set -e
cd "$(dirname "$0")/.."

OUT=offline_wheels
mkdir -p "$OUT"

# --platform win_amd64: 윈도우용 휠을 받는다 (순수 파이썬 휠은 자동 포함)
# --only-binary=:all: 소스 배포판 금지 — 노트북에서 빌드가 필요 없게
/home/jeongmin/anaconda3/envs/krri/bin/python -m pip download -r requirements.txt -d "$OUT" \
    --platform win_amd64 --python-version 3.11 \
    --implementation cp --only-binary=:all:

echo
echo "완료: $OUT/ ($(ls "$OUT" | wc -l)개 파일, $(du -sh "$OUT" | cut -f1))"
echo "저장소 폴더째 USB 로 복사하면 노트북에서 setup_windows.bat 가 알아서 쓴다."
