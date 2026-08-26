#!/bin/bash
# realsense-viewer 에 AprilTag 필터를 넣어 빌드·설치한다.
#
# 소스는 이 폴더(git 추적)에 두고, SDK 트리에는 심볼릭 링크만 건다.
# librealsense/ 는 .gitignore 대상이라 SDK 트리에 둔 코드는 저장소에 안 남기 때문이다.
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
RS="$HERE/../../../librealsense"
PS=/mnt/c/WINDOWS/System32/WindowsPowerShell/v1.0/powershell.exe

[ -d "$RS/build" ] || { echo "SDK 빌드 트리가 없다: $RS/build"; exit 1; }
pkg-config --exists apriltag || { echo "libapriltag-dev 가 없다: sudo apt install libapriltag-dev"; exit 1; }

echo "[1/4] 소스 링크"
ln -sf "$HERE/apriltag-detection.cpp" "$RS/tools/realsense-viewer/apriltag-detection.cpp"
grep -q "APRILTAG" "$RS/tools/realsense-viewer/CMakeLists.txt" \
  || { echo "  CMakeLists 에 apriltag 블록이 없다. cmake_snippet.txt 를 참고해 넣어라"; exit 1; }

echo "[2/4] 오버레이 패치 (박스가 작을 때 텍스트를 바깥에 그리게)"
if ! grep -q "KRRI: 박스 안에 안 들어가면" "$RS/common/viewer.cpp"; then
    # 패치 헤더가 a/common/... b/common/... 이라 -p1 로 a/ 를 벗기고 SDK 트리에서 적용한다.
    # (예전엔 -p0 -d "$RS/.." 였는데, 그러면 존재하지 않는 a/common/ 를 찾다 실패한다.
    #  이미 적용된 트리에서는 위 grep 이 먼저 걸려서 이 줄까지 안 와 여태 안 드러났다.)
    patch -p1 -d "$RS" < "$HERE/viewer_overlay.patch" || echo "  패치 실패 — 수동 확인 필요"
else
    echo "  이미 적용됨"
fi

echo "[3/4] 빌드"
cmake --build "$RS/build" -j"$(nproc)" --target realsense-viewer 2>&1 | tail -3

echo "[4/4] 설치 (root 권한은 윈도우 경유로 얻는다 — sudo 비번 불필요)"
$PS -NoProfile -Command "wsl -u root -e bash -lc '
  cp $RS/build/Release/realsense-viewer /usr/local/bin/realsense-viewer
  patchelf --set-rpath /usr/local/lib /usr/local/bin/realsense-viewer 2>/dev/null
  cp -a $RS/build/Release/librealsense2*.so* /usr/local/lib/ 2>/dev/null
  ldconfig'" 2>&1 | tr -d '\r' | tail -2

n=$(nm -C /usr/local/bin/realsense-viewer 2>/dev/null | grep -c apriltag_docking_pose || true)
echo
echo "  완료 — 필터 심볼 ${n}개.  realsense-viewer 실행 후 좌측 패널에서"
echo "  [AprilTag : Docking Pose] 체크박스를 켜라."
echo "  태그 크기는 ~/.realsense-config.json 의 apriltag.tag_size (현재 $(python -c "
import json,pathlib;p=pathlib.Path.home()/'.realsense-config.json'
print(json.loads(p.read_text()).get('apriltag.tag_size','미설정') if p.exists() else '미설정')" 2>/dev/null)) 로 바꾼다."
