# realsense-viewer AprilTag 필터

인텔 공식 `realsense-viewer` 안에서 AprilTag 검출·도킹 pose 를 보게 하는 확장이다.
viewer 를 새로 만드는 게 아니라, SDK 가 제공하는 **후처리 필터 슬롯**에 우리 알고리즘만 끼운다.

    realsense-viewer  ─┬─ 카메라 제어 / 스트림 / 노출·게인 UI   ← SDK 가 해줌
                       └─ [AprilTag : Docking Pose] 체크박스     ← 우리가 끼우는 것

## 왜 소스가 여기 있나

빌드는 SDK 트리(`../../../librealsense/`)에서 일어나지만, **`librealsense/` 는 `.gitignore` 대상**이라
거기 둔 코드는 저장소에 안 남는다. 그래서 원본은 여기 두고, 빌드할 때 SDK 트리로 심볼릭 링크만 건다.

    src/etc/viewer_filter/apriltag-detection.cpp      ← 원본. git 이 추적
              │ (symlink)
              ▼
    librealsense/tools/realsense-viewer/          ← 빌드만 여기서

## 쓰는 법

    ./build.sh          # 링크 → 빌드 → 설치
    realsense-viewer    # 좌측 패널에서 [AprilTag : Docking Pose] 체크

## 파일

| | |
|---|---|
| `apriltag-detection.cpp` | 필터 본체. 검출 + pose + 오버레이 |
| `cmake_snippet.txt`      | SDK 의 CMakeLists.txt 에 들어가는 내용 |
| `build.sh`               | 링크·빌드·설치 자동화 |

## 값이 파이썬과 같아야 한다

`lateral / forward / heading / tilt` 는 `src/models/tag_pose.py` 의 `docking_state()`, `tag_tilt_deg()` 와
**같은 규약**을 따른다. 특히 forward 는 AprilTag 태그 좌표계의 +z 가 태그 뒤를 향하는 것을 뒤집어
**양수 = 태그 앞쪽**으로 만든 값이다. 한쪽만 고치면 두 도구가 다른 값을 내므로 주의.
