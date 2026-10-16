# v6 실행 요약

## 1. 맥 → Jetson 코드 복사

현재 USB 연결 SSH 별칭을 사용한다. 현장 common과 측정 로그는 기본적으로 보존한다.

```bash
bash /Users/seojeongmin/krri/docking_systemv2/update_jetson.sh --host jetson-wired
```

맥에서 수정한 **common까지 덮어쓸 때만** 끝에 `--with-common`을 붙인다. v6 폴더만 따로 복사하면 공통 설정이 빠지므로 `docking_systemv2` 전체를 복사한다. 실행 중에는 업데이트하지 않는다.

이전 AprilTag 코드까지 모두 보낼 때는 맥에서 다음 하나를 실행한다. 위 명령도 별도로 실행할 필요는 없다.

```bash
bash /Users/seojeongmin/krri/update_jetson_all.sh --host jetson-wired
```

`apriltag_v1~v6`, `docking_system`, `docking_systemv2`의 코드·문서를 추가/갱신한다. 측정 로그·과거 백업·가상환경은 제외하고 Jetson의 두 `common.json`은 이미 있으면 보존한다. 대상 파일을 삭제하지 않는다. VS Code에서 `/home/jetson/krri`를 열어 전체 폴더를 확인한다. 코드 복사는 각 버전의 라이브러리 설치를 대신하지 않는다.

v6의 [algorithm.md](algorithm.md), [오차 보정 도식](docs/images/error_correction.png), [전체 주행 도식](docs/images/overall_flow.png)도 두 전송 명령 모두에 포함된다.

## 2. Jetson 최초 설치 — 나중에 한 번

Ubuntu 24.04 / ROS 2 Jazzy 구성이다. 다운로드할 때 인터넷이 필요하며 설치 후 USB 접속·실행에는 인터넷이 필요하지 않다.

```bash
cd /home/jetson/krri/docking_systemv2/robust_docking_v6
bash scripts/install_jetson.sh
```

RTAB-Map 지도도 사용할 때는 `bash scripts/install_jetson.sh --mapped`.

Jetson 저장소의 OpenCV 4.8과 Ubuntu 4.6 패키지가 섞이지 않도록 설치 스크립트가 OpenCV 개발 패키지·contrib·Python 패키지를 Ubuntu 4.6의 같은 버전으로 지정한다. 전역 패키지 우선순위는 변경하지 않으며, 기존 패키지를 삭제해야 하는 설치는 중단한다. 이전 스크립트에서 `libopencv-contrib-dev` 의존성 오류가 났다면 맥에서 코드를 다시 복사하고 위 설치 명령을 다시 실행한다.

기존 RealSense SDK는 Python 버전이 맞으면 그대로 연결한다. `pyrealsense2` 누락/ABI 오류만 남으면 다음을 실행한다. 커널·기존 SDK를 교체하지 않는 별도 RSUSB 빌드다.

```bash
bash scripts/install_realsense.sh
bash scripts/preflight.sh check --sensor-only
```

ROS 기본 패키지와 OpenCV 등 apt 의존성은 시스템에, OpenVINS와 브리지는 `~/.local/share/krri-v6/ros_ws`, 전용 Python은 `~/.venvs/docking-v6`에 둔다. 기존 `~/.venvs/krri`는 그대로 둔다. `(krri)`를 먼저 활성화할 필요 없이 실행 스크립트가 v6 전용 환경을 선택한다. 네이티브 브리지 코드가 업데이트됐으면 설치 스크립트를 다시 실행하여 증분 빌드한다. Kvaser 시스템 드라이버는 기존 설치를 사용한다.

설치 스크립트는 OpenVINS 원본을 보존하고 별도 빌드 사본의 CMake 설정만 C++17/Jazzy용으로 생성한다. 추정 알고리즘은 원본과 같으며 사용하지 않는 예제·시각화 실행 파일은 빌드하지 않는다.

## 3. 카메라와 추정 확인 — CAN 미사용

아래 실행 스크립트가 Python/ROS 환경을 로딩한다. 별도 `source activate`는 필요 없다.

```bash
cd /home/jetson/krri/docking_systemv2/robust_docking_v6
bash scripts/preflight.sh check --sensor-only
bash scripts/run_nano.sh measure --seconds 90 --view
```

화면은 **IR 영상+특징점**, 오른쪽은 컬러 태그 상태·현재 위치·포크 오차다. 컬러 태그 사각형을 IR 영상 위에 잘못 그리지 않는다. VS Code에서 8765 포트를 전달하고 맥의 `http://127.0.0.1:8765`를 연다.

OpenVINS 설치 전에 영상과 태그만 확인할 때:

```bash
bash scripts/run_nano.sh measure --camera-only --seconds 60 --view
```

`--camera-only`는 위치 추정·주행 준비 완료 판정을 하지 않는다. OpenVINS 초기화가 끝나도 태그가 유효하게 관측되기 전에는 입구 기준 pose가 없다.

## 4. 현장 입력·재장착 보정

`/home/jetson/krri/docking_systemv2/configs/common.json`의 `value`를 수정한다.

- 실측 차폭, 탑재부 내부 폭, 좌측 태그 거리(좌측 모드), 태그 크기·높이, 카메라 높이, 카메라→포크 끝 거리.
- 기본 목표: 중앙은 **포크 끝이 입구 0.5m 앞**, 좌측은 **포크 끝이 입구를 지나 1m**.
- 장착각 보정은 차량이 수평이고 태그면 법선과 차량 방향을 **독립적으로 맞춘 상태**에서 한다. 태그를 봤다는 사실만으로 이 기준을 알 수는 없다.

```bash
bash scripts/preflight.sh mount --reference-aligned --seconds 60 --view
bash scripts/preflight.sh check --mode central
# 좌측 모드는 입력 후:
bash scripts/preflight.sh check --mode left
```

mount는 최대 준비 시간을 두고 정지 관측을 수집한다. 통과 시 컬러에서 측정한 장착각을 공통 IR 기준으로 변환하여 common에 저장한다. 재장착했으면 다시 측정한다. 공통값·캘리브레이션 실패를 임의 숫자로 채우지 않는다.

## 5. 주행 흐름 검사와 실주행

```bash
# 상태 전환 확인, CAN 안 엶. 차량은 실제로 움직이지 않는다.
bash scripts/run_nano.sh shadow --mode central --view

# 실제 CAN 자동 도킹
bash scripts/run_nano.sh drive --mode central --execute --view
# 또는 좌측 태그
bash scripts/run_nano.sh drive --mode left --execute --view
```

시작 후 정지 관측 60초(준비 전체 최대 180초) → `READY` → SSH 터미널 **Enter**로 출발한다. `s`+Enter 또는 Ctrl+C로 정지한다. shadow는 명령을 표시하되 차량을 움직이지 않으므로 움직임 피드백이 없으면 동작 시간 제한으로 끝난다.

태그가 없으면 60°씩 최대 360° 탐색한다. 이후 최대 6단계 후보를 계획해 첫 구간을 실행하고 정지·되튐을 관측하여 다시 계획한다. 각 구간의 회전 뒤 전진도 최신 관측으로 방향·남은 거리·가시성·통로를 다시 확인한다. 마지막 진입은 좌우 ±3cm와 추정 오차 여유를 함께 검사한다. 방향은 이전 구간에서 맞추며 마지막에 별도 각도 허용 게이트를 중복 적용하지 않는다.

## 6. 선택 기능

```bash
# RTAB-Map 지도 기록 + 재방문 후보 진단. 기본 core보다 부하가 증가한다.
bash scripts/run_nano.sh measure --profile mapped --seconds 90 --view
```

`mapped`는 지도 DB를 이번 로그 폴더에 저장한다. 지도 보정은 `used_for_control: false`로 표시한다. 같은 VIO를 사용한 지도 후보를 검증 없이 위치에 더하지 않는다. ORB-SLAM3 자동 전환은 지원하지 않는다.

오프라인 도구를 쓸 터미널에서는 먼저 `source scripts/environment.sh`를 실행한다.

오프라인 렌즈 재보정: 실측 체커보드 여러 각도/영상 가장자리 15장 이상. `columns/rows`는 내부 교차점 수다.

```bash
"$DOCK_PYTHON" -m docking.calibration_tools intrinsics --images '/path/color/*.png' --columns 9 --rows 6 --square-m 0.025 --serial 실제시리얼 --stream color --output /home/jetson/krri/work/lens.json
```

독립 검증 후 결과의 `validated`를 true로 바꾸고 `--sensor-calibration /home/jetson/krri/work/lens.json`으로 mount/measure/run에 같은 파일을 지정한다. IR 보정이 바뀌면 mount도 다시 한다. 예시 체커보드 치수는 실물 값으로 바꾼다.

일반 hand-eye: 서로 평행하지 않은 축 회전이 포함된 독립 기준 자세 12개 이상. 평면 주행/정지 영상만으로 전체 외부 파라미터를 구할 수 없다.

```bash
"$DOCK_PYTHON" -m docking.calibration_tools hand-eye /path/independent_poses.json --output /home/jetson/krri/work/hand_eye_report.json
```

입력 형식은 `{"samples":[{"T_base_body":[4×4행렬],"T_camera_tag":[4×4행렬]}, ...]}`. 결과는 검토용이며 common을 자동 덮어쓰지 않는다. 일반 현장 재장착에서는 앞의 `preflight mount`를 사용한다.

코드 시험(센서/CAN 불필요): `DOCK_PYTHON=...`로 설치된 Python을 지정하거나 전용 환경에서 `python -m unittest discover -s tests -q`.

정지 준비 중에는 브레이크를 잡고 실제 정차를 유지한다. VIO 초기화 후 6초의 진동 기준 수집과 기존 60초 준비 측정을 거쳐 READY가 나온 뒤 Enter로 출발한다. 회전·직진 후에는 중립 이후의 2초 관측으로 잔여 이동·되튐을 확인한다. 실패 이유는 로그의 `stationary_report`에서 확인한다. 엔진 진동 허용과 ±3cm 도킹 정확도 검사는 별개다. OpenVINS 자체 초기화 실패는 이 판정으로 우회하지 않는다.

2026-10-16 공통 치수: 차폭 1.3m, 탑재부 내부 폭 1.4m(임시). 중앙에서 좌우 물리적 여유는 각각 5cm이며 정렬 허용 오차는 3cm를 유지한다. 이미 젠슨에 공통 설정이 있으면 일반 업데이트는 보존하므로, 해당 두 `value`를 젠슨에서도 수정하거나 의도적으로 `--with-common`을 사용한다(후자는 다른 공통값도 맥 값으로 덮어쓴다).
