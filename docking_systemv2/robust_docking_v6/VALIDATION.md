# v6 구현 검사 기록

2026-10-16, macOS arm64. Python 3.10.19 / NumPy 1.26.4 / OpenCV 4.12.

## 통과한 검사

- `python -m unittest discover -s tests -q`: **51개 통과**. 실제 장치 없이 합성 관측과 모의 CAN 전송을 사용했다.
- 지연된 태그의 촬영 시각별 재처리, 중복·이상값 거부, 상관된 프레임 반복으로 공분산이 0에 수렴하지 않는지 확인.
- 1.2m 실제 관측 이동과 태그 좌표 보정의 구분, VIO 원점의 불확실성 중복 계산 방지, 추적 연속성 상실 시 무효화.
- IMU 200Hz 표본 유지·가속도 보간·누락 감지, 정지 판단, 포크 끝 오차 전파·진입 거부, 준비/주행/정지 상태 전환.
- 장착 보정, 퇴화한 hand-eye 입력 거부, 렌즈/해상도/시리얼 바인딩, OpenCV가 생성된 OpenVINS YAML을 읽는지 확인.
- 정지 시 반복성과 실제 위치 편향을 구분하는 평가, camera-only의 주행 사용 거부.
- Python·셸 문법, JSON·ROS 패키지 XML, 실행 스크립트 도움말.
- v6 `canbus.py`, `canhealth.py`가 v5 원본과 바이트 단위로 동일함을 확인.
- 기존 v5/common/update 파일 및 보관한 공식 소스·참고 스냅샷 hash 유지 확인.
- OpenVINS 빌드 사본을 두 번 생성해 재실행 가능함을 확인. 알고리즘/헤더/데이터 109개 파일은 원본과 동일하며 생성 사본에 수동 변경이 있으면 중단한다.

## Jetson 설치 의존성 수정 — 2026-10-16

사용자 장비에서 NVIDIA `libopencv-dev` 4.8(우선순위 600)과 설치된 Ubuntu OpenCV 4.6(500)이 충돌함을 패키지 목록으로 확인했다. 설치 명령은 Ubuntu contrib의 4.6 버전과 개발/Python 패키지를 일치시키며, 전역 apt 우선순위는 바꾸지 않는다. 직접 수행하는 apt 설치에 `--no-remove`를 적용했다. [APT 공식 버전 지정·삭제 방지 옵션](https://manpages.ubuntu.com/manpages/noble/man8/apt-get.8.html).

수정 스크립트의 셸 문법 검사 및 모의 apt 입력 4개(4.6 선택, 후보 없음, 예상 밖 4.8 후보, apt 실패 전파)를 확인했다. 실제 패키지 설치·의존성 해결과 네이티브 빌드는 Jetson 재실행으로 확인해야 한다. 런타임 코드는 변경하지 않았다.

## 문서·전체 코드 복사 확인 — 2026-10-16

두 도식의 PNG/SVG를 현재 실행 경로에 맞춰 다시 생성하고 화면으로 확인했다. RTAB-Map 기록 경로와 미구현 ORB 자동 전환을 제어 보정과 구분했다.

루트 `update_jetson_all.sh`를 macOS 기본 rsync로 임시 로컬 대상에 시험했다(SSH만 대체). 미리보기 무변경, 첫 복사, 재복사에서 코드·문서·도식 포함 및 기존 common.json·로그·기존 폴더 보존을 확인했다. `/./` 접두부 처리에 의존하지 않고 상대 경로로 전달한다. 실제 Jetson 전송은 이 검사에 포함하지 않았다.

## 아직 실행하지 않은 검사

- Ubuntu 24.04 arm64 / ROS Jazzy에서 C++ 브리지 컴파일·링크 및 DDS 통신.
- 실제 RealSense의 IR stereo+color+depth+IMU 동시 수집, 공장 보정·타임스탬프·초기화.
- Jetson CPU/RAM/온도/처리 지연, RTAB-Map 지도 생성.
- Kvaser 실제 송수신, 차량 제동·진동·태그 소실 구간의 위치 정확도.

따라서 소프트웨어 모의 검사 통과를 실차 ±3cm 정확도나 설치 성공의 보증으로 해석하지 않는다. Jetson의 설치 스크립트는 네이티브 빌드·소프트웨어 검사를 수행한다. 이후 `run.md`의 measure → 현장 입력·mount → shadow/drive 순서로 실제 장치 동작을 확인한다. RTAB-Map은 지도 기록/후보 진단용이며 ORB-SLAM3 자동 전환은 구현하지 않았다.

### 2026-10-16 ROS manifest / RealSense extrinsics startup repair
- Replaced invalid `maintainer@localhost` with reserved placeholder `maintainer@example.com` in both local ROS manifests (not a real support address).
- Resolve requested calibration modes from a single SDK device instance, matching stream type/index/format/fps/video size. Avoid cross-instance extrinsics queries between independent pipeline resolutions. Keep IMU-before-video start order.
- Missing/ambiguous profiles or unavailable factory extrinsics still fail; errors name the transform. No fabricated calibration fallback.
- Existing 51 unit tests and 3 added profile-graph/error tests passed locally. `git diff --check` passed. Local environment has no catkin_pkg; actual rosdep/colcon validation and physical D435i validation remain on Jetson.

### 2026-10-16 bounded vibration-aware settling
- Startup-only 6s baseline, 0.25s median blocks, 2s post-neutral window; trend and block excursion reject drift/rebound. Tracking/IMU loss and discontinuities reset observation windows. Capture-time filtering excludes delayed pre-neutral frames.
- Learned vibration is bounded and retained as pose/yaw uncertainty; existing docking uncertainty/3cm gates and CAN bytes unchanged. Native OpenVINS initialization unchanged.
- Synthetic tests cover above-old-limit vibration, translation drift, rotation, excessive gyro/vibration, out-and-back rebound, neutral transitions, visual loss and gaps. These do not validate physical forklift vibration or certify stop detection.
- Common site values set to vehicle width 1.3m and temporary bay width 1.4m as requested.
