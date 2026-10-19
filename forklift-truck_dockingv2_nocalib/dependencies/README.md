# 의존성

실행 코드가 기대는 외부 소스는 **OpenVINS 하나**다. 2026-10-19 에 RTAB-Map·ORB-SLAM3 검토 소스와 참고 코드 사본(WPILib·robot_localization·Visual_Based_Odometry)은 한 번도 쓰지 않아 지웠다.

| 저장소 | 고정 commit | 역할 | 라이선스 |
|---|---|---|---|
| [OpenVINS](https://github.com/rpng/open_vins) | `69488123ed93` | VIO (IR 스테레오 + IMU) | GPL v3 — [OpenVINS-LICENSE](OpenVINS-LICENSE) 보존 |

- commit 전체 SHA 와 LICENSE hash 는 [sources.lock.json](sources.lock.json)에 있다. `scripts/install_jetson.sh` 가 이 commit 을 clone 하고(옛 작업 공간에 있으면 복사), `scripts/prepare_openvins.py` 가 세 패키지(ov_core·ov_init·ov_msckf)의 CMakeLists 만 바꿔 C++17·OpenCV 4 로 빌드 사본을 만든다. 알고리즘 소스는 원본과 같아야 하며, 생성 사본에 수동 수정이 있으면 덮어쓰지 않고 중단한다.
- `configs/openvins_template.yaml` 과 네이티브 브리지(`ros2/docking_openvins`)는 OpenVINS 예제 설정에서 유래했고 GPL v3 조건을 따른다.
- 그 밖의 의존성(ROS 2 Jazzy, OpenCV 4.6, librealsense, pupil-apriltags, canlib)은 apt·pip 로 설치하며 `install_jetson.sh` 에 적혀 있다.
