# 실제 검토한 코드와 v6 적용 방향

이 문서는 **코드 참고 결과**다. 아래 알고리즘이 이미 v6에 연결되었다는 뜻은 아니다. 전체 source SHA는 [lock](sources.lock.json), 작은 원본 스냅샷 hash는 [reference manifest](references/manifest.json)에 있다.

## OpenVINS — 직접 사용할 위치 추정 엔진

고정 버전 `69488123ed9362dd44b6f28e7f4680abbff1442b`.

| 원본 경로 | 확인한 내용 | v6에 적용할 방향 |
|---|---|---|
| [VioManager.cpp](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/core/VioManager.cpp) | IMU 수집, 카메라 처리, propagation, 시각 업데이트·ZUPT 호출 | 엔진은 활용하고 외부에서 태그·도킹 기준을 연결 |
| [State.h](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/State.h) / [Propagator.cpp](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/state/Propagator.cpp) | IMU 상태, 시간 구간별 전파, 카메라 시점 상태 | bias·공분산·촬영 시점 상태를 adapter에서 읽음. 내부 상태를 단순 xy/yaw 필터와 혼동하지 않음 |
| [ROS2Visualizer.cpp](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/ros/ROS2Visualizer.cpp) | `setup_subscribers`, `callback_inertial`, stereo callback, 카메라 큐 | IMU 시각이 영상보다 진행한 뒤 영상 처리. 내부 큐·락·QoS까지 지연 점검 |
| [UpdaterZeroVelocity.cpp](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/src/update/UpdaterZeroVelocity.cpp) | IMU·영상 기반 정지 판정과 업데이트 | 엔진 진동과 실제 미세 이동을 포함해 검증. CAN stop만으로 정지 강제 금지 |
| [CMakeLists.txt](https://github.com/rpng/open_vins/blob/69488123ed9362dd44b6f28e7f4680abbff1442b/ov_msckf/CMakeLists.txt) | Eigen/OpenCV/Boost/Ceres, ament 분기, OpenCV 일치 요구 | Jetson에서 빌드 시 ROS·OpenCV ABI 확인. 아직 빌드하지 않음 |

## RTAB-Map — 지도·재위치 보조

core `af1ced217f3a05a124bb418261f175baddb08adc`, ROS 2 `a8de404dc1c1cc3e6c3fd279827c8dc092080e8a`.

- [CoreWrapper.cpp](https://github.com/introlab/rtabmap_ros/blob/a8de404dc1c1cc3e6c3fd279827c8dc092080e8a/rtabmap_slam/src/CoreWrapper.cpp): `odom_frame_id`, 외부 odom 구독, stereo 입력과 변환 처리. OpenVINS odom과 좌우 IR를 연결할 지점이다.
- [Rtabmap.cpp](https://github.com/introlab/rtabmap/blob/af1ced217f3a05a124bb418261f175baddb08adc/corelib/src/Rtabmap.cpp): 지도 처리·loop closure와 최적화 경로를 검토한다. 지도 수정량을 차량의 순간 이동으로 해석하지 않는다.
- [Parameters.h](https://github.com/introlab/rtabmap/blob/af1ced217f3a05a124bb418261f175baddb08adc/corelib/include/rtabmap/core/Parameters.h): 키프레임·메모리·주기·특징점/등록 설정을 빌드 버전에 맞춰 정한다. 설정 이름을 추측해 넣지 않는다.

도킹 입구의 의미는 태그와 사용자 치수로 부여한다. 이미 OpenVINS를 사용해 만든 지도 pose를 독립 위치 센서처럼 중복 융합하지 않는다.

## ORB-SLAM3 — 선택적 스테레오 관성 추적

고정 버전 `4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4`.

- [stereo_inertial_realsense_D435i.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/Examples/Stereo-Inertial/stereo_inertial_realsense_D435i.cc): IR1/IR2 640×480 30fps, IMU 구간 전달, 비동기 자이로·가속도 보간, `TrackStereo` 입력을 확인했다. v6에서는 카메라 소유권을 공통 수집기로 옮긴 adapter가 필요하다. 예제의 모든 fallback 보간이나 보정 숫자를 무조건 복사하지 않는다.
- [Tracking.cc](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/src/Tracking.cc): `Relocalization()`은 키프레임 후보가 없으면 실패한다. `CreateMapInAtlas()`는 새 지도를 만들고 추적을 초기화한다. 이때 새 원점을 기존 odom과 이어 붙이면 안 된다.
- [System.h](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/include/System.h): 추적 상태·map 관련 인터페이스를 adapter 입력으로 사용할 계획이다. 반환 pose만으로 신뢰도를 판단하지 않는다.
- [build.sh](https://github.com/UZ-SLAMLab/ORB_SLAM3/blob/4452a3c4ab75b1cde34e5505a36ec3f9edcdc4c4/build.sh): 사전 압축 해제·Thirdparty 빌드 경로 확인용. 실행하지 않았다.

## WPILib — 설치하지 않고 지연 보정 참고

고정 버전 `34ef0d2a388fbc14974d6f9850ab57148d9cdfd1`. 이 버전은 Java package 경로가 `org/wpilib`다.

[PoseEstimator.java](https://github.com/wpilibsuite/allwpilib/blob/34ef0d2a388fbc14974d6f9850ab57148d9cdfd1/wpimath/src/main/java/org/wpilib/math/estimator/PoseEstimator.java)의 `addVisionMeasurement()`와 이력을 확인했다. 과거 시각의 odometry에서 비전 보정을 계산하고 현재까지 보상하는 방식을 참고한다. `setVisionMeasurementStdDevs()`의 이득은 `q / (q + sqrt(q*r))`, 기본 이력은 1.5초다. Q/R 설정을 고정하면 이득도 고정이며, full UKF의 시간별 공분산 전파와 다르다.

휠 모델과 API에 억지로 VIO 이동량을 휠 값처럼 넣지 않는다. v6의 지연 관측 처리는 IMU/영상 시각·보정 이벤트 이력에 맞춰 작성할 계획이다.

## robot_localization — 설치하지 않고 필터 처리 참고

고정 버전 `3efa714fb9c1ff40966327b7bed7053b2570be4d`, `jazzy-devel`.

- [ekf.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/3efa714fb9c1ff40966327b7bed7053b2570be4d/src/ekf.cpp): innovation의 Mahalanobis 검사와 Joseph 형태 covariance 갱신을 참고한다.
- [ros_filter.cpp](https://github.com/cra-ros-pkg/robot_localization/blob/3efa714fb9c1ff40966327b7bed7053b2570be4d/src/ros_filter.cpp): `smooth_lagged_data`, history와 과거 상태 복원 후 재처리를 참고한다.
- [ekf.yaml](https://github.com/cra-ros-pkg/robot_localization/blob/3efa714fb9c1ff40966327b7bed7053b2570be4d/params/ekf.yaml): 입력 축 선택·차분·지연 설정의 의미를 확인한다. 같은 값을 두 번 받는 문제를 `differential=true`만으로 해결한다고 보지 않는다.

표준 15상태 필터는 OpenVINS의 bias·카메라 clone 상태를 대체하지 않는다. OpenVINS 뒤에 같은 IMU를 다시 넣는 15상태 EKF를 기본 설치하지 않는다.

## Visual_Based_Odometry — 진단 방식 참고

고정 버전 `84a40ab43369a748e40130d1c3970c1d6217b372`.

- [tag_movement_tracker.py](https://github.com/jerWenger/Visual_Based_Odometry/blob/84a40ab43369a748e40130d1c3970c1d6217b372/visual_odometry/visual_odometry/tag_movement_tracker.py): 같은 태그 검출 시각의 TF를 구해 초기 대비 이동을 wheel odom과 비교한다.
- [helper_functions.py](https://github.com/jerWenger/Visual_Based_Odometry/blob/84a40ab43369a748e40130d1c3970c1d6217b372/visual_odometry/visual_odometry/helper_functions.py): `can_transform`/`lookup_transform`을 사용한다. v6에서도 TF를 문자열 해석으로 처리하지 않고 시간·축·유효성을 명시한다.

여기에는 OpenVINS를 대체할 특징점 VIO나 완성된 칼만 융합기가 없다. 태그가 사라진 뒤 일반 영상만으로 계속 이동을 추적하는 부분은 README의 향후 작업에 해당한다. 대기 중인 TF 요청이 제어 루프를 막지 않도록 구현해야 한다. 원본 코드 자체는 v6 실행 모듈로 복사하지 않았다.

## 현재 v5에서 보존할 내용

[canbus.py](../../robust_docking_v5/docking/canbus.py), [planner.py](../../robust_docking_v5/docking/planner.py), [controller.py](../../robust_docking_v5/docking/controller.py), [config.py](../../robust_docking_v5/docking/config.py)를 대조했다.

CAN 바이트·watchdog, 첫 동작만 실행하는 반복 재계획, 중앙/좌측 목표, 공통 치수 입력 구조를 보존한다. 기존 상수나 학습 반응값을 현재 위치의 측정치로 승격하지 않는다. 검토 시점 파일 hash를 lock에 남겼으며 기존 파일은 수정하지 않았다.
