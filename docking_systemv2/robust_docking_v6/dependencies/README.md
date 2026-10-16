# 의존성 준비 상태

2026-10-16에 공식 소스를 내려받아 버전을 고정했다. **다운로드 완료와 Jetson 설치·빌드 완료는 다르다.** 이번에는 빌드·설치 스크립트를 실행하지 않았으며 ROS, 카메라 드라이버, 기존 Python 환경을 변경하지 않았다.

## 다운로드한 공식 저장소

| 저장소 | 고정 commit | 역할 | 실제 로컬 디스크 사용량¹ | 원본 라이선스 |
|---|---|---|---:|---|
| [OpenVINS](https://github.com/rpng/open_vins) | `69488123ed93` | 기본 VIO | 534.2 MiB | GPL v3, 원본 LICENSE·헤더 보존 |
| [RTAB-Map](https://github.com/introlab/rtabmap) | `af1ced217f3a` | 지도 backend | 77.5 MiB | BSD 3-Clause |
| [RTAB-Map ROS 2](https://github.com/introlab/rtabmap_ros/tree/ros2) | `a8de404dc1c1` | ROS 2 인터페이스 | 12.5 MiB | BSD 3-Clause |
| [ORB-SLAM3](https://github.com/UZ-SLAMLab/ORB_SLAM3) | `4452a3c4ab75` | 선택적 보조 추적 | 1,618.6 MiB | GPL v3, Thirdparty별 라이선스 별도 |

¹ `.git`, 예제 자료, 포함된 사전 등을 포함한 `du -sk` 기준. 합계 약 **2.19 GiB**. 전송량·최소 설치 크기·실행 RAM 사용량을 뜻하지 않는다. 정확한 전체 SHA, 논리 파일 크기와 디스크 사용량은 [sources.lock.json](sources.lock.json)에 기록했다. 각 저장소는 준비 시 변경 없는 상태이며 Git submodule은 없었다. 포함된 Thirdparty 디렉터리는 그대로 보존했다.

원본 위치:

```text
/Users/seojeongmin/Documents/Codex/dependency_sources/robust_docking_v6/
├── open_vins/
├── rtabmap/
├── rtabmap_ros/
└── ORB_SLAM3/
```

기존 Jetson 업데이트는 `docking_systemv2` 전체를 복사한다. 위 소스는 그 경로 밖에 두어 일반 코드 업데이트에 큰 저장소나 중첩 `.git`이 따라가지 않게 했다. v6 Python 실행 코드·ROS 브리지 소스·문서·도식·작은 참고 스냅샷을 코드 트리에 둔다. Jetson용 소스 배치·빌드는 `../scripts/install_jetson.sh`로 수행한다. 이 스크립트는 아직 장치에서 실행하지 않았다.

## 가중치와 실행 부담

기본 선택은 전통적인 특징점·관성 추정이다. OpenVINS와 기본 RTAB-Map/ORB 구성을 실행하기 위한 대형 신경망 가중치를 별도로 다운로드하지 않았다. 선택적인 딥러닝 특징점·장소 인식 확장을 추가하는 경우는 별개다.

ORB의 `Vocabulary/ORBvoc.txt.tar.gz`는 신경망 가중치가 아닌 **BoW 시각 특징점 사전**이다.

- 압축 파일: **42,527,984 bytes**, 약 40.56 MiB.
- 압축 내부 `ORBvoc.txt`: **145,250,924 bytes**, 약 138.52 MiB.
- 압축 해제·로딩·빌드는 아직 하지 않았다. 파일 SHA-256은 lock에 기록했다.
- 실행 중 사전 자료구조, 키프레임, 지도점, 영상, 최적화 메모리가 추가된다. RAM 사용량은 실행하여 측정해야 한다.

세 프로그램을 동시에 실행하면 특징점 처리와 최적화가 겹친다. 다운로드 자체가 CPU를 계속 쓰는 것은 아니다. 기본 OpenVINS+태그, RTAB 추가, ORB 추가 프로필로 나누어 추정 지연·누락·peak RAM·온도를 측정한다. 성능 근거 없이 “Jetson에서 모두 실시간으로 충분하다”고 확정하지 않는다.

## Jetson 빌드 의존성

아래는 준비할 항목의 목록이며, 설치를 실행한 기록이 아니다. 모든 전이 의존성의 패키지 버전까지 고정된 오프라인 설치 묶음도 아직 아니다.

| 대상 | 필요한 기반 | 확인할 사항 |
|---|---|---|
| Jetson 기본 | Ubuntu 24.04 arm64, 실제 L4T/JetPack·카메라 모델 | 사용자 로그 기준 OS를 확인했지만 장치에서 새로 조회하지 않음 |
| ROS 2 Jazzy | 최소 ROS base, colcon, tf2, sensor/nav 메시지, cv_bridge, message_filters 등 | Mac에 Ubuntu 패키지를 설치하지 않음. 기존 Python venv와 ROS Python 경로 분리 |
| OpenVINS | Eigen3, OpenCV, Boost, Ceres, 선택한 ROS 2 메시지·도구 | 공식 ROS 2 코드가 있음. Jazzy/Tegra 조합 빌드는 미검증 |
| RealSense 수집 | 기존 librealsense 및 장치 드라이버, 선택 시 ROS 2 wrapper | 버전 충돌 없이 카메라 소유자를 하나로 구성. 자동으로 SDK 교체하지 않음 |
| RTAB-Map core/ROS | CMake, OpenCV, PCL 등 선택된 빌드 기능의 의존성 | GUI·고밀도 map 기능을 최소화. distro 패키지와 현재 소스 간 ABI 확인 |
| ORB-SLAM3 | Eigen3, OpenCV, Pangolin 및 포함된 DBoW2/g2o/Sophus | 공식 ROS 예제를 ROS 2 adapter로 간주하지 않음. ARM 컴파일 옵션 확인 |

설치 경로는 [ROS 2 Jazzy 공식 문서](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/Ubuntu-Install-Debs.rst)와 [OpenVINS 설치 문서](https://docs.openvins.com/gs-installing.html)를 기준으로 한다. OpenVINS 설치 문서의 오래된 Ubuntu/ROS 명령을 Ubuntu 24.04에 그대로 실행하지 않는다. OpenCV와 `cv_bridge`의 버전 일치가 필요하다.

이번 pin은 **검토한 소스의 식별자**다. 빌드 호환성·실차 성능이 검증된 조합이라는 의미는 아니다. 구현 과정에서 버전을 바꾸면 이유와 새 SHA를 기록하고 해당 검증을 다시 수행한다.

## 참고만 하는 코드

`references/`의 원본 스냅샷은 실행 경로에 포함하지 않는다. 출처·SHA·파일 hash는 [manifest.json](references/manifest.json)에 있다.

| 출처 | 보관·참고 내용 | 실행 의존성 |
|---|---|---|
| WPILib | PoseEstimator 및 라이선스: 촬영 시점 보정·odometry 이력 | 추가하지 않음 |
| robot_localization | EKF·ROS filter·설정 예시·라이선스: 이상 관측 검사·지연 재처리·공분산 갱신 | 추가하지 않음 |
| Visual_Based_Odometry | README와 코드 검토 출처. 태그/odom 시각 맞춤 진단 | 추가하지 않음 |

Visual_Based_Odometry의 코드 경로와 실제 역할은 [참고 코드 대응표](REFERENCE_CODE.md)에 정리했다. 이 저장소를 VIO 엔진으로 설명하거나 구현 코드를 본 프로젝트에 이식하지 않았다. 참고한 원본 코드의 라이선스·저작권 표시는 해당 파일 또는 원본 저장소에서 확인할 수 있다.

## 확인·재현

```bash
cd /Users/seojeongmin/krri/docking_systemv2/robust_docking_v6
python3 scripts/verify_preparation.py
```

이 검사는 로컬 source commit·변경 여부·LICENSE hash·사전 hash·참고 스냅샷 hash·문서/그림 존재를 확인한다. 카메라나 CAN 시험을 수행하지 않는다. 다른 컴퓨터로 원본 폴더를 옮겼다면 `--cache-root <원본이_있는_폴더>`를 지정한다.

원본을 다시 받을 때에는 각 공식 repository를 clone한 후 lock에 있는 **전체 SHA**로 checkout한다. 설치 단계에서 해당 commit이 사라졌거나 맞지 않으면 임의 최신 버전으로 계속하지 않는다. 참고 문서와 자료를 보려는 것만으로는 2.19GiB 원본을 Jetson에 복사할 필요가 없다.

## 이번 실행 코드의 선택

OpenVINS commit은 위 pin으로 빌드한다. RTAB-Map은 선택 시 ROS Jazzy apt 패키지로 설치하므로 위 검토 소스 pin과 동일 버전이라고 주장하지 않는다. ORB-SLAM3는 설치하지 않는다. 참고 자료와 설계 준비 검사 `verify_preparation.py`는 맥 소스 보관소용이며 Jetson 실행 전 검사는 `preflight.sh check`다. OpenVINS 예제 설정에서 유래한 `configs/openvins_template.yaml`과 네이티브 adapter에는 `OpenVINS-LICENSE`의 GPL v3 조건을 보존한다.

`prepare_openvins.py`는 `ros_ws/upstream/open_vins`의 고정 원본을 읽고 `ros_ws/src/open_vins`에 빌드 사본을 생성한다. 세 패키지의 CMakeLists만 바꿔 C++17·OpenCV 4와 추정 라이브러리만 빌드한다. 알고리즘 소스 109개 파일은 원본과 일치함을 확인했다. 기존 원본의 ROS 시각화 예제는 이전 헤더 이름을 사용하므로 v6 빌드에서 제외한다. 생성 파일에 수동 수정이 있으면 덮어쓰지 않고 중단한다.
