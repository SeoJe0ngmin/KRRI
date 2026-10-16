# Robust docking v6

IR 좌우 영상·IMU를 **OpenVINS**로 추적하고, 컬러 AprilTag로 입구 기준 위치를 보정하는 실행 코드다. Depth는 태그 거리의 모순 검사에 사용한다. 실제 이동량은 관측에서 구하며 CAN 명령 시간으로 위치를 만들지 않는다.

- [복사·설치·실행 요약](run.md)
- [구현 구조와 사용 방법론](algorithm.md)
- [검사 결과와 아직 확인하지 못한 부분](VALIDATION.md)
- [오차 보정 도식](docs/images/error_correction.png) · [전체 흐름](docs/images/overall_flow.png)
- [의존성과 출처](dependencies/README.md)

`core`는 OpenVINS+컬러 태그, `mapped`는 여기에 RTAB-Map 지도 기록·loop 후보 확인을 추가한다. RTAB-Map 후보를 독립 위치 센서로 합치거나 추적 실패 때 자동 전환하지 않는다. ORB-SLAM3는 검토·소스 준비 상태이며 이번 실행 경로에는 넣지 않았다.

기존 6단계 후보 계획과 CAN 전송/감시 코드를 사용한다. 차폭·탑재부 폭·좌측 태그 위치는 `../configs/common.json`에서 읽고, 재장착 보정도 같은 파일에 저장한다. 로그·실행 당시 센서 보정값·지도는 `../../work/docking_systemv2/robust_docking_v6/`에 저장한다.

Python 모의 검증과 CAN 모의 전송 검사는 포함했다. **이 맥에서 ROS 2 C++ 브리지를 빌드하거나 Jetson 카메라·실차를 시험하지는 않았다.** Jetson에서 설치 스크립트가 실제 네이티브 빌드를 수행하고, 검사에 실패하면 원인을 표시한다. 라이브러리 없이 가짜 위치로 대체하는 경로는 없다.
