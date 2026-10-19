# forklift-truck_dockingv2_nocalib

AprilTag + OpenVINS(IR 스테레오·IMU) 로 지게차를 탑재부에 자동 도킹하는 코드. Jetson Orin Nano 에서 돈다.
**초기 보정 없음**: 카메라가 차 정면을 똑바로 보고(장착각 0) 태그가 수직·수평으로 바르게 붙었다고 전제한다. 줄자 값(태그·카메라 높이, 좌우 위치)만 `configs/common.json` 에 넣는다. 장착 yaw 는 주행 중 좋은 직진에서만 다듬는다. (`forklift-truck_docking` 의 2026-10-19 사본)
상세: [실행 순서](run.md) · [알고리즘](algorithm.md) · [검사 기록](VALIDATION.md)

## 연결

```
# ~/.ssh/config (맥)
Host jetson-wired        # USB 직결
    HostName 192.168.55.1
    User jetson
Host jetson-kw           # 광운대 핫스팟
    HostName 172.20.10.4
    User jetson
```

아래 `<host>` 는 둘 중 하나. 이 문서는 `jetson-wired` 로 적는다.

## 맥 → Jetson 코드 보내기

```bash
bash /Users/seojeongmin/krri/forklift-truck_dockingv2_nocalib/update_jetson.sh --host jetson-wired   # 바뀐 파일만, work/와 common.json 은 안 건드림
bash /Users/seojeongmin/krri/forklift-truck_dockingv2_nocalib/update_jetson.sh --host jetson-wired --with-common   # 맥 common.json 으로 덮어쓸 때만
```

## Jetson 설치 (한 번)

```bash
ssh jetson-wired
cd /home/jetson/krri/forklift-truck_dockingv2_nocalibv2_nocalib
bash scripts/install_jetson.sh            # ROS Jazzy·OpenCV 확인, 가상환경 ~/.venvs/krri, OpenVINS+브리지 빌드 (30~60분)
bash scripts/install_jetson.sh --no-apt   # 인터넷 없이 빌드만 (apt·pip·rosdep 건너뜀)
```

만들어지는 것: 가상환경 `~/.venvs/krri`, 빌드 `~/.local/share/krri/ros_ws`. 코드는 이 폴더 하나.

## 브리지 재빌드 (C++ 가 바뀌었을 때만)

```bash
cd /home/jetson/krri/forklift-truck_dockingv2_nocalibv2_nocalib
bash scripts/rebuild_openvins.sh          # 우리 패키지 둘만 다시 컴파일, 몇 분. 파이썬만 바뀐 배포는 불필요
```

안 하고 돌리면 "OpenVINS 브리지 업데이트 필요" 로 스스로 멈춘다.

## 실행 (Jetson, VS Code Remote-SSH 터미널)

```bash
cd /home/jetson/krri/forklift-truck_dockingv2_nocalibv2_nocalib
bash scripts/preflight.sh check --sensor-only                 # 설치·장치 점검, CAN 안 엶
bash scripts/run_nano.sh measure --view                       # 센서·융합만, CAN 안 엶. 화면 = IR 흑백 + OpenVINS 점. Ctrl+C 로 끝
bash scripts/run_nano.sh measure --camera-only --view         # OpenVINS 없이 카메라만. 화면 = 컬러 + 태그 사각형
bash scripts/run_nano.sh shadow --mode left --view            # 상태기계만, 차 안 움직임
bash scripts/run_nano.sh drive --mode left --execute --view      # 실주행(좌측 태그). READY 뒤 Enter → 태그 탐색 → 30초 정차 → 주행. s+Enter / Ctrl+C 정지
```

화면: VS Code 에서 8765 포트 전달 → 맥 브라우저 `http://127.0.0.1:8765`.
실행 전 `configs/common.json` 의 태그 크기·높이·카메라 높이·차폭·탑재부 폭·뒤 공간·좌측 태그 거리를 실제 값으로. 장착 yaw 는 0 에서 시작해 주행 중 좋은 직진마다 다시 배운다.

## 로그

```bash
ls -t /home/jetson/krri/forklift-truck_dockingv2_nocalib/work | head -1                       # 가장 최근 판
scp -r jetson-wired:/home/jetson/krri/forklift-truck_dockingv2_nocalib/work/<폴더> /Users/seojeongmin/krri/forklift-truck_dockingv2_nocalib/work/   # 맥으로
```

`observations.jsonl`(관측 전부), `control.jsonl`(명령·CAN), `pipeline_status.json`(1초마다 센서·GC·프레임 공백), `openvins.log`.

## CPU 보기 (Jetson 두 번째 터미널)

```bash
tegrastats --interval 1000                                    # 코어별 %, RAM, 온도
top -d 1 -p $(pgrep -d, -f 'python.* -m docking|docking_openvins/bridge')   # 우리 프로세스만 (%는 코어 수 × 100 이 상한)
sudo jetson_clocks                                            # 실주행 때 클럭 고정 (재부팅까지)
```

## IMU 멈춤 조사 (Jetson, 앱과 무관하게 SDK 만)

```bash
source ~/.venvs/krri/bin/activate
python scripts/check_imu_cadence.py imu    --seconds 60   # 자이로+가속도만
python scripts/check_imu_cadence.py ir     --seconds 60   # + IR 좌우 640x480@30
python scripts/check_imu_cadence.py full   --seconds 60   # + 컬러 1280x720@30 (주행 구성, depth 끔)
```

스트림마다 호스트 도착 간격의 최대·100 ms/500 ms 초과 횟수와, 가장 큰 공백에서의 장치 시각 간격을 찍는다(장치 시각은 이어지는데 호스트 공백만 크면 "늦게 몰아서 옴").

## 코드 검사 (맥, 장치 불필요)

```bash
cd /Users/seojeongmin/krri/forklift-truck_dockingv2_nocalib && PYTHONPATH=.:tests python3 -m unittest discover -s tests -q
```
