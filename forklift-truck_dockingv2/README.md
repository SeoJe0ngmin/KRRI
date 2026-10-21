# forklift-truck_docking

AprilTag + OpenVINS(IR 스테레오·IMU) 로 지게차를 탑재부에 자동 도킹하는 코드. Jetson Orin Nano 에서 돈다.
상세: [화면·로그 읽는 법과 주행 흐름](run.md) · [알고리즘](algorithm.md) · [검사 기록](VALIDATION.md)

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

명령은 하나. 연결에 따라 `--host` 만 바꾼다.

```bash
bash /Users/seojeongmin/krri/forklift-truck_docking/update_jetson.sh --host jetson-wired            # USB 직결
bash /Users/seojeongmin/krri/forklift-truck_docking/update_jetson.sh --host jetson-kw               # 광운대 핫스팟
bash /Users/seojeongmin/krri/forklift-truck_docking/update_jetson.sh --host jetson-wired --dry-run  # 바뀔 파일·common.json 차이만 보기
```

- 바뀐 파일만 보낸다. `configs/common.json` 은 **맥 것으로 덮는다**(주인은 맥). 덮기 전에 Jetson 것과 다른 줄을 보여 준다.
- `work/`(로그)와 `configs/mount.json`(Jetson 에서 잰 장착각)은 보내지도 지우지도 않는다.
- 파이썬·설정만 바뀌었으면 이걸로 끝. C++ 브리지가 바뀐 날만 아래 "브리지 재빌드".
- Jetson 시계가 1970 년이면(배터리 없음, 인터넷 없이 켰을 때) 배포 중에 맥 시간으로 맞춘다(sudo 비밀번호 한 번). 주행엔 영향 없고 로그 폴더 이름과 빌드에만 관계.

## Jetson 설치 (한 번)

```bash
ssh jetson-wired
cd /home/jetson/krri/forklift-truck_dockingv2
bash scripts/install_jetson.sh            # ROS Jazzy·OpenCV 확인, 가상환경 ~/.venvs/krri, OpenVINS+브리지 빌드 (30~60분)
bash scripts/install_jetson.sh --no-apt   # 인터넷 없이 빌드만 (apt·pip·rosdep 건너뜀)
```

만들어지는 것: 가상환경 `~/.venvs/krri`, 빌드 `~/.local/share/krri/ros_ws`. 코드는 이 폴더 하나.

## 브리지 재빌드 (C++ 가 바뀌었을 때만)

```bash
cd /home/jetson/krri/forklift-truck_dockingv2
bash scripts/rebuild_openvins.sh          # 우리 패키지 둘만 다시 컴파일, 몇 분. 파이썬만 바뀐 배포는 불필요
```

안 하고 돌리면 "OpenVINS 브리지 업데이트 필요" 로 스스로 멈춘다.

## 실행 — 현장 5단계 (Jetson, VS Code Remote-SSH 터미널)

화면은 VS Code 에서 8765 포트 전달 → 맥 브라우저 `http://127.0.0.1:8765`. 모든 명령은 `cd /home/jetson/krri/forklift-truck_dockingv2v2` 에서. 끝낼 땐 Ctrl+C 한 번.

### 0. 맥 — 현장 값 넣고 배포

`configs/common.json` 의 `site` 를 줄자 값으로(탑재부는 **`truck_inner_width_m`** 안쪽 폭과 **`truck_depth_m`** 깊이(입구면→안쪽 끝, 비우면 검사 없음); 카메라를 옮겼으면 **렌즈 위치 셋**만 다시: `camera_to_fork_tip_m` = 렌즈→포크 앞끝 앞뒤 거리, `camera_lateral_m` = 차 중심선(차폭 정가운데)→렌즈, 왼쪽 + 오른쪽 −, `camera_height_m` = 바닥→렌즈. 렌즈는 좌측 IR(컬러 바로 오른쪽). `rear_extent_m` 은 비워 두면 전장 `vehicle_length_m` 으로 계산되고, 회전중심은 차에 붙은 값이라 그대로. 옮긴 뒤엔 장착각 재측정):
   - 눈 감고 들어간 뒤 한쪽 벽에 붙으면 `forward_path_bias_deg` 에 흐른 각을 넣는다(왼쪽 +; 오른쪽 9 cm/3.1 m = −1.7°). 비우면 보정 없음. 태그 크기(0.3)·태그 중심 높이·카메라 높이·차폭·탑재부 내부 폭·뒤 공간·**좌측 태그 거리 `left_tag_offset_m`**(없으면 left 모드 출발 거부)·카메라→포크 끝. 그 뒤 위 "맥 → Jetson 코드 보내기" 로 배포(common.json 은 맥 것이 Jetson 을 덮는다).

### 1. 점검

```bash
bash scripts/preflight.sh check --sensor-only        # 설치·장치, CAN 안 엶
bash scripts/run_nano.sh measure --view              # 1분. 화면 = IR 흑백 + OpenVINS 점, 태그가 accepted 로 유지되는지
#   OpenVINS 점이 포크·마스트 등 차체에 찍히면 안 된다(카메라와 같이 움직여 VIO 가 '정지' 로 봄). 바닥·벽에 찍혀야 한다
grep "\[pipeline\]" $(ls -td work/*/*_measure | head -1)/openvins.log | tail -1   # dropped=0 이면 VIO 정상
```
OpenVINS 없이 카메라만 볼 때: `bash scripts/run_nano.sh measure --camera-only --view` (컬러 태그 검출·depth 입구·버드아이뷰는 그대로 동작). 패널 고르기: `--panels color` / `--panels ir,depth` (기본 all; 끈 패널은 인코딩·전송도 안 해 CPU·전송량이 줄어듦).
화면(v2)은 카메라 패널 셋 — IR + OpenVINS 점 / 컬러 + 태그 사각형 + **지시선**(초록 = 차폭, 하늘 = 탑재부 안쪽 폭, 1~5 m 눈금, 노랑 = depth 로 찾은 입구 양끝, 빨강 = 입구 중심) / depth 흑백 + 입구 선 — 과 평면도(추정), **입구 고정 버드아이뷰**(depth 측정). depth 로 입구를 찾았는지는 세 번째 패널 제목과 `센서 상세` 의 `depth 입구` 줄로 본다(폭 `truck_inner_width_m` ± `truck_width_tol_m` 인 쌍만 입구로 인정).
버드아이뷰·지시선만 따로 보는 독립 스크립트(도킹 앱을 끈 상태에서): `source scripts/environment.sh && "$DOCK_PYTHON" scripts/bev_test.py` → 터미널의 `http://127.0.0.1:8082` 링크. `--width` 로 찾을 폭, `--emitter` 로 프로젝터, `--save DIR` 로 PNG.

### 2. 장착각 (카메라를 달 때마다)

차를 태그 **정면**에 세운다. 좌우 치우침은 상관없고 **차 진행축이 탑재부 축과 평행**이면 된다(탑재부 벽에 차체 옆면을 평행하게 대는 게 눈대중보다 정확). 거리 **3.3~4 m**(더 가까우면 태그 윗변이 화면에서 잘림).

```bash
bash scripts/run_nano.sh measure --view                                      # 30초 넘게 가만히, Ctrl+C
bash scripts/preflight.sh mount --folder $(ls -td work/*/* | head -1) --write   # 방금 폴더로 장착각 계산 → configs/mount.json (--write 없으면 계산만)
bash scripts/preflight.sh check --mode left                                  # 현장 값·장착각 서명까지 검사. 여기서 거부되면 drive 도 거부
```

### 3. 리허설 (차 안 움직임)

```bash
bash scripts/run_nano.sh shadow --mode left --view   # READY 뒤 Enter → 탐색 → 30초 정차 → 계획까지 흐르는지
```

### 4. 실주행

사람은 브레이크 자리. 두 번째 터미널에 `sudo jetson_clocks`(클럭 고정, 선택, 재부팅까지).

```bash
bash scripts/run_nano.sh drive --mode left --execute --view   # READY 뒤 Enter → 태그 탐색 → 30초 정차 → 주행. 정지 s+Enter / Ctrl+C
```

주행 흐름(v2, 2026-10-22 — 자세한 것은 run.md 5절): **러프 접근**(방향 0 유지, 좌우가 한 걸음(0.17 m) 이상이면 사이드스텝, 태그컷 1 m 앞까지 한 번에 직진) → **방향 재설정**(서서 태그로) → **세밀 조정**(depth 로 입구 중심을 재서 좌우 3 cm 넘으면 돌고-가고-되돌기 묶음, 아니면 직진; 입구 약 0.9 m 앞까지) → **최종 확인**(좌우 3 cm·방향 1° 안이면 진입, 아니면 회전·후진 후 다시, 최대 5 회) → **진입**(VIO + `forward_path_bias_deg`). 기준값은 common.json `operation` 에서 바꾼다.
터미널 줄의 `상태/단계`(ROUGH · FINE_ENTER · FINE · FINAL · ENTRY · FALLBACK)와 `depth 3.25m +0.012`(입구까지·좌우)로 진행을 읽는다.

### 5. 끝난 뒤 — 줄자 실측 기록

좌우 = (오른쪽 포크↔벽 틈 − 왼쪽 포크↔벽 틈) ÷ 2, 왼쪽 +. 방향 = 한쪽 벽과 차체 틈을 앞·뒤 두 곳에서 재서 atan(차이 ÷ 간격), 왼쪽 +. 종방향 = 입구면에서 포크 끝까지 − 목표(left 1.0 m), 더 들어감 +.

```bash
bash scripts/run_nano.sh note --folder $(ls -td work/*/* | head -1) --lateral-cm 2.5 --heading-deg -0.4 --longitudinal-cm 0 --memo "1회차"
```

장착 yaw 는 `mount.json` 값으로 시작해 주행 중 좋은 직진마다 다시 배운다. 포크 정지 목표는 `common.json` 의 `target_front_x_central_m`(−0.5) / `target_front_x_left_m`(1.0), 입구면 기준 안쪽 양수.

## 옵션 전부 (한 줄씩)

**`bash scripts/run_nano.sh <명령> [옵션]`** — 명령: `measure`(센서만, CAN 안 엶) · `shadow`(상태기계까지, 명령 표시만) · `drive`(실주행) · `note`(줄자 실측 기록)

| 옵션 | 한 줄 설명 |
|---|---|
| `--mode left` | 태그 위치 모드. 우리는 항상 `left`(기본값 central 이라 매번 써야 함) |
| `--view` | 대시보드 켬 → 터미널의 `http://127.0.0.1:8765` 링크 클릭 |
| `--port 8765` | 대시보드 포트 |
| `--panels all` | 카메라 패널 고르기: `ir` `color` `depth` 쉼표 조합. 끈 패널은 만들지도 보내지도 않음 |
| `--execute` | drive 에서 CAN 을 진짜 보냄(없으면 drive 거부). 대화형 터미널 필요 |
| `--seconds 60` | N 초 뒤 자동 종료(measure 에 유용) |
| `--camera-only` | measure 전용: OpenVINS 없이 카메라·태그·depth 만 |
| `--no-tag` | measure 전용: 태그 검출 끔 |
| `--output 경로` | 기록 폴더 직접 지정(기본 `work/<날짜>/<시각>_<id>_<명령>/`) |
| `--common 경로` | 다른 common.json 사용(기본 `configs/common.json`) |
| `--profile core` | `mapped` = RTAB-Map 지도 기록. 안 씀 |
| `--folder 폴더` | note: 기록할 판 폴더 |
| `--lateral-cm 2.5` | note: 줄자로 잰 포크 끝 좌우 [cm], 왼쪽 + |
| `--heading-deg -0.4` | note: 잰 방향 [°], 왼쪽 + |
| `--longitudinal-cm 0` | note: 목표 대비 종방향 [cm], 더 들어감 + |
| `--memo "1회차"` | note: 메모 |

**`bash scripts/preflight.sh <명령> [옵션]`** — 명령: `check`(설치·장치·현장값·장착각 검사) · `mount`(장착각 계산)

| 옵션 | 한 줄 설명 |
|---|---|
| `--mode left` | check: 그 모드 조건으로 검사(left 는 `left_tag_offset_m` 필수) |
| `--sensor-only` | check: 설치·장치만 보고 현장값·장착각은 안 봄 |
| `--camera-only` | check: OpenVINS 빌드 없이 카메라만 검사 |
| `--folder 폴더` | mount: 태그 정면에서 30 초 이상 가만히 찍은 measure 폴더 |
| `--write` | mount: 결과를 `configs/mount.json` 에 저장(없으면 계산만 출력) |
| `--common 경로` / `--profile` | run_nano 와 같음 |

**`bash update_jetson.sh [옵션]`** (맥에서)

| 옵션 | 한 줄 설명 |
|---|---|
| `--host jetson-wired` | SSH 호스트(`jetson-wired` USB / `jetson-kw` 핫스팟). 기본 jetson-wired |
| `--dry-run` | 보낼 파일만 보여 주고 전송 안 함 |

**`"$DOCK_PYTHON" scripts/bev_test.py [옵션]`** (Jetson, 도킹 앱 끈 상태, `source scripts/environment.sh` 먼저)

| 옵션 | 한 줄 설명 |
|---|---|
| `--port 8082` | 화면 포트 → `http://127.0.0.1:8082` |
| `--width 1.4` | 찾을 입구 안쪽 폭 [m](기본 common.json 값). 책상 실험은 의자 틈 폭으로 |
| `--width-tol 0.2` | 폭 허용 오차 [m] |
| `--emitter` | 프로젝터 켬(VIO 안 도는 테스트라 가능; 매끈한 면·어두움에 유리) |
| `--no-filter` | RealSense 공간·시간 필터 끔(원시 depth) |
| `--decay 0.7` | 버드아이뷰 누적(0 = 프레임마다 새로) |
| `--save 폴더` / `--every 2` | N 초마다 PNG 저장 |
| `--pitch 2` | mount.json 이 없을 때 카메라 숙임각 [°] |
| `--step 2` / `--fps 15` | 점구름 축소 배수 / 카메라 fps |
| `--mode left` | common.json 로드 모드 |

실행 중 키: READY 뒤 `Enter` 출발, `s`+Enter 또는 Ctrl+C 정지.

## 로그

```bash
ls -td /home/jetson/krri/forklift-truck_dockingv2/work/*/* | head -1                  # 가장 최근 판
scp -r jetson-wired:/home/jetson/krri/forklift-truck_dockingv2/work/<날짜> /Users/seojeongmin/krri/forklift-truck_docking/work/   # 그날 전부 맥으로 (판 하나면 <날짜>/<판>)
```

판 폴더는 `work/<날짜>/<시각>_<id>_<명령>/`(2026-10-20 부터 하루 한 폴더). 안에 `observations.jsonl`(관측 전부), `control.jsonl`(명령·CAN), `pipeline_status.json`(1초마다 센서·GC·프레임 공백), `openvins.log`(브리지; 2초마다 `[pipeline]` 줄).

도착 뒤 줄자 실측은 위 5단계의 `note` 명령으로 `manual_measurement.json` 에 넣는다.

판이 끝나면 브리지 줄 하나로 VIO 가 멀쩡했는지 본다(2026-10-20 기준 정상: `dropped=0`, `imu_lost` 수십 이하, 도착 간격 0.1 s 아래):

```bash
F=$(ls -td /home/jetson/krri/forklift-truck_dockingv2/work/*/*_measure | head -1)   # drive 면 *_drive
grep "\[pipeline\]" $F/openvins.log | tail -1      # dropped(버린 영상) · imu_lost/imu_sequence_jumps(빠진 IMU) · imu/image_arrival_gap_max_s(도착 간격 최대)
```

## CPU 보기 (Jetson 두 번째 터미널)

```bash
tegrastats --interval 1000                                    # 코어별 %, RAM, 온도
top -d 1 -p $(pgrep -d, -f 'python.* -m docking|multiprocessing.spawn|docking_openvins/bridge')   # 우리 프로세스 셋: 감독(python -m docking)·카메라 워커(multiprocessing.spawn)·브리지. %는 코어 수 × 100 이 상한
sudo jetson_clocks                                            # 실주행 때 클럭 고정 (재부팅까지)
```

## IMU 전달 점검 (Jetson, 앱과 무관하게 SDK 만 — 2026-10-20 VIO 멈춤 추적에 쓴 도구, 원인은 브리지 수신 QoS 였고 해결됨)

```bash
source ~/.venvs/krri/bin/activate
python scripts/check_imu_cadence.py imu    --seconds 60   # 자이로+가속도만
python scripts/check_imu_cadence.py ir     --seconds 60   # + IR 좌우 640x480@30
python scripts/check_imu_cadence.py full   --seconds 60   # + 컬러 1280x720@30 (주행 구성, depth 끔)
```

스트림마다 호스트 도착 간격의 최대·100 ms/500 ms 초과 횟수, 가장 큰 공백에서의 장치 시각 간격, 자이로가 가속도 짝을 기다린 시간(pairing wait)을 찍는다. 카메라·USB 쪽 의심이 생길 때만 쓴다.

## 코드 검사 (맥, 장치 불필요)

```bash
cd /Users/seojeongmin/krri/forklift-truck_docking && PYTHONPATH=.:tests python3 -m unittest discover -s tests -q
```
