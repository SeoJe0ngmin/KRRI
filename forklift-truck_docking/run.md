# v6 실행 요약

## 1. 맥 → Jetson 코드 복사

현재 USB 연결 SSH 별칭을 사용한다. 현장 common과 측정 로그는 기본적으로 보존한다.

```bash
bash /Users/seojeongmin/krri/forklift-truck_docking/update_jetson.sh --host jetson-wired
```

맥에서 수정한 **common까지 덮어쓸 때만** 끝에 `--with-common`을 붙인다. `forklift-truck_docking` 전체(코드 + `configs/`)를 복사한다. 실행 중에는 업데이트하지 않는다.

이 폴더는 다른 `krri` 폴더를 읽지 않는다(2026-10-19 확인). 실행에 필요한 것은 이 폴더 + 설치 스크립트가 만드는 환경(`~/.venvs/krri`, `~/.local/share/krri/`, `/opt/ros/jazzy`)뿐이다.
[algorithm.md](algorithm.md)와 도식도 같이 복사된다.

## 2. Jetson 최초 설치 — 나중에 한 번

Ubuntu 24.04 / ROS 2 Jazzy 구성이다. 다운로드할 때 인터넷이 필요하며 설치 후 USB 접속·실행에는 인터넷이 필요하지 않다.

```bash
cd /home/jetson/krri/forklift-truck_docking
bash scripts/install_jetson.sh
```

RTAB-Map 지도도 사용할 때는 `bash scripts/install_jetson.sh --mapped`.

Jetson 저장소의 OpenCV 4.8과 Ubuntu 4.6 패키지가 섞이지 않도록 설치 스크립트가 OpenCV 개발 패키지·contrib·Python 패키지를 Ubuntu 4.6의 같은 버전으로 지정한다. 전역 패키지 우선순위는 변경하지 않으며, 기존 패키지를 삭제해야 하는 설치는 중단한다. 이전 스크립트에서 `libopencv-contrib-dev` 의존성 오류가 났다면 맥에서 코드를 다시 복사하고 위 설치 명령을 다시 실행한다.

기존 RealSense SDK는 Python 버전이 맞으면 그대로 연결한다. `pyrealsense2` 누락/ABI 오류만 남으면 다음을 실행한다. 커널·기존 SDK를 교체하지 않는 별도 RSUSB 빌드다.

```bash
bash scripts/install_realsense.sh
bash scripts/preflight.sh check --sensor-only
```

ROS 기본 패키지와 OpenCV 등 apt 의존성은 시스템에, OpenVINS와 브리지는 `~/.local/share/krri/ros_ws`(2026-10-19 에 `krri-v6` 에서 옮겨 재빌드; 옛 자리는 새것이 확인될 때까지 그대로 인식됨), Python 은 **프로젝트 가상환경 하나 `~/.venvs/krri`** 를 쓴다(2026-10-19 사용자 결정: 폴더 하나·가상환경 하나). `(krri)`를 먼저 활성화할 필요 없이 실행 스크립트가 그 환경을 고른다. 빠진 패키지는 아래로 넣는다. Kvaser 시스템 드라이버는 기존 설치를 사용한다.

```bash
~/.venvs/krri/bin/python -m pip install 'pupil-apriltags==1.0.4.post11' 'canlib==1.33.358' scipy pyyaml
~/.venvs/krri/bin/python -c "import numpy,cv2,scipy,yaml,pupil_apriltags,pyrealsense2;print(cv2.__version__)"
```

**이미 설치를 마친 Jetson에서 브리지/메시지 변경을 반영할 때:** 코드 복사 후 아래 명령을 한 번 실행한다. apt·다운로드 없이 메시지·브리지 두 패키지를 다시 생성한다. 기존 실행은 먼저 종료한다. 이전 두 패키지의 build/install 산출물은 `~/.local/share/krri/ros_ws/rebuild_backups/`에 보관하고 OpenVINS 원본·세 라이브러리 빌드·측정 로그·common은 보존한다.

```bash
cd /home/jetson/krri/forklift-truck_docking
bash scripts/rebuild_openvins.sh
```

`Clock skew detected` 또는 날짜가 1970년이면 **맥 터미널에서** 먼저 현재 시간을 맞춘다. USB 연결만으로 가능하다.

```bash
ssh -t jetson-wired "sudo date -u -s '$(date -u '+%Y-%m-%d %H:%M:%S')'"
```

그 뒤 **Jetson 터미널에서** 다음처럼 `&&`로 연결하면 빌드 성공 시에만 측정한다.

```bash
bash scripts/rebuild_openvins.sh && bash scripts/run_nano.sh measure --seconds 90 --view
```

스크립트도 빌드 미완료 표시가 남으면 VIO 실행을 차단한다. 표시 파일을 수동으로 지우지 말고 재빌드를 완료한다. 시스템 날짜는 파일 기반 빌드에 영향을 주지만, IMU 시간 검사에는 장치 타임스탬프를 사용하므로 날짜 수정만으로 IMU 누락이 해결됐다고 판단하지 않는다.

설치 스크립트는 OpenVINS 원본을 보존하고 별도 빌드 사본의 CMake 설정만 C++17/Jazzy용으로 생성한다. 추정 알고리즘은 원본과 같으며 사용하지 않는 예제·시각화 실행 파일은 빌드하지 않는다.

## 3. 카메라와 추정 확인 — CAN 미사용

아래 실행 스크립트가 Python/ROS 환경을 로딩한다. 별도 `source activate`는 필요 없다.

```bash
cd /home/jetson/krri/forklift-truck_docking
bash scripts/preflight.sh check --sensor-only
bash scripts/run_nano.sh measure --seconds 90 --view
```

화면은 **IR 영상+특징점**, 오른쪽은 컬러 태그 상태·현재 위치·포크 오차다. 컬러 태그 사각형을 IR 영상 위에 잘못 그리지 않는다. VS Code에서 8765 포트를 전달하고 맥의 `http://127.0.0.1:8765`를 연다.

### 손으로 카메라를 들고 하는 선행 실험 (CAN 없음, 2026-10-19)

같은 `measure --view` 로 한다. 새 코드는 필요 없다. 태그를 벽에 붙이고 카메라를 손에 들고 천천히 움직이면 화면에서 다음을 본다.

- 둘째 줄: 포크 목표까지 남은 거리 · 중심선 좌우 · 방향 · 예상 도착 좌우 (입구 기준 융합 결과).
- 셋째 줄: OpenVINS 자기 기준 위치(x, y, yaw) · 앵커 뒤 이동 거리 · 태그 본 지 몇 초 · 방향 offset · 융합 상태(`refreshed` / `tag_jump_skipped` / `accepted_after_skip`)와 잔차·허용 점프 · σ · 정지 판정(자이로 °/s, VIO m/s).
- 왼쪽 평면도: 하늘색 궤적이 최근 추정 위치, 흰 점이 태그 창으로 앵커를 갱신한 순간, 자홍 고리가 충돌 뒤 재앵커, 초록 고리가 첫 앵커. 태그를 손으로 가리면 궤적이 VIO 만으로 이어지고, 다시 보이면 흰 점이 찍히며 보정되는 것이 보인다.
- 오른쪽 작은 창: OpenVINS 가 자기 원점 기준으로 그린 궤적. 추적이 끊겨 원점이 바뀌면 지워진다.

실험 전에 `configs/common.json` 세 값을 실제에 맞춘다. `tag_size_m` 은 **붙인 태그의 검은 테두리 바깥 변**(A4 에 뽑은 것은 0.2, 현장 것은 0.3), `tag_height_m` 은 태그 중심 높이, `camera_height_m` 은 손 높이. 장착 보정은 measure 에선 필요 없고, 손에 든 카메라는 광축이 곧 "차 앞 방향"이다. 처음 2초는 카메라를 가만히 두어야 OpenVINS 가 초기화된다. 로그는 `work/<시각>_measure/observations.jsonl` 에 그대로 남는다.

OpenVINS 설치 전에 영상과 태그만 확인할 때:

```bash
bash scripts/run_nano.sh measure --camera-only --seconds 60 --view
```

`--camera-only`는 위치 추정·주행 준비 완료 판정을 하지 않는다. 이때 화면은 컬러 영상에 태그 사각형(초록=채택, 주황=탈락)이고, 보통 때는 IR 흑백에 OpenVINS 점이다. OpenVINS 초기화가 끝나도 태그가 유효하게 관측되기 전에는 입구 기준 pose가 없다.

`odom` 확인 항목:
- `tracking_ok`: 현재 영상에서 앞 프레임과 같은 특징점을 계속 추적하는지.
- `corrected_this_frame` / `correction_recent` / `correction_age_s`: 보정 여부와 마지막 보정 이후 초. `correction_recent=false`만으로 추적 실패는 아니다.
- `ok`: 추적·시각·지연·IMU·상대 이동 연속성·최대 보정 공백 검사를 통과했는지. 실패 이유는 `reason`에 나온다.
- `usable_for_docking`: 입구 기준 위치와 불확실성까지 통과했는지. `ok=true`와 별개다.

카메라 첫 영상 대기는 최대 30초, OpenVINS 무응답 제한은 첫 영상 전달 이후 10초다. 준비 중 영상은 제어 관측으로 사용하지 않는다. 오류가 나면 실행 폴더의 `camera_error.txt`, `pipeline_status.json`, `openvins.log`를 확인한다. 마지막 파일의 `[pipeline]` 줄은 입력 수신·대기열·처리 단계·출력 수를 보여 준다. 입력만 계속 와도 무응답 제한을 갱신하지 않는다.

불완전한 첫 프레임셋은 위 제한 안에서 기다리며 완전한 새 IR 좌·우/컬러가 와야 영상 준비로 처리한다. IMU 표본 사이 공백·역전은 **세고 기록만 한다**(2026-10-19, apriltag_v4 규칙: 공백 구간은 적분하지 않고 `gaps` 를 올림, 3초 넘게 IMU 가 안 올 때만 정지 — `imu_age_s`, 2026-10-20 1→3 s). `pipeline_status.json` 의 `imu.gaps/largest_gap_s/last_gap` 과 `openvins.log` 의 `imu_gaps` 에 남는다. 누락 구간을 임의로 채우지는 않는다. 같은 파일에 `gc`(가비지 컬렉션 멈춤 시간)·`loop_stage_max_ms`(메인 루프 단계별 최대 소요)·`delivery_gaps`(카메라 프레임 공백)가 1초마다 적히고, `observations.jsonl` 에는 `kind: health` 행으로 남는다.

OpenVINS에 첫 데이터를 전달하기 전에는 IMU 수신을 0.5초 준비한다. 장치 시간과 호스트 수신 시간이 모두 연속 진행하고 좌·우 IR 영상 시각과 맞아야 시작한다. 준비 중 시간 점프가 있으면 양쪽 IMU 버퍼를 비우고 새 구간을 수집한다(`startup_resets`, `last_startup_reset` 기록). 오래된 샘플이 한꺼번에 도착하는 것만으로 준비 완료가 되지 않는다. 영상은 이때도 표시하며 `odom.reason=imu_startup_stabilizing`이다. 준비 제한은 기존 30초이며, 전달 시작 후 끊김은 재초기화로 숨기지 않고 중단한다. 이 준비는 정차 판정이나 IMU 바이어스 보정 완료와 별개다.

## 4. 현장 입력·재장착 보정

`/home/jetson/krri/forklift-truck_docking/configs/common.json`의 `value`를 수정한다.

- 실측 차폭, 탑재부 내부 폭, 좌측 태그 거리(좌측 모드), 태그 크기·높이, 카메라 높이, 카메라→포크 끝 거리.
- 기본 목표: 중앙은 **포크 끝이 입구 0.5m 앞**, 좌측은 **포크 끝이 입구를 지나 1m**.
- 장착각 보정은 아래 6절의 `mount`(태그 정면에 세우고 가만히 찍은 measure 로그로 계산, v4 camyaw 방식)로 한다. CAN 전진 방식과 수동 직진 방식은 2026-10-19 에 지웠다.

```bash
bash scripts/preflight.sh check --mode central
# 좌측 모드는 입력 후:
bash scripts/preflight.sh check --mode left
```

장착각은 컬러에서 측정해 공통 IR 기준으로 변환하여 common에 저장한다. 재장착했으면 다시 측정한다. 공통값·캘리브레이션 실패를 임의 숫자로 채우지 않는다.

## 5. 주행 흐름 검사와 실주행

```bash
# 상태 전환 확인, CAN 안 엶. 차량은 실제로 움직이지 않는다.
bash scripts/run_nano.sh shadow --mode central --view

# 실제 CAN 자동 도킹
bash scripts/run_nano.sh drive --mode central --execute --view
# 또는 좌측 태그
bash scripts/run_nano.sh drive --mode left --execute --view
```

시작 후 OpenVINS 초기화 + 자이로 정지 5초(준비 전체 최대 180초) → `READY` → SSH 터미널 **Enter**. 그 뒤 태그가 안 보이면 50°씩 돌며 찾고, 태그가 보이는 자리에서 **30초 정차**(σ 기준선) 후 첫 계획을 세운다. 장착 yaw 는 주행 중 좋은 직진(태그 5 m 안·β 10° 안·0.5 m 이상)마다 다시 배운다. 좋은 직진이 없었어도 들어가되 로그에 남긴다. `s`+Enter 또는 Ctrl+C로 정지한다. shadow는 명령을 표시하되 차량을 움직이지 않으므로 움직임 피드백이 없으면 동작 시간 제한으로 끝난다.

태그가 없으면 50°씩 최대 360° 탐색하고 찾으면 즉시 멈춘다. 이후 최대 6단계 후보를 계획해 첫 구간을 실행하고 정지·되튐을 관측하여 다시 계획한다. 각 구간의 회전 뒤 전진도 최신 관측으로 방향·남은 거리·가시성·통로를 다시 확인한다. 마지막 진입은 예상 도착 좌우 ±3cm로 판단하고 σ는 기록만 한다(2026-10-19). 방향은 이전 구간에서 맞추며 마지막에 별도 각도 허용 게이트를 중복 적용하지 않는다.

## 6. 선택 기능

```bash
# RTAB-Map 지도 기록 + 재방문 후보 진단. 기본 core보다 부하가 증가한다.
bash scripts/run_nano.sh measure --profile mapped --seconds 90 --view
```

`mapped`는 지도 DB를 이번 로그 폴더에 저장한다. 지도 보정은 `used_for_control: false`로 표시한다. 같은 VIO를 사용한 지도 후보를 검증 없이 위치에 더하지 않는다. ORB-SLAM3 자동 전환은 지원하지 않는다.

코드 시험(센서/CAN 불필요): `DOCK_PYTHON=...`로 설치된 Python을 지정하거나 전용 환경에서 `python -m unittest discover -s tests -q`.

정지 준비 중에는 브레이크를 잡고 실제 정차를 유지한다. VIO 초기화 후 자이로(0.5 초 창 p90 ≤ 0.5°/s)와 VIO 속도로 정지를 보고, 30초가 쌓이면 READY가 나온 뒤 Enter로 출발한다. 그 30초의 태그 흩어짐이 σ 기준선이 되며 합격·불합격은 없다(태그가 안 보이면 기본값을 쓰고 `startup_measurement.json`의 `sigma_baseline.source`에 적힌다). 회전·직진 후에는 같은 자이로·VIO 속도 판정으로 멈춤을 확인한다(VIO가 없으면 중립 뒤 2초). 이유는 로그의 `stationary_report`에서 확인한다. 엔진 진동 허용과 ±3cm 도킹 정확도 검사는 별개다. OpenVINS 자체 초기화 실패는 이 판정으로 우회하지 않는다.

2026-10-16 공통 치수: 차폭 1.3m, 탑재부 내부 폭 1.4m(임시). 중앙에서 좌우 물리적 여유는 각각 5cm이며 정렬 허용 오차는 3cm를 유지한다. 이미 젠슨에 공통 설정이 있으면 일반 업데이트는 보존하므로, 해당 두 `value`를 젠슨에서도 수정하거나 의도적으로 `--with-common`을 사용한다(후자는 다른 공통값도 맥 값으로 덮어쓴다).

### 장착각 (v4 camyaw 방식: 가만히 서서, CAN 없음)

```bash
cd /home/jetson/krri/forklift-truck_docking
bash scripts/run_nano.sh measure --view          # 차를 태그 정면에 눈대중으로 세우고 30초 이상 가만히 → Ctrl+C
bash scripts/preflight.sh mount --folder work/<방금 폴더>            # 계산만: yaw·pitch·roll·σ·경고
bash scripts/preflight.sh mount --folder work/<방금 폴더> --write    # common.json 에 저장
```

2026-10-19 사용자 결정: 차를 움직이지 않고 잰다(v4 `calibrate camyaw` 와 같음). 자이로가 "정지"라고 한 동안의 PnP 태그 회전을 0.5초 블록마다 모아 그 중앙값에서 **yaw 만** 장착각으로 쓴다(v4 `CAM_YAW_OFFSET_DEG`). pitch·roll 은 같은 로그의 IMU 중력 방향(OpenVINS 자세)으로 잰다(태그 기울기와 구분하려고; 수평 바닥 전제). 태그로 본 기울기와 5° 넘게 다르면 "태그가 기울어 붙음" 경고. **차체가 태그 법선과 평행하다는 것은 눈대중 가정**이며 검사하지 않고 경고로만 적는다(v4 주석: "절대 정확도는 모른다"). 눈대중이 틀린 만큼 yaw 가 틀리고, 그건 마지막 구간 조준에서 흡수해야 한다.
**통과 조건은 없다.** 정지 중 고유 태그 3장이 없을 때만 계산 불가. 30초 미만, 블록 흩어짐 1° 초과, 앞·뒤 절반 차이 0.5° 초과는 `warnings` 에 적고 저장한다. 결과는 컬러에서 재서 raw IR 기준으로 바꿔 `common.json` 의 `mount` 에 들어가고, 태그·카메라 치수 서명이 바뀌면 `drive` 가 다시 재라고 거부한다.
카메라 재장착 후 다시 보정해야 한다. (직진 방향으로 재는 방식과 CAN 5초 전진 방식은 같은 날 지웠다.)
