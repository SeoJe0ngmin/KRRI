# AprilTag 지게차 도킹

카메라로 AprilTag 을 보고 **탑재부 기준 지게차 위치**를 낸 뒤, 그 값으로 주행한다.

처음 받았다면 — 맥북 Ubuntu VM / Jetson 은 [CLAUDE.md](CLAUDE.md) 부터.
주요 문서:
[CLAUDE.md](CLAUDE.md) VM·Jetson 설치 (Windows 는 git clone + `pip install -r requirements.txt`) ·
아래 **값 사전** 절 (잴 것/정해진 것/카메라가 주는 것) ·
아래 **제어 코드 설계 노트** 절 (주석은 코드가 아니라 여기 있다)

```bash
pip install -r requirements.txt
realsense-viewer                      # 가끔 카메라 점검할 때 (SDK 기본 제공)
python tools/check/live_pose.py             # 화면으로 확인
python tools/check/device_check.py             # IMU 부호·드리프트 확인
python tools/run.py --dry-run         # 도킹 순서만 (CAN 안 씀)
python tools/run.py                   # 실제 주행
```

---

## 실사용 흐름

```
image.py  ->  detection_tag.py  ->  detection_pose.py  ->  control_from_pose.py
 프레임         태그 찾기            자세 + 도킹값          주행 명령
```

### detection 이 내놓는 것

`measure()` 가 30프레임을 모아 중앙값 하나를 낸다. 한 프레임은 못 쓴다 —
`lateral` 이 ±9mm 흔들리기 때문이다. 30프레임이면 ±0.7mm.

```python
{'lateral':  0.104,   # 태그 축에서 좌우로 벗어난 거리 [m]. +가 오른쪽
 'forward':  1.407,   # 태그면까지 남은 거리 [m]
 'heading_deg': 2.04, # 지게차가 태그 축과 몇 도 틀어졌나
 'tilt_deg': 23.1,    # 태그가 화면에서 찌그러진 정도. 각도 신뢰도 판단용
 'distance': 1.432, 'vertical': -0.248, 'z_optical': 1.388,
 'n': 30,                        # 실제로 쓴 프레임 수
 'spread': {'lateral': 0.0007, ...},   # 대표값의 표준오차
 'reliable_angle': True,         # tilt >= 10도. False 면 heading 을 믿지 마라
 'stable': True, 'reasons': []}  # False 면 명령 내지 말고 다시 재라
```

### control 이 받는 것

위 딕셔너리 하나를 그대로 받는다. 쓰는 값은 **세 개**다.

```python
plan_step(m) -> (동작, 양, 명령시간[s], 이유)
```

| 쓰는 값 | 무엇에 |
|---|---|
| `lateral` | 옆으로 얼마나 벗어났나 → 옆이동 |
| `forward` | 얼마나 더 가야 하나 → 전진 |
| `heading_deg` | 얼마나 틀어졌나 → 회전 각도 |
| `reliable_angle`, `stable` | 이 값을 써도 되나 |

동작은 CAN 명령 다섯 개로만 나온다 — 정지 / 전진 / 후진 / 제자리좌회전 / 제자리우회전.
명령에 양이 없어서 "얼마나"는 **시간**으로 준다.

```
거리 -> 시간   fwd_time_model.py       실측 적합 (다른 팀). 직진은 개루프
각도 -> 각도   CanDriver.rotate_by()   IMU 를 보며 목표각에서 멈춤. **폐루프**
```

회전은 시간으로 안 바꾼다 — 각속도를 몰라도 IMU 가 목표각에서 끊는다.
그래서 `ROT_DEG_PER_SEC` 같은 미측정 값이 결과를 안 바꾼다.

---

## 파일

### `src/`

| 파일 | 하는 일 |
|---|---|
| `config.py` | 직접 정한 숫자 전부. 현장값 / 판정 임계 / 측정·규격 |
| `models/detection/image.py` | 카메라·bag·영상 → 프레임 + 내부파라미터 |
| `models/detection/detection_tag.py` | 흑백 이미지 → 태그 검출 |
| `models/detection/detection_pose.py` | 검출 → 4x4 자세 → 도킹값. `measure()` 가 여기 |
| (아래 **검출 상세** 절) | 실측·근거 모음 (블러, 좌표계, SDK 소스 등) |
| `models/control/control_from_pose.py` | 도킹값 → 동작 하나. 순서·기하·회전 폐루프 |
| `models/control/control_forklift_v2.py` | CAN 프레임 전송 (다른 팀). 안 고침 |
| `models/control/fwd_time_model.py` | 거리 → 명령 시간 (다른 팀) |
| `utils/imu_yaw.py` | 자이로 적분 상대 yaw. 회전 폐루프의 눈 |
| `utils/camera.py` | 노출·AE ROI·프레임 드랍 등 카메라 설정 |
| `utils/tag_layout.py` | 태그 여러 개를 한 좌표계로. 아직 안 씀 |
| `utils/drawing.py` | 화면에 큐브·축 그리기. 표시 전용 |
| `utils/util.py` | 블로그 원본. 5개만 씀 |

| `utils/make_tag_pdf.py` | 인쇄용 태그 PDF (눈금자 포함) |

### `tools/`

현장에서 주로 쓰는 둘은 tools/ 바로 아래, 나머지는 성격별 폴더(v2 기준. v1 은 평면):

| 파일 | 하는 일 |
|---|---|
| `run.py` | **도킹 자동 실행.** 시작만 키보드, 그 뒤는 카메라가 정한다 |
| `analyze_run.py` | 운행 기록 분석 — 결과 판정, 회전·직진 파라미터 권장값 |
| `check/device_check.py` | 카메라 + IMU 장치 점검 (realsense + imu 합침). `--tag` 로 태그 교차검증 |
| `check/realsense_check.py` | 카메라만 깊게 (device_check 의 엔진) |
| `check/check_setup.py` | 설치 됐나 — 장치 없이 import·canlib |
| `check/live_pose.py` | 실시간 화면 + 숫자. `--log` JSON, `--record` .db3 |
| `check/verify.py` | 줄자로 잰 값과 대조 |
| `etc/can_pulse.py` | 회전 팔·직진 속도·정지 관성을 펄스로 실측 (`--camera`) |
| `etc/smoke_dock.py` | 무하드웨어 파이프라인 계약 검사 (코드 고친 뒤 회귀) |
| `etc/setup_ubuntu_arm64.sh` | 리눅스 arm64(VM·Jetson) 설치 — apt/conda/realsense/pip/can/check |

---

## 지금 막혀 있는 것

```
① IMU_YAW_SIGN 확인          python tools/check/device_check.py  — 반시계로 돌려 +가 나오나
② 회전 부호 확인              rotate_ccw 에 지게차가 왼쪽으로 도나 (눈으로)
③ 탑재부 허용 오차            LAT_TOL_M / HEAD_TOL_DEG 의 근거. 현장에서 받아야 함
④ 태그2 위치 실측             tag_layout 으로 좌표 환산해야 전환 때 값이 안 튄다
```

`ROT_DEG_PER_SEC` 은 더 이상 급하지 않다 — 회전이 폐루프라 결과를 안 바꾼다.


---

## 현장에서 재야 하는 값 / 이미 정해진 값

`config/control.py`(25개) + `config/detection.py`(33개) 를 성격으로 갈랐다.
**현장에서 손댈 것은 아래 A 의 9개뿐이고, 나머지 49개는 그대로 두면 된다.**

---

### A. 현장에서 재거나 받아야 하는 값 (9개)

#### A-1. 남한테 받아야 하는 것 — 탑재부 사양 (2개) ★가장 중요

| 값 | 지금 | 어떻게 정하나 |
|---|---|---|
| `LAT_TOL_M` | 0.030 | **탑재부가 허용하는 좌우 오차 [m]**. 지금 값은 우리 사정(명령 하한 17mm)만 보고 정한 것 |
| `HEAD_TOL_DEG` | 2.0 | 임시 느슨함. 정합값 0.7도(=asin(LAT_TOL/눈감는 2.43m))지만 회전 오버슈트 실측 전에 조이면 회전 탁구. **오버슈트 확인 후 0.7~1.0 으로** |

이 둘이 뿌리다. `detection.py` 의 `STABLE_LATERAL_M`/`STABLE_HEADING_DEG`/
`MAX_HEADING_SIGMA_DEG` 가 여기서 자동으로 파생되므로, **이 둘만 고치면
나머지는 따라온다.** 반대로 이 둘이 실제보다 빡세면 도킹이 영영 안 끝나고,
헐거우면 탑재부에 안 들어간다.

#### A-2. 자로 재는 것 (2개)

| 값 | 지금 | 어떻게 재나 |
|---|---|---|
| `TAG_SIZE_M` | 0.300 | 인쇄한 태그의 **검은 테두리 바깥 한 변**을 자로. 프린터가 30.0cm 로 정확히 안 뽑는다. 여기가 1% 틀리면 모든 거리가 1% 틀린다 |
| `DOCK_EXTRA_M` | 0.0 | 태그면에서 **실제 정지 목표까지 더 갈 거리**. 태그가 탑재부 끝에 안 붙어 있으면 그 차이를 양수로 |

#### A-3. 주행해 보고 로그에서 얻는 것 (5개)

주행 후 `python tools/analyze_run.py` 가 권장값을 숫자로 찍어준다.

| 값 | 지금 | 어디서 |
|---|---|---|
| `ROT_LEAD_DEG` | 0.0 | analyze_run 의 "오버슈트 평균" → 그 값을 넣는다 (관성만큼 미리 끊기) |
| `ROT_WATCHDOG_GAIN` | 5.0 | 지금은 넉넉하게 크게. 회전 시간이 안정되면 낮춰도 된다 |
| `ROT_SETTLE_RATE_K` / `ROT_SETTLE_RATE_FLOOR` / `ROT_SETTLE_RATE_CEIL` | 3.0 / 1.0 / 2.0 | 실차 진동에서 "멎었다" 판정이 너무 이르거나 늦으면 조정 |
| `CAM_YAW_OFFSET_DEG` | 0.0 | 도킹이 **매번 같은 방향으로** 삐뚤면 그 각도. 랜덤이면 이 문제가 아니다 |
| `CORNER_NOISE_PX` | 0.07 | 실장비 로그로 역산해 다시 넣을 것. **원거리에서 과소가 확인됨** — 5m 예측 0.38도 vs 합성영상 실측 1.49도 (4배). 1~2m 역산값이라서다. 실측 적합되면 STABLE_TAG_PX(50) 를 'sigma<=0.5×√30' 파생 규칙으로 교체 예정 |

> A-3 는 **지금 값이 전부 "아무것도 안 하는" 안전한 쪽**이다.
> `ROT_LEAD_DEG=0` 은 보정 안 함, `CAM_YAW_OFFSET_DEG=0` 은 카메라가 똑바르다고 봄.
> 즉 안 고치고 주행해도 위험하지 않고, 로그를 보고 나중에 채우면 된다.

---

### B. 이미 정해진 값 — 현장에서 안 건드린다 (49개)

#### B-1. 사실 (바꿀 수 없는 것)

```
COLOR_SIZE (1920,1080)   IR_SIZE          D435I_COLOR_REF     ASSUMED_HFOV_DEG(계산)
TAG_CELLS 8              MM_PER_INCH      COLOR_EXPOSURE_UNIT_US 100.0
TAG_ID 1                 TAG2_ID 2
```
장비 스펙·수학 상수·우리가 부여한 번호.

#### B-2. 이미 실측으로 확정한 것

```
BLUR_CLEAN_PX 10.0     여기까지 검출 100%
BLUR_DEAD_PX 32.0      여기서 0% (12px 96% / 20px 89% / 28px 50%)
MIN_TAG_PX 20.0        19.5px 에서 검출률 89.3%
MIN_DECISION_MARGIN 20.0   저조도 하한. 거리에는 둔감
DEFAULT_QUAD_BLUR 0.0  9가지 비교에서 안 넣는 게 최선
MAX_REPROJ_RMS_PX 2.0  코너 잡음 바닥의 약 6배
```

#### B-3. 계산·파생 (손으로 고치면 안 됨)

```
STABLE_LATERAL_M      = LAT_TOL_M / 3
STABLE_HEADING_DEG    = HEAD_TOL_DEG / 3
MAX_HEADING_SIGMA_DEG = HEAD_TOL_DEG / 4
MEASURE_MAX_FRAMES    = MEASURE_FRAMES * 5
ASSUMED_HFOV_DEG      = fx 에서 계산
```
A-1 을 고치면 자동으로 따라온다.

#### B-4. 설계 판단 — 틀려도 느려질 뿐

```
STEP_M 1.0            FWD_SAFETY 0.9
MAX_STEPS 30          HOLD_RETRY_SEC 0.2
SETTLE_SEC 0.15       ROT_POLL_SEC 0.01     SEARCH_BACKUP_M 0.5
ROT_SETTLE_MAX_SEC 1.0    ROT_SETTLE_MIN_SEC 0.2    ROT_SETTLE_POLL_SEC 0.05
SEARCH_AFTER_MISSES 3     SEARCH_MAX_ROUNDS 3       MEASURE_FRAMES 30
SIGMA_SAMPLES 60          STABLE_SPREAD_K 3.0       STABLE_TAG_PX 50.0
SIDESTEP_BACKWARD_GAIN_DEG 0.0                      TAG_CUT_MARGIN_PX 30.0
DEPTH_TOL_COEF 0.05       DEPTH_TOL_FLOOR_M 0.02    DEPTH_CHECK_MAX_Z 1.5
RELIABLE_TILT_DEG 10.0 (폴백 전용)
ROT_WRONG_WAY_DEG 5.0 (안전장치)
```
2배 틀려도 동작이 굼떠질 뿐 위험하지 않다. 급하지 않으면 그대로 둔다.

---

### 현장 순서 요약

```
가기 전     태그 인쇄 -> 자로 재서 TAG_SIZE_M 확정
현장 도착   탑재부 사양 확인 -> LAT_TOL_M / HEAD_TOL_DEG
            정지 목표까지 거리 재기 -> DOCK_EXTRA_M
주행        python tools/run.py --show --record-events
주행 후     python tools/analyze_run.py  -> ROT_LEAD_DEG 등 권장값 반영
```

---

### 카메라가 주는 것

`config.py` 에 **일부러 안 적었다.** 적으면 해상도를 바꿀 때 어긋난다.

| 이름 | 뜻 | 예시값 (1920×1080) |
|---|---|---|
| `fx`, `fy` | 초점거리 [px]. 거리 계산의 기준 | 1359.2 / 1359.0 |
| `cx`, `cy` | 광축이 화면에서 지나는 점 [px] | 956.9 / 571.3 |
| `distortion` | 렌즈 왜곡 계수 | 대부분 0 |
| `depth_scale` | depth 원값 → m 환산 | 0.001 |
| 실제 해상도 | 요청이 거부되면 다른 값이 온다 | — |
| 현재 노출·게인 | 우리가 건 값이 실제로 걸렸는지 확인용 | — |

`D435I_COLOR_REF` 만 예외로 적어뒀다 — **영상 파일을 볼 때처럼 카메라가 없는 경우의 대체값**이다.
다른 해상도는 세로 비율로 환산한다(실측 0.03px 이내).

---

### 코드가 내놓는 것

`measure()` 가 30프레임을 모아 중앙값 하나로 낸다.

| 이름 | 단위 | 뜻 |
|---|---|---|
| `lateral` | m | 태그 정면축에서 좌우로 벗어난 거리. **+ 면 지게차가 오른쪽** |
| `forward` | m | 태그면까지 남은 거리 |
| `heading_deg` | 도 | 지게차가 태그 정면축과 틀어진 각 |
| `vertical` | m | 카메라높이 − 태그높이 |
| `distance` | m | 직선거리 |
| `z_optical` | m | 광축 방향 거리 |
| `tilt_deg` | 도 | 태그가 얼마나 돌아가 있나 (pitch·yaw 합) |
| `heading_sigma_deg` | 도 | **`heading` 을 몇 도 오차로 아는가** |
| `tag_px` | px | 태그 한 변의 화면 크기 |
| `reproj_rms_px` | px | 구한 자세로 코너를 되찍은 오차 |
| `decision_margin` | — | 검출 확신도 (저조도에서 떨어짐) |
| `hamming` | — | 고친 비트 수. 0 이 아니면 의심 |
| `spread` | — | 대표값의 표준오차 |
| `n` | — | 실제로 쓴 프레임 수 |
| `reliable_angle` | T/F | 각도를 믿어도 되나 |
| `stable` / `reasons` | T/F | 명령을 내도 되나 / 아니면 왜 |

---

### 설정 파일 지도

설정은 성격별로 나눠 두었다. 고칠 때는 해당 파일을 연다.

```
config/detection.py   태그를 보고 위치를 내는 값   태그 크기·번호, 해상도, 품질 문턱, 장비 규격
config/control.py     어떻게 움직일지 정하는 값     도킹 허용치, 전진 방식, 종료, 회전, 탐색
config/imu.py         자이로 자체에 관한 값        주파수, 부호, 보정 시간, 끊김 판정
config/main.py      위 셋을 모아 부르는 자리      값은 여기 안 쓴다
```

의존은 한 방향뿐이다 — `detection` → `control`. 검출의 흔들림 문턱이
도킹 허용치에서 나오기 때문이다. `src/` 는 자기 갈래를 직접 부르고,
`tools/` 는 여러 갈래가 필요하니 `config.main` 을 부른다.


---

## 제어 코드 설계 노트

`src/models/control/` 의 주석을 이리로 옮겼다 — 코드는 로직만, 설명은 여기.
값의 근거는 아래 값 사전 절, 실행법은 위 실행 절.

---

### 전체 그림 (control_from_pose.py)

카메라가 잰 위치(lateral/forward/heading)를 보고 다음 동작 하나를 골라
지게차에 보낸다. 한 사이클:

    [측정] 30프레임 → [판단] plan_step() → [실행] 동작 하나 → 다시 측정

한 번에 완벽히 가려 하지 않고 "조금 움직이고 다시 재기"로 오차를 매번 지운다.

| 부품 | 역할 |
|---|---|
| `plan_lateral_clear` / `next_set3_step` / `drive_distance` | 기하·저수준 실행 |
| `plan_step` | 두뇌. 측정값 하나 → 동작 하나 (순수 계산, 하드웨어 없음) |
| `_execute` | 손. 고른 동작을 드라이버로 실행 |
| `dock_live` | 루프. 측정-판단-실행 반복 |
| `CanDriver` | 지게차 연결. `rotate_by()` 는 `rot_control.rotate_to()` 를 부른다 |

IMU 계기판(GyroYaw)은 `src/utils/imu_yaw.py`. 여기서 만들지 않고 run.py 가
만들어 CanDriver 에 넣어 준다(그래서 시뮬레이션은 FakeYaw 를 꽂을 수 있다).
calibrate() 는 보정 중 움직임이 의심되면(moving) **아무것도 커밋하지 않는다**
— 그 속도가 통째로 바이어스에 들어간 채 폐루프가 시작되는 게 최악이라서다.

#### 동작 세 묶음

    Set1  forward 만큼 직진 (개루프 — N초 가라)
    Set2  최소각 회전 → lateral 만큼 이동 → 반대로 90도 복귀
          (이동~복귀 동안 태그가 안 보인다. 그래서 한 묶음으로 통째로 실행)
    Set3  태그가 안 보이면: 1바퀴는 연속 회전(회전 중에도 프레임을 보다가
          보이면 즉시 정지), 못 찾으면 반화각 걸음으로 블러 없는 확인

#### 용어

    lateral   태그 정면축에서 좌우로 벗어난 거리 [m]. +면 내가 오른쪽
    forward   태그면까지 남은 거리 [m]
    heading   태그 축과 틀어진 각 [도]. +가 반시계(왼쪽)
    개루프    명령만 내리고 결과를 안 봄 — 직진
    폐루프    결과를 보며 조절 — 회전 (IMU 로 목표각에서 멈춤)
    오버슈트  정지 명령 후 관성으로 더 도는 양
    워치독    "이 시간 넘으면 무조건 정지" 안전 타이머

CAN 명령은 다섯 개뿐이다 — 정지/전진/후진/좌회전/우회전. 세기 조절이
없어서(고정 출력) "얼마나"를 직진=시간, 회전=IMU 각도로 만든다.

---

### 부호 약속

    rotate_ccw → heading 증가 (+ = 반시계 = 왼쪽)
    rotate_cw  → heading 감소

카메라 쪽 절반은 실측으로 확인했다(아래 실측 기록). 지게차가 반대로 돌면
`rot_control.rotate_to()` 안의 movement 매핑("rotate_ccw" if deg > 0 ...)
**두 곳만** 맞바꾼다 — `plan_step` 의 ccw/cw 를 바꾸면 IMU 목표 부호까지
같이 뒤집혀 wrong-way 정지만 반복된다. (뒤집힌 채 돌려도 반대 5도에서
알아채고 선다.)

---

### plan_step — if 사다리의 순서가 곧 우선순위

    ① 전진 직후 실종 & 후진 안 해봄  → recover_backup (0.5m 후진, 딱 한 번)
    ② 연속 미검출 ≥ 3               → search (Set3). 3바퀴 실패 → lost
    ③ 연속 hold ≥ 8                 → lost (마지막 이유를 들고 수동전환)
    ④ 태그 못 봄                    → hold
    ⑤ forward 가 직전보다 30mm 늘음  → hold ("뒤로 갔네?")
    ⑥ |lateral| > 허용치            → sidestep (Set2)
    ⑦ |heading| > 허용치            → rotate_ccw/cw
    ⑧ 화면 여유 < 30px              → final (남은 거리 통째) 또는 done
    ⑨ forward > 허용치              → forward (Set1, ≤1m 조각)
    ⑩ 나머지                        → done

#### 순서에 이유가 있는 곳

**Set2(⑥)가 heading 교정(⑦)보다 먼저** — lateral 이 큰 상태에서 heading 만
0으로 만들면 카메라가 태그와 나란한 방향을 보게 되어 태그를 놓친다
(실측: lateral 3m 에서 재현). Set2 가 둘을 같이 푼다.

**recover_backup(①)이 Set3(②)보다 먼저** — "방금까지 보이다 전진 직후
사라짐"은 방위를 아니 후진이 맞고, "여러 사이클 연속 안 보임"은 방위를
모르니 회전으로 찾아야 한다. 후진해도 안 보이면 backed_up_once 가 남아
다시 안 나오고, misses 만 쌓여 Set3 로 넘어간다.

#### 흔들린다고 멈추지 않는다 (2026-09-06 결정)

실차는 엔진·유압·노면 때문에 항상 흔들린다. 흔들림을 이유로 멈추면 재도
같은 값이 나와 계속 멈추고, 도킹이 시작조차 안 된다. 쓰는 값이 30프레임
중앙값이라 원래 흔들림에 강하고(1프레임 ±9mm → 30프레임 ±1.4mm), 한 걸음이
짧아(≤1m) 다음 사이클에 고칠 수 있다. 판정(stable/reasons)은 계속 계산해서
화면·기록에 남기고 **판단에만 안 쓴다**. `analyze_run.py` 가 흔들림 비율을
집계한다.

#### 각도는 무조건 믿는다 (2026-09-06 결정)

reliable_angle(각도 신뢰 판정)을 판단에서 뺐다 — 계산·기록만 남음.
30프레임 중앙값이면 5m 에서도 heading 오차가 도 단위 이하라 정렬 목표로
충분하고, 회전 자체는 IMU 폐루프다. 이전의 "못 믿으면 다가가기" 행동은
git 이력(d5d694b 이전)에 있다. 되살릴 거면 몬테카를로 대신
ALIGN_MAX_M(정렬 허용 최대 거리) 하나로 만드는 게 낫다 — sigma 곡선에서
오프라인 도출(30cm 태그 기준 ≈3m), 런타임 계산 없음.

#### 전진 중 heading 허용각 (fwd_abort_deg)

d 만큼 직진하면 옆으로 d×sin(heading) 밀리므로 asin(허용치/이번 걸음)이
기하 허용각이다. 호출부가 max(기하, HEAD_TOL_DEG)로 하한을 깔고 3프레임
연속을 요구하므로 별도 잡음 하한(옛 3×sigma)은 필요 없어져 제거했다.

---

### dock_live — 프레임 루프

측정할 때만 읽고 명령 중엔 눈을 감으면, 명령 한 번에 태그가 화면에서
잘리는 지점을 지나쳐 놓친다(실측: 4.0m 에서 1.32m 명령이 한계 2.55m 를
넘어갔다). 그래서 프레임을 끊김 없이 읽으며 상태(phase)를 굴린다:

    measure  30프레임 모아 중앙값 → plan_step
    command  명령 실행 중. 프레임마다 감시:
               heading 이 문턱을 **연속 3프레임** 넘으면 중단
                 문턱 = max(asin(허용치/이번 걸음), HEAD_TOL_DEG) — "이번에 달릴
                 거리" 기준이다. 남은 전체 거리로 재면 planner 가 방금 용인한
                 heading 이 곧바로 중단을 불러 전진→중단 무한루프가 된다.
                 3프레임 요구는 원시 1프레임 잡음(±0.45도 실측) 때문.
               화면 여유 < TAG_CUT_MARGIN_PX 면 즉시 중단 (기하라 즉시)
    search   Set3 회전 중
    final    마지막 개루프 직진. 끝나면 곧장 done — 태그가 안 보이는 게
             정상이므로 실종 처리(recover_backup)로 가지 않는다
    done / manual  종료. manual 은 사람이 필요하다는 뜻

탐색 걸음은 단계로 세지 않는다 — 30단계를 탐색만으로 소진하면 정작
도킹할 몫이 안 남는다.

TagPipeline 순회는 **Result 를 뱉는다** (3-튜플이 아니다 — 그건 pipe.frames
쪽 계약). 검출이 to_thread 안에서 끝나므로 이벤트 루프가 안 막힌다 —
CAN TX(5~10ms)와 heartbeat 가 검출 시간만큼 밀리면 지게차가 정지 판정을 낸다.

태그 전환은 **제거했다** (2026-09-06) — 첫 실차 시험은 태그1 하나로만
간다. 태그2 로 갈아타는 코드는 git 이력(b800d67 이전)에 있고, 되살릴 때는
전환 시 창 증거(margins/saw_any)와 기준값(prev_forward/margin_px) 리셋을
같이 가져와야 한다 — 안 그러면 태그1 증거가 태그2 판단에 섞인다.

search/final 완료 시 회전 결과를 확인한다 — IMU 가 죽어 회전이 거부되면
(imu-stale) 탐색이 제자리 헛돌기만 하므로 즉시 수동전환하고, final 에서
예외가 나면 done 이 아니라 manual 로 기록한다.

---

### 드라이버

**DryRunDriver** — 아무것도 안 보내고 찍기만. CAN 없이 순서를 볼 때.

**CanDriver** — control_forklift_v2 의 컨트롤러에 명령을 태운다.
`current_movement` 를 바꾸면 그쪽 TX 루프(10ms)가 알아서 계속 쏜다.
`_hold` 는 예외가 나도 반드시 "stop" 으로 돌려놓고(finally), SETTLE_SEC
만큼 관성이 잦아들기를 기다린 뒤 기록한다. `rotate_by` 는 시작할 때
GyroYaw 가 보정 전이면 경고를 찍는다(바이어스 0 이면 5.5도/분 흘러간다).

---

### rot_control.py — 회전 폐루프

직진은 속도를 모르니 시간으로 명령하지만, 회전은 IMU 가 있어 다르다 —
목표각에 닿을 때까지 보면서 돌리면 각속도를 몰라도 정확하다.
`rotate_to(controller, yaw, deg)` 가 그 전부다.

    매 10ms:  진행각 = 방향 × (yaw.angle_deg − 시작각)
       목표 도달       → 정지
       반대로 5도      → 정지 (wrong-way: CAN 부호 뒤집힘)
       IMU 끊김        → 정지 (imu-stale)
       워치독 초과     → 정지 (timeout)
    정지 후: 0.2~1.0초 사이에서 |rate| < 문턱이면 "멎었다" →
             turned/overshoot 를 재서 기록. 회전 중 자이로 샘플이
             유실됐으면(gyro-gaps) 실패로 표시

시작각은 매번 그 자리에서 스냅샷 뜬다(절대 영점 아님) — 드리프트가
회전 사이에 쌓여도 한 회전 안에서는 영향이 없다.

#### 시간 모델은 실행에 안 쓴다

`rot_sec_from_deg()`(ROT_T0 + 각도/ROT_DEG_PER_SEC)는 화면 표시·워치독
상한·IMU 없을 때 개루프 폴백, 셋에만 쓴다. ROT_T0/ROT_DEG_PER_SEC 는
미측정 가정값인데 **여기를 "안전하게 크게" 올리면 안 된다** — 워치독은
커질수록 안전하지만, 개루프 폴백은 그대로 실행 시간이 되어 더 오래/많이
돈다. 두 용도가 반대 방향이라, 워치독 여유는 ROT_MAX_SEC(30s, 90도를
3도/s 로 돌아도 들어옴)와 ROT_WATCHDOG_GAIN 으로만 준다.

#### 멎음 판정 문턱 (_settle_threshold_dps)

    문턱 = min(CEIL, max(FLOOR, 보정 때 잰 잡음 × K))  [도/s]

하한(FLOOR) 없이 잡음이 아주 작으면 문턱이 너무 빡빡해져 "멎었다"가 영영
안 나온다. 상한(CEIL) 없이 현장 진동으로 잡음이 크게 잡히면 아직 도는
중인데 멎었다고 오판한다 — 그러면 overshoot 가 과소평가돼 ROT_LEAD_DEG 를
잘못 추정한다. 실차로 못 재봤으니 양쪽을 다 막아 뒀다.

ROT_LEAD_DEG 는 관성만큼 목표를 앞당겨 끊는 값(goal = |deg| − lead).
지금 0 = 보정 안 함. 로그의 오버슈트 평균을 넣으면 된다.

---

### 실측 기록

| 항목 | 값 |
|---|---|
| 측정 흔들림 | 1프레임 lateral ±9.0mm / heading ±0.45도, 30프레임 ±1.4mm / ±0.08도 (1/√30) |
| 시야 | 좌우 ±35.2도, 위 22.8도 / 아래 20.5도 — 위아래가 다른 건 cy 가 정중앙이 아니라서. 올려다보는 배치에서 근접 한계에 그대로 들어간다 |
| 부호(카메라) | handspin: 반시계로 돌리니 heading 상승. 0.7s −1.4도 → 3.7s +28.4도(+10.0도/s), 복귀 −12.5도/s. +30.1도에서 태그 놓침(한계 ±35도) |
| 부호(CAN) | ~~JOYSTICK_ROTATE_CCW/CW = 30 → 97~~ 폐기. 2026-09-07 실차에서 byte4=97 은 **포크 상승**이었다. 제자리 회전은 조향축 byte1 을 ±20(147/107)으로 — 광운대 최신 control 과 동일, can_pulse 로 반시계(+yaw) 확인 |
| 회전(CAN) 실측 | can_pulse rotate_ccw 1.5s(강도 20): 명령→움직임 0.85s, 끝에 7.3도/s(가속 중), 정지 명령 뒤 +1.3도 더 돌고 0.4s 에 멎음. → ROT_T0 0.85, ROT_DEG_PER_SEC 8, ROT_LEAD_DEG 1.3 |
| 자이로 | 바이어스 미보정 5.5도/분 → 정지 2초 보정 후 1.0도/분 |
| 직진 조각 | 3m 를 1m×3 조각이면 1.46배 느려짐 (측정 시간 포함) |
| 명령→반응 지연 | 0.51s (다른 팀 전진 모델 FWD_T0 실측). 정지도 같은 경로 — 관성 예산(TAG_CUT 60px, SETTLE 0.8s, ROT_T0 0.5s)의 근거 |
| 검출 블러 | 10px 100% / 12px 96% / 20px 89% / 28px 50% / 32px 0% |

### 아직 안 한 것

- **태그2 좌표 환산** — 두 태그가 어긋나 있으면 전환 순간 lateral/forward 가
  튄다. 실측 후 `src/utils/tag_layout.py` 로 환산할 것.
- **비스듬히 접근** — 목표점을 향해 한 번 돌고 한 번에 가면 2~3배 빠르다
  (0.08m/3.0m 에서 21.4s vs 8.8s). 지금 방식 검증 후 재검토.
- **ROT_LEAD_DEG 실측** — analyze_run 의 오버슈트 평균을 config 에.
- **Set3 에 후진 없음** — 태그가 가까워서 위로 잘린 경우 돌아도 안 보인다.
  실제로 나면 후진을 다시 넣을 것.
- **dock_live 분해** — 241줄 한 함수. 실차 검증 후 measure 단계와 중단
  감시를 빼면 100줄 뼈대만 남는다.


---

## 검출 상세 (measure · 좌표계 · 블러 · SDK 소스)

카메라로 AprilTag 을 보고 **지게차가 탑재부 기준 어디에 어떻게 서 있는지**를 낸다.
그 값을 주행 명령으로 바꾸는 것이 다음 단계다.

```
src/models/tag_pose.py    카메라 → 태그 → lateral / forward / heading      (완료)
src/models/control.py     그 값 → 회전 몇 도, 몇 m 전진                     (빈 파일)
```

---

### 폴더

```
src/models/    image.py       1. 이미지 얻기 — 카메라/bag/영상 -> 프레임 + 내부파라미터
               detection_tag.py    2. 태그 찾기 — 흑백 이미지 -> 검출 결과
               detection_pose.py   3. 자세 구하기 — 검출 -> lateral/forward/heading + 파이프라인
               control.py     (빈 파일) 도킹값 -> 주행 명령
               fwd_time_model.py   다른 팀 모델. 거리 -> 명령 시간
src/utils/     camera.py      카메라 읽기·쓰기
               tag_layout.py  태그 여러 개를 한 좌표계로
               drawing.py     화면에 그리기 (표시 전용)
               util.py        블로그 원본 그대로 둔다. 5개만 씀
               make_tag_pdf.py                        실행 스크립트
tools/         run.py                도킹 자동 실행          <- 실사용
               check/live_pose.py    실시간 화면
               check/verify.py       실측 검증 (줄자)
               check/device_check.py 카메라 + IMU 점검
               viewer.sh      realsense-viewer 실행기 (rsview 로 링크됨)
legacy/        viewer_filter/ realsense-viewer 용 C++ 확장. 실사용 경로 아님
               옛 노트북, 블로그 원본 유틸, 캘리브레이션 스크립트
librealsense/  SDK 소스 (555MB, .gitignore)
```

의존 방향은 한 줄이다. 순환이 없다.

```
image.py  ->  detection_tag.py  ->  detection_pose.py
```


---

### 자주 쓰는 명령

```bash
rsview                                                # realsense-viewer (카메라 자동 연결)
python tools/check/live_pose.py --source realsense          # 실시간 화면
python tools/check/verify.py --frames 200 --truth-z 2.00    # 줄자 대조
python src/utils/make_tag_pdf.py --id 1 --size 200 --paper A3
```

---

## 알게 된 사실

아래는 전부 **실측이나 SDK 소스 확인**으로 얻은 것이다. 코드의 상수들이 왜 그 값인지가 여기 있다.

### 1. 카메라 (D435i)

#### 공장 내부파라미터 — 해상도마다 다르다

| 해상도 | fx | fy | cx | cy |
|---|---|---|---|---|
| 1920×1080 | 1359.2 | 1359.0 | 956.9 | 571.3 |
| 1280×720 | 906.1 | 906.0 | 637.9 | 380.9 |
| 640×480 | 604.1 | 604.0 | 318.6 | 253.9 |
| IR 640×480 | 389.6 | 389.6 | 324.9 | 232.9 |

같은 렌즈라도 해상도에 비례한다. `CameraIntrinsics.from_realsense()` 가 스트림에서
직접 읽으므로 해상도를 바꿔도 자동으로 맞는다.

초기에 HFOV 60° 로 가정해 `fx≈1662` 를 썼는데 실제는 1359.2 였다 — **22% 차이**.
거리가 그만큼 틀렸다.

#### 왜곡계수가 전부 0

RealSense 가 이미 보정해서 준다. `undistort()` 를 파이프라인에 연결할 필요가 없다.

#### 노출

- 단위는 **100 µs**. `exposure=83` 이 8.3 ms. 범위 1..10000 으로 확인.
- **`auto_exposure_limit` 은 color 센서에 없다.** depth 전용이고, 롤링셔터는 제외돼 있다
  (`d400-device.cpp:1062` — "ae / gain limit feature is not supported on rolling-shutter").
  → 블러를 막으려면 **수동 고정뿐**이다.
- 검출은 밝기에 매우 둔감하다. **약 11스톱** 범위에서 100% 검출.
  1/16 배로 어둡게 해도, 32배로 96% 픽셀이 포화돼도 검출된다.
  → **어두운 건 문제가 아니다. 노출이 길어져서 생기는 블러가 문제다.**

#### AE ROI (자동노출 영역 지정)

카메라 펌웨어 기능(`SETRGBAEROI = 0x75`). FW 5.10.9 이상. 우리는 5.15.1.55.

실장비로 확인한 제약 두 가지:

1. **자동노출이 켜져 있어야 한다.** 꺼두고 걸면 `hwmon command 0x75 failed`.
2. **스트림이 안정된 뒤라야 한다.** 시작 직후에 걸면 전부 거부, 30프레임쯤 받은 뒤엔 성공.

→ **노출 고정과 AE ROI 는 동시에 못 쓴다.** 정지 상태에서 ROI 로 적정 노출을 찾고,
그 값을 고정한 뒤 주행하는 2단계로 써야 한다.

#### 픽셀 포맷 — bgr8 vs yuyv

`bgr8` 을 요청하면 SDK 가 BT.601 limited-range 로 변환하는데, 이때
**Y≤16 은 전부 0, Y≥235 는 전부 255** 가 된다. 256단계 중 38단계(15%)가 사라진다.
하필 그 구간이 태그의 검은 칸과 흰 칸이다. 뭉개진 픽셀은 기울기가 0 이라
`refine_edges` 가 모서리를 정밀화할 재료가 없다.

`color_format="yuyv"` 로 받으면 센서 원본 휘도를 그대로 쓴다. 비용 약 0.6 ms/프레임.
(현재 기본값은 `bgr8`. 실카메라 A/B 미측정.)

#### 프레임 큐

파이프라인 출력 큐는 **용량 1** 이고 넘치면 **오래된 것을 버린다**.
`wait_for_frames` 는 절대 밀리지 않는 대신 조용히 드랍한다.
`frame_number` 의 구멍으로만 알 수 있다 → `FrameStats`.

#### 메타데이터 (WSL2)

프레임별 실제 노출·게인은 UVC 벤더 페이로드로 오는데, 리눅스에서는
`V4L2_META_FMT_D4XX` 메타 노드가 필요하고 그건 librealsense 커널 패치가 만든다.
**WSL2 기본 uvcvideo 에는 없다.** 없으면 SDK 가 시스템 시간으로 조용히 대체한다.
→ `global_time_enabled` 도 무의미하고, 타임스탬프를 미분해 속도를 내면 안 된다.

---

### 2. 모션 블러 — 실패의 진짜 원인

```
blur_px = fx · v · t / z          (핀홀 기하 그대로)
```

측정된 절벽 (1920×1080, 태그 238px = 10칸이므로 1칸 23.8px):

| 블러 | 검출률 |
|---|---|
| 0~10 px | 100% |
| 12 px | 96.4% |
| 20 px | 89.3% |
| 28 px | 50.0% |
| 32 px | **0%** |

블러가 **1칸의 0.42배까지는 멀쩡하고 1.34배에서 완전히 무너진다.**

실제 지게차 속도 0.284 m/s (다른 팀 `fwd_time_model` 의 vmax) 기준:

| 거리 | 자동노출 33.3 ms | 고정 8.3 ms |
|---|---|---|
| 3.0 m | 4.3 px (100%) | 1.1 px (100%) |
| 2.0 m | 6.4 px (100%) | 1.6 px (100%) |
| 1.0 m | 12.9 px (87%) | 3.2 px (100%) |
| 0.6 m | 21.4 px (48%) | 5.3 px (100%) |

**가까울수록 나빠진다** — 태그가 크게 보이니 화면상 이동량이 커진다.
하필 도킹 마지막 구간이다. **노출을 8.3 ms 로 고정하면 전 구간 100%.**

8.3 ms 는 60 Hz 반주기이기도 해서 형광등 플리커에도 유리하다.

---

### 3. 검출 라이브러리 — AT2 에서 AT3 로

| | AT2 (`apriltag` 0.0.16) | AT3 (`pupil_apriltags`) |
|---|---|---|
| lateral 오차 평균 | 333.0 mm | **60.5 mm** |
| lateral 오차 최대 | **3069.2 mm** | 235.2 mm |
| heading 오차 평균 | 3.81° | **1.07°** |
| 노이즈 σ=5 | **세그폴트** | 정상 |
| 노이즈 σ=70 | 세그폴트 | 정상 |

**AT2 는 가끔 자세를 통째로 뒤집는다.** 5 m / heading 30° 배치에서 lateral 을
−1.5 m 대신 +1.6 m 로, heading 을 30° 대신 −4° 로 보고했다. 평면 자세 모호성에서
잘못된 해를 고른 것이다. 주행 중 이러면 지게차가 반대로 움직인다.

그리고 노이즈가 조금만 있어도 `contour_detect` 에서 **세그폴트로 프로세스가 죽는다.**
파이썬 `try/except` 로 못 막는다 — 주행 중이면 도킹이 통째로 멈춘다.

#### 함정 — 모서리 순서가 반대다

```
AT2   반시계
AT3   시계
```

그대로 두면 자세가 뒤집힌다. `detect()` 가 AT3 결과를 뒤집어 AT2 순서로 맞춘다.
그래서 `_object_points()` 와 `pose_by_pnp()` 는 예전 그대로 동작한다.

#### decision_margin 은 거리를 못 잡는다

태그가 238 px → 19.5 px 로 줄어도 margin 은 71.8~73.2 로 평평했다.
검출률은 100% → 89.3% 로 떨어지는데도 그렇다.

→ **`tag_pixel_size()` 가 거리를 잡는 유일한 게이트다.**
margin 은 밝기에는 반응하므로 저조도 하한으로만 쓴다.

---

### 4. 전처리 — 넣지 않은 이유

28프레임 클립으로 9가지를 다 돌린 결과 **전부 28/28 (100%)** 로 같았다.

```
BGR2GRAY / B / G / R / max(BGR) / min(BGR) / CLAHE / R+CLAHE / LAB-L
```

**CLAHE 는 오히려 해롭다.** 모서리를 평균 0.392 px, 최대 2.572 px 밀어낸다
(단일 채널은 0.02~0.04 px). 얻는 건 margin 73.2 → 77.8 뿐인데 margin 은 이미
하한의 3.7배라 남아돈다.

이유: AprilTag 은 **내부에서 지역 적응 임계화**를 한다. 밖에서 대비를 키워봐야
중복이고, 비선형 변환은 모서리 부근 기울기를 뒤틀어 `refine_edges` 를 방해한다.

**선형 대비 확장은 다르다.** 어두운 이미지(80~130 압축)에서 원본 대비 모서리 오차가
1.095 px → 0.144 px 로 **7.6배 개선**됐다. 선형이라 밝기 곡선 모양이 보존되기 때문이다.
다만 노이즈도 같이 증폭되므로 실카메라 확인이 필요하다.

libealsense 의 처리 블록(spatial / temporal / hole-filling / disparity / colorizer 등)은
**거의 전부 depth 전용**이라 우리가 가져올 게 없었다. 소스에서
`_stream_filter.stream = RS2_STREAM_DEPTH` 로 못박혀 있다.

---

### 5. 좌표계 — 제일 헷갈리는 부분

#### 카메라 기준 vs 태그 기준

```
x, y, z                     카메라 기준 태그 위치.  라이브러리가 주는 것
lateral, vertical, forward  태그 기준 카메라 위치.  역변환한 것 = 제어에 쓰는 값
```

역변환은 `p = -Rᵀ·t` 다. **부호만 뒤집는 게 아니라 회전이 곱해진다.**

같은 자리에 서서 방향만 돌리면:

| heading | x | z | lateral | forward |
|---|---|---|---|---|
| −20° | −1.966 | 2.477 | −1.000 | 3.000 |
| 0° | −1.000 | 3.000 | −1.000 | 3.000 |
| +20° | +0.086 | 3.161 | −1.000 | 3.000 |
| +30° | +0.634 | 3.098 | −1.000 | 3.000 |

**`lateral`/`forward` 는 안 변하고 `x`/`z` 는 부호까지 바뀐다.**
그래서 제어에 `x` 를 쓰면 지게차가 고개를 돌릴 때마다 목표가 움직인다.

`vertical = 카메라높이 − 태그높이` 다 (그 반대가 아니다).

#### yaw ≠ heading

축 이름이 좌표계마다 다른 축을 가리킨다.

| | 항공기·CG | 카메라 (OpenCV) |
|---|---|---|
| x | 오른쪽 | 오른쪽 |
| y | **위** | **아래** |
| z | 앞 | 앞 |

| 실제 회전 | 항공기 기준 | 카메라 기준 |
|---|---|---|
| 좌우로 돎 | yaw | **pitch** |
| 갸우뚱 | roll | **yaw** |
| 위아래 끄덕 | pitch | **roll** |

측정으로 확인:

```
지게차가 30° 틀어짐   →  yaw   0.00°,  pitch 30.00°,  heading −30.00°
카메라만 30° 갸우뚱   →  yaw −30.00°,  pitch  0.00°,  heading   0.00°
지게차 20° + 카메라 15°  →  yaw −15.00°,  pitch 20.00°,  heading −20.00°
```

**`yaw` 는 카메라 장착 기울기를 잰다. 지게차 방향과 무관하다.**
`heading_deg` 만이 "지게차가 태그 축에서 몇 도 틀어졌나"에 답한다.

`docking_state()` 는 rpy 를 거치지 않고 **회전행렬에서 직접** heading 을 뽑는다
(`fwd = R·[0,0,1]`, `heading = atan2(fwd[0], fwd[2])`). 오일러 각은 분해 순서에 따라
값이 달라지고 짐벌락에서 튀기 때문이다.

#### approach vs heading — 독립이다

```
approach   어디 있나 (위치).  태그에서 봤을 때 축에서 몇 도 벗어난 자리인가
heading    어디 보나 (자세).  지게차가 향한 방향이 축과 몇 도 어긋났나
```

축 위에 있어도 고개를 돌리고 있으면 `heading≠0`, 옆에 비켜서 태그를 똑바로 보면
`heading=0` 인데 `approach≠0` 이다. **진입하려면 `lateral` 과 `heading` 이 둘 다 0.**

---

### 6. 각도를 언제 믿을 수 있나

`tilt_deg` = 태그면 법선과 카메라 광축이 이루는 각. **화면에서 찌그러진 정도**다.

3 m 거리, 25 cm 태그에서 좌변·우변 길이 차이:

| tilt | 좌변 | 우변 | 차이 |
|---|---|---|---|
| 0° | 75.5 px | 75.5 px | 0.0% |
| 2° | 75.7 px | 75.4 px | 0.3 px |
| 5° | 76.1 px | 75.5 px | 0.6 px |
| 10° | 77.2 px | 76.1 px | 1.1 px |
| 45° | 111.4 px | 102.5 px | 8.9 px |

**tilt 2° 에서는 좌우 변 차이가 0.3 px** 다. 모서리 검출 노이즈에 묻힌다.
그래서 `RELIABLE_TILT_DEG = 10.0`.

각도 오차 실측 (3 m, AT3):

| heading | 각도 오차 | lateral 오차 |
|---|---|---|
| 0° | −4.47° | 235 mm |
| 5° | −1.59° | 85 mm |
| 10° | −2.04° | 108 mm |
| 20° | −0.14° | 9 mm |
| 30° | +0.50° | −25 mm |

**검출은 어느 각도에서든 100% 된다. 틀리는 건 각도 값뿐이다.**

#### 판정 기준을 `approach` 에서 `tilt` 로 바꾼 이유

예전 `reliable_angle = approach >= 10°` 는 **축 위에 서면(lateral 0) 영영 false** 였다.
30° 틀어져 있어도 approach 가 0 이기 때문이다.

실측 로그에서도 `tilt 18.9°, approach 2.3° → reliable 0` 이 나왔다 —
각도를 잴 수 있는 상태인데 "믿지 마라"고 답한 것이다.

→ `reliable_angle = tilt_deg >= 10°`. 파이썬 2곳과 C++ 필터 1곳을 같이 맞췄다.

#### 설계상 잘 맞는다

```
멀리서 비스듬히   tilt 크다 → 각도 정확  → heading 으로 회전
가까이 정렬됨     tilt 작다 → 각도 부정확 → lateral 로 미세조정
```

도킹 목표가 heading 0° 이므로 **정렬될수록 각도를 못 재게 되는데, 그때는 각도가 필요 없다.**

---

### 7. 검출 거리와 화면 이탈

#### 태그 크기별 최대 거리 (1920×1080, fx=1359.2)

검출에는 태그 한 변이 최소 20 px, 안정적으로는 50 px 필요.

| 태그 | 하한(20px) | 실용(30px) | 안정(50px) |
|---|---|---|---|
| 20 cm | 13.6 m | 9.1 m | 5.4 m |
| 30 cm | 20.4 m | 13.6 m | 8.2 m |
| 40 cm | 27.2 m | 18.1 m | 10.9 m |

해상도를 낮추면 비례해서 준다 (640×480 이면 20 cm 태그가 안정 2.4 m).
IR 은 fx 가 389.6 이라 컬러의 약 1/3.

#### 가까울 때 안 되는 이유 — 화면 이탈

카메라를 수평으로 두고 태그가 위에 있으면, 가까이 갈수록 태그가 화면 위로 나간다.

| 높이차 | 태그 중심이 나가는 거리 | 테두리까지 |
|---|---|---|
| 0.0 m | 항상 보임 | 항상 |
| 0.2 m | 0.50 m | 0.81 m |
| 0.4 m | 1.01 m | **1.31 m** |
| 0.8 m | 2.01 m | 2.32 m |

태그 1.60 m / 카메라 1.20 m 배치면 **1.31 m 아래에서 잘린다.**
태그 크기 한계(0.30 m)나 초점(0.10 m)보다 훨씬 먼저 걸린다.

AprilTag 은 네 모서리로 사각형을 닫아야 하므로 **부분 검출이 없다.**
"점점 나빠지는" 게 아니라 어느 거리에서 뚝 끊긴다.

→ **카메라 높이를 태그 높이에 맞추면** 거리와 무관하게 항상 보인다.

#### 여백(quiet zone)

태그 주위에 흰 여백이 **한 칸 이상** 필요하다. 20 cm 태그면 25 mm.
A4 에는 20 cm 태그가 여백 5 mm 로 겨우 들어간다 → **A3 권장.**

---

### 8. 오차 예산

거리는 `z = fx · tag_size / 화면상_픽셀크기` 로 역산한다. 그래서:

| 거리 | 화면상 태그 | 모서리 0.2px 오차 | tag_size 1mm 오차 | fx 0.3% 오차 | 합성 |
|---|---|---|---|---|---|
| 1 m | 272 px | 0.7 mm | 5.0 mm | 3.0 mm | 5.9 mm |
| 2 m | 136 px | 2.9 mm | 10.0 mm | 6.0 mm | 12.0 mm |
| 3 m | 91 px | 6.6 mm | 15.0 mm | 9.0 mm | 18.7 mm |
| 5 m | 54 px | 18.4 mm | 25.0 mm | 15.0 mm | 34.5 mm |

**오차의 주범은 카메라가 아니라 `tag_size` 측정이다.**
자 대신 캘리퍼를 쓰면 5 m 오차가 34 mm → 23 mm 가 된다.

모서리 오차의 영향은 **거리의 제곱**에 비례한다 (멀수록 태그가 작아지므로).

#### 프레임간 흔들림 (정지 상태 실측, 1.44 m)

| 값 | 표준편차 |
|---|---|
| x, y, z | 0.05 ~ 0.31 mm |
| forward | 0.89 mm |
| **lateral** | **3.79 mm** |
| heading | 0.148° |

`lateral` 이 12배 더 흔들리는 이유: **회전 노이즈가 거리에 곱해진다.**

```
0.145° × 1.44 m = 3.64 mm    ≈ lateral 실측 3.79 mm
```

거리에 비례하므로 5 m 에서는 ±13 mm 가 된다.
**N프레임 평균을 내면 √N 로 줄어든다** (10프레임이면 1/3).

---

### 9. 환경 (WSL2)

#### 카메라 붙이기

```bash
```

재부팅·재연결 때마다 필요하다. 하는 일:

1. 윈도우쪽 `usbipd` 로 장치를 WSL 에 넘김 (interop 으로 `powershell.exe` 호출)
2. `uvcvideo` 커널 모듈 로드
3. `/dev/video*`, `/dev/bus/usb/*` 권한을 `plugdev` 로

**sudo 비밀번호가 필요 없다** — 윈도우 경유로 root 를 얻는다 (`wsl -u root`).

초기에 "WSL2 커널에 uvcvideo 가 없다"고 판단했는데 **틀렸다.**
모듈로 존재하고 로드만 안 돼 있었다 (`/proc/config.gz` 에 `CONFIG_USB_VIDEO_CLASS=m`).
`lsmod`(로드된 목록)만 보고 판단하면 안 된다.

WSL2 에는 udev 데몬이 없어 `/etc/udev/rules.d` 규칙이 자동 적용되지 않는다.
그래서 권한을 수동으로 맞춘다.

#### 카메라는 한 프로세스만 연다

```
1번 프로세스: 열림
2번 프로세스: 실패 — xioctl(VIDIOC_S_FMT) failed, errno=16 (EBUSY)
```

**언어와 무관하다.** 파이썬끼리도 안 된다. 리눅스 V4L2 드라이버 수준의 제약이다.

→ `realsense-viewer` 를 켜면 파이썬이 카메라를 못 연다. 동시에 쓰려면
DDS 네트워크 스트리밍(`rs-dds-adapter`)이 필요한데, SDK 를 `BUILD_WITH_DDS=ON` 으로
다시 빌드해야 하고 프로세스가 3개가 된다.

#### librealsense 가 두 벌

```
pip 휠 안         .../site-packages/pyrealsense2/*.so     파이썬이 씀 (정적 링크)
/usr/local/lib    librealsense2.so.2.58                   realsense-viewer 가 씀
```

pip 모듈은 시스템 라이브러리를 **참조하지 않는다** (`ldd` 에 realsense 없음).
서로 안 섞이지만, 업그레이드할 때는 **둘 다** 해야 버전이 안 갈라진다.

---

### 10. realsense-viewer

**태그를 모른다** (바이너리에 apriltag 문자열 0개). 그래서 C++ 필터를 만들어 끼웠다.

`legacy/viewer_filter/` — SDK 가 열어둔 후처리 필터 슬롯에 클래스 하나를 등록한다.
viewer 본체(39,647줄)는 손대지 않는다.

#### 그 과정에서 찾은 인텔 코드 버그 두 개

**① 오버레이가 매 틱 지워진다**

```cpp
if( ! odf ) {                     // D435i 엔 객체검출 스트림이 없어 항상 여기
    if( ++ticks_without_od_frame > 3 )
        objects->clear();         // 후처리 필터가 방금 채운 것도 같이 지움
}
```

객체검출 파이프라인과 후처리 필터가 **같은 컨테이너를 공유**하는데, OD 스트림이 없으면
4번째 틱부터 계속 비웠다. **인텔 자기네 얼굴검출 예제도 같은 이유로 깨져 있었다.**
→ `od_produced` 플래그로 OD 가 만든 것만 만료시키게 고쳤다.

**② 텍스트가 박스보다 크면 통째로 생략된다**

```cpp
if( size.y < h && size.x < bbox.w )   // 안 들어가면 아무것도 안 그림
```

2~5 m 에서 태그 박스는 50~130 px 인데 숫자 4줄은 그보다 넓다.
**정작 필요한 거리에서 아무것도 안 보였다.** → 박스 밖에 그리도록 고쳤다.

변경분은 `legacy/viewer_filter/viewer_overlay.patch` 에 있다.

#### 역할

```
realsense-viewer   카메라 세팅·캘리브레이션 점검용. 값이 밖으로 안 나간다
live_pose.py       실사용. 화면도 보여주고 값도 손안에 있다
```

viewer 에만 있는 유용한 기능: **On-Chip Calibration + Health-Check**
(공장 내부파라미터가 아직 맞는지 확인). 펌웨어 업데이트, .bag 원클릭 녹화.

---

### 11. 남은 일

```
① control.py            도킹값 → 주행 명령. 다른 팀 fwd_time_model 이 거리→시간을 맡는다
                        (회전 모델은 아직 없음)
② run.py                pose + control 통합 실행
③ 실측 검증             verify.py + 줄자. tag_size 를 캘리퍼로 재는 게 먼저
④ 태그 2               탑재부 내부 태그. tag_layout 에 실측 배치 입력
⑤ 젠슨 나노 포팅        지금 설치본은 전부 x86_64. pyrealsense2 는 ARM 휠이 없어
                        librealsense 를 소스 빌드해야 한다
```

#### `tag_size` 가 지금 가장 큰 오차원

```
tag size : 0.200 m  (ASSUMED default)
```

인쇄한 태그를 **검은 테두리 바깥까지** 재서 넣어야 한다. 1 mm 틀리면 2 m 에서 10 mm,
5 m 에서 25 mm 가 통째로 틀어진다.

---

### 도킹 순서 — 언제 태그를 보나

```
        멈춤          [측정]  30프레임 -> 중앙값. 움직이며 재지 않는다
          |
   1  회전 (90-heading)도      돌다가 35도 넘으면 태그가 시야 밖으로
          |
   2  직진 |lateral|   [실명]  옆을 보고 가므로 태그가 안 보인다. 개루프
          |
   3  회전 90도                태그가 다시 들어온다. heading 도 0 이 된다
          |
        멈춤          [측정]  lateral 이 아직 크면 1~3 반복
          |
   4  직진 min(남은거리, 1m)   태그를 보며 간다
          |                    조각마다 멈춰서 [측정] -> 4 반복
        멈춤          [측정]  태그1 이 화면에서 사라지기 전에 정지
          |
   5  태그2 검출        [측정]  tag_layout 으로 같은 좌표계 환산 -> 진입
```

**태그를 못 보는 구간은 2단계 하나뿐이다.** 그래서 옆이동은 한 번에 가고,
끝난 뒤 3단계에서 다시 보고 확인한다. 직진(4)은 태그가 계속 보이므로
나눠 가며 매번 고쳐 잡는다.

나누는 대가 (3m 직진, 측정 1s 포함):

| 조각 | 조각거리 | 전체 시간 | 느려짐 |
|---|---|---|---|
| 1 | 3.00m | 14.1s | 1.00배 |
| 3 | 1.00m | 19.1s | 1.46배 |
| 6 | 0.50m | 26.7s | 2.04배 |

옆이동은 반대다 — 나눌 때마다 회전이 2번씩 붙어 훨씬 비싸다.

#### 3단계가 heading 도 같이 고친다

```
처음                heading = 20도
(90-20)도 회전 후    heading = 90도
90도 되돌린 후       heading =  0도
```

따로 "정면 맞추기" 단계가 필요 없다.

#### 정해야 할 값

```
LAT_TOL   lateral 이 이보다 작으면 됐다고 본다. 명령 하한 17mm, 측정오차 1.4mm
STEP_M    직진 한 조각. 1.0m 면 1.46배 느려짐
STOP_M    태그1 을 놓기 전 멈출 거리. 지금 높이차(0.4m)면 1.31m 에서 잘린다
```

#### 막고 있는 것

1·3단계의 **회전 모델이 없다.** 받은 것은 직진(거리->시간)뿐이다.
다른 팀에 물어볼 것 — 제자리 회전 명령이 있나, 각도를 주나 시간을 주나, 몇 도에 몇 초인가.


---

### 12. SDK 소스 근거

코드에서 인용을 걷어냈으므로 여기 모은다. 경로는 `librealsense/src/` 기준.

| 알게 된 사실 | 소스 |
|---|---|
| `auto_exposure_limit` 은 depth 센서에만 등록. 그나마 `CAP_GLOBAL_SHUTTER` 일 때만 (주석: "not supported on rolling-shutter") | `ds/d400/d400-device.cpp:1062-1070` |
| 컬러 센서가 실제로 받는 옵션은 넷뿐 — EXPOSURE / GAIN / ENABLE_AUTO_EXPOSURE / AUTO_EXPOSURE_PRIORITY | `ds/ds-color-common.cpp:84-97` |
| 컬러 노출은 v4l2 로 직행하므로 **100 µs 눈금** (UVC 규격) | `linux/backend-v4l2.cpp:2476` → `V4L2_CID_EXPOSURE_ABSOLUTE` |
| **뎁스/IR 노출은 µs 단위** — 같은 enum 인데 센서에 따라 100배 다르다 | `ds/ds-color-common.cpp:117` |
| **노출을 쓰면 AE 가 조용히 꺼진다.** EXPOSURE 가 `auto_disabling_control` 로 감싸여 있어 set 하면 SDK 가 AE 를 0 으로 내린다 | `ds/ds-color-common.cpp:90-93`, `option.cpp:84-105` |
| gain 은 `value==0` 일 때만 기본값을 덮어쓴다 | `ds/ds-color-common.cpp:26-49` |
| `power_line_frequency` 값 매핑 (0=끔 1=50Hz 2=60Hz 3=자동) | `ds/d400/d400-color.cpp:205-212` |
| 파이프라인 출력 큐는 **용량 1**, 넘치면 오래된 것을 버린다 | `pipeline/aggregator.cpp:16` |
| pipeline 이 내부에 syncer 를 물고 있다 | `pipeline.cpp:219` |
| 프레임 메타데이터엔 `V4L2_META_FMT_D4XX` 노드가 필요하고, 그건 커널 패치가 만든다 | `linux/backend-v4l2.cpp:2875` |
| 메타데이터가 없으면 **조용히** 시스템 시간으로 대체 ("UVC metadata payloads not available") | `ds/ds-timestamp.cpp:60-74` |
| `time_of_arrival` 은 항상 온다 | `sensor.cpp:79` |
| AE ROI 기본값은 가운데 — 사방으로 크기의 1/8 씩 뗀 영역 | `common/stream-model.cpp:331-343` |
| BGR8 변환식 (BT.601 limited-range) | `color-formats-converter.cpp:293-301` |
| Y8 은 컬러 스트림 포맷 목록에 없다 | `device.cpp:197` `map_supported_color_formats` |

#### 여기서 나오는 함정 하나 — 노출과 AE 는 같이 못 쓴다

```
set_option(EXPOSURE, ...)   →  SDK 가 ENABLE_AUTO_EXPOSURE 를 0 으로 내린다
```

의도된 동작이지만 **아무 경고가 없다.** 쓴 뒤에는 반드시 되읽어 확인할 것.
AE ROI 가 실패하는 것도 같은 이유다 (ROI 는 AE 가 켜져 있어야 걸린다).


---

### 참고

- 원본 블로그: <https://joonhyung-lee.github.io/blog/2023/apriltag-pose-estimation/>
  (`legacy/blog_post.html` 에 저장. 블로그는 카메라 설정을 하나도 안 건드렸다 —
   정지 데모라 블러가 없었기 때문이다.)
- 다른 팀의 주행 명령 모델: `../forward/fwd_time_model.py`
  거리 → 명령 시간 변환. `t0=0.507s` 지연, 가속 2.036 s, `vmax=0.284 m/s`.
