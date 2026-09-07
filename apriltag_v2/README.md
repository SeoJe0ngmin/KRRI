# AprilTag 지게차 도킹 — v2 (조준 접근)

카메라로 AprilTag 을 보고 **탑재부 기준 지게차 위치**를 낸 뒤, 그 값으로 주행한다.

**v2 가 v1(`../apriltag_v1`)과 다른 것** — 실행법·설정·검출은 같고 주행 규칙만 다르다.
2026-09-07 실차 11회가 전부 "7~10m 에서 5~50cm 옆이동 반복"으로 끝난 뒤 바꿨다.

    v1  10m 밖에서부터 |lateral| > 30mm 면 옆이동(90도 회전 → 짧은 직진 → 90도 복귀)
    v2  4m 밖: 태그 축 위 4m 지점을 **조준해 곧장** 간다 (30mm/2도 안 봄)
        4m 안: "지금 직진하면 어디 도착하나"로 heading 을 정하고 간다.
              lateral 이 heading 으로 못 삼킬 만큼 크면 옆으로 안 가고 **물러난다**
        마지막 눈감는 직진: IMU 가 지켜보다 틀어지면 정지 → 수동전환

허용치(`LAT_TOL_M`/`HEAD_TOL_DEG`)는 그대로다. 언제부터 따지느냐만 바뀌었다.
규칙 설명은 [CODE_NOTES.md](CODE_NOTES.md), v1 기록으로 v2 판단을 미리 보려면
`python tools/replay_plan.py`.

처음 받았다면 — 현장 노트북(Windows)은 [SETUP_WINDOWS.md](SETUP_WINDOWS.md) 부터.
문서는 넷뿐이다:
[SETUP_WINDOWS.md](SETUP_WINDOWS.md) 현장 노트북 설치 ·
[FIELD_MEASURE.md](FIELD_MEASURE.md) 값 사전(잴 것/정해진 것/카메라가 주는 것) ·
[CODE_NOTES.md](CODE_NOTES.md) 제어 코드 설계 노트(주석은 코드가 아니라 여기 있다)

```bash
pip install -r requirements.txt
./tools/wsl_attach_camera.sh          # WSL2 에서 카메라 붙이기 (재부팅마다)
realsense-viewer                      # 가끔 카메라 점검할 때 (SDK 기본 제공)
python tools/live_pose.py             # 화면으로 확인
python tools/imu_check.py             # IMU 부호·드리프트 확인
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
| `models/detection/detection_README.md` | 실측·근거 모음 (블러, 좌표계, SDK 소스 등) |
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

| 파일 | 하는 일 |
|---|---|
| `run.py` | **도킹 자동 실행.** 시작만 키보드, 그 뒤는 카메라가 정한다 |
| `live_pose.py` | 실시간 화면 + 숫자. `--log` JSON, `--record` .db3 |
| `verify.py` | 줄자로 잰 값과 대조 |
| `imu_check.py` | IMU 드리프트·부호 확인. `--tag` 로 태그와 교차검증 |
| `realsense_check.py` | 환경 진단 — 어느 층에서 깨졌나 |
| `analyze_run.py` | 운행 기록 분석 — 결과 판정, 회전·직진 파라미터 권장값 |
| `sim.py` | 정답을 지어내서 오차 측정 (카메라 없이). `--live` 로 3D + 슬라이더 |
| `sim_measure.py` | 배치 → 렌더 → 검출 → 오차 |
| `sim_engine.py` | 그림과 정답을 만드는 엔진 |
| `wsl_attach_camera.sh` | WSL2 에 카메라 붙이기 |

---

## 지금 막혀 있는 것

```
① IMU_YAW_SIGN 확인          python tools/imu_check.py  — 반시계로 돌려 +가 나오나
② 회전 부호 확인              rotate_ccw 에 지게차가 왼쪽으로 도나 (눈으로)
③ 탑재부 허용 오차            LAT_TOL_M / HEAD_TOL_DEG 의 근거. 현장에서 받아야 함
④ 태그2 위치 실측             tag_layout 으로 좌표 환산해야 전환 때 값이 안 튄다
```

`ROT_DEG_PER_SEC` 은 더 이상 급하지 않다 — 회전이 폐루프라 결과를 안 바꾼다.
