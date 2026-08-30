# AprilTag 지게차 도킹

카메라로 AprilTag 을 보고 **탑재부 기준 지게차 위치**를 낸 뒤, 그 값으로 주행한다.

처음 받았다면 [SETUP.md](SETUP.md) 부터.

```bash
pip install -r requirements.txt
./tools/wsl_attach_camera.sh          # WSL2 에서 카메라 붙이기 (재부팅마다)
realsense-viewer                      # 가끔 카메라 점검할 때 (SDK 기본 제공)
python tools/live_pose.py             # 화면으로 확인
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
거리 -> 시간   fwd_time_model.py       실측 적합 (다른 팀)
각도 -> 시간   control_from_pose.py    **미측정.** 지금은 가정값
```

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
| `models/control/control_from_pose.py` | 도킹값 → 동작 하나. 순서와 기하 |
| `models/control/control_forklift_v2.py` | CAN 프레임 전송 (다른 팀). 안 고침 |
| `models/control/fwd_time_model.py` | 거리 → 명령 시간 (다른 팀) |
| `utils/camera.py` | 노출·AE ROI·프레임 드랍 등 카메라 설정 |
| `utils/tag_layout.py` | 태그 여러 개를 한 좌표계로. 아직 안 씀 |
| `utils/drawing.py` | 화면에 큐브·축 그리기. 표시 전용 |
| `utils/util.py` | 블로그 원본. 5개만 씀 |
| `utils/realsense_check.py` | 환경 진단 — 어느 층에서 깨졌나 |
| `utils/make_tag_pdf.py` | 인쇄용 태그 PDF (눈금자 포함) |

### `tools/`

| 파일 | 하는 일 |
|---|---|
| `run.py` | **도킹 자동 실행.** 시작만 키보드, 그 뒤는 카메라가 정한다 |
| `live_pose.py` | 실시간 화면 + 숫자. `--log` JSON, `--record` .db3 |
| `verify.py` | 줄자로 잰 값과 대조 |
| `sim.py` | 정답을 지어내서 오차 측정 (카메라 없이). `--live` 로 3D + 슬라이더 |
| `sim_measure.py` | 배치 → 렌더 → 검출 → 오차 |
| `sim_engine.py` | 그림과 정답을 만드는 엔진 |
| `wsl_attach_camera.sh` | WSL2 에 카메라 붙이기 |

---

## 지금 막혀 있는 것

```
① 회전 각도 -> 시간 모델      미측정. 카메라로 직접 잴 수 있다
② tag_size                   0.20m 가정. 캘리퍼로 재야 한다 (거리 오차의 최대 원인)
③ CanDriver 실장비 확인       회전 부호(ccw 가 heading 을 올리나)를 먼저 볼 것
```
