# detection 이 control 에 넘기는 것

## 한 줄 요약

`measure()` 가 **딕셔너리 하나**를 돌려준다. 리스트가 아니다.
태그 하나에 대한 값이고, 30프레임을 모아 낸 대표값이다.

```python
from src.config import TAG_ID, TAG_SIZE_M
from src.models import TagPipeline, measure

pipe = TagPipeline.from_realsense(TAG_SIZE_M)
m = measure(pipe, tag_id=TAG_ID)      # 약 1초 걸린다(30프레임)
```

---

## 실제로 나오는 것

```python
{'lateral':        0.0257,    # float
 'vertical':      -0.2501,
 'forward':        1.3645,
 'distance':       1.3874,
 'approach_deg':   1.0792,
 'heading_deg':   17.5907,
 'tilt_deg':      18.7035,
 'z_optical':      1.3127,
 'n':             30,         # int
 'spread':        {...},      # dict — 각 값의 표준오차
 'reliable_angle': True,      # bool
 'stable':         True,      # bool
 'reasons':        []}        # list[str]
```

**태그를 못 봤으면 `None` 을 돌려준다.** 반드시 먼저 확인할 것.

---

## 키 설명

### 제어에 쓰는 것 — 이 셋이면 된다

| 키 | 단위 | 뜻 |
|---|---|---|
| `lateral` | m | 태그 정면축에서 좌우로 벗어난 거리. **+ 가 오른쪽** |
| `forward` | m | 태그면까지 남은 거리 |
| `heading_deg` | deg | 지게차가 태그 축과 틀어진 각. **+ 가 반시계(왼쪽)** |

### 써도 되는 값인지 판정

| 키 | 뜻 |
|---|---|
| `stable` | `False` 면 **명령을 내지 말고 다시 재라.** 누가 지나갔거나 아직 안 멈춘 것 |
| `reasons` | `stable=False` 인 이유. 예: `["lateral 흔들림 15mm"]` |
| `reliable_angle` | `False` 면 **`heading_deg` 를 믿지 마라.** 태그가 정면에 가까워 각도를 못 잰다 |
| `spread` | 각 값의 표준오차 [같은 단위]. 예: `spread["lateral"] = 0.0007` = ±0.7mm |
| `n` | 실제로 쓴 프레임 수. 30 보다 작으면 태그를 놓친 프레임이 있었다 |

### 참고용 (제어에 안 씀)

| 키 | 단위 | 뜻 |
|---|---|---|
| `vertical` | m | 카메라높이 − 태그높이 |
| `distance` | m | 직선 거리 (높이차 포함) |
| `z_optical` | m | 카메라 광축 방향 거리. `forward` 와 다르다 |
| `approach_deg` | deg | 태그에서 봤을 때 축에서 몇 도 비켜 있나 (**위치**) |
| `tilt_deg` | deg | 태그가 화면에서 찌그러진 정도. `reliable_angle` 의 근거 |

---

## 쓰는 순서

```python
m = measure(pipe, tag_id=TAG_ID)

if m is None:
    ...                      # 태그를 못 봤다. 되돌아가거나 다시 찾는다
elif not m["stable"]:
    ...                      # 흔들린다. 다시 잰다 (m["reasons"] 에 이유)
else:
    lat  = m["lateral"]      # m
    fwd  = m["forward"]      # m
    head = m["heading_deg"]  # deg,  m["reliable_angle"] 이 True 일 때만 믿을 것
```

---

## 반드시 알아야 할 것 넷

**① 움직이면서 부르면 안 된다.** 30프레임을 모으는 1초 동안 지게차가 움직이면
값이 섞인다. **멈춘 뒤에 부를 것.**

**② `heading_deg` 는 `reliable_angle` 이 `False` 면 쓸 수 없다.**
태그를 정면에서 보면 원근 왜곡이 픽셀 이하라 각도를 못 잰다(tilt 2도면 좌우
변 길이차가 0.3px). 다행히 그때는 이미 정렬돼 있어 회전이 필요 없다.

**③ `approach_deg` 와 `heading_deg` 는 다르다.**
`approach` 는 **어디 있나**(위치), `heading` 은 **어디 보나**(자세)다.
축 위에 서서 고개만 돌리면 `approach=0` 인데 `heading≠0` 이다.
회전 명령에는 `heading_deg` 를 쓴다.

**④ `forward` 와 `z_optical` 은 다르다.**
`forward` 는 태그면까지, `z_optical` 은 카메라 광축 방향이다.
틀어져 있을수록 벌어진다(heading 20도에서 18cm, 30도에서 40cm).
**`fwd_time_model` 은 로그의 `dist_z` 로 적합됐으니** 어느 쪽을 넘길지 맞출 것.
정렬(heading≈0) 후에 전진하면 둘이 같아져서 문제가 안 된다.

---

## 값이 얼마나 정확한가 (실측)

1.4m 거리, 1920x1080, 20cm 태그 기준.

| | 1프레임 | 30프레임 (`measure`) |
|---|---|---|
| `lateral` | ±9.0 mm | **±0.7 mm** |
| `forward` | ±1.3 mm | ±0.2 mm |
| `heading_deg` | ±0.45 deg | **±0.03 deg** |

`lateral` 이 유독 흔들리는 이유: 각도를 재서 거리를 곱해 얻는 값이라
각도 노이즈가 거리만큼 증폭된다(`0.45도 × 1.4m ≈ 9mm`). **멀수록 커진다.**

거리 자체는 줄자와 대조해 **6.6mm (0.51%)** 안에서 맞았다.
