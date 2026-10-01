# 가져온 파일 — 한 글자도 안 고친다

**고치면 원본과 diff 가 안 된다.** 우리가 얹을 것은 별도 파일에 둔다.
`src/models/kwu/ORIGIN.md` 와 같은 규칙이다.

| 파일 | 원본 | 줄수 | 우리가 얹은 것 |
|---|---|---|---|
| `camera.py` | `apriltag_v3/src/utils/camera.py` | 785 | 없음. 그대로 쓴다 |
| `imu_yaw.py` | `apriltag_v3/src/utils/imu_yaw.py` | 477 | **`gyro.py` 의 `Gyro(GyroYaw)`** |
| `drawing.py` | `apriltag_v2/src/utils/drawing.py` | 67 | 없음. 그대로 쓴다 |

다른 곳:
| 파일 | 원본 |
|---|---|
| `../models/detection/image.py` | `apriltag_v3/src/models/detection/image.py` (= v2 + 9/21 USB2·타임아웃 수정) |
| `../models/detection/tag.py` | `apriltag_v2/src/models/detection/detection_tag.py` |
| `../../tools/make_tag_pdf.py` | `apriltag_v2/src/utils/make_tag_pdf.py` (경로 한 줄만 추가) |

## imu_yaw.py 는 **쓴다** — 다만 직접 생성하지 않는다

자이로 계산(각도 적분·바이어스·중력축 투영·끊김 감지)은 전부 이 파일이 한다.
`gyro.Gyro(GyroYaw)` 가 **상속**하므로 `Gyro` 를 하나 만들면 이 파일 코드가 그 안에서 돈다.
`GyroYaw(...)` 를 직접 만드는 코드만 없다 — 항상 `Gyro(...)` 를 만든다.

`Gyro` 가 얹은 것 둘:
1. **최신 가속도 보관** — GyroYaw 는 보정 중에만 모은다. 태그 기울기 보정에 계속 필요하다
2. **회전 정지 판정을 콜백 안에서** — 메인 루프가 밀려도 제때 멈추려고 (plan 4-2-1)

## 안 가져온 것과 이유

| 파일 | 왜 |
|---|---|
| `apriltag_v2/src/utils/util.py` | **matplotlib 을 import** 한다. 쓰는 건 `invert_T` 하나라 `pose.py` 안에 두고 출처를 적었다 |
| `apriltag_v2/src/utils/tag_layout.py` | 태그 여러 장을 한 좌표계로 묶는 것. 지금 1장. **근거리 태그 2장 안이 살아나면 가져온다** |
| `apriltag_v3/src/utils/run_log.py` | **동기 쓰기**라 디스크가 멈칫하면 루프가 같이 멈춘다. `record.py` 가 큐+스레드로 다시 썼다 (plan 6-6). 스키마(frame·imu·can·event·config)는 그대로 따랐다 |
| `apriltag_v3/src/utils/event_log.py` | 위와 같음. `snapshot_config` 는 `record.snapshot()` 이 대신한다 |
| `apriltag_v3/src/utils/calib.py` | 캘리브 파일 형식. `config/measured.py` 가 대신한다 |
| `apriltag_v3/src/utils/timing.py` | v3 전용 타이밍 계측층 |
