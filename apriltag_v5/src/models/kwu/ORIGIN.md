# 광운대 원본 — 한 글자도 안 고친 것만 여기 둔다

**여기 있는 파일은 전부 실제로 import 해서 돈다.** 참고만 한 코드는 넣지 않는다
(그러면 "도는 코드" 와 "죽은 참고본" 을 구분할 수 없다). 참고한 곳은 우리 파일에 한 줄로 적는다.

원본 위치: `project_backup_lean_20260921_174252/extracted/depth_cam/`
복사 날짜: 2026-09-22

| 파일 | 원본 | 줄수 | 내부 import | 쓰는 이유 |
|---|---|---|---|---|
| `control.py` | `calib/control.py` | 807 | 0 | CAN 송신 전부. 강도 변경·예약 회전·데드맨·TX 관측 |
| `adaptive_slope.py` | `calib/fsm_v4/adaptive_slope.py` | 37 | 0 | 온라인 학습 틀 (EMA + 클램프) |

**둘 다 내부 import 가 0 이라 연결 코드(shim)가 필요 없다.**

`calib/imu_stream.py` 는 **안 쓴다** — 자이로를 `get_motion_data().y` 원시값으로만 적분한다.
우리는 `src/utils/imu_yaw.py`(apriltag_v3)를 쓴다. 그쪽은 **중력에서 회전축을 뽑아 투영**해서
카메라가 기울어 달려도 안 틀어진다 (CLAUDE.md: "중력축 투영으로 바꾸며 부호가 뒤집혔다").

## 안 가져온 것 — 우리가 새로 쓴다

그쪽 config 상수를 많이 참조해서 그대로는 못 쓴다. 구조만 배우고 상수는 우리 것을 쓴다.

| 파일 | 그쪽 config 참조 | 왜 |
|---|---|---|
| `fsm_v4/route_planner.py` | 22 | 걸음 탐색 구조는 배우되 팔레트 기하라 그대로는 안 됨 |
| `fsm_v4/controllers.py` | 36 | 회전 예측 정지. 우리는 자이로 폐루프라 다름 |
| `fsm_v4/motion.py` | 16 | 예측 정지 계산. **그쪽 지게차 직진 모델이 섞여 있음** |
| `fsm_v4/planner.py` | 76 | 팔레트 삽입 기하 |
| `fsm_v4/top.py` | 151 | 그쪽 전체 흐름 |
| `rotation_fit/icr_fit.py` | — | 동심원 적합. 우리는 yaw 를 같이 써서 다시 씀 |
| `calib/runtime_threads.py` | — | 스레드 구조. 규율만 가져옴 |

## 주의

- `control.py` 의 `__all__` 에 포크·폴딩·리치 명령이 들어 있다. **우리는 6개 동작만 쓴다**
  (`stop` · `forward` · `forward_slow` · `backward` · `rotate_left_slow` · `rotate_right_slow`).
  시작할 때 동결 테이블이 바이트를 확인한다 — plan 9절.
- 기본 회전 강도가 이 파일은 20, 그쪽 `fsm_v4/config.py` 는 30 으로 덮어쓴다.
  **우리는 20 이다**(9/7·9/21 측정 근거). `config/control.py` 가 정한다.
