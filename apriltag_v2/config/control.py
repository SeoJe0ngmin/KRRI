LAT_TOL_M = 0.030              # 좌우 허용 오차 [m]  ★탑재부 사양으로 확정할 것
HEAD_TOL_DEG = 2.0             # 각도 허용 오차 [도] ★오버슈트 실측 후 0.7~1.0 으로 (FIELD_MEASURE)

STEP_M = 1.0                   # 직진 한 조각 [m]
FWD_SAFETY = 0.9               # 명령 거리 = 남은 거리 x 이 값 (모자라게 가는 쪽이 안전)

SIDESTEP_BACKWARD_GAIN_DEG = 0.0   # 0 = 회전이 작은 쪽을 고른다

TAG_CUT_MARGIN_PX = 60.0       # 잘리기 직전 문턱. 정지지연 실측 0.51s x 0.284m/s = 14cm
DOCK_EXTRA_M = -2.02            # 태그면을 지나 더 갈 거리 [m]. 카메라->포크 끝 1.52m + 여유 0.50m 앞에서 정지 (2026-09-07 실측)

MAX_STEPS = 30                 # 넘기면 수렴 실패 (탐색 걸음은 안 셈)
HOLD_MAX_CONSEC = 8            # hold 가 이만큼 연속이면 수동전환 (약 10초).
SETTLE_SEC = 0.8               # 명령 끊은 뒤 실정지까지 대기. 명령지연 실측 0.51s 기반

ROT_WATCHDOG_GAIN = 5.0        # 워치독 = max(30s, 예상시간 x 이 값). 90도 이하 회전은
ROT_LEAD_DEG = 2.5             # 관성만큼 미리 끊는 각 [도]. 2026-09-07 can_pulse 실측:
ROT_WRONG_WAY_DEG = 5.0        # 반대로 이만큼 돌면 부호가 뒤집힌 것 → 즉시 정지
ROT_POLL_SEC = 0.01            # 회전 중 IMU 확인 주기 [s]

ROT_SETTLE_MAX_SEC = 2.0       # 멎기 대기 상한 [s]. 명령지연 0.51s 실측 반영
ROT_SETTLE_MIN_SEC = 0.2       # 최소 대기 [s] (명령 반영 지연)
ROT_SETTLE_POLL_SEC = 0.05     # 멎었나 확인 주기 [s]
ROT_SETTLE_RATE_K = 3.0        # 멎음 판정 = 보정 때 잰 잡음 x 이 값  △실차 진동 보고
ROT_SETTLE_RATE_FLOOR = 1.0    # 그 판정의 하한 [도/s]                △
ROT_SETTLE_RATE_CEIL = 2.0     # 그 판정의 상한 [도/s]                △

SEARCH_BACKUP_M = 0.5          # 전진 중 관성으로 인한 실종 직후 후진 거리 [m]
SEARCH_AFTER_MISSES = 3        # 연속 미검출 이만큼이면 Set3 시작
SEARCH_MAX_ROUNDS = 3          # 1바퀴째=연속 360도(보이면 즉시 정지),

AIM_STANDOFF_M = 3.5           # T 까지 거리 [m] (카메라 기준). 태그가 화면 위로 나가는 거리가 실측 3.3m
CAM_TO_PIVOT_M = 1.46          # 카메라에서 제자리 회전 중심(뒷바퀴)까지 [m]. 2026-09-07 can_pulse rotate_ccw 2s --camera 실측 (Δlateral/sinΔheading)
AIM_NEAR_T_M = 1.0             # T 까지 이 안이면 조준 회전을 더 안 하고 정렬/태그 겨냥으로 넘어간다 [m]
AIM_CHUNK_MAX_M = 3.0          # 조준 뒤 한 번에 달리는 최대 거리 [m]. 모델이 ±5% 인 구간
AIM_MAX_BEARING_DEG = 45.0     # T 방위각이 이보다 크면 조준 대신 90도 사이드스텝(v1 규칙) 폴백
AIM_MAX_TAG_OFF_DEG = 25.0     # 조준한 뒤 태그가 코에서 이보다 벗어나면(반화각 35도) 직진 중 놓친다 -> 폴백.
AIM_TOL_DEG = 3.0              # 조준 오차 허용 [도]. 이 이하면 회전 없이 직진. 회전 잔차 ±1.5도라 2도면 되튄다
AIM_AT_T_M = 0.3               # T 까지 남은 거리가 이 이하면 도착으로 본다 [m]
AIM_FINAL_MAX_LAT_M = 0.10     # T 에서 lateral 이 이 이하면 후진 대신 태그를 직접 겨냥해 마지막 직진.
AIM_FINAL_TOL_DEG = 1.5        # 태그 겨냥 각 오차 허용 [도]. 이 이하면 회전 없이 마지막 직진. 1도면 1~2도 잔회전이 되풀이된다(회전 잔차 ±1.5도)
AIM_BACKUP_M = 1.5             # T 근처인데 lateral 이 AIM_FINAL_MAX_LAT_M 을 넘으면 이만큼 후진해 다시 조준 [m]
AIM_MAX_BACKUPS = 2            # 후진-재조준 최대 횟수. 넘으면 수동전환
AIM_ABORT_WINDOW = 30          # 직진 중 heading 중단 판정: 최근 이만큼 프레임(1초)의 중앙값으로 본다.
AIM_DRIFT_ABORT_DEG = 4.0      # 조준 직진 중 출발 heading 에서 이만큼 흘렀으면 중단 [도]
AIM_STOP_LEAD_M = 0.15         # 카메라 조기 정지: 목표 forward 보다 이만큼 앞에서 정지 명령 (정지지연 0.51s x 0.28m/s)
AIM_STOP_CONFIRM = 3           # 조기 정지 판정 연속 프레임 수
