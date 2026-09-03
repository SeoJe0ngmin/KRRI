"""도킹값 + IMU 회전각 -> 주행 명령.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
처음 읽는 사람을 위한 안내
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

이 파일이 하는 일을 한 문장으로:
    "카메라가 잰 내 위치(lateral/forward/heading)를 보고,
     다음에 할 동작 하나를 골라서, 지게차에 명령을 보낸다."

전체 흐름 (한 사이클):

    멈춤 -> [측정] 태그 30프레임 -> [판단] plan_step() -> [실행] 동작 하나
      ^                                                        |
      └────────────── 정지 후 다시 측정 ←──────────────────────┘

    골인(done)까지 반복한다. 한 번에 완벽하게 가려 하지 않고 "조금 움직이고
    다시 재기"를 반복해서 오차를 매번 지워나가는 구조다.

파일 안의 부품 (위에서 아래 순서):

    1. plan_step()    두뇌. 측정값 하나 -> 다음 동작 하나 (순수 계산, 하드웨어 없음)
    2. _execute()     손. plan_step 이 고른 동작을 드라이버로 실제 실행
    3. dock/dock_live 루프. 측정-판단-실행을 반복
    4. CanDriver      지게차와의 연결. rotate_by() 가 "목표각까지 돌기" 폐루프

    IMU 계기판(GyroYaw)은 src/utils/imu_yaw.py 에 있다. 여기서 만들지 않고
    run.py 가 만들어서 CanDriver 에 넣어 준다.

용어 사전:

    lateral      태그 정면축에서 좌우로 벗어난 거리 [m]. +면 내가 오른쪽에 있음
    forward      태그면까지 남은 거리 [m]
    heading      내가 태그 축과 몇 도 틀어져 있나. **+가 반시계(왼쪽)**
    개루프       명령만 내리고 결과를 안 보는 것. 직진이 이렇다 (N초 가라)
    폐루프       결과를 계속 보면서 명령을 조절하는 것. 회전이 이렇다
                 (IMU 를 보다가 목표각에 닿으면 멈춘다)
    오버슈트     정지 명령을 내려도 관성으로 조금 더 도는 것
    워치독       "이 시간 넘으면 무조건 정지" 안전 타이머

CAN 명령은 다섯 개뿐이다 — 정지/전진/후진/제자리좌회전/제자리우회전.
세기 조절이 없어서(고정 출력) "얼마나"는 직진=시간, 회전=IMU 각도로 만든다.

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
실측 기록
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    측정 흔들림   1프레임  lateral +-9.0mm / heading +-0.45도
                  30프레임 +-1.4mm / +-0.08도   (1/sqrt(30))
    시야          좌우 +-35.2도,  위 22.8도 / 아래 20.5도
                  위아래가 다른 이유는 cy 가 화면 정중앙이 아니라서다.
                  태그를 올려다보는 배치에서는 이 차이가 근접 한계에 그대로 들어간다.
    부호(카메라)  handspin 로그 — 카메라를 반시계로 돌리니 heading 이 올라갔다
                      0.7s -1.4도 -> 3.7s +28.4도  (+10.0 도/초)
                      되돌리니 -12.5 도/초. +30.1도에서 태그를 놓쳤다(한계 +-35도)
    부호(CAN)     JOYSTICK_ROTATE_CCW/CW = 30 -> 127-30 = 97.
                  스펙표는 118 이지만 **97 이 맞다** (2026-09-03 확인)
    자이로        바이어스 미보정 5.5도/분 -> 정지 2초 보정 후 1.0도/분
    직진 조각     3m 를 1m 씩 3조각이면 1.46배 느려진다 (측정 시간 포함)

━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━
아직 안 한 것
━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━━

    태그2 전환 시 tag_layout 으로 좌표 환산. 지금은 두 태그가 같은 수직면·
    같은 중심선에 있다고 보고 tag_id 만 바꾼다. 실제로 어긋나 있으면
    바뀌는 순간 lateral/forward 가 튄다. 실측 후 utils/tag_layout.py 로 환산할 것.

    비스듬히 접근. 지금은 "회전 -> 옆으로 -> 복귀" 3단계인데, 목표점을 향해
    한 번 돌고 한 번에 가는 쪽이 2~3배 빠르고 태그도 안 놓친다
    (lateral 0.08m / forward 3.0m 에서 21.4s vs 8.8s).
    지금 방식이 실제로 되는지 본 뒤에 재검토한다.

    ROT_LEAD_DEG 실측. rotate_by 가 회전마다 오버슈트를 재서 로그에 남긴다.
    몇 번 돌려 보고 그 평균을 config 에 넣으면 목표각을 더 정확히 맞춘다.
"""
import asyncio
import copy
import math

from config.system import (DOCK_EXTRA_M, FWD_ABORT_K, FWD_SAFETY, HEAD_TOL_DEG,
                       HOLD_RETRY_SEC, LAT_TOL_M, MAX_STEPS, TAG_SIZE_M,
                       TAG_CUT_MARGIN_PX, WARMUP_FRACTION,
                       MEASURE_FRAMES, MEASURE_MAX_FRAMES, ROT_DEG_PER_SEC,
                       ROT_LEAD_DEG, ROT_MAX_SEC, ROT_MIN_SEC, ROT_POLL_SEC,
                       ROT_SETTLE_MAX_SEC, ROT_SETTLE_MIN_SEC,
                       ROT_SETTLE_POLL_SEC, ROT_SETTLE_RATE_FLOOR,
                       ROT_SETTLE_RATE_K, ROT_T0_SEC, ROT_WATCHDOG_GAIN,
                       ROT_WRONG_WAY_DEG, SEARCH_AFTER_MISSES, SEARCH_BACKUP_M,
                       SEARCH_MAX_ROUNDS, SETTLE_SEC,
                       SIDESTEP_BACKWARD_GAIN_DEG, STEP_M, TAG_ID,
                       TAG2_ID)
from .fwd_time_model import fwd_sec_from_offset_piecewise

# ── 부호 약속 (실물 확정 2026-09-01) ────────────────────────────────────────
#
#   rotate_ccw  ->  heading 이 **증가**한다      (+ = 반시계 = 왼쪽)
#   rotate_cw   ->  heading 이 **감소**한다      (- = 시계 = 오른쪽)
#
# 카메라 쪽 절반은 실측으로 확인했다 (handspin 로그 + IMU 손 실험).
# 남은 절반은 실장비에서 눈으로 보면 된다:
#     rotate_ccw 명령을 보냈을 때 지게차가 **왼쪽(반시계)** 으로 도는가?
#     그렇다면 그대로 두면 되고, 오른쪽으로 돌면 **CanDriver.rotate_by 안의
#     movement 매핑("rotate_ccw" if deg > 0 else "rotate_cw") 두 곳만** 맞바꿔라.
#     plan_step 의 ccw/cw 를 바꾸면 IMU 목표 부호까지 같이 뒤집혀
#     wrong-way 정지만 반복된다. (뒤집힌 채 돌려도 rotate_by 가 반대 5도에서
#     알아채고 선다 — ROT_WRONG_WAY_DEG.)


# ── 시간 추정 (표시·워치독·폴백 전용 — 회전 실행에는 안 씀) ─────────────────

def rot_sec_from_deg(deg):
    """돌 각도[도] -> 예상 시간[s]. **회전 실행에는 안 쓴다.**

    회전 실행은 CanDriver.rotate_by() 가 IMU 를 보면서 한다(폐루프).
    이 함수는 ① 화면에 "예상 N초" 를 보여줄 때, ② 워치독 상한 계산,
    ③ IMU 가 없을 때의 비상 폴백 — 이 셋에만 남아 있다.
    ROT_T0_SEC / ROT_DEG_PER_SEC 가 미측정 가정값이라 정밀하지 않다.
    """
    d = abs(float(deg))
    if not d:
        return 0.0
    t = ROT_T0_SEC + d / ROT_DEG_PER_SEC      # 시동 지연 + 각도/각속도
    return max(ROT_MIN_SEC, min(ROT_MAX_SEC, t))


def rot_timeout_sec(deg):
    """회전 워치독 상한 [s]. **각도에 비례해서 늘어난다.**

    고정값이면 각속도가 가정보다 느릴 때 큰 회전이 매번 타임아웃으로 실패한다
    (15초 고정에 90도면 6도/s 밑에서 전부 실패). 예상 시간의 몇 배로 두면
    각속도를 몰라도 안전하다.
    """
    return max(ROT_MAX_SEC, rot_sec_from_deg(deg) * ROT_WATCHDOG_GAIN)


# ── 기하 ────────────────────────────────────────────────────────────────────

def _norm180(deg):
    """각도를 -180..180 으로 접는다. 예: 270 -> -90, -200 -> +160."""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


def _sidestep(lat, head):
    """옆이동 한 묶음을 정한다. -> (회전각[도], 거리[m], "forward"|"backward")

    ★ 숫자 예시로 이해하기:
        내가 오른쪽 300mm(lat=+0.3), heading +5도라면 —
        왼쪽으로 가야 하니 "왼쪽을 보는" heading 은 -90도.
          전진으로 가려면:  -90 - 5   = -95도 회전  (많이 돈다)
          후진으로 가려면:  -90+180-5 = +85도 회전  (적게 돈다)  <- 이걸 고른다
        +85도 돌면 heading 이 +90(왼쪽 등지고 오른쪽 봄)이 되고,
        거기서 후진하면 몸은 왼쪽으로 300mm 이동한다.

    회전이 작은 쪽을 고르는 이유 둘: ① 태그를 놓치는 시간이 짧다
    ② 짧은 회전은 관성(정지 오버슈트)도 작다. 이 규칙 덕에 회전이
    절대 90도를 안 넘는다. 복귀 회전은 어느 쪽이든 90도다.

    **가정: 전진과 후진 속도가 같다.** 이동 시간을 전진 모델(fwd_time_model)로
    같이 쓴다. 후진이 실제로 느리면 그만큼 덜 간다 — 재보고 다르면 고칠 것.
    SIDESTEP_BACKWARD_GAIN_DEG 를 올리면 그만큼 전진 쪽으로 치우친다
    (head 가 0 근처면 두 회전이 +-90 으로 붙어서 잡음에 선택이 뒤집히므로).
    """
    face = -90.0 if lat > 0 else 90.0        # 가야 할 방향을 "보는" heading
    turn_f = _norm180(face - head)            # 안 1: 그 방향을 보고 전진
    turn_b = _norm180(face + 180.0 - head)    # 안 2: 반대를 보고 후진
    if abs(turn_f) - abs(turn_b) > SIDESTEP_BACKWARD_GAIN_DEG:
        return turn_b, abs(lat), "backward"   # 후진 쪽 회전이 더 작다
    return turn_f, abs(lat), "forward"


def _search_move(step, half_fov_deg):
    """탐색 몇 번째 걸음 -> (회전각[도], 후진거리[m]). **순수 함수다.**

    한 걸음마다 멈춰서 다시 재므로, 한 번에 시야 절반(약 35도)씩만 돌린다.
    그래야 훑고 지나간 곳이 없다. 한 바퀴(원점 -> 왼쪽 끝 -> 오른쪽 끝 ->
    원점)를 다 돌고도 못 찾으면 뒤로 한 걸음 물러난다.

    **후진이 두 실종 원인을 한꺼번에 완화한다:**
        너무 가까워서 위로 잘렸다   -> 물러나면 세로 시야 안으로 들어온다
        좌우로 벗어났다             -> 물러나면 방위각 atan(lateral/거리) 이 준다
    그래서 원인을 판별할 필요가 없다.

        걸음  0   1   2   3   4   5   6   7   8
        회전 +35 +35 -35 -35 -35 -35 +35 +35  -      (누적 0->+70->-70->0)
        후진  -   -   -   -   -   -   -   -  0.5m
    """
    d = float(half_fov_deg)
    pattern = [+d, +d, -d, -d, -d, -d, +d, +d]     # 누적: 0 -> +2d -> -2d -> 0
    n = len(pattern) + 1                            # 마지막 한 걸음은 후진
    k = int(step) % n
    if k < len(pattern):
        return pattern[k], 0.0
    return 0.0, float(SEARCH_BACKUP_M)


def search_round_of(step, half_fov_deg=35.0):
    """탐색 몇 번째 걸음이 몇 번째 바퀴인가. SEARCH_MAX_ROUNDS 와 비교용."""
    return int(step) // 9


def _fmt_measure(m):
    """측정값 한 줄 요약. 무슨 값을 보고 그 명령을 냈는지 로그에 남긴다."""
    if not m:
        return "태그 안 보임"
    flag = ""
    if not m.get("reliable_angle", True):
        flag += "  [각도의심]"
    if not m.get("stable", True):
        flag += "  [불안정 %s]" % ", ".join(m.get("reasons", []))
    return ("lat %+7.1fmm   fwd %6.3fm   head %+6.1f도   tilt %4.1f도   n %2d%s"
            % (m["lateral"] * 1000, m["forward"], m["heading_deg"],
               m["tilt_deg"], m["n"], flag))


# ── 판단: 다음 동작 하나를 고른다 (이 파일의 두뇌) ──────────────────────────

def fwd_abort_deg(m, remaining_m):
    """전진 중 "이 각도를 넘으면 멈춰라" [도]. **상수가 아니라 계산이다.**

    d 만큼 직진하면 옆으로 d x sin(heading) 밀린다. 그게 허용치를 넘기 전에
    끊으면 되므로 asin(LAT_TOL_M / 남은거리) 가 곧 허용 각도다.
    멀수록 빡세진다 — 1도만 틀어져도 3m 가면 52mm 밀리기 때문이다.

    그런데 마침 멀수록 측정도 부정확해져서(3m 에서 1프레임 잡음 0.42도),
    기하 허용치만 쓰면 잡음에 걸려 계속 멈춘다. 그래서 잡음의 몇 배를
    하한으로 둔다. 잡음은 heading_sigma_deg 가 배치마다 알려준다.
    """
    geo = math.degrees(math.asin(min(1.0, LAT_TOL_M / max(remaining_m, 1e-6))))
    sig = (m or {}).get("heading_sigma_deg")
    floor = FWD_ABORT_K * sig if sig and math.isfinite(sig) else 0.0
    return max(geo, floor)


def plan_step(m, state=None):
    """측정값 하나 -> 다음에 할 동작 하나. **순수 함수다** (하드웨어가 필요 없다).

    m 은 measure() 의 반환값. state 는 호출하는 쪽이 들고 있는 것들:
        misses        **아무 태그도** 못 본 사이클이 몇 번 연속인지 (탐색 발동용)
        margin_px     태그가 화면 가장자리에서 몇 px 떨어져 있나 (잘리기 직전 판단)
        prev_forward  직전 사이클의 forward [m] (뒤로 가는 것 감지)
        warmed        맨 처음 다가가기를 마쳤나
        half_fov_deg  가로 시야 절반 (탐색 걸음 크기)

    돌려주는 것:
        (동작, 양, 명령시간[s], 이유)
        동작 = "forward" | "sidestep" | "rotate_ccw" | "rotate_cw"
             | "search" | "final" | "done" | "hold" | "lost"

    ★ 판단 우선순위 (위에서부터, 하나 걸리면 거기서 끝):
        0. 태그를 못 본 지 오래됐다  -> search   최종 안전망
        0'. 측정을 못 믿겠다         -> hold
        0". forward 가 커졌다        -> hold     뒤로 갔거나 오측정
        1. 방향이 틀어졌다           -> 제자리 회전 (전진 전에. 위치를 안 바꾸니 먼저)
        2. 아직 멀고 처음이다        -> forward  1/3 만 먼저 (가까울수록 정확해서)
        3. 옆으로 벗어났다           -> sidestep
        4. 태그가 잘리기 직전이다     -> final  (마지막 한 걸음 + DOCK_EXTRA_M, 그 뒤 끝)
        5. 그 외                    -> forward  남은 거리의 90% 씩

    ★ 멈출 거리를 숫자로 안 정한다.
        예전에는 STOP_M 상수를 뒀는데, 카메라·태그 높이를 바꾸면 안 따라와서
        실제로 틀린 값이 됐다(1.50m 인데 한계가 2.55m). 지금은 **화면에서
        태그가 가장자리에 얼마나 붙었나(margin_px)** 를 직접 본다. 장착
        pitch 든 높이 실측 오차든 전부 화면에 이미 반영되어 있다.

    ★ 한 번에 남은 거리의 90% 만 간다.
        전진은 개루프(시간만 주고 결과를 안 봄)이고 시간 모델이 완벽하지
        않다 — 속도가 5% 빠르면 3m 명령에 15cm 를 지나친다. 모자라게 가면
        다음 사이클에 또 가면 되지만, 지나치면 되돌리기 비싸다(후진 모델 없음).
    """
    st = state or {}
    misses = int(st.get("misses", 0))
    margin_px = st.get("margin_px")
    prev_fwd = st.get("prev_forward")
    warmed = bool(st.get("warmed", False))
    half_fov = float(st.get("half_fov_deg", 35.0))

    # 0) 최종 안전망 — 아무 태그도 오래 못 봤다
    if misses >= SEARCH_AFTER_MISSES:
        step = misses - SEARCH_AFTER_MISSES
        if search_round_of(step) >= SEARCH_MAX_ROUNDS:
            return ("lost", 0.0, 0.0,
                    "%d바퀴 찾아도 태그가 없다. 정지한다 — 사람이 봐야 한다"
                    % SEARCH_MAX_ROUNDS)
        turn, back = _search_move(step, half_fov)
        if back:
            return ("search", (0.0, back), fwd_sec_from_offset_piecewise(back),
                    "태그를 찾는다: %.1fm 후진 (시야를 넓힌다)" % back)
        return ("search", (turn, 0.0), rot_sec_from_deg(turn),
                "태그를 찾는다: %+.0f도 회전 (%d바퀴째)"
                % (turn, search_round_of(step) + 1))

    # 0') 측정을 못 믿으면 움직이지 않는다. 잘못 재고 움직이는 게 최악이라서.
    if m is None:
        return ("hold", 0.0, 0.0, "태그를 못 봤다")
    if not m["stable"]:
        return ("hold", 0.0, 0.0, "흔들린다: " + ", ".join(m["reasons"]))

    lat, fwd, head = m["lateral"], m["forward"], m["heading_deg"]

    # 0") 다가가는 중인데 멀어졌다 — 후진 명령이 잘못 갔거나 측정이 튄 것.
    #     그냥 두면 영영 못 닿으므로 멈추고 다시 잰다.
    if prev_fwd is not None and fwd > prev_fwd + LAT_TOL_M:
        return ("hold", 0.0, 0.0,
                "forward 가 늘었다 (%.2f -> %.2fm). 멈추고 다시 잰다" % (prev_fwd, fwd))

    # 1) 무엇을 하든 방향부터 편다. 틀어진 채 전진하면 그만큼 옆으로 밀리고
    #    (6도로 1.3m 가면 138mm), 전진 중 heading 감시에 바로 걸려 끊긴다.
    #    제자리 회전은 위치를 안 바꾸므로 먼저 해도 손해가 없다.
    if abs(head) > HEAD_TOL_DEG and m["reliable_angle"]:
        return ("rotate_ccw" if head < 0 else "rotate_cw", abs(head),
                rot_sec_from_deg(head), "heading %.1f도 를 지운다 (전진 전에)" % head)

    # 2) 그 다음은 옆정렬보다 다가가기가 먼저다.
    #    lateral 오차 = heading 오차 x 거리 라서, 멀리서 옆정렬을 하면 잡음을
    #    쫓는 꼴이다(5m 에서 lateral 이 +-314mm). 1/3 만 붙고 나서 재면 훨씬 정확하다.
    if not warmed and fwd > STEP_M:
        d = min(fwd * WARMUP_FRACTION, fwd * FWD_SAFETY)
        return ("forward", d, fwd_sec_from_offset_piecewise(d),
                "먼저 %.2fm 다가간다 (가까울수록 lateral 이 정확해서)" % d)

    # 3) 옆으로 벗어나 있으면 — 회전, 직진, 회전을 한 묶음으로
    if abs(lat) > LAT_TOL_M:
        if not m["reliable_angle"]:
            # 각도를 못 믿는 상태로 sidestep 하면 엉뚱한 방향으로 간다.
            return ("hold", 0.0, 0.0,
                    "각도를 못 믿는다 (tilt %.1f도). 비스듬한 자리로 옮겨서 다시 재라"
                    % m["tilt_deg"])
        turn, dist, way = _sidestep(lat, head)
        sec = (rot_sec_from_deg(turn) + fwd_sec_from_offset_piecewise(dist)
               + rot_sec_from_deg(90.0))     # 세 동작의 예상 시간 합 (표시용)
        return ("sidestep", (turn, dist, way), sec,
                "lateral %.0fmm 를 지운다 (%+.1f도 회전 -> %.0fmm %s -> 90도 복귀)"
                % (lat * 1000, turn, dist * 1000,
                   "전진" if way == "forward" else "후진"))

    # 4) 태그가 화면 가장자리에 붙었다 — 이 태그로 갈 수 있는 데까지 왔다.
    #    다음 태그가 있으면 호출하는 쪽이 이미 갈아탔을 것이다(보이는 순간 바꾼다).
    #    여기까지 왔다는 건 다음 태그가 없다는 뜻이라, 마지막 한 걸음을 간다.
    if margin_px is not None and margin_px < TAG_CUT_MARGIN_PX:
        d = fwd + DOCK_EXTRA_M
        if d <= 0.0:
            return ("done", 0.0, 0.0, "도착. forward %.2fm" % fwd)
        # "final" 은 forward 와 실행은 같지만 **끝내는 동작**이다.
        # 그냥 forward 로 돌려주면 루프가 계속 돌고, 그 뒤엔 태그가 안 보여서
        # 탐색으로 빠진다. 여기가 개루프 마지막 구간이라 짧을수록 좋다 —
        # 태그2를 낮게 붙일수록 짧아진다.
        return ("final", d, fwd_sec_from_offset_piecewise(d),
                "마지막 %.2fm (태그면 %.2fm + 여유 %.2fm). 그 뒤 정지"
                % (d, fwd, DOCK_EXTRA_M))

    # 5) 앞으로 — 남은 거리의 90% 씩. 태그를 보며 가니 매번 다시 잡는다.
    if fwd > LAT_TOL_M:
        d = min(fwd * FWD_SAFETY, STEP_M)
        return ("forward", d, fwd_sec_from_offset_piecewise(d),
                "남은 %.2fm 중 %.2fm 전진 (태그를 보며)" % (fwd, d))

    return ("done", 0.0, 0.0, "도착. forward %.2fm" % fwd)


# ── 실행: 동작 하나를 드라이버로 (dock / dock_live 공용) ────────────────────

async def _execute(driver, action, amount, sec):
    """plan_step 이 고른 동작 하나를 드라이버로 실행한다. 결과 dict 를 돌려준다.

    회전은 driver.rotate_by(각도) — IMU 폐루프. 직진은 시간 개루프 그대로.
    sidestep 은 **어느 회전이 실패해도 거기서 멈춘다** — 방향이 틀린 채로
    태그도 안 보이는 개루프 직진을 하는 게 최악이다. 멈춰만 있으면 다음
    측정 사이클이 다시 계획하고, 그래도 안 보이면 search 가 받는다.
    """
    ok, why = True, ""
    if action == "sidestep":
        turn, dist, way = amount
        r = await driver.rotate_by(turn)                       # ① 회전 작은 쪽으로
        if isinstance(r, dict) and not r.get("ok", True):
            await driver.stop()
            return {"ok": False, "reason": "회전 실패(%s)" % r.get("reason", "")}
        drive = driver.forward if way == "forward" else driver.backward
        await drive(fwd_sec_from_offset_piecewise(dist))       # ② |lateral| 만큼 이동
        # ③ 복귀는 반대 방향 90도 — 끝나면 태그를 다시 보고 heading 도 0
        r = await driver.rotate_by(-90.0 if turn > 0 else 90.0)
        if isinstance(r, dict) and not r.get("ok", True):
            # 여기서 실패하면 태그를 등지고 선다. 알려야 search 가 제대로 받는다.
            ok, why = False, "복귀 회전 실패(%s) — 태그를 등졌을 수 있다" % r.get("reason", "")
    elif action in ("forward", "final"):
        await driver.forward(sec)
    elif action == "search":
        turn, back = amount
        if back:
            await driver.backward(fwd_sec_from_offset_piecewise(back))
        else:
            r = await driver.rotate_by(turn)
            if isinstance(r, dict) and not r.get("ok", True):
                ok, why = False, "탐색 회전 실패(%s)" % r.get("reason", "")
    elif action == "rotate_ccw":       # plan_step 규약: ccw = heading 을 올린다 = +각
        r = await driver.rotate_by(abs(float(amount)))
        if isinstance(r, dict) and not r.get("ok", True):
            ok, why = False, "회전 실패(%s)" % r.get("reason", "")
    elif action == "rotate_cw":
        r = await driver.rotate_by(-abs(float(amount)))
        if isinstance(r, dict) and not r.get("ok", True):
            ok, why = False, "회전 실패(%s)" % r.get("reason", "")
    await driver.stop()
    return {"ok": ok, "reason": why}


# ── 루프: 측정-판단-실행을 반복 ─────────────────────────────────────────────

def _visible(res, tag_id):
    """이 프레임에서 그 태그가 쓸 만하게 보이나."""
    return (res.docking.get(tag_id) is not None
            and res.quality.get(tag_id, {}).get("ok", True))


def _margin_px(res, tag_id):
    """이 프레임에서 그 태그가 화면 가장자리에서 몇 px 떨어져 있나. 없으면 None.

    이 값이 0 에 가까워지면 곧 검출이 끊긴다 — 그때가 다음 태그로 넘기거나
    마지막 한 걸음을 갈 시점이다. 거리로 환산하지 않고 화면을 직접 보는 이유는
    장착 pitch·높이 실측 오차가 이미 화면에 반영되어 있어서다.
    """
    from ..detection.detection_tag import tag_edge_margin_px
    img = getattr(res, "image", None)
    shape = getattr(img, "shape", None)
    if shape is None:
        from config.system import COLOR_SIZE
        shape = (COLOR_SIZE[1], COLOR_SIZE[0])
    for d in getattr(res, "detections", []):
        if int(getattr(d, "tag_id", -1)) == int(tag_id):
            return tag_edge_margin_px(d, shape)
    return None


def _ours(res, tag_id):
    """이 프레임에 **우리가 쓰는 태그**가 하나라도 있나.

    res.docking 을 그냥 보면 안 된다 — 현장에 상관없는 tag36h11 인쇄물이
    하나라도 있으면 실종 판정이 영영 안 나서 탐색이 발동하지 않는다.
    """
    ids = [tag_id] if TAG2_ID is None else [tag_id, TAG2_ID]
    return any(t in res.docking for t in ids)


def _next_tag(res, current):
    """지금 쫓는 태그 말고 **다음 태그**가 보이면 그 번호. 아니면 None.

    태그1 이 화면에서 사라지기 전에 갈아탄다. 겹치는 구간이 넓어서
    (실측 배치에서 2.55~5.44m) 서두를 필요가 없다.
    """
    if TAG2_ID is None or current == TAG2_ID:
        return None
    return TAG2_ID if _visible(res, TAG2_ID) else None


def _half_fov_deg(pipe):
    """가로 시야 절반 [도]. 탐색 걸음 크기의 근거다.

    좁은 쪽(왼/오 중 작은 값)을 쓴다 — 그래야 한 걸음 돌 때 훑고 지나가는
    곳이 없다. 카메라가 주는 fx·cx 로 계산하므로 해상도를 바꿔도 따라온다.
    """
    from config.system import COLOR_SIZE
    from ..detection.image import fov_edges_deg, intrinsics_from_ref
    intr = getattr(pipe, "intr", None)
    if intr is None:
        intr = intrinsics_from_ref((COLOR_SIZE[1], COLOR_SIZE[0]))
    _, _, left, right = fov_edges_deg(intr)
    return float(min(left, right))


async def dock(pipe, driver, tag_id=None, max_steps=None, log=print):
    """멈춤 -> 측정 -> 동작 하나 -> 반복. (화면 없음)

    **async 인 이유**: control_forklift_v2 의 TX 루프(movement 10ms, control 5ms,
    heartbeat 200ms)가 같은 이벤트 루프에서 계속 돌아야 한다. 멈추면 수신기
    워치독이 물어서 지게차가 선다. measure() 는 30프레임 = 약 1초 블로킹이라
    to_thread 로 빼서 TX 가 안 끊기게 한다.

    driver 는 async forward/backward/stop(초) 에 더해 rotate_by(도) 를 갖춘 무엇이든.
    DryRunDriver 를 넣으면 CAN 없이 순서만 확인할 수 있다.
    멈출 거리는 안 받는다 — 태그가 화면에서 잘리기 직전까지 가고 거기서 끝낸다.
    """
    from ..detection.detection_pose import measure
    tag_id = TAG_ID if tag_id is None else tag_id
    max_steps = MAX_STEPS if max_steps is None else max_steps
    half_fov = _half_fov_deg(pipe)
    history, misses, i = [], 0, 0
    st = {"half_fov_deg": half_fov, "warmed": False, "prev_forward": None}

    # 탐색 걸음은 max_steps 에서 빼지 않는다 — max_steps 는 "도킹이 수렴하나"를
    # 보는 값이고, 탐색은 SEARCH_MAX_ROUNDS 가 따로 막는다. 같이 세면 탐색이
    # 잘려서 안전망이 제 역할을 못 한다.
    while i < max_steps:
        m = await asyncio.to_thread(measure, pipe, tag_id)     # 멈춰서 1초 측정
        misses = 0 if m else misses + 1
        st["misses"] = misses
        action, amount, sec, why = plan_step(m, st)
        if m:
            st["prev_forward"] = m["forward"]
        if action == "forward":
            st["warmed"] = True
        if action != "search":
            i += 1
        log("[%2d] %s" % (i, _fmt_measure(m)))
        log("     %-10s %s" % (action, why))
        history.append((action, amount, sec, why))

        if action in ("done", "lost"):
            await driver.stop()
            return history
        if action == "final":
            await _execute(driver, action, amount, sec)
            log("     도착")
            return history
        if action == "hold":
            await asyncio.sleep(HOLD_RETRY_SEC)
            continue
        r = await _execute(driver, action, amount, sec)        # 실행
        if not r["ok"]:
            log("     !! %s" % r["reason"])

    log("!! %d 단계를 넘겼다. 수렴하지 않는다" % max_steps)
    return history


async def dock_live(pipe, driver, tag_id=None, max_steps=None, log=print,
                    on_frame=None, n_frames=None, should_stop=None):
    """dock() 과 같은 일을 하되 **프레임을 계속 읽으며** 한다. (화면용, run.py 가 씀)

    dock() 은 measure() 안에서 1초, 명령 실행 중 몇 초씩 프레임을 안 읽는다.
    그동안 화면이 멈춘다. 여기서는 프레임 루프가 주인이고, 측정과 명령이
    그 안에서 상태(phase)로 돈다.

        on_frame(res, info)  프레임마다 불린다. 화면 그리는 쪽이 받는다.
            info = {"phase": "measure"|"command"|"search"|"done", "step": n,
                    "action": str, "why": str, "left": 남은 초, "n": 모은 프레임}
        should_stop()        프레임마다 확인하는 중단 스위치. True 를 돌려주면
            돌던 명령을 접고 정지 후 나간다 — run.py --show 의 ESC 가 이것이다.

    카메라는 한 프로세스만 열 수 있어서, 제어와 화면이 **한 프로세스**여야 한다.

    ★ 명령 중에도 프레임을 보는 덕에 탐색이 공짜로 붙는다 — 회전 명령을
      띄워 두고 매 프레임 태그를 확인하다가, 보이면 그 자리에서 명령을 끊는다.
    """
    from ..detection.detection_pose import measure
    tag_id = TAG_ID if tag_id is None else tag_id
    max_steps = MAX_STEPS if max_steps is None else max_steps
    n_frames = int(n_frames or MEASURE_FRAMES)
    half_fov = _half_fov_deg(pipe)
    st = {"half_fov_deg": half_fov, "warmed": False, "prev_forward": None}
    gen = iter(pipe)
    history, buf = [], []
    phase, step, task, info = "measure", 0, None, {}
    started = [0.0]
    waited = 0          # 이번 measure 단계에서 흘려보낸 프레임 수. 포기 판단용
    saw_any = False     # 이번 창에서 **아무 태그라도** 본 적이 있나
    misses = 0          # 그런 창이 몇 번 연속인지. 탐색 발동 기준
    margins = []        # 이번 창에서 본 화면 가장자리 여유 [px]
    abort = [None]      # 전진 중 heading 감시: 넘으면 안 되는 각 [도]

    def now():
        return asyncio.get_event_loop().time()

    async def run_action(action, amount, sec):
        return await _execute(driver, action, amount, sec)

    try:
        while True:
            item = await asyncio.to_thread(next, gen, None)
            if item is None:
                break
            i, ts, frame = item
            if frame is None or getattr(frame, "size", 1) == 0:
                continue
            res = pipe.process(frame, index=i, timestamp=ts)

            # 태그 전환 — 다음 태그가 보이면 갈아탄다. 어느 단계에서든 본다.
            nxt = _next_tag(res, tag_id)
            if nxt is not None and phase in ("measure", "search"):
                log("     태그%d -> 태그%d 로 갈아탄다" % (tag_id, nxt))
                tag_id, buf, waited, misses = nxt, [], 0, 0
                if phase == "search" and task is not None and not task.done():
                    task.cancel()
                phase = "measure"

            if phase == "search":
                # 탐색 중에는 프레임마다 확인만 한다. 보이면 즉시 명령을 끊는다.
                if _ours(res, tag_id):
                    log("     태그를 찾았다 — 탐색을 멈춘다")
                    if task is not None and not task.done():
                        task.cancel()
                        try:
                            await task
                        except asyncio.CancelledError:
                            pass
                    task = None
                    await driver.stop()
                    misses, waited, buf = 0, 0, []
                    phase = "measure"
                elif task is not None and task.done():
                    task = None
                    phase = "measure"       # 이 걸음 끝. 다시 재보고 또 찾는다

            elif phase == "measure":
                waited += 1
                if _ours(res, tag_id):
                    saw_any = True          # 창 안에서 한 번이라도 봤으면 실종이 아니다
                if _visible(res, tag_id):
                    # 이미지는 떼고 담는다 — 30프레임을 그대로 들면 1080p 에서 180MB 다
                    light = copy.copy(res)
                    light.image = None
                    buf.append(light)
                    mg = _margin_px(res, tag_id)
                    if mg is not None:
                        margins.append(mg)
                if len(buf) >= n_frames or waited >= MEASURE_MAX_FRAMES:
                    # n 을 같이 넘긴다 — 안 넘기면 measure 가 기본 30 으로 검사해서
                    # n_frames 를 줄였을 때 항상 '프레임 부족' 이 된다.
                    m = measure(buf, tag_id=tag_id, n=n_frames) if buf else None
                    # **아무 태그도** 안 보였을 때만 실종으로 센다.
                    # 쫓는 태그만 없는 것은 갈아타는 중일 수 있다.
                    # 창 전체를 보고 판단한다 — 마지막 한 프레임만 보면
                    # 태그가 깜빡였을 때 멀쩡한데도 실종으로 센다.
                    misses = 0 if saw_any else misses + 1
                    st["misses"] = misses
                    st["margin_px"] = (sorted(margins)[len(margins) // 2]
                                       if margins else None)
                    buf, waited, saw_any, margins = [], 0, False, []
                    action, amount, sec, why = plan_step(m, st)
                    if m:
                        st["prev_forward"] = m["forward"]
                    if action == "forward":
                        st["warmed"] = True
                        # 이 전진 동안 heading 이 이 각을 넘으면 끊는다.
                        abort[0] = fwd_abort_deg(m, m["forward"])
                    else:
                        abort[0] = None
                    # 탐색 걸음은 단계로 세지 않는다 (dock 과 같은 이유)
                    if action != "search":
                        step += 1
                    log("[%2d] %s" % (step, _fmt_measure(m)))
                    log("     %-10s %s" % (action, why))
                    history.append((action, amount, sec, why))
                    info = {"action": action, "why": why, "sec": sec}
                    if action in ("done", "lost"):
                        await driver.stop()
                        phase = "done"
                    elif action == "final":
                        task = asyncio.ensure_future(run_action(action, amount, sec))
                        started[0] = now()
                        phase = "final"      # 끝나면 done. 그 사이 화면은 계속 나간다
                    elif action == "hold":
                        pass
                    else:
                        task = asyncio.ensure_future(run_action(action, amount, sec))
                        started[0] = now()
                        phase = "search" if action == "search" else "command"
                    if step >= max_steps:
                        log("!! %d 단계를 넘겼다. 수렴하지 않는다" % max_steps)
                        phase = "done"

            elif phase == "final":
                if task is not None and task.done():
                    task = None
                    log("     도착")
                    await driver.stop()
                    phase = "done"

            elif phase == "command":
                # 전진 중에는 프레임마다 두 가지를 본다.
                #   ① heading — 1도만 틀어져도 3m 가면 52mm 밀린다
                #   ② 화면 여유 — 태그가 가장자리에 닿기 전에 끊어야 한다.
                #      안 보면 명령 한 번에 잘리는 지점을 지나쳐 태그를 잃는다
                #      (실제로 4.0m 에서 1.32m 명령이 한계 2.55m 를 넘어갔다).
                if abort[0] is not None and task is not None and not task.done():
                    why_cut = None
                    d = res.docking.get(tag_id)
                    if d is not None and abs(d["heading_deg"]) > abort[0]:
                        why_cut = ("heading %+.2f도 (허용 %.2f도)"
                                   % (d["heading_deg"], abort[0]))
                    mg = _margin_px(res, tag_id)
                    if why_cut is None and mg is not None and mg < TAG_CUT_MARGIN_PX:
                        why_cut = "태그가 화면 가장자리 %.0fpx (한계 %.0fpx)" % (mg, TAG_CUT_MARGIN_PX)
                    if why_cut is not None:
                        log("     !! 전진 중단 — %s" % why_cut)
                        task.cancel()
                        try:
                            await task
                        except asyncio.CancelledError:
                            pass
                        task, abort[0] = None, None
                        await driver.stop()
                        phase, waited = "measure", 0
                if task is not None and task.done():
                    exc = task.exception()
                    if exc is not None:
                        log("!! 명령 실행 예외: %r — 정지 후 재측정" % (exc,))
                        await driver.stop()
                    else:
                        r = task.result()
                        if isinstance(r, dict) and not r.get("ok", True):
                            log("     !! %s" % r["reason"])
                    task = None
                    phase = "measure"
                    waited = 0

            if on_frame is not None:
                left = 0.0
                if phase in ("command", "search"):
                    left = max(0.0, info.get("sec", 0.0) - (now() - started[0]))
                on_frame(res, dict(info, phase=phase, step=step, left=left,
                                   n=len(buf), need=n_frames, tag=tag_id))
            if should_stop is not None and should_stop():
                log("!! 중단 요청 — 정지한다")
                break
            if phase == "done":
                break
    finally:
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        await driver.stop()
    return history


# ── 명령을 보내는 층 (드라이버 두 가지) ─────────────────────────────────────

class DryRunDriver:
    """아무것도 안 보내고 찍기만 한다. CAN 하드웨어 없이 순서를 볼 때."""

    def __init__(self, log=print, realtime=False):
        self.log = log
        self.realtime = realtime      # True 면 명령 시간만큼 실제로 기다린다
        self.sent = []                # 보낸 척한 명령들이 쌓임 (테스트 확인용)

    async def _do(self, what, sec):
        self.sent.append((what, sec))
        self.log("       -> %-11s %5.2fs" % (what, sec))
        if self.realtime and sec:
            await asyncio.sleep(sec)

    async def forward(self, sec):
        await self._do("forward", sec)

    async def backward(self, sec):
        await self._do("backward", sec)

    async def rotate_by(self, deg):
        """각도 명령의 모의 실행. IMU 없이 시간 모델로 흉내만 낸다."""
        deg = float(deg)
        sec = rot_sec_from_deg(deg)
        self.sent.append(("rotate_by", deg))
        self.log("       -> rotate_by  %+7.1f도  (모의 %5.2fs)" % (deg, sec))
        if self.realtime and sec:
            await asyncio.sleep(sec)
        return {"target": deg, "turned": deg, "overshoot": 0.0,
                "ok": True, "reason": "dry-run"}

    async def stop(self):
        await self._do("stop", 0.0)


class CanDriver:
    """control_forklift_v2 의 컨트롤러에 명령을 태운다.

    그쪽 코드를 **한 줄도 안 고친다.** movement_tx_loop 이 10ms 마다
    MOVEMENT_TEMPLATES[self.current_movement] 를 보내고 있으므로,
    우리는 그 변수만 바꿨다가 되돌리면 된다. 키보드 루프가 하던 일과 같다.

        current_movement = "forward"   ->  N초 대기  ->  "stop"

    control_tx_loop / heartbeat_loop 는 그대로 돌아야 하므로
    이 코루틴에서 블로킹하면 안 된다(asyncio.sleep 만 쓴다).

    yaw 는 src/utils/imu_yaw.py 의 GyroYaw — rotate_by 의 눈이다.
    start()+calibrate() 까지 끝난 것을 받는다. **None 이면 회전이 시간 모델
    개루프로 떨어진다.** 그 시간 모델은 미측정 가정값이라 정상 경로가 아니다.
    run.py 가 붙여 준다.
    """

    def __init__(self, controller, yaw=None, log=None):
        self.c = controller           # control_forklift_v2 의 컨트롤러 객체
        self.yaw = yaw                # GyroYaw 계기판 (없으면 시간 폴백)
        self.log = log
        if yaw is not None and not getattr(yaw, "calibrated", False) and log:
            log("       !! GyroYaw 가 보정 전이다 — 바이어스가 0 이라 5.5도/분 흘러간다. "
                "정지 상태에서 calibrate() 를 부를 것")

    async def _hold(self, movement, sec):
        """movement 를 sec 초 유지했다가 stop 으로 되돌린다. 시간 기반 명령의 공통 몸통."""
        if self.log:
            self.log("       -> %-11s %5.2fs" % (movement, sec))
        self.c.current_movement = movement
        try:
            if sec:
                await asyncio.sleep(sec)
        finally:
            self.c.current_movement = "stop"   # 예외가 나도 반드시 선다
        await asyncio.sleep(SETTLE_SEC)        # 관성이 잦아들 시간. 재기 전에 확실히 선다

    async def forward(self, sec):
        await self._hold("forward", sec)

    async def backward(self, sec):
        await self._hold("backward", sec)

    def _settle_rate_dps(self):
        """"회전이 멎었다" 판정 문턱 [도/s].

        보정 때 잰 정지 잡음의 몇 배로 잡는다 — 장비·온도가 달라도 따라온다.
        보정 전이면 고정 하한을 쓴다.
        """
        n = getattr(self.yaw, "noise_dps", None) if self.yaw else None
        if not n:
            return ROT_SETTLE_RATE_FLOOR
        return max(ROT_SETTLE_RATE_FLOOR, n * ROT_SETTLE_RATE_K)

    async def rotate_by(self, deg, timeout=None):
        """deg 만큼 제자리 회전한다. +가 반시계 — heading 과 같은 부호. **폐루프.**

        ★ 동작을 말로 풀면:
            "rotate 명령을 켜 둔 채(고정 출력) IMU 계기판을 10ms 마다 흘끗
             보다가, 목표각에 닿으면 명령을 끈다. 그리고 관성으로 얼마나 더
             돌았는지 재서 로그로 남긴다."

        IMU 상대각을 보며 돌다가 |목표| - 리드각 에서 정지 명령을 낸다.
        관성이 잦아든 뒤 실회전각을 재서 로그로 남긴다 — 그 오버슈트가
        ROT_LEAD_DEG 를 정하는 실측값이다(회전마다 공짜로 쌓인다).

        지키는 것 넷. 어느 경우든 **멈추기만 하면** 다음 측정 사이클이 다시 계획한다:
            반대로 ROT_WRONG_WAY_DEG 넘게 돌면 즉시 정지 — CAN 부호 뒤집힘
            자이로가 IMU_STALE_SEC 넘게 끊기면 즉시 정지 — 눈 없이 안 돈다
            워치독(각도 비례) 넘으면 정지 — 스톨이거나 각속도 과대평가
            회전 중 자이로 샘플이 유실됐으면(gaps) 각도 과소집계라 실패로 강등

        돌려주는 것: {'target', 'turned', 'overshoot', 'ok', 'reason'}
        """
        deg = float(deg)
        if abs(deg) < 1e-9:
            return {"target": 0.0, "turned": 0.0, "overshoot": 0.0,
                    "ok": True, "reason": ""}

        if self.yaw is None:
            # IMU 없이 만든 드라이버. 미측정 시간 모델로 개루프 폴백.
            sec = rot_sec_from_deg(deg)
            if self.log:
                self.log("       -> rotate_by  %+7.1f도  (IMU 없음 — 시간모델 %.2fs 개루프)"
                         % (deg, sec))
            await self._hold("rotate_ccw" if deg > 0 else "rotate_cw", sec)
            return {"target": deg, "turned": None, "overshoot": None,
                    "ok": True, "reason": "time-fallback"}

        if not self.yaw.alive:
            # 계기판이 죽어 있으면 아예 출발하지 않는다. 눈 없이 돌면 못 세운다.
            if self.log:
                self.log("       !! rotate_by 거부 — 자이로가 %.1fs 째 안 온다. 안 돈다"
                         % self.yaw.age_sec)
            self.c.current_movement = "stop"
            return {"target": deg, "turned": 0.0, "overshoot": None,
                    "ok": False, "reason": "imu-stale"}

        loop = asyncio.get_event_loop()
        timeout = rot_timeout_sec(deg) if timeout is None else float(timeout)
        a0 = self.yaw.angle_deg               # 공유 계기판이라 zero() 대신 시작각을 뜬다
        gaps0 = self.yaw.stats().get("gaps", 0)   # 시작 시점의 유실 카운트 (끝에 대조)
        s = 1.0 if deg > 0 else -1.0          # 진행 방향 부호 (아래 prog 계산에 씀)
        # 리드각은 요청각의 절반까지만 — ROT_LEAD_DEG 를 실측치로 올린 뒤
        # 그보다 작은 회전(예: 2.5도)이 goal=0 무동작-성공이 되는 것을 막는다
        goal = abs(deg) - min(ROT_LEAD_DEG, abs(deg) / 2.0)
        if self.log:
            self.log("       -> rotate_by  %+7.1f도  (IMU 폐루프, 워치독 %.0fs)"
                     % (deg, timeout))

        ok, reason = True, ""
        t0 = loop.time()
        # ★ 여기서 회전이 시작된다 — TX 루프가 이 변수를 10ms 마다 CAN 으로 쏜다
        self.c.current_movement = "rotate_ccw" if deg > 0 else "rotate_cw"
        try:
            while True:
                await asyncio.sleep(ROT_POLL_SEC)        # TX 루프에 양보
                prog = s * (self.yaw.angle_deg - a0)     # +면 목표 쪽으로 간 각
                if prog >= goal:
                    break                                # 목표 도달 -> 정지
                if prog <= -ROT_WRONG_WAY_DEG:
                    ok, reason = False, "wrong-way"      # 반대로 돈다?! -> 즉시 정지
                    break
                if not self.yaw.alive:
                    ok, reason = False, "imu-stale"      # 계기판 끊김 -> 즉시 정지
                    break
                if loop.time() - t0 > timeout:
                    ok, reason = False, "timeout"        # 너무 오래 걸림 -> 정지
                    break
        finally:
            self.c.current_movement = "stop"   # 어떤 경로로 나가든 반드시 선다

        # 관성이 잦아들 때까지 기다렸다가 실회전각을 잰다 — 오버슈트를 로그에 남기려고.
        floor = self._settle_rate_dps()
        t1 = loop.time()
        while loop.time() - t1 < ROT_SETTLE_MAX_SEC:
            await asyncio.sleep(ROT_SETTLE_POLL_SEC)
            if (loop.time() - t1 >= ROT_SETTLE_MIN_SEC
                    and abs(self.yaw.rate_dps) < floor):
                break                          # 회전이 실제로 멎었다
        turned = self.yaw.angle_deg - a0
        over = s * turned - abs(deg)           # +면 목표를 지나쳤다
        gaps = self.yaw.stats().get("gaps", 0) - gaps0
        if ok and gaps > 0:
            # 유실 구간의 회전은 적분에서 빠졌다(과소집계) — 실기계는 계기판보다
            # 더 돌았을 수 있다. 성공으로 치면 sidestep 이 틀어진 채 실명 직진한다.
            ok, reason = False, "gyro-gaps"

        if self.log:
            if ok:
                self.log("          회전 끝: 목표 %+.1f도 -> 실제 %+.1f도 (오버슈트 %+.1f도)"
                         % (deg, turned, over))
            elif reason == "wrong-way":
                self.log("          !! 반대로 돌았다(%+.1f도) — CAN 회전 부호가 뒤집혔다. "
                         "**CanDriver.rotate_by 안의 movement 매핑 두 곳만** 맞바꿔라. "
                         "plan_step 을 바꾸면 IMU 목표 부호까지 뒤집혀 wrong-way 만 반복된다"
                         % turned)
            elif reason == "gyro-gaps":
                self.log("          !! 회전 중 자이로 %d구간 유실 — 각도를 과소집계했을 수 "
                         "있다. 정지하고 재측정" % gaps)
            else:
                self.log("          !! 회전 중단(%s): 목표 %+.1f도 중 %+.1f도에서 정지"
                         % (reason, deg, turned))
        return {"target": deg, "turned": turned, "overshoot": over,
                "ok": ok, "reason": reason}

    async def stop(self):
        self.c.current_movement = "stop"


__all__ = ["rot_sec_from_deg", "rot_timeout_sec", "fwd_abort_deg",
           "plan_step", "dock", "dock_live", "DryRunDriver", "CanDriver"]
