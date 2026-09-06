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

동작 세 묶음(Set):

    Set1  forward 만큼 직진.
    Set2  최소각 회전 -> lateral 만큼 이동 -> 반대방향 90도 복귀.
          90도 복귀가 끝나기 전까지는 태그가 안 보인다.
    Set3  태그가 안 보이면 반시계로 한 걸음씩 돈다, 보일 때까지.

    시작하면 IMU 를 2초 보정한 뒤(run.py), 태그가 보이면 Set2 -> Set1,
    안 보이면 Set3 로 찾은 뒤 Set2 -> Set1 순으로 간다.

파일 안의 부품 (위에서 아래 순서):

    1. plan_lateral_clear() / next_set3_step() / drive_distance()
                      Set2·Set3·직진의 기하·저수준 실행. 순수 계산 + 드라이버 호출
    2. plan_step()    두뇌. 측정값 하나 -> 다음 동작 하나 (순수 계산, 하드웨어 없음)
    3. _execute()     손. plan_step 이 고른 동작을 드라이버로 실제 실행
    4. dock/dock_live 루프. 측정-판단-실행을 반복
    5. CanDriver      지게차와의 연결. rotate_by() 는 rot_control.rotate_to() 를 부른다

    IMU 계기판(GyroYaw)은 src/utils/imu_yaw.py 에 있다. 여기서 만들지 않고
    run.py 가 만들어서 CanDriver 에 넣어 준다.
    회전 시간 추정(표시·워치독 전용)은 src/models/control/rot_control.py 에 있다.

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

    Set3 는 회전만 한다(후진 없음). 태그가 너무 가까워서 위로 잘린 경우는
    돌아도 안 보인다 — 그 자리에선 계속 돈다. 실제로 나면 후진을 다시 넣을 것.
"""
import asyncio
import copy
import math

from config.control import (DOCK_EXTRA_M, FWD_ABORT_K, FWD_SAFETY,
                            HEAD_TOL_DEG, HOLD_RETRY_SEC, LAT_TOL_M, MAX_STEPS,
                            SEARCH_AFTER_MISSES, SEARCH_BACKUP_M,
                            SEARCH_MAX_ROUNDS, SETTLE_SEC,
                            SIDESTEP_BACKWARD_GAIN_DEG, STEP_M,
                            TAG_CUT_MARGIN_PX, WARMUP_FRACTION)
from config.detection import (MAX_HEADING_SIGMA_DEG, MEASURE_FRAMES,
                              MEASURE_MAX_FRAMES, TAG_ID, TAG_SIZE_M, TAG2_ID)
from ...utils.event_log import record_event
from .fwd_time_model import fwd_sec_from_offset_piecewise
from .rot_control import rot_sec_from_deg, rot_timeout_sec, rotate_to

# ── 부호 약속 (실물 확정 2026-09-01) ────────────────────────────────────────
#
#   rotate_ccw  ->  heading 이 **증가**한다      (+ = 반시계 = 왼쪽)
#   rotate_cw   ->  heading 이 **감소**한다      (- = 시계 = 오른쪽)
#
# 카메라 쪽 절반은 실측으로 확인했다 (handspin 로그 + IMU 손 실험).
# 남은 절반은 실장비에서 눈으로 보면 된다:
#     rotate_ccw 명령을 보냈을 때 지게차가 **왼쪽(반시계)** 으로 도는가?
#     그렇다면 그대로 두면 되고, 오른쪽으로 돌면 **rot_control.rotate_to() 안의
#     movement 매핑("rotate_ccw" if deg > 0 else "rotate_cw") 두 곳만** 맞바꿔라.
#     plan_step 의 ccw/cw 를 바꾸면 IMU 목표 부호까지 같이 뒤집혀
#     wrong-way 정지만 반복된다. (뒤집힌 채 돌려도 반대 5도에서 알아채고 선다.)


# ── 기하 ────────────────────────────────────────────────────────────────────

def normalize_deg(deg):
    """각도를 -180..180 으로 접는다. 예: 270 -> -90, -200 -> +160."""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


def drive_distance(driver, distance_m, direction):
    """distance_m 만큼 전진("forward") 또는 후진("backward"). Set1·Set2 공용."""
    seconds = fwd_sec_from_offset_piecewise(distance_m)
    return driver.forward(seconds) if direction == "forward" else driver.backward(seconds)


def plan_lateral_clear(lateral_m, heading_deg):
    """Set2 — lateral_m 을 지우는 최소각 회전을 고른다.
    -> (회전각[도], 이동거리[m], "forward"|"backward")

    lateral 이 +면 오른쪽에 있다는 뜻이라 왼쪽을 보는 heading(-90도) 을
    보고 가거나, 반대(+90도) 를 보고 후진해도 같은 곳에 닿는다. 회전이
    작은 쪽을 고른다 — 태그를 놓치는 시간과 정지 오버슈트가 둘 다 작다.
    이 규칙 덕에 회전이 절대 90도를 안 넘는다. 복귀 회전은 항상 90도.

    가정: 전진과 후진 속도가 같다(전진 시간 모델을 같이 쓴다). 후진이
    실제로 느리면 그만큼 덜 간다 — 재보고 다르면 SIDESTEP_BACKWARD_GAIN_DEG
    를 올려 전진 쪽으로 치우치게 할 것.
    """
    face_heading = -90.0 if lateral_m > 0 else 90.0
    turn_forward = normalize_deg(face_heading - heading_deg)
    turn_backward = normalize_deg(face_heading + 180.0 - heading_deg)
    if abs(turn_forward) - abs(turn_backward) > SIDESTEP_BACKWARD_GAIN_DEG:
        return turn_backward, abs(lateral_m), "backward"
    return turn_forward, abs(lateral_m), "forward"


def next_set3_step(half_fov_deg):
    """Set3 — 태그가 안 보이면 반시계로 한 걸음 돈다. 걸음마다 다시 재서 확인한다."""
    return float(half_fov_deg)


def set3_rounds_done(rotations_done, half_fov_deg):
    """Set3 걸음 수 -> 몇 바퀴(360도)째인가. SEARCH_MAX_ROUNDS 와 비교용."""
    steps_per_round = max(1, round(360.0 / half_fov_deg))
    return int(rotations_done) // steps_per_round


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
    """측정값 하나 -> 다음 동작 하나. 순수 함수 (하드웨어 없이 계산만).

    m 은 measure() 의 반환값. state 는 호출하는 쪽이 들고 있는 것들:
        misses            아무 태그도 못 본 사이클 수 (Set3 발동 기준)
        margin_px         태그가 화면 가장자리에서 몇 px 떨어져 있나
        prev_forward      직전 사이클의 forward [m] (뒤로 가는 것 감지)
        warmed            웜업(1/3 접근)을 마쳤나 — 기본 비활성
        half_fov_deg      가로 시야 절반 (Set3 걸음 크기)
        drove_forward     직전 동작이 forward/final 이었나 (recover_backup 발동 조건)
        backed_up_once    이번 실종에서 recover_backup 을 이미 써봤나

    돌려주는 것: (동작, 양, 명령시간[s], 이유)
        동작 = "forward" | "sidestep" | "rotate_ccw" | "rotate_cw" | "search"
             | "recover_backup" | "final" | "done" | "hold" | "lost"

    전체 순서:
        전진 직후 실종 -> recover_backup(딱 한 번 후진)
        -> Set3(그래도 못 봄 -> 반시계 회전, 다 돌아도 못 찾으면 lost=수동전환)
        -> 정지 판정(못 믿음 / 뒤로 감)
        -> [웜업, 기본 꺼짐]
        -> Set2(lateral 지우기: 최소각 회전 -> 이동 -> 반대방향 90도 복귀. heading 도 같이 지워짐)
        -> heading 만 남았으면 마저 편다 (Set1 전에)
        -> 마무리(태그 잘리기 직전이면 final)
        -> Set1(forward 만큼 직진, 90% 씩 조각내서)

    heading 을 lateral 보다 먼저 독립적으로 지우면 안 된다 — lateral 이 큰
    상태에서 heading 만 0으로 만들면 카메라가 태그와 나란한 방향을 보게 되어
    태그를 놓친다(실측: lateral 3m 에서 재현됨). Set2 가 둘을 같이 풀어야 한다.

    recover_backup 과 Set3 는 트리거가 다르다 — recover_backup 은 "방금까지
    보이다가 전진 직후 막 사라짐"(방위는 아니 후진이 맞다), Set3 는 "여러
    사이클 연속으로 아예 안 보임"(방위를 모르니 회전으로 찾는다). 후진해도
    안 보이면 backed_up_once 가 True 로 남아 recover_backup 은 다시 안
    나오고, misses 만 쌓여 그대로 Set3 로 넘어간다.
    """
    st = state or {}
    misses = int(st.get("misses", 0))
    margin_px = st.get("margin_px")
    prev_forward = st.get("prev_forward")
    warmed = bool(st.get("warmed", False))
    half_fov = float(st.get("half_fov_deg", 35.0))
    drove_forward = bool(st.get("drove_forward", False))
    backed_up_once = bool(st.get("backed_up_once", False))

    # 전진 직후 막 실종 — Set3 로 넘기기 전에 딱 한 번 후진해서 다시 본다.
    if misses >= 1 and drove_forward and not backed_up_once:
        return ("recover_backup", SEARCH_BACKUP_M,
                fwd_sec_from_offset_piecewise(SEARCH_BACKUP_M),
                "전진 중 태그를 놓쳤다. %.1fm 후진해서 다시 본다" % SEARCH_BACKUP_M)

    # Set3 — 태그를 오래 못 봤다. 반시계로 한 걸음씩 돌며 찾는다.
    if misses >= SEARCH_AFTER_MISSES:
        rotations_done = misses - SEARCH_AFTER_MISSES
        if set3_rounds_done(rotations_done, half_fov) >= SEARCH_MAX_ROUNDS:
            return ("lost", 0.0, 0.0,
                    "%d바퀴 찾아도 태그가 없다. 수동전환 — 사람이 확인해야 한다"
                    % SEARCH_MAX_ROUNDS)
        turn = next_set3_step(half_fov)
        return ("search", turn, rot_sec_from_deg(turn),
                "Set3: 반시계 %.0f도 회전 (%d바퀴째)"
                % (turn, set3_rounds_done(rotations_done, half_fov) + 1))

    if m is None:
        return ("hold", 0.0, 0.0, "태그를 못 봤다")
    if not m["stable"]:
        return ("hold", 0.0, 0.0, "흔들린다: " + ", ".join(m["reasons"]))

    lateral_m, forward_m, heading_deg = m["lateral"], m["forward"], m["heading_deg"]

    if prev_forward is not None and forward_m > prev_forward + LAT_TOL_M:
        return ("hold", 0.0, 0.0,
                "forward 가 늘었다 (%.2f -> %.2fm). 멈추고 다시 잰다"
                % (prev_forward, forward_m))

    # 웜업(1/3 먼저 접근) — 기본 비활성. 켜려면 아래 두 줄의 주석을 지운다.
    # lateral 오차 = heading 오차 x 거리라, 멀리서 Set2 를 하면 잡음을 쫓는
    # 꼴이다(5m 에서 lateral +-314mm). 가까이 붙고 나서 재면 훨씬 정확하다.
    # if not warmed and forward_m > STEP_M:
    #     approach_m = min(forward_m * WARMUP_FRACTION, forward_m * FWD_SAFETY)
    #     return ("forward", approach_m, fwd_sec_from_offset_piecewise(approach_m),
    #             "웜업: %.2fm 먼저 다가간다" % approach_m)

    # Set2 — 옆으로 벗어나 있으면 회전-이동-복귀 한 묶음. heading 도 여기서 같이
    # 지워진다(plan_lateral_clear 가 둘을 같이 본다) — lateral 이 클 때 heading 만
    # 따로 0으로 만들면 카메라가 태그와 나란한 방향을 보게 되어 태그를 놓친다.
    if abs(lateral_m) > LAT_TOL_M:
        if not m["reliable_angle"]:
            # **다가가는 것이 곧 해결이다.** 각도 잡음은 거리의 제곱으로 줄어든다
            # (30cm 태그 기준 5m 1.49도 -> 3m 0.49도 -> 2m 0.27도). 못 믿는 각도로
            # Set2 를 하면 잡음을 쫓아 lateral 을 오히려 키우므로, 먼저 붙는다.
            #
            # 여기서 멈춰 서면(옛 동작) 재도 값이 같아서 영원히 hold 였다 —
            # 자율주행인데 "비스듬한 자리로 옮겨서 다시 재라"고 할 대상이 없다.
            # 실제로 계획한 시작 거리(2.5~5.9m)가 전부 그 구간이었다.
            sigma = m.get("heading_sigma_deg")
            if forward_m > STEP_M:
                approach_m = min(forward_m * WARMUP_FRACTION, forward_m * FWD_SAFETY,
                                 STEP_M)
                return ("forward", approach_m,
                        fwd_sec_from_offset_piecewise(approach_m),
                        "각도 잡음 %.2f도 (한계 %.2f도) — %.2fm 다가가서 다시 잰다"
                        % (sigma if sigma is not None else float("nan"),
                           MAX_HEADING_SIGMA_DEG, approach_m))
            # 이만큼 붙었는데도 못 믿으면 거리 탓이 아니다 (가림·흔들림·조명).
            return ("hold", 0.0, 0.0,
                    "%.2fm 까지 붙었는데도 각도 잡음 %.2f도 (한계 %.2f도) — "
                    "가림·조명·진동을 의심하라"
                    % (forward_m, sigma if sigma is not None else float("nan"),
                       MAX_HEADING_SIGMA_DEG))
        turn, distance_m, direction = plan_lateral_clear(lateral_m, heading_deg)
        estimated_sec = (rot_sec_from_deg(turn) + fwd_sec_from_offset_piecewise(distance_m)
                         + rot_sec_from_deg(90.0))
        return ("sidestep", (turn, distance_m, direction), estimated_sec,
                "Set2: lateral %.0fmm -> %+.1f도 회전 -> %.0fmm %s -> 90도 복귀"
                % (lateral_m * 1000, turn, distance_m * 1000,
                   "전진" if direction == "forward" else "후진"))

    # lateral 은 이미 됐는데 heading 만 남았다 — Set1(전진) 전에 편다.
    if abs(heading_deg) > HEAD_TOL_DEG and m["reliable_angle"]:
        return ("rotate_ccw" if heading_deg < 0 else "rotate_cw", abs(heading_deg),
                rot_sec_from_deg(heading_deg), "heading %.1f도 를 지운다" % heading_deg)

    # 태그가 화면 가장자리에 붙었다 — 다음 태그가 없다는 뜻이니 마지막 한 걸음.
    if margin_px is not None and margin_px < TAG_CUT_MARGIN_PX:
        final_m = forward_m + DOCK_EXTRA_M
        if final_m <= 0.0:
            return ("done", 0.0, 0.0, "도착. forward %.2fm" % forward_m)
        return ("final", final_m, fwd_sec_from_offset_piecewise(final_m),
                "마지막 %.2fm (forward %.2fm + 여유 %.2fm). 그 뒤 정지"
                % (final_m, forward_m, DOCK_EXTRA_M))

    # Set1 — 앞으로. 남은 거리의 90% 씩 조각내서, 태그를 보며 매번 다시 잰다.
    if forward_m > LAT_TOL_M:
        step_m = min(forward_m * FWD_SAFETY, STEP_M)
        return ("forward", step_m, fwd_sec_from_offset_piecewise(step_m),
                "Set1: 남은 %.2fm 중 %.2fm 전진" % (forward_m, step_m))

    return ("done", 0.0, 0.0, "도착. forward %.2fm" % forward_m)


# ── 실행: 동작 하나를 드라이버로 (dock / dock_live 공용) ────────────────────

async def _execute(driver, action, amount, sec):
    """plan_step 이 고른 동작 하나를 드라이버로 실행한다. 결과 dict 를 돌려준다.

    sidestep(Set2) 은 회전이 실패하면 거기서 멈춘다 — 방향이 틀린 채로
    태그도 안 보이는 개루프 직진을 하는 게 최악이다. 멈춰만 있으면 다음
    측정 사이클이 다시 계획하고, 그래도 안 보이면 Set3(search) 가 받는다.
    """
    ok, why = True, ""
    if action == "sidestep":
        turn, distance_m, direction = amount
        result = await driver.rotate_by(turn)
        if isinstance(result, dict) and not result.get("ok", True):
            await driver.stop()
            return {"ok": False, "reason": "회전 실패(%s)" % result.get("reason", "")}
        await drive_distance(driver, distance_m, direction)
        result = await driver.rotate_by(-90.0 if turn > 0 else 90.0)
        if isinstance(result, dict) and not result.get("ok", True):
            ok, why = False, "복귀 회전 실패(%s) — 태그를 등졌을 수 있다" % result.get("reason", "")
    elif action in ("forward", "final"):
        await drive_distance(driver, amount, "forward")
    elif action == "recover_backup":
        await drive_distance(driver, amount, "backward")
    elif action == "search":
        result = await driver.rotate_by(amount)
        if isinstance(result, dict) and not result.get("ok", True):
            ok, why = False, "탐색 회전 실패(%s)" % result.get("reason", "")
    elif action == "rotate_ccw":       # plan_step 규약: ccw = heading 을 올린다 = +각
        result = await driver.rotate_by(abs(float(amount)))
        if isinstance(result, dict) and not result.get("ok", True):
            ok, why = False, "회전 실패(%s)" % result.get("reason", "")
    elif action == "rotate_cw":
        result = await driver.rotate_by(-abs(float(amount)))
        if isinstance(result, dict) and not result.get("ok", True):
            ok, why = False, "회전 실패(%s)" % result.get("reason", "")
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
        from config.detection import COLOR_SIZE
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
    from config.detection import COLOR_SIZE
    from ..detection.image import fov_edges_deg, intrinsics_from_ref
    intr = getattr(pipe, "intr", None)
    if intr is None:
        intr = intrinsics_from_ref((COLOR_SIZE[1], COLOR_SIZE[0]))
    _, _, left, right = fov_edges_deg(intr)
    return float(min(left, right))


async def dock(pipe, driver, tag_id=None, max_steps=None, log=print, record_dir=None):
    """멈춤 -> 측정 -> 동작 하나 -> 반복. (화면 없음)

    **async 인 이유**: control_forklift_v2 의 TX 루프(movement 10ms, control 5ms,
    heartbeat 200ms)가 같은 이벤트 루프에서 계속 돌아야 한다. 멈추면 수신기
    워치독이 물어서 지게차가 선다. measure() 는 30프레임 = 약 1초 블로킹이라
    to_thread 로 빼서 TX 가 안 끊기게 한다.

    driver 는 async forward/backward/stop(초) 에 더해 rotate_by(도) 를 갖춘 무엇이든.
    DryRunDriver 를 넣으면 CAN 없이 순서만 확인할 수 있다.
    멈출 거리는 안 받는다 — 태그가 화면에서 잘리기 직전까지 가고 거기서 끝낸다.
    record_dir 를 주면 측정값도 기록한다 (그 폴더 안에 measure.jsonl 로,
    드라이버의 rotation/drive 기록과 같은 폴더에 종류별 파일로 나뉜다).
    """
    from ..detection.detection_pose import measure
    tag_id = TAG_ID if tag_id is None else tag_id
    max_steps = MAX_STEPS if max_steps is None else max_steps
    half_fov = _half_fov_deg(pipe)
    history, misses, i = [], 0, 0
    st = {"half_fov_deg": half_fov, "warmed": False, "prev_forward": None,
          "drove_forward": False, "backed_up_once": False}

    # 탐색 걸음은 max_steps 에서 빼지 않는다 — max_steps 는 "도킹이 수렴하나"를
    # 보는 값이고, 탐색은 SEARCH_MAX_ROUNDS 가 따로 막는다. 같이 세면 탐색이
    # 잘려서 안전망이 제 역할을 못 한다.
    while i < max_steps:
        m = await asyncio.to_thread(measure, pipe, tag_id)     # 멈춰서 1초 측정
        if m:
            record_event(record_dir, "measure", tag_id=tag_id, **m)
            misses, st["drove_forward"], st["backed_up_once"] = 0, False, False
        else:
            misses += 1
        st["misses"] = misses
        action, amount, sec, why = plan_step(m, st)
        if m:
            st["prev_forward"] = m["forward"]
        if action in ("forward", "final"):
            st["warmed"], st["drove_forward"] = True, True
        elif action == "recover_backup":
            st["backed_up_once"] = True
        elif action != "hold":
            st["drove_forward"] = False
        if action != "search":
            i += 1
        log("[%2d] %s" % (i, _fmt_measure(m)))
        log("     %-10s %s" % (action, why))
        history.append((action, amount, sec, why))
        record_event(record_dir, "decision", step=i, action=action, why=why,
                     misses=misses, margin_px=st.get("margin_px"))

        if action in ("done", "lost"):
            await driver.stop()
            record_event(record_dir, "result",
                         outcome="manual" if action == "lost" else "done", steps=i)
            return history
        if action == "final":
            await _execute(driver, action, amount, sec)
            log("     도착")
            record_event(record_dir, "result", outcome="done", steps=i)
            return history
        if action == "hold":
            await asyncio.sleep(HOLD_RETRY_SEC)
            continue
        r = await _execute(driver, action, amount, sec)        # 실행
        if not r["ok"]:
            log("     !! %s" % r["reason"])

    log("!! %d 단계를 넘겼다. 수렴하지 않는다" % max_steps)
    record_event(record_dir, "result", outcome="max_steps", steps=max_steps)
    return history


async def dock_live(pipe, driver, tag_id=None, max_steps=None, log=print,
                    on_frame=None, n_frames=None, should_stop=None,
                    record_dir=None):
    """dock() 과 같은 일을 하되 **프레임을 계속 읽으며** 한다. (화면용, run.py 가 씀)

    record_dir 를 주면 측정값을 그 폴더의 measure.jsonl 에 남긴다 — 드라이버의
    rotation/drive 기록과 같은 폴더, 종류별 파일. ts 로 시간순 병합이 된다.

    dock() 은 measure() 안에서 1초, 명령 실행 중 몇 초씩 프레임을 안 읽는다.
    그동안 화면이 멈춘다. 여기서는 프레임 루프가 주인이고, 측정과 명령이
    그 안에서 상태(phase)로 돈다.

        on_frame(res, info)  프레임마다 불린다. 화면 그리는 쪽이 받는다.
            info = {"phase": "measure"|"command"|"search"|"final"|"done"|"manual",
                    "step": n, "action": str, "why": str, "left": 남은 초, "n": 모은 프레임}
            "manual" 은 Set3 가 다 돌아도 못 찾아 수동전환된 상태 — "done"(정상
            도착)과 구분해서 화면에 다르게 보여야 한다.
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
    st = {"half_fov_deg": half_fov, "warmed": False, "prev_forward": None,
          "drove_forward": False, "backed_up_once": False}
    gen = iter(pipe)
    history, buf = [], []
    phase, step, task, info = "measure", 0, None, {}
    started = [0.0]
    waited = 0          # 이번 measure 단계에서 흘려보낸 프레임 수. 포기 판단용
    saw_any = False     # 이번 창에서 **아무 태그라도** 본 적이 있나
    misses = 0          # 그런 창이 몇 번 연속인지. 탐색 발동 기준
    outcome = "incomplete"   # result 기록용. 정상 종료 경로마다 덮어쓴다
    margins = []        # 이번 창에서 본 화면 가장자리 여유 [px]
    abort = [None]      # 전진 중 heading 감시: 넘으면 안 되는 각 [도]

    def now():
        return asyncio.get_event_loop().time()

    async def run_action(action, amount, sec):
        return await _execute(driver, action, amount, sec)

    try:
        while True:
            # TagPipeline 순회는 **Result 를 뱉는다** (3-튜플이 아니다 —
            # 3-튜플은 pipe.frames 쪽 계약이다). 검출까지 이 스레드 안에서
            # 끝나므로 이벤트 루프가 안 막힌다 — CAN TX 루프(5~10ms)와
            # heartbeat 가 검출 시간만큼 밀리면 지게차가 정지 판정을 낸다.
            res = await asyncio.to_thread(next, gen, None)
            if res is None:
                break
            if res.image is None or getattr(res.image, "size", 1) == 0:
                continue

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
                    if m:
                        record_event(record_dir, "measure", tag_id=tag_id, **m)
                    # **아무 태그도** 안 보였을 때만 실종으로 센다.
                    # 쫓는 태그만 없는 것은 갈아타는 중일 수 있다.
                    # 창 전체를 보고 판단한다 — 마지막 한 프레임만 보면
                    # 태그가 깜빡였을 때 멀쩡한데도 실종으로 센다.
                    if saw_any:
                        misses, st["drove_forward"], st["backed_up_once"] = 0, False, False
                    else:
                        misses += 1
                    st["misses"] = misses
                    st["margin_px"] = (sorted(margins)[len(margins) // 2]
                                       if margins else None)
                    buf, waited, saw_any, margins = [], 0, False, []
                    action, amount, sec, why = plan_step(m, st)
                    if m:
                        st["prev_forward"] = m["forward"]
                    if action in ("forward", "final"):
                        st["drove_forward"] = True
                    if action == "forward":
                        st["warmed"] = True
                        # 이 전진 동안 heading 이 이 각을 넘으면 끊는다.
                        abort[0] = fwd_abort_deg(m, m["forward"])
                    else:
                        abort[0] = None
                    if action == "recover_backup":
                        st["backed_up_once"] = True
                    # 탐색 걸음은 단계로 세지 않는다 (dock 과 같은 이유)
                    if action != "search":
                        step += 1
                    log("[%2d] %s" % (step, _fmt_measure(m)))
                    log("     %-10s %s" % (action, why))
                    history.append((action, amount, sec, why))
                    record_event(record_dir, "decision", step=step, action=action,
                                 why=why, misses=misses, margin_px=st.get("margin_px"))
                    info = {"action": action, "why": why, "sec": sec}
                    if action == "lost":
                        await driver.stop()
                        outcome = "manual"
                        phase = "manual"        # 수동전환. done 과 구분해서 화면에 다르게 보인다
                    elif action == "done":
                        await driver.stop()
                        outcome = "done"
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
                        outcome = "max_steps"
                        phase = "done"

            elif phase == "final":
                if task is not None and task.done():
                    task = None
                    log("     도착")
                    await driver.stop()
                    outcome = "done"
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
                        record_event(record_dir, "abort", step=step, why=why_cut)
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
                outcome = "user_stop"
                break
            if phase in ("done", "manual"):
                break
    finally:
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        await driver.stop()
        # 어떻게 끝났는지 한 줄. "incomplete" 는 예외로 튕겨 나갔다는 뜻이다.
        record_event(record_dir, "result", outcome=outcome, steps=step)
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

    record_dir 를 주면 회전·직진마다 결과를 그 폴더의 rotation.jsonl /
    drive.jsonl 에 남긴다(utils/event_log.record_event) — 나중에 ROT_T0/
    ROT_DEG_PER_SEC/ROT_LEAD_DEG 를 실측 적합할 데이터다. 기록 실패가
    도킹을 막지는 않는다(event_log 가 삼킴).
    """

    def __init__(self, controller, yaw=None, log=None, record_dir=None):
        self.c = controller           # control_forklift_v2 의 컨트롤러 객체
        self.yaw = yaw                # GyroYaw 계기판 (없으면 시간 폴백)
        self.log = log
        self.record_dir = record_dir
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
        record_event(self.record_dir, "drive", movement=movement, sec=sec)

    async def forward(self, sec):
        await self._hold("forward", sec)

    async def backward(self, sec):
        await self._hold("backward", sec)

    async def rotate_by(self, deg, timeout=None):
        """deg 만큼 제자리 회전한다. +가 반시계. 실제 알고리즘은 rot_control.rotate_to()."""
        return await rotate_to(self.c, self.yaw, deg, log=self.log, timeout=timeout,
                               record=lambda result: record_event(
                                   self.record_dir, "rotation", **result))

    async def stop(self):
        self.c.current_movement = "stop"


__all__ = ["rot_sec_from_deg", "rot_timeout_sec", "fwd_abort_deg",
           "drive_distance", "plan_lateral_clear", "plan_step",
           "dock", "dock_live", "DryRunDriver", "CanDriver"]
