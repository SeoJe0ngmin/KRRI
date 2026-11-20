"""도킹값 + IMU 회전각 -> 주행 명령.  (v2: 조준-전진, lateral 을 직접 안 잡는다)

한 사이클: [측정] 30프레임 -> [판단] plan_step() -> [실행] 동작 하나 -> 반복.
설계 이유·부호 약속·용어·실측 기록은 전부 ../../../../code_explanation.md 에 있다.
"""
import asyncio
import collections
import copy
import math
import statistics

from config import control as _CFG
from config import detection as D
from ...utils.event_log import record_event
from ..detection.detection_pose import MEASURE_FRAMES, MEASURE_MAX_FRAMES
from .fwd_time_model import fwd_sec_from_offset_piecewise
from .rot_control import rot_sec_from_deg, rot_timeout_sec, rotate_to

# ── v2 전용 상수 (2026-09-21 config/control.py 에서 옮겨 왔다) ───────────────
# plan 대문제 2·3 "상수 회계" 가 전부 **삭제 대상**으로 결론 낸 값들이다. v3 실행경로
# (`tools/dock.py` → dock_fsm/estimate/dynamics)는 하나도 읽지 않는다 — 계약 §4.7 이
# 그렇게 못 박았다. 지우지 않고 여기로 내린 이유: `tools/run.py`(v2 폴백)가 내일
# 새 코드가 이상할 때의 퇴로라서 그대로 돌아야 한다. config 에는 "현장에서 사람이
# 정하는 값" 만 남긴다(CLAUDE.md 상수 최소화).
STEP_M = 1.0                   # 직진 한 조각 [m]
FWD_SAFETY = 0.9               # 명령 거리 = 남은 거리 x 이 값 (모자라게 가는 쪽이 안전)
SIDESTEP_BACKWARD_GAIN_DEG = 0.0   # 0 = 회전이 작은 쪽을 고른다
TAG_CUT_MARGIN_PX = 60.0       # 잘리기 직전 문턱 [px]
MAX_STEPS = 30                 # 넘기면 수렴 실패 (탐색 걸음은 안 셈)
HOLD_MAX_CONSEC = 8            # hold 가 이만큼 연속이면 수동전환 (약 10초)
SETTLE_SEC = 0.8               # 명령 끊은 뒤 실정지까지 대기 [s]
SEARCH_BACKUP_M = 0.5          # 전진 중 관성으로 인한 실종 직후 후진 거리 [m]
SEARCH_AFTER_MISSES = 3        # 연속 미검출 이만큼이면 Set3 시작
SEARCH_MAX_ROUNDS = 3          # 1바퀴째=연속 360도, 2바퀴째부터=반화각 걸음
CAM_TO_PIVOT_M = 1.46          # 카메라→제자리 회전 중심 [m]. ★철회된 값 — 9/7 18:14 로그
                               # 13회 회전의 Δβ/Δψ 0.77~0.95 와 안 맞는다(|A| ≲ 0.4 또는
                               # 자이로 스케일 오차). v3 는 회전팔 A 를 실시간 도출한다
AIM_STANDOFF_M = 3.5           # T 까지 거리 [m] (카메라 기준)
AIM_NEAR_T_M = 1.0             # T 까지 이 안이면 조준 회전을 더 안 한다 [m]
AIM_CHUNK_MAX_M = 3.0          # 조준 뒤 한 번에 달리는 최대 거리 [m]
AIM_MAX_BEARING_DEG = 45.0     # T 방위각이 이보다 크면 90도 사이드스텝(v1 규칙) 폴백
AIM_MAX_TAG_OFF_DEG = 25.0     # 조준 뒤 태그가 코에서 이보다 벗어나면 폴백
AIM_TOL_DEG = 3.0              # 조준 오차 허용 [도]
AIM_AT_T_M = 0.3               # T 도착으로 보는 거리 [m]
AIM_FINAL_MAX_LAT_M = 0.10     # T 에서 lateral 이 이 이하면 태그를 직접 겨냥
AIM_FINAL_TOL_DEG = 1.5        # 태그 겨냥 각 오차 허용 [도]
AIM_BACKUP_M = 1.5             # T 근처인데 lateral 이 크면 이만큼 후진해 재조준 [m]
AIM_MAX_BACKUPS = 2            # 후진-재조준 최대 횟수
AIM_ABORT_WINDOW = 30          # 직진 중 heading 중단 판정 창 [프레임]
AIM_DRIFT_ABORT_DEG = 4.0      # 조준 직진 중 출발 heading 에서 이만큼 흘렀으면 중단 [도]
AIM_STOP_LEAD_M = 0.15         # 카메라 조기 정지 리드 [m]
AIM_STOP_CONFIRM = 3           # 조기 정지 판정 연속 프레임 수
#: 태그면을 지나 더 갈 거리 [m] = −2.02. 값은 config 의 둘에서 나온다(plan 4-5 분리)
DOCK_EXTRA_M = -(_CFG.CAM_TO_REF_M + _CFG.STANDOFF_M)


class _C:
    """옛 `C.XXX` 표기를 그대로 두기 위한 얇은 이름공간.

    config 에 남은 값(LAT_TOL_M 등)은 config 로 넘기고, 위에서 옮겨 온 v2 전용
    값은 이 모듈 것을 쓴다. 한 줄도 안 고치고 상수만 옮기려고 이렇게 했다.
    """
    _moved = {k: v for k, v in list(globals().items())
              if k.isupper() and not k.startswith("_")}

    def __getattr__(self, name):
        try:
            return self._moved[name]
        except KeyError:
            return getattr(_CFG, name)


C = _C()


def normalize_deg(deg):
    """각도를 -180..180 으로 접는다. 예: 270 -> -90, -200 -> +160."""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


def drive_distance(driver, distance_m, direction):
    """distance_m 만큼 전진("forward") 또는 후진("backward"). Set1·Set2 공용."""
    seconds = fwd_sec_from_offset_piecewise(distance_m)
    return driver.forward(seconds) if direction == "forward" else driver.backward(seconds)


def plan_lateral_clear(lateral_m, heading_deg):
    """Set2 — lateral_m 을 지우는 최소각 회전을 고른다."""
    face_heading = -90.0 if lateral_m > 0 else 90.0
    turn_forward = normalize_deg(face_heading - heading_deg)
    turn_backward = normalize_deg(face_heading + 180.0 - heading_deg)
    if abs(turn_forward) - abs(turn_backward) > C.SIDESTEP_BACKWARD_GAIN_DEG:
        return turn_backward, abs(lateral_m), "backward"
    return turn_forward, abs(lateral_m), "forward"


def next_set3_step(half_fov_deg, rotations_done=0):
    """Set3 걸음 크기. 첫 바퀴는 연속 360도 — 회전 중에도 dock_live 가
    프레임마다 태그를 보다가 보이면 즉시 회전을 끊으므로, 돌면서 찾는다.
    (걸음마다 서던 옛 방식은 걸음 사이 빈 측정창 5초 x 10걸음이 낭비였다.)

    연속 회전이 실패하는 유일한 경우는 모션블러로 못 본 것 — 그래서
    두 바퀴째부터는 반화각씩 끊어 "블러 없는 깨끗한 한 번" 을 보장한다.
    """
    return 360.0 if int(rotations_done) == 0 else float(half_fov_deg)


def set3_rounds_done(rotations_done, half_fov_deg):
    """Set3 걸음 수 -> 몇 바퀴째인가. 1바퀴 = 연속 1걸음, 이후 = 반화각 걸음들."""
    r = int(rotations_done)
    if r <= 0:
        return 0
    steps_per_round = max(1, round(360.0 / half_fov_deg))
    return 1 + (r - 1) // steps_per_round


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


def fwd_abort_deg(step_m):
    """전진 중 "이 각도를 넘으면 멈춰라" [도] — 이번 걸음에서 30mm 새는 각."""
    return math.degrees(math.asin(min(1.0, C.LAT_TOL_M / max(step_m, 1e-6))))


def plan_aim(lateral_m, forward_m, heading_deg, margin_px, st):
    """v2 조준-전진. 측정 하나 -> 동작 하나. 순수 함수.

    부호: lateral + 는 태그가 오른쪽(트럭이 축의 왼쪽), heading/회전 + 는 반시계.
    계산은 전부 **회전 중심(뒷바퀴)** 기준으로 한다 — 카메라는 회전 중심에서
    CAM_TO_PIVOT_M 앞에 있어 제자리 회전만 해도 lateral 이 A*sin(각) 만큼 변한다
    (18:14 로그에서 T 근처 회전 6번 되풀이의 원인). 회전 중심의 lateral/forward 는
    회전해도 안 변하므로 조준이 흔들리지 않는다.
        lat_p = lateral - A*sin(h),  fwd_p = forward + A*cos(h)
    T 는 축 위, 회전 중심 기준으로 태그에서 S + A 앞 (heading 0 이면 카메라가 S 에 온다).
    """
    S, A = C.AIM_STANDOFF_M, C.CAM_TO_PIVOT_M
    h = math.radians(heading_deg)
    lat_p = lateral_m - A * math.sin(h)
    fwd_p = forward_m + A * math.cos(h)
    dx = fwd_p - (S + A)                                  # 회전 중심에서 T 까지 축 방향 거리
    bearing = math.degrees(math.atan2(-lat_p, dx))       # T 방위각, 반시계 +
    dist_t = math.hypot(lat_p, dx)
    st["aim_bearing"], st["aim_dist_t"], st["aim_lat_p"] = bearing, dist_t, lat_p
    tag_cut = margin_px is not None and margin_px < C.TAG_CUT_MARGIN_PX
    near_t = dist_t <= C.AIM_NEAR_T_M

    if dist_t <= C.AIM_AT_T_M or dx <= C.AIM_AT_T_M or tag_cut or (near_t and abs(lat_p) <= C.AIM_FINAL_MAX_LAT_M):
        # T 도착(또는 태그가 화면 위로 나가기 직전, 또는 T 1m 안에서 lateral 이 대충 맞음).
        # 순서: lateral -> heading -> 마지막 직진. 정렬 전에 직진하면 틀어진 채 들어간다.
        if C.LAT_TOL_M < abs(lat_p) <= C.AIM_FINAL_MAX_LAT_M:
            # 조금 벗어남: 축과 나란히 서는 대신 태그를 직접 겨냥해 들어간다. 회전 중심이 태그를
            # 향하면 카메라·포크도 그 선 위에 있다. 도착 heading 은 atan(lat/fwd) 로 작다.
            h_star = math.degrees(math.atan2(-lat_p, fwd_p))
            turn = normalize_deg(h_star - heading_deg)
            if abs(turn) > C.AIM_FINAL_TOL_DEG:
                return ("rotate_ccw" if turn > 0 else "rotate_cw", abs(turn), rot_sec_from_deg(turn),
                        "정렬(태그 겨냥): 회전중심 lateral %.0fmm -> 목표 heading %+.1f도, 지금 %+.1f도 -> %+.1f도 회전"
                        % (lat_p * 1000, h_star, heading_deg, turn))
            final_m = math.hypot(lat_p, fwd_p) - A + C.DOCK_EXTRA_M
            if final_m <= 0.0:
                return ("done", 0.0, 0.0, "도착. forward %.2fm" % forward_m)
            return ("final", final_m, fwd_sec_from_offset_piecewise(final_m),
                    "태그 겨냥 끝(heading %+.1f도). 마지막 %.2fm. 그 뒤 정지" % (heading_deg, final_m))
        if abs(lat_p) > C.AIM_FINAL_MAX_LAT_M:
            # 많이 벗어남: 여기서 lateral 은 조준으로 못 고친다(방위각이 90도에 가깝다). 물러나서 다시.
            if int(st.get("aim_backups", 0)) >= C.AIM_MAX_BACKUPS:
                return ("lost", 0.0, 0.0,
                        "T 에서 lateral %.0fmm 이 %d번 후진해도 안 맞는다. 수동전환"
                        % (lat_p * 1000, C.AIM_MAX_BACKUPS))
            return ("aim_backup", C.AIM_BACKUP_M, fwd_sec_from_offset_piecewise(C.AIM_BACKUP_M),
                    "T 근처인데 회전중심 lateral %.0fmm — %.1fm 후진해 다시 조준"
                    % (lat_p * 1000, C.AIM_BACKUP_M))
        if abs(heading_deg) > C.HEAD_TOL_DEG:
            return ("rotate_ccw" if heading_deg < 0 else "rotate_cw", abs(heading_deg),
                    rot_sec_from_deg(heading_deg),
                    "정렬: T 에서 heading %+.1f도 를 지운다" % heading_deg)
        final_m = forward_m + C.DOCK_EXTRA_M
        if final_m <= 0.0:
            return ("done", 0.0, 0.0, "도착. forward %.2fm" % forward_m)
        return ("final", final_m, fwd_sec_from_offset_piecewise(final_m),
                "정렬 끝. 마지막 %.2fm (forward %.2fm + 여유 %.2fm). 그 뒤 정지"
                % (final_m, forward_m, C.DOCK_EXTRA_M))

    tag_dir = math.degrees(math.atan2(-lateral_m, forward_m))      # 카메라에서 본 태그 방향 (축 기준)
    tag_off = abs(normalize_deg(tag_dir - bearing))                  # 조준한 뒤 태그가 코에서 몇 도 옆에
    if abs(bearing) > C.AIM_MAX_BEARING_DEG or tag_off > C.AIM_MAX_TAG_OFF_DEG:
        # 태그 옆/뒤에서 시작 — 조준으로 가면 태그가 화면 가장자리에 걸린다. v1 사이드스텝 폴백
        turn, distance_m, direction = plan_lateral_clear(lateral_m, heading_deg)
        estimated_sec = (rot_sec_from_deg(turn) + fwd_sec_from_offset_piecewise(distance_m)
                         + rot_sec_from_deg(90.0))
        return ("sidestep", (turn, distance_m, direction), estimated_sec,
                "폴백 Set2 (T 방위각 %.0f도, 조준 뒤 태그 %.0f도 옆): lateral %.0fmm -> %+.1f도 회전 -> %.0fmm %s -> 90도 복귀"
                % (abs(bearing), tag_off, lateral_m * 1000, turn,
                   distance_m * 1000, "전진" if direction == "forward" else "후진"))

    turn = normalize_deg(bearing - heading_deg)
    if abs(turn) > C.AIM_TOL_DEG:
        return ("rotate_ccw" if turn > 0 else "rotate_cw", abs(turn), rot_sec_from_deg(turn),
                "조준: T 방위각 %+.1f도, heading %+.1f도 -> %+.1f도 회전 (회전중심 T 까지 %.2fm, lateral %.0fmm)"
                % (bearing, heading_deg, turn, dist_t, lat_p * 1000))

    d = min(C.AIM_CHUNK_MAX_M, dist_t)
    b = math.radians(bearing)
    # 회전 중심이 d 만큼 가면 카메라(방위각 방향으로 A 앞)의 forward 는 이만큼 남는다
    stop_forward = fwd_p - d * math.cos(b) - A * math.cos(b)
    return ("aim_drive", (d, bearing, stop_forward), fwd_sec_from_offset_piecewise(d),
            "조준 직진 %.2fm (회전중심 T 까지 %.2fm, 방위각 %+.1f도). 카메라 forward %.2fm 에서 조기 정지"
            % (d, dist_t, bearing, stop_forward))

def plan_step(m, state=None):
    """측정값 하나 -> 다음 동작 하나. 순수 함수 (하드웨어 없이 계산만)."""
    st = state or {}
    misses = int(st.get("misses", 0))
    holds = int(st.get("holds", 0))
    margin_px = st.get("margin_px")
    prev_forward = st.get("prev_forward")
    half_fov = float(st.get("half_fov_deg", 35.0))
    drove_forward = bool(st.get("drove_forward", False))
    backed_up_once = bool(st.get("backed_up_once", False))


    if misses >= 1 and drove_forward and not backed_up_once:
        return ("recover_backup", C.SEARCH_BACKUP_M,
                fwd_sec_from_offset_piecewise(C.SEARCH_BACKUP_M),
                "전진 중 태그를 놓쳤다. %.1fm 후진해서 다시 본다" % C.SEARCH_BACKUP_M)


    if misses >= C.SEARCH_AFTER_MISSES:
        rotations_done = misses - C.SEARCH_AFTER_MISSES
        if set3_rounds_done(rotations_done, half_fov) >= C.SEARCH_MAX_ROUNDS:
            return ("lost", 0.0, 0.0,
                    "%d바퀴 찾아도 태그가 없다. 수동전환 — 사람이 확인해야 한다"
                    % C.SEARCH_MAX_ROUNDS)
        turn = next_set3_step(half_fov, rotations_done)
        label = ("연속 1바퀴 — 보이면 즉시 정지" if turn >= 360.0
                 else "반시계 %.0f도 걸음 (블러 없는 확인)" % turn)
        return ("search", turn, rot_sec_from_deg(turn),
                "Set3: %s (%d바퀴째)"
                % (label, set3_rounds_done(rotations_done, half_fov) + 1))


    if holds >= C.HOLD_MAX_CONSEC:
        return ("lost", 0.0, 0.0,
                "%d번 연속 멈춰 있다 (마지막 이유: %s). 수동전환 — 사람이 확인해야 한다"
                % (holds, st.get("hold_why") or "?"))

    if m is None:
        return ("hold", 0.0, 0.0, "태그를 못 봤다")


    lateral_m, forward_m, heading_deg = m["lateral"], m["forward"], m["heading_deg"]

    # forward 가 늘었으면 잘못 간 것 — 단, 직전 동작이 직진일 때만 본다. 회전 뒤에는
    # 카메라가 뒷바퀴 축을 중심으로 호를 그려 forward 가 몇 cm 늘어나는 게 정상이다
    # (2026-09-07 로그: 90도 사이드스텝 뒤 +0.05~0.26m 로 매번 hold 한 스텝 낭비).
    if (prev_forward is not None and st.get("last_action") in ("forward", "final", "aim_drive")
            and forward_m > prev_forward + C.LAT_TOL_M):
        return ("hold", 0.0, 0.0,
                "forward 가 늘었다 (%.2f -> %.2fm). 멈추고 다시 잰다"
                % (prev_forward, forward_m))

    return plan_aim(lateral_m, forward_m, heading_deg, margin_px, st)


async def _execute(driver, action, amount, sec):
    """plan_step 이 고른 동작 하나를 드라이버로 실행한다. 결과 dict 를 돌려준다."""
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
    elif action == "aim_drive":
        d, _bearing, _stop = amount
        await drive_distance(driver, d, "forward")
    elif action == "aim_backup":
        await drive_distance(driver, amount, "backward")
    elif action == "recover_backup":
        await drive_distance(driver, amount, "backward")
    elif action == "search":
        result = await driver.rotate_by(amount)
        if isinstance(result, dict) and not result.get("ok", True):
            ok, why = False, "탐색 회전 실패(%s)" % result.get("reason", "")
    elif action == "rotate_ccw":
        result = await driver.rotate_by(abs(float(amount)))
        if isinstance(result, dict) and not result.get("ok", True):
            ok, why = False, "회전 실패(%s)" % result.get("reason", "")
    elif action == "rotate_cw":
        result = await driver.rotate_by(-abs(float(amount)))
        if isinstance(result, dict) and not result.get("ok", True):
            ok, why = False, "회전 실패(%s)" % result.get("reason", "")
    await driver.stop()
    return {"ok": ok, "reason": why}


def _visible(res, tag_id):
    """이 프레임에서 그 태그가 쓸 만하게 보이나."""
    return (res.docking.get(tag_id) is not None
            and res.quality.get(tag_id, {}).get("ok", True))


def _margin_px(res, tag_id):
    """이 프레임에서 그 태그가 화면 가장자리에서 몇 px 떨어져 있나. 없으면 None."""
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
    """이 프레임에 우리가 쫓는 태그가 있나."""
    return tag_id in res.docking


def _half_fov_deg(pipe):
    """가로 시야 절반 [도]. 탐색 걸음 크기의 근거다."""
    from config.detection import COLOR_SIZE
    from ..detection.image import fov_edges_deg, intrinsics_from_ref
    intr = getattr(pipe, "intr", None)
    if intr is None:
        intr = intrinsics_from_ref((COLOR_SIZE[1], COLOR_SIZE[0]))
    _, _, left, right = fov_edges_deg(intr)
    return float(min(left, right))
async def dock_live(pipe, driver, tag_id=None, max_steps=None, log=print,
                    on_frame=None, n_frames=None, should_stop=None,
                    record_dir=None):
    """도킹 루프 — 프레임을 계속 읽으며 측정-판단-실행을 반복한다."""
    from ..detection.detection_pose import measure
    tag_id = D.TAG_ID if tag_id is None else tag_id
    max_steps = C.MAX_STEPS if max_steps is None else max_steps
    n_frames = int(n_frames or MEASURE_FRAMES)
    half_fov = _half_fov_deg(pipe)
    st = {"half_fov_deg": half_fov, "prev_forward": None,
          "drove_forward": False, "backed_up_once": False}
    gen = iter(pipe)
    history, buf = [], []
    phase, step, task, info = "measure", 0, None, {}
    started = [0.0]
    waited = 0
    saw_any = False
    misses = 0
    outcome = "incomplete"
    margins = []
    abort = [None]
    abort_hits = 0      # 전진 중 heading 이 문턱을 연속 몇 프레임 넘었나
    aim_heading, aim_stop, stop_hits = 0.0, None, 0   # 직진의 기준 heading / 조기정지 forward
    head_win = collections.deque(maxlen=int(C.AIM_ABORT_WINDOW))   # 직진 중 heading 중앙값용 창

    def now():
        return asyncio.get_event_loop().time()

    async def run_action(action, amount, sec):
        return await _execute(driver, action, amount, sec)

    try:
        while True:


            res = await asyncio.to_thread(next, gen, None)
            if res is None:
                break
            if res.image is None or getattr(res.image, "size", 1) == 0:
                continue



            if phase == "search":

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
                    exc = task.exception()
                    r = None if exc is not None else task.result()
                    task = None
                    if exc is not None:
                        log("!! 탐색 회전 예외: %r — 정지, 수동전환" % (exc,))
                        await driver.stop()
                        outcome, phase = "manual", "manual"
                    elif isinstance(r, dict) and not r.get("ok", True):
                        log("     !! 탐색 회전 실패(%s)" % r.get("reason"))
                        if r.get("reason") == "imu-stale":
                            # IMU 가 죽었다 — 탐색이 제자리 헛돌기만 한다
                            await driver.stop()
                            outcome, phase = "manual", "manual"
                        else:
                            phase = "measure"
                    else:
                        phase = "measure"

            elif phase == "measure":
                waited += 1
                if _ours(res, tag_id):
                    saw_any = True
                if _visible(res, tag_id):

                    light = copy.copy(res)
                    light.image = None
                    buf.append(light)
                    mg = _margin_px(res, tag_id)
                    if mg is not None:
                        margins.append(mg)
                if len(buf) >= n_frames or waited >= MEASURE_MAX_FRAMES:


                    m = measure(buf, tag_id=tag_id, n=n_frames) if buf else None
                    if m:
                        record_event(record_dir, "measure", tag_id=tag_id, **m)


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
                    if action in ("forward", "final", "aim_drive"):
                        st["drove_forward"] = True
                    aim_heading, aim_stop, stop_hits = 0.0, None, 0
                    head_win.clear()
                    if action == "forward":
                        # 문턱은 "이번에 실제로 달릴 거리" 기준이어야 한다 — 남은
                        # 전체 거리로 재면 planner 가 방금 용인한 heading(<=허용치)이
                        # 곧바로 중단을 부르는 사각지대가 생긴다.
                        abort[0] = max(fwd_abort_deg(amount), C.HEAD_TOL_DEG)
                    elif action == "aim_drive":
                        # 비스듬히 달리는 동안 heading 은 방위각 근처가 정상. 출발 때 잰
                        # heading 에서 얼마나 흘렀나로 본다(조준 잔차는 다음 조각이 고친다).
                        # 카메라 forward 가 목표에 닿으면 시간 모델보다 먼저 정지한다.
                        _d, _bearing, aim_stop = amount
                        aim_heading = float((m or {}).get("heading_deg", _bearing))
                        abort[0] = max(fwd_abort_deg(_d), C.HEAD_TOL_DEG, C.AIM_DRIFT_ABORT_DEG)
                    else:
                        abort[0] = None
                    abort_hits = 0
                    if action == "aim_backup":
                        st["aim_backups"] = int(st.get("aim_backups", 0)) + 1
                    st["last_action"] = action
                    if action == "recover_backup":
                        st["backed_up_once"] = True
                    if action == "hold":
                        st["holds"] = st.get("holds", 0) + 1
                        st["hold_why"] = why
                    else:
                        st["holds"] = 0


                    if action != "search":
                        step += 1
                    log("[%2d] %s" % (step, _fmt_measure(m)))
                    log("     %-10s %s" % (action, why))
                    history.append((action, amount, sec, why))
                    record_event(record_dir, "decision", step=step, action=action,
                                 why=why, misses=misses, margin_px=st.get("margin_px"),
                                 stable=(m or {}).get("stable"),
                                 reasons=(m or {}).get("reasons"),
                                 bearing_deg=st.get("aim_bearing"), dist_t=st.get("aim_dist_t"), lat_pivot=st.get("aim_lat_p"))
                    info = {"action": action, "why": why, "sec": sec}
                    if action == "lost":
                        await driver.stop()
                        outcome = "manual"
                        phase = "manual"
                    elif action == "done":
                        await driver.stop()
                        outcome = "done"
                        phase = "done"
                    elif action == "final":
                        task = asyncio.ensure_future(run_action(action, amount, sec))
                        started[0] = now()
                        phase = "final"
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
                    exc = task.exception()
                    task = None
                    await driver.stop()
                    if exc is not None:
                        log("!! 마지막 직진 예외: %r — 수동전환" % (exc,))
                        outcome, phase = "manual", "manual"
                    else:
                        log("     도착")
                        outcome, phase = "done", "done"

            elif phase == "command":


                if abort[0] is not None and task is not None and not task.done():
                    why_cut = None
                    d = res.docking.get(tag_id)
                    # heading 은 원시 1프레임 값(잡음 +-0.45도 실측)이라 연속
                    # 3프레임을 요구한다. 화면 여유는 기하라 즉시 끊는다.
                    # heading 은 원시 1프레임 값이 10m 에서 ±2도 흔들린다. 최근 창의
                    # 중앙값이 문턱을 3번 연속 넘어야 끊는다 (창이 차기 전엔 판정 안 함).
                    dev = None
                    if d is not None:
                        head_win.append(float(d["heading_deg"]))
                        if len(head_win) >= head_win.maxlen:
                            dev = normalize_deg(statistics.median(head_win) - aim_heading)
                    if dev is not None and abs(dev) > abort[0]:
                        abort_hits += 1
                        if abort_hits >= 3:
                            why_cut = ("heading 중앙값 %+.2f도 (기준 %+.1f, 허용 %.2f도, %d프레임 창)"
                                       % (statistics.median(head_win), aim_heading, abort[0], head_win.maxlen))
                    elif dev is not None:
                        abort_hits = 0
                    reached = False
                    if d is not None and aim_stop is not None and why_cut is None:
                        if d["forward"] <= aim_stop + C.AIM_STOP_LEAD_M:
                            stop_hits += 1
                            if stop_hits >= C.AIM_STOP_CONFIRM:
                                reached = True
                                why_cut = ("카메라 forward %.2fm <= 목표 %.2fm + %.2f — 조기 정지"
                                           % (d["forward"], aim_stop, C.AIM_STOP_LEAD_M))
                        else:
                            stop_hits = 0
                    mg = _margin_px(res, tag_id)
                    if why_cut is None and mg is not None and mg < C.TAG_CUT_MARGIN_PX:
                        why_cut = "태그가 화면 가장자리 %.0fpx (한계 %.0fpx)" % (mg, C.TAG_CUT_MARGIN_PX)
                    if why_cut is not None:
                        if reached:
                            log("     %s" % why_cut)
                            record_event(record_dir, "stop", step=step, why=why_cut)
                        else:
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

        record_event(record_dir, "result", outcome=outcome, steps=step)
    return history


class DryRunDriver:
    """아무것도 안 보내고 찍기만 한다. CAN 하드웨어 없이 순서를 볼 때."""

    def __init__(self, log=print, realtime=False):
        self.log = log
        self.realtime = realtime
        self.sent = []

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
    """control_forklift_v2 의 컨트롤러에 명령을 태운다."""

    def __init__(self, controller, yaw=None, log=None, record_dir=None):
        self.c = controller
        self.yaw = yaw
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
            self.c.current_movement = "stop"
        await asyncio.sleep(C.SETTLE_SEC)
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
           "dock_live", "DryRunDriver", "CanDriver"]
