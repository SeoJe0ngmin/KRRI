"""도킹값 + IMU 회전각 -> 주행 명령.  **v2 — 조준 접근**

한 사이클: [측정] 30프레임 -> [판단] plan_step() -> [실행] 동작 하나 -> 반복.

v2 규칙 (2026-09-07 실차 11회가 전부 작은 옆이동 반복으로 끝난 뒤):
    원거리(ALIGN_M 밖)  태그 축 위 조준점을 **조준해 곧장** 간다. 30mm/2도를 안 본다.
    근거리(ALIGN_M 안)  "지금 직진하면 어디 도착하나"(도착 예측)로 heading 을 정하고 간다.
                        heading 으로 삼킬 수 없을 만큼 lateral 이 크면 축을 본 뒤 **물러난다**.
    눈감는 마지막 직진  IMU yaw 가 지켜보다 틀어지면 정지 -> 수동전환.
v2 첫 실차(2026-09-07 18:25)에서 배운 것 둘이 기하에 들어 있다:
    제자리 회전이 카메라를 옆으로 옮긴다 (CAM_PIVOT_M) — 조준 회전은 이걸 넣어서 푼다.
    후진은 heading 이 0 일 때만 축 방향이다 — 물러나기 전에 축을 본다.
설계 이유·부호 약속·용어·실측 기록은 전부 ../../../CODE_NOTES.md 에 있다.
"""
import asyncio
import copy
import math

from config import control as C
from config import detection as D
from ...utils.event_log import record_event
from .fwd_time_model import fwd_sec_from_offset_piecewise
from .rot_control import rot_sec_from_deg, rot_timeout_sec, rotate_to


def normalize_deg(deg):
    """각도를 -180..180 으로 접는다. 예: 270 -> -90, -200 -> +160."""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


def drive_distance(driver, distance_m, direction):
    """distance_m 만큼 전진("forward") 또는 후진("backward"). 모든 직진의 공용 몸통."""
    seconds = fwd_sec_from_offset_piecewise(distance_m)
    return driver.forward(seconds) if direction == "forward" else driver.backward(seconds)


def plan_lateral_clear(lateral_m, heading_deg):
    """Set2 — lateral_m 을 지우는 최소각 회전을 고른다. v2 에서는 lateral 이 1m 급일 때만."""
    face_heading = -90.0 if lateral_m > 0 else 90.0
    turn_forward = normalize_deg(face_heading - heading_deg)
    turn_backward = normalize_deg(face_heading + 180.0 - heading_deg)
    if abs(turn_forward) - abs(turn_backward) > C.SIDESTEP_BACKWARD_GAIN_DEG:
        return turn_backward, abs(lateral_m), "backward"
    return turn_forward, abs(lateral_m), "forward"


def next_set3_step(half_fov_deg, rotations_done=0):
    """Set3 걸음 크기. 첫 바퀴는 연속 360도 — 회전 중에도 dock_live 가 프레임마다
    태그를 보다가 보이면 즉시 회전을 끊으므로, 돌면서 찾는다. 두 바퀴째부터는
    반화각씩 끊어 "블러 없는 깨끗한 한 번" 을 보장한다."""
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


# ── v2 기하 ──────────────────────────────────────────────────────────────────
# 부호 약속(실차로 확인된 v1 옆이동에서 그대로 가져옴):
#   heading h 로 d 만큼 직진하면  lateral += d·sin(h),  forward -= d·cos(h)
#   (h=+90 으로 직진하면 lateral 이 +쪽으로 움직인다. 2026-09-07 -2238mm -> +15mm 실측)
# 제자리 회전(CAM_PIVOT_M 모델, 2026-09-07 v2 첫 주행 실측):
#   카메라 = 회전중심 + CAM_PIVOT_M·(sin h, -cos h).  회전 θ 뒤
#   lateral += CAM_PIVOT_M·(sin(h+θ) − sin h),  forward −= CAM_PIVOT_M·(cos(h+θ) − cos h)

def _spread(m, key):
    """측정 대표값의 표준오차. 없으면 0 (= 잡음 문턱이 허용치로 떨어진다)."""
    try:
        return float((m.get("spread") or {}).get(key, 0.0) or 0.0)
    except (TypeError, ValueError, AttributeError):
        return 0.0


def cam_shift(heading_deg, turn_deg):
    """제자리 회전 turn_deg 뒤 카메라가 옮겨지는 (Δlateral, Δforward) [m]."""
    r = C.CAM_PIVOT_M
    h0, h1 = math.radians(heading_deg), math.radians(heading_deg + turn_deg)
    return r * (math.sin(h1) - math.sin(h0)), -r * (math.cos(h1) - math.cos(h0))


def aim_turn(lateral_m, forward_m, heading_deg, target_forward_m):
    """축 위 target_forward_m 지점을 향하려면 몇 도 돌아야 하나.

    회전이 카메라를 옮기는 것(cam_shift)까지 넣어 고정점 반복으로 푼다 — 돌고 난
    자리에서 봐도 조준점을 향하도록.  반환 (turn_deg, aim_deg, lateral_after,
    forward_after, dist_after).  aim_deg 는 회전 뒤 heading.
    """
    dz = forward_m - target_forward_m
    turn = normalize_deg(math.degrees(math.atan2(-lateral_m, dz)) - heading_deg)
    for _ in range(12):
        dl, df = cam_shift(heading_deg, turn)
        aim = math.degrees(math.atan2(-(lateral_m + dl), forward_m + df - target_forward_m))
        new_turn = normalize_deg(aim - heading_deg)
        done = abs(new_turn - turn) < 0.01
        turn = new_turn
        if done:
            break
    dl, df = cam_shift(heading_deg, turn)
    lat_after, fwd_after = lateral_m + dl, forward_m + df
    return (turn, normalize_deg(heading_deg + turn), lat_after, fwd_after,
            math.hypot(lat_after, fwd_after - target_forward_m))


def landing_lateral_m(lateral_m, remaining_m, heading_deg):
    """지금 heading 그대로 remaining_m 직진하면 도착하는 lateral [m] (도착 예측)."""
    return lateral_m + remaining_m * math.sin(math.radians(heading_deg))


def is_near(forward_m, st):
    """근거리 규칙을 쓸 거리인가. 한 번 들어오면 NEAR_EXIT_M 만큼 더 멀어져야 나간다 (경계 왕복 방지)."""
    if st.get("near"):
        return forward_m <= C.ALIGN_M + C.NEAR_EXIT_M
    return forward_m <= C.ALIGN_M + C.APPROACH_MIN_LEG_M


def _sidestep(lateral_m, heading_deg, label):
    turn, distance_m, direction = plan_lateral_clear(lateral_m, heading_deg)
    estimated_sec = (rot_sec_from_deg(turn) + fwd_sec_from_offset_piecewise(distance_m)
                     + rot_sec_from_deg(90.0))
    return ("sidestep", (turn, distance_m, direction), estimated_sec,
            "Set2(%s): lateral %.0fmm -> %+.1f도 회전 -> %.0fmm %s -> 90도 복귀"
            % (label, lateral_m * 1000, turn, distance_m * 1000,
               "전진" if direction == "forward" else "후진"))


def _rotate(turn_deg, why):
    return ("rotate_ccw" if turn_deg > 0 else "rotate_cw", abs(turn_deg),
            rot_sec_from_deg(turn_deg), why)


def plan_step(m, state=None):
    """측정값 하나 -> 다음 동작 하나. 순수 함수 (하드웨어 없이 계산만).

    반환 (동작, 양, 예상시간[s], 이유).  동작과 양:
        approach  (leg_m, aim_deg)            조준 직진 한 조각 (원거리)
        backup    back_m                      물러남 (근거리, lateral 을 heading 으로 못 삼킬 때)
        forward / final  step_m               근거리 직진 / 눈감는 마지막 직진
        rotate_ccw / rotate_cw  |deg|         조준 회전
        sidestep  (turn, dist, direction)     v1 옆이동 — lateral 이 SIDESTEP_MIN_LAT_M 이상일 때만
        recover_backup / search / hold / lost / done   v1 과 같음
    state 는 dock_live 가 관리한다: misses, holds, margin_px, prev_forward, half_fov_deg,
    drove_forward, backed_up_once, last_action, near, backups, near_rots.
    """
    st = state or {}
    misses = int(st.get("misses", 0))
    holds = int(st.get("holds", 0))
    margin_px = st.get("margin_px")
    prev_forward = st.get("prev_forward")
    half_fov = float(st.get("half_fov_deg", 35.0))
    drove_forward = bool(st.get("drove_forward", False))
    backed_up_once = bool(st.get("backed_up_once", False))
    last_action = st.get("last_action")

    # ① 전진 직후 실종 -> 딱 한 번 후진 (방위는 아니까)
    if misses >= 1 and drove_forward and not backed_up_once:
        return ("recover_backup", C.SEARCH_BACKUP_M,
                fwd_sec_from_offset_piecewise(C.SEARCH_BACKUP_M),
                "전진 중 태그를 놓쳤다. %.1fm 후진해서 다시 본다" % C.SEARCH_BACKUP_M)

    # ② 연속 미검출 -> Set3 회전 탐색
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

    # ③ 연속 hold 가 쌓이면 수동전환 (안 움직인 채 단계만 소진하는 최악 경로 차단)
    if holds >= C.HOLD_MAX_CONSEC:
        return ("lost", 0.0, 0.0,
                "%d번 연속 멈춰 있다 (마지막 이유: %s). 수동전환 — 사람이 확인해야 한다"
                % (holds, st.get("hold_why") or "?"))

    # ④ 태그 못 봄
    if m is None:
        return ("hold", 0.0, 0.0, "태그를 못 봤다")

    lateral_m, forward_m, heading_deg = m["lateral"], m["forward"], m["heading_deg"]
    sp_lat, sp_head, sp_fwd = _spread(m, "lateral"), _spread(m, "heading_deg"), _spread(m, "forward")

    # ⑤ 직진류 뒤에 forward 가 잡음보다 크게 늘었다 -> "뒤로 갔네?" hold.
    #    옆이동·후진·회전 뒤에는 forward 가 늘어나는 게 정상이라 안 본다.
    if (prev_forward is not None and last_action in ("approach", "forward", "final")
            and forward_m > prev_forward + max(C.LAT_TOL_M, C.NOISE_K * sp_fwd)):
        return ("hold", 0.0, 0.0,
                "forward 가 늘었다 (%.2f -> %.2fm). 멈추고 다시 잰다"
                % (prev_forward, forward_m))

    remaining_m = forward_m + C.DOCK_EXTRA_M
    lat_noise_m = max(C.LAT_TOL_M, C.NOISE_K * sp_lat)
    head_noise_deg = C.NOISE_K * sp_head

    # ⑥ 원거리 — 축 위 정렬선(ALIGN_M) 지점을 조준해 곧장 간다. 단 조준점은 최소
    #    APPROACH_MIN_DZ_M 앞에 둔다: 정렬선 바로 앞에서 정렬선을 조준하면 lateral 몇 cm 가
    #    조준각 수십 도가 된다 (2026-09-07 18:25 실측: 0.3m 앞 점 조준 -> 28도).
    if not is_near(forward_m, st):
        target_m = min(C.ALIGN_M, forward_m - C.APPROACH_MIN_DZ_M)
        turn, aim_deg, lat_after, fwd_after, dist_after = aim_turn(
            lateral_m, forward_m, heading_deg, target_m)
        if abs(aim_deg) > C.APPROACH_MAX_DEG:
            # 곧장 가면 태그가 시야를 벗어난다. 조준점이 깊어서 여기 오는 건 lateral 이 1m 급일 때뿐
            return _sidestep(lateral_m, heading_deg,
                             "조준각 %+.0f도 > %.0f도" % (aim_deg, C.APPROACH_MAX_DEG))
        if abs(turn) > max(C.FAR_ROT_MIN_DEG, head_noise_deg):
            return _rotate(turn,
                           "조준: 축 위 %.1fm 지점을 향해 %+.1f도 회전 (lateral %.0fmm -> 회전 뒤 %.0fmm, 조준각 %+.1f도)"
                           % (target_m, turn, lateral_m * 1000, lat_after * 1000, aim_deg))
        # 정렬선을 지나치지 않게 조각을 자른다 (전진 성분 = leg·cos(aim))
        to_line_m = max(0.0, forward_m - C.ALIGN_M) / max(math.cos(math.radians(heading_deg)), 0.5)
        leg_m = min(dist_after, C.APPROACH_STEP_M, to_line_m)
        if leg_m >= C.APPROACH_MIN_LEG_M:
            return ("approach", (leg_m, heading_deg), fwd_sec_from_offset_piecewise(leg_m),
                    "조준 직진: %.2fm (heading %+.1f도, 정렬선까지 %.2fm)"
                    % (leg_m, heading_deg, forward_m - C.ALIGN_M))
        # 정렬선 바로 앞이라 조각이 안 나온다 -> 근거리 규칙으로

    # ⑦ 근거리 — 도킹 지점(축 위 -DOCK_EXTRA_M)을 조준한다. 회전 뒤 heading 이
    #    NEAR_ABSORB_DEG 안이면 그 heading 으로 곧장 가서 lateral 0 에 도착한다.
    turn, aim_deg, lat_after, fwd_after, _ = aim_turn(
        lateral_m, forward_m, heading_deg, -C.DOCK_EXTRA_M)
    absorbable = abs(aim_deg) <= C.NEAR_ABSORB_DEG or abs(lateral_m) <= lat_noise_m
    if not absorbable:
        if abs(lateral_m) >= C.SIDESTEP_MIN_LAT_M:
            return _sidestep(lateral_m, heading_deg,
                             "근거리, lateral %.0fmm >= %.0fmm"
                             % (lateral_m * 1000, C.SIDESTEP_MIN_LAT_M * 1000))
        if int(st.get("backups", 0)) >= C.BACKUP_MAX_COUNT:
            return ("lost", 0.0, 0.0,
                    "%d번 물러나도 정렬이 안 된다 (lateral %.0fmm, heading %+.1f도). 수동전환"
                    % (C.BACKUP_MAX_COUNT, lateral_m * 1000, heading_deg))
        if abs(heading_deg) > max(C.NEAR_FACE_DEG, head_noise_deg):
            # 비스듬히 후진하면 lateral 이 sin(heading) 만큼 밀린다 (실측 +142 -> -423mm).
            # 먼저 축을 본다. 회전이 카메라를 옮기는 건 다음 측정이 다시 잰다.
            return _rotate(-heading_deg,
                           "물러나기 전 축을 본다: heading %+.1f도 -> 0 (lateral %.0fmm 는 %.1f도로 못 삼킨다)"
                           % (heading_deg, lateral_m * 1000, C.NEAR_ABSORB_DEG))
        need_m = abs(lateral_m) / math.sin(math.radians(C.NEAR_ABSORB_DEG))
        back_m = min(max(need_m - remaining_m, C.APPROACH_MIN_LEG_M), C.BACKUP_MAX_M)
        return ("backup", back_m, fwd_sec_from_offset_piecewise(back_m),
                "물러남: lateral %.0fmm 는 %.1f도로는 %.1fm 부터 삼킬 수 있다 -> %.2fm 후진 (heading %+.1f도)"
                % (lateral_m * 1000, C.NEAR_ABSORB_DEG, need_m, back_m, heading_deg))

    # ⑧ 근거리 조준 회전 — 이 지게차 회전은 ±1도라 NEAR_ROT_MIN_DEG 아래는 안 돈다.
    #    같은 자리에서 NEAR_ROT_MAX_COUNT 번 돌았으면 더 돌지 않고 간다 (회전이 잡음을 못 이긴다).
    landing_mm = landing_lateral_m(lateral_m, remaining_m, heading_deg) * 1000
    if (abs(turn) > max(C.NEAR_ROT_MIN_DEG, head_noise_deg)
            and int(st.get("near_rots", 0)) < C.NEAR_ROT_MAX_COUNT):
        return _rotate(turn,
                       "도착 예측 %+.0fmm -> heading %+.2f도가 되게 %+.2f도 회전 (lateral %.0fmm -> 회전 뒤 %.0fmm)"
                       % (landing_mm, aim_deg, turn, lateral_m * 1000, lat_after * 1000))

    # ⑨ 화면 여유가 없으면 마지막 (눈감고 remaining 통째). 아니면 한 조각 전진
    if margin_px is not None and margin_px < C.TAG_CUT_MARGIN_PX:
        if remaining_m <= 0.0:
            return ("done", 0.0, 0.0, "도착. forward %.2fm" % forward_m)
        return ("final", remaining_m, fwd_sec_from_offset_piecewise(remaining_m),
                "마지막 %.2fm (forward %.2fm + 여유 %.2fm). 도착 예측 %+.0fmm, heading %+.2f도. 그 뒤 정지"
                % (remaining_m, forward_m, C.DOCK_EXTRA_M, landing_mm, heading_deg))

    if remaining_m > C.LAT_TOL_M:
        step_m = min(remaining_m * C.FWD_SAFETY, C.STEP_M)
        return ("forward", step_m, fwd_sec_from_offset_piecewise(step_m),
                "Set1: 남은 %.2fm 중 %.2fm 전진 (도착 예측 %+.0fmm)"
                % (remaining_m, step_m, landing_mm))

    return ("done", 0.0, 0.0, "도착. forward %.2fm" % forward_m)


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
    elif action == "approach":
        leg_m, _aim_deg = amount
        await drive_distance(driver, leg_m, "forward")
    elif action in ("forward", "final"):
        await drive_distance(driver, amount, "forward")
    elif action in ("backup", "recover_backup"):
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


def _abort_plan(action, amount, m):
    """직진 중 감시 기준. (기준 heading[도], 허용 편차[도]) 또는 None.

    직진은 지금 heading 을 유지하는 것이라 기준은 출발 heading 이다 — 0 을 기준으로
    보면 대각선 주행이 곧바로 중단된다. 허용 편차는 잡음(1프레임 표준편차 ≈ spread×√n)
    보다 크게 잡는다. 출발 직후 LEG_GRACE_SEC 동안은 dock_live 가 heading 을 안 본다.
    """
    if not m:
        return None
    n = max(1, int(m.get("n", 1) or 1))
    frame_sigma = _spread(m, "heading_deg") * math.sqrt(n)
    if action == "approach":
        _leg_m, ref_deg = amount
        return ref_deg, max(C.FAR_ABORT_DEG, C.NOISE_K * frame_sigma)
    if action == "forward":
        return m["heading_deg"], max(fwd_abort_deg(amount), C.HEAD_TOL_DEG, C.NOISE_K * frame_sigma)
    return None


def _yaw_now(driver):
    """드라이버에 살아 있는 IMU 가 있으면 지금 yaw [도], 아니면 None."""
    yaw = getattr(driver, "yaw", None)
    if yaw is None or not getattr(yaw, "alive", False):
        return None
    try:
        return float(yaw.angle_deg)
    except (TypeError, ValueError, AttributeError):
        return None


async def dock_live(pipe, driver, tag_id=None, max_steps=None, log=print,
                    on_frame=None, n_frames=None, should_stop=None,
                    record_dir=None):
    """도킹 루프 — 프레임을 계속 읽으며 측정-판단-실행을 반복한다."""
    from ..detection.detection_pose import measure
    tag_id = D.TAG_ID if tag_id is None else tag_id
    max_steps = C.MAX_STEPS if max_steps is None else max_steps
    n_frames = int(n_frames or D.MEASURE_FRAMES)
    half_fov = _half_fov_deg(pipe)
    st = {"half_fov_deg": half_fov, "prev_forward": None,
          "drove_forward": False, "backed_up_once": False, "last_action": None,
          "near": False, "backups": 0, "near_rots": 0}
    gen = iter(pipe)
    history, buf = [], []
    phase, step, task, info = "measure", 0, None, {}
    started = [0.0]
    waited = 0
    saw_any = False
    misses = 0
    outcome = "incomplete"
    margins = []
    abort = [None]          # (기준 heading, 허용 편차) — 직진 중에만
    abort_hits = 0          # 직진 중 heading 이 문턱을 연속 몇 프레임 넘었나
    final_yaw0 = None       # 마지막 직진 시작 때 IMU yaw. None 이면 IMU 감시 없음

    def now():
        return asyncio.get_event_loop().time()

    async def run_action(action, amount, sec):
        return await _execute(driver, action, amount, sec)

    async def cancel_task():
        nonlocal task
        if task is not None and not task.done():
            task.cancel()
            try:
                await task
            except asyncio.CancelledError:
                pass
        task = None

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
                    await cancel_task()
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
                if len(buf) >= n_frames or waited >= D.MEASURE_MAX_FRAMES:
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
                    if m:
                        st["near"] = is_near(m["forward"], st)
                    action, amount, sec, why = plan_step(m, st)
                    if m:
                        st["prev_forward"] = m["forward"]
                    if action in ("forward", "final", "approach"):
                        st["drove_forward"] = True
                    if action in ("forward", "final", "approach", "backup", "sidestep"):
                        st["near_rots"] = 0
                    elif action in ("rotate_ccw", "rotate_cw") and st.get("near"):
                        st["near_rots"] = st.get("near_rots", 0) + 1
                    if action == "backup":
                        st["backups"] = st.get("backups", 0) + 1
                    abort[0] = _abort_plan(action, amount, m)
                    abort_hits = 0
                    if action == "recover_backup":
                        st["backed_up_once"] = True
                    if action == "hold":
                        st["holds"] = st.get("holds", 0) + 1
                        st["hold_why"] = why
                    else:
                        st["holds"] = 0
                    st["last_action"] = action

                    if action != "search":
                        step += 1
                    log("[%2d] %s" % (step, _fmt_measure(m)))
                    log("     %-10s %s" % (action, why))
                    history.append((action, amount, sec, why))
                    extra = {"near": bool(st.get("near"))}
                    if m:
                        extra.update({"spread_lateral": _spread(m, "lateral"),
                                      "spread_heading": _spread(m, "heading_deg"),
                                      "landing_mm": landing_lateral_m(
                                          m["lateral"], m["forward"] + C.DOCK_EXTRA_M,
                                          m["heading_deg"]) * 1000})
                    record_event(record_dir, "decision", step=step, action=action,
                                 why=why, misses=misses, margin_px=st.get("margin_px"),
                                 stable=(m or {}).get("stable"),
                                 reasons=(m or {}).get("reasons"), planner="v2", **extra)
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
                        final_yaw0 = _yaw_now(driver)
                        if final_yaw0 is None:
                            log("     (IMU 없음 — 마지막 직진을 감시 없이 간다)")
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
                # 눈감는 구간 — 카메라는 못 보지만 IMU 는 본다. 도착 예측이 창을 벗어날
                # 만큼 yaw 가 틀어지면 여기서 세운다. 근거리에서 Set3 회전은 위험하므로
                # (태그가 위로 잘려 돌아도 안 보인다) 재탐색 대신 곧장 수동전환.
                if task is not None and not task.done() and final_yaw0 is not None:
                    yaw_now = _yaw_now(driver)
                    if yaw_now is not None:
                        dev = normalize_deg(yaw_now - final_yaw0)
                        if abs(dev) > C.FINAL_YAW_ABORT_DEG:
                            why_cut = ("마지막 직진 중 yaw %+.2f도 틀어짐 (한계 %.1f도)"
                                       % (dev, C.FINAL_YAW_ABORT_DEG))
                            log("     !! %s — 정지, 수동전환" % why_cut)
                            record_event(record_dir, "abort", step=step, why=why_cut)
                            await cancel_task()
                            await driver.stop()
                            outcome, phase = "manual", "manual"
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
                    ref_deg, tol_deg = abort[0]
                    why_cut = None
                    d = res.docking.get(tag_id)
                    # heading 은 원시 1프레임 값이라 연속 3프레임을 요구한다. 출발 직후
                    # LEG_GRACE_SEC 동안은 조향이 되돌아오며 2~5도 흔들리므로(실측) 안 본다.
                    # 화면 여유는 기하라 즉시 끊는다.
                    in_grace = (now() - started[0]) < C.LEG_GRACE_SEC
                    if d is not None and not in_grace and \
                            abs(normalize_deg(d["heading_deg"] - ref_deg)) > tol_deg:
                        abort_hits += 1
                        if abort_hits >= 3:
                            why_cut = ("heading %+.2f도, 기준 %+.2f도에서 %.2f도 초과 (3프레임 연속)"
                                       % (d["heading_deg"], ref_deg, tol_deg))
                    elif d is not None:
                        abort_hits = 0
                    mg = _margin_px(res, tag_id)
                    if why_cut is None and mg is not None and mg < C.TAG_CUT_MARGIN_PX:
                        why_cut = "태그가 화면 가장자리 %.0fpx (한계 %.0fpx)" % (mg, C.TAG_CUT_MARGIN_PX)
                    if why_cut is not None:
                        log("     !! 직진 중단 — %s" % why_cut)
                        record_event(record_dir, "abort", step=step, why=why_cut)
                        await cancel_task()
                        abort[0] = None
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
                if phase in ("command", "search", "final"):
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
        await cancel_task()
        await driver.stop()
        record_event(record_dir, "result", outcome=outcome, steps=step, planner="v2")
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
           "aim_turn", "cam_shift", "landing_lateral_m", "is_near",
           "dock_live", "DryRunDriver", "CanDriver"]
