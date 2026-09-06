"""도킹값 + IMU 회전각 -> 주행 명령.

한 사이클: [측정] 30프레임 -> [판단] plan_step() -> [실행] 동작 하나 -> 반복.
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


def fwd_abort_deg(m, remaining_m):
    """전진 중 "이 각도를 넘으면 멈춰라" [도]. **상수가 아니라 계산이다.**"""
    geo = math.degrees(math.asin(min(1.0, C.LAT_TOL_M / max(remaining_m, 1e-6))))
    sig = (m or {}).get("heading_sigma_deg")
    floor = C.FWD_ABORT_K * sig if sig and math.isfinite(sig) else 0.0
    return max(geo, floor)


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
        turn = next_set3_step(half_fov)
        return ("search", turn, rot_sec_from_deg(turn),
                "Set3: 반시계 %.0f도 회전 (%d바퀴째)"
                % (turn, set3_rounds_done(rotations_done, half_fov) + 1))


    if holds >= C.HOLD_MAX_CONSEC:
        return ("lost", 0.0, 0.0,
                "%d번 연속 멈춰 있다 (마지막 이유: %s). 수동전환 — 사람이 확인해야 한다"
                % (holds, st.get("hold_why") or "?"))

    if m is None:
        return ("hold", 0.0, 0.0, "태그를 못 봤다")


    lateral_m, forward_m, heading_deg = m["lateral"], m["forward"], m["heading_deg"]

    if prev_forward is not None and forward_m > prev_forward + C.LAT_TOL_M:
        return ("hold", 0.0, 0.0,
                "forward 가 늘었다 (%.2f -> %.2fm). 멈추고 다시 잰다"
                % (prev_forward, forward_m))


    if abs(lateral_m) > C.LAT_TOL_M:
        # 측정값을 그대로 믿는다 (2026-09-06 결정) — 30프레임 중앙값이면
        # 5m 에서도 heading 오차가 도 단위 이하라 정렬 목표로 충분하고,
        # 회전 자체는 IMU 폐루프라 목표각 오차만큼만 틀린다. 각도 신뢰
        # 판정(reliable_angle)은 계산·기록만 하고 판단에는 안 쓴다.
        turn, distance_m, direction = plan_lateral_clear(lateral_m, heading_deg)
        estimated_sec = (rot_sec_from_deg(turn) + fwd_sec_from_offset_piecewise(distance_m)
                         + rot_sec_from_deg(90.0))
        return ("sidestep", (turn, distance_m, direction), estimated_sec,
                "Set2: lateral %.0fmm -> %+.1f도 회전 -> %.0fmm %s -> 90도 복귀"
                % (lateral_m * 1000, turn, distance_m * 1000,
                   "전진" if direction == "forward" else "후진"))


    if abs(heading_deg) > C.HEAD_TOL_DEG:
        return ("rotate_ccw" if heading_deg < 0 else "rotate_cw", abs(heading_deg),
                rot_sec_from_deg(heading_deg), "heading %.1f도 를 지운다" % heading_deg)


    if margin_px is not None and margin_px < C.TAG_CUT_MARGIN_PX:
        final_m = forward_m + C.DOCK_EXTRA_M
        if final_m <= 0.0:
            return ("done", 0.0, 0.0, "도착. forward %.2fm" % forward_m)
        return ("final", final_m, fwd_sec_from_offset_piecewise(final_m),
                "마지막 %.2fm (forward %.2fm + 여유 %.2fm). 그 뒤 정지"
                % (final_m, forward_m, C.DOCK_EXTRA_M))


    if forward_m > C.LAT_TOL_M:
        step_m = min(forward_m * C.FWD_SAFETY, C.STEP_M)
        return ("forward", step_m, fwd_sec_from_offset_piecewise(step_m),
                "Set1: 남은 %.2fm 중 %.2fm 전진" % (forward_m, step_m))

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
    elif action in ("forward", "final"):
        await drive_distance(driver, amount, "forward")
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
    n_frames = int(n_frames or D.MEASURE_FRAMES)
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
                    action, amount, sec, why = plan_step(m, st)
                    if m:
                        st["prev_forward"] = m["forward"]
                    if action in ("forward", "final"):
                        st["drove_forward"] = True
                    if action == "forward":
                        # 문턱은 "이번에 실제로 달릴 거리" 기준이어야 한다 — 남은
                        # 전체 거리로 재면 planner 가 방금 용인한 heading(<=허용치)이
                        # 곧바로 중단을 부르는 사각지대가 생긴다.
                        abort[0] = max(fwd_abort_deg(m, amount), C.HEAD_TOL_DEG)
                    else:
                        abort[0] = None
                    abort_hits = 0
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
                                 reasons=(m or {}).get("reasons"))
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
                    if d is not None and abs(d["heading_deg"]) > abort[0]:
                        abort_hits += 1
                        if abort_hits >= 3:
                            why_cut = ("heading %+.2f도 (허용 %.2f도, 3프레임 연속)"
                                       % (d["heading_deg"], abort[0]))
                    elif d is not None:
                        abort_hits = 0
                    mg = _margin_px(res, tag_id)
                    if why_cut is None and mg is not None and mg < C.TAG_CUT_MARGIN_PX:
                        why_cut = "태그가 화면 가장자리 %.0fpx (한계 %.0fpx)" % (mg, C.TAG_CUT_MARGIN_PX)
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
