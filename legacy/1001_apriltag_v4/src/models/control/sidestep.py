"""처음 한 번, 통로 밖이면 옆으로 비켜선다 (2026-09-30 결정 ⑤). 후진은 마지막 단계의 최후 수단이다 (결정 ⑥).

    ① 태그를 보고 좌우를 읽는다                 needed() 가 통로(limits.corridor_half_m)와 견준다
    ② 법선에 직각으로 돈다  (±90 − 지금방향)      plan() → rotate(), IMU 폐루프
    ③ 읽은 좌우 x SIDESTEP_FRACTION 만큼 간다     forward_timed(), 눈 감고 시간으로
    ④ 다시 법선을 향해 돈다 (0 − 지금방향)        지금방향 = ①의 방향 + 그동안 자이로가 잰 변화
    ⑤ 다시 재고 계획으로                        **부르는 쪽(run.py)이 estimator.reset() 을 부른 뒤**

반복하지 않는다. 사이드스텝은 딱 한 번이고, 그 뒤엔 통로 안이든 밖이든 **바로 중간점 계획**으로 간다.
후진(backup())은 여기서 안 한다 — 중간점을 다 갔는데(태그컷 도착) 진입 조건이 안 될 때, 마지막 단계에서 한 번만.
바라볼 각은 v1 plan_lateral_clear (apriltag_v1/src/models/control/control_from_pose.py:31) 그대로,
단 **앞으로만** 간다 — v1 의 '뒤로 가면 회전이 작다' 선택지는 뺐다(후진은 마지막 단계 전용, 결정 ⑥).

부호 약속 (pose.state · limits.lateral_leak_m): 좌우 +가 왼쪽, 방향 +가 반시계(자이로와 같다),
방향 h 로 d 만큼 전진하면 좌우가 d x sin(h) 만큼 는다. 그래서 좌우 +를 지우려면 −90 을 보고 전진한다.
"""
from dataclasses import dataclass

from config import control as C
from ... import limits
from ...utils import clock
from .forward import forward_timed, min_step_m, strength_of
from .rotate import rotate

#: 회전 상한 [도] = 직각 + 한 걸음 보정 상한. ④는 90 에 ②의 오차가 얹히므로 90 딱 맞추면 절반은 거부된다
TURN_CAP_DEG = 180.0        # 정상 범위 확인용. 작은 쪽으로 접으니 |turn1| ≤ 90 + |방향| ≤ 180


def normalize_deg(deg):
    """각도를 -180..180 으로 접는다. v1 control_from_pose.py L17-20 과 같은 식."""
    d = (float(deg) + 180.0) % 360.0 - 180.0
    return d + 360.0 if d <= -180.0 else d


@dataclass
class SidestepPlan:
    turn1_deg: float = 0.0          # ② 법선에 직각이 되게
    drive_m: float = 0.0            # ③
    movement: str = "forward"       # ③ forward / backward
    turn2_deg: float = 0.0          # ④ 이상값(= −바라볼 각). 실행 땐 자이로로 다시 계산한다
    why: str = ""                   # 비어 있으면 실행 가능
    heading0_deg: float = 0.0       # ① 때 방향. ④의 기준
    lateral0_m: float = 0.0         # ① 때 좌우

    @property
    def ok(self):
        return not self.why


@dataclass
class BackupPlan:
    """마지막 단계의 최후 수단 (2026-09-30 결정 ⑥). 중간점 후보에 후진은 없고, 사이드스텝 직후에도 없다.

    후진량 = clamp(max(필요량, 최소걸음), 0, 뒤공간)
      필요량   limits.backup_needed_m — 태그컷에 서 있으면 |좌우| / (K·tan 경로각상한). 물러나면 깔때기가 넓어진다
      최소걸음  배운 후진 정속 x 관성시간 (forward.min_step_m). 끊어도 그만큼은 더 간다
    한 번만이다. 횟수(MAX_CORRECTIONS)·시간(FINE_TIME_LIMIT_S) 상한은 부르는 쪽이 같은 구간에서 센다.
    실행도 부르는 쪽이: forward(movement="backward") 에 뒤로 남은 거리를 주거나, 태그가 안 보이면 forward_timed.
    """
    need_m: float = 0.0             # 통로에 들 때까지 물러날 거리
    floor_m: float = 0.0            # 배운 최소걸음. 모르면 0
    space_m: float = 0.0            # 뒤공간 (config BACK_MAX_M)
    distance_m: float = 0.0         # 실제로 물러날 거리
    sigma_m: float = 0.0            # 그때 좌우 σ — 2σ 문턱의 근거
    why: str = ""                   # no_fix / uncertain / no_space → 정지·사람.  inside → 후진 불필요, 다시 계획

    @property
    def ok(self):
        return not self.why and self.distance_m > 0.0


@dataclass
class SidestepResult:
    ok: bool = False
    why: str = ""
    plan: SidestepPlan = None
    turn1: object = None            # RotationResult
    leg: object = None              # LegResult
    turn2: object = None            # RotationResult
    heading_est_deg: float = 0.0    # ④ 직전, 자이로로 추정한 방향
    heading_end_deg: float = 0.0    # 끝났을 때(어디서 끊겼든) 자이로로 추정한 방향. 사람이 볼 값
    gyro_gaps: int = 0              # 그동안 자이로 유실 구간 수. 0 이 아니면 추정 방향을 의심
    t_start: float = 0.0
    t_end: float = 0.0


def needed(fix, intr=None, cut_m=None):
    """통로 밖인가. 못 믿는 측정(fix.ok=False)이면 False — 사이드스텝은 확실할 때만 한다."""
    if not fix.ok:
        return False
    return abs(fix.lateral_m) > limits.corridor_half_m(fix.forward_m, intr, cut_m)


def plan(fix, learner, back_max_m=None):
    """①에서 ②③④를 정한다. 방향 규칙은 v1 plan_lateral_clear 그대로."""
    back_max = C.BACK_MAX_M if back_max_m is None else float(back_max_m)
    p = SidestepPlan(heading0_deg=float(fix.heading_deg), lateral0_m=float(fix.lateral_m))
    if not fix.ok:
        p.why = "no_fix"
        return p
    lat, h0 = p.lateral0_m, p.heading0_deg
    p.drive_m = abs(lat) * C.SIDESTEP_FRACTION
    if p.drive_m <= 0.0:
        p.why = "zero"
        return p
    face = -90.0 if lat > 0 else 90.0                     # 전진해서 지우려면 바라볼 각 (v1 L31)
    p.turn1_deg, p.movement = normalize_deg(face - h0), "forward"   # 작은 쪽으로 접는다 (-180..180)
    # 뒤로 가는 선택지는 없다 — 후진은 마지막 단계의 최후 수단 한 곳뿐 (2026-10-01 결정)
    p.turn2_deg = normalize_deg(-face)
    if abs(p.turn1_deg) > TURN_CAP_DEG:
        p.why = "turn_too_big"                            # 접은 각이 180 을 넘을 순 없다 — 계산 오류
    return p


def backup(fix, learner=None, intr=None, back_space_m=None, cut_m=None):
    """중간점을 다 갔는데 진입 조건이 안 될 때 한 번. 그 뒤엔 다시 중간점 계획, 그래도 안 되면 정지·사람.

    남은 좌우가 2σ(UNCERTAIN_FACTOR) 안이면 진짜 오차인지 모르니 물러나도 소용없다 → uncertain.
    필요량 > 뒤공간이면 물러나도 통로에 못 든다 → no_space. 최소걸음 > 뒤공간이면 서기 전에 뒤를 친다 → 같다.
    """
    b = BackupPlan(space_m=C.BACK_MAX_M if back_space_m is None else float(back_space_m))
    if not fix.ok:
        b.why = "no_fix"
        return b
    b.sigma_m = float(fix.lateral_sigma_m)
    if abs(fix.lateral_m) <= C.UNCERTAIN_FACTOR * b.sigma_m:
        b.why = "uncertain"
        return b
    b.need_m = limits.backup_needed_m(fix.lateral_m, fix.forward_m, intr, cut_m)
    if b.need_m <= 0.0:
        b.why = "inside"                                  # 통로 안이다 — 대각선 한 번이면 된다
        return b
    if learner is not None:
        b.floor_m = min_step_m(learner, strength_of("backward")) or 0.0
    want = max(b.need_m, b.floor_m)
    b.distance_m = min(want, b.space_m)
    if want > b.space_m:
        b.why = "no_space"
    return b


def execute(driver, gyro, learner, plan, rec=None, log=print):
    """②③④를 차례로. 하나라도 어긋나면 그 자리에서 선다(정지·사람 — plan 3-10). 반복 없음.

    돌아온 뒤 부르는 쪽이 estimator.reset() 을 하고 다시 잰다 — 옛 점이 섞이면 기울기가 거짓말한다.
    자이로 영점(gyro.zero)은 이 안에서 부르지 않는다: ④가 ①의 방향 + 자이로 변화로 계산되기 때문.
    """
    out = SidestepResult(plan=plan, t_start=clock.now())
    if not plan.ok:
        out.why = plan.why
        _record(rec, out, log)
        return out
    g0 = gyro.angle_deg
    gaps0 = gyro.stats().get("gaps", 0)
    log("       == 사이드스텝: 좌우 %+.3f m, 방향 %+.1f도 → 돌기 %+.1f도 · %s %.3f m · 법선으로"
        % (plan.lateral0_m, plan.heading0_deg, plan.turn1_deg, plan.movement, plan.drive_m))
    _record(rec, out, log, stage="plan")

    out.turn1 = rotate(driver, gyro, learner, plan.turn1_deg, rec, log,
                       cap_deg=TURN_CAP_DEG, safety_s=C.SIDESTEP_ROT_SAFETY_S)
    if not out.turn1.ok:
        out.why = "turn1:" + (out.turn1.reason or "?")
        return _finish(out, gyro, plan, g0, gaps0, rec, log)

    out.leg = forward_timed(driver, learner, plan.drive_m, plan.movement, rec, log)
    if not out.leg.timed_ok:
        out.why = "drive:" + (out.leg.reason or "?")
        return _finish(out, gyro, plan, g0, gaps0, rec, log)

    out.heading_est_deg = normalize_deg(plan.heading0_deg + (gyro.angle_deg - g0))
    turn2 = normalize_deg(-out.heading_est_deg)
    out.turn2 = rotate(driver, gyro, learner, turn2, rec, log,
                       cap_deg=TURN_CAP_DEG, safety_s=C.SIDESTEP_ROT_SAFETY_S)
    if out.turn2.ok or out.turn2.reason in ("zero", "too_small"):   # 이미 법선이면 안 돌아도 된다
        out.ok = True
    else:
        out.why = "turn2:" + (out.turn2.reason or "?")
    return _finish(out, gyro, plan, g0, gaps0, rec, log)


# ── 속 ──────────────────────────────────────────────────────────────
def _finish(out, gyro, plan, g0, gaps0, rec, log):
    out.t_end = clock.now()
    out.gyro_gaps = gyro.stats().get("gaps", 0) - gaps0
    out.heading_end_deg = normalize_deg(plan.heading0_deg + (gyro.angle_deg - g0))
    _record(rec, out, log, stage="end")
    if out.ok:
        log("          사이드스텝 끝: 법선 기준 %+.2f도 → 돌아서 %+.2f도 (%.1fs)"
            % (out.heading_est_deg, out.heading_end_deg, out.t_end - out.t_start))
    else:
        log("          !! 사이드스텝 실패(%s): 추정 방향 %+.2f도 — 정지·사람"
            % (out.why, out.heading_end_deg))
    return out


def _record(rec, out, log, stage="refused"):
    """기록이 주행을 막지 않는다 (plan 6-6 ④)."""
    if rec is None:
        return
    try:
        p = out.plan
        rec.event("sidestep", stage=stage, ok=out.ok, why=out.why,
                  turn1_deg=p.turn1_deg, drive_m=p.drive_m, movement=p.movement,
                  turn2_ideal_deg=p.turn2_deg, heading0_deg=p.heading0_deg,
                  lateral0_m=p.lateral0_m, heading_est_deg=out.heading_est_deg,
                  heading_end_deg=out.heading_end_deg,
                  turn1_actual=(out.turn1.turned_deg if out.turn1 else None),
                  turn2_actual=(out.turn2.turned_deg if out.turn2 else None),
                  gyro_gaps=out.gyro_gaps, t_start=out.t_start, t_end=out.t_end)
    except Exception as e:
        log("          !! 사이드스텝 기록 실패: %s (주행은 계속한다)" % e)


if __name__ == "__main__":
    # 자체 시험 — 하드웨어 없이. (1) 통로 판정 표 (2) 계획: 방향 4가지 (3) 2차원 세계에서 실행 —
    # 가상 시계로 회전·시간직진을 실제 코드 그대로 돌려 좌우가 절반 줄고 법선을 다시 보는지 본다.
    import math
    import time
    from dataclasses import dataclass as _dc
    from src.models.control import learn as M
    from ...utils.gyro import Gyro
    from .learn import Ema, Learner

    @_dc
    class Fix:                       # estimate.Fix 중 여기서 쓰는 것만
        ok: bool = True
        lateral_m: float = 0.0
        heading_deg: float = 0.0
        forward_m: float = 5.0
        lateral_sigma_m: float = 0.0

    quiet = lambda *_: None

    # (1) 통로 판정 표
    print("거리[m]  반폭[m]   좌우 0.3 / 1.0 / 2.5 m 일 때 사이드스텝?")
    for d in (4.0, 5.0, 6.0, 8.0):
        half = limits.corridor_half_m(d)
        row = [needed(Fix(lateral_m=l, forward_m=d)) for l in (0.3, 1.0, 2.5)]
        print("  %4.1f    %.3f    %s" % (d, half, "  ".join("예" if x else "아니오" for x in row)))
    assert not needed(Fix(lateral_m=0.3, forward_m=8.0)) and needed(Fix(lateral_m=2.5, forward_m=8.0))
    assert not needed(Fix(ok=False, lateral_m=9.0))                     # 못 믿으면 안 한다
    assert needed(Fix(lateral_m=0.5, forward_m=4.0)) and not needed(Fix(lateral_m=0.5, forward_m=6.0))

    # (2) 계획 — 전진만, 작은 쪽으로 접는다, 상한 180 (2026-10-01 사용자 확정: 뒤로 가는 선택지 없음)
    lrn = Learner(seeds=M.seeds())
    p = plan(Fix(lateral_m=1.0, heading_deg=-10.0), lrn)                # 왼쪽에 있고 태그를 본다
    assert p.ok and (p.turn1_deg, p.movement, p.turn2_deg) == (-80.0, "forward", 90.0), p
    assert abs(p.drive_m - 0.5) < 1e-9                                  # 잰 좌우의 절반 (SIDESTEP_FRACTION)
    p = plan(Fix(lateral_m=-1.0, heading_deg=10.0), lrn)                # 오른쪽, 거울
    assert p.ok and (p.turn1_deg, p.movement, p.turn2_deg) == (80.0, "forward", -90.0), p
    p = plan(Fix(lateral_m=1.0, heading_deg=30.0), lrn)                 # 등지고 있어도 앞으로 (−120 ≤ 180)
    assert p.ok and (p.turn1_deg, p.movement, p.turn2_deg) == (-120.0, "forward", 90.0), p
    p = plan(Fix(lateral_m=-0.5, heading_deg=-20.0), lrn)
    assert p.ok and (p.turn1_deg, p.movement, p.turn2_deg) == (110.0, "forward", -90.0), p
    p = plan(Fix(lateral_m=1.0, heading_deg=-170.0), lrn)               # 작은 쪽으로 접는다: −90 − (−170) = +80 (260 이 아니다)
    assert p.ok and p.turn1_deg == 80.0 and abs(p.turn1_deg) <= TURN_CAP_DEG, p
    lrn_b = Learner(seeds=M.seeds())                                    # 후진 모델 — 아래 backup 시험용 (사이드스텝은 안 쓴다)
    lrn_b.fwd_speed["187"] = Ema(value=0.25, seed=0.25, lo=0.02, hi=1.5)
    lrn_b.fwd_startup["187"] = Ema(value=1.5, seed=1.5, lo=0.0, hi=5.0)
    assert plan(Fix(lateral_m=1.0, heading_deg=30.0), lrn_b).movement == "forward"   # 후진 모델이 있어도 앞으로
    assert plan(Fix(ok=False), lrn).why == "no_fix"
    print("계획: 전진만 · 작은 각 · 상한 180 확인")

    # 후진 계획 — 마지막 단계(태그컷에 서 있다)에서 한 번
    cut = limits.tag_cut_m()
    b = backup(Fix(lateral_m=0.30, forward_m=cut, lateral_sigma_m=0.05), lrn)    # 필요 0.65 > 뒤공간 0.5
    assert b.why == "no_space" and b.distance_m == C.BACK_MAX_M and b.need_m > C.BACK_MAX_M, b
    b = backup(Fix(lateral_m=0.30, forward_m=cut, lateral_sigma_m=0.05), lrn, back_space_m=2.0)
    assert b.ok and abs(b.distance_m - b.need_m) < 1e-9 and b.floor_m == 0.0, b   # 후진 모델 없으면 최소걸음 0
    print("태그컷에서 좌우 0.30 m → 후진 %.3f m (좌우의 %.3f 배 = 1/(K·tan 경로각상한))" % (b.need_m, b.need_m / 0.30))
    assert abs(b.need_m / 0.30 - 1.0 / (C.CORRIDOR_K * math.tan(math.radians(limits.path_angle_max_deg(cut))))) < 1e-9
    assert backup(Fix(lateral_m=0.08, forward_m=cut, lateral_sigma_m=0.05), lrn).why == "uncertain"   # 2σ = 0.10 > 0.08
    assert backup(Fix(lateral_m=0.30, forward_m=cut, lateral_sigma_m=float("inf")), lrn).why == "uncertain"
    lrn_b.fwd_tau["187"] = Ema(value=0.35, seed=0.35, lo=0.05, hi=2.0)          # 후진 관성을 배웠으면 최소걸음이 생긴다
    b = backup(Fix(lateral_m=0.02, forward_m=cut, lateral_sigma_m=0.005), lrn_b, back_space_m=2.0)
    assert b.ok and b.floor_m > b.need_m > 0 and abs(b.distance_m - b.floor_m) < 1e-9, b
    b = backup(Fix(lateral_m=0.02, forward_m=cut, lateral_sigma_m=0.005), lrn_b, back_space_m=0.05)
    assert b.why == "no_space", b                                                  # 최소걸음 > 뒤공간 — 서기 전에 뒤를 친다
    assert backup(Fix(ok=False), lrn).why == "no_fix"
    assert backup(Fix(lateral_m=0.1, forward_m=8.0, lateral_sigma_m=0.01), lrn).why == "inside"
    print("후진 계획 확인")

    # (3) 2차원 세계에서 실행
    DT = 0.005

    class VClock:
        t = 2000.0

    class World:
        """태그 좌표계. lat +가 왼쪽, heading +가 반시계. 전진하면 lat += d sin(h), 거리는 d cos(h) 준다."""
        def __init__(self, lat, fwd, heading):
            self.lat, self.fwd, self.h = lat, fwd, heading
            self.w = self.v = 0.0
            self.movement, self._t_set = "stop", VClock.t

        def step(self, movement):
            if movement != self.movement:
                self.movement, self._t_set = movement, VClock.t
            since = VClock.t - self._t_set
            want_w = want_v = 0.0
            if movement.startswith("rotate") and since >= 1.082:
                want_w = 12.01 * (1 if movement == "rotate_left_slow" else -1)
            if movement in ("forward", "backward") and since >= 1.4977:
                want_v = 0.28973 * (1 if movement == "forward" else -1)
            self.w += (want_w - self.w) * (1 - math.exp(-DT / 0.283))
            self.v += (want_v - self.v) * (1 - math.exp(-DT / 0.352))
            self.h += self.w * DT
            d = self.v * DT
            self.lat += d * math.sin(math.radians(self.h))
            self.fwd -= d * math.cos(math.radians(self.h))

    class FakeDriver:
        def __init__(self):
            self.movement, self.deadline, self.hold = "stop", None, None

        def set(self, m, why=""):
            assert self.deadline is not None, "set 전에 lease"
            self.movement = m

        def stop(self, why=""):
            self.movement = "stop"

        stop_now = stop

        def lease(self, s):
            self.deadline = VClock.t + s

        def lease_expired(self):
            return self.deadline is not None and VClock.t >= self.deadline

        def clear_lease(self):
            self.deadline = None

        def arm_rotation_timeout(self, m, hold_s):
            self.hold = hold_s

        def set_rotate_strength(self, fine):
            assert self.movement == "stop"                # 서 있을 때만 강도를 바꾼다
            self.fine = bool(fine)

    class FakeGyro(Gyro):
        def __init__(self, world):
            Gyro.__init__(self, hz=200)
            self.world = world
        angle_deg = property(lambda s: s.world.h + 123.4)      # 영점은 임의 — 차이만 쓴다
        rate_dps = property(lambda s: s.world.w)
        alive = property(lambda s: True)
        calibrated = property(lambda s: True)
        noise_dps = property(lambda s: 0.1)

        def stats(self):
            return {"n": 0, "gaps": 0, "hz": 200.0}

        def tick(self):
            self.intervals.append(DT * 1000)
            if self._rot is not None:
                self._judge(VClock.t)

    for lat0, h0, back_model in ((1.2, -10.0, False), (-0.9, 15.0, False), (1.0, 30.0, True)):
        world = World(lat0, 7.0, h0)
        drv, gy = FakeDriver(), FakeGyro(world)

        def vsleep(s, world=world, drv=drv, gy=gy):
            for _ in range(max(1, int(round(s / DT)))):
                VClock.t += DT
                world.step(drv.movement)
                gy.tick()
        clock.now = lambda: VClock.t
        time.sleep = vsleep
        lrn = Learner(seeds=M.seeds())
        if back_model:
            lrn.fwd_speed["187"] = Ema(value=0.28973, seed=0.28973, lo=0.02, hi=1.5)
            lrn.fwd_startup["187"] = Ema(value=1.4977, seed=1.4977, lo=0.0, hi=5.0)
        p = plan(Fix(lateral_m=world.lat, heading_deg=world.h, forward_m=world.fwd), lrn)
        assert p.ok, p
        r = execute(drv, gy, lrn, p, log=quiet)
        print("  좌우 %+.2f 방향 %+.1f → %s: 돌기 %+.1f(실제 %+.2f) · %s %.2f m · 돌기 %+.2f → 좌우 %+.3f 방향 %+.2f (%s)"
              % (lat0, h0, "ok" if r.ok else r.why, p.turn1_deg, r.turn1.turned_deg, p.movement,
                 p.drive_m, r.turn2.turned_deg, world.lat, world.h, "%.0fs" % (r.t_end - r.t_start)))
        assert r.ok, r.why
        assert abs(world.h) < 1.5, world.h                                    # 다시 법선을 본다
        assert abs(r.heading_end_deg - world.h) < 1e-6                        # 자이로 추정 = 실제
        assert abs(abs(world.lat) - abs(lat0) * (1 - C.SIDESTEP_FRACTION)) < 0.12, (lat0, world.lat)
        assert world.lat * lat0 > 0                                           # 지나쳐 반대편으로 가지 않았다
        assert drv.hold is not None and drv.hold <= C.SIDESTEP_ROT_SAFETY_S
        assert drv.deadline is None and drv.movement == "stop"
    print("sidestep 자체 시험 통과")
