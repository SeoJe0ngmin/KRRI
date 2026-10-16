"""회전 하나. 거는 건 여기, **끊는 건 자이로 콜백**이다 (plan 4-2-1).

메인 루프가 화면·기록에 밀려도 정지 판정은 SDK 콜백 안에서 도니까 늦지 않는다.
여기는 세 가지만 한다: 돌아도 되는지 보고 · 안전망을 걸고 · 끝나면 배운다.

안전망이 세 겹이다 — 하나가 뚫려도 차는 선다:
  1) 자이로 콜백      목표각에서 끊는다 (정상 경로)
  2) 데드맨 lease     우리가 죽으면 CAN 스레드가 LEASE_S 안에 세운다
  3) arm_timed_rotation  전부 죽어도 CAN 스레드가 hold_s 에 세운다 (광운대 control.py L317)

상한 둘(cap_deg · safety_s)은 기본이 반화각·config 값이고, 사이드스텝(180)·탐색(50)·접근(화면 제한)은 부르는 쪽이 넘긴다.

v5 (2026-10-01 결정, plan 0절):
  · 여기서 배우지 않는다 — τ·잔여·출발지연·각속도는 학습 파일(config/learned/<장소>.json)이 주행 시작 때 정한 값으로 고정
    (Learner.set_rotation). 대신 배울 재료를 rotate_end 에 남긴다: 목적(purpose)·그때 쓴 τ·잔여·명령 직전 각속도·시작 각.
    되튐 뒤 각은 run.py 가 회전 뒤 1초 대기가 끝날 때 rotate_final 로 남긴다.
  · 끝나면 **무조건 정지를 한 번 더** 보낸다 (#1 ②) — 콜백의 정지가 회전 명령보다 먼저 나가 덮어써진 경우의 보험.
  · 기본 상한 = 반화각 (17.1 상수를 뺐다, R2).
"""
import statistics
import time

from config import control as C
from config import imu as I
from ... import limits
from ...utils import clock
from ...utils.gyro import Rotation, RotationResult

POLL_S = 0.01          # 기다리는 쪽의 확인 주기. 판정은 콜백이 하니 여긴 느슨해도 된다
LEASE_S = C.DEADMAN_S  # 이 안에 명령이 안 갱신되면 CAN 스레드가 세운다. 값은 config (플랫폼을 탄다 — VM 1.0 / 직결 0.30)
ABORT_FACTOR = 2.0     # 목표의 이 배를 넘으면 무조건 끊는다. 측정값이 아니라 중단 규칙이다


def min_turn_deg(learner, fine=False):
    """이보다 작은 각은 안 돈다 [도] — 정지지연 동안 도는 각. 돌려도 그만큼 지나친다.

    광운대의 2.5도(ROT_MIN_COMMANDABLE_ANGLE_DEG)는 우리 하한이 아니다 — 그쪽은 시간으로
    끊어 짧은 명령을 못 맞추고, 우리는 자이로를 보고 끊는다. 씨앗 0.81 은 유도값이고
    calibrate.py rotfloor 가 재서 config ROT_FLOOR_DEG 에 적는다. 못 쟀으면 None 이고 그때는 제한도 없다.

    약한 회전(fine)은 **하한의 절반**까지 받는다 (2026-10-01): 하한은 "움직이자마자 끊어도 도는 각의 최대" 라, 필요각이
    그 절반 이상이면 최대로 돌아도 |남는 오차| ≤ |지금 오차| 다. 마지막 구간은 돌고 나서 화면위치로 다시 재니 그걸로 충분하다.
    (하한을 그대로 걸면 태그컷에서 빗나감 30~38 mm 가 '넘었는데 못 고치는' 띠가 된다 — 0.84도 × 2.6 m)
    """
    if fine:
        return learner.rot_floor_fine_deg / 2.0 if learner.rot_floor_fine_deg else learner.rot_floor_fine_deg
    return learner.rot_floor_deg


def rotate(driver, gyro, learner, deg, rec=None, log=print, cap_deg=None, safety_s=None, fine=False, purpose="step"):
    """제자리로 deg 만큼 돈다. +가 반시계. 돌린 결과(RotationResult)를 준다.
    fine=True 면 약한 회전(ROTATE_FINE_JOYSTICK_DEFLECTION) — 사이드스텝·탐색 말고는 전부 이것 (v5 결정 R1).
    purpose = step / sidestep / search / calibrate — 기록에 남는다. 학습 파일은 step 만 배운다.

    cap_deg 를 넘는 요청은 거부한다(기본 반화각). safety_s 는 CAN 쪽 시계 안전망의
    상한(기본 ROT_SAFETY_MAX_S). 하한(rot_floor) 아래 요청도 거부 — 돌려봐야 지나친다.
    """
    deg = float(deg)
    cap = limits.half_fov_deg() if cap_deg is None else float(cap_deg)
    safety = C.ROT_SAFETY_MAX_S if safety_s is None else float(safety_s)
    side = learner.rot_side(deg, fine)
    movement = "rotate_left_slow" if deg > 0 else "rotate_right_slow"
    rate = learner.rot_rate_dps(deg, fine)
    used = {"purpose": purpose, "tau_used": learner.rot_tau[side].value, "residual_used": learner.rot_residual[side].value}

    refused = _refuse(gyro, learner, deg, cap, fine)
    if refused:
        log("       !! 회전 거부(%s): %+.2f도" % (refused, deg))
        res = RotationResult(target_deg=deg)
        res.reason = refused
        _record(rec, res, movement, rate, used, log, fine)
        return res

    driver.set_rotate_strength(fine)        # 서 있을 때 강도를 써넣고 8바이트를 확인한다 (frames.set_rotate_strength)
    hold_s = _hold_s(learner, deg, rate, safety, fine)
    driver.arm_rotation_timeout(movement, hold_s)
    pre = getattr(gyro, "recent_rate_dps", None)
    used["pre_rate_dps"] = pre() if pre else None              # 명령 직전 각속도 — 덜 멎은 채 시작한 회전은 학습에서 뺀다
    used["gyro_scale"] = getattr(gyro, "scale", 1.0)
    rot = Rotation(target_deg=deg,
                   tau_s=used["tau_used"],
                   residual_deg=used["residual_used"],
                   period_s=_period_s(gyro),
                   start_angle=gyro.angle_deg,
                   t_cmd=clock.now(),
                   max_deg=abs(deg) * ABORT_FACTOR)
    gyro.arm(rot, driver.stop_now)          # 명령보다 **먼저** 건다. 안 보는 구간을 안 만든다
    log("       -> 회전%s %+7.2f도  (안전망 %.1fs, 판정주기 %.1fms)"
        % ("(약)" if fine else "", deg, hold_s, rot.period_s * 1000))
    driver.lease(LEASE_S)                   # 명령보다 먼저 — set 직후 죽어도 데드맨이 세운다
    try:
        driver.set(movement, why="rotate %+.2f도" % deg)
    except BaseException:
        gyro.disarm()
        driver.stop("회전 명령 실패")
        driver.clear_lease()
        raise

    fault = _wait(driver, gyro, rot, hold_s)
    res = gyro.result() or RotationResult(target_deg=deg, start_angle=rot.start_angle)
    if fault:
        driver.stop("회전 " + fault)        # 콜백이 못 끊었다 — 여기서 끊는다
        gyro.disarm()
        res.reason = res.reason or fault
        if not res.done:
            res.turned_deg = gyro.angle_deg - rot.start_angle
    # 무조건 한 번 더 (#1 ②). 콜백의 stop_now 가 회전 명령보다 먼저 나갔다면 회전 명령이 그걸 덮어써 켜진 채다 —
    # 그때는 자이로 감시도 lease 도 끝났다. 직진(forward)은 원래 이렇게 끝낸다
    driver.stop("회전 끝 " + (res.reason or "?"))
    driver.clear_lease()

    _record(rec, res, movement, rate, used, log, fine)
    if res.ok:
        log("          끝: %+.2f도 요청 -> %+.2f도 (관성 %+.2f도, %.2fs)"
            % (deg, res.turned_deg, res.coast_deg, res.t_settled - res.t_cmd))
    else:
        log("          !! 회전 실패(%s): %+.2f도 중 %+.2f도에서 멈춤"
            % (res.reason, deg, res.turned_deg))
        if res.reason == "wrong_way":
            log("             CAN 회전 부호가 뒤집혔다 — driver.MOVES 의 rotate 두 줄을 맞바꿔라")
    return res


# ── 속 ──────────────────────────────────────────────────────────────
def _refuse(gyro, learner, deg, cap, fine=False):
    """돌기 전에 막는다. 돌다가 막는 것보다 싸다."""
    if abs(deg) < 1e-6:
        return "zero"
    if not gyro.calibrated:
        return "gyro_uncalibrated"
    if not gyro.alive:
        return "gyro_stale"
    if abs(deg) > cap:
        return "too_big"
    floor = min_turn_deg(learner, fine)
    if floor and abs(deg) < floor:
        return "too_small"          # 정지지연만큼 지나친다. 회전 대신 좌우로 비켜선다 (plan 3-6)
    return ""


def _hold_s(learner, deg, rate, safety, fine=False):
    """CAN 스레드가 회전을 쥐고 있을 최대 시간 [s]. 못 쟀으면 사람이 정한 상한."""
    startup = learner.rot_startup_s(deg, fine)
    if not rate or startup is None:
        return safety
    return min(safety, (abs(deg) / rate + startup) * ABORT_FACTOR)


def _period_s(gyro):
    """판정 주기 = 자이로가 실제로 오는 간격. **지금 잰다** — 저장하지 않는다 (plan 4-2)."""
    xs = list(gyro.intervals)[-200:]
    if len(xs) >= 20:
        return statistics.median(xs) / 1000.0
    return 1.0 / I.IMU_GYRO_HZ


def _wait(driver, gyro, rot, hold_s):
    """끝나기를 기다린다. 콜백이 판정하므로 여기서 하는 일은 감시뿐."""
    deadline = rot.t_cmd + hold_s + gyro.SETTLE_MAX_S + LEASE_S
    while True:
        res = gyro.result()
        if res is None or res.done:
            return ""
        now = clock.now()
        if now > deadline:
            return "watchdog"
        if not gyro.alive:
            return "gyro_stale"     # 프레임이 안 오면 콜백도 판정을 못 한다
        if driver.lease_expired():
            return "lease"
        driver.lease(LEASE_S)       # 우리가 살아있다는 신호
        time.sleep(POLL_S)


def _record(rec, res, movement, rate, used, log, fine=False):
    """기록이 주행을 막지 않는다 (plan 6-6 ④). 못 적으면 시끄럽게 알리고 계속 간다.
    used = 목적·그때 쓴 τ·잔여·명령 직전 각속도·눈금 — 학습 파일(learnfile)이 이걸로 "기준값으로 끊었다면" 을 역산한다."""
    if rec is None:
        return
    try:
        rec.event("rotate_end", movement=movement, fine=fine,
                  deflection=(C.ROTATE_FINE_JOYSTICK_DEFLECTION if fine else C.ROTATE_JOYSTICK_DEFLECTION),
                  ok=res.ok, reason=res.reason,
                  target_deg=res.target_deg, turned_deg=res.turned_deg,
                  turned_at_stop=res.turned_at_stop, coast_deg=res.coast_deg,
                  omega_at_stop=res.omega_at_stop, rate_model_dps=rate,
                  tau_observed=res.tau_observed, gaps=res.gaps_during,
                  start_angle_deg=res.start_angle,
                  t_cmd=res.t_cmd, t_onset=res.t_onset,
                  t_stop_cmd=res.t_stop_cmd, t_settled=res.t_settled,
                  **(used or {}))
    except Exception as e:
        log("          !! 회전 기록 실패: %s (주행은 계속한다)" % e)


if __name__ == "__main__":
    # 자체 시험 — 하드웨어 없이. 가상 시계로 돈다: time.sleep 이 자이로 콜백을 5 ms 씩 밀어준다.
    # 차 모델은 광운대 씨앗(출발 1.08 s · 12 도/s · 관성 0.283 s)을 1차 지연으로 흉내낸다.
    import math
    from collections import deque
    from src.models.control import learn as M
    from ...utils.gyro import Gyro
    from .learn import Learner

    class VClock:
        t = 1000.0

    class FakeDriver:
        def __init__(self):
            self.movement, self.deadline, self.log = "stop", None, []

        def set(self, m, why=""):
            assert self.deadline is not None, "set 전에 lease 가 걸려 있어야 한다"
            self.movement = m
            self.log.append((VClock.t, m))

        def stop(self, why=""):
            self.movement = "stop"
            self.log.append((VClock.t, "stop:" + why))

        def stop_now(self):
            self.movement = "stop"
            self.log.append((VClock.t, "stop_now"))

        def lease(self, s):
            self.deadline = VClock.t + s

        def lease_expired(self):
            return self.deadline is not None and VClock.t >= self.deadline

        def clear_lease(self):
            self.deadline = None

        def arm_rotation_timeout(self, m, hold_s):
            self.hold = hold_s

        def set_rotate_strength(self, fine):
            assert self.movement == "stop", "서 있을 때만 강도를 바꾼다"
            self.fine = bool(fine)

    class FakeGyro(Gyro):
        """GyroYaw 의 SDK 부분을 건너뛰고 _judge 만 그대로 쓴다."""
        DT = 0.005

        def __init__(self, drv, rate_dps=12.01, startup_s=1.082, tau_s=0.283):
            Gyro.__init__(self, hz=200)
            self.drv, self.rate_cmd, self.startup, self.tau = drv, rate_dps, startup_s, tau_s
            self._a, self._w, self._t_set, self._m = 0.0, 0.0, None, "stop"
            self.calibrated_fake = True

        angle_deg = property(lambda s: s._a)
        rate_dps = property(lambda s: s._w)
        alive = property(lambda s: True)
        calibrated = property(lambda s: s.calibrated_fake)
        noise_dps = property(lambda s: 0.1)

        def stats(self):
            return {"n": 0, "gaps": 0, "hz": 200.0}

        def step(self):
            m = self.drv.movement
            if m != self._m:
                self._m, self._t_set = m, VClock.t
            want = 0.0
            if m.startswith("rotate") and VClock.t - self._t_set >= self.startup:
                want = self.rate_cmd * (1 if m == "rotate_left_slow" else -1)
            # 1차 지연: 관성시간 tau 로 목표 각속도에 붙는다 (정지도 같은 tau 로 죽는다)
            self._w += (want - self._w) * (1 - math.exp(-self.DT / self.tau))
            self._a += self._w * self.DT
            self.intervals.append(self.DT * 1000)
            if self._rot is not None:
                self._judge(VClock.t)

    drv = FakeDriver()
    gy = FakeGyro(drv)

    def vsleep(s):
        n = max(1, int(round(s / FakeGyro.DT)))
        for _ in range(n):
            VClock.t += FakeGyro.DT
            gy.step()
    clock.now = lambda: VClock.t
    time.sleep = vsleep

    lrn = Learner(seeds=M.seeds())
    quiet = lambda *_: None                                                       # noqa: E731
    # 1) 기본 상한 = 반화각(35.1도) — 넘으면 거부, 하한(씨앗 0.81) 아래도 거부
    half = limits.half_fov_deg()
    assert rotate(drv, gy, lrn, half + 1.0, log=quiet).reason == "too_big"
    assert rotate(drv, gy, lrn, 0.5, log=quiet).reason == "too_small"
    assert rotate(drv, gy, lrn, 0.0, log=quiet).reason == "zero"
    # 2) 보통 회전 — 예측 정지로 목표 근처에 선다. 끝은 늘 정지 명령 (#1 ②)
    for deg in (10.0, -10.0, 5.0, 30.0):
        r = rotate(drv, gy, lrn, deg, log=quiet)
        assert r.ok, (deg, r.reason)
        assert abs(r.turned_deg - deg) < 1.0, (deg, r.turned_deg)
        assert drv.movement == "stop" and drv.log[-1][1].startswith("stop:회전 끝"), drv.log[-3:]
        print("  %+6.1f도 요청 -> %+6.2f도 (관성 %.2f도, tau %.3f s)"
              % (deg, r.turned_deg, r.coast_deg, r.tau_observed))
    assert drv.deadline is None                       # 끝나면 lease 를 지운다
    # 3) 사이드스텝용: 90도 는 cap_deg 를 올려야 통과하고, safety_s 가 hold 상한이 된다
    assert rotate(drv, gy, lrn, 90.0, log=quiet).reason == "too_big"
    r = rotate(drv, gy, lrn, 90.0, log=quiet, cap_deg=180.0, safety_s=C.SIDESTEP_ROT_SAFETY_S, purpose="sidestep")
    assert r.ok and abs(r.turned_deg - 90.0) < 1.5, (r.reason, r.turned_deg)
    assert drv.hold <= C.SIDESTEP_ROT_SAFETY_S and drv.hold > C.ROT_SAFETY_MAX_S
    print("  +90.0도 요청 -> %+6.2f도 (안전망 %.1fs)" % (r.turned_deg, drv.hold))
    # 3-b) 약한 회전(fine) — 강도를 바꿔 건다. 가짜 차를 9/21 강도 20 실측(8.47 도/s · 관성 0.168 s)으로 바꿔 본다
    gy.rate_cmd, gy.startup, gy.tau = 8.47, 1.0, 0.168
    before = {k: (e.value, e.n) for k, e in lrn.rot_tau.items()}
    for deg in (2.0, -1.0, 0.5, 30.0):
        r = rotate(drv, gy, lrn, deg, log=quiet, fine=True)
        assert r.ok and drv.fine is True, (deg, r.reason)
        print("  (약) %+5.2f도 요청 -> %+6.2f도 (관성 %.2f도)" % (deg, r.turned_deg, r.coast_deg))
        assert abs(r.turned_deg - deg) < 0.6, (deg, r.turned_deg)
    assert {k: (e.value, e.n) for k, e in lrn.rot_tau.items()} == before        # 주행 중엔 안 배운다 (v5 — 학습 파일이 맡는다)
    assert rotate(drv, gy, lrn, 0.05, log=quiet, fine=True).reason == "too_small"  # 약한 회전의 최소 요청 = 하한(씨앗 0.20)의 절반
    assert rotate(drv, gy, lrn, 0.12, log=quiet, fine=True).reason != "too_small"
    r = rotate(drv, gy, lrn, 5.0, log=quiet)                                      # 다시 거친 회전 — 강도가 되돌아간다
    assert drv.fine is False
    # 3-c) 학습 파일 값 — 좌우가 같은 값, 강도별로 따로
    lrn.set_rotation({False: {"tau_s": 0.1, "residual_deg": 1.5, "startup_s": 0.96, "rate_dps": 6.0},
                      True: {"tau_s": 0.05, "residual_deg": 1.25, "startup_s": None, "rate_dps": None}})
    assert lrn.rot_tau["L"].value == lrn.rot_tau["R"].value == 0.1 and lrn.rot_residual["Rf"].value == 1.25
    assert lrn.rot_rate_dps(-3.0, True) == 8.47                                 # None 이면 씨앗 그대로
    # 4) 자이로 미보정이면 거부
    gy.calibrated_fake = False
    assert rotate(drv, gy, lrn, 5.0, log=quiet).reason == "gyro_uncalibrated"
    print("rotate 자체 시험 통과")
