"""GyroYaw 위에 두 가지를 얹는다 — 최신 가속도 보관, 회전 정지 판정.

적분·바이어스·중력축 투영·끊김은 전부 부모 imu_yaw.GyroYaw 가 한다 — **그게 실제로 도는 코드다.**
다만 `GyroYaw(...)` 를 직접 만들지는 않는다. 항상 이 파일의 `Gyro(...)` 를 만들고,
그러면 부모 코드가 그 안에서 돈다 (imu_yaw.py 는 v3 원본 그대로 둬서 diff 가 되게 한다).
정지 판정을 **콜백 안에서** 하는 이유: 메인 루프가 밀려도 제때 멈추려고 (plan 4-2-1).
"""
import time
from collections import deque
from dataclasses import dataclass, field

from .imu_yaw import GyroYaw


@dataclass(frozen=True)
class Rotation:
    """한 번의 회전 목표. 통째로 갈아끼운다 — 필드를 따로 고치면 중간 상태가 생긴다."""
    target_deg: float           # 부호 포함. +가 반시계
    tau_s: float                # 관성 시간 (배운 값)
    residual_deg: float = 0.0   # 조건별로 배운 잔여 보정
    period_s: float = 0.005     # 판정 주기. 자이로 200 Hz 면 5 ms
    start_angle: float = 0.0
    t_cmd: float = 0.0
    max_deg: float = 0.0        # 자이로가 죽어도 이보다 더 돌지 않는다


@dataclass
class RotationResult:
    target_deg: float = 0.0
    turned_deg: float = 0.0
    turned_at_stop: float = 0.0
    coast_deg: float = 0.0
    omega_at_stop: float = 0.0
    t_cmd: float = 0.0
    t_onset: float = 0.0
    t_stop_cmd: float = 0.0
    t_settled: float = 0.0
    reason: str = ""            # predicted / crossed / max_angle / gyro_stale
    tau_observed: float = 0.0
    gaps_during: int = 0

    @property
    def done(self):
        return self.t_settled > 0.0

    @property
    def ok(self):
        """제대로 끝난 회전. 배우는 것도 다음 걸음도 이것만 믿는다."""
        return self.done and self.reason == "predicted"


class Gyro(GyroYaw):
    # 판정 문턱. apriltag_v3/src/models/control/rot_control.py 의 실측 근거 값들
    ONSET_DEG = 0.15            # 이만큼 움직이면 "출발했다"
    WRONG_WAY_DEG = 5.0         # 반대로 이만큼 돌면 부호가 뒤집힌 것 → 즉시 정지
    SETTLE_K = 3.0              # 멎음 판정 = 보정 때 잰 잡음 x 이 값
    SETTLE_FLOOR_DPS = 0.5      # 그 판정의 하한. 광운대 COARSE_STABLE_RATE_DEG_S.
                                # 1.0 이면 아직 도는 중에 멎었다고 본다 (꼬리 0.3도)
    SETTLE_CEIL_DPS = 2.0       # 그 판정의 상한. 광운대 COARSE_BIAS_MAX_RATE_SPREAD_DEG_S
    SETTLE_MIN_S = 0.2          # 끊은 뒤 최소 대기 (명령 반영 지연)
    SETTLE_MAX_S = 2.0          # 멎기 대기 상한

    def __init__(self, **kw):
        super().__init__(**kw)
        self.accel = None                       # 최신 중력 방향. 태그 기울기 보정 (plan 5-3)
        self.intervals = deque(maxlen=4000)     # 처리 간격 — GIL 대기를 여기서 본다
        self._mono_last = None
        self._rot = None
        self._stop_fn = None
        self._result = None
        self._gaps0 = 0

    # ── SDK 콜백 ────────────────────────────────────────────────────
    def _on_frame(self, f):
        super()._on_frame(f)
        try:
            if not f.is_motion_frame():
                return
            m = f.as_motion_frame()
            kind = m.get_profile().stream_type().name
        except Exception:
            return
        if kind == "accel":
            # GyroYaw 는 보정 중에만 가속도를 모은다. 우리는 항상 최신값이 필요하다
            v = m.get_motion_data()
            self.accel = (v.x, v.y, v.z)
            return
        if kind != "gyro":
            return
        now = time.monotonic()
        if self._mono_last is not None:
            self.intervals.append((now - self._mono_last) * 1000.0)
        self._mono_last = now
        if self._rot is not None:
            self._judge(now)

    # ── 회전 ────────────────────────────────────────────────────────
    def arm(self, rot: Rotation, stop_fn):
        """회전을 건다. stop_fn 은 콜백 안에서 불린다 — 변수 하나 바꾸는 수준이어야 한다."""
        self._result = RotationResult(target_deg=rot.target_deg, t_cmd=rot.t_cmd)
        self._gaps0 = self.stats().get("gaps", 0)
        self._stop_fn = stop_fn
        self._rot = rot                          # 마지막에 건다. 먼저 걸면 미완성 상태를 본다

    def disarm(self):
        self._rot, self._stop_fn = None, None

    def result(self):
        return self._result

    def _judge(self, now):
        rot, res = self._rot, self._result
        if rot is None or res is None:
            return
        if not self.alive:
            self._cut(now, "gyro_stale", self.angle_deg - rot.start_angle)
            return
        turned = self.angle_deg - rot.start_angle
        if res.t_onset == 0.0 and abs(turned) >= self.ONSET_DEG:
            res.t_onset = now
        if res.t_stop_cmd == 0.0:
            remaining = abs(rot.target_deg) - abs(turned)
            lead = abs(self.rate_dps) * (rot.tau_s + rot.period_s / 2) + rot.residual_deg
            lead = min(lead, abs(rot.target_deg) / 2)   # 작은 각에서 시작하자마자 끊는 걸 막는다
            if remaining <= lead:
                self._cut(now, "predicted", turned)
            elif turned * rot.target_deg < 0 and abs(turned) >= self.WRONG_WAY_DEG:
                self._cut(now, "wrong_way", turned)      # 부호가 뒤집혔다
            elif rot.max_deg > 0 and abs(turned) >= rot.max_deg:
                self._cut(now, "max_angle", turned)
            return
        # 끊은 뒤 — 조용해지면 끝. 문턱을 보정 때 잰 잡음에서 뽑는다 (고정값보다 낫다)
        waited = now - res.t_stop_cmd
        quiet = min(self.SETTLE_CEIL_DPS,
                    max(self.SETTLE_FLOOR_DPS, (self.noise_dps or 0.3) * self.SETTLE_K))
        settled = waited > self.SETTLE_MIN_S and abs(self.rate_dps) < quiet
        if settled or waited > self.SETTLE_MAX_S:
            # 문턱을 넘은 순간엔 아직 잡음만큼 돌고 있다. 그 **남은 꼬리**를 모델로 채운다.
            # 안 채우면 실제보다 적게 적히고(3도/s·tau 0.3 이면 0.27도), 잔여보정이 그만큼
            # 반대로 배워서 차는 조용히 더 돈다. 관성이 지수면 꼬리 = 지금각속도 x tau 다.
            tail = self.rate_dps * rot.tau_s if settled else 0.0
            res.t_settled = now
            res.turned_deg = turned + tail
            res.coast_deg = res.turned_deg - res.turned_at_stop
            res.tau_observed = (abs(res.coast_deg) / abs(res.omega_at_stop)
                                if abs(res.omega_at_stop) > 0.2 else 0.0)
            res.gaps_during = self.stats().get("gaps", 0) - self._gaps0
            if not settled:
                res.reason = "settle_timeout"      # 아직 돌고 있다 — 믿지 마라
            self.disarm()

    def _cut(self, now, reason, turned):
        res = self._result
        res.t_stop_cmd, res.reason = now, reason
        res.omega_at_stop, res.turned_at_stop = self.rate_dps, turned
        fn, self._stop_fn = self._stop_fn, None
        if fn is not None:
            try:
                fn()
            except Exception:
                pass
        if reason == "gyro_stale":
            res.t_settled = now
            res.turned_deg = turned
            self.disarm()

    # ── 품질 ────────────────────────────────────────────────────────
    def quality(self):
        import statistics
        xs = sorted(self.intervals)
        st = self.stats()
        p99 = xs[int(0.99 * (len(xs) - 1))] if xs else None
        st["process_interval_ms"] = {          # 5 ms 를 크게 넘으면 GIL 대기다
            "median": round(statistics.median(xs), 2) if xs else None,
            "p99": round(p99, 2) if p99 else None,
            "max": round(xs[-1], 2) if xs else None}
        return st
