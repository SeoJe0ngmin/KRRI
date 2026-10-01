"""주행하며 배운다. 세 층을 **따로** 둔다 (plan 4-2 · 6절).

    tau        관성 시간.  물리라 모드를 안 탄다        배운다, 세션 간 이어간다
    판정주기    지금 잰 값                              잰다, 저장 안 한다
    잔여보정    설명 안 되는 나머지                      배운다, **모드별로 따로**

하나로 뭉쳐 "최종 오차가 0 이 되도록" 배우면 화면·기록 같은 그때 사정까지 tau 가 흡수해서
조건이 바뀌면 안 맞는다. 각속도 배율은 광운대 kwu/adaptive_slope.SessionSlope 를 그대로 쓴다.
"""
import json
import math
from dataclasses import dataclass, field
from pathlib import Path

from ..kwu.adaptive_slope import SessionSlope

# ── 씨앗 — 광운대 실측(조향 강도 30). 우리가 배우기 전까지 쓸 **출발점**이다. 학습이 덮어쓰고, 끝나면 seeds.json 으로
#    다음 주행에 넘긴다 (2026-10-02 부터 measured.json 없이 이렇게만). 강도를 30 으로 맞춘 이유가 이걸 물려받기 위해서다
#    (2026-09-22 결정). 출처 project_backup_lean_20260921_174252/extracted/depth_cam/calib/fsm_v4/config.py
KWU_DEFLECTION = 30
KWU_SEED = {
    # ROT_RESPONSE_STARTUP_DELAY_SEC = 1.082 (+-0.105). 우리 실측 1.0~1.32 와 겹친다
    "rot_startup_s": {"L": 1.082, "R": 1.082},
    # ROT_RESPONSE_MAX_RATE_DEG_S = 12.01. 우리 강도 20 실측 8.2~8.8 과 방향이 맞는다
    "rot_rate_dps": {"L": 12.01, "R": 12.01},
    # 끊고 더 도는 시간 [s]. 그쪽 프로파일에서 뽑았다:
    #   유지 STOP_DELAY 0.352 s + 감속 12.01/(2*300) 0.020 s = 0.372 s
    #   되튐 보정 COAST_HEURISTIC_REDUCTION 1.07도 / 12.01 = 0.089 s 를 뺀다
    #   -> 0.283 s. (우리 강도 20 실측 순코스팅은 0.168 s 였다 — 강도를 타는 값이다)
    "rot_tau_s": {"L": 0.283, "R": 0.283},
    # 이보다 작은 각은 못 돈다 [도] — **유도값, 실측 아님**. 정지지연 동안 가속하며 도는 각
    # ½ x 13.06 x 0.352² = 0.81 로 계산했다(그쪽 fallback 세트). config ROT_FLOOR_DEG(calibrate 실측)가 있으면 그걸로
    "rot_floor_deg": 0.81,
    # 직진(byte 67) — 그쪽이 고른 delayed_linear 적합 (motion_trajectory.selected.json, 10회)
    #   거리 = 0.28973 x 유지시간 - 0.43394,  죽은시간 1.4977 s.  우리 9/7 실측 0.284 m/s 와 일치
    #   사이드스텝(눈감고 시간으로 갈 때)과 직진 lead 의 출발점
    # 후진(byte 187)은 **아무도 안 쟀다 — 직진과 같다고 가정**(2026-10-02 사용자 결정). 첫 후진에서 학습이 덮는다.
    #   후진은 마지막 단계 최후의 보루 한 번뿐이고 짧다(수십 cm) — 틀려도 물러난 뒤 다시 재서 접근한다
    "fwd_speed_mps": {"67": 0.28973, "187": 0.28973},
    "fwd_startup_s": {"67": 1.4977, "187": 1.4977},
}


def seeds(learned=None):
    """학습이 시작할 값 = 광운대 씨앗 위에 지난 주행이 남긴 seeds.json(있으면). 기하(회전중심·카메라 어긋난 각)는
    여기 없다 — 차마다 달라서 남의 값을 쓰면 안 되고, config 에 calibrate 실측을 적는다."""
    s = {k: dict(v) if isinstance(v, dict) else v for k, v in KWU_SEED.items()}
    for k, v in (learned or {}).items():
        if isinstance(v, dict):
            for kk, vv in v.items():
                if vv is not None:
                    s.setdefault(k, {})[kk] = vv
        elif v is not None and k in s:
            s[k] = v
    return s


def last_seeds(work_root):
    """가장 최근 seeds.json — runs/ · calibrate/ 어느 쪽이든 수정 시각이 늦은 것. (dict, path). 없으면 (None, None)."""
    cands = []
    for sub in ("runs", "calibrate"):
        cands += list((Path(work_root) / sub).glob("*/seeds.json"))
    if not cands:
        return None, None
    p = max(cands, key=lambda x: x.stat().st_mtime)
    try:
        return json.loads(p.read_text(encoding="utf-8")), p
    except Exception:
        return None, p


@dataclass
class Ema:
    """조금씩 갱신. 클램프는 이상한 값이 눌러앉는 걸 막는다."""
    value: float = 0.0
    n: int = 0
    alpha: float = 0.50
    lo: float = 0.0
    hi: float = float("inf")
    seed: float = 0.0
    samples: list = field(default_factory=list)

    def observe(self, x):
        if not math.isfinite(x):
            return self.value
        x = min(self.hi, max(self.lo, float(x)))
        # 첫 3 표본은 계수를 크게 — 물려받은 값이 오늘 안 맞으면 오래 끌려간다
        a = 0.5 if self.n < 3 else self.alpha
        self.value = x if self.n == 0 and self.seed == 0.0 else self.value + a * (x - self.value)
        self.n += 1
        self.samples.append(round(x, 5))
        del self.samples[:-50]
        return self.value

    @property
    def spread(self):
        if len(self.samples) < 2:
            return None
        m = sum(self.samples) / len(self.samples)
        return math.sqrt(sum((s - m) ** 2 for s in self.samples) / len(self.samples))

    def dump(self):
        return {"seed": round(self.seed, 5), "final": round(self.value, 5),
                "n": self.n, "spread": None if self.spread is None else round(self.spread, 5)}


class Learner:
    """회전·직진의 tau 와 잔여를 배운다. 모드는 잔여에만 붙는다."""

    #: 회전 tau 의 허용 범위 [s]. 물리적으로 말이 되는 구간 밖이면 안 받는다
    TAU_ROT = (0.05, 1.0)
    TAU_FWD = (0.05, 2.0)
    #: 회전 각속도 [도/s] 의 말 되는 범위. 상한은 광운대 ROT_RATE_LIMIT_DEG_S.
    #: 실측 기준점 — 강도 20 에서 8.2~8.8(우리 9/7·9/21), 강도 30 에서 12.0(광운대)
    RATE_ROT = (0.3, 60.0)
    #: 명령 -> 실제로 움직이기 시작 [s]. 실측 1.0~1.32(우리) · 1.082(광운대)
    STARTUP_ROT = (0.0, 5.0)
    #: 직진 속도 [m/s] 의 말 되는 범위. 9/7 실측 정속 0.28~0.30 (강도 67)
    SPEED_FWD = (0.02, 1.5)
    STARTUP_FWD = (0.0, 5.0)

    def __init__(self, mode="record", seeds=None):
        self.mode = mode
        s = seeds or {}
        self.rot_tau = {d: self._ema(s.get("rot_tau_s", {}).get(d), *self.TAU_ROT)
                        for d in ("L", "R")}
        self.fwd_tau = {k: self._ema(s.get("fwd_tau_s", {}).get(k), *self.TAU_FWD)
                        for k in ("67", "187")}
        self.rot_rate = {d: self._ema(s.get("rot_rate_dps", {}).get(d), *self.RATE_ROT)
                         for d in ("L", "R")}
        self.rot_startup = {d: self._ema(s.get("rot_startup_s", {}).get(d), *self.STARTUP_ROT)
                            for d in ("L", "R")}
        self.rot_residual = {d: self._ema(s.get("rot_residual_deg", {}).get(d), -2.0, 2.0)
                             for d in ("L", "R")}
        self.fwd_speed = {k: self._ema(s.get("fwd_speed_mps", {}).get(k), *self.SPEED_FWD)
                          for k in ("67", "187")}
        self.fwd_startup = {k: self._ema(s.get("fwd_startup_s", {}).get(k), *self.STARTUP_FWD)
                            for k in ("67", "187")}
        self.fwd_residual = self._ema(s.get("fwd_residual_m"), -0.2, 0.2)
        self.rate = SessionSlope()      # 각속도 배율. 광운대 것 그대로
        self.rot_floor_deg = s.get("rot_floor_deg")   # 배우지 않는다. 씨앗 또는 config ROT_FLOOR_DEG (calibrate 실측)
        self.rejected = 0

    @staticmethod
    def _ema(seed, lo, hi):
        v = float(seed) if seed is not None else 0.0
        return Ema(value=v, seed=v, lo=lo, hi=hi, n=0)

    # ── 회전 ────────────────────────────────────────────────────────
    def rotation(self, res):
        """회전 하나에서 배운다. 물리(tau)와 잔여를 **다른 곳에** 넣는다.

        필요한 시각은 전부 res 안에 있다 — 부르는 쪽이 따로 계산해 넘기지 않는다.
        """
        if not res or not res.done or res.reason != "predicted":
            self.rejected += 1
            return {}
        # 하한 아래 회전은 정지지연만큼 지나치는 게 정상이라, 배우면 잔여가 오염된다
        if self.rot_floor_deg and abs(res.target_deg) < self.rot_floor_deg:
            self.rejected += 1
            return {}
        side = "L" if res.target_deg > 0 else "R"
        out = {}
        if res.tau_observed > 0:
            out["tau"] = self.rot_tau[side].observe(res.tau_observed)
        out["residual"] = self.rot_residual[side].observe(
            abs(res.turned_deg) - abs(res.target_deg))
        if res.t_onset:
            out["startup"] = self.rot_startup[side].observe(res.t_onset - res.t_cmd)
        active = res.t_stop_cmd - res.t_onset if res.t_onset else 0.0
        if active > 0.1 and abs(res.turned_at_stop) > 0:
            out["rate"] = self.rot_rate[side].observe(abs(res.turned_at_stop) / active)
            base = self.rot_rate[side].seed
            if base > 0:
                # 광운대 배율 — 물려받은 각속도 대비 오늘이 얼마나 빠른가 (adaptive_slope.py)
                out["multiplier"] = self.rate.observe(
                    "ROT_LEFT" if side == "L" else "ROT_RIGHT", base,
                    res.t_onset - res.t_cmd, res.t_stop_cmd - res.t_cmd,
                    abs(res.turned_deg)).get("slope_next_multiplier")
        return out

    def rot_rate_dps(self, target_deg):
        """이 방향의 각속도 [도/s]. 아직 한 번도 안 돌았으면 None — 부르는 쪽이 대비한다."""
        return self._known(self.rot_rate["L" if target_deg > 0 else "R"])

    def rot_startup_s(self, target_deg):
        """명령 -> 실제 회전 시작 [s]. 모르면 None."""
        return self._known(self.rot_startup["L" if target_deg > 0 else "R"])

    @staticmethod
    def _known(e):
        return e.value if (e is not None and (e.n > 0 or e.seed > 0)) else None

    # ── 직진 ────────────────────────────────────────────────────────
    def forward(self, res):
        """직진 하나에서 배운다. 필요한 값은 전부 res 안에 있다.

        tau = (정지명령 때 남은거리 - 최종 남은거리) / 속도 - 검출 지연
        카메라는 "조금 전" 을 알려주므로 낡은 만큼이 그대로 섞여 들어온다. 아는 몫(검출 지연)만
        빼면 남는 건 `노출->도착 지연 + 진짜 관성`, 둘 다 상수다 (plan 4-10).
        """
        if not res or not res.ok:
            self.rejected += 1
            return {}
        key = str(int(res.strength))
        if key not in self.fwd_tau:
            return {}
        out = {}
        # travelled_at_stop_m 은 **마지막으로 본 시각**까지의 거리다. 시간도 거기에 맞춘다
        active = (res.t_stop_cmd - res.detect_age_s - res.t_onset) if res.t_onset else 0.0
        if active > 0.3 and res.travelled_m > 0:
            out["speed"] = self.fwd_speed[key].observe(res.travelled_at_stop_m / active)
        if res.t_onset:
            out["startup"] = self.fwd_startup[key].observe(res.t_onset - res.t_cmd)
        v = out.get("speed") or res.speed_mps
        if v and v > 0.02:
            out["tau"] = self.fwd_tau[key].observe(
                (res.d_at_stop_cmd - res.d_final) / v - max(0.0, res.detect_age_s))
        out["residual"] = self.fwd_residual.observe(res.travelled_m - res.target_m)
        return out

    def fwd_speed_mps(self, strength):
        """이 강도의 정속 [m/s]. 아직 못 쟀으면 None."""
        return self._known(self.fwd_speed.get(str(int(strength))))

    def fwd_startup_s(self, strength):
        return self._known(self.fwd_startup.get(str(int(strength))))

    def fwd_lead_m(self, strength, speed, detect_age_s, period_s):
        """지금 끊어야 할 남은 거리 [m] = 속도 x (검출지연 + tau + 판정주기/2) + 잔여."""
        e = self.fwd_tau.get(str(int(strength)))
        tau = e.value if e else 0.0
        return speed * (max(0.0, detect_age_s) + tau + period_s / 2) + self.fwd_residual.value

    # ── 저장 ────────────────────────────────────────────────────────
    def dump(self):
        """seed -> final 을 둘 다 남긴다. 많이 다르면 "매번 재야 하는 값" 이다 (plan 6-3 F)."""
        return {
            "mode": self.mode, "rejected": self.rejected,
            "rot_tau_s": {d: e.dump() for d, e in self.rot_tau.items()},
            "rot_rate_dps": {d: e.dump() for d, e in self.rot_rate.items()},
            "rot_startup_s": {d: e.dump() for d, e in self.rot_startup.items()},
            "rot_residual_deg": {d: e.dump() for d, e in self.rot_residual.items()},
            "fwd_tau_s": {k: e.dump() for k, e in self.fwd_tau.items()},
            "fwd_speed_mps": {k: e.dump() for k, e in self.fwd_speed.items()},
            "fwd_startup_s": {k: e.dump() for k, e in self.fwd_startup.items()},
            "fwd_residual_m": self.fwd_residual.dump(),
            "rate_multiplier": dict(self.rate.multipliers),
        }

    def seeds(self):
        """다음 세션이 물려받을 값. 수렴한 것만 넘긴다."""
        def pick(e):
            return round(e.value, 5) if e.n >= 3 else None
        return {"rot_tau_s": {d: pick(e) for d, e in self.rot_tau.items()},
                "rot_rate_dps": {d: pick(e) for d, e in self.rot_rate.items()},
                "rot_startup_s": {d: pick(e) for d, e in self.rot_startup.items()},
                "rot_residual_deg": {d: pick(e) for d, e in self.rot_residual.items()},
                "fwd_tau_s": {k: pick(e) for k, e in self.fwd_tau.items()},
                "fwd_speed_mps": {k: pick(e) for k, e in self.fwd_speed.items()},
                "fwd_startup_s": {k: pick(e) for k, e in self.fwd_startup.items()},
                "fwd_residual_m": pick(self.fwd_residual)}
