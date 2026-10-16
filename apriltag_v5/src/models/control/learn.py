"""주행하며 배운다. 세 층을 **따로** 둔다 (plan 4-2 · 6절).

    tau        관성 시간.  물리라 모드를 안 탄다        배운다, 세션 간 이어간다
    판정주기    지금 잰 값                              잰다, 저장 안 한다
    잔여보정    설명 안 되는 나머지                      배운다, **모드별로 따로**

하나로 뭉쳐 "최종 오차가 0 이 되도록" 배우면 화면·기록 같은 그때 사정까지 tau 가 흡수해서
조건이 바뀌면 안 맞는다. 각속도 배율은 광운대 kwu/adaptive_slope.SessionSlope 를 그대로 쓴다.

v5 (2026-10-01·02 결정, plan 0절): **주행 중에는 아무것도 배우지 않는다.** 주행 중 계수 0.5 로 고치던 학습은 "이번에 넘친 양" 을
배워 직전 하나의 절반만 기억했다. 회전 응답(τ·잔여·출발지연·각속도)과 직진(속도·죽은시간·τ·잔여)은 학습 파일
config/learned/<장소>.json(learnfile)이 주행을 **시작할 때** 기록에서 걸러 쌓고, 이 주행 내내 그 값이 고정이다 —
set_rotation() · set_forward(). 그래서 seeds.json(주행이 끝날 때 다음 주행에 넘기던 값)은 없앴다 (2026-10-02).
이 파일의 씨앗은 학습 파일에 자료가 없을 때만 쓰인다.
"""
import math
from dataclasses import dataclass, field

from ..kwu.adaptive_slope import SessionSlope

# ── 씨앗 — 광운대 실측(조향 강도 30). 학습 파일에 자료가 없을 때 쓸 **출발점**이다 (자료가 MIN_N 번 쌓이면 학습 파일 값).
#    강도를 30 으로 맞춘 이유가 이걸 물려받기 위해서다 (2026-09-22 결정).
#    출처 project_backup_lean_20260921_174252/extracted/depth_cam/calib/fsm_v4/config.py
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
    #   시간 직진(사이드스텝 옆 이동 · 눈 감고 진입)의 출발점. 우리 자료(2026-10-01 카메라 직진 24번)는 0.290 m/s · 1.20 s —
    #   학습 파일에 MIN_N 번 쌓이면 그 값을 쓴다 (learnfile straight)
    # 후진(byte 187)은 **아무도 안 쟀다 — 직진과 같다고 가정**(2026-10-02 사용자 결정). 후진 자료가 MIN_N 번 쌓일 때까지
    #   학습 파일도 전진 값을 쓴다. 후진은 마지막 단계 최후의 보루 한 번뿐이고 짧다(수십 cm) — 틀려도 물러난 뒤 다시 재서 접근한다
    "fwd_speed_mps": {"67": 0.28973, "187": 0.28973},
    "fwd_startup_s": {"67": 1.4977, "187": 1.4977},
}


#: 약한 회전(강도 20) 씨앗 — **우리 차 9/21 실측**(apriltag_v3/work_dirs/20260921_측정기록.json, 강도 20):
#: 1.0 s 무반응 → 정속 8.19~8.75 도/s, 끊고 나서 0.168 s × 각속도 만큼 더 돈다. 열쇠는 방향 + "f"(fine)
FINE_SEED = {
    "rot_startup_s": {"Lf": 1.0, "Rf": 1.0},
    "rot_rate_dps": {"Lf": 8.47, "Rf": 8.47},
    "rot_tau_s": {"Lf": 0.168, "Rf": 0.168},
    # 하한 [도] — **유도값, 실측 아님**: 출발 문턱 0.15(gyro.ONSET_DEG) + 정지 유지 0.15 s 동안 가속 4.7 도/s² 로 도는 ½·4.7·0.15² = 0.05.
    # calibrate rotfloor 실측(2026-10-01)은 최대 0.84 · 되튐 뒤 0.04~0.5도 — config ROT_FLOOR_FINE_DEG 가 있으면 그걸 쓴다
    "rot_floor_fine_deg": 0.20,
}
#: 직진 관성 기준 [s] — **우리 실측** 2026-10-01 카메라 직진 24번(시작 측정 버그 1번 뺌, 멈춤 확인 실패 포함)의 중앙.
#: 정지 명령부터 멈출 때까지 미끄러진 시간 = (정지 명령 때 마지막으로 본 남은거리 − 최종 남은거리) ÷ 속도 − 그 프레임의 나이.
#: 나이를 빼는 이유: 끊는 식(fwd_lead_m)이 "마지막으로 본 뒤 간 거리" 를 속도 × 나이로 **따로** 더한다 — 안 빼면(0.48 s) 두 번 센다.
#: (오늘 자료로 0.48 을 기준으로 쓰면 자료 없이 끊을 때 평균 −4.4 cm 덜 가고, 0.295 면 +0.7 cm.) 후진은 안 쟀다 — 전진과 같다고 본다.
#: 학습 파일이 끊는 값을 역산하는 기준이자, 자료가 없을 때 쓰는 값이다 (learnfile.FWD_TAU_REF_S)
FWD_SEED = {"fwd_tau_s": {"67": 0.295, "187": 0.295}}
ROT_SIDES = ("L", "R", "Lf", "Rf")
FWD_KEYS = ("67", "187")


def seeds():
    """학습기가 시작할 값 = 광운대 씨앗 + 약한 회전 씨앗(우리 9/21) + 직진 관성 기준(우리 10/1). 주행은 이 위에 학습 파일 값을
    덮는다 — 회전 Learner.set_rotation(), 직진 Learner.set_forward(). 기하(회전중심·카메라 어긋난 각)는 여기 없다 — 차마다 달라서
    남의 값을 쓰면 안 되고, config 에 calibrate 실측을 적는다."""
    s = {k: dict(v) if isinstance(v, dict) else v for k, v in KWU_SEED.items()}
    for src in (FINE_SEED, FWD_SEED):
        for k, v in src.items():
            if isinstance(v, dict):
                s.setdefault(k, {}).update(v)
            else:
                s[k] = v
    return s


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
    """회전·직진의 값을 들고 있다 — 씨앗 위에 학습 파일 값(set_rotation · set_forward). 주행 중에는 안 바뀐다 (v5)."""

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
    #: 직진 잔여 [m] 의 범위 — 2026-10-01 까지 학습기가 쓰던 클램프 그대로 (한 걸음 상한 1.5 m 에 비해 작게)
    FWD_RESIDUAL = (-0.2, 0.2)

    def __init__(self, mode="record", seeds=None):
        self.mode = mode
        s = seeds or {}
        self.rot_tau = {d: self._ema(s.get("rot_tau_s", {}).get(d), *self.TAU_ROT)
                        for d in ROT_SIDES}
        self.fwd_tau = {k: self._ema(s.get("fwd_tau_s", {}).get(k), *self.TAU_FWD)
                        for k in FWD_KEYS}
        self.rot_rate = {d: self._ema(s.get("rot_rate_dps", {}).get(d), *self.RATE_ROT)
                         for d in ROT_SIDES}
        self.rot_startup = {d: self._ema(s.get("rot_startup_s", {}).get(d), *self.STARTUP_ROT)
                            for d in ROT_SIDES}
        self.rot_residual = {d: self._ema(s.get("rot_residual_deg", {}).get(d), -2.0, 2.0)
                             for d in ROT_SIDES}
        self.fwd_speed = {k: self._ema(s.get("fwd_speed_mps", {}).get(k), *self.SPEED_FWD)
                          for k in FWD_KEYS}
        self.fwd_startup = {k: self._ema(s.get("fwd_startup_s", {}).get(k), *self.STARTUP_FWD)
                            for k in FWD_KEYS}
        self.fwd_residual = {k: self._ema((s.get("fwd_residual_m") or {}).get(k), *self.FWD_RESIDUAL)
                             for k in FWD_KEYS}            # 전진·후진 따로 (v5 — 예전엔 하나를 같이 썼다)
        self.fwd_min_step = {k: None for k in FWD_KEYS}    # 가장 빨리 끊었을 때 가는 거리 [m] — 학습 파일에서만 (F1-3). 없으면 forward.min_step_m 이 정속 × τ
        self.rate = SessionSlope()      # 각속도 배율. 광운대 것 그대로
        self.rot_floor_deg = s.get("rot_floor_deg")   # 배우지 않는다. 씨앗 또는 config ROT_FLOOR_DEG (calibrate 실측)
        self.rot_floor_fine_deg = s.get("rot_floor_fine_deg")   # 약한 회전의 하한. 배우지 않는다 — 씨앗(유도값) 또는 config ROT_FLOOR_FINE_DEG
        # (주행 중 "요청보다 더 돌면 하한을 그 각으로 올리던" 규칙은 뺐다: 2026-10-01 실주행에서 +6.51 → +7.18 한 번에 하한이 7.18도가 돼 이후 보정이 전부 막혔다)

    @staticmethod
    def _ema(seed, lo, hi):
        v = float(seed) if seed is not None else 0.0
        return Ema(value=v, seed=v, lo=lo, hi=hi, n=0)

    # ── 회전 ────────────────────────────────────────────────────────
    def set_rotation(self, models):
        """회전 응답을 학습 파일 값으로 정한다 — 이 주행 내내 고정 (v5). models = {False: 강도 30, True: 강도 20},
        각각 learnfile.use() 의 {"tau_s", "residual_deg", "startup_s", "rate_dps"} (startup·rate 가 None 이면 씨앗 그대로).
        좌우는 같은 값이다 (2026-10-01 실측 좌우 차이 없음). 열쇠(L/R/Lf/Rf)는 rotate·hud 가 읽는 그대로 둔다."""
        for fine, m in (models or {}).items():
            if not m:
                continue
            for side in (("Lf", "Rf") if fine else ("L", "R")):
                for table, key in ((self.rot_tau, "tau_s"), (self.rot_residual, "residual_deg"),
                                   (self.rot_startup, "startup_s"), (self.rot_rate, "rate_dps")):
                    v = m.get(key)
                    if v is not None:
                        e = table[side]
                        e.value = e.seed = float(v)

    @staticmethod
    def rot_side(target_deg, fine=False):
        """회전 학습값의 열쇠 — 방향(L/R) + 약한 회전이면 f."""
        return ("L" if target_deg > 0 else "R") + ("f" if fine else "")

    def rot_rate_dps(self, target_deg, fine=False):
        """이 방향의 각속도 [도/s]. 아직 한 번도 안 돌았으면 None — 부르는 쪽이 대비한다."""
        return self._known(self.rot_rate[self.rot_side(target_deg, fine)])

    def rot_startup_s(self, target_deg, fine=False):
        """명령 -> 실제 회전 시작 [s]. 모르면 None."""
        return self._known(self.rot_startup[self.rot_side(target_deg, fine)])

    @staticmethod
    def _known(e):
        return e.value if (e is not None and (e.n > 0 or e.seed > 0)) else None

    # ── 직진 ────────────────────────────────────────────────────────
    def set_forward(self, models):
        """직진 값을 학습 파일 값으로 정한다 — 이 주행 내내 고정 (v5, 2026-10-02 결정 F1). models = {"67": 모델, "187": 모델},
        각각 learnfile.use()["straight"] 의 {"speed_mps", "dead_s", "tau_s", "residual_m"} (None 이면 씨앗 그대로).
        speed·dead = 시간 직진(사이드스텝 옆 이동·눈 감고 진입)의 유지 시간 = dead + 거리 ÷ speed. 카메라 직진의 정속 대신으로도 쓴다.
        tau·residual = 카메라 직진을 끊는 식 (fwd_lead_m). min_step_m = 이보다 짧게는 못 간다 (forward.min_step_m, F1-3)."""
        for key, m in (models or {}).items():
            if not m or key not in self.fwd_tau:
                continue
            if m.get("min_step_m") is not None:
                self.fwd_min_step[key] = float(m["min_step_m"])
            for table, name in ((self.fwd_speed, "speed_mps"), (self.fwd_startup, "dead_s"),
                                (self.fwd_tau, "tau_s"), (self.fwd_residual, "residual_m")):
                v = m.get(name)
                if v is not None:
                    e = table[key]
                    e.value = e.seed = float(v)

    def fwd_tau_s(self, strength):
        e = self.fwd_tau.get(str(int(strength)))
        return e.value if e else 0.0

    def fwd_residual_m(self, strength):
        e = self.fwd_residual.get(str(int(strength)))
        return e.value if e else 0.0

    def fwd_min_step_m(self, strength):
        """학습 파일의 최소걸음 [m]. 없으면 None."""
        return self.fwd_min_step.get(str(int(strength)))

    def fwd_speed_mps(self, strength):
        """이 강도의 정속 [m/s]. 아직 못 쟀으면 None."""
        return self._known(self.fwd_speed.get(str(int(strength))))

    def fwd_startup_s(self, strength):
        return self._known(self.fwd_startup.get(str(int(strength))))

    def fwd_lead_m(self, strength, speed, detect_age_s, period_s):
        """지금 끊어야 할 남은 거리 [m] = 속도 x (검출지연 + tau + 판정주기/2) + 잔여."""
        return speed * (max(0.0, detect_age_s) + self.fwd_tau_s(strength) + period_s / 2) + self.fwd_residual_m(strength)

    # ── 저장 ────────────────────────────────────────────────────────
    def dump(self):
        """이 주행이 쓴 값 (seed = final — 주행 중엔 안 바뀐다). 판 폴더 values_used.json 으로 남는다."""
        return {
            "mode": self.mode,
            "rot_tau_s": {d: e.dump() for d, e in self.rot_tau.items()},
            "rot_rate_dps": {d: e.dump() for d, e in self.rot_rate.items()},
            "rot_startup_s": {d: e.dump() for d, e in self.rot_startup.items()},
            "rot_residual_deg": {d: e.dump() for d, e in self.rot_residual.items()},
            "fwd_tau_s": {k: e.dump() for k, e in self.fwd_tau.items()},
            "fwd_speed_mps": {k: e.dump() for k, e in self.fwd_speed.items()},
            "fwd_startup_s": {k: e.dump() for k, e in self.fwd_startup.items()},
            "fwd_residual_m": {k: e.dump() for k, e in self.fwd_residual.items()},
            "fwd_min_step_m": dict(self.fwd_min_step),
            "rate_multiplier": dict(self.rate.multipliers),
        }
