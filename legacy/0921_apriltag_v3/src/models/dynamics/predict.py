"""명령 → 결과 예측.  (B 층: plan 2-1~2-7, contracts_B §3)

이 층이 답하는 것은 하나다 — **지금 이 명령을 주면 어디서 멈추나.**

    직진 정지거리  D̂ = τ_eff·v̂ + v̂·Δt_ctrl/2                        plan 2-1·2-2
    명령시간       T(d) = 죽은시간 + 선형 (S(T) 표가 있으면 표)        plan 2-1②
    회전 정지      θ_rem ≤ ω_pred·τ_r + ½α_up·τ_r² + ω_pred²/(2α_r)   plan 2-4
    최소 신뢰 증분 δ_x·θ_min,inc 과 그 **결과공간(최종 lateral) 값 δ_q**  plan 2-5·3-5
    명령 버퍼      임의 시각의 예상 v̂·ω̂ — 추정기 Q 스케줄 prior        plan 2-2

안 하는 것
    · CAN 을 건드리지 않는다. 송신은 `SafeCanTx` 하나뿐이고 여기선 숫자만 낸다.
    · 온라인 적응·학습 없음(plan 2-6). 값은 **생성 시점에 고정**, 잔차는 로그·경보만.
    · `rot_control.rotate_to()` 를 부르지 않는다 — ROT_LEAD_DEG(고정 리드각) 옛 규칙이다(계약 §3.3).
    · 시그모이드 거리모델 없음. 죽은시간 + 선형이다(9/7 로그).

캘리브가 없으면 (내일 아침의 현실 — 세 파일이 전부 없다)
    9/7 실측에서 나온 가정값으로 돌되 `source="assumed"` 를 붙이고 `degraded=True`
    가 된다. **조용히 넘어가지 않는다** — 계약 §5.4 강등 사다리:
        97 강제 · D̂ ×1.5(일찍 선다) · 한 다리 ≤1.0 m · 회전 1회 ≤15°.
    ⚠ **97 이 없다고 67 로 올라가지 않는다.** 더 빠른 쪽이 더 위험하다.
    97 을 유지하고 다리를 짧게 잘라 시간캡·IMU 정지판정으로 닫는다(계약 §5.1).
    (`config/control.py` 의 "97 이 비면 67 로 떨어진다" 주석은 방향이 반대다 — 통합 담당 정정 항목.)

부호는 contracts_B §1: v + = 태그로 접근(x 감소), ω + = 반시계(rotate_ccw, byte1=147).
"""
import math
import os
import sys
from dataclasses import dataclass

_ROOT = os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.dirname(os.path.abspath(__file__)))))
try:
    from config import control as C
except ImportError:                      # 단독 실행(python src/models/dynamics/predict.py)
    sys.path.insert(0, _ROOT)
    from config import control as C


# ═══════════════════════════════════════════════════════════════════════════
# 코드 내부 상수 — 구현 세부다. config 에 올리지 않는다(CLAUDE.md 상수 최소화).
# ═══════════════════════════════════════════════════════════════════════════

#: 불확실도 여유 배수. 계약 §3.4 "k = 2 (코드 내부 상수. config 아님)"
K_SIGMA = 2.0
#: 강등(가정값) 상태의 정지거리 배수 — 일찍 선다. 계약 §5.4
DEGRADED_STOP_GAIN = 1.5
#: 강등 상태의 한 다리 상한 [m] / 회전 1회 상한 [°]. 계약 §5.4
DEGRADED_LEG_M = 1.0
DEGRADED_TURN_DEG = 15.0
#: 후진 다리 상한 [m] — 후방 무관측이라 짧게. 계약 §5.1 `tau_eff_s.*_bwd`
BACKWARD_LEG_M = 0.30
#: 표본이 이보다 적으면 provisional 을 못 뗀다 (analyze_first_run.N_FIRM 과 같은 값)
N_FIRM = 10
#: 회전 시간 캡 = 예상 ×이 값 (plan 2-4 "시간 캡 = ω̂ 기준 예상 ×2")
ROT_TIMEOUT_GAIN = 2.0
#: 회전 시간 캡의 하한·상한 [s]. 죽은시간 0.85 s + 정지판정이 들어갈 자리
ROT_TIMEOUT_MIN_S, ROT_TIMEOUT_MAX_S = 3.0, 30.0
#: 명령시간 클램프 [s]. 1 s 미만 펄스는 τ_start 안에 끝나 의미가 없다(plan 2-5 ③)
CMD_MIN_S, CMD_MAX_S = 1.0, 15.0
#: "최소 신뢰 증분" 의 정의 — 이 CV 아래로 내려오는 최소 명령 (plan 2-1②)
CV_TRUST = 0.30
#: 9/7 실측 두 점 — 2 s 미만 명령은 CV 100%+, 3 s 이상은 ~10%. 사이는 선형보간(곡선 모양은 미측정)
CV_T_LO, CV_LO = 2.0, 1.00
CV_T_HI, CV_HI = 3.0, 0.10
#: 정지 판정 문턱의 여유배율 SF (plan 2-3 "코드 내부 1개")
SETTLE_SF = 3.0
#: 정지 판정에 필요한 최소 표본 수 / 블라인드 유지시간 [s] (plan 2-3 "≥0.3 s @200 Hz")
SETTLE_MIN_N, SETTLE_HOLD_S = 5, 0.30
#: 정지 문턱의 임시 하한 — 실측(note_still) 이 오면 그쪽이 이긴다.
#: γ_ω 는 plan 1-7 GYRO_STILL_DPS(0.3) 자리, γ_v 는 30 fps 에서 3 프레임에 1 cm.
GYRO_STILL_FLOOR_DPS = 0.30
V_STILL_FLOOR_MPS = 0.03
ACC_STILL_FLOOR_MPS2 = 0.15          # ★미측정 — σ_a,still 실측 전 임시 하한
#: 정지 판정 시간 캡의 꼬리 여유 [s] (plan 2-3 "τ_d,q95 + v̂/a + 0.5")
SETTLE_TAIL_S = 0.5
#: 회전 공칭 각속도 [°/s]. 9/7 강도 20 에서 1.5 s 끝에 7.3°/s, 아직 가속 중 → 보수적으로 8.
#: 워치독·표시 전용(rot_control.ROT_DEG_PER_SEC 와 같은 출처).
ROT_NOMINAL_DPS = 8.0
#: 그 공칭 각속도에 닿는 데 걸리는 시간 [s] — 9/7 펄스 1.5 s 관측. α_up 이 캘리브에 오면 대체된다.
ROT_RAMP_S = 1.5

#: 우리가 다루는 주행 레벨(byte2 편향). 67 = 127−60, 97 = 127−30(FORWARD_SLOW)
LEVELS = (67, 97)
SIDES = ("L", "R")
#: CAN 중립. 레벨 이름(67/97)이 config.FORWARD_SLOW 와 어긋나면 경고한다
AN_NEUTRAL = 127

#: 캘리브가 없을 때 쓰는 9/7 실측 가정값. **출처 없는 숫자는 여기 들어오지 않는다.**
ASSUMED = {
    # 정지지연 ~0.5 s. 관성 12~14 cm ÷ 0.28~0.30 m/s 와 맞는다(계약 §5.1)
    "tau_eff_s": 0.50,
    "sigma_e_m": 0.02,          # plan 2-7 통과기준
    "bwd_gain": 1.2,            # 후진은 fwd ×1.2 (계약 §5.1)
    "v_67": 0.28,               # 9/7 정속 0.28~0.30 의 아래쪽
    "v_97": 0.12,               # ★미측정. 없어도 97 을 유지한다(계약 §5.1)
    "tau_r_s": 0.18,            # plan 2-4 임시값 — 관성 1.3~1.5° ÷ 7.3°/s
    "tau_start_fwd_s": 1.00,    # 9/7 주행 출발 지연
    "tau_start_rot_s": 0.85,    # 9/7 회전 출발 지연(ROT_T0)
    "min_fwd_67_m": 0.30,       # plan 2-1 (fwd_time_model d_acc 0.289 와 같은 자리)
    "min_fwd_97_m": 0.12,       # plan 2-1 (97 ≈0.12 m)
    "min_turn_deg": 2.0,        # plan 2-4
    "rot_sigma_deg": 1.0,       # 9/7 폐루프 ±2° → σ 1.0 (계약 §5.1 rot_closed)
}

_NAN = float("nan")


# ═══════════════════════════════════════════════════════════════════════════
# 잔손질
# ═══════════════════════════════════════════════════════════════════════════

def cell_name(level, direction="fwd"):
    """캘리브 셀 이름. `tau_eff_s.<cell>` 에 그대로 들어간다."""
    return "%d_%s" % (int(level), direction)


def side_of(movement_or_side):
    """'rotate_ccw'/'L'/+1 → 'L'(좌·반시계), 'rotate_cw'/'R'/−1 → 'R'. 모르면 None."""
    x = movement_or_side
    if x is None:
        return None
    if isinstance(x, (int, float)) and not isinstance(x, bool):
        return "L" if x > 0 else ("R" if x < 0 else None)
    s = str(x).strip().lower()
    if s in ("l", "left", "ccw", "rotate_ccw", "+"):
        return "L"
    if s in ("r", "right", "cw", "rotate_cw", "-"):
        return "R"
    return None


def movement_for(level=67, direction="fwd"):
    """레벨·방향 → `can_tx.SAFE_MOVEMENTS` 문자열.

    후진 템플릿은 **하나뿐**이다(byte2=187). 그래서 레벨과 무관하게 'backward' 다 —
    τ 는 67_bwd 셀을 쓴다(187 = 127+60 으로 전진 67 과 크기가 같다).
    """
    if direction == "bwd":
        return "backward"
    return "forward_slow" if int(level) == 97 else "forward"


def _f(x):
    """수치면 float, 아니면 None. bool 은 수치로 안 본다."""
    if x is None or isinstance(x, bool):
        return None
    try:
        v = float(x)
    except (TypeError, ValueError):
        return None
    return v if math.isfinite(v) else None


def _dig(data, dotted, default=None):
    node = data
    for key in str(dotted).split("."):
        if not isinstance(node, dict) or key not in node:
            return default
        node = node[key]
    return default if node is None else node


def _getter(calib):
    """`utils.calib.Calib` / 생 dict / None 을 하나의 get(dotted, default) 로."""
    if calib is None:
        return lambda dotted, default=None: default
    if isinstance(calib, dict):
        return lambda dotted, default=None: _dig(calib, dotted, default)
    get = getattr(calib, "get", None)
    if callable(get):
        return lambda dotted, default=None: get(dotted, default)
    return lambda dotted, default=None: default


_FWD_MODULE = None


def _fwd_model():
    """`control/fwd_time_model.py`. 계약 §3.2 — S(T) 표가 없을 때 67 의 폴백이다.

    파일 직접 로드 경로는 **단독 실행 자기검증**에서만 탄다: 패키지로 import 하면
    `src.models.__init__` 가 cv2 를 끌고 오는데 개발용 맥엔 cv2 가 없다.
    """
    global _FWD_MODULE
    if _FWD_MODULE is None:
        try:
            from ..control import fwd_time_model as F
        except (ImportError, ValueError):
            import importlib.util
            path = os.path.join(_ROOT, "src", "models", "control", "fwd_time_model.py")
            spec = importlib.util.spec_from_file_location("_fwd_time_model", path)
            F = importlib.util.module_from_spec(spec)
            sys.modules[spec.name] = F      # dataclasses 가 __module__ 로 찾는다
            spec.loader.exec_module(F)
        _FWD_MODULE = F
    return _FWD_MODULE


def _cv_of_command(T):
    """명령시간 T [s] 의 거리 변동계수. 9/7 두 점 사이 **선형보간**(곡선은 미측정)."""
    t = _f(T)
    if t is None:
        return CV_LO
    if t <= CV_T_LO:
        return CV_LO
    if t >= CV_T_HI:
        return CV_HI
    f = (t - CV_T_LO) / (CV_T_HI - CV_T_LO)
    return CV_LO + f * (CV_HI - CV_LO)


# ═══════════════════════════════════════════════════════════════════════════
# 내놓는 것
# ═══════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class Prediction:
    """동역학 한 답. 계약 §3.1 그대로."""
    kind: str          # "stop_forward" | "stop_rotate" | "leg_time" | "min_inc"
    value: float       # 정지거리 [m] / 정지각 [°] / 명령시간 [s] / 최소증분
    sigma: float       # 1σ. 모르면 nan
    q95: float         # 상측 95%. 모르면 value + 2*sigma, 그것도 못 내면 nan
    level: int         # 67 | 97
    cell: str          # "67_fwd" | "97_fwd" | "67_bwd" | "97_bwd" | "L" | "R"
    source: str        # "calib" | "assumed"      ← "assumed" 면 화면에 경고가 뜬다
    provisional: bool  # 캘리브가 provisional 이거나 n 이 모자람
    note: str = ""

    def as_dict(self):
        """events.jsonl·can.jsonl 에 그대로 넣는 꼴."""
        return {"kind": self.kind, "value": self.value, "sigma": self.sigma,
                "q95": self.q95, "level": self.level, "cell": self.cell,
                "source": self.source, "provisional": self.provisional,
                "note": self.note}

    def __str__(self):
        unit = {"stop_forward": "m", "stop_rotate": "°", "leg_time": "s"}.get(self.kind, "")
        s = "%.3f%s" % (self.value, unit) if math.isfinite(self.value) else "?"
        q = "  q95 %.3f" % self.q95 if math.isfinite(self.q95) else ""
        return "%s %s [%s %s%s]%s" % (self.kind, s, self.cell, self.source,
                                      " 잠정" if self.provisional else "", q)


def _pred(kind, value, sigma, level, cell, source, provisional, note="", q95=None):
    v = _f(value)
    v = _NAN if v is None else v
    sg = _f(sigma)
    sg = _NAN if sg is None else sg
    if q95 is None:
        q95 = v + K_SIGMA * sg if (math.isfinite(v) and math.isfinite(sg)) else _NAN
    return Prediction(kind=kind, value=v, sigma=sg, q95=_f(q95) if _f(q95) is not None else _NAN,
                      level=int(level), cell=str(cell), source=source,
                      provisional=bool(provisional), note=note)


@dataclass(frozen=True)
class _Num:
    """캘리브에서 읽은 값 하나 + 그 출처."""
    value: float
    source: str        # "calib" | "assumed"
    n: int
    path: str

    @property
    def assumed(self):
        return self.source == "assumed"

    @property
    def thin(self):
        return self.source == "calib" and self.n < N_FIRM


# ═══════════════════════════════════════════════════════════════════════════
# Dynamics
# ═══════════════════════════════════════════════════════════════════════════

class Dynamics:
    """`dynamics_calib.json`(없으면 9/7 가정값) 로 명령 결과를 예측한다.

        dyn = Dynamics(calib.load("dynamics"), log=print)
        dyn.report()                                   # 무엇이 실측이고 무엇이 가정인지
        if dyn.should_stop_forward(x_rem, v_hat, 97, dt): tx.stop(...)

    값은 **생성 시점에 고정**된다(plan 2-6 적응 금지). 캘리브가 바뀌면 새로 만들어라.
    """

    def __init__(self, calib, *, log=None):
        self._log = log
        self._get = _getter(calib)
        self._calib = calib
        self._assumed = []          # 가정값을 쓴 경로 (순서 유지)
        self._thin = []             # 캘리브엔 있으나 n < N_FIRM
        self.warns = []

        if calib is None:
            self.missing = True
            self.calib_provisional = True
        else:
            self.missing = bool(getattr(calib, "missing", False)) or \
                (getattr(calib, "data", calib) is None)
            self.calib_provisional = bool(
                calib.get("provisional", True) if isinstance(calib, dict)
                else getattr(calib, "provisional", True))
        if getattr(calib, "reject", None):
            self.warns.append("캘리브 거부됨: %s" % calib.reject)
            self.missing = True

        # ── 직진 ────────────────────────────────────────────────────────────
        self.tau_eff, self.sigma_e, self.q95_m = {}, {}, {}
        for lvl in LEVELS:
            for d in ("fwd", "bwd"):
                cell = cell_name(lvl, d)
                base = ASSUMED["tau_eff_s"] * (ASSUMED["bwd_gain"] if d == "bwd" else 1.0)
                # 97 의 제동은 67 과 같은 유압이다 — 97 셀이 비면 67 셀을 먼저 본다(계약 §5.1)
                alt = cell_name(67, d) if lvl == 97 else None
                self.tau_eff[cell] = self._read("tau_eff_s.%s.value" % cell, base, alt=alt)
                self.sigma_e[cell] = self._read("tau_eff_s.%s.sigma_e_m" % cell,
                                                ASSUMED["sigma_e_m"], alt=alt, quiet=True)
                self.q95_m[cell] = self._read("tau_eff_s.%s.q95_m" % cell, None,
                                              alt=alt, quiet=True)
        self.v = {67: self._read("v_mps.67.value", ASSUMED["v_67"]),
                  97: self._read("v_mps.97.value", ASSUMED["v_97"])}
        # 스키마는 calib.blank 한 곳에서만 정의한다: tau_start_s.{fwd,rot}.value
        # (예전엔 analyze 가 스칼라로 쓰고 소비자마다 다른 경로로 읽어 전부 None 이었다)
        self.tau_start_fwd = self._read("tau_start_s.fwd.value",
                                        ASSUMED["tau_start_fwd_s"])
        self.tau_start_rot = self._read("tau_start_s.rot.value",
                                        ASSUMED["tau_start_rot_s"])

        # ── 회전 ────────────────────────────────────────────────────────────
        self.tau_r, self.alpha_r, self.alpha_up = {}, {}, {}
        self.rot_sigma, self.rot_mean, self.rot_n = {}, {}, {}
        for side in SIDES:
            self.tau_r[side] = self._read("tau_r_s.%s.value" % side, ASSUMED["tau_r_s"])
            # α 는 개루프 펄스 적합에서만 나온다(plan 2-4). 없으면 그 항은 0 = 순수지연만.
            self.alpha_r[side] = self._read("tau_r_s.%s.alpha_r_dps2" % side, None, quiet=True)
            self.alpha_up[side] = self._read("tau_r_s.%s.alpha_up_dps2" % side, None, quiet=True)
            self.rot_sigma[side] = self._read("rot_closed.%s.sigma_deg" % side,
                                              ASSUMED["rot_sigma_deg"])
            self.rot_mean[side] = self._read("rot_closed.%s.mean_deg" % side, 0.0, quiet=True)
            self.rot_n[side] = int(_f(self._get("rot_closed.%s.n" % side, 0)) or 0)

        # ── 최소 증분·데드밴드 ──────────────────────────────────────────────
        self.min_inc = {
            67: self._read("min_inc.fwd_67_m", ASSUMED["min_fwd_67_m"]),
            97: self._read("min_inc.fwd_97_m", ASSUMED["min_fwd_97_m"]),
            "turn": self._read("min_inc.turn_deg", ASSUMED["min_turn_deg"]),
        }
        self.deadzone = _f(self._get("deadzone_level", None))
        self.stiction_ok = self._get("stiction_ok", None)
        if self.stiction_ok is None:
            self.warns.append("stiction 미확인 — 97 출발 무응답 감시 필수(FM4)")
        self.tag_cut_m = self._read("tag_cut.forward_m", 3.3, quiet=True)

        # ── S(T) 표 ─────────────────────────────────────────────────────────
        self.s_of_t = {}
        for lvl in LEVELS:
            rows = self._get("S_of_T.%d" % lvl, []) or []
            self.s_of_t[lvl] = [r for r in rows if isinstance(r, dict)
                                and _f(r.get("T")) is not None
                                and _f(r.get("median")) is not None]
            if not self.s_of_t[lvl]:
                # S(T) 부재는 강등 사다리(97 강제·×1.5)가 아니라 **다리 제한**이다(계약 §5.1)
                self.warns.append("S(T) 표 없음(level %d) — 다리 ≤%.1f m 로 쪼개고 재관측"
                                  % (lvl, DEGRADED_LEG_M))

        # 정지 판정 문턱의 실측 입력(매 정지 note_still 로 갱신). 없으면 하한만 쓴다.
        self.still = {"omega_dps": None, "v_mps": None, "accel_mps2": None}

        # 레벨 이름(97)과 config.FORWARD_SLOW 가 어긋나면 셀 이름이 거짓말을 한다
        fs = _f(getattr(C, "FORWARD_SLOW", None))
        if fs is not None and int(AN_NEUTRAL - fs) != 97:
            self.warns.append("config.FORWARD_SLOW=%g → byte2 %d 인데 이 층은 97 로 부른다"
                              % (fs, AN_NEUTRAL - fs))
        if self._log:
            for w in self.warns:
                self._log("  !! 동역학: %s" % w)

    # ── 읽기 ───────────────────────────────────────────────────────────────
    def _read(self, path, assumed, alt=None, quiet=False):
        """캘리브 경로 하나. 없으면 가정값 + `assumed` 목록에 기록(조용히 안 넘어간다)."""
        parts = path.split(".")
        # alt = "이 셀이 비면 대신 볼 셀"(예: 97_fwd → 67_fwd). 점이 둘 이상인 경로에만 쓴다
        alt_path = ".".join([parts[0], alt] + parts[2:]) if (alt and len(parts) >= 3) else None
        for p in (path, alt_path):
            if not p:
                continue
            raw = self._get(p, None)
            if isinstance(raw, dict):           # 'x.value' 가 아니라 'x' 로 준 경우
                raw = raw.get("value")
            v = _f(raw)
            if v is not None:
                n = int(_f(self._get(p.rsplit(".", 1)[0] + ".n", 0)) or 0)
                num = _Num(v, "calib", n, p)
                if num.thin and not quiet:
                    self._thin.append("%s(n=%d)" % (p, n))
                return num
        if not quiet:
            self._assumed.append(path)
        a = _f(assumed)
        return _Num(_NAN if a is None else a, "assumed", 0, path)

    @classmethod
    def from_dir(cls, directory=None, *, log=None):
        """편의 — `utils.calib.load("dynamics")` 를 대신 해 준다(도구·시험용)."""
        from ...utils import calib as CAL       # 지연 import: 의존 방향 utils → dynamics
        return cls(CAL.load("dynamics", directory=directory), log=log)

    # ── 상태 ───────────────────────────────────────────────────────────────
    @property
    def degraded(self):
        """가정값을 하나라도 썼다."""
        return bool(self._assumed) or self.missing

    @property
    def force_slow(self):
        """True 면 전진은 forward_slow(97) 만. 67 금지 (계약 §5.4).

        ⚠ 반대 방향(97 이 없으니 67) 으로 떨어지지 않는다 — 더 빠른 쪽이 더 위험하다.
        """
        return self.degraded or self.calib_provisional

    @property
    def degraded_reason(self):
        """화면 상단 "가정값 사용 중: …" 한 줄 (계약 §5.4 2))."""
        if not self.degraded:
            return ""
        return "가정값 사용 중: " + ", ".join(self._assumed[:8]) + \
               (" 외 %d" % (len(self._assumed) - 8) if len(self._assumed) > 8 else "")

    def resolve_level(self, level):
        """실제로 쓸 레벨. 바꿨으면 이유를 같이 돌려준다 — **조용히 바꾸지 않는다.**

        Returns: (level, why)  why 가 "" 면 요청 그대로.
        """
        lvl = int(level)
        if lvl == 67 and self.force_slow:
            return 97, "강등(%s) — 67 금지, 97 로 간다" % ("가정값" if self.degraded else "잠정 캘리브")
        if lvl == 97 and self.v[97].assumed:
            # 97 을 **유지**한다. 속도를 모를 뿐이고, 모를 땐 느린 쪽이 안전하다.
            return 97, "v97 미측정(가정 %.2f m/s) — 97 유지, 다리 %.1f m 로 제한" % (
                self.v[97].value, self.leg_limit_m(97))
        return lvl, ""

    def leg_limit_m(self, level=67, direction="fwd"):
        """한 다리 상한 [m]. 강등이거나 S(T) 가 없으면 짧게 끊고 재관측한다."""
        if direction == "bwd":
            return BACKWARD_LEG_M
        if self.degraded or not self.s_of_t.get(int(level)):
            return DEGRADED_LEG_M
        return float("inf")

    def turn_limit_deg(self):
        """회전 1회 상한 [°]. τ_r 이 가정값이면 15° (계약 §5.1)."""
        side_assumed = any(self.tau_r[s].assumed for s in SIDES)
        return DEGRADED_TURN_DEG if (self.degraded or side_assumed) else 90.0

    def report(self, log=print):
        """무엇이 실측이고 무엇이 가정인지 한 화면. 출발 전에 사람이 본다."""
        log("  동역학 예측층 (B) — 캘리브 %s%s" % (
            "없음" if self.missing else "있음",
            " · provisional" if self.calib_provisional else ""))
        for cell in ("67_fwd", "97_fwd", "67_bwd"):
            t = self.tau_eff[cell]
            log("    τ_eff[%-6s] %5.3f s   %-7s  D̂(정속) %.3f m"
                % (cell, t.value, "실측" if not t.assumed else "가정",
                   t.value * self.v[int(cell.split("_")[0])].value))
        for lvl in LEVELS:
            n = self.v[lvl]
            log("    v[%d]          %5.3f m/s %-7s  최소증분 %.2f m  S(T) %s"
                % (lvl, n.value, "실측" if not n.assumed else "가정",
                   self.min_forward_m(lvl), "표 %d점" % len(self.s_of_t[lvl])
                   if self.s_of_t[lvl] else "없음"))
        for s in SIDES:
            log("    τ_r[%s]        %5.3f s   %-7s  σ_θ %.2f°  최소회전 %.2f°"
                % (s, self.tau_r[s].value, "실측" if not self.tau_r[s].assumed else "가정",
                   self.rot_sigma[s].value, self.min_turn_deg(s)))
        log("    출발지연 주행 %.2f s / 회전 %.2f s   데드밴드 %s   stiction %s"
            % (self.tau_start_fwd.value, self.tau_start_rot.value,
               "%g" % self.deadzone if self.deadzone else "모름", self.stiction_ok))
        if self._thin:
            log("    !! 표본 부족(n<%d): %s" % (N_FIRM, ", ".join(self._thin[:6])))
        if self.degraded:
            log("    !! %s" % self.degraded_reason)
            log("    !! 강등 규칙: 97 강제 · D̂ ×%.1f · 다리 ≤%.1f m · 회전 ≤%.0f°"
                % (DEGRADED_STOP_GAIN, DEGRADED_LEG_M, DEGRADED_TURN_DEG))
        for w in self.warns:
            log("    !! %s" % w)

    # ── 직진 ───────────────────────────────────────────────────────────────
    def stop_distance(self, v_mps, level=67, direction="fwd", dt_ctrl_s=0.0):
        """정지 발령용 D̂ = τ_eff·v̂ + v̂·Δt_ctrl/2.  (plan 2-1 m=0, 2-2 제어주기)

        강등 상태에서는 계약 §5.4 대로 ×1.5 한 **발령값**을 돌려준다(일찍 선다).
        v̂ 는 카메라 기준 접근속도(+ = 다가감). 음수·NaN 이면 0 을 돌려준다.
        """
        lvl = int(level)
        cell = cell_name(lvl if direction == "fwd" else 67, direction)
        tau, sg, q = self.tau_eff[cell], self.sigma_e[cell], self.q95_m[cell]
        v = _f(v_mps)
        if v is None or v <= 0.0:
            return _pred("stop_forward", 0.0 if v is not None else _NAN, sg.value, lvl, cell,
                         tau.source, True, "v̂ 가 %s — 정지거리 0 으로 본다"
                         % ("없다" if v is None else "0 이하"))
        base = tau.value * v + v * max(0.0, _f(dt_ctrl_s) or 0.0) / 2.0
        gain = DEGRADED_STOP_GAIN if self.degraded else 1.0
        value = base * gain
        q95 = value + (q.value if not q.assumed else K_SIGMA * sg.value)
        note = ""
        if gain != 1.0:
            note = "강등 ×%.1f (원값 %.3f m)" % (gain, base)
        if lvl == 67 and self.force_slow:
            note += (" · " if note else "") + "강등 중 67 금지 — resolve_level() 로 97 을 써라"
        return _pred("stop_forward", value, sg.value, lvl, cell,
                     tau.source, tau.assumed or tau.thin or self.calib_provisional, note, q95)

    def should_stop_forward(self, x_rem_m, v_mps, level=67, dt_ctrl_s=0.0):
        """`x_rem ≤ D̂` 이면 True.

        x_rem 은 **카메라 forward 기준**이다(계약 §3.3) — 기준점(포크 끝)으로 환산하지 마라.
        정지식은 카메라 forward 로 세워져 있고 A·x_off 와 무관해야 한다(plan 3-2).
        판정은 계약 §2.3 의 **긍정형**으로 쓴다: "계속 가도 되나" 가 참일 때만 안 선다
        → 입력이 NaN 이면 자동으로 **정지**(안전한 쪽)가 된다.
        """
        d = self.stop_distance(v_mps, level=level, dt_ctrl_s=dt_ctrl_s)
        x = _f(x_rem_m)
        keep_going = (x is not None and math.isfinite(d.value) and x > d.value)
        return not keep_going

    def command_time_for(self, dist_m, level=97):
        """블라인드·짧은 다리용 명령시간 [s]. S(T) 표가 있으면 표, 없으면 죽은시간+선형.

        계약 §3.2. 67 의 폴백은 `fwd_time_model`(9/7 로그 적합, 램프까지 들어있다),
        97 은 표도 적합도 없으므로 **τ_start + d/v97** 로 낸다(램프만큼 **덜 간다** = 안전한 쪽).
        """
        lvl = int(level)
        d = _f(dist_m)
        cell = cell_name(lvl, "fwd")
        if d is None or d <= 0.0:
            return _pred("leg_time", _NAN, _NAN, lvl, cell, "assumed", True, "거리가 없다")
        rows = self.s_of_t.get(lvl) or []
        if len(rows) >= 2:
            T, cv = self._invert_s_of_t(rows, d)
            if T is not None:
                T = min(max(T, CMD_MIN_S), CMD_MAX_S)
                return _pred("leg_time", T, (cv or _cv_of_command(T)) * d, lvl, cell,
                             "calib", self.calib_provisional,
                             "S(T) 표 %d점 역산" % len(rows))
        if lvl == 67:
            F = _fwd_model()
            # FWD_SCALE/FWD_BIAS 는 쓰지 않는다(plan 2-2 삭제 대상) — 1.0/0.0 으로 고정
            p = F.PiecewiseFwdParams(scale=1.0, bias=0.0,
                                     min_sec=CMD_MIN_S, max_sec=CMD_MAX_S)
            T = F.time_from_distance_piecewise(d, p)
            note = "S(T) 없음 — fwd_time_model(67 적합) 폴백. 다리 ≤%.1f m 로 쪼개고 재관측" % DEGRADED_LEG_M
        else:
            v = self.v[lvl].value
            T = min(max(self.tau_start_fwd.value + d / max(v, 1e-6), CMD_MIN_S), CMD_MAX_S)
            note = "S(T)·적합 없음 — 죽은시간 %.2f s + d/v%d(%.2f m/s). 램프만큼 덜 간다" % (
                self.tau_start_fwd.value, lvl, v)
        return _pred("leg_time", T, _cv_of_command(T) * d, lvl, cell, "assumed", True, note)

    def distance_for_command(self, sec, level=97):
        """명령시간 → 예상 거리 [m]. `command_time_for` 의 역. 사후 잔차 계산용."""
        lvl = int(level)
        t = _f(sec)
        cell = cell_name(lvl, "fwd")
        if t is None or t <= 0:
            return _pred("leg_time", _NAN, _NAN, lvl, cell, "assumed", True, "시간이 없다")
        rows = self.s_of_t.get(lvl) or []
        if len(rows) >= 2:
            rows = sorted(rows, key=lambda r: _f(r["T"]))
            lo = max([r for r in rows if _f(r["T"]) <= t], key=lambda r: _f(r["T"]), default=None)
            hi = min([r for r in rows if _f(r["T"]) >= t], key=lambda r: _f(r["T"]), default=None)
            if lo is not None and hi is not None:
                t0, t1 = _f(lo["T"]), _f(hi["T"])
                d0, d1 = _f(lo["median"]), _f(hi["median"])
                d = d0 if t1 == t0 else d0 + (d1 - d0) * (t - t0) / (t1 - t0)
                cv = _f(hi.get("cv")) or _cv_of_command(t)
                return _pred("leg_time", d, cv * d, lvl, cell, "calib",
                             self.calib_provisional, "S(T) 표 보간")
        if lvl == 67:
            F = _fwd_model()
            p = F.PiecewiseFwdParams(scale=1.0, bias=0.0)
            # 시간 → 거리 (piecewise 의 역): 죽은시간 → 램프 → 정속
            dt = max(0.0, t - p.t0)
            d = 0.5 * p.a * dt * dt if dt <= p.t1 else p.d_acc + (dt - p.t1) * p.vmax
            note = "fwd_time_model(67 적합) 역산"
        else:
            dt = max(0.0, t - self.tau_start_fwd.value)
            d = dt * self.v[lvl].value
            note = "죽은시간 + 선형"
        return _pred("leg_time", d, _cv_of_command(t) * d, lvl, cell, "assumed", True, note)

    @staticmethod
    def _invert_s_of_t(rows, dist_m):
        """S(T) 표에서 거리 → 명령시간. (T, cv) 또는 (None, None)."""
        rows = sorted(rows, key=lambda r: _f(r["median"]))
        lo = max([r for r in rows if _f(r["median"]) <= dist_m],
                 key=lambda r: _f(r["median"]), default=None)
        hi = min([r for r in rows if _f(r["median"]) >= dist_m],
                 key=lambda r: _f(r["median"]), default=None)
        if lo is None or hi is None:
            return None, None
        d0, d1 = _f(lo["median"]), _f(hi["median"])
        t0, t1 = _f(lo["T"]), _f(hi["T"])
        T = t0 if d1 == d0 else t0 + (t1 - t0) * (dist_m - d0) / (d1 - d0)
        return T, (_f(hi.get("cv")) or _f(lo.get("cv")))

    # ── 회전 (plan 2-4) ────────────────────────────────────────────────────
    def stop_angle(self, omega_dps, side=None, accelerating=False):
        """정지 명령을 지금 내면 더 도는 각 [°] (리드각).

            lead = ω_pred·τ_r + [가속중]·½·α_up·τ_r² + ω_pred²/(2·α_r)
            ω_pred = ω̂ + [가속중]·α_up·τ_r

        α_up·α_r 은 개루프 펄스 적합(plan 2-4)에서만 나온다. 없으면 그 항은 0 —
        **순수지연 τ_r 하나만** 쓴다(0.18 s + α 동시 사용은 같은 점의 이중 역산이다).
        """
        s = side_of(side) or "L"
        tau = self.tau_r[s]
        w = abs(_f(omega_dps) or 0.0)
        a_up = self.alpha_up[s].value if not self.alpha_up[s].assumed else None
        a_r = self.alpha_r[s].value if not self.alpha_r[s].assumed else None
        w_pred = w + (a_up * tau.value if (accelerating and a_up) else 0.0)
        lead = w_pred * tau.value
        if accelerating and a_up:
            lead += 0.5 * a_up * tau.value ** 2
        if a_r and a_r > 0:
            lead += w_pred ** 2 / (2.0 * a_r)
        sg = self.rot_sigma[s]
        note = "순수지연만(α 미측정)" if not (a_up or a_r) else ""
        if accelerating:
            note = ("가속중 · " + note) if note else "가속중"
        return _pred("stop_rotate", lead, sg.value, 0, s, tau.source,
                     tau.assumed or sg.assumed or self.calib_provisional, note)

    def should_stop_rotate(self, theta_rem_deg, omega_dps, side=None, accelerating=False):
        """`θ_rem ≤ 리드각` 이면 True. (plan 2-4 실시간 규칙 — 고정 ROT_LEAD_DEG 폐기)

        **IMU 콜백 주기로 평가한다**(8°/s × 60 ms = 0.5° 지터 회피). 단 CAN 상태를
        바꾸는 건 `SafeCanTx` 한 곳뿐이다 — 여기선 stop 요청만 올린다(계약 §3.3).
        긍정형: 입력이 NaN 이면 **정지**(안전한 쪽).
        """
        lead = self.stop_angle(omega_dps, side=side, accelerating=accelerating)
        rem = _f(theta_rem_deg)
        keep_turning = (rem is not None and math.isfinite(lead.value)
                        and abs(rem) > lead.value)
        return not keep_turning

    def rotate_timeout_s(self, deg, omega_dps=None):
        """회전 워치독 [s] = ω̂ 기준 예상 ×2 (plan 2-4). 캡은 코드 내부."""
        w = abs(_f(omega_dps) or 0.0)
        if w < 1.0:                      # 아직 안 돌고 있으면 공칭값으로 잡는다
            w = ROT_NOMINAL_DPS
        expect = self.tau_start_rot.value + abs(_f(deg) or 0.0) / w
        return min(max(ROT_TIMEOUT_GAIN * expect, ROT_TIMEOUT_MIN_S), ROT_TIMEOUT_MAX_S)

    # ── 최소 신뢰 증분 · 데드밴드 (plan 2-1②·2-5·3-5) ──────────────────────
    def min_forward_m(self, level=97):
        """δ_x — 이 차량이 "의미 있게" 낼 수 있는 가장 짧은 직진 [m].

        S(T) 표가 있으면 **CV < 30% 인 최소 명령**의 거리(plan 2-1②), 없으면 캘리브
        `min_inc`, 그것도 없으면 9/7 가정값(67 0.30 / 97 0.12).
        """
        lvl = int(level)
        rows = sorted(self.s_of_t.get(lvl) or [], key=lambda r: _f(r["T"]))
        for r in rows:
            cv = _f(r.get("cv"))
            if cv is not None and cv < CV_TRUST:
                return float(_f(r["median"]))
        return float(self.min_inc[lvl].value)

    def min_turn_deg(self, side=None):
        """θ_min,inc — 의미 있는 최소 회전 [°] = max(2σ_θ, 2.0°) (계약 §5.1)."""
        s = side_of(side)
        sides = [s] if s else list(SIDES)
        base = float(self.min_inc["turn"].value)
        sg = max(float(self.rot_sigma[x].value) for x in sides)
        return max(base, K_SIGMA * sg)

    def deadband_level(self):
        """움직이기 시작하는 byte2 편향. 모르면 None (계약 §3.2)."""
        return None if self.deadzone is None else int(self.deadzone)

    def delta_q(self, lever_m, level=97, side=None, psi_deg=0.0):
        """δ_q — 최소 신뢰 증분을 **결과공간(기준점 최종 lateral)** 으로 옮긴 값 [m].

        plan 3-5: `δ_q = max(θ_min,inc[rad]·lever, δ_x·|sin ψ|)`.
        lever 는 계약 §3.4 의 `x_ref + s = d − x_stop` 를 넣어라(x_ref 만 넣으면 54% 과소평가).
        행동 문턱이 `|e| > max(k·σ, δ_q)` 이고, 수용 밖인데 δ_q 보다 작으면
        **양자화 공백**(Tier 3 + "저속 회전 단 필요" 로그)이다.
        """
        lev = abs(_f(lever_m) or 0.0)
        turn = math.radians(self.min_turn_deg(side)) * lev
        fwd = self.min_forward_m(level) * abs(math.sin(math.radians(_f(psi_deg) or 0.0)))
        value = max(turn, fwd)
        s = side_of(side) or "L"
        sg = self.rot_sigma[s]
        return _pred("min_inc", value, math.radians(sg.value) * lev, int(level), s,
                     "calib" if not (sg.assumed or self.min_inc["turn"].assumed) else "assumed",
                     sg.assumed or self.min_inc["turn"].assumed or self.calib_provisional,
                     "회전 %.3f m / 직진 %.3f m (레버 %.2f m)" % (turn, fwd, lev))

    # ── 정지 판별 2모드 (plan 2-3). 문턱은 **매 정지 실시간 도출** — config 아님 ──
    def note_still(self, *, omega_dps=None, v_mps=None, accel_mps2=None):
        """정지창에서 실제로 잰 잡음(σ_ω,still·σ_v·σ_a,still)을 넣는다.

        plan 2-3: 문턱 γ = 그 지표 × 여유배율 SF. 안 넣으면 코드 내부 하한만 쓴다.
        """
        for k, v in (("omega_dps", omega_dps), ("v_mps", v_mps), ("accel_mps2", accel_mps2)):
            x = _f(v)
            if x is not None:
                self.still[k] = abs(x)
        return self.still

    def _gamma(self, key, floor):
        m = self.still.get(key)
        return max(floor, SETTLE_SF * m) if m is not None else floor

    def settle_visible(self, v_hist, omega_hist=()):
        """가시 모드 — 창 안 **전 샘플**이 문턱 아래면 멎었다 (plan 2-3 Autoware 형).

        v_hist 는 KF v̂ [m/s], omega_hist 는 ω [°/s]. 긍정형이라 비었으면 False.
        """
        vs = [abs(x) for x in map(_f, v_hist or []) if x is not None]
        ws = [abs(x) for x in map(_f, omega_hist or []) if x is not None]
        if len(vs) < SETTLE_MIN_N:
            return False
        g_v = self._gamma("v_mps", V_STILL_FLOOR_MPS)
        g_w = self._gamma("omega_dps", GYRO_STILL_FLOOR_DPS)
        return max(vs) < g_v and (not ws or max(ws) < g_w)

    def settle_blind(self, imu_window, hold_s=SETTLE_HOLD_S):
        """블라인드 모드 — IMU 만으로 (plan 2-3).

        imu_window = `run_log.imu` 와 같은 꼴 [{"s":"gyro"|"accel","t":..,"x","y","z"}].
        gyro 는 rad/s 로 들어온다(imu_yaw 원시). 축을 모르므로 **벡터 크기**를 쓴다 —
        어느 축이든 움직이면 "아직" 이 되는 안전한 쪽이다.
        판정: 마지막 hold_s 구간에서 |ω| < γ_ω ∧ std(a) < γ_a.
        """
        rows = [r for r in (imu_window or []) if isinstance(r, dict) and _f(r.get("t")) is not None]
        if not rows:
            return False
        t_end = max(_f(r["t"]) for r in rows)
        win = [r for r in rows if _f(r["t"]) >= t_end - hold_s]
        t0 = min(_f(r["t"]) for r in win)
        if (t_end - t0) < hold_s * 0.9:          # 창이 아직 안 찼다
            return False
        gyro = [math.sqrt(sum((_f(r.get(k)) or 0.0) ** 2 for k in "xyz"))
                for r in win if r.get("s") == "gyro"]
        acc = [[_f(r.get(k)) or 0.0 for k in "xyz"] for r in win if r.get("s") == "accel"]
        if len(gyro) < SETTLE_MIN_N:
            return False
        g_w = self._gamma("omega_dps", GYRO_STILL_FLOOR_DPS)
        if max(math.degrees(w) for w in gyro) >= g_w:
            return False
        if len(acc) >= SETTLE_MIN_N:
            g_a = self._gamma("accel_mps2", ACC_STILL_FLOOR_MPS2)
            for i in range(3):
                col = [a[i] for a in acc]
                mean = sum(col) / len(col)
                std = math.sqrt(sum((x - mean) ** 2 for x in col) / len(col))
                if std >= g_a:
                    return False
        return True

    def settle_timeout_s(self, v_mps, level=67):
        """정지 판정 시간 캡 [s] = τ_d,q95 + v̂/a + 0.5 (plan 2-3, 코드 내부).

        a 를 따로 재지 않았으므로 지수 감속 모델의 초기 감속 a = v̂/τ_eff 를 쓴다
        → v̂/a = τ_eff. 결과적으로 캡 = (1.5+1)·τ_eff + 0.5.
        """
        cell = cell_name(int(level), "fwd")
        tau = self.tau_eff[cell].value
        return DEGRADED_STOP_GAIN * tau + tau + SETTLE_TAIL_S

    # ── 명령 버퍼가 쓰는 프로파일 ───────────────────────────────────────────
    def profile(self, movement, side=None):
        """명령 하나의 (목표 v·ω, 죽은시간, 램프시간, 코스팅 τ_v·τ_w).

        직진 램프는 9/7 적합(fwd_time_model: 0.284 m/s 까지 2.04 s)의 가속도로,
        회전 램프는 9/7 펄스(1.5 s 에 7.3°/s)로 잡는다. α_up 이 캘리브에 오면 그쪽.
        **코스팅 τ 는 축마다 다르다** — 주행 0.5 s, 회전 0.18 s. 섞으면 회전 관성이
        2.8 배로 부풀어 리드각이 틀어진다. stop 일 때의 회전 τ 는 side 로 고른다.
        """
        mv = str(movement or "stop")
        s = side_of(side) or "L"
        if mv in ("forward", "forward_slow", "backward"):
            lvl = 97 if mv == "forward_slow" else 67
            d = "bwd" if mv == "backward" else "fwd"
            v = self.v[lvl].value * (-1.0 if mv == "backward" else 1.0)
            F = _fwd_model()
            a = F.FWD_A                        # 램프 가속도 [m/s²] (9/7 적합)
            return {"v": v, "w": 0.0, "dead": self.tau_start_fwd.value,
                    "ramp": abs(v) / max(a, 1e-6),
                    "tau_v": self.tau_eff[cell_name(lvl, d)].value,
                    "tau_w": self.tau_r[s].value}
        if mv in ("rotate_ccw", "rotate_cw"):
            s = "L" if mv == "rotate_ccw" else "R"
            w = ROT_NOMINAL_DPS * (1.0 if s == "L" else -1.0)
            a_up = self.alpha_up[s].value if not self.alpha_up[s].assumed else None
            ramp = abs(w) / a_up if a_up else ROT_RAMP_S
            return {"v": 0.0, "w": w, "dead": self.tau_start_rot.value, "ramp": ramp,
                    "tau_v": self.tau_eff["67_fwd"].value, "tau_w": self.tau_r[s].value}
        return {"v": 0.0, "w": 0.0, "dead": 0.0, "ramp": 0.0,
                "tau_v": self.tau_eff["67_fwd"].value, "tau_w": self.tau_r[s].value}


# ═══════════════════════════════════════════════════════════════════════════
# 명령 버퍼 — 언제 무슨 명령을 보냈나 → 임의 시각의 예상 v̂·ω̂  (plan 2-2)
# ═══════════════════════════════════════════════════════════════════════════

@dataclass(frozen=True)
class CommandRecord:
    """보낸 명령 한 줄. t_tx 는 payload 가 실제로 나간 시각(ChannelProbe)."""
    seq: int
    movement: str
    t_set: float
    t_tx: float
    why: str = ""
    v0: float = 0.0        # 이 명령이 나간 순간의 v (코스팅 이어붙이기용)
    w0: float = 0.0

    def as_dict(self):
        return {"seq": self.seq, "movement": self.movement, "t_set": self.t_set,
                "t_tx": self.t_tx, "why": self.why, "v0": self.v0, "w0": self.w0}


@dataclass(frozen=True)
class CommandState:
    """임의 시각 t 의 예상 상태. 추정기가 prior 로 쓴다."""
    t: float
    movement: str
    phase: str          # "still" | "dead" | "ramp" | "steady" | "coast"
    v_mps: float        # + = 태그로 접근(전진)
    omega_dps: float    # + = 반시계
    moving: bool
    transient: bool     # Q 스케줄 창 (plan 2-2) — 여기선 KF Q 를 키워야 한다
    since_cmd_s: float
    seq: int = -1

    def as_dict(self):
        return {"t": self.t, "movement": self.movement, "phase": self.phase,
                "v_mps": self.v_mps, "omega_dps": self.omega_dps,
                "moving": self.moving, "transient": self.transient,
                "since_cmd_s": self.since_cmd_s, "seq": self.seq}


class CommandBuffer:
    """보낸 명령의 기록 + 임의 시각의 예상 속도.

        buf = CommandBuffer(dyn)
        buf.push("forward_slow", t_set=t0, t_tx=t1, why="대각 다리")
        st = buf.at(now)          # st.v_mps 가 추정기 prior, st.transient 면 Q 를 키운다

    **CAN 을 건드리지 않는다.** 송신은 SafeCanTx 가 하고 여기엔 "보냈다" 만 들어온다.
    모델: 죽은시간 → 선형 램프 → 정속 → (정지 명령) 지수 코스팅 τ.
    코스팅 적분이 정확히 τ·v0 = D̂ 라서 `stop_distance` 와 같은 모델이다(plan 2-1).
    "움직인다" 의 문턱은 정지판정 하한(V_STILL_FLOOR_MPS·GYRO_STILL_FLOOR_DPS)과 같게 둔다 —
    그보다 느린 꼬리는 어차피 센서로 못 가른다. 적분(travel_m)은 문턱과 무관하게 정확하다.
    """

    def __init__(self, dynamics=None, maxlen=512):
        self.dyn = dynamics
        self.maxlen = int(maxlen)
        self.records = []
        self._seq = 0

    # -- 쓰기 --------------------------------------------------------------
    def push(self, movement, t_set=None, t_tx=None, why=""):
        """명령 하나를 기록한다. t_tx 가 없으면 t_set 을 쓴다(아직 TXACK 이 없을 때)."""
        import time as _time
        ts = _f(t_set)
        tx = _f(t_tx)
        if ts is None and tx is None:
            ts = tx = _time.time()
        ts = ts if ts is not None else tx
        tx = tx if tx is not None else ts
        prev = self.at(tx)
        rec = CommandRecord(seq=self._seq, movement=str(movement), t_set=ts, t_tx=tx,
                            why=str(why), v0=prev.v_mps, w0=prev.omega_dps)
        self._seq += 1
        self.records.append(rec)
        if len(self.records) > self.maxlen:
            del self.records[:len(self.records) - self.maxlen]
        return rec

    # -- 읽기 --------------------------------------------------------------
    @property
    def current(self):
        return self.records[-1] if self.records else None

    def record_at(self, t):
        t = _f(t)
        act = None
        for r in self.records:
            if t is None or r.t_tx <= t:
                act = r
            else:
                break
        return act

    def at(self, t):
        """시각 t 의 예상 상태. 명령이 없으면 정지 상태."""
        tt = _f(t)
        rec = self.record_at(tt)
        if rec is None or tt is None:
            return CommandState(t=tt if tt is not None else 0.0, movement="stop",
                                phase="still", v_mps=0.0, omega_dps=0.0, moving=False,
                                transient=False, since_cmd_s=0.0)
        age = max(0.0, tt - rec.t_tx)
        # stop 의 회전 코스팅 τ 는 돌던 방향(w0 부호)으로 고른다 — L/R 이 다르다
        p = (self.dyn.profile(rec.movement, side="L" if rec.w0 >= 0 else "R")
             if self.dyn is not None
             else {"v": 0.0, "w": 0.0, "dead": 0.0, "ramp": 0.0,
                   "tau_v": ASSUMED["tau_eff_s"], "tau_w": ASSUMED["tau_r_s"]})
        v = self._axis(age, rec.v0, p["v"], p["dead"], p["ramp"], p["tau_v"])
        w = self._axis(age, rec.w0, p["w"], p["dead"], p["ramp"], p["tau_w"])
        target_v, target_w = p["v"], p["w"]
        moving = abs(v) > V_STILL_FLOOR_MPS or abs(w) > GYRO_STILL_FLOOR_DPS
        if target_v == 0.0 and target_w == 0.0:
            phase = "coast" if moving else "still"
        elif age < p["dead"]:
            phase = "dead"
        elif age < p["dead"] + p["ramp"]:
            phase = "ramp"
        else:
            phase = "steady"
        transient = phase in ("dead", "ramp", "coast")
        return CommandState(t=tt, movement=rec.movement, phase=phase, v_mps=v,
                            omega_dps=w, moving=moving, transient=transient,
                            since_cmd_s=age, seq=rec.seq)

    @staticmethod
    def _axis(age, x0, target, dead, ramp, tau):
        """한 축(v 또는 ω)의 예상값. 죽은시간엔 이전 값이 코스팅한다."""
        tau = max(1e-6, tau)
        if target == 0.0:
            return x0 * math.exp(-age / tau)
        if age < dead:
            return x0 * math.exp(-age / tau)
        x_dead = x0 * math.exp(-dead / tau)
        dt = age - dead
        if ramp > 1e-6 and dt < ramp:
            # 램프는 선형(9/7 적합이 등가속) — 이전 값에서 목표까지 밀어 올린다
            return x_dead + (target - x_dead) * (dt / ramp)
        return target

    # -- 적분 (DR·다리 길이) ------------------------------------------------
    def travel_m(self, t0, t1, step=0.01):
        """[t0, t1] 예상 주행거리 [m]. + = 전진. 사다리꼴 적분(10 ms)."""
        return self._integrate(t0, t1, step, "v_mps")

    def turned_deg(self, t0, t1, step=0.01):
        """[t0, t1] 예상 회전각 [°]. + = 반시계."""
        return self._integrate(t0, t1, step, "omega_dps")

    def _integrate(self, t0, t1, step, attr):
        a, b = _f(t0), _f(t1)
        if a is None or b is None or b <= a:
            return 0.0
        n = max(1, int(math.ceil((b - a) / max(step, 1e-3))))
        h = (b - a) / n
        total = 0.0
        prev = getattr(self.at(a), attr)
        for i in range(1, n + 1):
            cur = getattr(self.at(a + i * h), attr)
            total += 0.5 * (prev + cur) * h
            prev = cur
        return total

    def rows(self):
        """로그용. `run_log.can`/events 에 그대로 넣는다."""
        return [r.as_dict() for r in self.records]


__all__ = ["Dynamics", "Prediction", "CommandBuffer", "CommandRecord", "CommandState",
           "ASSUMED", "LEVELS", "SIDES", "cell_name", "side_of", "movement_for"]


# ═══════════════════════════════════════════════════════════════════════════
# 자기 검증 — 9/7 실측 숫자를 넣으면 관측된 값이 재현되는가
#   python src/models/dynamics/predict.py            (맥: 카메라·CAN 없이 그대로 돈다)
# ═══════════════════════════════════════════════════════════════════════════

_FAILS = []


def _ok(cond, label, detail=""):
    mark = "OK  " if cond else "FAIL"
    print("  %s %-52s %s" % (mark, label, detail))
    if not cond:
        _FAILS.append(label)
    return bool(cond)


def _calib_9_7():
    """9/7 실측으로 채운 **가짜** dynamics_calib — 강등이 아닌 경로를 재려고 쓴다."""
    return {
        "kind": "dynamics", "provisional": False,
        "tau_eff_s": {"67_fwd": {"value": 0.50, "sigma_e_m": 0.02, "q95_m": 0.035, "n": 20},
                      "97_fwd": {"value": 0.50, "sigma_e_m": 0.02, "q95_m": 0.035, "n": 12},
                      "67_bwd": {"value": 0.60, "sigma_e_m": 0.03, "q95_m": 0.05, "n": 10},
                      "97_bwd": {"value": 0.60, "sigma_e_m": 0.03, "q95_m": 0.05, "n": 10}},
        "v_mps": {"67": {"value": 0.29, "n": 20}, "97": {"value": 0.12, "n": 12}},
        "tau_r_s": {"L": {"value": 0.18, "sigma": 0.02, "n": 12},
                    "R": {"value": 0.19, "sigma": 0.02, "n": 12}},
        # 스키마는 calib.blank 한 곳에서만 정의한다 — 주행/회전을 나눈다
        "tau_start_s": {"fwd": {"value": 1.00, "n": 20},
                        "rot": {"value": 0.85, "n": 12}},
        "rot_closed": {"L": {"sigma_deg": 0.9, "mean_deg": 0.1, "n": 12},
                       "R": {"sigma_deg": 1.0, "mean_deg": -0.1, "n": 12}},
        "min_inc": {"fwd_67_m": 0.30, "fwd_97_m": 0.12, "turn_deg": 2.0},
        "deadzone_level": 30, "stiction_ok": True,
        "tag_cut": {"forward_m": 3.3, "margin_px": 60, "n": 5},
        "S_of_T": {"67": [], "97": []},
    }


def _selftest_stop():
    print("\n[1] 정지거리 — 9/7 관측 12~14 cm 가 재현되는가")
    dyn = Dynamics(_calib_9_7())
    _ok(not dyn.degraded, "실측 캘리브면 degraded=False", "assumed=%s" % dyn._assumed)
    for v in (0.28, 0.29, 0.30):
        d = dyn.stop_distance(v, level=67)
        _ok(0.12 <= d.value <= 0.16, "v=%.2f m/s → D̂ 가 12~16 cm" % v, str(d))
    d29 = dyn.stop_distance(0.29, level=67)
    _ok(abs(d29.value - 0.145) < 1e-9, "D̂ = τ_eff·v̂ = 0.50 × 0.29", "%.4f m" % d29.value)
    _ok(d29.value >= 0.14, "관측 상한(14 cm) 이상 → 일찍 선다(보수)", "%.3f m" % d29.value)
    # 제어주기 항 (plan 2-2)
    d_dt = dyn.stop_distance(0.29, level=67, dt_ctrl_s=0.10)
    _ok(abs(d_dt.value - (0.145 + 0.29 * 0.05)) < 1e-9,
        "Δt_ctrl 0.1 s → +v̂·Δt/2", "%.4f m" % d_dt.value)
    # 발령
    _ok(dyn.should_stop_forward(0.14, 0.29, 67) is True, "x_rem 0.14 ≤ D̂ → 정지")
    _ok(dyn.should_stop_forward(0.20, 0.29, 67) is False, "x_rem 0.20 > D̂ → 계속")
    _ok(dyn.should_stop_forward(float("nan"), 0.29, 67) is True,
        "x_rem=NaN 이면 **정지**(긍정형 게이트)")
    _ok(dyn.should_stop_forward(2.0, float("nan"), 67) is True, "v̂=NaN 이면 **정지**")
    # 후진 셀
    db = dyn.stop_distance(0.29, level=67, direction="bwd")
    _ok(db.cell == "67_bwd" and db.value > d29.value, "후진 셀은 따로·더 길다", str(db))
    return dyn


def _selftest_degraded():
    print("\n[2] 캘리브가 없을 때 — 내일 아침의 기본 경로")
    dyn = Dynamics(None)
    _ok(dyn.degraded and dyn.force_slow, "degraded ∧ force_slow", dyn.degraded_reason[:60])
    d = dyn.stop_distance(0.28, level=67)
    _ok(abs(d.value - 1.5 * 0.14) < 1e-9, "D̂ ×1.5 로 일찍 선다", str(d))
    _ok(d.source == "assumed" and d.provisional, "source=assumed ∧ provisional")
    lvl, why = dyn.resolve_level(67)
    _ok(lvl == 97 and why, "67 요청 → 97 로 바꾸고 이유를 돌려준다", why)
    lvl2, why2 = dyn.resolve_level(97)
    _ok(lvl2 == 97 and "유지" in why2, "v97 미측정이어도 **97 을 유지**한다(67 로 안 올라간다)", why2)
    _ok(abs(dyn.v[97].value - 0.12) < 1e-9 and dyn.v[97].assumed, "v97 가정 0.12 m/s")
    _ok(dyn.leg_limit_m(97) == 1.0 and dyn.turn_limit_deg() == 15.0,
        "강등: 다리 ≤1.0 m · 회전 ≤15°")
    _ok(dyn.deadband_level() is None, "데드밴드 모르면 None")
    dyn.report(log=lambda s: print("   " + s))
    return dyn


def _selftest_rotate():
    print("\n[3] 회전 — 9/7 정지 관성 1.3~1.5° 가 재현되는가")
    dyn = Dynamics(_calib_9_7())
    for w in (7.3, 8.0):
        lead = dyn.stop_angle(w, side="L")
        _ok(1.3 <= lead.value <= 1.5, "ω=%.1f °/s → 리드각 1.3~1.5°" % w, str(lead))
    _ok(dyn.should_stop_rotate(1.0, 7.3, "L") is True, "θ_rem 1.0° ≤ 리드 → 정지")
    _ok(dyn.should_stop_rotate(5.0, 7.3, "L") is False, "θ_rem 5.0° > 리드 → 계속")
    _ok(dyn.should_stop_rotate(float("nan"), 7.3, "L") is True, "θ_rem=NaN 이면 정지")
    l_, r_ = dyn.stop_angle(8.0, "L").value, dyn.stop_angle(8.0, "R").value
    _ok(r_ > l_, "L/R 비대칭이 분리돼 있다", "L %.3f° / R %.3f°" % (l_, r_))
    # 가속 중 + α 가 있는 캘리브
    cal = _calib_9_7()
    cal["tau_r_s"]["L"].update({"alpha_up_dps2": 5.0, "alpha_r_dps2": 40.0})
    d2 = Dynamics(cal)
    acc = d2.stop_angle(4.0, "L", accelerating=True).value
    con = d2.stop_angle(4.0, "L", accelerating=False).value
    _ok(acc > con, "가속 중이면 리드각이 커진다", "%.3f° vs %.3f°" % (acc, con))
    _ok(3.0 <= dyn.rotate_timeout_s(15.0, 8.0) <= 30.0, "회전 워치독 = 예상 ×2",
        "%.2f s" % dyn.rotate_timeout_s(15.0, 8.0))
    _ok(side_of("rotate_ccw") == "L" and side_of("rotate_cw") == "R",
        "rotate_ccw(147)=L(+반시계) · rotate_cw(107)=R")
    return dyn


def _selftest_legs():
    print("\n[4] 명령시간·최소 증분·δ_q")
    dyn = Dynamics(_calib_9_7())
    t67 = dyn.command_time_for(1.0, 67)
    back = dyn.distance_for_command(t67.value, 67)
    _ok(abs(back.value - 1.0) < 0.02, "67: 거리→시간→거리 왕복", "T %.2f s → %.3f m"
        % (t67.value, back.value))
    t97 = dyn.command_time_for(1.0, 97)
    _ok(abs(t97.value - (1.0 + 1.0 / 0.12)) < 1e-6, "97: 죽은시간 + d/v (시그모이드 아님)",
        "%.2f s" % t97.value)
    _ok(t97.value > t67.value, "97 이 67 보다 오래 걸린다")
    _ok(dyn.command_time_for(1.28, 97).sigma > 0, "블라인드 1.28 m 의 σ 가 붙는다",
        "σ %.3f m" % dyn.command_time_for(1.28, 97).sigma)
    # S(T) 표가 있으면 표를 쓴다
    cal = _calib_9_7()
    cal["S_of_T"]["97"] = [{"T": 2.0, "median": 0.12, "cv": 0.55, "n": 10},
                           {"T": 3.0, "median": 0.24, "cv": 0.22, "n": 10},
                           {"T": 4.0, "median": 0.36, "cv": 0.12, "n": 10},
                           {"T": 6.0, "median": 0.60, "cv": 0.08, "n": 10}]
    d2 = Dynamics(cal)
    p = d2.command_time_for(0.30, 97)
    _ok(p.source == "calib" and 3.0 < p.value < 4.0, "S(T) 표가 있으면 표를 역산", str(p))
    _ok(abs(d2.min_forward_m(97) - 0.24) < 1e-9,
        "최소 신뢰 증분 = CV<30% 최소 명령(T=3.0 s, cv 0.22)",
        "%.3f m" % d2.min_forward_m(97))
    _ok(abs(dyn.min_forward_m(97) - 0.12) < 1e-9, "표가 없으면 min_inc 가정값",
        "%.3f m" % dyn.min_forward_m(97))
    # δ_q — 결과공간 환산. 레버는 x_ref + s = d − x_stop (계약 §3.4 E3)
    lever = 3.3 - 0.5
    q = dyn.delta_q(lever, level=97, side="R")
    _ok(q.value > C.LAT_TOL_M, "δ_q(레버 %.2f m) 가 LAT_TOL 0.030 보다 크다 → 양자화 공백 위험"
        % lever, "%.3f m" % q.value)
    q_short = dyn.delta_q(C.CAM_TO_REF_M, level=97, side="R")
    _ok(q_short.value < q.value, "레버가 짧아지면 δ_q 가 준다(근거리 태그의 이유)",
        "%.3f m" % q_short.value)
    _ok(abs(dyn.min_turn_deg("R") - 2.0) < 1e-9, "θ_min,inc = max(2σ_θ, 2.0°)",
        "%.2f°" % dyn.min_turn_deg("R"))
    cal2 = _calib_9_7()
    cal2["rot_closed"]["R"]["sigma_deg"] = 1.6
    _ok(abs(Dynamics(cal2).min_turn_deg("R") - 3.2) < 1e-9, "σ_θ 1.6° → 최소회전 3.2°")
    return dyn


def _selftest_settle():
    print("\n[5] 정지 판별 — 문턱은 매 정지 실시간 도출")
    dyn = Dynamics(_calib_9_7())
    _ok(dyn.settle_visible([0.001] * 10, [0.05] * 10) is True, "v̂·ω 가 작으면 멎었다")
    _ok(dyn.settle_visible([0.001] * 3) is False, "표본이 모자라면 False(긍정형)")
    _ok(dyn.settle_visible([0.001] * 9 + [0.09]) is False, "창 안에 하나라도 크면 False")
    _ok(dyn.settle_visible([]) is False, "빈 창은 False")
    dyn.note_still(omega_dps=0.4)
    _ok(dyn.settle_visible([0.001] * 10, [1.0] * 10) is True,
        "실측 σ_ω 0.4 → 문턱 1.2°/s 로 넓어진다(SF %.0f)" % SETTLE_SF)
    # 블라인드: 200 Hz 0.4 s
    still = []
    for i in range(80):
        t = i / 200.0
        still.append({"s": "gyro", "t": t, "x": 0.0, "y": 0.001, "z": 0.0})
        still.append({"s": "accel", "t": t, "x": 0.01, "y": -9.81, "z": 0.0})
    _ok(dyn.settle_blind(still) is True, "IMU 창이 조용하면 멎었다")
    moving = [dict(r) for r in still]
    for r in moving:
        if r["s"] == "gyro":
            r["y"] = 0.14              # ≈8 °/s
    _ok(dyn.settle_blind(moving) is False, "자이로가 돌면 False")
    _ok(dyn.settle_blind(still[:20]) is False, "창이 0.3 s 를 못 채우면 False")
    _ok(dyn.settle_blind([]) is False, "빈 창은 False")
    _ok(1.0 < dyn.settle_timeout_s(0.29, 67) < 3.0, "정지 판정 캡",
        "%.2f s" % dyn.settle_timeout_s(0.29, 67))
    return dyn


def _selftest_buffer():
    print("\n[6] 명령 버퍼 — 임의 시각의 예상 v̂ (추정기 prior)")
    dyn = Dynamics(_calib_9_7())
    buf = CommandBuffer(dyn)
    buf.push("forward", t_set=99.9, t_tx=100.0, why="대각 다리")
    _ok(buf.at(100.5).phase == "dead" and abs(buf.at(100.5).v_mps) < 1e-6,
        "죽은시간 1.0 s 안에는 안 움직인다")
    _ok(buf.at(101.5).phase == "ramp", "그 뒤는 램프")
    st = buf.at(110.0)
    _ok(st.phase == "steady" and abs(st.v_mps - 0.29) < 1e-9, "정속 0.29 m/s", str(st.v_mps))
    _ok(st.transient is False and st.moving is True, "정속 구간은 transient=False")
    buf.push("stop", t_tx=110.0, why="목표 도달")
    coast = buf.travel_m(110.0, 115.0)
    d_hat = dyn.stop_distance(0.29, 67).value
    _ok(abs(coast - d_hat) < 0.002, "코스팅 적분 = D̂ (같은 모델)",
        "%.4f m vs %.4f m" % (coast, d_hat))
    _ok(buf.at(110.2).transient is True, "코스팅은 Q 스케줄 창(transient)")
    _ok(buf.at(113.0).phase == "still", "다 멎으면 still")
    buf.push("rotate_ccw", t_tx=120.0, why="태그 겨냥")
    _ok(buf.at(120.5).omega_dps == 0.0, "회전도 죽은시간 0.85 s")
    w = buf.at(124.0).omega_dps
    _ok(abs(w - 8.0) < 1e-9, "반시계는 + (byte1=147)", "%.2f °/s" % w)
    buf.push("rotate_cw", t_tx=130.0)
    _ok(buf.at(135.0).omega_dps < 0, "시계는 − (byte1=107)")
    turned = buf.turned_deg(120.0, 130.0)
    _ok(turned > 0, "적분한 회전각도 반시계 +", "%.1f°" % turned)
    buf2 = CommandBuffer(dyn)
    buf2.push("rotate_ccw", t_tx=200.0)
    buf2.push("stop", t_tx=210.0, why="목표각 도달")
    coast_w = buf2.turned_deg(210.0, 215.0)
    lead = dyn.stop_angle(8.0, "L").value
    _ok(abs(coast_w - lead) < 0.02, "회전 코스팅 적분 = 리드각 (τ_r 0.18 s 를 쓴다)",
        "%.3f° vs %.3f°" % (coast_w, lead))
    _ok(abs(buf2.travel_m(210.0, 215.0)) < 1e-9, "회전만 했으면 직진 적분은 0")
    _ok(len(buf.rows()) == 4 and buf.current.movement == "rotate_cw", "기록이 남는다")
    _ok(buf.at(90.0).movement == "stop", "명령 전 시각은 정지")
    return buf


def _selftest_plant():
    """fake_rig 의 FakePlant 로 닫아 본다 — 예측한 자리에 실제로 서는가.

    fake_rig 는 cv2·numpy 를 import 한다(렌더용). 맥엔 cv2 가 없어서 **없을 때만**
    빈 스텁을 꽂고 `FakePlant`(순수 물리)만 쓴다. 렌더는 부르지 않는다 —
    계약 §7.4 대로 fake_rig 의 화면이 180° 뒤집혀 있어서 픽셀 값은 믿을 수 없다.
    """
    print("\n[7] fake_rig FakePlant 로 닫기 — 예측한 자리에 서는가")
    import importlib.util
    import types
    try:
        import numpy  # noqa: F401
    except ImportError:
        print("   (건너뜀) numpy 가 없다 — 이 파이썬으로는 FakePlant 를 못 돌린다")
        return None
    if "cv2" not in sys.modules:
        try:
            import cv2  # noqa: F401
        except ImportError:
            sys.modules["cv2"] = types.ModuleType("cv2")
            print("   (알림) cv2 없음 → 빈 스텁. FakePlant(물리)만 쓰고 렌더는 안 부른다")
    path = os.path.join(_ROOT, "tools", "etc", "fake_rig.py")
    spec = importlib.util.spec_from_file_location("_fake_rig", path)
    rig = importlib.util.module_from_spec(spec)
    sys.modules[spec.name] = rig
    spec.loader.exec_module(rig)

    # 플랜트의 진짜 파라미터를 그대로 캘리브로 준다(모델이 맞으면 오차 ≈ 0 이어야 한다)
    cal = _calib_9_7()
    cal["tau_eff_s"]["67_fwd"]["value"] = rig.FakePlant.TAU_FWD
    cal["v_mps"]["67"]["value"] = rig.FakePlant.V["forward"]
    cal["tau_eff_s"]["97_fwd"]["value"] = rig.FakePlant.TAU_FWD
    cal["v_mps"]["97"]["value"] = rig.FakePlant.V["forward_slow"]
    dyn = Dynamics(cal)
    _ok(not dyn.degraded, "플랜트 파라미터를 캘리브로 → degraded=False")

    for mv, lvl in (("forward", 67), ("forward_slow", 97)):
        plant = rig.FakePlant(forward=5.0, lateral=0.0, heading_deg=0.0)
        buf = CommandBuffer(dyn)
        target = 2.02                       # 내일의 정지점: 태그면 앞 CAM_TO_REF + STANDOFF
        dt, t_end, stopped = 0.01, plant.t + 40.0, False
        plant.set_cmd(mv)
        buf.push(mv, t_tx=plant.t)
        while plant.t < t_end:
            plant.step(dt)
            x_rem = plant.forward - target
            if not stopped and dyn.should_stop_forward(x_rem, plant.v, lvl, dt_ctrl_s=dt):
                plant.set_cmd("stop")
                buf.push("stop", t_tx=plant.t, why="x_rem ≤ D̂")
                stopped = True
                v_at_stop, x_at_stop = plant.v, plant.forward
            if stopped and abs(plant.v) < 1e-4:
                break
        err = plant.forward - target
        coast = x_at_stop - plant.forward
        pred = dyn.stop_distance(v_at_stop, lvl).value
        _ok(abs(err) <= 0.02, "%s: 목표 %.2f m 에 ±2 cm 로 선다" % (mv, target),
            "오차 %+.3f m · 코스팅 %.3f m(예측 %.3f)" % (err, coast, pred))
        _ok(abs(coast - pred) <= 0.01, "%s: 실제 코스팅 ≈ 예측 D̂" % mv,
            "%.4f vs %.4f m" % (coast, pred))
        _ok(abs(buf.travel_m(buf.records[-1].t_tx, plant.t) - coast) < 0.01,
            "%s: 명령버퍼 적분 ≈ 실제 코스팅" % mv, "%.4f m"
            % buf.travel_m(buf.records[-1].t_tx, plant.t))

    # 회전: 리드각만큼 남았을 때 끊으면 목표에 선다
    plant = rig.FakePlant(forward=3.0, heading_deg=0.0)
    cal["tau_r_s"]["L"]["value"] = rig.FakePlant.TAU_ROT
    dyn2 = Dynamics(cal)
    goal, dt = 15.0, 0.005
    plant.set_cmd("rotate_ccw")
    t_end, stopped = plant.t + 30.0, False
    while plant.t < t_end:
        plant.step(dt)
        rem = goal - plant.heading
        if not stopped and plant.omega > 0.5 and \
                dyn2.should_stop_rotate(rem, plant.omega, "L", accelerating=False):
            plant.set_cmd("stop")
            stopped = True
        if stopped and abs(plant.omega) < 1e-3:
            break
    _ok(abs(plant.heading - goal) <= 0.5, "회전 15° 목표에 ±0.5° 로 선다 (σ_θ 요건)",
        "%.3f°" % (plant.heading - goal))
    return dyn


def _selftest():
    print("═" * 74)
    print("dynamics/predict.py 자기 검증 — 9/7 실측 숫자 재현 + fake_rig 닫기")
    print("═" * 74)
    _selftest_stop()
    _selftest_degraded()
    _selftest_rotate()
    _selftest_legs()
    _selftest_settle()
    _selftest_buffer()
    _selftest_plant()
    print("\n" + "═" * 74)
    if _FAILS:
        print("실패 %d 개: %s" % (len(_FAILS), " / ".join(_FAILS)))
        return 1
    print("전부 통과")
    return 0


if __name__ == "__main__":
    sys.exit(_selftest())
