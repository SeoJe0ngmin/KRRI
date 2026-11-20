"""캘리브 파일 3종 — 읽기·검사·거부 규칙.  (plan 4-2 "캘리브 사슬 2층 로드 규칙")

    timing_calib.json       L·ε·Δ_FS·T_line·TXACK 오프셋       (first_run timing → analyze)
    perception_calib.json   static(θ_min·b·σ_c) / dynamic(A·s·ε) (Day 0 격자 / 방문 A 회전)
    dynamics_calib.json     τ_eff·τ_r·v97·stiction·S(T)          (방문 A forward/rotate/creep)

**여기 있는 값은 사람이 정하는 값이 아니다** — 전부 측정 산출물이다. 그래서 config 가
아니라 파일로 둔다(CLAUDE.md 상수 최소화, plan 2-6 "캘리브 파일 부재 시 출발 거부,
삭제 상수의 코드 기본값 부활 금지").

로드 규칙은 2층이다 (plan 4-2):
    1층 정책   queue=1 / timestamp domain=GLOBAL / stale 규칙 / 스키마 — **불일치면 거부**
    2층 측정값 |ε_session − ε_calib| ≤ 15 ms ∧ gyro gaps = 0 ∧ L p99 < stale — 창 밖이면 거부
    생산출처   git·호스트·날짜·n — **경고만** (커밋 하나 했다고 현장에서 못 굴리면 안 된다).
               plan 4-2 는 git·호스트도 정책 해시에 넣으라 했지만, 그러면 코드를 한 줄만
               고쳐도(=dirty) 방문 당일 아침에 출발 거부가 난다. 그래서 여기서는
               정책(하드) / 생산출처(경고) 로 갈랐다. 판단 기록: G_code-A 보고서.

부재·거부 → **실주행 거부(dry-run 만)**. provisional=true → 큰 경고 후 진행.
"""
import hashlib
import json
import os
import platform
import time

# stale 문턱은 타이밍층이 원산(코드 내부 상수, config 아님) — 정책 해시에 들어간다.
from .timing import STALE_S

SCHEMA = 1
KINDS = ("timing", "perception", "dynamics")

#: 캘리브 파일이 사는 곳. analyze_first_run.py 가 여기에 쓰고 run.py 가 여기서 읽는다.
CALIB_DIR = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(
    os.path.abspath(__file__)))), "config")

FILENAME = {"timing": "timing_calib.json",
            "perception": "perception_calib.json",
            "dynamics": "dynamics_calib.json"}

#: 2층 허용창 (plan 4-2). 코드 내부 상수 — 현장에서 사람이 고칠 값이 아니다.
EPS_WINDOW_MS = 15.0          # |ε_session − ε_calib| 상한 [ms]
GYRO_GAPS_MAX = 0             # 세션 자이로 유실 구간 허용 개수


# ── 정책 ────────────────────────────────────────────────────────────────────

def git_commit():
    """현재 커밋(짧은 해시, 미커밋이면 +dirty). 실패하면 None."""
    import subprocess
    here = os.path.dirname(os.path.abspath(__file__))
    try:
        rev = subprocess.run(["git", "-C", here, "rev-parse", "--short", "HEAD"],
                             capture_output=True, text=True, timeout=3).stdout.strip()
        if not rev:
            return None
        dirty = subprocess.run(["git", "-C", here, "status", "--porcelain"],
                               capture_output=True, text=True, timeout=3).stdout.strip()
        return rev + ("+dirty" if dirty else "")
    except Exception:
        return None


def host_kind():
    """호스트 종류 한 줄. VM/Jetson/그램을 구분할 만큼만."""
    return "%s-%s" % (platform.system(), platform.machine())


def policy():
    """1층 정책 — 이게 다르면 캘리브 값을 그대로 쓸 수 없다(거부)."""
    return {"schema": SCHEMA,
            "frames_queue_size": 1,
            "timestamp_domain": "global_time",
            "stale_ms": round(STALE_S * 1000.0, 3)}


def policy_hash(pol=None):
    pol = policy() if pol is None else pol
    blob = json.dumps(pol, sort_keys=True, ensure_ascii=False).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()[:12]


def content_hash(obj):
    """파일 내용 해시 — config.json 스냅샷에 남겨 "이 주행이 쓴 캘리브" 를 못 박는다."""
    blob = json.dumps(obj, sort_keys=True, ensure_ascii=False, default=str).encode("utf-8")
    return hashlib.sha256(blob).hexdigest()[:12]


# ── 빈 틀 (provisional) ─────────────────────────────────────────────────────

def _v(value=None, **kw):
    d = {"value": value, "n": 0}
    d.update(kw)
    return d


def blank(kind, source=""):
    """측정 전 틀. provisional=true — 이 상태로도 굴러가되 큰 경고가 붙는다."""
    if kind not in KINDS:
        raise ValueError("모르는 캘리브 종류: %s" % kind)
    base = {"schema": SCHEMA, "kind": kind, "provisional": True,
            "date": time.strftime("%Y-%m-%d %H:%M:%S"),
            "host": host_kind(), "git": git_commit(), "source": source,
            "policy": policy(), "policy_hash": policy_hash(), "notes": []}
    if kind == "timing":
        base.update({
            "L_ms": {"median": None, "p99": None, "n": 0},
            "epsilon_ms": _v(None, sigma=None),
            "delta_fs_ms": _v(None),
            "t_line_us": _v(None),
            "txack_offset_ms": _v(None, p99=None),
            "loop_tick_ms": {"p99": None, "max": None, "n": 0},
            "gyro": {"gaps": None, "hz": None, "n": 0},
            "gate": {"txack_first10_max_ms": None, "gyro_gaps": None, "passed": False}})
    elif kind == "perception":
        base.update({
            "static": {"theta_min_deg": _v(15.0),        # plan 1-3 초기값. 격자로 확정
                       "heading_bias_deg": [],           # [{d, tilt, bias, sigma, n}]
                       "sigma_c_px": _v(None),
                       "flip_rate": _v(None),
                       "roll_deg": _v(None)},
            "dynamic": {"A_m": _v(0.0, sigma=None),      # plan 1-1: CAM_TO_PIVOT_M 1.46 철회
                        "gyro_scale": _v(1.0, sigma=None),
                        "epsilon_ms": _v(None),
                        "x_off_m": _v(None),
                        "cam_yaw_offset_deg": _v(None),
                        "h_tag_cam_m": _v(None)}})
    else:
        base.update({
            "tau_eff_s": {k: _v(None, sigma_e_m=None, q95_m=None)
                          for k in ("67_fwd", "97_fwd", "67_bwd", "97_bwd")},
            "tau_r_s": {"L": _v(0.18, sigma=None), "R": _v(0.18, sigma=None)},
            # 출발 죽은시간. **주행과 회전을 나눈다** — 예전엔 스칼라 하나라 analyze 가
            # 회전 값을 쓰고 소비자 셋이 저마다 다른 경로로 읽어 전부 None 을 받았다.
            "tau_start_s": {"fwd": _v(None), "rot": _v(None)},
            "rot_closed": {"L": _v(None, sigma_deg=None, n=0),
                           "R": _v(None, sigma_deg=None, n=0)},
            "min_inc": {"fwd_67_m": None, "fwd_97_m": None, "turn_deg": None},
            "v_mps": {"67": _v(None), "97": _v(None)},
            "stiction_ok": None,
            "deadzone_level": None,
            "S_of_T": {"67": [], "97": []},
            "gyro_bias_dps": None,
            "tag_cut": {"forward_m": None, "margin_px": None, "n": 0}})
    return base


# ── 읽기 ────────────────────────────────────────────────────────────────────

class Calib:
    """캘리브 파일 하나 + 그 판정. data 는 없으면 None."""

    def __init__(self, kind, path, data=None, missing=False, reject=None, warns=None):
        self.kind = kind
        self.path = path
        self.data = data
        self.missing = bool(missing)
        self.reject = reject                # 문자열이면 거부 사유
        self.warns = list(warns or [])

    # -- 판정 -------------------------------------------------------------
    @property
    def ok(self):
        """실주행에 써도 되나 (provisional 이어도 True — 경고는 warns 로)."""
        return self.data is not None and not self.missing and not self.reject

    @property
    def provisional(self):
        return bool((self.data or {}).get("provisional", True))

    @property
    def hash(self):
        return None if self.data is None else content_hash(self.data)

    def get(self, dotted, default=None):
        """'tau_eff_s.67_fwd.value' 처럼 점으로 찾는다. 없거나 null 이면 default."""
        node = self.data
        for key in dotted.split("."):
            if not isinstance(node, dict) or key not in node:
                return default
            node = node[key]
        return default if node is None else node

    def summary(self):
        if self.missing:
            return "%-10s 없음 (%s)" % (self.kind, os.path.basename(self.path))
        if self.reject:
            return "%-10s 거부: %s" % (self.kind, self.reject)
        tag = "provisional(잠정)" if self.provisional else "실측"
        return "%-10s %s  n출처 %s  %s  hash %s" % (
            self.kind, tag, self.data.get("source") or "?",
            self.data.get("date") or "?", self.hash)


def path_for(kind, directory=None):
    return os.path.join(directory or CALIB_DIR, FILENAME[kind])


def load(kind, path=None, directory=None):
    """캘리브 하나를 읽고 1층(정책·스키마)까지 판정한다. 예외를 던지지 않는다."""
    if kind not in KINDS:
        raise ValueError("모르는 캘리브 종류: %s" % kind)
    p = path or path_for(kind, directory)
    if not os.path.exists(p):
        return Calib(kind, p, missing=True)
    try:
        with open(p, encoding="utf-8") as f:
            data = json.load(f)
    except Exception as exc:
        return Calib(kind, p, reject="읽기 실패 (%s: %s)" % (type(exc).__name__, exc))

    warns = []
    if data.get("kind") != kind:
        return Calib(kind, p, data, reject="kind 가 %r 이다" % data.get("kind"))
    if int(data.get("schema", -1)) != SCHEMA:
        return Calib(kind, p, data,
                     reject="schema %s (이 코드는 %d)" % (data.get("schema"), SCHEMA))
    if data.get("fake_source"):
        # dry-run 가짜 소스로 만든 캘리브 — 실주행에 쓰면 가짜 숫자로 차가 움직인다
        return Calib(kind, p, data,
                     reject="가짜 소스(dry-run)로 만든 캘리브다 — 실측으로 다시 만들 것")
    want = policy()
    got = data.get("policy") or {}
    diff = [k for k in want if got.get(k) != want[k]]
    if diff:
        return Calib(kind, p, data,
                     reject="정책 불일치 %s (파일 %s / 지금 %s)"
                            % (diff, {k: got.get(k) for k in diff},
                               {k: want[k] for k in diff}))
    # 생산출처는 경고만 (거부하면 커밋 한 번에 현장이 멈춘다 — 파일 머리 주석 참고)
    if data.get("host") and data["host"] != host_kind():
        warns.append("호스트가 다르다: 파일 %s / 지금 %s" % (data["host"], host_kind()))
    now_git = git_commit()
    if data.get("git") and now_git and data["git"] != now_git:
        warns.append("git 이 다르다: 파일 %s / 지금 %s" % (data["git"], now_git))
    if data.get("provisional", True):
        warns.append("provisional=true — 실측으로 채워지지 않은 잠정값이다")
    return Calib(kind, p, data, warns=warns)


def load_all(directory=None):
    return {k: load(k, directory=directory) for k in KINDS}


def save(data, path=None, directory=None):
    """캘리브를 쓴다(정책·해시 갱신 포함). 쓴 경로를 돌려준다."""
    kind = data["kind"]
    p = path or path_for(kind, directory)
    data = dict(data)
    data["policy"] = policy()
    data["policy_hash"] = policy_hash()
    os.makedirs(os.path.dirname(os.path.abspath(p)), exist_ok=True)
    with open(p, "w", encoding="utf-8") as f:
        json.dump(data, f, ensure_ascii=False, indent=1, sort_keys=True)
    return p


# ── 2층: 세션 측정값 허용창 ─────────────────────────────────────────────────

class SessionTiming:
    """이번 세션에서 실제로 잰 타이밍. 2층 판정 입력."""

    def __init__(self, epsilon_ms=None, gyro_gaps=0, l_p99_ms=None,
                 domain_color=None, domain_gyro=None):
        self.epsilon_ms = epsilon_ms
        self.gyro_gaps = gyro_gaps
        self.l_p99_ms = l_p99_ms
        self.domain_color = domain_color
        self.domain_gyro = domain_gyro

    def as_dict(self):
        return {"epsilon_ms": self.epsilon_ms, "gyro_gaps": self.gyro_gaps,
                "l_p99_ms": self.l_p99_ms,
                "domain_color": self.domain_color, "domain_gyro": self.domain_gyro}


def check_session(calibs, session):
    """2층 허용창. (reasons, warns) 를 돌려준다. reasons 가 있으면 거부."""
    reasons, warns = [], []
    if session is None:
        return reasons, ["세션 타이밍 측정이 없다 — 2층 허용창을 못 봤다"]
    if session.gyro_gaps is None:
        warns.append("자이로 gaps 미측정")
    elif session.gyro_gaps > GYRO_GAPS_MAX:
        reasons.append("자이로 유실 %d 구간 (허용 %d)" % (session.gyro_gaps, GYRO_GAPS_MAX))
    if session.l_p99_ms is not None and session.l_p99_ms >= STALE_S * 1000.0:
        reasons.append("프레임 지연 L p99 %.0f ms ≥ stale %.0f ms"
                       % (session.l_p99_ms, STALE_S * 1000.0))
    tc = calibs.get("timing") if isinstance(calibs, dict) else None
    eps_c = None if tc is None else tc.get("epsilon_ms.value")
    if eps_c is not None and session.epsilon_ms is not None:
        if abs(session.epsilon_ms - eps_c) > EPS_WINDOW_MS:
            reasons.append("ε 가 캘리브에서 %.0f ms 벗어남 (세션 %.1f / 캘리브 %.1f, 허용 %.0f)"
                           % (abs(session.epsilon_ms - eps_c), session.epsilon_ms,
                              eps_c, EPS_WINDOW_MS))
    elif eps_c is None:
        warns.append("timing_calib 에 ε 가 없다 (provisional)")
    for name in ("domain_color", "domain_gyro"):
        dom = getattr(session, name)
        if dom is not None and "global" not in str(dom).lower():
            reasons.append("%s 타임스탬프 도메인이 %s — GLOBAL 이 아니다" % (name, dom))
    return reasons, warns


# ── 출발 판정 ───────────────────────────────────────────────────────────────

class Decision:
    def __init__(self, allow, reasons, warns, calibs):
        self.allow = bool(allow)
        self.reasons = list(reasons)
        self.warns = list(warns)
        self.calibs = calibs

    def report(self, log=print):
        for k in KINDS:
            c = self.calibs.get(k)
            if c is not None:
                log("  캘리브 %s" % c.summary())
        for w in self.warns:
            log("  !! 경고: %s" % w)
        if self.allow:
            log("  캘리브 판정: 실주행 가능")
        else:
            log("  캘리브 판정: **실주행 거부** — dry-run 만 가능")
            for r in self.reasons:
                log("     - %s" % r)
        return self.allow


def gate(directory=None, session=None, need=("timing", "dynamics"), calibs=None):
    """출발 전 캘리브 판정 하나. need 에 든 파일이 없거나 거부면 실주행 금지."""
    calibs = calibs if calibs is not None else load_all(directory)
    reasons, warns = [], []
    for k in KINDS:
        c = calibs[k]
        if c.missing:
            (reasons if k in need else warns).append(
                "%s 캘리브 없음 (%s) — analyze_first_run.py 로 만들 것" % (k, c.path))
            continue
        if c.reject:
            (reasons if k in need else warns).append("%s 캘리브 %s" % (k, c.reject))
            continue
        warns.extend("%s: %s" % (k, w) for w in c.warns)
    r2, w2 = check_session(calibs, session)
    reasons.extend(r2)
    warns.extend(w2)
    return Decision(not reasons, reasons, warns, calibs)


def snapshot(calibs):
    """--record-events config.json 에 넣을 캘리브 스냅샷(내용 + 해시)."""
    out = {}
    for k, c in (calibs or {}).items():
        out[k] = {"path": c.path, "missing": c.missing, "reject": c.reject,
                  "hash": c.hash, "provisional": c.provisional, "data": c.data}
    out["policy"] = policy()
    out["policy_hash"] = policy_hash()
    return out


__all__ = ["CALIB_DIR", "KINDS", "SCHEMA", "Calib", "Decision", "SessionTiming",
           "blank", "load", "load_all", "save", "gate", "check_session",
           "snapshot", "policy", "policy_hash", "content_hash", "path_for",
           "git_commit", "host_kind"]
