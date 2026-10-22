"""화면 패널. **계산은 하나도 안 한다** — 받은 것을 그리기만 (plan 12-0-1, 광운대 camera_overlay.py 규율).

run.py 와 tools/check/live_pose.py 가 같은 것을 쓴다. drawing.py(태그 모서리·축) 위에 얹는다:

    drawing.draw_corners(img, det)
    img = hud.draw(img, info)           # 왼쪽 위에 패널. 제자리에 그린다

info 는 dict. 없는 키·None·NaN·inf 는 "-" 로 찍히고, 어떤 키가 빠져도 예외는 없다.
    state      str | (str, level)       지금 단계. level = "ok"/"warn"/"bad" 는 색만 정한다
    fix        estimate.Fix | None      값은 **짝 σ 가 유한할 때만** 찍힌다 (Fix 규약: 모르는 σ = inf).
                                        few/stale/clock 의 기본값 0.0 이 측정값처럼 보이면 안 된다
    sigma      dict                     lateral_m heading_deg distance_m + drift_*/fast_* 조각 (결정 8).
                                        없으면 fix 의 *_sigma_* / *_drift_sigma_* / *_fast_sigma_* 로
    corridor   dict                     half_m inside(bool) lateral_m cut_m cut_from(live/fallback)
    plan       dict                     kind turn_deg drive_m why
    progress   dict                     kind("turn"/"drive"/…) done target unit  → 진행바
    learner    dict | Learner           Learner.dump() (+ rot_floor_deg 가 있으면 회전 하한도 — 결정 3)
    clock      dict | CameraClock       CameraClock.report()
    gyro       dict | Gyro              Gyro.quality() 에 angle_deg rate_dps alive calibrated 를 얹은 것
    can        dict | Driver            Driver.status()
    warnings   list[str | (level, str)]
    fps        float
    latency_ms float                    검출 지연
    loop_ms    float                    루프 주기
    run        dict                     elapsed_s steps mode("record"/"no-record"/"dry")
                                        + corrections fine_elapsed_s (마지막 단계 이중 상한 — 결정 6)
객체를 주면 dump()/report()/quality()/status() 를 여기서 부른다 — 부르는 쪽이 dict 로 안 바꿔도 된다.

글자는 영문·숫자다 — cv2 Hershey 는 ASCII 만 그린다(2026-09-21 현장에서 경고 줄이 '???' 로 나왔다).
한글이 섞인 문자열(why·warnings·state)은 PIL 로 그리되 **패널 영역만** 한 번 변환한다.
CJK 폰트가 없으면 '?' 로 바꿔 ASCII 로 찍는다 — 폰트 경로는 config 로 받지 않는다.
"""
if __package__ in (None, ""):
    # python src/utils/hud.py 로 직접 돌릴 때(자체 시험). 평소엔 패키지로 import 된다
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(_Path(__file__).resolve().parents[2]))
    __package__ = "src.utils"

import math
import os

import cv2
import numpy as np

from config import control as C
from config import detection as D
from config import imu as I
from .clock import age_ok

# ── 색 (BGR). 세 단계 + 글자. apriltag_v3/tools/check/live_pose.py 의 PC 팔레트 ────
COL = {"ok": (140, 255, 170), "warn": (60, 200, 255), "bad": (90, 90, 255),
       "txt": (245, 245, 245), "lab": (150, 150, 150), "dim": (110, 110, 110),
       "rule": (58, 58, 58)}
BG = (20, 24, 28)
ALPHA = 0.70                # 패널 배경 불투명도. 영상이 비치되 글자가 읽히는 정도

# ── 배치 (1080p 기준 [px]. 화면 높이에 비례해 줄인다) ─────────────────────
FONT = cv2.FONT_HERSHEY_SIMPLEX
S_KV, S_BIG = 0.5, 0.9      # Hershey 배율
MIN_SCALE = 0.4 / S_KV      # 배율 0.4(약 11 px) 아래면 Hershey 가 안 읽힌다 — 작은 화면의 하한
ROW_H = {"big": 34, "kv": 20, "bar": 22, "rule": 8}
MARGIN, PAD, GAP, LABEL_W, BAR_W = 12, 10, 10, 62, 220
PIL_PX_PER_SCALE = 32       # Hershey 배율 -> PIL 픽셀. 26(v3 live_pose)은 옆 영문보다 작아 보였다

# ── 한글 폰트 후보. v3 live_pose.py + 광운대 calib/hud.py FONT_PATH_CANDIDATES ──
_CJK_FONTS = (
    "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc",   # 우분투/Jetson: apt install fonts-noto-cjk
    "/usr/share/fonts/truetype/noto/NotoSansCJK-Regular.ttc",
    "/usr/share/fonts/truetype/nanum/NanumGothic.ttf",          # fonts-nanum
    "/System/Library/Fonts/AppleSDGothicNeo.ttc",               # macOS
    "/System/Library/Fonts/Supplemental/AppleGothic.ttf",
    "/Library/Fonts/AppleGothic.ttf",
    "C:/Windows/Fonts/malgun.ttf",                              # Windows(그램)
)
_font_cache = {}
_font_warned = [False]


# ── 공개 ────────────────────────────────────────────────────────────
def draw(img, info, scale=None, origin=None):
    """패널을 img 왼쪽 위에 그리고 img 를 돌려준다.

    제자리에 그린다. 읽기전용이거나 흑백이면 새 배열을 돌려주니 **반환값을 써라**.
    scale 을 안 주면 화면 높이에 맞춘다(1080p = 1.0) — 축소해 그려도 글자 크기가 유지된다.
    origin 은 패널 왼쪽 위 (x, y). 기본은 화면 왼쪽 위 구석.
    화면이 주행을 세우면 안 되므로 **여기서 난 예외는 밖으로 안 나간다** — 패널에 적힌다.
    """
    if img is None or getattr(img, "ndim", 0) < 2:
        return img
    if img.ndim == 2:
        img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    elif not img.flags.writeable:
        img = img.copy()                # cv2 는 writeable 플래그를 안 본다 — 원본을 지켜야 한다
    h = img.shape[0]
    k = float(scale) if scale else max(MIN_SCALE, h / float(D.COLOR_SIZE[1]))
    try:
        rows = _rows(info or {})
    except Exception as e:
        rows = [("big", [("HUD ERROR", COL["bad"])]),
                ("kv", "", [("%s: %s" % (type(e).__name__, e), COL["bad"])])]
    try:
        _paint(img, rows, k, origin or (MARGIN, MARGIN))
    except Exception as e:
        try:
            cv2.putText(img, "HUD: " + _ascii(str(e))[:80], (MARGIN, MARGIN + 20),
                        FONT, S_KV, COL["bad"], 1, cv2.LINE_AA)
        except Exception:
            pass
    return img


def hangul_ok():
    """한글을 그릴 수 있나. run.py 가 출발 전에 한 번만 알려주는 용도."""
    return _cjk_font(PIL_PX_PER_SCALE * S_KV) is not None


# ── 내용 (형식만 만든다. 값은 전부 받은 것) ──────────────────────────
def _rows(info):
    g = info.get
    rows = []
    fix, run = g("fix"), g("run") or {}
    mode = str(_get(run, "mode") or "-")
    txt, lab, dim = COL["txt"], COL["lab"], COL["dim"]

    state, lvl = _state(g("state"))
    rows.append(("big", [(state, COL[lvl])]))

    # 값은 짝 σ 가 유한할 때만 (Fix 규약: 모르는 σ = inf). few/stale/clock 의 기본값 0.0 을 안 찍는다
    flvl = _fix_level(fix)
    lat = _val(fix, "lateral_m", "lateral_fast_sigma_m")
    head = _val(fix, "heading_deg", "heading_fast_sigma_deg")
    dist = _val(fix, "distance_m", "distance_sigma_m")
    beta = _val(fix, "beta_deg", "beta_sigma_deg")
    rows.append(("big", [
        ("LAT", lab), (_f(lat, "%+.3f", " m"), COL[flvl]),
        ("HEAD", lab), (_f(head, "%+.2f", " deg"), COL[flvl]),
        ("DIST", lab), (_f(dist, "%.2f", " m"), txt)]))
    rows.append(("rule",))

    # RUN — 모드·시간·걸음·주기. 상한은 config 것을 옆에 적는다
    toks = [(mode, COL["ok"] if mode == "record" else COL["warn"]),
            _cap("t", _get(run, "elapsed_s"), C.TIME_LIMIT_S, _mmss, " s"),
            _cap("step", _get(run, "steps"), C.MAX_STEPS, lambda x: "%d" % x)]
    # 마지막 단계의 이중 상한 (결정 6). 마지막 단계에 들어가야 값이 온다
    if _get(run, "corrections") is not None:
        toks.append(_cap("fix", _get(run, "corrections"), C.MAX_CORRECTIONS, lambda x: "%d" % x))
    if _get(run, "fine_elapsed_s") is not None:
        toks.append(_cap("fine", _get(run, "fine_elapsed_s"), C.FINE_TIME_LIMIT_S, _mmss, " s"))
    toks += [("fps %s" % _f(g("fps"), "%.1f"), txt),
             ("det %s" % _f(g("latency_ms"), "%.0f", " ms"), txt),
             ("loop %s" % _f(g("loop_ms"), "%.0f", " ms"), txt)]
    rows.append(("kv", "RUN", toks))

    # TAG — 이 프레임의 검출이 어떤 상태인가
    if fix is None:
        rows.append(("kv", "TAG", [("-", dim)]))
    else:
        age = _num(_get(fix, "age_s"))
        edge = _num(_get(fix, "edge_px"))
        amb = int(_num(_get(fix, "ambiguous")) or 0)
        nfit = int(_num(_get(fix, "n_fit")) or 0)
        n = "n %s" % _f(_get(fix, "n"), "%d")
        if nfit or amb:                  # 좌우 직선에 실제로 쓴 점 · 두 해가 헷갈려 뺀 장 (결정 8)
            n += " (%s)" % " ".join(t for t in ("fit %d" % nfit if nfit else "",
                                                 "amb %d" % amb if amb else "") if t)
        toks = [(str(_get(fix, "why") or "-"), COL[flvl]),
                ("beta %s" % _f(beta, "%+.2f", " deg"), txt),
                ("fwd %s" % _f(_val(fix, "forward_m", "lateral_fast_sigma_m"), "%.2f", " m"), txt),
                ("age %s" % _f(None if age is None else age * 1000, "%.0f", " ms"),
                 dim if age is None else (COL["ok"] if age_ok(age) else COL["bad"])),
                (n, txt),
                ("v %s" % _f(None if dist is None else _get(fix, "closing_mps"), "%+.2f", " m/s"), txt),
                ("edge %s" % _f(edge, "%.0f", " px"),
                 txt if edge is None or edge >= D.TAG_EDGE_MARGIN_PX else COL["warn"]),
                ("tilt %s" % _f(_get(fix, "tilt_deg"), "%.1f"), dim)]
        roll = _num(_get(fix, "tag_roll_deg"))
        if roll is not None:
            toks.append(("roll %+.2f" % roll, dim))
        rows.append(("kv", "TAG", toks))

    # SIGMA — σ = √(밀림² + 떨림²) (결정 8). 조각은 estimate 가 Fix 에 나눠 준다
    sg = g("sigma")
    if sg is None and fix is not None:
        sg = {"lateral_m": _get(fix, "lateral_sigma_m"), "heading_deg": _get(fix, "heading_sigma_deg"),
              "distance_m": _get(fix, "distance_sigma_m"),
              "drift_lateral_m": _get(fix, "lateral_drift_sigma_m"),
              "fast_lateral_m": _get(fix, "lateral_fast_sigma_m"),
              "drift_heading_deg": _get(fix, "heading_drift_sigma_deg"),
              "fast_heading_deg": _get(fix, "heading_fast_sigma_deg")}
    rows.append(("kv", "SIGMA", [
        ("lat %s" % _mm(_get(sg, "lateral_m")), txt),
        ("(drift %s / fast %s)" % (_mm(_get(sg, "drift_lateral_m")),
                                   _mm(_get(sg, "fast_lateral_m"))), dim),
        ("head %s" % _f(_get(sg, "heading_deg"), "%.2f", " deg"), txt),
        ("(%s / %s)" % (_f(_get(sg, "drift_heading_deg"), "%.2f"),
                        _f(_get(sg, "fast_heading_deg"), "%.2f")), dim),
        ("dist %s" % _mm(_get(sg, "distance_m")), txt)]))

    # CORR — 통로 반폭과 안/밖
    co = g("corridor")
    if co is None:
        rows.append(("kv", "CORR", [("-", dim)]))
    else:
        inside = _get(co, "inside")
        rows.append(("kv", "CORR", [
            ("half %s" % _f(_get(co, "half_m"), "%.3f", " m"), txt),
            ("-", dim) if inside is None else
            (("IN", COL["ok"]) if inside else ("OUT", COL["warn"])),
            ("lat %s" % _f(_get(co, "lateral_m"), "%+.3f", " m"), txt),
            # 태그컷 — 프레임마다 다시 잰 값. '?' 는 폴백(아직 vertical·윗변 행이 없다)
            ("cut %s%s" % (_f(_get(co, "cut_m"), "%.2f", " m"), "" if _get(co, "cut_from") == "live" else "?"), txt)]))

    # PLAN — 다음 걸음
    pl = g("plan")
    if pl is None:
        rows.append(("kv", "PLAN", [("-", dim)]))
    else:
        toks = []
        kind = _get(pl, "kind")
        if kind:
            toks.append((str(kind).upper(), txt))
        toks += [("turn %s" % _f(_get(pl, "turn_deg"), "%+.2f", " deg"), txt),
                 ("drive %s" % _f(_get(pl, "drive_m"), "%.2f", " m"), txt)]
        why = _get(pl, "why")
        if why:
            toks.append(("why: %s" % str(why)[:48], dim))
        rows.append(("kv", "PLAN", toks))

    # 진행바 — 회전 남은각 · 직진 남은거리 (광운대 calib/hud.py _draw_progress_bar)
    pr = g("progress")
    if pr is not None:
        done, tgt = _num(_get(pr, "done")), _num(_get(pr, "target"))
        ratio = abs(done) / abs(tgt) if (done is not None and tgt) else None
        text = "%s %s / %s %s" % (str(_get(pr, "kind") or "").upper(),
                                  _f(done, "%.2f"), _f(tgt, "%.2f"), _get(pr, "unit") or "")
        if ratio is not None:
            text += " (%d%%)" % round(ratio * 100)
        rows.append(("bar", ratio, text.strip(),
                     COL["ok"] if ratio is None or ratio <= 1.0 else COL["warn"]))

    # LEARN — dump() 의 final. n=0(씨앗뿐) 이면 흐리게
    L = _as_dict(g("learner"), "dump", ("rot_floor_deg",))
    if L is None:
        rows.append(("kv", "LEARN", [("-", dim)]))
    else:
        rows.append(("kv", "LEARN", [
            ("rot", lab),
            _pair(L, "rot_tau_s", "tau", "%.3f", " s"),
            _pair(L, "rot_rate_dps", "rate", "%.1f", " dps"),
            _pair(L, "rot_startup_s", "start", "%.2f", " s"),
            _pair(L, "rot_residual_deg", "res", "%+.2f", " deg"),
            ("floor %s" % _f(_get(L, "rot_floor_deg"), "%.2f", " deg"), txt),
            ("mult %s/%s" % (_f(_get(_get(L, "rate_multiplier"), "ROT_LEFT"), "%.2f"),
                             _f(_get(_get(L, "rate_multiplier"), "ROT_RIGHT"), "%.2f")), dim)]))
        keys = [k for k in sorted((_get(L, "fwd_speed_mps") or {}))
                if any(_known(_get(_get(L, q), k))
                       for q in ("fwd_speed_mps", "fwd_startup_s", "fwd_tau_s"))]
        tail = [("res %s" % _ema(_get(L, "fwd_residual_m"), "%+.3f", " m")[0],
                 txt if _known(_get(L, "fwd_residual_m")) else dim),
                ("rej %s" % _f(_get(L, "rejected"), "%d"), txt),
                ("mode %s" % (_get(L, "mode") or "-"), dim)]
        if not keys:
            rows.append(("kv", "", [("fwd -", dim)] + tail))
        for i, key in enumerate(keys):
            toks = [("fwd%s" % key, lab),
                    _one(_get(_get(L, "fwd_speed_mps"), key), "v", "%.3f", " m/s"),
                    _one(_get(_get(L, "fwd_startup_s"), key), "start", "%.2f", " s"),
                    _one(_get(_get(L, "fwd_tau_s"), key), "tau", "%.3f", " s")]
            rows.append(("kv", "", toks + (tail if i == len(keys) - 1 else [])))

    # GYRO — 회전은 이 숫자만 보고 돈다. 화면에 없으면 부호·드리프트·끊김을 못 본다
    gy = _as_dict(g("gyro"), "quality", ("angle_deg", "rate_dps", "alive", "calibrated"))
    if gy is None:
        rows.append(("kv", "GYRO", [("-", dim)]))
    else:
        alive, calib = _get(gy, "alive"), _get(gy, "calibrated")
        glvl = "txt" if alive is None else ("ok" if (alive and calib) else ("warn" if alive else "bad"))
        pi = _get(gy, "process_interval_ms") or {}
        gaps = _num(_get(gy, "gaps"))
        toks = [(_f(_get(gy, "angle_deg"), "%+.2f", " deg"), COL[glvl]),
                (_f(_get(gy, "rate_dps"), "%+.2f", " dps"), txt),
                ("hz %s" % _f(_get(gy, "hz"), "%.0f"), txt),
                ("gaps %s" % _f(gaps, "%d"), COL["warn"] if gaps else txt),
                ("proc %s/%s/%s ms" % (_f(_get(pi, "median"), "%.1f"), _f(_get(pi, "p99"), "%.1f"),
                                       _f(_get(pi, "max"), "%.1f")), txt),
                ("(nom %.1f)" % (1000.0 / I.IMU_GYRO_HZ), dim)]
        if alive is False:
            toks.append(("STALE", COL["bad"]))
        elif calib is False:
            toks.append(("UNCAL", COL["warn"]))
        rows.append(("kv", "GYRO", toks))

    # CLOCK — 카메라 시계 기준점 (clock.CameraClock.report)
    ck = _as_dict(g("clock"), "report")
    if ck is None:
        rows.append(("kv", "CLOCK", [("-", dim)]))
    else:
        pulled = _num(_get(ck, "pulled_after_warmup_ms"))
        rows.append(("kv", "CLOCK", [
            ("ready", COL["ok"]) if _get(ck, "frozen") else ("warmup", COL["warn"]),
            ("n %s" % _f(_get(ck, "n"), "%d"), txt),
            ("spread %s" % _f(_get(ck, "latency_spread_ms"), "%.1f", " ms"), txt),
            ("pulled %s" % _f(pulled, "%.1f", " ms"), COL["warn"] if pulled else dim)]))

    # CAN — Driver.status(). dry 면 못 보내는 게 정상이라 빨갛게 안 한다
    st = _as_dict(g("can"), "status")
    if st is None:
        rows.append(("kv", "CAN", [("-", dim)]))
    else:
        if mode == "dry":
            first = (mode, COL["warn"])
        elif _get(st, "ready"):
            first = ("ready", COL["ok"])
        elif _get(st, "alive"):
            first = ("alive", COL["warn"])
        else:
            first = ("DOWN", COL["bad"])
        tx = _get(st, "tx_counts")
        txn = sum(_num(v) or 0 for v in tx.values()) if isinstance(tx, dict) else _num(tx)
        err = _num(_get(st, "errors"))
        toks = [first, ("mv %s" % (_get(st, "movement") or "-"), txt),
                ("tx %s" % _f(txn, "%d"), txt),
                ("err %s" % _f(err, "%d"), COL["warn"] if err else txt)]
        le, bus = _get(st, "last_error"), _get(st, "bus_status")
        if err and bus:
            toks.append(("bus %s" % str(bus)[:24], COL["warn"]))
        if le:
            toks.append((str(le)[:40], COL["bad"]))
        rows.append(("kv", "CAN", toks))

    for i, w in enumerate(g("warnings") or ()):
        if isinstance(w, (tuple, list)) and len(w) == 2:
            wl, wt = (w[0] if w[0] in COL else "warn"), str(w[1])
        else:
            wl, wt = "warn", str(w)
        rows.append(("kv", "WARN" if i == 0 else "", [(wt, COL[wl])]))
    return rows


# ── 형식 도우미 ─────────────────────────────────────────────────────
def _get(o, k):
    """dict 든 dataclass 든 같은 방법으로 읽는다. 없으면 None."""
    if o is None:
        return None
    if isinstance(o, dict):
        return o.get(k)
    return getattr(o, k, None)


def _as_dict(o, method, extra=()):
    """객체면 dump()/report()/quality()/status() 를 불러 dict 로. extra 는 객체 속성에서 덧붙인다."""
    fn = None if isinstance(o, dict) else getattr(o, method, None)
    if not callable(fn):
        return o
    d = dict(fn() or {})
    for k in extra:
        d.setdefault(k, getattr(o, k, None))
    return d


def _val(fix, key, sigma_key):
    """fix 의 값. 짝 σ 가 있는데 유한하지 않으면(inf = 모름) None — 기본값 0.0 을 측정값처럼 안 찍는다."""
    if fix is None:
        return None
    sig = _get(fix, sigma_key)
    if sig is not None and _num(sig) is None:
        return None
    return _num(_get(fix, key))


def _num(v):
    """숫자면 float, 아니면 None. NaN·inf·문자열·None 전부 None."""
    if isinstance(v, bool):
        return float(v)
    try:
        x = float(v)
    except (TypeError, ValueError):
        return None
    return x if math.isfinite(x) else None


def _f(v, fmt="%.3f", unit=""):
    x = _num(v)
    return "-" if x is None else (fmt % x) + unit


def _mm(v):
    x = _num(v)
    return "-" if x is None else "%.1f mm" % (x * 1000)


def _mmss(x):
    return "%d:%02d" % divmod(int(x), 60)


def _cap(name, v, limit, fmt, unit=""):
    """'값 / 상한'. 상한에 닿으면 주의색 — 곧 강제 정지다."""
    x = _num(v)
    lim = ("%d" % limit) if float(limit).is_integer() else ("%g" % limit)
    return ("%s %s / %s%s" % (name, "-" if x is None else fmt(x), lim, unit),
            COL["warn"] if (x is not None and x >= limit) else COL["txt"])


def _state(v):
    if isinstance(v, (tuple, list)) and len(v) == 2:
        return str(v[0]), (v[1] if v[1] in COL else "txt")
    return ("-" if v is None else str(v)), "txt"


def _fix_level(fix):
    if fix is None:
        return "dim"
    if _get(fix, "ok"):
        return "ok"
    # no_tag·clock 은 밖에서 고칠 일(태그·시계)이고, few·stale·no_sigma 는 더 보거나 재면 풀린다
    return "bad" if _get(fix, "why") in ("no_tag", "clock") else "warn"


def _known(e):
    """Ema.dump() 가 배운 값인가 — n>0 이거나 씨앗이 있으면 값이 있다."""
    return bool(e) and ((_num(_get(e, "n")) or 0) > 0 or (_num(_get(e, "final")) or 0) != 0)


def _ema(e, fmt, unit):
    """Ema.dump() -> (글자, 배웠나)."""
    return _f(_get(e, "final"), fmt, unit), bool(e) and (_num(_get(e, "n")) or 0) > 0


def _one(e, name, fmt, unit):
    s, learned = _ema(e, fmt, unit)
    return ("%s %s" % (name, s), COL["txt"] if learned else COL["dim"])


def _pair(L, key, name, fmt, unit):
    """좌/우 한 쌍을 한 토큰으로. 둘 다 씨앗뿐이면 흐리게."""
    d = _get(L, key) or {}
    l, ll = _ema(_get(d, "L"), fmt, "")
    r, rl = _ema(_get(d, "R"), fmt, "")
    return ("%s %s/%s%s" % (name, l, r, unit), COL["txt"] if (ll or rl) else COL["dim"])


# ── 그리기 ──────────────────────────────────────────────────────────
def _layout(row, k):
    """한 줄의 토큰을 (글자, x오프셋, 배율, 굵기, 색) 으로. 재기와 그리기가 같은 자리를 쓴다."""
    kind = row[0]
    out, x = [], PAD * k
    if kind == "big":
        s = S_BIG * k
        for text, color in row[1]:
            out.append((text, x, s, 2, color))
            x += _width(text, s, 2) + GAP * k
    elif kind == "kv":
        s = S_KV * k
        if row[1]:
            out.append((row[1], x, s, 1, COL["lab"]))
        x += LABEL_W * k
        for text, color in row[2]:
            out.append((text, x, s, 1, color))
            x += _width(text, s, 1) + GAP * k
    elif kind == "bar":
        # 막대는 고정 폭, 글은 그 오른쪽 — 채운 색 위에 글을 얹으면 안 읽힌다
        s = S_KV * k
        x += LABEL_W * k + BAR_W * k + GAP * k
        out.append((row[2], x, s, 1, COL["txt"]))
        x += _width(row[2], s, 1) + GAP * k
    return out, x + PAD * k


def _paint(img, rows, k, origin):
    h, w = img.shape[:2]
    laid = [(_layout(r, k), r) for r in rows]
    pw = max(L[1] for L, _ in laid)
    ph = sum(ROW_H[r[0]] for r in rows) * k + 2 * PAD * k
    x0, y0 = max(0, int(origin[0])), max(0, int(origin[1]))
    x1, y1 = min(w, int(x0 + pw)), min(h, int(y0 + ph))
    if x1 <= x0 or y1 <= y0:
        return
    roi = img[y0:y1, x0:x1]
    bg = np.empty_like(roi)
    bg[:] = BG
    cv2.addWeighted(roi, 1.0 - ALPHA, bg, ALPHA, 0.0, roi)

    pending = []
    y = y0 + PAD * k
    for (toks, _), row in laid:
        kind = row[0]
        rh = ROW_H[kind] * k
        base = int(y + rh * 0.74)               # cv2 의 org 는 글자 아래 기준선
        if kind == "rule":
            yy = int(y + rh / 2)
            cv2.line(img, (int(x0 + PAD * k), yy), (int(x1 - PAD * k), yy), COL["rule"], 1)
        elif kind == "bar":
            bx0 = int(x0 + PAD * k + LABEL_W * k)
            bx1 = int(bx0 + BAR_W * k)
            by0, by1 = int(y + 3 * k), int(y + rh - 3 * k)
            cv2.rectangle(img, (bx0, by0), (bx1, by1), COL["rule"], -1)
            if row[1] is not None:
                fx = bx0 + int((bx1 - bx0) * max(0.0, min(1.0, row[1])))
                cv2.rectangle(img, (bx0, by0), (fx, by1), row[3], -1)
            cv2.rectangle(img, (bx0, by0), (bx1, by1), COL["lab"], 1)
        for text, dx, s, thick, color in toks:
            _put(img, text, int(x0 + dx), base, s, color, thick, pending)
        y += rh
    if pending:
        _flush(img, (x0, y0, x1, y1), pending)


def _ascii(text):
    return str(text).encode("ascii", "replace").decode()


def _cjk_font(px):
    """크기별 CJK 폰트(캐시). 없으면 None — 그러면 '?' 로 옮겨 적는다."""
    px = max(9, int(px))
    if px in _font_cache:
        return _font_cache[px]
    f = None
    try:
        from PIL import ImageFont
        for path in _CJK_FONTS:
            if os.path.isfile(path):
                try:
                    f = ImageFont.truetype(path, px)
                    break
                except Exception:
                    continue
    except Exception:
        f = None
    _font_cache[px] = f
    return f


def _width(text, s, thick):
    text = str(text)
    if text.isascii():
        return cv2.getTextSize(text, FONT, s, thick)[0][0]
    f = _cjk_font(PIL_PX_PER_SCALE * s)
    if f is not None:
        return int(f.getlength(text))
    return cv2.getTextSize(_ascii(text), FONT, s, thick)[0][0]


def _put(img, text, x, base, s, color, thick, pending):
    """ASCII 는 바로 찍고, 한글은 모아 뒀다가 _flush 가 프레임당 한 번 그린다 (v3 live_pose _put)."""
    text = str(text)
    if text.isascii():
        cv2.putText(img, text, (x, base), FONT, s, color, thick, cv2.LINE_AA)
    elif _cjk_font(PIL_PX_PER_SCALE * s) is not None:
        pending.append((x, base, text, color, PIL_PX_PER_SCALE * s))
    else:
        if not _font_warned[0]:
            print("  !! CJK 폰트가 없다 — 한글은 '?' 로 나온다 (우분투: sudo apt install fonts-noto-cjk)")
            _font_warned[0] = True
        cv2.putText(img, _ascii(text), (x, base), FONT, s, color, thick, cv2.LINE_AA)


def _flush(img, rect, pending):
    """모아 둔 한글을 **패널 영역만** PIL 로 한 번에 그린다. 전체 프레임을 왕복하면 10 ms 대다."""
    x0, y0, x1, y1 = rect
    try:
        from PIL import Image, ImageDraw
        roi = img[y0:y1, x0:x1]
        pil = Image.fromarray(cv2.cvtColor(roi, cv2.COLOR_BGR2RGB))
        d = ImageDraw.Draw(pil)
        for x, base, text, color, px in pending:
            d.text((x - x0, base - y0), text, font=_cjk_font(px),
                   fill=(int(color[2]), int(color[1]), int(color[0])), anchor="ls")
        roi[:] = cv2.cvtColor(np.asarray(pil), cv2.COLOR_RGB2BGR)
    except Exception:
        for x, base, text, color, px in pending:
            cv2.putText(img, _ascii(text), (x, base), FONT, px / PIL_PX_PER_SCALE,
                        color, 1, cv2.LINE_AA)


# ── 자체 시험: 빈 화면에 가짜 info 로 그려 파일로 남긴다 ────────────────
if __name__ == "__main__":
    import sys
    import tempfile
    import time
    from pathlib import Path
    from types import SimpleNamespace

    out = Path(sys.argv[1]) if len(sys.argv) > 1 else Path(tempfile.gettempdir())
    out.mkdir(parents=True, exist_ok=True)

    fix = SimpleNamespace(ok=True, why="ok", lateral_m=0.083, heading_deg=-1.2, forward_m=4.2,
                          distance_m=4.213, beta_deg=0.31, lateral_sigma_m=0.012,
                          heading_sigma_deg=0.35, closing_mps=0.29, edge_px=212.0,
                          tilt_deg=15.2, tag_roll_deg=-0.4, n=12, ambiguous=1,
                          t_capture=100.0, age_s=0.033)
    ema = lambda v, n: {"seed": v, "final": v, "n": n, "spread": None}  # noqa: E731
    full = {
        "state": ("APPROACH", "ok"), "fix": fix,
        "sigma": {"lateral_m": 0.012, "heading_deg": 0.35, "drift_lateral_m": 0.010,
                  "drift_heading_deg": 0.30, "fast_lateral_m": 0.006, "fast_heading_deg": 0.18},
        "corridor": {"half_m": 0.212, "inside": True, "lateral_m": 0.083},
        "plan": {"kind": "step", "turn_deg": 3.2, "drive_m": 0.8, "why": "통로 안, 좌우 σ 2배 넘음"},
        "progress": {"kind": "turn", "done": 7.3, "target": 12.0, "unit": "deg"},
        "learner": {"mode": "record", "rejected": 0,
                    "rot_tau_s": {"L": ema(0.283, 0), "R": ema(0.283, 2)},
                    "rot_rate_dps": {"L": ema(12.01, 0), "R": ema(12.3, 2)},
                    "rot_startup_s": {"L": ema(1.082, 0), "R": ema(1.1, 2)},
                    "rot_residual_deg": {"L": ema(0.0, 0), "R": ema(-0.1, 2)},
                    "fwd_tau_s": {"97": ema(0.0, 0), "67": ema(0.0, 0)},
                    "fwd_speed_mps": {"97": ema(0.0, 0), "67": ema(0.28973, 0)},
                    "fwd_startup_s": {"97": ema(0.0, 0), "67": ema(1.4977, 0)},
                    "fwd_residual_m": ema(0.0, 0),
                    "rate_multiplier": {"ROT_LEFT": 1.0, "ROT_RIGHT": 1.02}},
        "clock": {"n": 1234, "frozen": True, "offset_s": -1.7e9, "latency_spread_ms": 2.1,
                  "pulled_after_warmup_ms": None},
        "gyro": {"n": 24000, "gaps": 0, "hz": 199.8,
                 "process_interval_ms": {"median": 5.0, "p99": 5.4, "max": 9.1},
                 "angle_deg": 12.34, "rate_dps": 0.02, "alive": True, "calibrated": True},
        "can": {"enabled": True, "alive": True, "ready": True, "movement": "forward",
                "tx_counts": {"movement": 1234, "control": 10, "heartbeat": 400},
                "errors": 0, "last_error": None, "bus_status": "OK"},
        "warnings": ["gyro gaps rising", ("bad", "!! 뒤공간 부족 — 사람 호출")],
        "fps": 29.7, "latency_ms": 22.0, "loop_ms": 33.0,
        "run": {"elapsed_s": 42.0, "steps": 3, "mode": "record"},
    }
    nasty = dict(full, fix=SimpleNamespace(ok=False, why="few", lateral_m=float("nan"),
                                           heading_deg=None, distance_m="4.1", beta_deg=0.0,
                                           age_s=-5.0, n=2, edge_px=float("inf")),
                 sigma=None, corridor={"half_m": None, "inside": None}, plan={"why": None},
                 progress={"kind": "drive", "done": 0.9, "target": 0.8, "unit": "m"},
                 gyro={"alive": False, "calibrated": False, "gaps": 3, "process_interval_ms": None},
                 clock={"frozen": False, "n": 7, "pulled_after_warmup_ms": 0.7},
                 can={"ready": False, "alive": False, "errors": 2, "last_error": "TXACK missing",
                      "tx_counts": "oops"},
                 learner={"fwd_speed_mps": {}, "rot_tau_s": None, "fwd_residual_m": None},
                 warnings=[123, ("nope", "level unknown"), None],
                 run={"elapsed_s": "x", "steps": None, "mode": "dry"}, state=None)

    def check(name, img, info, **kw):
        before = img.sum() if img.flags.writeable else None
        t = time.perf_counter()
        res = draw(img, info, **kw)
        dt = (time.perf_counter() - t) * 1000
        assert res is not None and res.ndim == 3, name
        assert res.sum() != (before or -1), "%s: 아무것도 안 그렸다" % name
        assert not ("HUD ERROR" in str(info)), name
        path = out / ("hud_selftest_%s.png" % name)
        cv2.imwrite(str(path), res)
        print("  ok  %-8s %dx%d  %.2f ms  -> %s" % (name, res.shape[1], res.shape[0], dt, path))
        return res

    check("1080", np.zeros((1080, 1920, 3), np.uint8), full)
    check("480", np.zeros((480, 640, 3), np.uint8), full)
    check("empty", np.zeros((720, 1280, 3), np.uint8), {})
    check("none", np.zeros((720, 1280, 3), np.uint8), None)
    check("nasty", np.zeros((1080, 1920, 3), np.uint8), nasty)
    check("gray", np.zeros((540, 960), np.uint8), full, scale=0.9)
    ro = np.zeros((540, 960, 3), np.uint8)
    ro.flags.writeable = False
    res = check("readonly", ro, full)
    assert res is not ro and ro.sum() == 0, "읽기전용은 복사본에 그려야 한다"
    assert draw(None, full) is None
    check("origin", np.zeros((720, 1280, 3), np.uint8), full, origin=(400, 300))
    # ── σ 규약: 값은 짝 σ 가 유한할 때만. 진짜 Fix 로 — few/stale/clock 의 기본값 0.0 이 찍히면 안 된다
    def big(info):
        return [t for t, _ in _rows(info)[1][1]]

    def row(info, label):
        r = next(r for r in _rows(info) if r[0] == "kv" and r[1] == label)
        return " ".join(t for t, _ in r[2])

    try:
        from src.models.detection.estimate import Fix
    except Exception as e:                  # 검출 패키지가 없는 머신 — 규약 검사만 건너뛴다
        print("  --  Fix 규약 검사 건너뜀 (%s)" % e)
        Fix = None
    if Fix is not None:
        assert big({"fix": Fix()}) == ["LAT", "-", "HEAD", "-", "DIST", "-"]
        assert big({"fix": Fix(why="clock", n=5, age_s=-5.0)}) == ["LAT", "-", "HEAD", "-", "DIST", "-"]
        assert _fix_level(Fix(why="clock")) == "bad" and _fix_level(Fix(why="few")) == "warn"
        few = Fix(why="few", n=4, distance_m=4.2, distance_sigma_m=0.01,
                  beta_deg=0.3, beta_sigma_deg=0.05)
        assert big({"fix": few}) == ["LAT", "-", "HEAD", "-", "DIST", "4.20 m"]
        assert "beta +0.30 deg" in row({"fix": few}, "TAG") and "fwd -" in row({"fix": few}, "TAG")
        nos = Fix(why="no_sigma", n=6, n_fit=5, ambiguous=1, lateral_m=0.083, lateral_fast_sigma_m=0.006,
                  heading_deg=-1.2, heading_fast_sigma_deg=0.18, forward_m=4.19,
                  distance_m=4.2, distance_sigma_m=0.01, beta_deg=0.3, beta_sigma_deg=0.05)
        assert big({"fix": nos}) == ["LAT", "+0.083 m", "HEAD", "-1.20 deg", "DIST", "4.20 m"]
        assert "n 6 (fit 5 amb 1)" in row({"fix": nos}, "TAG") and "fwd 4.19 m" in row({"fix": nos}, "TAG")
        assert row({"fix": nos}, "SIGMA").startswith("lat - (drift - / fast 6.0 mm)")   # σ 는 없고 떨림만
        okf = Fix(ok=True, why="ok", n=6, n_fit=6, lateral_m=0.083, lateral_fast_sigma_m=0.006,
                  lateral_drift_sigma_m=0.010, lateral_sigma_m=0.012, heading_deg=-1.2,
                  heading_fast_sigma_deg=0.18, heading_drift_sigma_deg=0.30, heading_sigma_deg=0.35,
                  forward_m=4.19, distance_m=4.2, distance_sigma_m=0.01, closing_mps=0.29,
                  beta_deg=0.3, beta_sigma_deg=0.05, age_s=0.03, edge_px=212.0, tilt_deg=15.2)
        assert "fwd 4.19 m" in row({"fix": okf}, "TAG")
        assert row({"fix": okf}, "SIGMA") == ("lat 12.0 mm (drift 10.0 mm / fast 6.0 mm) "
                                              "head 0.35 deg (0.30 / 0.18) dist 10.0 mm")
        check("fix", np.zeros((720, 1280, 3), np.uint8), dict(full, fix=okf, sigma=None))

    # 객체를 그대로 줘도 된다 — dump()/report()/quality()/status() 를 여기서 부른다
    class _Clock:
        def report(self):
            return {"n": 40, "frozen": True, "latency_spread_ms": 1.5, "pulled_after_warmup_ms": None}

    class _Gyro:
        angle_deg, rate_dps, alive, calibrated = 3.0, 0.1, True, True

        def quality(self):
            return {"n": 10, "gaps": 0, "hz": 200.0,
                    "process_interval_ms": {"median": 5.0, "p99": 5.2, "max": 6.0}}

    class _Learner:
        rot_floor_deg = 0.81

        def dump(self):
            return full["learner"]

    objs = dict(full, clock=_Clock(), gyro=_Gyro(), learner=_Learner())
    assert row(objs, "CLOCK").startswith("ready n 40") and row(objs, "GYRO").startswith("+3.00 deg")
    assert "floor 0.81 deg" in row(objs, "LEARN")
    check("objects", np.zeros((720, 1280, 3), np.uint8), objs)
    # 마지막 단계 이중 상한 (결정 6) — 닿으면 주의색
    r = _rows(dict(full, run={"elapsed_s": 42.0, "steps": 3, "mode": "record",
                              "corrections": 5, "fine_elapsed_s": 31.0}))
    toks = dict(next(x for x in r if x[0] == "kv" and x[1] == "RUN")[2])
    assert toks["fix 5 / 5"] == COL["warn"] and toks["fine 0:31 / 120 s"] == COL["txt"]
    assert toks["t 0:42 / 300 s"] == COL["txt"] and "fix" not in " ".join(dict(_rows(full)[3][2]))
    # CJK 폰트가 없는 머신(우분투에 fonts-noto-cjk 미설치)을 흉내 — '?' 로라도 그려야 한다
    _saved, _CJK_FONTS = _CJK_FONTS, ()
    _font_cache.clear()
    check("nofont", np.zeros((1080, 1920, 3), np.uint8), full)
    assert not hangul_ok()
    _CJK_FONTS = _saved
    _font_cache.clear()
    # 한글이 섞이면 패널 영역만 PIL 을 탄다 — 프레임당 비용을 본다
    img = np.zeros((1080, 1920, 3), np.uint8)
    t = time.perf_counter()
    for _ in range(30):
        draw(img, full)
    print("  1080p 평균 %.2f ms/프레임 (한글 %s)" % ((time.perf_counter() - t) / 30 * 1000,
                                              "PIL" if hangul_ok() else "없음 -> '?'"))
    print("전부 통과")
