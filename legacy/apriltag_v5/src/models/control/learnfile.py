"""학습 파일 config/learned/<장소>.json — 회전 응답(inertia)·직진(straight)·자이로(gyro)를 **기록(로그)에서** 쌓는다
(v5, 2026-10-01·02 결정. plan 0절). 장소(config/site.py SITE)마다 파일이 따로이고, 그 장소 기록(work_dirs/<장소>/)에서만 만든다.
그 장소 자료가 칸마다 MIN_N 번이 안 되면 다른 장소 파일의 값을 빌려 쓴다(use) — 회전중심은 빌리지 않는다(카메라 장착마다).

왜 바꿨나: 주행 중에 계수 0.5 로 고치던 학습은 "이번에 넘친 양" 을 배웠는데, 그 회전은 이미 잔여만큼 일찍 끊은 결과라
  다음 잔여 = ½ × (넘친 양 + 쓴 잔여) — **직전 회전 하나의 절반만 기억했다**. 그래서
  ① 주행 중에는 값을 고정하고 ② 주행을 **시작할 때** 아직 안 넣은 기록만 읽어 걸러 합계에 더한다.
  끝날 때 쓰지 않으니 비정상 종료·긴급정지에도 사슬이 안 끊긴다. upto(어디까지 넣었나)로 같은 기록을 두 번 안 넣는다.

파일 모양 — 숫자만 (판 목록·뺀 이유는 파일에 안 남기고 갱신할 때 화면 = 주행 로그에 찍는다. 2026-10-01 사용자 결정):
    upto                                  여기까지(날짜_시각) 넣었다
    straight                              직진 — 전진(forward, byte 67) · 후진(backward, 187). 후진 자료가 없으면 전진 값을 쓴다
      speed_mps · dead_s                  시간 직진(사이드스텝 옆 이동 · 눈 감고 진입): 유지 시간 = dead_s + 거리 ÷ speed_mps.
                                          카메라 직진들의 "명령 유지 시간 → 실제 간 거리" 를 직선 하나로 맞춘 값 (F1-1)
      tau_s · residual_m                  카메라 직진을 끊는 식 "남은 거리 ≤ v·(지연 + τ + 주기/2) + 잔여" 에 넣는 값 (F1-2)
      min_step_m                          최소걸음 = 가장 빨리 끊었을 때 실제로 가는 거리의 평균 (F1-3). MIN_N 번 미만이면 null
      n_timed · n_stop · n_min · sums     위 값을 다시 낼 합계
    inertia                               회전 응답 — 강도별로 따로, 좌우는 합친다 (2026-10-01 실측 좌우 차이 없음)
      strength_30 / strength_20           30 = 사이드스텝·탐색 · 20 = 걸음 회전 전부 (결정 R1)
        tau_s · residual_deg              끊는 식 "남은 각 ≤ ω·(τ + 주기/2) + 잔여" 에 넣는 값
        startup_s · rate_dps              회전 안전망 시간에만 쓰인다
        n · sums                          위 값을 다시 낼 합계
    gyro                                  자이로 (결정 R5) — 정답 자는 태그 화면위치·거리 (0.004° 정밀)
      scale                               눈금. 확실할 때만 1.0 이 아니다
      cam_to_rot_center_m · rot_center_lateral_m · n
                                          회전중심 (config 와 같은 이름·부호, 차체 축). n 이 MIN_N 이상일 때만 값이 있다.
                                          흩어짐(ROT_CENTER_RMS_MM)은 config 의 calibrate 값을 그대로 쓴다 — 아래 무게 때문에 합계의 잔차는 mm 가 아니다
      sums.mount                          지금 카메라 장착의 합계. calibrate 회전중심 단계가 새로 시작시킨다(since = 그 calibrate) —
                                          회전중심은 카메라를 다시 달면 바뀌니까
      sums.closed                         지난 장착들의 눈금 몫 — 눈금은 카메라 안 센서라 장착이 바뀌어도 이어진다
      sums.runs                           판마다 따로 맞춘 눈금 — 판끼리 흩어짐으로 "우연이 아닌가" 를 본다

회전 응답 — 회전 하나가 남기는 것 (강도별):
    ω = 끊을 때 각속도 [°/s]
    e = **기준값(τ 씨앗 · 잔여 0)으로 끊었다면** 되튐 뒤에 남았을 오차 [°] — 그때 쓴 τ·잔여의 몫을 역산해 뺀다
        e = (되튐 뒤 실제 각 − 요청) − [ω·(τ_기준 − τ_씀) + (0 − 잔여_씀)]       (끊는 순간 각속도는 같다고 본다)
  합계 n · Σω · Σe · Σω² · Σωe · Σe² 로 "오차 = a + c·ω" 를 언제든 다시 맞춘다 → 끊는 식에 τ = τ_기준 + c, 잔여 = a.
  자료가 적으면 a(평균)만 쓰고, c 는 점이 MIN_N 이상이고 |c| > UNCERTAIN_FACTOR × 표준오차일 때만 쓴다.
  출발지연·각속도(회전 안전망 시간에만 쓰인다)도 같은 걸러진 회전으로 평균낸다.
  되튐 뒤 각 = run.py 가 회전 뒤 1초 대기 끝에 남기는 rotate_final (옛 기록은 그 시각 프레임의 자이로 각).

자이로 — 서서 태그를 본 채 돈 회전 하나가 남기는 것 (목적 상관없이, 회전 전후로 태그가 보였으면):
    q1, q2 = 회전 전·후 서 있을 때 태그 자리 (카메라 수평, 앞·왼쪽) = ρ·(cos β, −sin β),  ρ = √(거리² − 높이차²)
    g      = 그동안 자이로가 잰 회전 (눈금 안 곱한 값)
  차가 회전중심 c 둘레로 k·g 만큼 돌면 q2 = R(−k·g)·(q1 − c) + c. k = 1 + δ 를 δ 에 대해 펼치면 (δ 는 0.01 급)
    q2 − R(−g)·q1 = (I − R(−g))·c + δ·u,   u = −g·R(−g)·J·(q1 − c0)        J = [[0,−1],[1,0]], c0 = 그 장착의 출발 중심
  미지수 셋(δ, c)에 선형 → 합계(XᵀX 6개 · Xᵀy 3개 · yᵀy · n)만 들고 다닌다. 중심은 장착마다, δ 는 전부 같이.
  무게: 거리 방향 오차 ∝ 거리², 화면위치 방향 오차 ∝ 거리 (_rows) — 같은 무게면 먼 표본의 거리 오차가 좌우 중심을 끌고 간다.
  눈금은 판이 MIN_N 이상이고 |δ| > UNCERTAIN_FACTOR × max(합친 표준오차, 판끼리 흩어짐의 표준오차) 일 때만 쓴다
  (2026-10-01: 36번으로 0.990, 판 하나씩 빼면 0.982~0.9995 — 표준오차 하나만 믿으면 우연을 진짜로 본다).
  중심은 그 눈금(안 쓰면 1)으로 고정하고 맞춘다. calibrate 스윙의 정지점 쌍도 같은 식으로 들어간다.

거르는 것 — 판: dry-run · 판 폴더에 exclude 파일(사람이 뺌, 안에 이유 한 줄) · 기록 없음.
          회전 응답만: 강도 설정이 지금 config 와 다름 · 목적이 step 이 아님(사이드스텝·탐색·calibrate) · 정상 종료 아님 ·
                      출발을 보기 전에 끊김 · 시작 때 덜 멎음(각속도 > IMU_MOVING_DPS) · 자이로 끊김 · 하한 미만 요청 ·
                      되튐 뒤 각을 못 잼 · 역산이 그 판 기록과 안 맞음.
          자이로만: 정상 종료 아님 · 자이로 끊김 · 시작 때 덜 멎음 · 전후 태그 프레임이 MIN_N 미만 · 회전 뒤 바로 움직임.
          직진만: 시간 직진(카메라가 없다) · 끝이 정상이 아님(멈춤 확인 실패 settle_timeout 은 **넣는다** — 판정 문턱이 작았을 뿐 거리는 재져 있다) ·
                 시작 측정이 못 믿을 값(시작 남은거리가 0.0 — 2026-10-01 16:55 의 0.67 m 직진은 실제 1.01 m 를 갔는데 0.63 m 로 남았다) ·
                 태그를 BLIND_MAX_S 넘게 놓침.

직진 — 카메라로 거리를 보며 간 직진 하나가 남기는 것 (2026-10-02 결정 F1):
    시간 직진용: (유지 시간 h, 실제 간 거리 d) → 합계로 d = a·h + b 를 맞춘다 → speed = a, dead = −b/a
                 (2026-10-01 직진 24번: 0.290 m/s · 1.20 s, 예측 오차 RMS 11 cm — 광운대 고정값 13 cm · 옛 학습 24 cm)
    끊는 값:     e = 기준값(τ_기준 FWD_TAU_REF_S · 잔여 0)으로 끊었다면 지나쳤을 양 = (간 거리 − 요청) + v·(τ_씀 − τ_기준) + 잔여_씀
                 합계로 e = a + c·v → τ = τ_기준 + c (기울기가 우연이 아닐 때만), 잔여 = a. 회전과 같은 틀
                 (2026-10-01: 목표 대비 오차 RMS 옛 학습 8.1 cm → 이 방식 5.0 cm)
    최소걸음:    출발을 인정한 순간(forward 의 t_onset_ok — 정지 명령을 보낼 수 있는 첫 순간)까지 간 거리 + 그때 속도 × τ_기준
                 (2026-10-11 결정 F1-3). 거리·속도는 태그까지 3D 거리(프레임)에 그 순간 앞뒤 WINDOW_S 로 직선을 맞춘 값.
                 출발을 인정하자마자 끊긴 직진(cut_at_onset)은 실제로 간 거리를 그대로 — 이 직진은 τ·잔여·시간 직진엔 안 넣는다.
                 t_onset_ok 가 없는 옛 기록은 forward 의 출발 규칙(3D 거리 속도 > 정속 × ONSET_FRACTION 이 ONSET_N 장 연속)을
                 프레임으로 다시 돌리고 그 직진의 처리 지연(detect_age_s)을 더한다 — 추정기는 튀는 점을 빼고 맞추니 근사다.
                 (2026-10-01 전진: 출발 인정까지 약 11 cm + 미끄럼 약 5 cm ≈ 0.16 m. 정속 × τ = 0.085 m 는 출발 인정까지를 빼먹었다.
                  τ_기준은 정속에서 잰 값이라 낮은 속도의 미끄럼을 1~2 cm 크게 잡는다 — 최소걸음이 그만큼 크다)

    python -m src.models.control.learnfile                    # 새 기록만 더한다 (주행 시작 때 run.py 가 할 일). 장소는 config/site.py
    python -m src.models.control.learnfile --relearn          # 비우고 지금부터 새로 (차가 바뀌었을 때)
    python -m src.models.control.learnfile --relearn 20261001 # 비우고 그 날짜부터의 기록을 지금 규칙으로 다시 읽어 쌓는다
    python -m src.models.control.learnfile --site krri ...    # 다른 장소 파일을 다룬다
"""
if __package__ in (None, ""):
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(next(d for d in _Path(__file__).resolve().parents
                                 if (d / "config").is_dir() and (d / "src").is_dir())))
    __package__ = "src.models.control"

import bisect
import json
import math
import statistics
from datetime import datetime
from pathlib import Path

from config import control as C
from config import detection as D
from config import imu as I
from config.site import SITE
from ...bootstrap import ROOT, session_key, session_name, sessions
from ...utils.gyro import Gyro, max_rate_dps
from ..detection.estimate import MIN_N, WINDOW_S
from . import forward as F
from . import learn as LN


def path_for(site=None):
    """그 장소의 학습 파일 — config/learned/<장소>.json (2026-10-02: 장소마다 따로)."""
    return ROOT / "config" / "learned" / ("%s.json" % (site or SITE))


STRENGTHS = (C.ROTATE_JOYSTICK_DEFLECTION, C.ROTATE_FINE_JOYSTICK_DEFLECTION)       # 30, 20
REF_TAU = {C.ROTATE_JOYSTICK_DEFLECTION: LN.KWU_SEED["rot_tau_s"]["L"],             # 0.283 s 광운대 응답에서 유도
           C.ROTATE_FINE_JOYSTICK_DEFLECTION: LN.FINE_SEED["rot_tau_s"]["Lf"]}      # 0.168 s 우리 9/21 실측
TAU_LIMIT = LN.Learner.TAU_ROT          # (0.05, 1.0) s — 학습기와 같은 범위
RES_LIMIT_DEG = 2.0                     # 잔여 ±2° — 학습기(Learner.rot_residual)와 같은 범위
FINAL_AFTER_S = Gyro.STILL_AFTER_S      # 되튐 뒤 각 = 회전이 멎은 뒤 이만큼 (옛 기록). run.py STILL_S(회전 뒤 기다리는 1초)와 같은 근거
EMA_ALPHA = 0.5                         # 옛 기록 역산용 — 2026-10-01 까지 학습기의 계수 (learn.Ema)
MOTION = ("forward", "forward_slow", "backward", "rotate_left_slow", "rotate_right_slow")
DIRS = {"forward": str(F.strength_of("forward")), "backward": str(F.strength_of("backward"))}   # 직진 칸 이름 → CAN byte2 (67 · 187)
FWD_TAU_REF_S = LN.FWD_SEED["fwd_tau_s"][DIRS["forward"]]   # 0.295 s 직진 관성 기준 — 우리 실측 (learn.FWD_SEED)
FWD_RES_LIMIT_M = LN.Learner.FWD_RESIDUAL[1]        # 잔여 ±0.2 m — 학습기와 같은 범위


def _skey(k):
    return "strength_%d" % k


def empty():
    return {"upto": None, "straight": {d: _straight_acc() for d in DIRS},
            "inertia": {_skey(k): _inertia_acc() for k in STRENGTHS}, "gyro": _gyro_acc()}


def _straight_acc():
    return {"timed": {"n": 0, "h": 0.0, "d": 0.0, "hh": 0.0, "hd": 0.0, "dd": 0.0},
            "stop": {"n": 0, "v": 0.0, "e": 0.0, "vv": 0.0, "ve": 0.0, "ee": 0.0},
            "min": {"n": 0, "x": 0.0, "xx": 0.0}}


def _inertia_acc():
    return {"n": 0, "w": 0.0, "e": 0.0, "ww": 0.0, "we": 0.0, "ee": 0.0,
            "startup_n": 0, "startup": 0.0, "rate_n": 0, "rate": 0.0}


def _gyro_acc():
    return {"mount": None,
            "closed": {"a": 0.0, "b": 0.0, "c": 0.0, "n": 0, "mounts": 0},
            "runs": {"n": 0, "w": 0.0, "wd": 0.0, "wdd": 0.0}}


def _mount(since, c0):
    return {"since": since, "c0": [float(c0[0]), float(c0[1])], "xx": [0.0] * 6, "xy": [0.0] * 3, "yy": 0.0, "n": 0}


def load(path=None, log=None):
    """파일 → 합계. 옛 모양(직진 칸이 없다)이면 처음부터 다시 읽게 빈 것을 준다 — 기록이 남아 있어 그대로 다시 쌓인다."""
    p = Path(path) if path else path_for()
    store = empty()
    if not p.is_file():
        return store
    body = json.loads(p.read_text(encoding="utf-8"))
    if "inertia" not in body or "straight" not in body or any(
            "min" not in (((body["straight"].get(d) or {}).get("sums")) or {}) for d in DIRS):   # 최소걸음 칸 없음 (2026-10-11 전)
        if log:
            log("  학습 파일이 옛 모양이다 — 기록을 처음부터 다시 읽어 새 모양으로 쌓는다")
        return store
    store["upto"] = body.get("upto")
    for d in DIRS:
        src = ((body.get("straight") or {}).get(d) or {}).get("sums") or {}
        for part in ("timed", "stop", "min"):
            store["straight"][d][part].update({k: v for k, v in (src.get(part) or {}).items() if k in store["straight"][d][part]})
    for k in STRENGTHS:
        src = ((body.get("inertia") or {}).get(_skey(k)) or {}).get("sums") or {}
        store["inertia"][_skey(k)].update({key: src[key] for key in _inertia_acc() if key in src})
    sums = (body.get("gyro") or {}).get("sums") or {}
    if sums.get("mount"):
        store["gyro"]["mount"] = dict(_mount(sums["mount"].get("since"), sums["mount"].get("c0") or (0.0, 0.0)), **sums["mount"])
    for part in ("closed", "runs"):
        store["gyro"][part].update({k: v for k, v in (sums.get(part) or {}).items() if k in store["gyro"][part]})
    return store


def save(store, path=None):
    """파일에는 칸마다 **그 장소·그 칸 자료로 낸 값**만 적는다 — 빌린 값(다른 장소 · 후진 칸의 전진 값)이 아니다.
    무엇을 빌려 쓰는지는 주행이 시작할 때 use 가 정하고 로그에 찍는다. 시간 직진이 MIN_N 번 미만이면 speed·dead 는 null."""
    u = use(store, borrow=False)
    out = {"upto": store["upto"], "straight": {}, "inertia": {}, "gyro": {}}
    for d in DIRS:
        m = straight_model(store, d)
        out["straight"][d] = {"speed_mps": m["speed_mps"], "dead_s": m["dead_s"], "tau_s": m["tau_s"], "residual_m": m["residual_m"],
                              "min_step_m": m["min_step_m"], "n_timed": m["n_timed"], "n_stop": m["n_stop"], "n_min": m["n_min"],
                              "sums": store["straight"][d]}
    for k in STRENGTHS:
        m = u["inertia"][k]
        out["inertia"][_skey(k)] = {"tau_s": m["tau_s"], "residual_deg": m["residual_deg"], "startup_s": m["startup_s"],
                                    "rate_dps": m["rate_dps"], "n": m["n"], "sums": store["inertia"][_skey(k)]}
    g = u["gyro"]
    out["gyro"] = {"scale": g["scale"], "cam_to_rot_center_m": g["cam_to_rot_center_m"],
                   "rot_center_lateral_m": g["rot_center_lateral_m"], "n": g["n"], "sums": store["gyro"]}
    p = Path(path) if path else path_for()
    p.parent.mkdir(parents=True, exist_ok=True)
    tmp = p.with_suffix(".tmp")
    tmp.write_text(json.dumps(out, ensure_ascii=False, indent=1), encoding="utf-8")
    tmp.replace(p)                                      # 쓰다 죽어도 반쪽 파일이 안 남게


# ── 합계 → 쓸 값 ────────────────────────────────────────────────────────
def use(store, site=None, borrow=True, others=None):
    """{"inertia": {30: 모델, 20: 모델}, "straight": {"forward": 모델, "backward": 모델}, "gyro": 모델} — 주행이 이번 run 내내 쓸 값.
    borrow=True 면 그 장소 자료가 칸마다 MIN_N 번이 안 될 때 다른 장소 파일(config/learned/*.json)에서 빌린다 (2026-10-02 결정:
    krri 첫 run 은 kw 값으로 — 같은 차·같은 콘크리트). 모델마다 "from" 에 출처(own / 장소 이름 / seed)를 적는다.
    회전중심은 빌리지 않는다(카메라 장착마다 다르다). 후진은 자료가 없으면 전진 값을 쓴다(2026-10-02 사용자 결정).
    others = [(장소, 합계)] — 주지 않으면 config/learned/ 의 다른 장소 파일을 읽는다."""
    site = site or SITE
    if not borrow:
        others = []
    elif others is None:
        others = _other_stores(site)
    out = {"inertia": {}, "straight": {}, "gyro": None}
    for k in STRENGTHS:
        m = dict(rotation_model(store, k), **{"from": "own" if store["inertia"][_skey(k)]["n"] else "seed"})
        if m["n"] < MIN_N:
            cand = [(name, rotation_model(st, k)) for name, st in others]
            cand = [(name, c) for name, c in cand if c["n"] >= MIN_N]
            if cand:
                name, c = max(cand, key=lambda x: x[1]["n"])
                m = dict(c, **{"from": name})
        out["inertia"][k] = m
    own = {d: straight_model(store, d) for d in DIRS}
    oth = [(name, {d: straight_model(st, d) for d in DIRS}) for name, st in others]
    for d in DIRS:
        out["straight"][d] = _pick_straight(d, own, oth)
    g = dict(gyro_model(store), **{"scale_from": "own"})
    if g["scale_runs"] < MIN_N:                                  # 눈금은 카메라 안 센서 값이라 장소를 안 탄다 — 판이 많은 쪽에서
        cand = [(name, gyro_model(st)) for name, st in others]
        cand = [(name, go) for name, go in cand if go["scale_runs"] >= MIN_N]
        if cand:
            name, go = max(cand, key=lambda x: x[1]["scale_runs"])
            g.update(scale=go["scale"], scale_fit=go["scale_fit"], scale_se=go["scale_se"], scale_used=go["scale_used"],
                     scale_from=name)
    out["gyro"] = g
    return out


def _other_stores(site):
    """다른 장소들의 학습 파일 [(장소, 합계)]. 못 읽는 파일은 건너뛴다."""
    out = []
    folder = path_for(site).parent
    for p in sorted(folder.glob("*.json")) if folder.is_dir() else []:
        if p.stem == site:
            continue
        try:
            out.append((p.stem, load(p)))
        except Exception:
            continue
    return out


def _pick_straight(d, own, oth):
    """직진 한 칸을 고른다 — 시간 직진(speed·dead)과 끊는 값(τ·잔여)을 따로, 자료가 MIN_N 번 넘는 첫 출처에서.
    순서: 그 장소 → 다른 장소(자료가 많은 쪽) → (후진이면) 같은 순서로 전진 → 씨앗. timed_from · stop_from 에 출처를 적는다."""
    dirs = (d,) if d == "forward" else (d, "forward")

    def first(ok, nkey):
        for dd in dirs:
            tag = "" if dd == d else "·전진"
            if ok(own[dd]):
                return "own" + tag, own[dd]
            cand = [(name, m[dd]) for name, m in oth if ok(m[dd])]
            if cand:
                name, x = max(cand, key=lambda c: c[1][nkey])
                return name + tag, x
        return None
    m = dict(own[d])
    src = first(lambda x: x["n_timed"] >= MIN_N and x["speed_mps"] is not None, "n_timed")
    if src:
        m.update(speed_mps=src[1]["speed_mps"], dead_s=src[1]["dead_s"], timed_from=src[0])
    else:
        key = DIRS[d]
        m.update(speed_mps=LN.KWU_SEED["fwd_speed_mps"][key], dead_s=LN.KWU_SEED["fwd_startup_s"][key], timed_from="seed")
    src = first(lambda x: x["n_stop"] >= MIN_N, "n_stop")
    if src:
        m.update(tau_s=src[1]["tau_s"], residual_m=src[1]["residual_m"], stop_from=src[0])
    else:
        m.update(stop_from="own" if m["n_stop"] else "seed")
    src = first(lambda x: x["min_step_m"] is not None, "n_min")
    if src:
        m.update(min_step_m=src[1]["min_step_m"], min_step_sd_m=src[1]["min_step_sd_m"], min_from=src[0])
    else:
        m.update(min_from="formula")                      # forward.min_step_m 이 정속 × τ 로 (작게 나온다)
    return m


def straight_model(store, d):
    """그 장소 자료만으로 낸 직진 값 {"speed_mps", "dead_s", "tau_s", "residual_m", "min_step_m", "n_timed", "n_stop", "n_min", ...}.
    시간 직진·최소걸음은 MIN_N 번이 안 되거나 말이 안 되는 값이면 None (use 가 다른 데서 빌리거나 씨앗·식으로)."""
    acc = (store.get("straight") or {}).get(d) or _straight_acc()
    t, s = acc["timed"], acc["stop"]
    mn = acc.get("min") or {"n": 0, "x": 0.0, "xx": 0.0}
    min_step = min_sd = None
    if mn["n"] >= MIN_N:
        min_step = mn["x"] / mn["n"]
        min_sd = math.sqrt(max(0.0, mn["xx"] - mn["n"] * min_step * min_step) / (mn["n"] - 1))
    speed = dead = None
    if t["n"] >= MIN_N:
        n = t["n"]
        mh, md = t["h"] / n, t["d"] / n
        shh = t["hh"] - n * mh * mh
        shd = t["hd"] - n * mh * md
        if shh > 1e-9:
            a = shd / shh
            b = md - a * mh
            if LN.Learner.SPEED_FWD[0] <= a <= LN.Learner.SPEED_FWD[1] and LN.Learner.STARTUP_FWD[0] <= -b / a <= LN.Learner.STARTUP_FWD[1]:
                speed, dead = a, -b / a
    n = s["n"]
    a = c = 0.0
    se_c, slope = None, False
    if n:
        mv, me = s["v"] / n, s["e"] / n
        a = me
        sxx = s["vv"] - n * mv * mv
        sxy = s["ve"] - n * mv * me
        syy = s["ee"] - n * me * me
        if n >= MIN_N and sxx > 1e-12:
            c_fit = sxy / sxx
            resid = max(0.0, syy - c_fit * sxy)
            se_c = math.sqrt(resid / (n - 2) / sxx) if n > 2 else None
            if se_c is not None and abs(c_fit) > C.UNCERTAIN_FACTOR * se_c:
                c, a, slope = c_fit, me - c_fit * mv, True
    tau, res = FWD_TAU_REF_S + c, a
    clamped = []
    lo, hi = LN.Learner.TAU_FWD
    if not (lo <= tau <= hi):
        tau = min(hi, max(lo, tau))
        clamped.append("tau")
        if n:
            c = tau - FWD_TAU_REF_S
            res = a = s["e"] / n - c * s["v"] / n
    if abs(res) > FWD_RES_LIMIT_M:
        res = math.copysign(FWD_RES_LIMIT_M, res)
        clamped.append("residual")
    return {"speed_mps": None if speed is None else round(speed, 4), "dead_s": None if dead is None else round(dead, 3),
            "tau_s": round(tau, 4), "residual_m": round(res, 4), "n_timed": t["n"], "n_stop": n,
            "min_step_m": None if min_step is None else round(min_step, 4), "min_step_sd_m": None if min_sd is None else round(min_sd, 4),
            "n_min": mn["n"],
            "a": round(a, 4), "c": round(c, 4), "se_c": None if se_c is None else round(se_c, 4), "slope_used": slope,
            "clamped": clamped, "ref_tau_s": FWD_TAU_REF_S}


def rotation_model(store, strength):
    """{"tau_s", "residual_deg", "n", "a", "c", "se_c", "slope_used", "clamped", "startup_s", "rate_dps"} — 자료가 없으면 기준값."""
    acc = store["inertia"].get(_skey(strength)) or _inertia_acc()
    ref = REF_TAU[strength]
    n = acc["n"]
    a = c = 0.0
    se_c, slope = None, False
    if n:
        mw, me = acc["w"] / n, acc["e"] / n
        a = me
        sxx = acc["ww"] - n * mw * mw
        sxy = acc["we"] - n * mw * me
        syy = acc["ee"] - n * me * me
        if n >= MIN_N and sxx > 1e-9:
            c_fit = sxy / sxx
            resid = max(0.0, syy - c_fit * sxy)                  # Σ(e − a − cω)²
            se_c = math.sqrt(resid / (n - 2) / sxx) if n > 2 else None
            if se_c is not None and abs(c_fit) > C.UNCERTAIN_FACTOR * se_c:
                c, a, slope = c_fit, me - c_fit * mw, True      # 기울기가 우연이 아닐 때만
    tau, res = ref + c, a
    clamped = []
    if not (TAU_LIMIT[0] <= tau <= TAU_LIMIT[1]):
        tau = min(TAU_LIMIT[1], max(TAU_LIMIT[0], tau))
        clamped.append("tau")
        if n:                                                   # 기울기를 잘랐으면 잔여를 그 기울기에 맞춰 다시 낸다 (최소제곱)
            c = tau - ref
            res = a = acc["e"] / n - c * acc["w"] / n
    if abs(res) > RES_LIMIT_DEG:
        res = math.copysign(RES_LIMIT_DEG, res)
        clamped.append("residual")
    return {"tau_s": round(tau, 4), "residual_deg": round(res, 3), "n": n, "a": round(a, 3), "c": round(c, 4),
            "se_c": None if se_c is None else round(se_c, 4), "slope_used": slope, "clamped": clamped, "ref_tau_s": ref,
            "startup_s": round(acc["startup"] / acc["startup_n"], 3) if acc["startup_n"] else None, "startup_n": acc["startup_n"],
            "rate_dps": round(acc["rate"] / acc["rate_n"], 2) if acc["rate_n"] else None, "rate_n": acc["rate_n"]}


# ── 자이로: 작은 행렬 ─────────────────────────────────────────────────
_IDX = ((0, 0), (0, 1), (0, 2), (1, 1), (1, 2), (2, 2))


def _rot(g):
    """R(−g) — 차가 g [rad] 만큼 왼쪽(반시계)으로 돌면 고정된 점은 차 좌표(앞·왼쪽)에서 −g 만큼 돈다."""
    c, s = math.cos(g), math.sin(g)
    return ((c, s), (-s, c))


def _rows(q1, q2, g, c0):
    """회전 하나 → 두 줄 X(2×3) · y(2). 열 = (δ, 중심 앞, 중심 왼쪽).
    두 줄을 q2 쪽(지름 = 거리 방향)과 그 직각(접선 = 화면위치 방향)으로 돌려 **무게**를 준다 (무게 = 1/분산):
      지름 — 거리 오차는 거리²에 비례한다 (estimate 의 밀림 모델: 거리 σ 가 (거리/기준거리)² 로 는다)   → 1/d⁴
      접선 — 화면위치 β 는 0.004° 로 정밀하고, 남는 건 각 오차(자이로·되튐)라 거리에 비례              → 1/d²
    모양(d² · d)은 거리 잡음 모델에서 온 것이다. 2026-10-01 잔차로 모양까지 맞추면 지수가 1.3 · 1.6 이 가장 그럴듯하지만, 기록을 다시 뽑아
    200번 맞추면 0.45~2.05 · 0.9~2.2 (90 %) 로 흔들린다 — 오늘 자료로는 모양을 못 가르고 2 · 1 도 그 안이다. 어느 모양으로 줘도
    회전중심은 앞 2 mm · 좌우 5 mm 만 달랐다. 틀린 건 같은 무게뿐이었다. 자료가 쌓여 범위가 좁아지면 다시 본다.
    같은 무게로 맞추면 8~10 m 표본의 거리 오차(잔차 130~140 mm, 보통의 5배)가 회전중심 좌우를 끌고 갔다:
    같은 무게 — 좌우 +0.064(판 하나 빼면 −0.003) · 이 무게 — −0.013(판 하나씩 빼면 −0.033~+0.032, 앞뒤는 0.400~0.412 로 안정)."""
    R = _rot(g)
    r1 = (R[0][0] * q1[0] + R[0][1] * q1[1], R[1][0] * q1[0] + R[1][1] * q1[1])
    y = (q2[0] - r1[0], q2[1] - r1[1])
    dx, dy = q1[0] - c0[0], q1[1] - c0[1]
    jd = (-dy, dx)                                               # J·(q1 − c0)
    u = (-g * (R[0][0] * jd[0] + R[0][1] * jd[1]), -g * (R[1][0] * jd[0] + R[1][1] * jd[1]))
    X = ((u[0], 1.0 - R[0][0], -R[0][1]), (u[1], -R[1][0], 1.0 - R[1][1]))
    d = math.hypot(q2[0], q2[1])
    if d <= 0:
        return X, y
    er, et = (q2[0] / d, q2[1] / d), (-q2[1] / d, q2[0] / d)    # 지름 · 접선 단위벡터
    Xw, yw = [], []
    for e, w in ((er, 1.0 / d ** 4), (et, 1.0 / d ** 2)):
        s = math.sqrt(w)
        Xw.append(tuple(s * (e[0] * X[0][i] + e[1] * X[1][i]) for i in range(3)))
        yw.append(s * (e[0] * y[0] + e[1] * y[1]))
    return tuple(Xw), tuple(yw)


def _accumulate(acc, X, y):
    for r in range(2):
        for k, (i, j) in enumerate(_IDX):
            acc["xx"][k] += X[r][i] * X[r][j]
        for i in range(3):
            acc["xy"][i] += X[r][i] * y[r]
        acc["yy"] += y[r] * y[r]
    acc["n"] += 1


def _parts(acc):
    """XᵀX = [[a, uᵀ], [u, M]] · Xᵀy = [p, v]."""
    a00, a01, a02, a11, a12, a22 = acc["xx"]
    p, v1, v2 = acc["xy"]
    return a00, (a01, a02), ((a11, a12), (a12, a22)), p, (v1, v2)


def _inv2(M):
    det = M[0][0] * M[1][1] - M[0][1] * M[1][0]
    scale = abs(M[0][0]) + abs(M[1][1])
    if scale <= 0 or abs(det) <= 1e-12 * scale * scale:
        return None
    return ((M[1][1] / det, -M[0][1] / det), (-M[1][0] / det, M[0][0] / det))


def _mul(M, x):
    return (M[0][0] * x[0] + M[0][1] * x[1], M[1][0] * x[0] + M[1][1] * x[1])


def _dot(a, b):
    return a[0] * b[0] + a[1] * b[1]


def _reduce(acc):
    """그 장착 안에서 중심을 지운 δ 의 2차식: SSE(δ) = C − 2δ·B + δ²·A. (A, B, C) 또는 None."""
    a, u, M, p, v = _parts(acc)
    Mi = _inv2(M)
    if Mi is None:
        return None
    Miu, Miv = _mul(Mi, u), _mul(Mi, v)
    return a - _dot(u, Miu), p - _dot(u, Miv), acc["yy"] - _dot(v, Miv)


def _center(acc, delta):
    """δ 를 고정하고 그 장착의 중심 (앞, 왼쪽) · 잔차 SSE · M⁻¹. 못 풀면 None."""
    a, u, M, p, v = _parts(acc)
    Mi = _inv2(M)
    if Mi is None:
        return None
    c = _mul(Mi, (v[0] - u[0] * delta, v[1] - u[1] * delta))
    th = (delta, c[0], c[1])
    S = ((a, u[0], u[1]), (u[0], M[0][0], M[0][1]), (u[1], M[1][0], M[1][1]))
    b = (p, v[0], v[1])
    sse = acc["yy"] - 2 * sum(th[i] * b[i] for i in range(3)) + sum(th[i] * S[i][j] * th[j] for i in range(3) for j in range(3))
    return c, max(0.0, sse), Mi


def _to_vehicle(cx, cy, yaw_deg):
    """카메라 축 중심 (앞 cx, 왼쪽 cy) → config 꼴 (CAM_TO_ROT_CENTER_M = −앞, ROT_CENTER_LATERAL_M = 왼쪽), 차체 축.
    tools/analyze_calibrate.to_vehicle_frame 과 같은 식 — 카메라가 CAM_YAW_OFFSET_DEG 만큼 틀어져 달려 있다."""
    r = math.radians(yaw_deg or 0.0)
    cs, sn = math.cos(r), math.sin(r)
    return -(cs * cx - sn * cy), sn * cx + cs * cy


def _from_vehicle(A, b, yaw_deg):
    r = -math.radians(yaw_deg or 0.0)
    cs, sn = math.cos(r), math.sin(r)
    ox, oy = -A, b
    return cs * ox - sn * oy, sn * ox + cs * oy


def gyro_model(store):
    """{"scale", "scale_fit", "scale_se", "scale_runs", "scale_used", "cam_to_rot_center_m", "rot_center_lateral_m",
    "n", "center_se_m", "since", "center_cam_m"} — 눈금은 확실할 때만 1 이 아니고,
    중심은 지금 장착 자료가 MIN_N 이상일 때만 값이 있다."""
    g = store["gyro"]
    mnt, cl, runs = g.get("mount"), g["closed"], g["runs"]
    A, B, Cq, n, mounts = cl["a"], cl["b"], cl["c"], cl["n"], cl["mounts"]
    if mnt and mnt["n"]:
        red = _reduce(mnt)
        if red is not None:
            A, B, Cq, n, mounts = A + red[0], B + red[1], Cq + red[2], n + mnt["n"], mounts + 1
    delta = se_pool = se_runs = None
    if A > 0 and n:
        delta = B / A
        dof = 2 * n - 1 - 2 * mounts
        if dof > 0:
            se_pool = math.sqrt(max(0.0, Cq - B * B / A) / dof / A)
    if runs["n"] >= 2 and runs["w"] > 0:
        dw = runs["wd"] / runs["w"]
        se_runs = math.sqrt(max(0.0, runs["wdd"] - runs["w"] * dw * dw) / ((runs["n"] - 1) * runs["w"]))
    ses = [x for x in (se_pool, se_runs) if x is not None]
    se = max(ses) if ses else None
    used = (delta is not None and se is not None and runs["n"] >= MIN_N and abs(delta) > C.UNCERTAIN_FACTOR * se)
    out = {"scale": round(1.0 + delta, 5) if used else 1.0, "scale_fit": None if delta is None else round(1.0 + delta, 5),
           "scale_se": None if se is None else round(se, 5), "scale_runs": runs["n"], "scale_used": used,
           "cam_to_rot_center_m": None, "rot_center_lateral_m": None,
           "n": mnt["n"] if mnt else 0, "center_se_m": None, "since": mnt["since"] if mnt else None,
           "center_cam_m": None}
    if mnt and mnt["n"] >= MIN_N:
        got = _center(mnt, delta if used else 0.0)
        if got is not None:
            (cx, cy), sse, Mi = got
            s2 = sse / max(1, 2 * mnt["n"] - 2)
            A_v, b_v = _to_vehicle(cx, cy, C.CAM_YAW_OFFSET_DEG)
            out.update(cam_to_rot_center_m=round(A_v, 4), rot_center_lateral_m=round(b_v, 4),
                       center_se_m=[round(math.sqrt(max(0.0, s2 * Mi[0][0])), 4), round(math.sqrt(max(0.0, s2 * Mi[1][1])), 4)],
                       center_cam_m=[round(cx, 4), round(cy, 4)])
    return out


def _close_mount(store):
    """지금 장착을 접는다 — 그 눈금 몫(중심을 지운 2차식)만 closed 에 더한다."""
    g = store["gyro"]
    m = g.get("mount")
    if m and m["n"]:
        red = _reduce(m)
        if red is not None:
            cl = g["closed"]
            cl["a"] += red[0]
            cl["b"] += red[1]
            cl["c"] += red[2]
            cl["n"] += m["n"]
            cl["mounts"] += 1
    g["mount"] = None


def _ensure_mount(store, since=None, c0=None):
    g = store["gyro"]
    if g.get("mount") is None:
        if c0 is None:                                           # calibrate 회전중심 없이 시작한 판 — config 값에서 출발
            A = C.CAM_TO_ROT_CENTER_M if C.CAM_TO_ROT_CENTER_M is not None else 0.0
            b = C.ROT_CENTER_LATERAL_M if C.ROT_CENTER_LATERAL_M is not None else 0.0
            c0 = _from_vehicle(A, b, C.CAM_YAW_OFFSET_DEG)
        g["mount"] = _mount(since, c0)
    return g["mount"]


def _run_scale(samples, c0):
    """판 하나만으로 맞춘 δ 와 그 분산. 판끼리 흩어짐을 보려고. 못 풀면 None."""
    if len(samples) < MIN_N:
        return None
    acc = _mount(None, c0)
    for q1, q2, g in samples:
        _accumulate(acc, *_rows(q1, q2, g, c0))
    red = _reduce(acc)
    if red is None or red[0] <= 0:
        return None
    A, B, Cq = red
    d = B / A
    dof = 2 * acc["n"] - 3
    var = (max(0.0, Cq - B * B / A) / dof) / A if dof > 0 else None
    return (d, var) if var else None


# ── 기록 읽기 ──────────────────────────────────────────────────────────
def _jsonl(p):
    out = []
    if p.is_file():
        for line in p.read_text(encoding="utf-8").splitlines():
            try:
                out.append(json.loads(line))
            except json.JSONDecodeError:
                pass                                     # 죽으면서 반만 적힌 줄
    return out


def _skip_reason(d, cfg):
    """기록 통째로 뺄 이유. 없으면 ""."""
    ex = d / "exclude"
    if ex.exists():
        return "사람이 뺌: " + (ex.read_text(encoding="utf-8").strip() or "(이유 없음)")
    if not cfg:
        return "config.json 없음 (출발 전에 끝난 판)"
    if cfg.get("mode") == "dry":
        return "dry-run"
    return ""


def _q(rows, dz):
    """서 있는 프레임들 → 태그 자리 (앞, 왼쪽) — 화면위치 β 와 거리의 중앙값. PnP 방향·좌우는 안 쓴다(자세마다 치우친다)."""
    d = statistics.median(r["distance"] for r in rows)
    b = math.radians(statistics.median(r["beta_deg"] for r in rows))
    rho = math.sqrt(max(0.0, d * d - dz * dz))
    return rho * math.cos(b), -rho * math.sin(b)


def _dz(rows):
    vs = [-r["vertical"] for r in rows if r.get("vertical") is not None and r.get("angle_ok", True)]
    return statistics.median(vs) if vs else (D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M)


class _Frames:
    """frame.jsonl 중 태그를 본 프레임 — 시각으로 찾는다."""

    def __init__(self, rows):
        self.rows = [f for f in rows if f.get("t") is not None and f.get("gyro_deg") is not None]
        self.ts = [f["t"] for f in self.rows]

    def between(self, t0, t1, need=("distance", "beta_deg")):
        i, j = bisect.bisect_left(self.ts, t0), bisect.bisect_right(self.ts, t1)
        return [f for f in self.rows[i:j] if all(f.get(k) is not None for k in need)]

    def gyro_near(self, t):
        i = bisect.bisect_left(self.ts, t)
        best = None
        for j in (i - 1, i):
            if 0 <= j < len(self.rows) and abs(self.ts[j] - t) <= WINDOW_S and (best is None or abs(self.ts[j] - t) < abs(self.ts[best] - t)):
                best = j
        return self.rows[best]["gyro_deg"] if best is not None else None


def run_samples(d):
    """판 폴더 하나 → {"why": 통째 뺀 이유, "inertia": 회전 응답 표본, "gyro": 자이로 표본 [(q1, q2, g)],
    "straight": {"forward": 직진 표본, "backward": ...}, "skip_<칸>": {뺀 이유: 개수}}."""
    d = Path(d)
    out = {"why": "", "inertia": [], "skip_inertia": {}, "gyro": [], "skip_gyro": {},
           "straight": {k: [] for k in DIRS}, "skip_straight": {}, "minstep": {k: [] for k in DIRS}, "skip_minstep": {}}
    cfg = json.loads((d / "config.json").read_text(encoding="utf-8")) if (d / "config.json").is_file() else None
    out["why"] = _skip_reason(d, cfg)
    if out["why"]:
        return out
    ev = _jsonl(d / "event.jsonl")
    fr = _Frames(_jsonl(d / "frame.jsonl"))
    finals = {round(e["t_cmd"], 6): e for e in ev if e.get("event") == "rotate_final" and e.get("t_cmd") is not None}
    motion_t = sorted(e["t_cmd"] for e in ev if e.get("event") == "cmd" and e.get("movement") in MOTION and e.get("t_cmd") is not None)
    out["inertia"], out["skip_inertia"] = _inertia_samples(cfg, ev, fr, finals)
    out["gyro"], out["skip_gyro"] = _gyro_samples(cfg, ev, fr, finals, motion_t)
    out["straight"], out["skip_straight"] = _straight_samples(cfg, ev)
    out["minstep"], out["skip_minstep"] = _min_step_samples(ev, fr)
    return out


def _start_remaining(e):
    """직진 시작 때 남은거리. 새 기록은 forward 가 적은 값, 옛 기록은 간 거리 + 최종 남은거리 (간 거리 = 시작 − 끝)."""
    if "start_remaining_m" in e:
        return e["start_remaining_m"]
    return e["travelled_m"] + e["d_final"]


def _straight_samples(cfg, ev):
    """카메라 직진들 → ({"forward": [표본], "backward": [...]}, {뺀 이유: 개수}).
    표본 = {"h": 명령을 쥔 시간 [s], "d": 실제 간 거리 [m], "v": 끊을 때 속도, "e": 기준값으로 끊었다면 지나쳤을 양, "err": 실제 지나친 양}."""
    skipped, out = {}, {k: [] for k in DIRS}

    def skip(r):
        skipped[r] = skipped.get(r, 0) + 1
    by_key = {k: name for name, k in DIRS.items()}                  # "67" → "forward"
    seeds = cfg.get("seeds") or {}
    # 옛 기록(2026-10-01 까지 — 주행 중 계수 0.5 로 배우던 때) 역산용: 그때 쓰던 τ(강도별)·잔여(전진·후진이 하나)를 따라간다.
    # 시작값 = 그 판이 받은 씨앗 (τ 씨앗이 없었으면 학습기는 0 으로 시작했다. 잔여는 다음 판에 안 넘어가서 판마다 0 에서 시작했다)
    tau = {k: float((seeds.get("fwd_tau_s") or {}).get(k) or 0.0) for k in by_key}
    res = seeds.get("fwd_residual_m")
    res = float(res) if isinstance(res, (int, float)) else 0.0
    for e in ev:
        if e.get("event") != "forward_end":
            continue
        key = str(e.get("strength"))
        tau_used = e.get("tau_used", tau.get(key, 0.0))           # 새 기록(v5)은 forward 가 쓴 값을 적는다
        res_used = e.get("residual_used", res)
        L = e.get("learned") or {}
        if "tau" in L and key in tau:
            tau[key] = L["tau"]
        if "residual" in L:
            res = L["residual"]
        # ── 거르기 (2026-10-02 결정) ──
        why = ""
        if e.get("timed"):
            why = "시간 직진"
        elif key not in by_key:
            why = "강도 %s" % key
        elif e.get("reason") not in ("predicted", "settle_timeout"):
            why = "끝이 정상이 아님 (%s)" % e.get("reason")
        elif e.get("start_ok") is False or _start_remaining(e) is None or abs(_start_remaining(e)) < 1e-6:
            why = "시작 측정을 못 믿음"
        elif (e.get("blind_s") or 0.0) > F.BLIND_MAX_S:
            why = "태그를 %.1f s 넘게 놓침" % F.BLIND_MAX_S
        elif not e.get("t_onset") or e["t_onset"] > e["t_stop_cmd"] + 1e-6:
            why = "출발 보기 전에 끊김"
        elif not (e.get("speed_mps") or 0.0) > LN.Learner.SPEED_FWD[0]:
            why = "끊을 때 속도 모름"
        elif e.get("cut_at_onset"):
            why = "출발 인정하자마자 끊김 (최소걸음 재료)"     # 더 일찍 못 끊어 지나친 것 — τ·잔여 탓이 아니다 (F1-3)
        if why:
            skip(why)
            continue
        v, err = e["speed_mps"], e["travelled_m"] - e["target_m"]
        out[by_key[key]].append({"h": e["t_stop_cmd"] - e["t_cmd"], "d": e["travelled_m"], "v": v, "err": err,
                                 "e": err + v * (tau_used - FWD_TAU_REF_S) + res_used})
    return out, skipped


def _line(rows):
    """[(t, x)] 최소제곱 직선 (기울기, 절편). 점이 MIN_N 개 미만이거나 시간이 다 같으면 None."""
    n = len(rows)
    if n < MIN_N:
        return None
    mt, mx = sum(r[0] for r in rows) / n, sum(r[1] for r in rows) / n
    stt = sum((r[0] - mt) ** 2 for r in rows)
    if stt <= 0:
        return None
    a = sum((r[0] - mt) * (r[1] - mx) for r in rows) / stt
    return a, mx - a * mt


def _dist(fr, t0, t1):
    return [(f["t"], f["distance"]) for f in fr.between(t0, t1, need=("distance",))]


def _onset_replay(e, fr, sgn, cruise):
    """옛 기록(t_onset_ok 없음): forward 의 출발 규칙을 프레임으로 다시 돌린다 — 명령 뒤 프레임마다 그때까지 WINDOW_S 창의 3D 거리 기울기가
    가는 쪽으로 정속 × ONSET_FRACTION 을 ONSET_N 장 연속 넘으면 그 장 + 처리 지연(그 직진의 detect_age_s). 못 찾으면 None."""
    run = 0
    for t, _ in _dist(fr, e["t_cmd"] + 1e-9, e["t_stop_cmd"]):
        ln = _line(_dist(fr, t - WINDOW_S, t))
        if ln is not None and -sgn * ln[0] > cruise * F.ONSET_FRACTION:
            run += 1
            if run >= F.ONSET_N:
                return t + (e.get("detect_age_s") or 0.0)
        else:
            run = 0
    return None


def _min_step_samples(ev, fr):
    """카메라 직진들 → ({"forward": [최소걸음 표본 m], "backward": [...]}, {뺀 이유: 개수}) (F1-3, 2026-10-11).
    표본 = 가장 빨리 끊었으면 실제로 갔을 거리 — 위 모듈 설명 "최소걸음"."""
    skipped, out = {}, {k: [] for k in DIRS}

    def skip(r):
        skipped[r] = skipped.get(r, 0) + 1
    by_key = {k: name for name, k in DIRS.items()}
    for e in ev:
        if e.get("event") != "forward_end" or e.get("timed") or str(e.get("strength")) not in by_key:
            continue                                          # 이유는 직진 칸이 이미 센다
        name = by_key[str(e["strength"])]
        sgn = -1.0 if name == "backward" else 1.0
        if e.get("reason") not in ("predicted", "settle_timeout"):
            skip("끝이 정상이 아님 (%s)" % e.get("reason"))
            continue
        if (e.get("blind_s") or 0.0) > F.BLIND_MAX_S:
            skip("태그를 %.1f s 넘게 놓침" % F.BLIND_MAX_S)
            continue
        if not e.get("t_onset") or e["t_onset"] > e["t_stop_cmd"] + 1e-6:
            skip("출발 보기 전에 끊김")
            continue
        pre = [x for _, x in _dist(fr, e["t_cmd"] - WINDOW_S, e["t_cmd"])]
        if len(pre) < MIN_N:
            skip("명령 전 프레임 모자람")
            continue
        d0 = statistics.median(pre)
        if e.get("cut_at_onset"):
            end = [x for _, x in _dist(fr, e["t_settled"] - WINDOW_S / 2, e["t_settled"] + WINDOW_S / 2)]
            if len(end) < MIN_N:
                skip("멈춘 뒤 프레임 모자람")
                continue
            out[name].append(sgn * (d0 - statistics.median(end)))      # 실제로 간 거리 그대로
            continue
        cruise = e.get("speed_used") or LN.KWU_SEED["fwd_speed_mps"][str(e["strength"])]
        t_ok = e.get("t_onset_ok") or _onset_replay(e, fr, sgn, cruise)
        if not t_ok or t_ok > e["t_stop_cmd"]:
            skip("출발 인정 전에 끊김")                          # 그보다 일찍 끊었다 — 지금 규칙의 최소걸음이 아니다
            continue
        ln = _line(_dist(fr, t_ok - WINDOW_S / 2, t_ok + WINDOW_S / 2))
        if ln is None:
            skip("출발 인정 때 프레임 모자람")
            continue
        moved = sgn * (d0 - (ln[0] * t_ok + ln[1]))
        out[name].append(moved + max(0.0, -sgn * ln[0]) * FWD_TAU_REF_S)
    return out, skipped


def _still_before(e, fr):
    """명령 직전에 차가 멎어 있었나. 새 기록은 rotate() 가 적은 각속도, 옛 기록은 그 사이 프레임의 자이로 각. 모르면 None."""
    if e.get("pre_rate_dps") is not None:
        return abs(e["pre_rate_dps"]) <= I.IMU_MOVING_DPS
    pts = [(f["t"], f["gyro_deg"]) for f in fr.between(e["t_cmd"] - WINDOW_S, e["t_cmd"], need=())]
    if len(pts) < 2 or pts[-1][0] - pts[0][0] < WINDOW_S / 2:
        return None                                              # 그 사이 태그를 못 봐서 자이로 각이 안 남았다
    return max_rate_dps([q[0] for q in pts], [q[1] for q in pts], WINDOW_S) <= I.IMU_MOVING_DPS


def _final_t(e, finals):
    f = finals.get(round(e["t_cmd"], 6))
    return (f["t"], f) if f else (e["t_settled"] + FINAL_AFTER_S, None)


def _inertia_samples(cfg, ev, fr, finals):
    skipped = {}

    def skip(r):
        skipped[r] = skipped.get(r, 0) + 1
    ctl = cfg.get("control") or {}
    if ctl.get("ROTATE_JOYSTICK_DEFLECTION") != C.ROTATE_JOYSTICK_DEFLECTION:
        skip("회전 강도가 지금과 다름 (%s) — 판 통째" % ctl.get("ROTATE_JOYSTICK_DEFLECTION"))
        return [], skipped
    seeds = (cfg.get("seeds") or {})
    floor = {C.ROTATE_JOYSTICK_DEFLECTION: C.ROT_FLOOR_DEG if C.ROT_FLOOR_DEG is not None else LN.KWU_SEED["rot_floor_deg"],
             C.ROTATE_FINE_JOYSTICK_DEFLECTION: C.ROT_FLOOR_FINE_DEG if C.ROT_FLOOR_FINE_DEG is not None else LN.FINE_SEED["rot_floor_fine_deg"]}
    # 옛 기록(주행 중 학습) 역산용: 자리별로 그때 쓰던 τ·잔여를 따라간다. 시작값 = 그 판이 물려받은 씨앗
    tau = {s: (seeds.get("rot_tau_s") or {}).get(s) for s in LN.ROT_SIDES}
    res = {s: (seeds.get("rot_residual_deg") or {}).get(s) or 0.0 for s in LN.ROT_SIDES}
    res_seed = dict(res)
    n_learned = {s: 0 for s in LN.ROT_SIDES}
    mismatch = False
    purpose, out = "step", []
    for e in ev:
        k = e.get("event")
        if k == "decision":
            purpose = "sidestep" if e.get("decision") == "sidestep" else "step"
        elif k == "state" and e.get("state") == "SEARCH":
            purpose = "search"
        if k != "rotate_end":
            continue
        fine = bool(e.get("fine"))
        strength = C.ROTATE_FINE_JOYSTICK_DEFLECTION if fine else C.ROTATE_JOYSTICK_DEFLECTION
        side = LN.Learner.rot_side(e.get("target_deg", 0.0), fine)
        tau_used = e.get("tau_used", tau.get(side) or REF_TAU[strength])
        res_used = e.get("residual_used", res.get(side, 0.0))
        # 옛 학습 따라가기 — 이 회전이 그때 배운 값으로 다음 회전의 τ·잔여가 바뀐다. 기록된 값과 맞나도 본다 (v5 기록엔 learned 가 없다)
        L = e.get("learned") or {}
        if "residual" in L:
            x = max(-RES_LIMIT_DEG, min(RES_LIMIT_DEG, abs(e["turned_deg"]) - abs(e["target_deg"])))
            pred = x if (n_learned[side] == 0 and res_seed[side] == 0.0) else res[side] + EMA_ALPHA * (x - res[side])
            if abs(pred - L["residual"]) > 0.02:
                mismatch = True
            res[side] = L["residual"]
            n_learned[side] += 1
        if "tau" in L:
            tau[side] = L["tau"]
        # ── 거르기 ──
        why = ""
        p = e.get("purpose", purpose)
        if p != "step":
            why = "목적이 %s" % p
        elif e.get("reason") != "predicted" or not e.get("t_settled"):
            why = "정상 종료 아님 (%s)" % e.get("reason")
        elif not e.get("t_onset") or e["t_onset"] > e["t_stop_cmd"] + 1e-6:
            why = "출발 보기 전에 끊김"
        elif e.get("gaps", 0):
            why = "자이로 끊김"
        elif abs(e["target_deg"]) < floor[strength]:
            why = "하한 미만 요청"
        elif fine and (cfg.get("control") or {}).get("ROTATE_FINE_JOYSTICK_DEFLECTION") != C.ROTATE_FINE_JOYSTICK_DEFLECTION:
            why = "약한 회전 강도가 지금과 다름"
        if why:
            skip(why)
            continue
        still = _still_before(e, fr)
        if still is None:
            skip("시작 때 멎었는지 못 봄")
            continue
        if not still:
            skip("시작 때 덜 멎음")
            continue
        sgn = 1.0 if e["target_deg"] > 0 else -1.0
        t1, fin = _final_t(e, finals)
        if fin is not None:                                           # 새 기록: 회전 뒤 1초 대기 끝의 자이로 각 (run.py rotate_final)
            final = fin["final_deg"] * sgn
        else:
            g0, g1 = fr.gyro_near(e["t_cmd"]), fr.gyro_near(t1)
            final = None if (g0 is None or g1 is None) else (g1 - g0) * sgn
        if final is None:
            skip("되튐 뒤 각을 못 잼")
            continue
        w = abs(e.get("omega_at_stop") or 0.0)
        err = final - abs(e["target_deg"])
        e_ref = err - (w * (REF_TAU[strength] - tau_used) + (0.0 - res_used))
        out.append({"strength": strength, "w": w, "e": e_ref, "err_actual": err, "target": abs(e["target_deg"]),
                    "startup": e["t_onset"] - e["t_cmd"],
                    "rate": (abs(e["turned_at_stop"]) / (e["t_stop_cmd"] - e["t_onset"])) if e["t_stop_cmd"] - e["t_onset"] > 0.1 else None})
    if mismatch:
        skipped["역산이 기록과 안 맞음 — 판 통째 (옛 학습식을 다시 따라가 본 값이 기록된 학습값과 다르다)"] = len(out)
        return [], skipped
    return out, skipped


def _gyro_samples(cfg, ev, fr, finals, motion_t):
    """서서 태그를 본 채 돈 회전들 → [(q1, q2, g [rad])]. 목적은 안 가린다."""
    skipped, out = {}, []

    def skip(r):
        skipped[r] = skipped.get(r, 0) + 1
    scale = float(cfg.get("gyro_scale") or 1.0)                  # 그 판이 곱해 쓴 눈금 — 나눠서 센서 값으로
    for e in ev:
        if e.get("event") != "rotate_end" or not e.get("t_cmd"):
            continue
        if e.get("reason") != "predicted" or not e.get("t_settled"):
            skip("정상 종료 아님")
            continue
        if e.get("gaps", 0):
            skip("자이로 끊김")
            continue
        if not _still_before(e, fr):
            skip("시작 때 덜 멎음·못 봄")
            continue
        t1, _ = _final_t(e, finals)
        k = bisect.bisect_right(motion_t, e["t_stop_cmd"])          # 정지 명령 뒤 — 이 회전 자신의 명령은 그 앞이다
        if k < len(motion_t) and motion_t[k] <= t1:
            skip("회전 뒤 바로 움직임")
            continue
        before = fr.between(e["t_cmd"] - WINDOW_S, e["t_cmd"])
        after = fr.between(t1 - WINDOW_S, t1)
        if len(before) < MIN_N or len(after) < MIN_N:
            skip("전후 태그 프레임 부족")
            continue
        dz = _dz(before + after)
        g = (statistics.median(f["gyro_deg"] for f in after) - statistics.median(f["gyro_deg"] for f in before)) / scale
        out.append((_q(before, dz), _q(after, dz), math.radians(g)))
    return out, skipped


def calibrate_samples(d):
    """calibrate 폴더 → (새 장착의 출발 중심 c0 또는 None, 스윙 정지점 쌍 [(q1, q2, g)], 메모).
    회전중심 단계가 끝까지 간 calibrate 만 장착을 새로 시작시킨다 (카메라를 다시 달았다는 뜻)."""
    d = Path(d)
    if (d / "exclude").exists():
        return None, [], "사람이 뺌"
    ev = _jsonl(d / "event.jsonl")
    rc = [e for e in ev if e.get("event") == "rotcenter"]
    if not rc:
        return None, [], "회전중심 단계 없음"
    e = rc[-1]
    if e.get("cam_to_rot_center_m") is None:
        return None, [], "회전중심 적합 없음"
    c0 = (-float(e["cam_to_rot_center_m"]), float(e.get("rot_center_lateral_m") or 0.0))     # 카메라 축 (calibrate 가 낸 그대로)
    still = next((s for s in ev if s.get("event") == "still" and s.get("why") == "rotcenter" and s.get("vertical") is not None), None)
    dz = -still["vertical"] if still else (D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M)
    # 기록 순서대로: 정지점 · (그 사이 회전들) · 정지점. 사이 회전에 자이로 끊김이 있거나 정상 종료가 아니면 그 쌍은 뺀다 —
    # 2026-10-01 141357 스윙: 끊긴 두 회전(−9.57 · −13.06, USB 멈춤)은 자이로가 실제보다 18~21 % 적게 셌다
    out, prev, clean = [], None, True
    for x in ev:
        k = x.get("event")
        if k == "rotate_end":
            clean = clean and not x.get("gaps") and x.get("reason") == "predicted"
        elif k == "rotcenter_stop" and all(x.get(f) is not None for f in ("distance", "beta_deg", "gyro_deg")):
            if prev is not None and clean:
                out.append((_q([prev], dz), _q([x], dz), math.radians(x["gyro_deg"] - prev["gyro_deg"])))
            prev, clean = x, True
    return c0, out, ""


def _add_inertia(store, s):
    acc = store["inertia"][_skey(s["strength"])]
    w, e = s["w"], s["e"]
    acc["n"] += 1
    acc["w"] += w
    acc["e"] += e
    acc["ww"] += w * w
    acc["we"] += w * e
    acc["ee"] += e * e
    for key in ("startup", "rate"):
        v = s.get(key)
        if v is not None:
            acc[key + "_n"] += 1
            acc[key] += v


def _add_straight(store, name, s):
    t = store["straight"][name]["timed"]
    h, x = s["h"], s["d"]
    t["n"] += 1
    t["h"] += h
    t["d"] += x
    t["hh"] += h * h
    t["hd"] += h * x
    t["dd"] += x * x
    p = store["straight"][name]["stop"]
    v, e = s["v"], s["e"]
    p["n"] += 1
    p["v"] += v
    p["e"] += e
    p["vv"] += v * v
    p["ve"] += v * e
    p["ee"] += e * e


def _add_min(store, name, x):
    m = store["straight"][name]["min"]
    m["n"] += 1
    m["x"] += x
    m["xx"] += x * x


def _add_gyro(store, samples, run=True):
    """지금 장착에 더한다. run=True 면 그 판만으로 맞춘 눈금도 판끼리 합계에 — (δ, 표준오차) 또는 None."""
    m = _ensure_mount(store)
    c0 = m["c0"]
    for q1, q2, g in samples:
        _accumulate(m, *_rows(q1, q2, g, c0))
    if run:
        got = _run_scale(samples, c0)
        if got is not None:
            d, var = got
            w = 1.0 / var
            r = store["gyro"]["runs"]
            r["n"] += 1
            r["w"] += w
            r["wd"] += w * d
            r["wdd"] += w * d * d
            return d, math.sqrt(var)
    return None


# ── 주행 시작 때 / --relearn ─────────────────────────────────────────
def update(work_root, path=None, log=print, until=None, site=None):
    """upto 보다 뒤의 기록만 읽어 거른 뒤 더하고 저장한다. 기록마다 넣은 것·뺀 이유는 log 로 (파일에는 숫자만).
    until(날짜_시각)부터는 안 읽는다 — 지금 막 시작한 판의 빈 폴더를 '넣었다' 로 치면 그 판은 영영 안 들어간다.
    기록 하나를 못 읽어도 멈추지 않는다 — 그 기록은 빼고 이유를 찍는다 (--relearn 으로 다시 읽을 수 있다).
    work_root 는 그 장소의 기록 루트(work_dirs/<장소>), path 는 그 장소의 학습 파일 — site 로 같이 정한다."""
    site = site or SITE
    path = path or path_for(site)
    store = load(path, log)
    added = []
    for d in sessions(work_root):
        key = session_key(d)
        if store["upto"] and key <= store["upto"]:
            continue
        if until and key >= until:
            break
        kind = d.parent.name
        try:
            if kind == "calibrate":
                c0, ss, why = calibrate_samples(d)
                if c0 is not None:
                    _close_mount(store)
                    _ensure_mount(store, since=key, c0=c0)
                    _add_gyro(store, ss, run=False)
                    added.append((session_name(d), "회전중심 단계 → 카메라 장착을 새로 시작 (스윙 정지점 쌍 %d개)" % len(ss)))
                else:
                    added.append((session_name(d), "자이로·회전중심 자료 없음 (%s)" % why))
            elif kind == "run":
                r = run_samples(d)
                if r["why"]:
                    added.append((session_name(d), "뺌 — " + r["why"]))
                else:
                    for x in r["inertia"]:
                        _add_inertia(store, x)
                    sc = _add_gyro(store, r["gyro"]) if r["gyro"] else None
                    for name, xs in r["straight"].items():
                        for x in xs:
                            _add_straight(store, name, x)
                    for name, xs in r["minstep"].items():
                        for x in xs:
                            _add_min(store, name, x)

                    def skipped(k):
                        return (" (뺌 " + ", ".join("%s %d" % kv for kv in r[k].items()) + ")") if r[k] else ""
                    added.append((session_name(d), "회전 응답 %d번%s · 자이로 %d번%s%s · 직진 %s%s · 최소걸음 %s%s" % (
                        len(r["inertia"]), skipped("skip_inertia"), len(r["gyro"]), skipped("skip_gyro"),
                        (" · 이 판 눈금 %.4f ±%.4f" % (1 + sc[0], sc[1])) if sc else "",
                        " · ".join("%s %d번" % (("전진" if name == "forward" else "후진"), len(xs)) for name, xs in r["straight"].items()),
                        skipped("skip_straight"),
                        " · ".join("%s %d번" % (("전진" if name == "forward" else "후진"), len(xs)) for name, xs in r["minstep"].items()),
                        skipped("skip_minstep"))))
        except Exception as e:
            added.append((session_name(d), "!! 읽기 실패 — 뺌 (%s: %s)" % (type(e).__name__, e)))
        store["upto"] = key
    save(store, path)
    if log:
        log("  학습 파일 %s (장소 %s) — 새 기록 %d개 (여기까지 넣음: %s)" % (Path(path).name, site, len(added), store["upto"]))
        for name, what in added:
            log("    %s %s" % (name, what))
        u = use(store, site)
        for name in DIRS:
            m = u["straight"][name]
            log("    직진 %s: 시간 직진 = %s · 끊는 값 = τ %.3f s · 잔여 %+.3f m (%s)%s"
                % ("전진" if name == "forward" else "후진",
                   ("유지 %.2f s + 거리 ÷ %.3f m/s (%s)" % (m["dead_s"], m["speed_mps"], _src(m["timed_from"], m["n_timed"])))
                   if m["speed_mps"] else "-",
                   m["tau_s"], m["residual_m"], _src(m["stop_from"], m["n_stop"]),
                   (" ⚠ 범위 밖이라 자름: %s" % m["clamped"]) if m["clamped"] and m["stop_from"] == "own" else ""))
            log("      최소걸음 %s" % (("%.3f m ±%.3f (%s)" % (m["min_step_m"], m["min_step_sd_m"], _src(m["min_from"], m["n_min"])))
                                     if m["min_step_m"] is not None else "식(정속 × τ — 작게 나온다) — 이 장소 자료 %d번" % m["n_min"]))
        for k in STRENGTHS:
            m = u["inertia"][k]
            log("    회전 강도 %d: %d번%s · τ %.3f s · 잔여 %+.2f° (오차 = %+.2f %+.3f×ω, 기울기 %s)%s · 출발지연 %s · 각속도 %s"
                % (k, m["n"], "" if m["from"] in ("own", "seed") else " (%s 에서 빌림)" % m["from"],
                   m["tau_s"], m["residual_deg"], m["a"], m["c"], "씀" if m["slope_used"] else "안 씀(우연 범위)",
                   (" ⚠ 범위 밖이라 자름: %s" % m["clamped"]) if m["clamped"] else "",
                   ("%.2f s (%d)" % (m["startup_s"], m["startup_n"])) if m["startup_s"] else "-",
                   ("%.1f °/s (%d)" % (m["rate_dps"], m["rate_n"])) if m["rate_dps"] else "-"))
        g = u["gyro"]
        log("    자이로 눈금 %.4f (%s) · 맞춘 값 %s ±%s · 판 %d개%s"
            % (g["scale"], "씀" if g["scale_used"] else "안 씀 — 아직 우연 범위",
               ("%.4f" % g["scale_fit"]) if g["scale_fit"] is not None else "-",
               ("%.4f" % g["scale_se"]) if g["scale_se"] is not None else "-", g["scale_runs"],
               "" if g["scale_from"] == "own" else " (%s 에서 빌림)" % g["scale_from"]))
        if g["cam_to_rot_center_m"] is not None:
            log("    회전중심 (장착 %s 부터 %d번): 앞뒤 %+.3f · 좌우 %+.3f m (±%.3f / ±%.3f)  — config %s / %s"
                % (g["since"] or "config", g["n"], g["cam_to_rot_center_m"], g["rot_center_lateral_m"],
                   g["center_se_m"][0], g["center_se_m"][1], C.CAM_TO_ROT_CENTER_M, C.ROT_CENTER_LATERAL_M))
        else:
            log("    회전중심: 지금 장착 자료 %d번 — 모자라 config 값을 쓴다 (%s / %s)" % (g["n"], C.CAM_TO_ROT_CENTER_M, C.ROT_CENTER_LATERAL_M))
    return store, added


def _src(frm, n):
    """출처 한 마디 — 이 장소 n번 / 다른 장소·전진 값을 빌림 / 씨앗."""
    if frm == "own":
        return "%d번" % n
    if frm == "seed":
        return "씨앗 — 이 장소 자료 %d번" % n
    site, _, fwd = frm.partition("·")
    name = " ".join(x for x in ("" if site == "own" else site, "전진" if fwd else "") if x)
    return "%s 값 — 이 장소 %s자료 %d번 < %d" % (name, "후진 " if fwd else "", n, MIN_N)


def relearn(work_root, since=None, path=None, log=print, site=None):
    """비우고 다시. since 없음 = 지금부터 새로(지금까지의 기록은 안 읽는다) · since = "YYYYMMDD" = 그날부터 다시 읽어 쌓는다."""
    site = site or SITE
    path = path or path_for(site)
    store = empty()
    store["upto"] = since or datetime.now().strftime("%Y%m%d_%H%M%S")     # "20261001" 은 "20261001_xxxxxx" 보다 앞이라 그날 기록이 다 들어온다
    save(store, path)
    return update(work_root, path, log, site=site)


if __name__ == "__main__":
    import argparse
    import random
    import sys
    from ...bootstrap import work_root as _work_root
    ap = argparse.ArgumentParser(description="학습 파일 config/learned/<장소>.json 을 그 장소 기록에서 갱신 (회전 응답 · 직진 · 자이로)")
    ap.add_argument("--relearn", nargs="?", const="", default=None, metavar="YYYYMMDD",
                    help="비우고 다시 쌓는다. 날짜를 주면 그날부터의 기록을 다시 읽는다")
    ap.add_argument("--site", default=SITE, help="장소 (기본 config/site.py SITE = %s)" % SITE)
    ap.add_argument("--out", default=None, help="기록 기준 폴더 (bootstrap.work_root 와 같은 규칙 — 그 밑의 <장소>/ 를 읽는다)")
    ap.add_argument("--path", default=None, help="학습 파일 경로 (기본 config/learned/<장소>.json)")
    ap.add_argument("--selftest", action="store_true")
    a = ap.parse_args()
    if a.selftest:
        # (1) 회전 응답 합계 → 직선: 오차 = 1.0 − 0.1·ω 를 정확히 주면 그대로 나온다. 점 둘이면 평균만. 범위 밖은 자른다
        st = empty()
        assert set(st) == {"upto", "straight", "inertia", "gyro"} and set(st["inertia"]) == {"strength_30", "strength_20"}
        assert set(st["straight"]) == {"forward", "backward"} and DIRS == {"forward": "67", "backward": "187"}
        for w in (4.0, 8.0, 12.0, 16.0):
            _add_inertia(st, {"strength": 30, "w": w, "e": 1.0 - 0.1 * w, "startup": 1.0, "rate": 10.0})
        m = rotation_model(st, 30)
        assert m["slope_used"] and abs(m["c"] + 0.1) < 1e-9 and abs(m["a"] - 1.0) < 1e-9 and m["startup_s"] == 1.0, m
        st2 = empty()
        for w, noise in ((4.0, 0.05), (8.0, -0.05), (12.0, 0.05), (16.0, -0.05), (6.0, 0.0), (14.0, 0.0)):
            _add_inertia(st2, {"strength": 30, "w": w, "e": 1.0 - 0.1 * w + noise})
        m2 = rotation_model(st2, 30)
        assert m2["slope_used"] and abs(m2["c"] + 0.1) < 0.02 and abs(m2["tau_s"] - (REF_TAU[30] - 0.1)) < 0.02, m2
        st3 = empty()
        for w in (4.0, 8.0):
            _add_inertia(st3, {"strength": 30, "w": w, "e": 0.5})
        m3 = rotation_model(st3, 30)
        assert not m3["slope_used"] and m3["residual_deg"] == 0.5 and m3["n"] == 2
        st4 = empty()
        _add_inertia(st4, {"strength": 20, "w": 5.0, "e": 9.0})
        m4 = rotation_model(st4, 20)
        assert m4["residual_deg"] == RES_LIMIT_DEG and "residual" in m4["clamped"]
        assert rotation_model(empty(), 30)["tau_s"] == round(REF_TAU[30], 4) and rotation_model(empty(), 30)["residual_deg"] == 0.0
        # (2) 자이로 — 펼친 식이 정확한 회전과 맞나, 합계에서 눈금·중심이 나오나 (수식 점검이지 정확도 주장이 아니다)
        rng = random.Random(3)

        def turn(q, c, ang):
            R = _rot(ang)
            d = (q[0] - c[0], q[1] - c[1])
            return (R[0][0] * d[0] + R[0][1] * d[1] + c[0], R[1][0] * d[0] + R[1][1] * d[1] + c[1])

        def make(k, c, n, noise=0.0, dist=(3.0, 9.0)):
            out = []
            for _ in range(n):
                rho, b = rng.uniform(*dist), math.radians(rng.uniform(-20, 20))
                q1 = (rho * math.cos(b), -rho * math.sin(b))
                g = math.radians(rng.choice((-1, 1)) * rng.uniform(2, 34))
                q2 = turn(q1, c, k * g)
                out.append((q1, (q2[0] + rng.gauss(0, noise), q2[1] + rng.gauss(0, noise)), g))
            return out
        c_true = (0.42, 0.06)
        st5 = empty()
        st5["gyro"]["mount"] = _mount("t", (0.465, -0.12))
        for q1, q2, g in make(0.99, c_true, 40):
            _accumulate(st5["gyro"]["mount"], *_rows(q1, q2, g, (0.465, -0.12)))
        A_, B_, _ = _reduce(st5["gyro"]["mount"])
        d_hat = B_ / A_
        (cx, cy), _, _ = _center(st5["gyro"]["mount"], d_hat)
        print("펼친 식 점검 (잡음 0): 눈금 %.5f (참 0.99) · 중심 %.4f %+.4f (참 0.42 +0.06)" % (1 + d_hat, cx, cy))
        assert abs(d_hat + 0.01) < 2e-4 and abs(cx - 0.42) < 5e-3 and abs(cy - 0.06) < 5e-3
        # 판끼리 흩어짐이 작으면 판이 MIN_N 이 될 때부터 눈금을 쓴다
        st6 = empty()
        for r in range(4):
            _add_gyro(st6, make(0.99, c_true, 10, noise=0.002))
            gm = gyro_model(st6)
            assert gm["scale_used"] == (r + 1 >= MIN_N), (r, gm)
        print("판 4개 (잡음 2 mm): 눈금 %.4f ±%.4f 씀 · 중심 카메라 축 %s" % (gm["scale"], gm["scale_se"], gm["center_cam_m"]))
        # 판마다 눈금이 크게 다르면(흩어짐) 합친 표준오차가 작아도 안 쓴다
        st7 = empty()
        for k in (0.97, 1.02, 0.985, 1.01):
            _add_gyro(st7, make(k, c_true, 10, noise=0.002))
        gm7 = gyro_model(st7)
        print("판마다 눈금이 0.97~1.02 로 흩어지면: 맞춘 값 %.4f ±%.4f → %s" % (gm7["scale_fit"], gm7["scale_se"], "씀" if gm7["scale_used"] else "안 씀"))
        assert not gm7["scale_used"], gm7
        # 장착을 바꿔도 눈금 몫은 이어지고(closed), 중심은 새 장착 것만
        _close_mount(st6)
        _ensure_mount(st6, since="new", c0=(0.465, -0.12))
        _add_gyro(st6, make(0.99, (0.70, -0.20), 10, noise=0.002))
        gm2 = gyro_model(st6)
        assert gm2["since"] == "new" and abs(gm2["center_cam_m"][0] - 0.70) < 0.02 and st6["gyro"]["closed"]["mounts"] == 1, gm2
        # 차체 축 ↔ 카메라 축 왕복, config 값(카메라 축 −0.463/−0.121, cam_yaw 1.14)이 config 줄(−0.465/−0.112)로
        assert all(abs(x - y) < 1e-12 for x, y in zip(_from_vehicle(*_to_vehicle(0.4, 0.07, 1.14), 1.14), (0.4, 0.07)))
        assert all(abs(x - y) < 1e-3 for x, y in zip(_to_vehicle(0.463, -0.121, 1.14), (-0.465, -0.112)))
        # (3) 직진 — 시간 직진 직선 · 끊는 값. 거리 = 0.29 × (유지 − 1.2) 를 정확히 주면 그대로 나온다
        st8 = empty()
        for h in (3.0, 5.0, 7.0):
            _add_straight(st8, "forward", {"h": h, "d": 0.29 * (h - 1.2), "v": 0.29, "e": 0.01})
        m8 = straight_model(st8, "forward")
        assert abs(m8["speed_mps"] - 0.29) < 1e-9 and abs(m8["dead_s"] - 1.2) < 1e-9, m8
        assert m8["tau_s"] == round(FWD_TAU_REF_S, 4) and m8["residual_m"] == 0.01 and not m8["slope_used"], m8   # 속도가 같으면 평균만
        st9 = empty()
        for v in (0.20, 0.24, 0.28, 0.32, 0.36):
            _add_straight(st9, "forward", {"h": 5.0, "d": 1.0, "v": v, "e": 0.02 - 0.1 * v})
        m9 = straight_model(st9, "forward")
        assert m9["slope_used"] and abs(m9["c"] + 0.1) < 1e-9 and abs(m9["tau_s"] - (FWD_TAU_REF_S - 0.1)) < 1e-4, m9
        assert m9["speed_mps"] is None                                   # 유지 시간이 다 같으면 직선을 못 맞춘다 → 다른 데서
        e0 = straight_model(empty(), "forward")
        assert e0["tau_s"] == round(FWD_TAU_REF_S, 4) and e0["residual_m"] == 0.0 and e0["speed_mps"] is None
        # (4) 빌려 쓰기 — 이 장소 자료가 MIN_N 번 미만인 칸만 다른 장소에서. 후진은 전진 값. 회전중심은 안 빌린다
        u0 = use(empty(), site="krri", others=[("kw", st8)])
        assert u0["straight"]["forward"]["timed_from"] == "kw" and abs(u0["straight"]["forward"]["speed_mps"] - 0.29) < 1e-9
        assert u0["straight"]["forward"]["stop_from"] == "kw" and u0["straight"]["forward"]["residual_m"] == 0.01
        assert u0["straight"]["backward"]["timed_from"] == "kw·전진" and u0["straight"]["backward"]["stop_from"] == "kw·전진"
        u1 = use(st8, site="kw", others=[])
        assert u1["straight"]["backward"]["timed_from"] == "own·전진" and u1["straight"]["forward"]["timed_from"] == "own"
        assert use(empty(), site="krri", others=[])["straight"]["forward"]["timed_from"] == "seed"
        assert use(empty(), site="krri", others=[])["straight"]["forward"]["speed_mps"] == LN.KWU_SEED["fwd_speed_mps"]["67"]
        st10 = empty()
        _add_straight(st10, "forward", {"h": 4.0, "d": 0.8, "v": 0.3, "e": 0.05})            # 이 장소 1번 — 모자라 빌린다
        u2 = use(st10, site="krri", others=[("kw", st8)])
        assert u2["straight"]["forward"]["stop_from"] == "kw" and u2["straight"]["forward"]["n_stop"] == 1
        u3 = use(empty(), site="krri", others=[("kw", st)])
        assert u3["inertia"][30]["from"] == "kw" and u3["inertia"][30]["n"] == 4 and u3["inertia"][20]["from"] == "seed"
        u4 = use(empty(), site="krri", others=[("kw", st6)])
        assert u4["gyro"]["scale_from"] == "kw" and u4["gyro"]["scale_used"] and u4["gyro"]["cam_to_rot_center_m"] is None
        assert use(st6, site="kw", others=[])["gyro"]["cam_to_rot_center_m"] is not None
        # 옛 기록 역산 — 그때 쓴 τ·잔여를 learned 로 따라가고, 시작 측정 0.0(F2)·시간 직진·거부는 뺀다
        ev = [{"event": "forward_end", "strength": 67, "reason": "predicted", "target_m": 1.0, "travelled_m": 1.05, "d_final": 0.2,
               "speed_mps": 0.3, "blind_s": 0.2, "t_cmd": 0.0, "t_onset": 1.3, "t_stop_cmd": 4.0, "learned": {"tau": 0.4, "residual": 0.05}},
              {"event": "forward_end", "strength": 67, "reason": "settle_timeout", "target_m": 1.0, "travelled_m": 0.98, "d_final": 0.1,
               "speed_mps": 0.3, "blind_s": 0.2, "t_cmd": 10.0, "t_onset": 11.3, "t_stop_cmd": 14.0, "learned": {}},
              {"event": "forward_end", "strength": 67, "reason": "predicted", "target_m": 0.67, "travelled_m": 0.63, "d_final": -0.63,
               "speed_mps": 0.3, "blind_s": 0.2, "t_cmd": 20.0, "t_onset": 21.3, "t_stop_cmd": 24.0, "learned": {}},
              {"event": "forward_end", "strength": 67, "reason": "timed", "timed": True, "target_m": 1.0, "travelled_m": 1.0,
               "d_final": 0.0, "speed_mps": 0.29, "blind_s": 0.0, "t_cmd": 30.0, "t_onset": 0.0, "t_stop_cmd": 35.0},
              {"event": "forward_end", "strength": 67, "reason": "too_small", "target_m": 0.05, "travelled_m": 0.0, "d_final": 0.0,
               "speed_mps": 0.0, "blind_s": 0.0, "t_cmd": 0.0, "t_onset": 0.0, "t_stop_cmd": 0.0}]
        ss, sk = _straight_samples({"seeds": {"fwd_tau_s": {"67": 0.5}}}, ev)
        assert len(ss["forward"]) == 2 and not ss["backward"], ss
        assert abs(ss["forward"][0]["e"] - (0.05 + 0.3 * (0.5 - FWD_TAU_REF_S) + 0.0)) < 1e-12        # 첫 직진: 씨앗 τ 0.5 · 잔여 0
        assert abs(ss["forward"][1]["e"] - (-0.02 + 0.3 * (0.4 - FWD_TAU_REF_S) + 0.05)) < 1e-12     # 둘째: 첫 직진이 배운 값
        assert sk == {"시작 측정을 못 믿음": 1, "시간 직진": 1, "끝이 정상이 아님 (too_small)": 1}, sk
        new = [dict(ev[0], learned={}, tau_used=0.3, residual_used=-0.01, start_ok=True, start_remaining_m=1.25)]
        ss2, _ = _straight_samples({"seeds": {"fwd_tau_s": {"67": 0.5}}}, new)
        assert abs(ss2["forward"][0]["e"] - (0.05 + 0.3 * (0.3 - FWD_TAU_REF_S) - 0.01)) < 1e-12     # 새 기록은 적힌 값
        # (5) 최소걸음 (F1-3) — 표본 = 출발 인정 때까지 간 거리 + 그때 속도 × τ_기준. 합성 프레임: 1.2 초부터 0.29 m/s 로 곧게
        def dist_at(t):
            return 5.0 - 0.29 * max(0.0, t - 1.2)
        frs = _Frames([{"t": i / 30.0, "distance": dist_at(i / 30.0), "gyro_deg": 0.0} for i in range(-30, 181)])
        base = {"event": "forward_end", "strength": 67, "reason": "predicted", "target_m": 1.0, "travelled_m": 1.0, "d_final": 0.0,
                "speed_mps": 0.29, "blind_s": 0.1, "t_cmd": 0.0, "t_onset": 1.3, "t_stop_cmd": 4.0, "t_settled": 5.0, "detect_age_s": 0.0}
        ms, mk = _min_step_samples([dict(base, t_onset_ok=2.0), dict(base, cut_at_onset=True), base,
                                    dict(base, t_onset_ok=4.5), dict(base, reason="watchdog")], frs)
        assert abs(ms["forward"][0] - (0.29 * 0.8 + 0.29 * FWD_TAU_REF_S)) < 1e-9, ms        # 적힌 순간 — 곧은 구간이라 정확히
        assert abs(ms["forward"][1] - 0.29 * 3.8) < 1e-9, ms                                  # 인정하자마자 끊김 — 실제로 간 거리
        t_rep = _onset_replay(base, frs, 1.0, 0.29)
        assert 1.2 < t_rep < 1.2 + WINDOW_S and len(ms["forward"]) == 3, (t_rep, ms)          # 옛 기록 — 규칙을 다시 돌림
        assert mk == {"출발 인정 전에 끊김": 1, "끝이 정상이 아님 (watchdog)": 1}, mk
        _, sk2 = _straight_samples({}, [dict(base, cut_at_onset=True, start_ok=True, start_remaining_m=1.0)])
        assert sk2 == {"출발 인정하자마자 끊김 (최소걸음 재료)": 1}, sk2                        # τ·잔여엔 안 넣는다
        st11 = empty()
        for x in (0.14, 0.16, 0.18):
            _add_min(st11, "forward", x)
        m11 = straight_model(st11, "forward")
        assert abs(m11["min_step_m"] - 0.16) < 1e-9 and abs(m11["min_step_sd_m"] - 0.02) < 1e-9 and m11["n_min"] == 3, m11
        assert straight_model(st8, "forward")["min_step_m"] is None                         # 자료 없으면 None → 식
        u5 = use(empty(), site="krri", others=[("kw", st11)])
        assert u5["straight"]["forward"]["min_from"] == "kw" and u5["straight"]["backward"]["min_from"] == "kw·전진"
        assert use(empty(), site="krri", others=[])["straight"]["forward"]["min_from"] == "formula"
        # 최소걸음 칸이 없는 옛 파일은 처음부터 다시 읽게 빈 것
        tmp = path_for("selftest")
        tmp.write_text(json.dumps({"upto": "x", "inertia": {}, "straight": {d: {"sums": {"timed": {}, "stop": {}}} for d in DIRS}}),
                       encoding="utf-8")
        assert load(tmp)["upto"] is None
        tmp.unlink()
        # 저장 → 읽기 왕복 (직진 합계 포함)
        st6["straight"] = st8["straight"]
        tmp = path_for("selftest")
        save(st6, tmp)
        back = load(tmp)
        assert back["gyro"]["mount"]["n"] == st6["gyro"]["mount"]["n"] and back["gyro"]["closed"] == st6["gyro"]["closed"]
        assert gyro_model(back)["center_cam_m"] == gm2["center_cam_m"]
        assert back["straight"] == st8["straight"] and straight_model(back, "forward") == m8
        tmp.unlink()
        print("learnfile 자체 시험 통과")
        sys.exit(0)
    root = _work_root(a.out, site=a.site)
    if a.relearn is not None:
        relearn(root, since=a.relearn or None, path=a.path, site=a.site)
    else:
        update(root, path=a.path, site=a.site)
