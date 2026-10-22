"""다음 걸음 하나를 고른다 — 몇 걸음 앞을 그려보고 **첫 걸음만** (plan 3-2, 2026-09-30 결정 ④⑤⑥⑦).

    ① 못 믿는 측정(fix.ok=False)   → uncertain   더 보기 / 곧장 조금 전진 중 택일 (plan 3-6 ①②)
    ② 첫 판단이고 통로 밖          → sidestep    한 번만. 그 뒤엔 통로 밖이어도 ⑤ (결정 ⑤)
    ③ 태그컷 도착 · 진입 여유 > 0   → commit      눈 감고 직진 (limits.commit_margin_m)
    ④ 태그컷 도착 · 진입 안 됨      → step(방향 회전) → backup(딱 한 번, 결정 ⑥) → stop
                                   고치는 동작 합계 ≤ MAX_CORRECTIONS, 진입 후 FINE_TIME_LIMIT_S (먼저 걸리는 쪽)
    ⑤ 아니면 빔서치                 → step        후보 (회전, 직진) · LOOKAHEAD_STEPS 깊이 · BEAM_WIDTH
                                   지평 안에 못 닿으면 남은 걸음을 제일 줄이는 걸음(통로 안일 때만) ·
                                   경로가 없으면 stop (plan 3-10 ⑥) · 후보 자체가 없으면 uncertain (더 본다)

빔서치 — "체스에서 몇 수 앞을 읽고 한 수만 두는 것" (plan 3-2):
    회전 후보  0 · 조준(태그컷에서 정면 보면 축에 서는 방향) · 정면 · ±상한×(1·½·¼). 하한·kσ_방향 아래는 뺀다.
              돌 이유가 잡음 안이면(조준각·방향 둘 다 kσ 안) 회전 후보를 안 낸다 — 유령을 안 쫓는다 (worth_fixing, plan 3-6)
              하한 간격의 고운 격자는 안 쓴다 — 빔(8)이 비슷한 후보로 차서 정작 닿는 가지를 버렸다
    직진 후보  걸음 상한 × (1 · ½ · ¼) + 태그컷까지 + 축에 닿는 거리. 최소걸음(속도×관성) 아래는 forward 가 거부한다
              걸음 상한: 마지막 다리(태그컷에 닿는 것)는 좌우 예산 SIDE_GAP ÷ sin σ_방향, 중간 다리는 통로 반폭 ÷ sin σ_방향 (plan 3-3 ①)
    굴리기    제자리 회전 = 회전중심은 제자리, 카메라가 호를 그린다 (plan 4-6). 직진 = 좌우 += d·sin h, 거리 −= d·cos h
    제약      돌고 나서도 · 가고 나서도 태그가 화면 안 (β 와 태그컷). 직선·회전 모두 β 가 단조라 끝점만 보면 된다
    목표      태그컷에서 진입 여유 > 0. σ 는 그 거리로 옮긴 값(방향 ∝ 거리, 좌우 ∝ 거리², plan 3-6).
              걸음 수 최소, 동률이면 좌우를 **일찍** 지우는 쪽 (Σ|예측 도착 좌우| 최소)
    σ 때문에만 안 되면  σ 없이 기하만으로 닿는 경로의 첫 걸음을 간다 — 가까이 가면 σ 가 준다 (plan 3-6 ①)

회전중심 좌표 — 제자리 회전은 회전중심을 안 움직이므로 상태를 (중심 좌우 Lc, 중심 거리 Fc, 방향 h) 로 들고
카메라는 거기서 계산한다. 부호 실수가 한 곳(_camera)에 모인다:
    카메라 좌우 = Lc + A·sin h − b·cos h      카메라 거리 = Fc − A·cos h − b·sin h
    A = config CAM_TO_ROT_CENTER_M (음수 = 카메라 앞) · b = ROT_CENTER_LATERAL_M (+ 왼쪽) — analyze_calibrate._ls_center 와 같은 틀
    h=0 에서 +Δ 돌면 카메라 좌우가 A·sin Δ 만큼 는다: 중심이 앞(A<0)이면 왼쪽으로 돌 때 카메라는 **오른쪽**으로 (plan 4-6)
    그때 태그의 화면 위치는 Δ·(d+A)/d 만큼 움직인다 — limits.turn_cap_deg 의 배율과 같은 식 (자체 시험이 확인)

부호: 좌우 + 왼쪽, 방향 + 반시계, β = 방향 + atan2(좌우, 거리) (+ 는 화면 오른쪽). pose.state · sidestep.py 와 같다.
계산은 늘 한다 (정지·직진 중 둘 다). 회전 실행은 부르는 쪽이 **멈춘 뒤에만** 건다 (결정 ⑦).
"""
if __package__ in (None, ""):
    # python src/models/planning/plan.py 로 직접 돌릴 때(자체 시험). 평소엔 패키지로 import 된다
    import sys as _sys
    from pathlib import Path as _Path
    _sys.path.insert(0, str(next(d for d in _Path(__file__).resolve().parents
                                 if (d / "config").is_dir() and (d / "src").is_dir())))
    __package__ = "src.models.planning"

import math
from dataclasses import dataclass, replace

from config import control as C
from config import detection as D
from ... import limits
from ...utils import clock
from ..control import sidestep
from ..control.forward import min_step_m, strength_of
from ..control.rotate import min_turn_deg
from .aim import turn_for
from ..detection.estimate import worth_fixing

RELAXED_FACTOR = 1.5               # plan 3-6 ③ "기준을 낮춘다 2배 → 1.5배". Planner.relax() 가 켠다
FRACTIONS = (1.0, 0.5, 0.25)        # 상한의 이 배들이 후보 — 직진(걸음 상한)·회전(회전 상한) 둘 다.
                                    # 광운대 fsm_v4/config.py L375 MULTISTEP_FORWARD_FRACTIONS
BUCKET = (0.025, 0.10, 2.0)         # 빔에서 같은 자리로 보는 칸 [m, m, 도]. 광운대 fsm_v4/route_planner.py L117-121
AIM_ITERS = 3                       # 조준각 고정점 반복. 돌면 카메라 거리가 A(1−cos) 만큼 변해 다시 맞춘다 — 3번이면 mm 아래
STOP_REASON = "no_converge"         # 계획기가 세우는 이유는 전부 "수렴 못 함" (record.STOP_REASONS)


# ── 결과 ─────────────────────────────────────────────────────────────
@dataclass
class Decision:
    """다음에 할 것 하나. hud 의 plan 칸이 kind · turn_deg · drive_m · why 를 그대로 읽는다."""
    kind: str                           # step / sidestep / backup / commit / uncertain / stop
    why: str = ""
    turn_deg: float = 0.0               # step: 먼저 이만큼 돌고 (+ 반시계)
    drive_m: float = 0.0                # step: 그다음 직진 · commit: 눈 감고 갈 거리 · backup: 물러날 거리
                                        # uncertain: 좌우·방향 없이도 곧장 가도 되는 거리 (0 이면 더 본다)
    movement: str = "forward"           # forward / backward (결정 ②: 직진은 byte 67)
    fwd_target_m: float | None = None   # 동작이 끝났을 때 태그면까지 거리(예상). look_fn 의 남은거리 = fix.forward_m − 이것
    stop_reason: str = ""               # stop 일 때 record.STOP_REASONS 중 하나
    correction: bool = False            # 마지막 단계의 고치는 동작인가 (횟수 상한 MAX_CORRECTIONS 에 센다)
    at_cut: bool = False                # 계획이 겨누는 선(정렬선, 마지막 구간에선 태그컷)에 와 있나
    to_ref: str = ""                    # 이 직진이 "line"(정렬선) / "cut"(태그컷) 에서 끝나는 다리인가 — run.py 가 그 선을 프레임마다 다시 재서 목표를 따라 옮긴다
    ref_m: float = 0.0                  # 결정 때 그 선의 거리
    cut_m: float = 0.0                  # 결정 때의 태그컷 (실시간이면 그 순간 값)
    line_m: float = 0.0                 # 결정 때의 정렬선 = 태그컷 + FINAL_STRAIGHT_M (마지막 구간을 안 쓰면 태그컷과 같다)
    final: bool = False                 # 마지막 직진 구간의 판단인가 (PnP 방향·좌우 대신 조준 빗나감으로)
    fine: bool = False                  # 이 걸음의 회전을 약한 강도로 (rotate(fine=True))
    miss_m: float | None = None         # 조준 빗나감 — 차 맨 앞 가운데가 목표에서 벗어날 양 (+ 목표가 왼쪽). aim.Aim.miss
    miss_sigma_m: float | None = None
    cut_from: str = ""                  # live(자세의 높이 + 윗변 행) / fallback(sigma_still 높이차 또는 config)
    corridor_half_m: float = 0.0
    inside: bool = True
    margin_m: float | None = None       # 진입 여유 (태그컷에서만 뜻이 있다)
    predicted_m: float | None = None    # 이대로 곧장 태그컷까지 가서 정면을 보면 서게 될 좌우 (진단)
    route: tuple = ()                   # 빔서치가 그린 경로 ((turn, drive), ...). 실행은 첫 걸음만
    waypoints: tuple = ()               # route 의 걸음마다 **카메라가 서게 될 자리** ((좌우, 앞, 방향), ...) — 겨냥하는 법선 기준.
                                        # 기록·화면용. 다음 판단의 fix 와 견주면 "계획한 점 vs 실제로 간 점" 이 된다
    complete: bool = False              # route 가 σ 까지 넣고 진입 조건에 닿았나 (False 면 기하만)
    sidestep: object = None             # SidestepPlan (kind == sidestep)
    backup: object = None               # BackupPlan (kind == backup)

    def summary(self):
        s = self.kind.upper()
        if self.kind in ("step", "backup", "sidestep"):
            s += " 돌기%s %+.2f도 · %s %.2f m" % ("(약)" if self.fine else "", self.turn_deg, self.movement, self.drive_m)
        elif self.kind == "commit":
            s += " 눈 감고 %.2f m" % self.drive_m
        elif self.kind == "uncertain" and self.drive_m > 0:
            s += " 곧장 %.2f m" % self.drive_m
        if self.route and len(self.route) > 1:
            s += " (%d걸음%s)" % (len(self.route), "" if self.complete else "·기하만")
        return s + (" — " + self.why if self.why else "")


@dataclass
class Progress:
    """부르는 쪽(run.py)이 들고 있는 이력. Planner.executed() 가 채운다."""
    steps: int = 0                      # 실행한 step 수. 사이드스텝은 "첫 판단" 에만 — 그 전에 걸음이 없었어야 한다
    sidestepped: bool = False
    backed_up: bool = False
    corrections: int = 0                # 마지막 단계 진입 뒤 실행한 고치는 동작 (step · backup)
    fine_since: float | None = None     # 처음 태그컷에 닿은 시각 (clock.now). 여기서 FINE_TIME_LIMIT_S 가 돈다
    k_sigma: float = C.UNCERTAIN_FACTOR  # "모르겠다" 문턱 배수. relax() 가 RELAXED_FACTOR 로


@dataclass
class _Node:
    Lc: float                           # 회전중심 좌우 (+ 왼쪽)
    Fc: float                           # 회전중심 → 태그면 거리
    h: float                            # 방향 [도]
    path: tuple = ()
    cost: float = 0.0                   # Σ|예측 도착 좌우| — 좌우를 일찍 지울수록 작다
    last_turn: float = 0.0


# ── 기하 한 벌 ────────────────────────────────────────────────────────
class _Geo:
    """측정값에서 계획기가 쓰는 기하만 뽑는다. 없으면 config 폴백 (회전중심은 폴백 없음 → cap 5도)."""

    def __init__(self, learner, intr=None, A=None, b=None, rms_mm=None, height_diff_m=None, final_m=0.0):
        """intr = 카메라에서 읽은 intrinsics(없으면 D435I_COLOR_REF 폴백). A·b·rms_mm = config 의 calibrate 실측
        (CAM_TO_ROT_CENTER_M · ROT_CENTER_LATERAL_M · ROT_CENTER_RMS_MM). A 가 None 이면 회전중심 미확정 → cap 5도."""
        self.intr = intr or None                              # {} 도 폴백(D435I_COLOR_REF) — limits._intr
        # 높이차·태그컷 **폴백** — 없으면 config 명목값. 실행 중엔 see() 가 프레임마다 다시 잰다
        # (limits.tag_cut_live_m: 자세의 vertical + 윗변 행 → 경사·요철·피치 포함, 2026-10-01)
        self.dz0 = float(height_diff_m) if height_diff_m is not None else (D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M)
        self.tagcut0 = limits.tag_cut_m(self.intr, height_diff_m=self.dz0)
        # final_m > 0 이면 **정렬선** = 태그컷 + final_m 을 접근 계획의 목표선으로 쓴다 (2026-10-02 결정): 좌우 지우기·정면
        # 맞추기는 그 선까지 끝내고, 그 안쪽 final_m 은 _final() 이 조준으로만 다룬다. cut 은 "접근 계획이 겨누는 선" 이다
        self.final_m = float(final_m)
        self.dz, self.tagcut, self.cut_from = self.dz0, self.tagcut0, "fallback"
        self.cut = self.tagcut + self.final_m
        self.center_known = A is not None
        self.A = float(A) if A is not None else 0.0
        self.b = float(b) if b is not None else 0.0
        self.scatter_m = float(rms_mm) / 1000.0 if rms_mm else 0.0   # 회전중심 흩어짐 → 마지막 회전이 남기는 좌우 σ (plan 4-7)
        self.fork_tip = C.CAM_TO_FORK_TIP_M                   # 차 치수 — config 만 (2026-10-01)
        self.back_space = C.BACK_MAX_M                        # 현장 값 — config 만
        self.floor = min_turn_deg(learner) or 0.0             # 회전 하한. 모르면 0 (제한 없음)
        self.learner = learner                                # 약한 회전의 하한은 주행 중 올라갈 수 있어 그때그때 읽는다
        self.min_step = min_step_m(learner, strength_of("forward")) or 0.0
        # "태그컷 도착" = 더 갈 거리가 앞뒤 허용 안이거나 최소걸음(forward 가 too_small 로 거부) 아래
        self.arrive_tol = max(C.FWD_TOL_M, self.min_step)

    def see(self, fix):
        """이 fix 의 높이(vertical)·윗변 행으로 태그컷을 다시 잰다. 둘이 없으면(few·no_tag) **마지막 값**을 쓴다 —
        폴백으로 되돌리지 않는다. 눈 감은 뒤 경사가 바뀌는 건 못 본다."""
        v, top = getattr(fix, "vertical_m", float("nan")), getattr(fix, "top_px", float("nan"))
        fwd = float(getattr(fix, "forward_m", 0.0))
        if math.isfinite(v) and math.isfinite(top) and fwd > 0:
            self.dz = -v
            self.tagcut = limits.tag_cut_live_m(self.intr, self.dz, fwd, top)
            self.cut = self.tagcut + self.final_m
            self.cut_from = "live"
        return self.cut

    def center(self, lat, fwd, h):
        r = math.radians(h)
        return (lat - self.A * math.sin(r) + self.b * math.cos(r),
                fwd + self.A * math.cos(r) + self.b * math.sin(r))

    def camera(self, Lc, Fc, h):
        r = math.radians(h)
        return (Lc + self.A * math.sin(r) - self.b * math.cos(r),
                Fc - self.A * math.cos(r) - self.b * math.sin(r))

    def visible(self, lat, fwd, h):
        """이 자리에서 태그가 화면에 남나 — 태그컷(위로 잘림)과 경로각 상한(옆으로 잘림)."""
        if fwd < self.tagcut - self.arrive_tol:
            return False
        beta = h + math.degrees(math.atan2(lat, fwd))
        return abs(beta) <= limits.path_angle_max_deg(fwd, self.intr)

    def blind(self, fwd):
        """눈 감고 갈 거리. 포크 끝이 태그면 앞 STANDOFF_M 에서 서게 — 주행실험은 탑재부가 없어
        STANDOFF 0 이면 포크 끝이 벽에 닿는다 (2026-10-01 확인). 실증 땐 config 를 0 으로."""
        """여기서 눈 감으면 포크 끝이 입구에 닿을 때까지 갈 거리. 태그컷 3.30 이면 BLIND_M 1.78 과 같다."""
        return max(0.0, fwd - self.fork_tip - C.STANDOFF_M)

    def predicted(self, Lc, Fc, h, lever=None):
        """이대로 곧장 태그컷까지 가서 정면(h=0)을 보면 카메라가 서는 좌우. 회전중심이 축에 와야 한다."""
        if lever is None:
            lever = self.camera(Lc, Fc, h)[1] - self.cut
        return Lc + max(0.0, lever) * math.tan(math.radians(h)) - self.b


# ── 계획기 ────────────────────────────────────────────────────────────
class Planner:
    """한 번의 도킹 동안 하나. decide() 는 순수 계산이고, 이력은 executed() 로만 바뀐다 (fine_since 만 decide 가 찍는다)."""

    def __init__(self, learner, progress=None, **geo):
        """geo = _Geo 의 인자 (intr · A · b · rms_mm · height_diff_m). run.py 가 config 와 카메라에서 채운다."""
        self.learner = learner
        self.geo = _Geo(learner, **geo)
        self.progress = progress or Progress()

    # ── 공개 ──
    def decide(self, fix, now=None, aim=None):
        """aim = planning.aim.Aim (run.py 가 직진 다리마다 채운다). 마지막 구간(final_m > 0)의 판단에 쓴다."""
        now = clock.now() if now is None else now
        geo, p = self.geo, self.progress
        geo.see(fix)                                            # 태그컷·높이차를 이 프레임으로 (있을 때만)
        if not fix.ok:
            adv = self._advance_m(fix) if fix.why == "few" else 0.0
            return Decision("uncertain", drive_m=adv, fwd_target_m=None, cut_m=geo.tagcut, line_m=geo.cut,
                            cut_from=geo.cut_from,
                            why=fix.why + (": 곧장 %.2f m 더 가본다" % adv if adv > 0 else ": 더 본다"))

        # 겨냥하는 법선 = 탑재부 중앙. 태그가 중앙에서 비켜 붙었으면 그만큼 옮긴 축을 0 으로 본다 (config TAG_LATERAL_OFFSET_M)
        lat, fwd, h = float(fix.lateral_m) - C.TAG_LATERAL_OFFSET_M, float(fix.forward_m), float(fix.heading_deg)
        k = p.k_sigma
        d = Decision("uncertain", cut_m=geo.tagcut, line_m=geo.cut, cut_from=geo.cut_from)
        d.corridor_half_m = limits.corridor_half_m(fwd, geo.intr, geo.cut)
        d.inside = abs(lat) <= d.corridor_half_m
        Lc, Fc = geo.center(lat, fwd, h)
        d.predicted_m = geo.predicted(Lc, Fc, h)
        room = fwd - geo.cut
        d.at_cut = room <= geo.arrive_tol
        if d.at_cut and geo.final_m > 0:
            return self._final(fix, d, now, aim)                # 정렬선 안쪽 — 마지막 직진 구간. 아래 분기는 안 탄다

        # ② 사이드스텝 — 첫 판단(걸음 0)이고 통로 밖일 때 한 번. 태그컷에선 통로가 폭 0 이라 누구나 "밖" 이지만
        #    비켜선 뒤 대각선을 그릴 거리가 없다 — 그건 ④(후진)의 일이다
        if not d.inside and not d.at_cut and not p.sidestepped and p.steps == 0:
            sp = sidestep.plan(replace(fix, lateral_m=lat), self.learner, geo.back_space)   # 같은 축 기준
            if sp.ok:
                return replace(d, kind="sidestep", sidestep=sp, turn_deg=sp.turn1_deg,
                               drive_m=sp.drive_m, movement=sp.movement,
                               waypoints=self._waypoints(Lc, Fc, h, ((sp.turn1_deg, sp.drive_m), (sp.turn2_deg, 0.0))),
                               why="통로 밖 |%.2f| > %.2f m" % (lat, d.corridor_half_m))
            # 등지고 있고 뒤로도 못 간다 — 빔서치가 여러 걸음으로 풀어 보고, 안 되면 stop

        # ③ 태그컷 — 진입 판정, 상한
        if d.at_cut:
            if p.fine_since is None:
                p.fine_since = now
            d.margin_m = limits.commit_margin_m(lat, h, fix.lateral_sigma_m, fix.heading_sigma_deg,
                                                geo.blind(fwd))
            # 방향이 아직 고칠 수 있는 크기(하한·kσ 위)면 먼저 고친다 — 좌우로 상쇄해 눈 감는 건 못 고치는 잔여에만 (plan 3-9)
            if d.margin_m > 0 and not _real(h, fix.heading_sigma_deg, geo.floor, k):
                return replace(d, kind="commit", drive_m=geo.blind(fwd), fwd_target_m=geo.fork_tip,
                               why="진입 여유 %+.0f mm (좌우 %+.0f 방향 %+.2f도)" % (d.margin_m * 1e3, lat * 1e3, h))
            if p.corrections >= C.MAX_CORRECTIONS:
                return replace(d, kind="stop", stop_reason=STOP_REASON,
                               why="보정 %d회 다 씀, 여유 %+.0f mm" % (p.corrections, d.margin_m * 1e3))
            if now - p.fine_since >= C.FINE_TIME_LIMIT_S:
                return replace(d, kind="stop", stop_reason=STOP_REASON,
                               why="마지막 단계 %.0f s 넘김, 여유 %+.0f mm" % (now - p.fine_since, d.margin_m * 1e3))

        # ⑤ 빔서치 (태그컷에서는 회전만 남는다 = ④의 방향 회전)
        full, geom, partial, expanded = self._search(lat, fwd, h, fix.lateral_sigma_m, fix.heading_sigma_deg, k)
        best, why = full, ""
        if full is not None:
            why = "진입까지 %d걸음" % len(full.path)
        elif geom is not None and not d.at_cut:
            # σ 가 아직 커서 못 닫았을 뿐 — 가까이 가면 준다. 태그컷에선 안 준다 → 회전만 되풀이하게 된다 → ④
            best, why = geom, "σ 커서 미확정 — 기하 경로 %d걸음" % len(geom.path)
        elif partial is not None and d.inside and not d.at_cut:
            # 지평(LOOKAHEAD_STEPS) 안에 못 닿았을 뿐이다 — 남은 걸음 추정을 제일 줄이는 걸음.
            # 통로 밖이면 안 쓴다: 그건 기하로 못 들어가는 것(plan 3-10 ⑥). 태그컷에서도 안 쓴다: 제자리 회전만으로는
            # 카메라가 호를 그릴 뿐 진전이 아니다 — 거긴 ④(후진). 광운대 fsm_v4/route_planner.py L177-182 fallback
            best, why = partial, "지평 %d걸음 안엔 못 닿음 — 남은 걸음을 줄이는 쪽" % C.LOOKAHEAD_STEPS
        if best is not None:
            turn, drive = best.path[0]
            h1 = h + turn
            lat1, fwd1 = geo.camera(Lc, Fc, h1)
            fwd_after = fwd1 - drive * math.cos(math.radians(h1))
            if not d.inside:
                why += " (통로 밖)"
            return replace(d, kind="step", turn_deg=turn, drive_m=drive, movement="forward",
                           fwd_target_m=fwd_after, ref_m=geo.cut,
                           to_ref=("line" if geo.final_m > 0 else "cut") if fwd_after <= geo.cut + geo.arrive_tol else "",
                           route=best.path, waypoints=self._waypoints(Lc, Fc, h, best.path),
                           complete=full is not None, correction=d.at_cut, why=why)

        # ④ 태그컷 — 방향이 진짜 틀렸으면 정면부터 (plan 3-9 의 첫 보정). 비스듬한 채 잰 좌우는 회전중심 오차를
        #    A·sin h 로 안고 있어(plan 4-7 ±0.07 m) 그걸로 후진량을 정하면 틀린다. 정면에서 다시 재면 그 항이 없다
        if d.at_cut and _real(h, fix.heading_sigma_deg, geo.floor, k):
            cap = limits.turn_cap_deg(geo.center_known, fwd, geo.A if geo.center_known else None, geo.intr)
            turn = max(-cap, min(cap, sidestep.normalize_deg(-h)))
            return replace(d, kind="step", turn_deg=turn, drive_m=0.0, movement="forward",
                           fwd_target_m=geo.camera(Lc, Fc, h + turn)[1], route=((turn, 0.0),),
                           waypoints=self._waypoints(Lc, Fc, h, ((turn, 0.0),)),
                           correction=True, why="정면부터 — 방향 %+.2f도, 여유 %+.0f mm" % (h, d.margin_m * 1e3))
        # ④ 태그컷인데 회전으로는 안 된다 → 후진 한 번 → 그래도 안 되면 stop
        if d.at_cut:
            if p.backed_up:
                return replace(d, kind="stop", stop_reason=STOP_REASON,
                               why="후진은 한 번뿐 — 여유 %+.0f mm" % (d.margin_m * 1e3))
            b, rem = self._backup(fix, Lc, Fc, h)
            if b.ok:
                return replace(d, kind="backup", backup=b, drive_m=b.distance_m, movement="backward",
                               fwd_target_m=fwd + b.distance_m, correction=True,
                               waypoints=self._waypoints(Lc, Fc, h, ((0.0, -b.distance_m),)),
                               why="후진 %.2f m (남은 좌우 %+.0f mm · 필요 %.2f · 최소걸음 %.2f · 뒤공간 %.2f)"
                                   % (b.distance_m, rem * 1e3, b.need_m, b.floor_m, b.space_m))
            if b.why == "uncertain":
                why = ("남은 좌우 %.0f mm 가 %.1fσ(%.0f mm) 안 — 후진해도 소용없다"
                       % (abs(rem) * 1e3, C.UNCERTAIN_FACTOR, C.UNCERTAIN_FACTOR * b.sigma_m * 1e3))
            elif b.why == "no_space":
                why = "후진 필요 %.2f m > 뒤공간 %.2f m" % (max(b.need_m, b.floor_m), b.space_m)
            elif b.why == "inside":
                why = "좌우는 맞는데 진입 조건이 안 된다 (여유 %+.0f mm)" % (d.margin_m * 1e3)
            else:
                why = b.why
            return replace(d, kind="stop", stop_reason=STOP_REASON, backup=b, why=why)

        if expanded:
            return replace(d, kind="stop", stop_reason=STOP_REASON,
                           why="%d걸음 안에 진입 경로 없음 (좌우 %+.2f m, 통로 ±%.2f)"
                               % (C.LOOKAHEAD_STEPS, lat, d.corridor_half_m))
        return replace(d, kind="uncertain", why="고칠 만한 게 잡음 안 — 더 본다")

    def executed(self, decision, ok=True):
        """동작을 마친 뒤 부른다. 실패(ok=False)도 센다 — 시도가 곧 소비다."""
        p = self.progress
        if decision.kind == "step":
            p.steps += 1
        elif decision.kind == "sidestep":
            p.sidestepped = True
        elif decision.kind == "backup":
            p.backed_up = True
        if p.fine_since is not None and decision.kind in ("step", "backup") and (decision.correction or not decision.final):
            p.corrections += 1                                  # 마지막 구간의 '곧장 가기만 하는 걸음' 은 보정이 아니다

    def _final(self, fix, d, now, aim):
        """마지막 직진 구간 (정렬선 ~ 태그컷, FINAL_STRAIGHT_M). **PnP 의 방향·좌우를 안 쓴다** — 조준 빗나감(aim)만 본다:
        차 맨 앞 가운데가 목표에서 얼마나 벗어나게 가고 있나 = 거리 × sin(c − β) + 카메라·목표 오프셋.

        2026-10-01 결정: 구간을 **한 번에** 간다(반씩 끊지 않는다). 고치는 건 서 있는 두 곳 — 정렬선과 태그컷 — 에서만이고,
        기준은 **빗나감이 한쪽 여유(SIDE_GAP_M 3 cm)를 넘을 때**. 돌고 나면 가지 않고 그 자리에서 다시 잰다(회전만 하는 걸음).

            빗나감을 모른다(직진 다리가 아직 없었다)     → 태그컷까지 곧장 가며 c 를 잰다 (태그컷이면 정지·사람)
            |빗나감| > 3 cm · 돌 수 있다                 → 약한 회전만 (보정 횟수에 센다) → 다시 판단
            |빗나감| ≤ 3 cm · 구간 안                    → 태그컷까지 곧장
            |빗나감| ≤ 3 cm · 태그컷                     → 진입 (눈 감고 직진)
            |빗나감| > 3 cm 인데 못 돈다 · 태그컷         → 정지·사람
        '돌 수 있다' = 필요한 회전이 약한 회전의 최소 요청각 이상 (rotate.min_turn_deg). 회전각은 aim.turn_for (회전중심 둘레).
        후진은 여기 없다 — 물러나면 같은 빗나감에 필요한 회전이 더 작아져 오히려 못 고친다.
        """
        geo, p = self.geo, self.progress
        if p.fine_since is None:
            p.fine_since = now
        fwd = float(fix.forward_m)
        room = fwd - geo.tagcut
        at_cut = room <= geo.arrive_tol
        miss, sig = aim.miss(fix, geo.dz) if aim is not None else (None, None)
        d = replace(d, final=True, at_cut=at_cut, miss_m=miss, miss_sigma_m=sig, inside=True)
        drive = min(max(room, 0.0), C.STEP_FORWARD_HARD_MAX_M)   # 태그컷까지 한 번에 (forward 의 한 걸음 상한 안에서)
        if drive < geo.min_step:
            drive = 0.0

        def go(turn, dist, why, correction=False):
            ref = "cut" if (dist > 0 and room - dist <= geo.arrive_tol) else ""
            return replace(d, kind="step", turn_deg=turn, drive_m=dist, movement="forward", fine=True,
                           fwd_target_m=fwd - dist, to_ref=ref, ref_m=geo.tagcut, route=((turn, dist),),
                           correction=correction, why=why)

        def stop(why):
            return replace(d, kind="stop", stop_reason=STOP_REASON, why=why)

        if miss is None:
            if at_cut or drive <= 0:
                return stop("조준각을 모른다 — 구간 안에서 직진 다리를 한 번도 못 봤다")
            if now - p.fine_since >= C.FINE_TIME_LIMIT_S:
                return stop("마지막 구간 %.0f s 넘김" % (now - p.fine_since))
            return go(0.0, drive, "조준각을 재려고 곧장 %.2f m" % drive)
        d.margin_m = C.SIDE_GAP_M - abs(miss) - sig              # 기록용 — σ 까지 뺀 여유. 판단은 |빗나감| 대 3 cm
        tag = "빗나감 %+.0f±%.0f mm" % (miss * 1e3, sig * 1e3)
        if abs(miss) <= C.SIDE_GAP_M:                            # 3 cm 안 — 안 고친다. 상한(횟수·시간)보다 먼저 본다
            if at_cut:
                return replace(d, kind="commit", drive_m=geo.blind(fwd), fwd_target_m=geo.fork_tip,
                               why="진입 — %s" % tag)
            if drive > 0 and now - p.fine_since < C.FINE_TIME_LIMIT_S:
                return go(0.0, drive, "곧장 %.2f m — %s" % (drive, tag))
        if p.corrections >= C.MAX_CORRECTIONS:
            return stop("보정 %d회 다 씀 (%s)" % (p.corrections, tag))
        if now - p.fine_since >= C.FINE_TIME_LIMIT_S:
            return stop("마지막 구간 %.0f s 넘김 (%s)" % (now - p.fine_since, tag))
        need = turn_for(miss, fwd, geo.A)                        # 목표 평면까지 앞거리는 정면 근처라 법선거리와 같다고 본다
        floor = min_turn_deg(self.learner, fine=True) or 0.0
        cap = limits.turn_cap_deg(geo.center_known, fwd, geo.A if geo.center_known else None, geo.intr)
        if abs(miss) > C.SIDE_GAP_M and abs(need) >= floor:
            turn = max(-cap, min(cap, need))                     # 상한에 잘려도 돌기만 하니 다음 판단에서 마저 돈다
            return go(turn, 0.0, "조준 %+.2f도 (돌고 다시 잰다) — %s" % (turn, tag), correction=True)
        return stop("더 못 고친다 — %s, 필요 회전 %+.2f도 < 약한 회전 최소 %.2f도" % (tag, need, floor))

    def _waypoints(self, Lc, Fc, h, path):
        """(회전, 직진) 걸음들을 차례로 밟았을 때 카메라가 서는 자리 ((좌우, 앞, 방향), ...). _expand 와 같은 기하 —
        회전은 회전중심 둘레로, 직진은 그 방향으로(음수면 후진). 좌우는 겨냥하는 법선(TAG_LATERAL_OFFSET 반영) 기준."""
        geo, out = self.geo, []
        for turn, drive in path:
            h = sidestep.normalize_deg(h + turn)
            r = math.radians(h)
            Lc, Fc = Lc + drive * math.sin(r), Fc - drive * math.cos(r)
            lat, fwd = geo.camera(Lc, Fc, h)
            out.append((round(lat, 3), round(fwd, 3), round(h, 2)))
        return tuple(out)

    def relax(self):
        """plan 3-6 ③ — "모르겠다" 가 안 풀리면 문턱을 낮춘다. 한 번만 내려간다."""
        self.progress.k_sigma = min(self.progress.k_sigma, RELAXED_FACTOR)

    def fine_elapsed_s(self, now=None):
        if self.progress.fine_since is None:
            return None
        return (clock.now() if now is None else now) - self.progress.fine_since

    # ── 속: 빔서치 ──
    def _search(self, lat, fwd, h, sig_lat0, sig_h0, k):
        """(σ 까지 넣고 되는 경로, 기하만으로 되는 경로, 못 닿았지만 제일 나아진 노드, 뭔가 그려봤나).
        걸음 수 최소가 목적이라 σ 경로는 찾은 깊이에서 멈춘다 — 더 깊은 경로는 걸음이 더 많다."""
        geo = self.geo
        Lc, Fc = geo.center(lat, fwd, h)
        root = _Node(Lc, Fc, h)
        scale = lambda f: (f / fwd) if fwd > 0 else 1.0            # noqa: E731
        sig = lambda node_fwd: (sig_lat0 * scale(node_fwd) ** 2, sig_h0 * scale(node_fwd))  # noqa: E731
        pri = lambda n: self._priority(n, sig, k)                  # noqa: E731
        root_pri = pri(root)
        frontier, full, geom, partial, expanded = [root], None, None, None, 0
        for _ in range(C.LOOKAHEAD_STEPS):
            children = []
            for node in frontier:
                for child in self._expand(node, sig, k):
                    expanded += 1
                    m_full, m_geom = self._terminal(child, sig, k)
                    # 걸음 수 → 좌우를 일찍 지우는 쪽(여유 한 칸 SIDE_GAP 아래 차이는 잡음) → 진입 여유가 큰 쪽.
                    # 여유를 안 보면 잔여 방향 −0.8도(하한 바로 아래)를 0도보다 고르는 일이 생긴다 — 새는 몫 25 mm
                    cost = round(child.cost / C.SIDE_GAP_M)
                    if m_full is not None and m_full > 0:
                        key = (len(child.path), cost, -m_full)
                        if full is None or key < full[0]:
                            full = (key, child)
                    elif m_geom is not None and m_geom > 0:
                        key = (len(child.path), cost, -m_geom)
                        if geom is None or key < geom[0]:
                            geom = (key, child)
                    else:
                        children.append(child)
                        p = pri(child)
                        if p < root_pri and (partial is None or p < partial[0]):
                            partial = (p, child)
            if full is not None or not children:
                break
            # 빔 유지 — 남은 걸음 추정이 적은 순. 같은 칸은 하나만 (광운대 route_planner _bucket)
            uniq = {}
            for ch in sorted(children, key=pri):
                uniq.setdefault(self._bucket(ch), ch)
            frontier = list(uniq.values())[:C.BEAM_WIDTH]
        return ((full[1] if full else None), (geom[1] if geom else None),
                (partial[1] if partial else None), expanded > 0)

    def _expand(self, node, sig, k):
        geo = self.geo
        lat, fwd = geo.camera(node.Lc, node.Fc, node.h)
        sig_lat, sig_h = sig(fwd)
        for turn in self._turns(node, lat, fwd, sig_lat, sig_h, k):
            h1 = sidestep.normalize_deg(node.h + turn)
            lat1, fwd1 = geo.camera(node.Lc, node.Fc, h1)
            if turn and not geo.visible(lat1, fwd1, h1):
                continue
            for drive in self._drives(node, h1, lat1, fwd1, sig(fwd1)[1]):
                if not turn and not drive:
                    continue                                    # 아무것도 안 하는 걸음은 없다
                r1 = math.radians(h1)
                Lc2, Fc2 = node.Lc + drive * math.sin(r1), node.Fc - drive * math.cos(r1)
                lat2, fwd2 = geo.camera(Lc2, Fc2, h1)
                if drive and not geo.visible(lat2, fwd2, h1):
                    continue
                pred = geo.predicted(Lc2, Fc2, h1, fwd2 - geo.cut)
                yield _Node(Lc2, Fc2, h1, node.path + ((turn, drive),), node.cost + abs(pred), turn)

    def _turns(self, node, lat, fwd, sig_lat, sig_h, k):
        """회전 후보 — 0 · 조준(태그컷에서 정면 보면 축에 서는 방향) · 정면 · ±상한×(1·½·¼). 하한·kσ 아래는 뺀다.
        돌 이유(조준각 또는 방향)가 잡음 안이면 0 뿐이다. 하한 간격의 고운 격자는 안 쓴다 — 빔(8)이 비슷한 후보로
        차서 정작 닿는 가지를 버렸다."""
        geo = self.geo
        aim, sig_aim = self._aim(node, fwd, sig_lat, sig_h)
        if not (_real(aim, sig_aim, geo.floor, k) or _real(node.h, sig_h, geo.floor, k)):
            return (0.0,)
        cap = limits.turn_cap_deg(geo.center_known, fwd, geo.A if geo.center_known else None, geo.intr)
        lo = max(geo.floor, k * sig_h)
        out = {0.0}
        for t in [aim, -node.h] + [s * cap * f for s in (1.0, -1.0) for f in FRACTIONS]:
            t = max(-cap, min(cap, t))
            if abs(t) >= lo and abs(t) > 0:
                out.add(round(t, 4))
        return tuple(sorted(out, key=abs))

    def _aim(self, node, fwd, sig_lat, sig_h):
        """곧장 태그컷까지 가서 정면을 보면 축 위에 서는 방향까지의 회전각과 그 흔들림."""
        geo = self.geo
        lever = fwd - geo.cut
        if lever <= geo.arrive_tol:
            return sidestep.normalize_deg(-node.h), sig_h        # 태그컷: 정면만 남았다
        h1 = node.h
        for _ in range(AIM_ITERS):
            fwd1 = geo.camera(node.Lc, node.Fc, h1)[1]
            h1 = math.degrees(math.atan2(geo.b - node.Lc, max(fwd1 - geo.cut, 1e-6)))
        sig_aim = math.hypot(sig_h, math.degrees(math.atan2(sig_lat, lever)))
        return sidestep.normalize_deg(h1 - node.h), sig_aim

    def _drives(self, node, h1, lat1, fwd1, sig_h1):
        """직진 후보 (내림차순). 태그컷에서는 0 뿐 (회전만)."""
        geo = self.geo
        room = fwd1 - geo.cut
        if room <= geo.arrive_tol:
            return (0.0,)
        r1 = math.radians(h1)
        cosh = max(math.cos(r1), 1e-6)
        d_view = room / cosh                                    # 이 이상 가면 태그가 위로 잘린다
        ref = geo.cut + d_view                                  # 화각 항 = 잰 태그컷 기준 (명목값 섞지 않음)
        d_leg = limits.step_forward_max_m(sig_h1, limits.corridor_half_m(fwd1, geo.intr, geo.cut), ref, d_view,
                                          geo.intr, cut_m=geo.cut)
        d_fin = limits.step_forward_max_m(sig_h1, C.SIDE_GAP_M, ref, d_view, geo.intr, cut_m=geo.cut)
        cands = {d_leg * f for f in FRACTIONS}
        cands.add(d_view)
        if abs(math.sin(r1)) > 1e-9:
            hit = (geo.b - node.Lc) / math.sin(r1)              # 정면 보면 축에 서는 자리까지
            if 0 < hit <= d_leg:
                cands.add(hit)
        out = []
        for d in cands:
            if d <= 0 or d > d_view + 1e-9 or d < geo.min_step:
                continue
            final = fwd1 - d * cosh <= geo.cut + geo.arrive_tol
            if final and d > d_fin + 1e-9:
                continue                                        # 마지막 다리는 좌우 예산이 SIDE_GAP 뿐
            out.append(min(d, d_view))
        return tuple(sorted(set(round(d, 4) for d in out), reverse=True))

    def _terminal(self, node, sig, k):
        """태그컷에 서 있고 방향은 더 못 고칠 만큼 작을 때(하한·kσ 아래) 진입 여유 (σ 포함, 기하만) [m]. 아니면 None 둘.
        방향 조건이 없으면 "8도 비뚤게 서서 눈 감고 새는 몫으로 상쇄" 같은 경로가 통과한다 — 비켜서기(plan 3-9)는
        못 고치는 잔여에만 쓰는 것이지 전략이 아니다 (plan 1절: 둘 다 0)."""
        geo = self.geo
        lat, fwd = geo.camera(node.Lc, node.Fc, node.h)
        if fwd - geo.cut > geo.arrive_tol:
            return None, None
        sig_lat, sig_h = sig(fwd)
        if _real(node.h, sig_h, geo.floor, k):
            return None, None
        # 마지막 회전은 다시 재서 고칠 기회가 없다 — 회전중심 흩어짐이 남긴 좌우 σ 를 얹는다 (plan 4-5 표 마지막 줄)
        sig_lat = math.hypot(sig_lat, geo.scatter_m * abs(math.sin(math.radians(node.last_turn))))
        blind = geo.blind(fwd)
        return (limits.commit_margin_m(lat, node.h, sig_lat, sig_h, blind),
                limits.commit_margin_m(lat, node.h, 0.0, 0.0, blind))

    def _priority(self, node, sig, k):
        """빔에 남길 순서 — 남은 걸음 수의 낙관적 추정이 먼저, 그다음 예측 도착 좌우, 그다음 가까운 순.
        추정 = max(거리로 필요한 다리, 좌우로 필요한 다리) + 아직 안 한 회전(돌아서기 · 정면).
        예측 좌우만 보면 태그컷에 선 노드가 "아직 먼데 선 위에 있는" 노드에 밀리고, 다리 수만 보면
        곧장 가는 노드(회전 0)가 대각선 노드를 밀어낸다 — 둘 다 겪었다."""
        geo = self.geo
        lat, fwd = geo.camera(node.Lc, node.Fc, node.h)
        room = fwd - geo.cut
        pred = geo.predicted(node.Lc, node.Fc, node.h, room)
        sig_lat, sig_h = sig(fwd)
        lat_real = _real(pred, sig_lat, C.SIDE_GAP_M, k)
        head_real = _real(node.h, sig_h, geo.floor, k)
        # 정수로 세면 태그컷에서 −33도와 −15도가 같은 "2회전" 이 되고, 예측 좌우가 다 0 이라 제자리 스핀 후보가
        # 빔을 채운다 — 그래서 걸음을 실수로 센다 (다리 = 거리/상한, 회전 = 각/상한)
        legs_dist = 0.0 if room <= geo.arrive_tol else room / C.STEP_FORWARD_HARD_MAX_M
        erase = C.STEP_FORWARD_HARD_MAX_M * math.sin(math.radians(limits.path_angle_max_deg(geo.cut, geo.intr)))
        legs_lat = abs(pred) / erase if lat_real else 0.0                  # 한 다리가 지울 수 있는 최대 좌우로 나눈 것
        cap = limits.turn_cap_deg(geo.center_known, fwd, geo.A if geo.center_known else None, geo.intr)
        if lat_real:
            # 지금 방향으론 축에 안 닿는다 — 돌아서기 한 번 + 조준 방향에서 다시 정면 보기 (상한 넘는 각은 여러 번)
            aim, _ = self._aim(node, fwd, sig_lat, sig_h)
            turns = 1.0 + abs(node.h + aim) / cap
        else:
            turns = abs(node.h) / cap if head_real else 0.0
        return (max(legs_dist, legs_lat) + turns, abs(pred), fwd)

    @staticmethod
    def _bucket(node):
        return (round(node.Lc / BUCKET[0]), round(node.Fc / BUCKET[1]), round(node.h / BUCKET[2]))

    # ── 속: 후진 · 못 믿을 때 ──
    def _remaining(self, fix, Lc, Fc, h):
        """마지막 단계에서 아직 남은 좌우와 그 σ — 정면을 본 뒤 카메라가 설 자리(Lc − b). 방향이 하한 아래라
        못 돌면 그 방향으로 눈 감고 새는 몫까지. 회전이 좌우를 A·sin h 옮기므로 fix.lateral_m 그대로가 아니다."""
        geo = self.geo
        residual = h if abs(h) < geo.floor else 0.0
        blind = geo.blind(fix.forward_m)
        rem = limits.arrival_lateral_m(Lc - geo.b, residual, blind)
        sig = math.hypot(fix.lateral_sigma_m, abs(geo.A) * math.radians(fix.heading_sigma_deg),
                         limits.lateral_leak_m(limits.tip_lever_m(blind), fix.heading_sigma_deg) if residual else 0.0)
        return rem, sig

    def _backup(self, fix, Lc, Fc, h):
        """(BackupPlan, 남은 좌우). sidestep.backup 에 '남은 좌우' 를 fix.lateral_m 자리에 넣어 준다."""
        geo = self.geo
        rem, sig = self._remaining(fix, Lc, Fc, h)
        b = sidestep.backup(replace(fix, lateral_m=rem, lateral_sigma_m=sig), self.learner,
                            geo.intr, geo.back_space, geo.cut)
        if b.why in ("", "no_space"):
            # 물러난 자리가 아직 "태그컷 도착"(더 갈 거리 ≤ arrive_tol) 이면 다음 걸음이 없다 — 그 밖으로 나가게,
            # 후진 자체도 앞뒤 허용(FWD_TOL_M)만큼 틀리니 그만큼 더. 뒤공간을 넘으면 그대로 no_space
            room = float(fix.forward_m) - geo.cut
            want = max(b.need_m, b.floor_m, geo.arrive_tol - room) + C.FWD_TOL_M
            b.distance_m, b.why = min(want, b.space_m), ("no_space" if want > b.space_m else "")
        return b, rem

    def _advance_m(self, fix):
        """좌우·방향 없이 β 와 거리만으로 — 태그를 놓치지 않고 곧장 갈 수 있는 거리 (plan 3-6 ①).
        방향을 몰라도 거리는 걸음보다 덜 줄어드니 '거리 − 걸음 ≥ 태그컷' 이면 위로는 안 잘린다."""
        geo = self.geo
        dist = float(fix.distance_m)
        dz = geo.dz                                             # 잰 높이차 (명목값 아님)
        rng = math.sqrt(max(0.0, dist * dist - dz * dz)) if dist > 0 else float(fix.forward_m)
        cap = min(C.STEP_FORWARD_HARD_MAX_M, rng - geo.cut)
        if cap < geo.min_step or cap <= 0:
            return 0.0
        beta = math.radians(float(fix.beta_deg))
        ahead, side = rng * math.cos(beta), rng * math.sin(beta)
        s, step = 0.0, C.FWD_TOL_M                             # 앞뒤 허용만큼씩 더듬는다
        while s + step <= cap + 1e-9:
            s2 = s + step
            beta2 = math.degrees(math.atan2(side, ahead - s2))
            if abs(beta2) > limits.path_angle_max_deg(rng - s2, geo.intr):
                break
            s = s2
        s = min(s, cap)                                         # 0.1 을 15번 더하면 1.5 를 2e-16 넘는다
        return s if s >= geo.min_step else 0.0


def _real(value, sigma, tol, k):
    """estimate.worth_fixing 과 같은 판정 — 흔들림보다 작은 오차는 유령이다. k 만 3-6 ③ 으로 낮출 수 있다."""
    if k == C.UNCERTAIN_FACTOR:
        return worth_fixing(value, sigma, tol)
    return abs(value) > max(tol, k * sigma)


def decide(fix, learner, progress=None, now=None, **geo):
    """한 번짜리. 이력이 필요 없을 때(도구·시험)."""
    return Planner(learner, progress, **geo).decide(fix, now)


if __name__ == "__main__":
    # 자체 시험 — 하드웨어 없이. (1) 회전 기하가 limits 의 배율과 맞나 (2) 시작 위치 표 (3) 태그컷 분기
    # (4) 못 믿을 때 (5) 2차원 세계에서 닫힌 고리 — 첫 걸음만 실행하고 다시 재기를 반복해 진입하는지 (모델 오차 포함)
    import time
    from ..control.learn import Learner, seeds
    from ..detection.estimate import Fix

    quiet = lambda *_: None                                     # noqa: E731
    lrn = Learner(seeds=seeds())
    meas = dict(A=-1.5, b=0.0, rms_mm=5.0)                       # plan 4-6 추정 (config 의 calibrate 값 자리)
    A0 = meas["A"]
    ref = limits.tag_cut_m()
    print("태그컷 %.2f m · 회전 하한 %.2f도 · 회전중심 %.2f m · 최소걸음 %.3f m" % (ref, lrn.rot_floor_deg, A0,
                                                                     min_step_m(lrn, strength_of("forward")) or 0.0))

    def fix_at(lat, fwd, h=0.0, sl=None, sh=None, ok=True, why="ok", **kw):
        """거리에 비례하는 σ (좌우 ∝ 거리², 방향 ∝ 거리). 9/21 실측 밀림(3.7 m 에서 좌우 31 mm)이면 태그컷에서
        σ 만으로 여유 30 mm 를 다 먹어 commit 이 영영 안 난다(plan 문제 5-1) — 여기선 그 1/4 로 놓고 논리만 본다"""
        r = fwd / 3.7
        sl = 0.008 * r * r if sl is None else sl
        sh = 0.15 * r if sh is None else sh
        dz = D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M
        return Fix(ok=ok, why=why, lateral_m=lat, heading_deg=h, forward_m=fwd,
                   distance_m=math.sqrt(lat * lat + fwd * fwd + dz * dz),
                   beta_deg=h + math.degrees(math.atan2(lat, fwd)) if fwd else 0.0,
                   lateral_sigma_m=sl, heading_sigma_deg=sh, **kw)

    # (1) 회전 기하 — h=0 에서 Δ 돌면 카메라 좌우 A·sinΔ, 태그 화면위치 변화 ≈ Δ·(d+A)/d (limits.turn_cap_deg 의 배율)
    geo = _Geo(lrn, **meas)
    for d0 in (3.5, 8.0):
        Lc, Fc = geo.center(0.0, d0, 0.0)
        lat1, fwd1 = geo.camera(Lc, Fc, 5.0)
        assert abs(lat1 - A0 * math.sin(math.radians(5.0))) < 1e-9
        beta1 = 5.0 + math.degrees(math.atan2(lat1, fwd1))
        gain = (d0 + A0) / d0
        assert abs(beta1 / 5.0 - gain) < 0.01, (d0, beta1 / 5.0, gain)
        assert abs(geo.camera(*geo.center(0.3, d0, -7.0), -7.0)[0] - 0.3) < 1e-9        # 왕복이 맞다
    assert geo.camera(*geo.center(0.0, 5.0, 0.0), 10.0)[0] < 0                          # 앞 중심, 왼쪽 회전 → 카메라 오른쪽
    g2 = _Geo(lrn, A=-1.5, b=0.05)
    assert abs(g2.camera(*g2.center(0.2, 5.0, 3.0), 3.0)[0] - 0.2) < 1e-9                # 좌우 오프셋 있어도 왕복
    print("회전 기하: 배율 (d+A)/d 일치, 왕복 일치")

    # (2) 시작 위치 표
    print("\n%-28s %-10s %s" % ("시작", "결정", "내용"))
    rows = [("8 m · 좌우 1.0", fix_at(1.0, 8.0), Progress()),
            ("5 m · 좌우 0.5", fix_at(0.5, 5.0), Progress()),
            ("3.6 m · 좌우 0.005", fix_at(0.005, 3.6), Progress()),
            ("3.6 m · 좌우 0.025", fix_at(0.025, 3.6), Progress()),
            ("8 m · 좌우 3.0 (첫 판단)", fix_at(3.0, 8.0), Progress()),
            ("8 m · 좌우 3.0 (걸음 뒤)", fix_at(3.0, 8.0), Progress(steps=1)),
            ("4 m · 좌우 1.0 (사이드스텝 뒤)", fix_at(1.0, 4.0), Progress(sidestepped=True)),
            ("8 m · 좌우 0.1 (잡음 안)", fix_at(0.1, 8.0, sl=0.08), Progress()),
            ("6 m · 좌우 0 · 방향 +8", fix_at(0.0, 6.0, 8.0), Progress())]
    got = {}
    for name, f, pr in rows:
        t0 = time.perf_counter()
        dcs = Planner(lrn, pr, **meas).decide(f, now=1000.0)
        ms = (time.perf_counter() - t0) * 1e3
        got[name] = dcs
        print("%-28s %-10s %s  [%.0f ms]" % (name, dcs.kind, dcs.summary(), ms))
    d = got["8 m · 좌우 1.0"]
    assert d.kind == "step" and d.turn_deg < 0 and d.drive_m > 0, d                     # 왼쪽에 있으니 오른쪽으로 돌아 대각선
    assert len(d.waypoints) == len(d.route) and abs(d.waypoints[0][1] - d.fwd_target_m) < 1e-3, d.waypoints
    wl, wf, wh = d.waypoints[-1]                                                        # 끝까지 그린 경로의 마지막 점 = 태그컷 · 법선 위 · 정면
    assert abs(wf - ref) <= C.FWD_TOL_M + 1e-6 and abs(wl) < C.SIDE_GAP_M and abs(wh) < 1.0, d.waypoints
    print("   경로점(좌우, 앞, 방향): " + " → ".join("(%+.2f, %.2f, %+.0f)" % w for w in d.waypoints))
    ws = got["8 m · 좌우 3.0 (첫 판단)"].waypoints                                       # 사이드스텝: 옆으로 절반 → 정면
    assert len(ws) == 2 and abs(ws[1][0] - 1.5) < 0.05 and abs(ws[1][2]) < 1e-6, ws
    assert d.route and d.complete, d
    d = got["5 m · 좌우 0.5"]
    assert d.kind == "step" and d.turn_deg < 0 and d.drive_m > 0, d
    d = got["3.6 m · 좌우 0.005"]
    assert d.kind == "step" and d.turn_deg == 0.0 and abs(d.drive_m - (3.6 - ref)) < 1e-3, d   # 그대로 태그컷까지
    d = got["3.6 m · 좌우 0.025"]
    assert d.kind == "step" and d.complete and d.route[-1][0] * d.route[0][0] < 0, d           # 곧장은 여유 5 mm 모자라(눈 감는 거리 = 3.3 − 1.52 − STANDOFF 0.5) — 짧게 비껴 갔다 돌아온다
    assert got["8 m · 좌우 3.0 (첫 판단)"].kind == "sidestep"
    d = got["8 m · 좌우 3.0 (걸음 뒤)"]
    assert d.kind in ("step", "stop") and d.kind != "sidestep", d                       # 한 번뿐 — 걸음 뒤엔 사이드스텝 없음
    assert got["4 m · 좌우 1.0 (사이드스텝 뒤)"].kind == "stop"                          # 통로 밖·경로 없음 → 정지·사람
    d = got["8 m · 좌우 0.1 (잡음 안)"]
    assert d.kind == "step" and d.turn_deg == 0.0, d                                   # 유령은 안 쫓는다 — 곧장 간다
    d = got["6 m · 좌우 0 · 방향 +8"]
    assert d.kind == "step" and d.turn_deg < 0, d                                      # 방향이 진짜 오차면 돈다
    assert all(v.kind != "sidestep" or v.sidestep.ok for v in got.values())

    # (3) 태그컷 분기 — commit · 방향 회전 · 후진 · 정지(2σ 안 · 뒤공간 · 횟수 · 시간)
    print("\n태그컷(%.2f m)에서:" % ref)
    cases = [
        ("정렬됨", fix_at(0.0, ref, 0.0, sl=0.008, sh=0.2), Progress(), "commit"),
        # 정면을 보면 카메라가 A·sin3 = 7.8 cm 움직인다 — 그만큼 반대쪽에 서 있어야 회전만으로 된다 (plan 4-6)
        ("방향 +3도 · 좌우 −0.078", fix_at(A0 * math.sin(math.radians(3.0)), ref, 3.0,
                                        sl=0.008, sh=0.2), Progress(), "step"),
        ("방향 +3도 · 좌우 0", fix_at(0.0, ref, 3.0, sl=0.008, sh=0.2), Progress(), "step"),      # 정면부터
        ("방향 0 · 좌우 0.078", fix_at(0.078, ref, 0.0, sl=0.008, sh=0.2), Progress(), "backup"),  # 정면인데 남았다 → 후진
        ("좌우 0.15 σ 0.02", fix_at(0.15, ref, 0.0, sl=0.02, sh=0.2), Progress(), "backup"),
        ("좌우 0.05 σ 0.04 (2σ 안)", fix_at(0.05, ref, 0.0, sl=0.04, sh=0.2), Progress(), "stop"),
        ("좌우 0.40 (뒤공간 0.5 부족)", fix_at(0.40, ref, 0.0, sl=0.02, sh=0.2), Progress(), "stop"),
        ("좌우 0.15 후진 뒤", fix_at(0.15, ref, 0.0, sl=0.02, sh=0.2), Progress(backed_up=True), "stop"),
        ("보정 5회 소진", fix_at(0.15, ref, 0.0, sl=0.02, sh=0.2), Progress(corrections=C.MAX_CORRECTIONS), "stop"),
        ("120 s 소진", fix_at(0.15, ref, 0.0, sl=0.02, sh=0.2), Progress(fine_since=1000.0 - C.FINE_TIME_LIMIT_S), "stop"),
    ]
    for name, f, pr, want in cases:
        dcs = Planner(lrn, pr, **meas).decide(f, now=1000.0)
        print("  %-26s → %-8s %s" % (name, dcs.kind, dcs.summary()))
        assert dcs.kind == want, (name, dcs)
        assert dcs.at_cut and pr.fine_since is not None
    dcs = Planner(lrn, **meas).decide(fix_at(A0 * math.sin(math.radians(3.0)), ref, 3.0,
                                           sl=0.008, sh=0.2), now=1000.0)
    assert abs(dcs.turn_deg + 3.0) < 0.5 and dcs.drive_m == 0.0 and dcs.correction, dcs
    dcs = Planner(lrn, **meas).decide(fix_at(0.0, ref, 3.0, sl=0.008, sh=0.2), now=1000.0)
    assert abs(dcs.turn_deg + 3.0) < 1e-9 and dcs.drive_m == 0.0 and dcs.correction, dcs   # 비스듬히 잰 좌우로 후진량을 안 정한다
    dcs = Planner(lrn, **meas).decide(fix_at(0.078, ref, 0.0, sl=0.008, sh=0.2), now=1000.0)
    assert dcs.kind == "backup" and abs(dcs.backup.need_m - limits.backup_needed_m(0.078, ref)) < 1e-9, dcs
    # 물러난 자리가 "태그컷 도착" 밖이어야 다음 걸음이 있다: 필요량 + 앞뒤 허용
    assert abs(dcs.drive_m - (max(dcs.backup.need_m, C.FWD_TOL_M) + C.FWD_TOL_M)) < 1e-9, dcs
    dcs = Planner(lrn, **meas).decide(fix_at(0.15, ref, 0.0, sl=0.02, sh=0.2), now=1000.0)
    assert dcs.movement == "backward" and dcs.correction and abs(dcs.fwd_target_m - (ref + dcs.drive_m)) < 1e-9
    assert abs(dcs.backup.need_m - limits.backup_needed_m(0.15, ref)) < 1e-9
    # 회전이 좌우를 옮기는 걸 계획기가 안다 — 좌우 0.14, 방향 −5도: 정면 보면 A·sin5 = 0.13 이 지워져 9 mm 남는다 → 회전 하나로 진입
    dcs = Planner(lrn, **meas).decide(fix_at(0.14, ref, -5.0, sl=0.008, sh=0.2), now=1000.0)
    print("  좌우 0.14 · 방향 −5도       → %-8s %s" % (dcs.kind, dcs.summary()))
    assert dcs.kind == "step" and dcs.turn_deg > 0 and dcs.complete, dcs
    # 상한 계수: 마지막 단계에서 step/backup 실행이 corrections 를 올린다
    pl = Planner(lrn, **meas)
    dcs = pl.decide(fix_at(A0 * math.sin(math.radians(3.0)), ref, 3.0, sl=0.008, sh=0.2), now=1000.0)
    pl.executed(dcs)
    assert dcs.kind == "step" and pl.progress.corrections == 1 and pl.progress.steps == 1
    pl.executed(Decision("backup"))
    assert pl.progress.corrections == 2 and pl.progress.backed_up
    pl2 = Planner(lrn, **meas)
    pl2.executed(pl2.decide(fix_at(1.0, 8.0), now=0.0))
    assert pl2.progress.corrections == 0 and pl2.progress.steps == 1                    # 태그컷 전엔 보정이 아니다

    # (4) 못 믿을 때 — few 는 β·거리로 곧장 가볼 수 있고, stale/clock/no_tag 는 더 본다
    f = fix_at(0.0, 6.0, ok=False, why="few")
    dcs = Planner(lrn, **meas).decide(f, now=1000.0)
    print("\nfew (6 m, β 0)        → %s" % dcs.summary())
    assert dcs.kind == "uncertain" and 0 < dcs.drive_m <= C.STEP_FORWARD_HARD_MAX_M
    f = fix_at(2.0, 6.0, ok=False, why="few")                                           # β 18도 — 옆으로 잘리기 전까지만
    dcs = Planner(lrn, **meas).decide(f, now=1000.0)
    print("few (6 m, β %.0f도)     → %s" % (f.beta_deg, dcs.summary()))
    assert dcs.kind == "uncertain" and dcs.drive_m < 6.0 - ref
    for why in ("stale", "clock", "no_tag", "no_sigma"):
        dcs = Planner(lrn, **meas).decide(fix_at(0.0, 6.0, ok=False, why=why), now=1000.0)
        assert dcs.kind == "uncertain" and dcs.drive_m == 0.0 and dcs.why.startswith(why)
    assert Planner(lrn, **meas).decide(fix_at(0.0, ref, ok=False, why="few"), now=1000.0).drive_m == 0.0
    # 회전중심을 모르면 한 걸음 회전이 5도 (결정 ③)
    dcs = Planner(lrn).decide(fix_at(1.0, 8.0), now=1000.0)
    assert dcs.kind == "step" and abs(dcs.turn_deg) <= C.TURN_MAX_UNKNOWN_CENTER_DEG + 1e-9, dcs
    # 완화 (3-6 ③): 2σ 안이던 좌우가 1.5σ 로는 진짜가 된다
    pl = Planner(lrn, **meas)
    f = fix_at(0.09, 6.0, sl=0.05, sh=0.3)
    assert pl.decide(f, now=0.0).turn_deg == 0.0
    pl.relax()
    assert pl.decide(f, now=0.0).turn_deg != 0.0

    # (4-b) 마지막 직진 구간 (final_m > 0) — 정렬선 = 태그컷 + FINAL_STRAIGHT_M. 그 안쪽은 조준 빗나감으로만 판단한다
    class FakeAim:
        def __init__(self, m, sg):
            self.m, self.sg = m, sg

        def miss(self, fix, dz):
            return self.m, self.sg
    FM = C.FINAL_STRAIGHT_M
    mk = lambda pr=None: Planner(lrn, pr, final_m=FM, **meas)   # noqa: E731
    floor_f = lrn.rot_floor_fine_deg
    print("\n마지막 구간 (정렬선 %.2f m · 태그컷 %.2f m · 약한 회전 하한 %.2f도):" % (ref + FM, ref, floor_f))
    dcs = mk().decide(fix_at(1.0, 8.0), now=0.0, aim=FakeAim(None, None))          # 멀리서는 접근 계획이 **정렬선**을 겨눈다
    assert dcs.kind == "step" and not dcs.final and abs(dcs.waypoints[-1][1] - (ref + FM)) <= C.FWD_TOL_M + 1e-6, dcs.waypoints
    assert abs(dcs.line_m - (ref + FM)) < 1e-9 and abs(dcs.cut_m - ref) < 1e-9
    f_in = fix_at(0.10, ref + FM, 3.0)                                               # 정렬선 위. PnP 는 좌우 10 cm · 방향 3도라 하지만 안 본다
    dcs = mk().decide(f_in, now=0.0, aim=FakeAim(None, None))
    print("  조준각 모름                 → %s" % dcs.summary())
    assert dcs.kind == "step" and dcs.final and dcs.fine and dcs.turn_deg == 0 and abs(dcs.drive_m - FM) < 1e-9 and not dcs.correction
    assert dcs.to_ref == "cut" and abs(dcs.ref_m - ref) < 1e-9                       # 구간을 한 번에 — 태그컷에서 끝난다
    dcs = mk().decide(f_in, now=0.0, aim=FakeAim(0.06, 0.005))
    print("  빗나감 +60±5 mm             → %s" % dcs.summary())
    assert dcs.kind == "step" and dcs.fine and dcs.correction and abs(dcs.turn_deg - turn_for(0.06, ref + FM, A0)) < 1e-9 and dcs.turn_deg > 0
    assert dcs.drive_m == 0.0 and dcs.to_ref == ""                                   # 돌기만 — 그 자리에서 다시 잰다
    for m_in in (0.008, 0.025, -0.029):                                              # 3 cm 안 → 안 돌고 태그컷까지 한 번에
        dcs = mk().decide(f_in, now=0.0, aim=FakeAim(m_in, 0.005))
        assert dcs.kind == "step" and dcs.turn_deg == 0 and abs(dcs.drive_m - FM) < 1e-9 and not dcs.correction and dcs.to_ref == "cut", (m_in, dcs)
    dcs = mk().decide(fix_at(0.0, ref + FM / 2, 0.0), now=0.0, aim=FakeAim(0.0, 0.004))
    assert dcs.to_ref == "cut" and abs(dcs.drive_m - FM / 2) < 1e-9                  # 구간 중간에 서 있으면 남은 만큼
    f_cut = fix_at(0.25, ref, -13.0)                                                 # 태그컷. PnP 가 뭐라 하든
    dcs = mk().decide(f_cut, now=0.0, aim=FakeAim(0.012, 0.004))
    print("  태그컷 · 빗나감 +12±4 mm    → %s" % dcs.summary())
    assert dcs.kind == "commit" and abs(dcs.drive_m - geo.blind(ref)) < 1e-9 and dcs.margin_m > 0
    assert mk().decide(f_cut, now=0.0, aim=FakeAim(0.028, 0.041)).kind == "commit"    # σ 가 커도 3 cm 안이면 간다 (판단은 빗나감)
    dcs = mk().decide(f_cut, now=0.0, aim=FakeAim(0.05, 0.004))
    print("  태그컷 · 빗나감 +50±4 mm    → %s" % dcs.summary())
    assert dcs.kind == "step" and dcs.drive_m == 0 and dcs.fine and dcs.correction and dcs.turn_deg > 0
    lrn.rot_floor_fine_deg = 5.0                                                     # 약한 회전 최소 요청각이 2.5도라면 → 더 못 고친다
    dcs = mk().decide(f_cut, now=0.0, aim=FakeAim(0.05, 0.004))
    print("  같은데 하한이 5도면          → %s" % dcs.summary())
    assert dcs.kind == "stop"
    lrn.rot_floor_fine_deg = floor_f
    assert mk(Progress(corrections=C.MAX_CORRECTIONS)).decide(f_cut, now=0.0, aim=FakeAim(0.05, 0.004)).kind == "stop"
    assert mk(Progress(corrections=C.MAX_CORRECTIONS)).decide(f_cut, now=0.0, aim=FakeAim(0.004, 0.004)).kind == "commit"   # 다섯 번째 보정이 맞았으면 간다
    assert mk().decide(f_cut, now=0.0, aim=FakeAim(None, None)).kind == "stop"       # 태그컷인데 조준각을 모른다
    pl = mk()
    d1 = pl.decide(f_in, now=0.0, aim=FakeAim(0.008, 0.005))
    pl.executed(d1)
    assert pl.progress.corrections == 0                                              # 곧장만 간 걸음은 보정이 아니다
    d2 = pl.decide(f_in, now=1.0, aim=FakeAim(0.06, 0.005))
    pl.executed(d2)
    assert pl.progress.corrections == 1
    # 닫힌 고리: 정렬선에서 빗나감 m0 으로 출발. 회전은 요청보다 0.3도 더 돌고, 조준은 ±2 mm 로 흔들린다
    import random as _r
    rg = _r.Random(11)
    for m0 in (0.12, -0.07, 0.02, 0.60):
        pl, fwd, m, hops = mk(), ref + FM, m0, []
        for step in range(12):
            dcs = pl.decide(fix_at(0.0, fwd, 0.0), now=float(step), aim=FakeAim(m + rg.gauss(0, 0.002), 0.004))
            hops.append("%s%+.2f/%.2f" % (dcs.kind[0], dcs.turn_deg, dcs.drive_m))
            if dcs.kind in ("commit", "stop"):
                break
            assert not (dcs.turn_deg and dcs.drive_m), dcs                           # 돌거나 가거나 — 한 걸음에 둘 다는 없다
            if dcs.turn_deg:
                done = dcs.turn_deg + math.copysign(0.3, dcs.turn_deg)
                m -= (fwd + A0) * math.sin(math.radians(done))                       # 진행선이 회전중심 둘레로 돈다
            fwd -= dcs.drive_m
            pl.executed(dcs)
        print("  빗나감 %+.0f mm 에서 출발 → %s: 끝 빗나감 %+.0f mm  %s" % (m0 * 1e3, dcs.kind, m * 1e3, " ".join(hops)))
        assert dcs.kind == "commit" and abs(m) < C.SIDE_GAP_M + 0.006, (m0, dcs)
        assert sum(1 for h in hops if h.startswith("s") and h.endswith("/%.2f" % FM)) == 1, hops   # 직진은 1.5 m 한 번

    # (5) 2차원 세계에서 닫힌 고리 — 세계의 회전중심이 계획기 가정과 0.06 m 다르고(plan 4-7 필요 정확도 ±0.07 안.
    #     0.1 m 면 9도 회전마다 1.6 cm 가 틀려 태그컷에서 2σ 안 잔여로 정지·사람이 난다 — 그게 결정 ⑥의 뜻이다),
    #     회전은 하한의 30 % 만큼 지나치고, 측정엔 σ 만큼 잡음이 섞인다. 첫 걸음만 실행하고 다시 잰다
    import random
    rng = random.Random(7)
    print("\n닫힌 고리:")
    for lat0, fwd0, h0, wA in ((1.0, 8.0, 0.0, -1.44), (-0.6, 6.0, 5.0, -1.56), (0.25, 5.0, -3.0, -1.5), (1.8, 8.0, 0.0, -1.44)):
        world = _Geo(lrn, A=wA)
        Lc, Fc, h = world.center(lat0, fwd0, h0)[0], world.center(lat0, fwd0, h0)[1], h0
        pl, log, outcome = Planner(lrn, **meas), [], None
        for step in range(C.MAX_STEPS):
            lat, fwd = world.camera(Lc, Fc, h)
            f = fix_at(lat, fwd, h)
            f.lateral_m += rng.gauss(0, f.lateral_sigma_m / 2)
            f.heading_deg += rng.gauss(0, f.heading_sigma_deg / 2)
            dcs = pl.decide(f, now=float(step))
            log.append("%s%+.1f/%.2f" % (dcs.kind[0], dcs.turn_deg, dcs.drive_m))
            if dcs.kind in ("commit", "stop", "uncertain"):
                outcome = dcs
                break
            if dcs.kind == "sidestep":
                sp = dcs.sidestep
                h = sidestep.normalize_deg(h + sp.turn1_deg)
                r = math.radians(h)
                sgn = 1.0 if sp.movement == "forward" else -1.0
                Lc, Fc = Lc + sgn * sp.drive_m * math.sin(r), Fc - sgn * sp.drive_m * math.cos(r)
                h = 0.0
            else:
                turn = dcs.turn_deg + math.copysign(lrn.rot_floor_deg * 0.3, dcs.turn_deg) if dcs.turn_deg else 0.0
                h = sidestep.normalize_deg(h + turn)
                r = math.radians(h)
                sgn = -1.0 if dcs.movement == "backward" else 1.0
                Lc, Fc = Lc + sgn * dcs.drive_m * math.sin(r), Fc - sgn * dcs.drive_m * math.cos(r)
            pl.executed(dcs)
        lat, fwd = world.camera(Lc, Fc, h)
        arrive = limits.arrival_lateral_m(lat, h, world.blind(fwd))
        print("  (%.2f, %.1f, %+.0f도, 세계 A %.2f) → %s: %d걸음, 도착 좌우 %+.0f mm  %s"
              % (lat0, fwd0, h0, wA, outcome.kind if outcome else "?", pl.progress.steps, arrive * 1e3, " ".join(log)))
        assert outcome is not None and outcome.kind == "commit", (lat0, outcome and outcome.summary(), log)
        assert abs(arrive) < C.SIDE_GAP_M, arrive
        assert pl.progress.steps <= C.MAX_STEPS
    print("\nplan 자체 시험 통과")
