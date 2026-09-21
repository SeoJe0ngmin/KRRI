"""도킹 실행 — **딸각 하나**.  (v3 연속 추정 + 수렴 제어)

    python tools/dock.py                      # 실주행 (SPACE 로 출발, Ctrl+C 로 즉시 정지)
    python tools/dock.py --dry-run            # 장비 없이 가짜 리그로 배선 확인 (맥에서도 됨)
    python tools/dock.py --sim                # 플랜트 폐루프 (카메라·검출 없이 빠르게)
    python tools/dock.py --no-can             # 카메라·IMU 는 진짜, CAN 만 안 연다
    python tools/dock.py --show --record-events

내일 아침(캘리브 3종이 아직 없을 때)의 문
────────────────────────────────────────────────────────────────────────────
캘리브 게이트가 막아도 **여기서 멈추지 않는다** — 경고하고 `--assume-calib` 를 권한다.
가정값으로 돌면 자동으로: 전진 97 한정 · 정지거리 ×1.5 · 다리 ≤1.0 m · 회전 ≤15° ·
FINAL 은 `--final-anyway` 없이는 못 들어감. 화면 맨 위에 "가정값 사용 중" 이 계속 뜬다.

부품 (전부 남이 만든 것 — 여기는 붙이기만 한다)
────────────────────────────────────────────────────────────────────────────
    TagPipeline          프레임 → 검출 → docking_state            (detection)
    Estimator            FrameObs → Estimate (매 프레임 하나)      (estimate)
    Dynamics             정지거리·명령시간·최소 증분               (dynamics)
    DockFSM              관측→결정→실행→정지→확인, Tier, CAN 창구  (control)
    SafeCanTx            **유일한 송신 창구** + 데드맨 0.5 s       (control.can_tx)
    TimingSession/LoopTick/RunLogger/calib                        (utils)

**CAN 을 직접 건드리는 줄은 이 파일에 없다.** tx 를 DockFSM 에 넣어 주고, 그쪽이
DECIDE→EXECUTE 한 자리에서만 `set_movement` 를 부른다(계약 §4.2·C팀 편차). 여기서
직접 보내면 같은 명령이 두 번 래치된다.

안전
────────────────────────────────────────────────────────────────────────────
· 출발 전 CAN 템플릿 검사 — byte1/2 외 비중립이면 **출발 거부**(byte4 는 포크 리프트다)
· 자이로 먼저 열고 카메라 나중 (RSUSB 는 먼저 연 쪽이 IMU 를 갖는다)
· 타임스탬프 GLOBAL 전환 대기, CAN 채널 프로브
· Ctrl+C·예외·종료 — 어느 길로 끝나도 stop 프레임
· ABORT 뒤에도 stop + heartbeat 를 **계속** 보낸다(버스 침묵 금지, AbortKeeper)
"""
import argparse
import asyncio
import dataclasses
import math
import os
import signal
import sys
import time
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools", "etc"))

from config import control as C                                    # noqa: E402
from config import detection as D                                  # noqa: E402
from src.models.control import can_tx                              # noqa: E402
from src.models.control.dock_fsm import DockFSM, Phase, TERMINAL    # noqa: E402
from src.models.detection.detection_pose import (bearing_px_deg,    # noqa: E402
                                                 pnp2_solutions)
from src.models.detection.detection_tag import (tag_edge_margin_px,  # noqa: E402
                                                tag_pixel_size)
from src.models.dynamics import Dynamics                           # noqa: E402
from src.models.estimate import FrameObs, Estimator, est_row, fork_tip_error  # noqa: E402
from src.utils import calib as CAL                                 # noqa: E402
from src.utils import timing as TM                                 # noqa: E402
from src.utils.run_log import RunLogger                            # noqa: E402

WORK = os.path.join(ROOT, "work_dirs", "dock")

#: 시뮬 한 스텝 [s]. 실주행에서는 카메라가 박자를 준다
SIM_DT = 1.0 / 30.0
#: 종단(DONE/ABORT) 뒤에도 이만큼 더 돌며 stop 을 계속 내보낸다 [s]
TAIL_S = 1.5
#: 한 줄 상태를 이보다 자주 다시 찍지 않는다 [s] (터미널이 루프를 잡아먹지 않게)
STATUS_EVERY_S = 0.25
#: --standoff 하한 [m]. 이보다 작으면 포크 끝 목표가 태그면 안쪽이 된다(벽 충돌)
STANDOFF_MIN_M = 0.20

#: Ctrl+C 가 눌린 순간 **먼저 차를 세우기** 위한 손잡이. 파이썬이 예외를 풀어내는 데
#: 시간이 걸려도 여기서 current_movement 를 stop 으로 바꾸면 다른 팀 TX 루프(10 ms)가
#: 그 다음 프레임부터 stop 을 보낸다.
_ACTIVE = {"run": None}


def _sigint(signum, frame):
    r = _ACTIVE.get("run")
    if r is not None:
        try:
            if r.tx is not None:
                r.tx.stop("SIGINT")
            if r.ctrl is not None:
                r.ctrl.current_movement = "stop"
        except Exception:
            pass
    raise KeyboardInterrupt


def _now_str():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _read_key_tty():
    """터미널에서 키 하나. 줄 단위·에코 없이. ESC 는 '\\x1b'."""
    import termios
    import tty
    fd = sys.stdin.fileno()
    old = termios.tcgetattr(fd)
    try:
        tty.setcbreak(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old)


def _err(fsm, est):
    """화면·로그가 쓰는 (e_l, σ) — **상태기계 판정과 같은 레버·같은 σ 식**.

    예전에는 blind_s_m 을 빼먹어 레버가 1.52 m 고정이고 σ 도 달라서, 같은 프레임에
    상태줄과 events.jsonl 의 숫자가 서로 달랐다(현장에서 줄자와 대조를 못 한다).
    """
    s = 0.0
    if est.x_m == est.x_m:
        s = max(0.0, est.x_m - fsm.x_stop_cam)
    extra = math.sqrt(max(0.0, fsm.sigma_theta_deg ** 2 + fsm.sigma_delta_deg ** 2))
    return fork_tip_error(est, C.CAM_TO_REF_M, fsm.x_off_m, blind_s_m=s,
                          sigma_extra_deg=extra, sigma_roll_m=fsm.sigma_roll_m)


def _f(v, fmt="%.3f", none="?"):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return none
    return none if x != x else fmt % x


# ═══════════════════════════════════════════════════════════════════════════
# 프레임 한 장 → frame.jsonl 한 줄 (= FrameObs 의 입력)
# ═══════════════════════════════════════════════════════════════════════════

def row_from(res, tag_id, seq, gyro=None, tx=None, timing=None):
    """`Result` → 로그 한 줄. **키 이름은 first_run._log_frame 과 같아야 한다** —
    그래야 내일 로그로 오프라인 재생·시뮬·실주행이 한 코드로 돈다(계약 §2.1).
    """
    g = gyro
    row = {"i": seq, "seen": False,
           "movement": (tx.movement if tx is not None else None),
           "t_cmd_set": (tx.t_cmd_set if tx is not None else None),
           "gyro_deg": (g.angle_deg if g is not None else None),
           "gyro_dps": (g.rate_dps if g is not None else None),
           "gyro_gaps": (g.stats().get("gaps") if g is not None else 0),
           "gyro_alive": bool(getattr(g, "alive", False)) if g is not None else False,
           "gyro_age_s": (g.age_sec if g is not None else None)}
    if tx is not None and tx.probe is not None and tx.probe.tx_marks:
        row["t_cmd_tx"] = tx.probe.tx_marks[-1][0]
    pr = res.primary(tag_id)
    if pr is None:
        return row
    det, doc, q = pr["detection"], pr["docking"], (pr["quality"] or {})
    shape = getattr(res.image, "shape", (0, 0))
    row.update({
        "seen": True,
        "tag_px": q.get("tag_px") or tag_pixel_size(det),
        "margin_px": tag_edge_margin_px(det, shape),
        "beta_px_deg": bearing_px_deg(det, res.intrinsics),
        "center_px": [float(x) for x in det.center],
        "decision_margin": float(getattr(det, "decision_margin", 0.0)),
        "hamming": int(getattr(det, "hamming", 0)),
        "reproj_rms_px": q.get("reproj_rms_px"),
        "quality_ok": bool(q.get("ok", True)),
        "pnp2": pnp2_solutions(det, res.intrinsics, D.TAG_SIZE_M)})
    if doc is not None:
        row.update({"lateral": doc["lateral"], "forward": doc["forward"],
                    "vertical": doc["vertical"], "heading_deg": doc["heading_deg"],
                    "tilt_deg": doc["tilt_deg"], "distance": doc["distance"]})
    stamps = getattr(res, "stamps", None)
    if stamps is not None:
        # 태그 중심 행으로 롤링셔터 보정을 다시 건다(t_line_us 가 캘리브에 있을 때만).
        # 예전엔 row_px 를 적어 두기만 하고 t_capture 를 안 고쳐서 보정이 영원히 0 이었다.
        if timing is not None:
            timing.apply_row(stamps, float(det.center[1]))
        elif stamps.row_px is None:
            stamps.row_px = float(det.center[1])
    return row


# ═══════════════════════════════════════════════════════════════════════════
# 한 번의 도킹
# ═══════════════════════════════════════════════════════════════════════════

class Dock:
    def __init__(self, args):
        self.args = args
        self.sim = bool(args.sim)
        self.dry = bool(args.dry_run)
        self.fake = self.sim or self.dry
        self.dir = None
        if args.record_events or not args.no_record:
            self.dir = os.path.join(WORK, "%s%s" % (_now_str(),
                                                    "_sim" if self.sim else
                                                    ("_dry" if self.dry else "")))
        self.logger = RunLogger(self.dir)
        # 캘리브를 읽기 전이라 기본값으로 먼저 세운다. open() 이 timing_calib 의
        # t_line_us·Δ_FS·L p99 로 **다시 만든다**(그 전까지 프레임을 받지 않는다).
        self.timing = TM.TimingSession(log=self.say)
        self.tick = TM.LoopTick()
        self.lines = []
        self.pipe = self.cam = self.plant = None
        self.gyro = self.ctrl = self.tx = None
        self.est = self.dyn = self.fsm = None
        self.calibs = None
        self.decision = None
        self.tasks = []
        self.tag_id = int(args.tag_id)
        self.tag_size = float(args.tag_size)
        self.seq = 0
        self.t_status = 0.0
        self.win = None

    # ── 출력 ───────────────────────────────────────────────────────────────
    def say(self, text=""):
        print("  %s" % text, flush=True)
        self.lines.append(text)

    def now(self):
        """세션 시계.

        **가짜 리그(sim·dry-run)는 반드시 플랜트의 가상 시계를 쓴다.** 벽시계를 쓰면
        플랜트는 프레임당 1/30 s 씩 가는데 루프는 그보다 훨씬 빨리 돌아 상태기계의
        '지금' 과 차의 '지금' 이 어긋난다 — 명령시간이 실제보다 수십 배 길어져
        태그면을 지나 16 m 를 더 가는 것을 2026-09-21 dry-run 에서 봤다.
        """
        if self.fake and self.plant is not None:
            return self.plant.t
        return time.time()

    # ── 열기 ───────────────────────────────────────────────────────────────
    async def open(self):
        self.say("기록 폴더 %s" % (self.dir or "(안 남김)"))
        self.calibs = CAL.load_all()
        self._make_timing()
        self.decision = CAL.gate(calibs=self.calibs, need=("timing", "dynamics"))
        self.decision.report(self.say)

        # CAN 템플릿 — 보내기 전에 무조건. byte4 는 이 지게차에서 포크 리프트다
        try:
            tpl = can_tx.check_templates(log=self.say,
                                         forward_slow_expect=C.FORWARD_SLOW)
            self.logger.event("can_templates", templates=tpl)
        except ImportError as exc:
            if not self.fake and not self.args.no_can:
                raise
            self.say("(canlib 없음 — 템플릿 확인 생략: %s)" % exc)

        if self.sim:
            await self._open_sim()
        elif self.dry:
            await self._open_fake()
        else:
            await self._open_real()

        # ── 부품 조립 ──────────────────────────────────────────────────────
        # 프레임 채택 문턱을 캘리브로 덮는다(없으면 config 기본값 그대로)
        per = self.calibs.get("perception")
        if self.pipe is not None and per is not None and per.ok:
            th = {k: per.get("static.%s" % k) for k in
                  ("stable_tag_px", "max_reproj_rms_px", "min_decision_margin",
                   "reliable_tilt_deg")}
            th = {k: v for k, v in th.items() if v is not None}
            self.pipe.quality_thresholds = th
            if th:
                self.say("프레임 채택 문턱(캘리브): %s" % th)
        intr = getattr(self.pipe, "intr", None) if self.pipe else getattr(self.plant, "intr", None)
        self.est = Estimator(intrinsics=intr, calibs=self.calibs,
                             tag_size_m=self.tag_size)
        self.dyn = Dynamics(self.calibs.get("dynamics"), log=self.say)
        self.dyn.report(self.say)
        self.fsm = DockFSM(calibs=self.calibs, dyn=self.dyn, tx=self.tx,
                           logger=self.logger, log=self.say,
                           assume_calib=self.args.assume_calib,
                           final_anyway=self.args.final_anyway,
                           clock=self.now)
        self.fsm.level_cap = int(self.args.level)     # 97 을 주면 저속만 (올라가진 않는다)
        if self.args.standoff is not None:
            # 정지점을 현장에서 당기거나 미는 문. 파생값이라 config 를 안 고친다.
            # **음수를 막는다** — x_ref(포크 끝)가 카메라 앞 1.52 m 라 standoff < 0 이면
            # 목표가 태그면 **뒤**가 되고, 코드는 그걸 성공(done_dr)으로 끝냈다.
            if float(self.args.standoff) < STANDOFF_MIN_M:
                raise SystemExit(
                    "--standoff %.2f m 는 하한 %.2f m 보다 작다 — 포크 끝 목표가 태그면 "
                    "안쪽으로 들어간다(벽 충돌). 0 이상으로 줄 것"
                    % (self.args.standoff, STANDOFF_MIN_M))
            self.fsm.x_stop_cam = C.CAM_TO_REF_M + float(self.args.standoff)
            self.fsm.x_T = max(self.fsm.x_stop_cam + self.dyn.min_forward_m(97),
                               self.fsm.tag_cut_m + 0.20)
            self.say("--standoff %.2f m → 카메라 정지점 %.2f m (기본 %.2f)"
                     % (self.args.standoff, self.fsm.x_stop_cam,
                        C.CAM_TO_REF_M + C.STANDOFF_M))
        self.logger.snapshot(self.calibs, argv=sys.argv,
                             extra={"dock": {"sim": self.sim, "dry_run": self.dry,
                                             "fake": self.fake,
                                             "assume_calib": self.args.assume_calib,
                                             "final_anyway": self.args.final_anyway,
                                             "level": self.args.level,
                                             "x_stop_cam": self.fsm.x_stop_cam,
                                             "x_T": self.fsm.x_T}})
        _ACTIVE["run"] = self
        self.logger.event("dock_open", sim=self.sim, dry_run=self.dry, fake=self.fake,
                          gate_allow=self.decision.allow,
                          domain_color=self.timing.domain_color,
                          domain_gyro=self.timing.domain_gyro)

    def _make_timing(self):
        """timing_calib 을 **실제로 읽어** 세션 시계를 만든다 (plan 4-1·4-2).

        예전에는 `TimingSession(log=...)` 만 불러 t_line_us·Δ_FS 가 영영 None 이고
        stale 문턱이 150 ms 고정이었다 — 1080p + USB 패스스루에서 L 이 그걸 조금만
        넘으면 모든 프레임이 stale 이 되어 차가 한 발짝도 못 나가는데 손잡이가 없었다.
        """
        tc = self.calibs.get("timing")
        g = (lambda k, d=None: d if tc is None else tc.get(k, d))
        l99 = g("L_ms.p99")
        stale = TM.STALE_S
        if l99 is not None and float(l99) * 1.2e-3 > stale:
            stale = float(l99) * 1.2e-3
            self.say("!! 프레임 지연 L p99 %.0f ms (캘리브) — stale 문턱을 %.0f ms 로 넓힌다"
                     % (float(l99), stale * 1000.0))
        self.timing = TM.TimingSession(t_line_us=g("t_line_us.value"),
                                       delta_fs_ms=g("delta_fs_ms.value"),
                                       stale_s=stale, log=self.say)
        self.say("  타이밍: T_line %s µs · Δ_FS %s ms · stale %.0f ms"
                 % (_f(g("t_line_us.value"), "%.1f"), _f(g("delta_fs_ms.value"), "%.1f"),
                    stale * 1000.0))

    def _session_timing(self):
        """이번 세션에서 **실제로 잰** 타이밍 — 캘리브 2층 허용창의 입력."""
        summ = self.timing.summary()
        gaps = None
        if self.gyro is not None:
            try:
                gaps = int(self.gyro.stats().get("gaps") or 0)
            except Exception:
                gaps = None
        return CAL.SessionTiming(epsilon_ms=None, gyro_gaps=gaps,
                                 l_p99_ms=summ.get("L_ms_p99"),
                                 domain_color=self.timing.domain_color,
                                 domain_gyro=self.timing.domain_gyro)

    async def _open_sim(self):
        import sim_plant as SP
        self.say("SIM — 플랜트 폐루프. 카메라·검출·CAN 없음, 가상 시계로 돈다")
        psi = self.args.psi
        if psi is None:                       # 기본은 태그를 겨냥한 자세에서 출발
            psi = -math.degrees(math.atan2(self.args.lat, max(self.args.dist, 0.1)))
        self.plant = SP.SimPlant(forward=self.args.dist, lateral=self.args.lat,
                                 heading_deg=psi, seed=self.args.seed,
                                 timing=self.timing)
        self.say("시작 자세 d %.2f m  ℓ %+.2f m  ψ %+.1f°  (seed %d)"
                 % (self.args.dist, self.args.lat, psi, self.args.seed))
        from fake_rig import FakeController
        self.ctrl = FakeController(plant=self.plant, logger=self.logger)
        self.tx = can_tx.SafeCanTx(self.ctrl, logger=self.logger, log=self.say,
                                   clock=self.now)

    async def _open_fake(self):
        from fake_rig import FakeCamera, FakeController, FakeGyro, FakePlant
        from src.models import TagPipeline
        from src.models.detection.image import intrinsics_from_ref
        self.say("DRY-RUN — 가짜 카메라·자이로·CAN. **아무것도 안 보낸다**. "
                 "검출·자세·추정·상태기계는 진짜 코드로 돈다")
        intr = intrinsics_from_ref((480, 640))      # 가짜는 작게 — 검출은 진짜로 돈다
        psi = self.args.psi if self.args.psi is not None else \
            -math.degrees(math.atan2(self.args.lat, max(self.args.dist, 0.1)))
        self.plant = FakePlant(forward=self.args.dist, lateral=self.args.lat,
                               heading_deg=psi, vertical=-1.10, seed=self.args.seed)
        self.cam = FakeCamera(self.plant, intr, self.tag_size, tag_id=self.tag_id,
                              timing=self.timing)
        self.pipe = TagPipeline(frames=self.cam, intrinsics=intr,
                                tag_size=self.tag_size, heading_sigma=False,
                                label="fake")
        self.gyro = FakeGyro(self.plant).enable_raw()
        self.timing.note_domain("gyro", self.gyro.domain)
        self.ctrl = FakeController(self.plant, logger=self.logger)
        self.tx = can_tx.SafeCanTx(self.ctrl, logger=self.logger, log=self.say,
                                   clock=self.now)
        self.say("시작 자세 d %.2f m  ℓ %+.2f m  ψ %+.1f°"
                 % (self.args.dist, self.args.lat, psi))

    async def _open_real(self):
        from src.models import TagPipeline
        from src.utils.camera import CameraSettings
        from src.utils.imu_yaw import GyroYaw
        if not self.args.no_reset:
            TM.hardware_reset(log=self.say)
        # 1) 자이로 **먼저** — RSUSB 는 먼저 연 쪽이 IMU 를 갖는다. 순서를 바꾸면
        #    뒤에 여는 자이로가 'failed to set power state' 로 안 열린다
        self.gyro = GyroYaw().start().enable_raw()
        self.say("자이로 열림. 정지 보정 — 차를 세워 둘 것...")
        rep = await asyncio.to_thread(self.gyro.calibrate)
        self.say("   축 %s, 잡음 %.3f 도/s, 드리프트 %.2f 도/분%s"
                 % (rep["axis_src"], rep["noise_dps"], rep["drift_dpm"],
                    "  !! 보정 중 움직임" if rep["moving"] else ""))
        self.logger.event("gyro_calibrate",
                          **{k: v for k, v in rep.items() if k != "accel_mean"})
        # 2) 카메라 — queue=1 + global_time. heading_sigma=False 로 프레임당 4 ms
        #    몬테카를로를 끈다(추정기가 σ_ψ 를 해석식으로 낸다, 계약 §2.5)
        tune = CameraSettings.docking()
        if self.args.auto_exposure:
            tune.enable_auto_exposure = True
            tune.exposure = None
            tune.gain = None
        self.pipe = TagPipeline.from_realsense(self.tag_size, tune=tune,
                                               timing=self.timing,
                                               heading_sigma=False, label="dock")
        self.say("카메라 열림 (queue=1, global_time). 태그 %d, %.3f m"
                 % (self.tag_id, self.tag_size))
        # 3) CAN
        if self.args.no_can:
            from fake_rig import FakeController
            self.ctrl = FakeController(None, logger=self.logger)
            self.say("--no-can — CAN 을 안 연다(카메라·IMU 만. 아무것도 안 보낸다)")
        else:
            from src.models.control.control_forklift_v2 import DirectFrameForkliftController
            self.ctrl = DirectFrameForkliftController()
            if not self.ctrl.connect_can():
                raise SystemExit("CAN 연결 실패")
            self.ctrl.is_running = True
            self.tasks = [asyncio.create_task(self.ctrl.control_tx_loop()),
                          asyncio.create_task(self.ctrl.movement_tx_loop()),
                          asyncio.create_task(self.ctrl.heartbeat_loop())]
        self.tx = can_tx.SafeCanTx(self.ctrl, logger=self.logger, log=self.say)
        probe = self.tx.attach()
        self.tasks.append(asyncio.create_task(self.tx.watchdog()))
        await asyncio.sleep(0.5)
        if probe is not None:
            self.say("CAN 채널 프로브: write %d 건, TXACK %s"
                     % (probe.writes, "켜짐" if probe.txack_on else "안 켜짐"))
            self.logger.event("can_probe", **probe.stats())
        # 4) 타임스탬프 GLOBAL 전환 대기 (컬러·자이로 둘 다)
        await self.wait_global()

    async def wait_global(self):
        t0 = time.time()
        while time.time() - t0 < TM.GLOBAL_WAIT_S:
            await self.pump_frame(log=False)
            self.timing.note_domain("gyro", getattr(self.gyro, "domain", None))
            if self.timing.global_ready:
                self.say("타임스탬프 GLOBAL 확인 (컬러 %.1fs / 자이로 %.1fs)"
                         % (self.timing.t_global_color - self.timing.t_open,
                            self.timing.t_global_gyro - self.timing.t_open))
                self.logger.event("global_time", ok=True)
                return True
        miss = self.timing.global_missing()
        self.say("!! GLOBAL 전환 실패 %s — 시각이 안 맞으면 정지 예측이 통째로 틀어진다"
                 % miss)
        self.logger.event("global_time", ok=False, missing=miss)
        if not self.args.force:
            raise SystemExit("타임스탬프 도메인이 GLOBAL 이 아니다 — --force 로만 강행")
        return False

    # ── 닫기 ───────────────────────────────────────────────────────────────
    async def close(self):
        # **stop 이 먼저다.** 동기로 — 루프가 취소된 뒤에도 값은 바뀐다
        try:
            if self.tx is not None:
                self.tx.stop("dock_close")
            if self.ctrl is not None:
                self.ctrl.current_movement = "stop"
        except Exception:
            pass
        try:
            if not self.fake:
                await asyncio.sleep(0.3)       # stop 프레임이 몇 번 나갈 시간
        except Exception:
            pass
        for t in self.tasks:
            t.cancel()
        for closer in ((lambda: self.logger.imu(self.gyro.drain_raw())),
                       (lambda: self.gyro.close()),
                       (lambda: self.pipe.close()),
                       (lambda: self.win and __import__("cv2").destroyAllWindows())):
            try:
                closer()
            except Exception:
                pass
        try:
            if self.ctrl is not None and not self.fake and not self.args.no_can:
                self.ctrl.is_running = False
                self.ctrl.disconnect_can()
        except Exception:
            pass
        self.logger.event("dock_close", frames=self.seq,
                          timing=self.timing.summary(), loop_tick=self.tick.summary(),
                          can=(self.tx.stats() if self.tx is not None else None),
                          fsm=(self.fsm.summary() if self.fsm is not None else None),
                          estimator=(self.est.summary() if self.est is not None else None))
        self.logger.close()
        _ACTIVE["run"] = None
        if self.dir:
            self.say("기록: %s  (%s)" % (self.dir, self.logger.summary()))

    # ── 프레임 ─────────────────────────────────────────────────────────────
    async def pump_frame(self, log=True):
        """프레임 한 장을 읽어 row 로 만든다. 상태기계는 아직 안 돌린다."""
        self.tick.tick()
        if self.tx is not None:
            self.tx.feed()                     # 데드맨: 판단 루프가 살아 있다
        if self.sim:
            self.plant.step(SIM_DT)
            row = self.plant.observe()
            self.seq = row["seq"]
            return row, None
        if self.dry:
            item = self.cam.next_frame()
        else:
            item = await asyncio.to_thread(next, self.pipe.frames, None)
        if item is None:
            return None, None
        i, ts, img = item
        res = self.pipe.process(img, index=i, timestamp=ts)
        self.seq += 1
        row = row_from(res, self.tag_id, self.seq, gyro=self.gyro, tx=self.tx,
                       timing=self.timing)
        return row, res

    # ── 본 루프 ────────────────────────────────────────────────────────────
    async def run(self):
        # 출발 직전에 **세션에서 실제로 잰 값**(L p99·gaps·도메인)을 넣어 2층 허용창을
        # 다시 본다. 예전에는 session= 을 안 넘겨 2층이 통째로 죽어 있었고,
        # `timing.gate.passed` 를 아무도 안 읽어 '게이트 미통과' 캘리브로 실주행이 됐다.
        self.decision = CAL.gate(calibs=self.calibs, need=("timing", "dynamics"),
                                 session=self._session_timing())
        tc = self.calibs.get("timing")
        if tc is not None and tc.ok and tc.get("gate.passed") is False:
            self.decision.allow = False
            self.decision.reasons.append(
                "timing_calib 의 출발 게이트가 미통과다(gate.passed=false) — "
                "TXACK 미측정 또는 자이로 유실. analyze_first_run 을 다시 돌릴 것")
        for r in self.decision.reasons:
            self.say("  캘리브 거부 사유: %s" % r)
        gate_ok = bool(self.decision.allow)
        if not gate_ok and not self.args.assume_calib:
            self.say("")
            self.say("!! 캘리브가 없다(또는 거부). 계약 §5.4 대로 기본은 **실주행 거부** 다.")
            self.say("!! 내일 아침처럼 캘리브가 아직 없는 상태로 한 번 돌리려면 "
                     "`--assume-calib` 를 직접 줘라 —")
            self.say("!!   그러면 9/7 가정값으로 돌되 97 한정 · 다리 ≤1.0 m · 회전 ≤15° · "
                     "정지거리 ×1.5 가 강제된다.")
            if not self.fake:
                raise SystemExit("캘리브 게이트 미통과 — --assume-calib 로만 강행")
        if self.args.assume_calib:
            self.say("!! **가정값 사용 중** (--assume-calib). %s"
                     % (self.dyn.degraded_reason or "9/7 실측 가정값"))
        if self.args.level == 67 and self.dyn.force_slow:
            self.say("!! --level 67 을 줬지만 강등 상태다 — 97 로 간다 (계약 §5.4)")
        elif self.args.level == 97:
            self.say("--level 97 : FAR 에서도 저속만 쓴다")

        if not self.fake:
            await self._wait_start()
        self.say("")
        self.say("출발. Ctrl+C 로 즉시 정지·종료")
        self.fsm.arm(gate_ok=gate_ok or self.args.assume_calib, why="SPACE")

        t_end = self.now() + self.fsm.budget_s + 30.0
        while self.now() < t_end:
            row, res = await self.pump_frame()
            if row is None:
                self.say("프레임 소스가 끝났다")
                break
            stamps = row.get("stamps") if self.sim else (
                getattr(res, "stamps", None) if res is not None else None)
            obs = FrameObs.from_row(row, stamps=stamps)
            est = self.est.update(obs)
            if self.fake and self.plant is not None:
                # 추정기는 t_pub 에 호스트 벽시계를 넣는다. 가짜 리그는 가상 시계로
                # 도니까 상태기계가 쓰는 '지금' 을 가상 시계로 바꿔 끼운다
                est = dataclasses.replace(est, t_pub=self.plant.t)
            if self.tx is not None:
                self.tx.check()
            imu_rows = None
            if self.gyro is not None:
                try:
                    imu_rows = self.gyro.drain_raw()
                except Exception:
                    imu_rows = None
                if imu_rows:
                    self.logger.imu(imu_rows)
            cmd = self.fsm.step(est, self.dyn,
                                gyro_deg=row.get("gyro_deg"),
                                i1_obs_deg=self.est.identity_obs_deg,
                                imu_rows=imu_rows)
            self._log(row, res, est, stamps)
            self._status(est, cmd)
            if self.args.show and res is not None:
                try:
                    if not self._draw(res, est):
                        self.say("창에서 종료 요청")
                        break
                except Exception as exc:
                    # 화면이 죽어도 주행은 죽으면 안 된다(DISPLAY 없음·X 끊김 등)
                    self.args.show = False
                    self.say("!! 화면을 못 그린다 — 끄고 계속한다 (%s: %s)"
                             % (type(exc).__name__, exc))
            if self.fsm.phase in TERMINAL:
                break
        else:
            self.say("!! 바깥 시간 캡 — 루프를 끝낸다")

        # 종단 뒤에도 stop 을 계속 흘린다 (ABORT heartbeat, 계약 §4.6).
        # 상태기계는 이미 종단이라 step() 을 안 도니 여기서 AbortKeeper 를 대신 굴린다.
        # **ABORT 는 사람이 Ctrl+C 를 누를 때까지 계속 보낸다** — 차량 워치독 T_wd 가
        # 미측정이라 '마지막 명령 유지' 를 전제로 버스를 끊으면 안 된다(계약 §4.6).
        # DONE/DONE_UNVERIFIED 는 TAIL_S 만 보내고 stop 상태로 닫는다.
        tail = TAIL_S
        if self.fsm is not None and self.fsm.phase is Phase.ABORT and not self.fake:
            tail = float("inf")
            self.say("!! ABORT — stop + heartbeat 를 **계속** 보낸다. 끝내려면 Ctrl+C")
        t0 = self.now()
        while self.now() - t0 < tail:
            if self.tx is not None:
                if self.tx.movement != "stop":        # 같은 명령을 매 틱 다시 래치하지 않는다
                    self.tx.stop(why="terminal")
                self.tx.feed()
            if self.fsm is not None:
                self.fsm.keeper.pump(self.now())
            if self.sim:
                self.plant.step(SIM_DT)
            elif self.dry:
                self.cam.next_frame()          # 가짜 카메라가 플랜트를 굴린다
            else:
                await asyncio.sleep(0.02)
        self._report()
        return self.fsm.phase

    async def _wait_start(self):
        if os.name != "nt" and sys.stdin.isatty():
            self.say("SPACE 를 누르면 시작. ESC 면 중단.")
            while True:
                k = await asyncio.to_thread(_read_key_tty)
                if k == " ":
                    return
                if k in ("\x1b", "q"):
                    raise KeyboardInterrupt
        await asyncio.to_thread(input, "  시작하려면 엔터 > ")

    # ── 기록 ───────────────────────────────────────────────────────────────
    def _log(self, row, res, est, stamps):
        e = est_row(est, self.fsm.delta_deg)
        e["i1_obs_deg"] = (None if self.est.identity_obs_deg != self.est.identity_obs_deg
                           else round(self.est.identity_obs_deg, 5))
        e_l, sig = _err(self.fsm, est)
        # 시뮬 row 는 stamps 객체를 안에 들고 온다 — 로그에는 as_dict 로 풀어 들어가므로 뺀다
        fields = {k: v for k, v in row.items() if k != "stamps"}
        self.logger.frame(stamps=stamps, est=e,
                          phase=self.fsm.phase.value, zone=int(self.fsm.zone),
                          tier=int(self.fsm.tier), e_l=e_l, sigma_e_l=sig,
                          **fields)

    # ── 한 줄 상태 (현장에서는 이게 전부다) ────────────────────────────────
    def _line(self, est, cmd):
        e_l, sig = _err(self.fsm, est)
        nxt = (cmd.why if cmd is not None else
               (self.fsm.command.why if self.fsm.command is not None
                and self.fsm.phase is Phase.EXECUTE else ""))
        flags = "".join((
            "" if est.tag_seen else "태그없음 ",
            "" if est.gyro_alive else "자이로X ",
            "AMBIG " if est.ambiguous else "",
            "STALE " if est.stale else "",
            "H" if est.heading_valid else "h",
            "L" if est.lateral_valid else "l"))
        return ("%-5s %-9s T%d  d %sm  e_l %smm(σ%s)  ψ %s°(σ%s)  β %s°  v %s  %s | %s"
                % (self.fsm.zone.name, self.fsm.phase.value, int(self.fsm.tier),
                   _f(est.x_m, "%5.2f"), _f(e_l * 1000.0, "%+4.0f"),
                   _f(sig * 1000.0, "%3.0f"), _f(est.psi_deg, "%+5.1f"),
                   _f(est.sigma_psi_deg, "%.1f"), _f(est.beta_deg, "%+5.1f"),
                   _f(est.v_mps, "%.2f"), flags, nxt[:52]))

    def _status(self, est, cmd):
        now = time.time()
        if cmd is None and (now - self.t_status) < STATUS_EVERY_S:
            return
        self.t_status = now
        print("  " + self._line(est, cmd), flush=True)

    # ── 화면 ───────────────────────────────────────────────────────────────
    def _draw(self, res, est):
        try:
            import cv2
            import numpy as np
        except Exception:
            return True
        img = np.asarray(res.image)
        if img.ndim == 2:
            img = cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
        else:
            img = img.copy()
        pr = res.primary(self.tag_id)
        if pr is not None:
            from src.utils.drawing import draw_corners
            draw_corners(img, pr["detection"])
            c = pr["detection"].center
            cv2.circle(img, (int(c[0]), int(c[1])), 4, (0, 255, 255), -1)
        h, w = img.shape[:2]
        cv2.line(img, (w // 2, 0), (w // 2, h), (80, 80, 80), 1)
        top = self.dyn.degraded_reason
        if top:
            cv2.rectangle(img, (0, 0), (w, 22), (0, 0, 140), -1)
            cv2.putText(img, "ASSUMED: " + top[:70], (6, 16),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.45, (255, 255, 255), 1)
        # 한글은 cv2 로 못 그린다 — 화면에는 숫자만, 한국어는 터미널 줄이 맡는다
        e_l, sig = _err(self.fsm, est)
        txt = ["%s / %s / tier%d" % (self.fsm.zone.name, self.fsm.phase.value,
                                     int(self.fsm.tier)),
               "d %s m   e_l %s mm (s %s)" % (_f(est.x_m, "%.2f"),
                                              _f(e_l * 1000, "%+.0f"),
                                              _f(sig * 1000, "%.0f")),
               "psi %s deg (s %s)   beta %s" % (_f(est.psi_deg, "%+.1f"),
                                                _f(est.sigma_psi_deg, "%.2f"),
                                                _f(est.beta_deg, "%+.1f")),
               "v %s m/s   tx %s" % (_f(est.v_mps, "%.2f"),
                                     self.tx.movement if self.tx else "-")]
        for k, t in enumerate(txt):
            cv2.putText(img, t, (6, 44 + 20 * k), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 1)
        self.win = "dock"
        cv2.imshow(self.win, img)
        return (cv2.waitKey(1) & 0xFF) not in (27, ord("q"))

    # ── 끝내고 나서 ────────────────────────────────────────────────────────
    def _report(self):
        self.say("")
        self.say("═══ 결과 ═══")
        s = self.fsm.report(self.say)
        acc = self.fsm.last_error or {}
        if acc:
            self.say("마지막 판정: e_l %s mm (σ %s)  ψ %s°  여유 %s mm"
                     % (_f(acc.get("e_l", float("nan")) * 1000, "%+.0f"),
                        _f(acc.get("sigma_e_l", float("nan")) * 1000, "%.0f"),
                        _f(acc.get("psi_deg"), "%+.2f"),
                        _f(acc.get("margin_m", float("nan")) * 1000, "%+.0f")))
        if self.sim and self.plant is not None:
            tr = self.plant.truth()
            e_l = tr.lat_m + C.CAM_TO_REF_M * math.sin(math.radians(tr.psi_deg))
            worst = max(abs(e_l),
                        abs(e_l - C.DOCK_DEPTH_M * math.tan(math.radians(tr.psi_deg))))
            self.say("진값(시뮬 채점): x %.2f m  ℓ %+.3f  ψ %+.2f°  →  e_l %+.0f mm  "
                     "평행사변형 최악 %.0f mm (허용 %.0f)  %s"
                     % (tr.x_m, tr.lat_m, tr.psi_deg, e_l * 1000, worst * 1000,
                        C.LAT_TOL_M * 1000,
                        "성공" if worst <= C.LAT_TOL_M else "실패"))
        if self.fsm.phase is Phase.DONE_UNVERIFIED:
            self.say("!! DONE_UNVERIFIED — 카메라로 확인 못 했다. **줄자로 재라**")
        if self.fsm.phase is Phase.ABORT:
            self.say("!! ABORT(%s) — 사람이 볼 것. 버스에는 stop 이 계속 나간다"
                     % (self.fsm.abort_reason.value if self.fsm.abort_reason else "?"))
        self.logger.event("dock_result", **s)


# ═══════════════════════════════════════════════════════════════════════════

def build_args(argv=None):
    p = argparse.ArgumentParser(
        description="도킹 실행 — 연속 추정 + 수렴 제어 (v3)")
    p.add_argument("--dry-run", action="store_true",
                   help="가짜 리그(카메라·자이로·CAN). 아무것도 안 보낸다")
    p.add_argument("--sim", action="store_true",
                   help="플랜트 폐루프(검출 없이 해석형 관측). 가장 빠르다")
    p.add_argument("--no-can", action="store_true", help="카메라·IMU 만 진짜로 연다")
    p.add_argument("--show", action="store_true", help="화면을 띄운다")
    p.add_argument("--record-events", action="store_true",
                   help="기록 폴더를 반드시 남긴다(기본도 남긴다)")
    p.add_argument("--no-record", action="store_true", help="아무것도 안 남긴다")
    p.add_argument("--assume-calib", action="store_true",
                   help="캘리브가 없어도 9/7 가정값으로 출발한다(97 한정·다리 제한)")
    p.add_argument("--final-anyway", action="store_true",
                   help="수용식을 못 넘어도 FINAL 로 들어간다. 결과는 무조건 DONE_UNVERIFIED")
    p.add_argument("--level", type=int, choices=(67, 97), default=67,
                   help="전진 단. 강등 상태면 무조건 97 (계약 §5.4)")
    p.add_argument("--standoff", type=float, default=None,
                   help="태그면 앞 정지 여유 [m] (기본 %.2f, 하한 %.2f). "
                        "정지점 = %.2f + 이 값. **음수 불가**"
                        % (C.STANDOFF_M, STANDOFF_MIN_M, C.CAM_TO_REF_M))
    p.add_argument("--force", action="store_true",
                   help="GLOBAL 전환 실패에도 강행")
    p.add_argument("--no-reset", action="store_true", help="hardware_reset 생략")
    p.add_argument("--auto-exposure", action="store_true")
    p.add_argument("--tag-id", type=int, default=D.TAG_ID)
    p.add_argument("--tag-size", type=float, default=D.TAG_SIZE_M)
    # 시뮬·dry-run 시작 자세
    p.add_argument("--dist", type=float, default=6.0, help="[sim/dry] 시작 거리 [m]")
    p.add_argument("--lat", type=float, default=0.8, help="[sim/dry] 시작 lateral [m]")
    p.add_argument("--psi", type=float, default=None, help="[sim/dry] 시작 heading [도]")
    p.add_argument("--seed", type=int, default=0, help="[sim] 난수 씨앗")
    return p.parse_args(argv)


async def _main(args):
    run = Dock(args)
    code = 1
    try:
        await run.open()
        phase = await run.run()
        code = 0 if phase in (Phase.DONE, Phase.DONE_UNVERIFIED) else 2
    except KeyboardInterrupt:
        run.say("")
        run.say("!! Ctrl+C — 정지")
        code = 130
    finally:
        await run.close()
    return code


def main(argv=None):
    args = build_args(argv)
    if args.level == 97:
        pass          # 강등 여부와 무관하게 97 을 원하면 그대로 (내려가는 건 항상 허용)
    signal.signal(signal.SIGINT, _sigint)
    print("")
    print("  도킹 — 허용치 lateral %.0f mm · heading %.1f° · 탑재부 깊이 %.1f m(가정) · "
          "정지점 카메라 %.2f m"
          % (C.LAT_TOL_M * 1000, C.HEAD_TOL_DEG, C.DOCK_DEPTH_M,
             C.CAM_TO_REF_M + (args.standoff if args.standoff is not None
                               else C.STANDOFF_M)))
    return asyncio.run(_main(args))


if __name__ == "__main__":
    sys.exit(main())
