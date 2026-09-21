"""내일 현장에서 쓰는 측정 도구 — **원시 기록만 하고 판정은 안 한다**.

    python tools/first_run.py all                 # 딸각: 정해진 순서로 자동 연속 (권장)
    python tools/first_run.py all --resume        # Ctrl+C·오류 뒤 다음 단계부터
    python tools/first_run.py timing              # 한 단계만
    python tools/first_run.py all --dry-run --n 2 # 장비 없이 코드 경로만 (맥에서도 됨)

순서는 plan 4-7 방문 A 그대로:

    timing → safety → mount → rotate → forward → creep → tagcut → grid → oblique → search
    (그 뒤 analyze_first_run.py 가 자동으로 돌아 캘리브 3종을 만든다)

    **timing 이 게이트다** — TXACK 첫 10건 max < 20 ms ∧ gyro gaps 0.
    미통과면 이후 단계는 원시 기록만 하고 캘리브 파일을 만들지 않는다(plan 4-2).
    **forward 까지가 필수** — 여기까지 돌면 dynamics_calib 이 보장된다(plan 4-7).

기록은 `work_dirs/first_run/<시각>_<mode>/` 에 frame/imu/can/events.jsonl + config.json.
판정(τ_eff·A·θ_min…)은 전부 `tools/analyze_first_run.py` 가 오프라인에서 한다 —
기록 시점에 판정을 박으면 그 판정이 틀렸을 때 로그가 통째로 못 쓰게 된다(plan 2-7).

안전
────────────────────────────────────────────────────────────────────────
· 시작 전 CAN 템플릿 검사(byte1/2 외 비중립 거부 — byte4 는 포크다)
· 모든 종료·Ctrl+C·예외에서 stop 프레임
· 이동 모드는 최대 시간 캡, 송신 데드맨 0.5 s
· 자이로 먼저 열고 카메라 나중 (RSUSB 규칙)
· safety 단계는 **사람이 제동 위치**에 서 있는지 확인 프롬프트가 필수다.
  포크는 **우리가 건드리지 않는다** — byte4(리프트)는 모든 템플릿에서 중립이고
  check_templates 가 그걸 검사한다. 포크를 올리거나 내리라고 시키지 않는다.
· 맥 절전(뚜껑 닫힘)이면 VM 이 멈춘다 → 시작할 때 확인 프롬프트
"""
import argparse
import asyncio
import glob
import io
import json
import math
import os
import statistics
import sys
import time
from datetime import datetime

ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
sys.path.insert(0, ROOT)
sys.path.insert(0, os.path.join(ROOT, "tools", "etc"))

from config import control as C                                   # noqa: E402
from config import detection as D                                 # noqa: E402
from src.models.control import can_tx                             # noqa: E402
from src.models.detection.detection_pose import (bearing_px_deg,   # noqa: E402
                                                 pnp2_solutions)
from src.models.detection.detection_tag import tag_edge_margin_px  # noqa: E402
from src.utils import calib as CAL                                # noqa: E402
from src.utils import timing as TM                                # noqa: E402
from src.utils.run_log import RunLogger                           # noqa: E402

WORK = os.path.join(ROOT, "work_dirs", "first_run")

#: all 이 도는 순서 (plan 4-7 방문 A). forward 까지가 필수.
ORDER = ("timing", "safety", "mount", "rotate", "forward", "creep",
         "tagcut", "grid", "oblique", "search")
REQUIRED = ("timing", "safety", "mount", "rotate", "forward")

#: 이동 명령 한 번의 상한 [s]. 어떤 모드도 이보다 길게 못 보낸다.
MAX_CMD_SEC = 6.0
#: 태그를 이만큼 연속으로 못 보면 그 단계를 중단한다 [s] (plan 4-7)
TAG_LOST_ABORT_S = 5.0
#: 단계 사이 정지 [s] — 자이로 바이어스 재추정도 여기서
STAGE_GAP_S = 2.0
#: 자동 복귀 허용 오차 [m]
RETURN_TOL_M = 0.10
#: 진입 게이트 (plan 4-2)
GATE_TXACK_MS = 20.0
#: 자동 복귀 명령 시간을 이 비율로 모자라게 잡는다 (옛 config.FWD_SAFETY — 구현 세부라 여기 둔다)
RETURN_SAFETY = 0.9
#: 짧은 명령 격자 [s] — δ_x(최소 신뢰 증분)·S(T) 표는 **길이가 여러 개**라야 나온다.
#: 죽은시간 ~1.0 s 위로 골랐다(그 아래는 아예 안 움직여 CV 가 뜻이 없다).
SHORT_CMD_S = (1.5, 2.0, 3.0)
#: tagcut 단계에서 '잘리기 직전' 으로 보는 픽셀 여유. 여기서 **재는** 값이라 config 가 아니다
#: (dynamics_calib.tag_cut.margin_px 의 기본값과 같은 60 px)
CUT_MARGIN_PX = 60.0

#: tagcut 안전 바닥 — 포크 끝이 태그면(벽)에서 이만큼 앞이면 컷 판정과 무관하게 선다 [m].
#: tagcut 은 전진을 켜 놓고 폴링하는 유일한 모드라 시간 말고 거리 backstop 이 필요하다.
TAGCUT_FLOOR_M = 0.40
#: 회전팔 A 측정 자세 — tilt 가 이보다 작으면 각도를 못 믿어 A 가 무의미해진다.
#: (tilt = 태그 법선과 카메라 광축 사이 각. 옆으로만 서고 앞을 보면 0 이다 —
#:  **옆으로 비켜서서 태그 쪽으로 몸을 틀어야** 생긴다.)
ARM_TILT_MIN_DEG = 20.0
ARM_TILT_WANT_DEG = 25.0        # 안내용 권장값(여유 5도)

#: 움직이기 시작했다고 보는 문턱. 카메라 3 cm(잡음 ~1 cm), 자이로 0.3도.
ONSET_FWD_M = 0.03
ONSET_ROT_DEG = 0.3
#: 9/7 실측 출발 지연 [s] — 현장에서 눈으로 대조하라고 옆에 같이 찍는다(값 자체는 안 쓴다).
REF_ONSET_FWD_S = 1.00
REF_ONSET_ROT_S = 0.85

#: grid 가 훑는 자리 [m] — 도킹축에서 왼쪽으로. **자리가 tilt 를 정한다**(회전이 아니라).
#: 3.5 m 기준 대략 tilt 0 / 8 / 16 / 23 / 30 도. 걸음으로 재도 될 만큼만 정확하면 된다.
GRID_LATERALS_M = (0.0, 0.5, 1.0, 1.5, 2.0)
#: 그 자리에서 차가 흔드는 각 [도]. 태그가 화면에 남을 만큼 작아야 한다.
GRID_WIGGLE_DEG = 3.0

#: safety 안전 바닥 — 포크 끝이 태그면에서 이만큼 앞이면 그 단계를 멈춘다 [m].
#: safety 는 **일부러 CAN 을 끊고 4 s 를 기어가는** 시험이라 제일 위험하다.
#: 시행마다 복귀하지만 복귀가 모자랄 수 있으니 바닥을 따로 둔다.
SAFETY_FLOOR_M = 0.80
#: tagcut 한 번에 갈 수 있는 최대 주행량 [m] (정상 경로는 0.2 m 쯤이다).
TAGCUT_MAX_TRAVEL_M = 1.20

# 단계별 시작 거리 [m]. **태그 컷(≈3.3 m, 카메라가 태그보다 1.1 m 아래)** 위에 남아야
# 한 번의 주행을 끝까지 카메라로 볼 수 있다 — dry-run 에서 3.5 m 시작이 주행 도중
# 태그를 잃는 것을 보고 잡았다(67·4 s ≈ 1.25 m, 97·4 s ≈ 0.6 m).
#: 단계별 시작 거리 [m] — 두 가지가 싸운다.
#:  · 멀수록 **안전**(벽까지 여유). safety 는 일부러 제어를 끊고 기어가는 시험이라 제일 멀리.
#:  · 가까울수록 **정밀**. 8 m 는 lateral 잡음 60~145 mm(9/7), 태그가 작아 깊이 역산도 흐리다.
#:    정지 관성 12~14 cm 를 재려면 그보다 σ 가 작아야 해서 forward 는 5.5 m 가 한계.
#:    회전팔 A 는 효과가 (1 + A·cosβ/d) 라 **멀수록 신호가 준다** → rotate 는 3.5·6 m.
START_M = {"safety": 8.0, "forward": 5.5, "creep": 4.5, "oblique": 5.0}
START_M_DEFAULT = 3.5


#: Ctrl+C 가 눌린 순간 **먼저 차를 세우기** 위한 현재 세션 손잡이.
#: 파이썬이 예외를 풀어내는 데 시간이 걸려도, 핸들러 안에서 current_movement 를
#: 바로 stop 으로 바꾸면 다른 팀 TX 루프(10 ms)가 그 다음 프레임부터 stop 을 보낸다.
_ACTIVE = {"session": None}


def _sigint(signum, frame):
    s = _ACTIVE.get("session")
    if s is not None:
        try:
            if s.tx is not None:
                s.tx.stop("SIGINT")
            if s.ctrl is not None:
                s.ctrl.current_movement = "stop"
        except Exception:
            pass
    raise KeyboardInterrupt


def _now_str():
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def _cf():
    """control_forklift_v2 모듈. canlib 이 없는 개발 기계에서는 None (dry-run 용)."""
    try:
        from src.models.control import control_forklift_v2 as CF
        return CF
    except Exception:
        return None


#: canlib 이 없을 때 쓰는 CAN ID (control_forklift_v2 와 같은 값 — 기록·차단 시험용)
CAN_IDS = {"heartbeat": 0x764, "movement": 0x01E3, "control": 0x02E3}


def _median(v):
    v = [x for x in v if x is not None and x == x]
    return statistics.median(v) if v else None


# ═══════════════════════════════════════════════════════════════════════════
# 세션 — 장비·기록·안전 한 벌
# ═══════════════════════════════════════════════════════════════════════════

class Session:
    def __init__(self, args, mode, root=None):
        self.args = args
        self.mode = mode
        self.gaps_at_open = 0            # 기동이 끝난 시점의 누적 gaps (차분 기준점)
        self.dry = bool(args.dry_run)
        self.root = root
        self.dir = os.path.join(root or WORK, "%s_%s" % (_now_str(), mode))
        self.log_lines = []
        self.logger = RunLogger(self.dir)
        self.timing = TM.TimingSession(log=self.say)
        self.loop_tick = TM.LoopTick()
        self.pipe = None
        self.gyro = None
        self.ctrl = None
        self.tx = None
        self.plant = None
        self.cam = None                 # dry-run 가짜 카메라
        self.tasks = []
        self.calibs = None
        self.gate_passed = None         # timing 게이트 결과 (None = 아직)
        self.tag_id = args.tag_id
        self.tag_size = args.tag_size
        # 시작 거리: --start-m 을 주면 그 값, 아니면 단계별 기본값
        self.start_m = (args.start_m if args.start_m
                        else START_M.get(mode, START_M_DEFAULT))
        self.frames_seen = 0
        self.last_seen_t = None
        self.last_doc = None
        self.t_decide = None
        self.stage = mode
        self.block = None               # 지금 무슨 블록을 재고 있나 (분석이 이걸로 묶는다)
        self.fake = self.dry

    # ── 로그 ───────────────────────────────────────────────────────────────
    def say(self, text):
        line = "  %s" % text
        print(line, flush=True)
        self.log_lines.append(line)

    def event(self, kind, **kw):
        self.logger.event(kind, stage=self.stage, **kw)

    # ── 시계 ───────────────────────────────────────────────────────────────
    def now(self):
        """세션 시계. dry-run 은 가상 시계(플랜트)라 실시간 대기를 안 한다."""
        return self.plant.t if (self.dry and self.plant is not None) else time.time()

    def dur(self, sec):
        """관측만 하는 구간의 길이. dry-run 에서는 줄여서 빨리 돈다."""
        return float(sec) * (0.05 if self.dry else 1.0)

    def settle_sec(self):
        """정지 명령 뒤 실제로 멎을 때까지 기다리는 시간 [s].

        **이건 dur() 로 줄이지 않는다** — 코스팅(τ_eff ≈ 0.5 s)이 끝난 뒤의 프레임이
        있어야 분석이 D_obs 를 잴 수 있다. dry-run 에서도 가상 2.5 s 는 준다.
        """
        return 2.5 if self.dry else 3.0

    # ── 열기/닫기 ──────────────────────────────────────────────────────────
    async def open(self):
        self.say("기록 폴더 %s" % self.dir)
        self.calibs = CAL.load_all()
        dec = CAL.gate(calibs=self.calibs)
        for k in CAL.KINDS:
            self.say("캘리브 %s" % self.calibs[k].summary())
        if not dec.allow:
            self.say("(캘리브가 아직 없다 — 이 도구가 그 캘리브를 **만드는** 도구다. 계속한다)")
        self.say("DOCK_DEPTH_M=%.1f m 은 미측정 보수 가정값 — HEAD_TOL_DEG=%.1f 도가 여기서 나왔다"
                 % (C.DOCK_DEPTH_M, C.HEAD_TOL_DEG))

        # CAN 템플릿 검사 — 보내기 전에 무조건
        try:
            tpl = can_tx.check_templates(log=self.say, forward_slow_expect=C.FORWARD_SLOW)
            self.event("can_templates", templates=tpl)
        except ImportError as exc:
            if not self.dry:
                raise
            self.say("(canlib 없음 — 템플릿 확인 생략: %s)" % exc)
        except SystemExit:
            raise

        if self.dry:
            await self._open_fake()
        else:
            await self._open_real()

        self.logger.snapshot(self.calibs, argv=sys.argv,
                             extra={"first_run": {"mode": self.mode, "dry_run": self.dry,
                                                  "fake": self.fake, "n": self.args.n,
                                                  "tag_id": self.tag_id,
                                                  "tag_size": self.tag_size}})
        _ACTIVE["session"] = self
        self.event("session_open", dry_run=self.dry, fake=self.fake,
                   domain_color=self.timing.domain_color,
                   domain_gyro=self.timing.domain_gyro)

    async def _open_fake(self):
        from fake_rig import FakeCamera, FakeController, FakeGyro, FakePlant
        from src.models import TagPipeline
        from src.models.detection.image import intrinsics_from_ref
        self.say("DRY-RUN — 가짜 카메라·자이로·CAN. 아무것도 안 보낸다")
        intr = intrinsics_from_ref((480, 640))      # 가짜는 작게 — 검출은 진짜로 돈다
        _lat, _psi = _dry_pose(self.args, self.mode, self.start_m)
        self.plant = FakePlant(forward=self.start_m, lateral=_lat,
                               heading_deg=_psi, vertical=-1.10)
        # vertical < 0 = 카메라가 태그보다 낮다 (계약 §1.1). 이 리그가 그렇다.
        # 예전 +1.10 은 fake_rig 렌더가 180° 뒤집혀 있던 때의 보상값이었다(E6, 9/21 고침)
        self.cam = FakeCamera(self.plant, intr, self.tag_size, tag_id=self.tag_id,
                              timing=self.timing)
        self.pipe = TagPipeline(frames=self.cam, intrinsics=intr,
                                tag_size=self.tag_size, label="fake")
        self.gyro = FakeGyro(self.plant).enable_raw()
        self.timing.note_domain("gyro", self.gyro.domain)
        self.ctrl = FakeController(self.plant, logger=self.logger)
        self.tx = can_tx.SafeCanTx(self.ctrl, logger=self.logger, log=self.say,
                                   clock=self.now)

    async def _open_real(self):
        from src.models import TagPipeline
        from src.utils.camera import CameraSettings
        from src.utils.imu_yaw import GyroYaw
        # 0) 세션 시작 hardware_reset (plan 4-1: 71.6 분 랩 역행 대응)
        if not self.args.no_reset:
            TM.hardware_reset(log=self.say, allow_vm=getattr(self.args, 'reset', False))
        # 1) 자이로 먼저 (RSUSB: 먼저 연 쪽이 IMU 를 갖는다)
        self.gyro = GyroYaw().start().enable_raw()
        self.say("자이로 열림. %.1fs 정지 보정 — 차를 세워 둘 것..." % 2.0)
        rep = await asyncio.to_thread(self.gyro.calibrate)
        self.say("   축 %s, 잡음 %.3f 도/s, 드리프트 %.2f 도/분%s"
                 % (rep["axis_src"], rep["noise_dps"], rep["drift_dpm"],
                    "  !! 보정 중 움직임" if rep["moving"] else ""))
        self.event("gyro_calibrate", **{k: v for k, v in rep.items() if k != "accel_mean"})
        # 2) 카메라 — queue=1 + global_time (CameraSettings.docking)
        tune = CameraSettings.docking()
        if self.args.auto_exposure:
            tune.enable_auto_exposure = True
            tune.exposure = None
            tune.gain = None
        self.pipe = TagPipeline.from_realsense(self.tag_size, tune=tune,
                                               timing=self.timing, label="first_run")
        self.say("카메라 열림 (queue=1, global_time). 태그 %d, %.3f m"
                 % (self.tag_id, self.tag_size))
        self.ctrl = None
        if self.args.no_can:
            from fake_rig import FakeController
            self.ctrl = FakeController(None, logger=self.logger)
            self.say("--no-can — CAN 을 안 연다(카메라·IMU 만 기록)")
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
        self.tx.attach()
        self.tasks.append(asyncio.create_task(self.tx.watchdog()))
        await asyncio.sleep(0.5)
        # 3) GLOBAL 전환 대기 — 컬러·자이로 둘 다 (plan 4-1)
        await self.wait_global()

    async def wait_global(self):
        t0 = time.time()
        while time.time() - t0 < TM.GLOBAL_WAIT_S:
            await self.pump()
            self.timing.note_domain("gyro", getattr(self.gyro, "domain", None))
            if self.timing.global_ready:
                self.say("타임스탬프 GLOBAL 확인 (컬러 %.1fs / 자이로 %.1fs)"
                         % (self.timing.t_global_color - self.timing.t_open,
                            self.timing.t_global_gyro - self.timing.t_open))
                self.event("global_time", ok=True,
                           color_s=self.timing.t_global_color - self.timing.t_open,
                           gyro_s=self.timing.t_global_gyro - self.timing.t_open)
                # 여기까지가 **기동**이다. 스트림이 붙는 동안 자이로 유실 1회는 흔하고
                # 무해하므로, 게이트·회전 판정은 이 시점 이후의 **증가분**만 본다.
                self.gaps_at_open = self._gaps_now()
                if self.gaps_at_open:
                    self.say("  (기동 중 자이로 유실 %d회 — 무해. 이후 증가분만 센다)"
                             % self.gaps_at_open)
                self.event("gaps_at_open", n=self.gaps_at_open)
                return True
        miss = self.timing.global_missing()
        self.say("!! GLOBAL 전환 실패 %s — **실주행 거부**. 기록은 계속하되 캘리브는 못 만든다"
                 % miss)
        self.event("global_time", ok=False, missing=miss)
        self.gate_passed = False
        if not self.args.force:
            raise SystemExit("타임스탬프 도메인이 GLOBAL 이 아니다 — --force 로만 강행")
        return False

    async def close(self):
        # **stop 이 먼저다.** 여기는 await 없이 동기로 — 루프가 취소된 뒤에도 값은 바뀐다.
        try:
            if self.tx is not None:
                self.tx.stop("session_close")
            if self.ctrl is not None:
                self.ctrl.current_movement = "stop"
        except Exception:
            pass
        try:
            if not self.dry:
                await asyncio.sleep(0.2)      # stop 프레임이 몇 번 나갈 시간
        except (asyncio.CancelledError, Exception):
            time.sleep(0.2 if not self.dry else 0.0)
        for t in self.tasks:
            t.cancel()
        try:
            if self.gyro is not None:
                self.logger.imu(self.gyro.drain_raw())
                self.gyro.close()
        except Exception:
            pass
        try:
            if self.ctrl is not None and not self.dry and not self.args.no_can:
                self.ctrl.is_running = False
                self.ctrl.disconnect_can()
        except Exception:
            pass
        try:
            if self.pipe is not None:
                self.pipe.close()
        except Exception:
            pass
        self.event("session_close", frames=self.frames_seen,
                   timing=self.timing.summary(), loop_tick=self.loop_tick.summary(),
                   can=(self.tx.stats() if self.tx is not None else None))
        self.logger.close()
        _ACTIVE["session"] = None
        self.say("기록: %s  (%s)" % (self.dir, self.logger.summary()))

    # ── 프레임 ─────────────────────────────────────────────────────────────
    async def pump(self, note=None):
        """프레임 한 장을 읽어 기록한다. 모든 대기는 이걸 돌리며 흐른다."""
        self.loop_tick.tick()
        if self.tx is not None:
            self.tx.feed()                      # 데드맨: 루프가 살아 있다
        if self.dry:
            item = self.cam.next_frame()
        else:
            item = await asyncio.to_thread(next, self.pipe.frames, None)
        if item is None:
            return None
        i, ts, img = item
        res = self.pipe.process(img, index=i, timestamp=ts)
        self.frames_seen += 1
        self._log_frame(res, note)
        if self.gyro is not None and self.frames_seen % 10 == 0:
            self.logger.imu(self.gyro.drain_raw())
        return res

    def _log_frame(self, res, note=None):
        tid = self.tag_id
        det = None
        for d in getattr(res, "detections", []):
            if int(getattr(d, "tag_id", -1)) == int(tid):
                det = d
                break
        doc = res.docking.get(tid)
        q = res.quality.get(tid, {}) or {}
        row = {"i": res.index, "stage": self.stage, "block": self.block, "note": note,
               "movement": (self.tx.movement if self.tx is not None else None),
               "t_cmd_set": (self.tx.t_cmd_set if self.tx is not None else None),
               "t_decide": self.t_decide,
               "gyro_deg": (self.gyro.angle_deg if self.gyro is not None else None),
               "gyro_dps": (self.gyro.rate_dps if self.gyro is not None else None),
               "gyro_gaps": (self.gyro.stats().get("gaps") if self.gyro is not None else None),
               "seen": det is not None}
        if self.tx is not None and self.tx.probe is not None and self.tx.probe.tx_marks:
            row["t_cmd_tx"] = self.tx.probe.tx_marks[-1][0]
        if det is not None:
            shape = getattr(res.image, "shape", (0, 0))
            row.update({"tag_px": q.get("tag_px"),
                        "margin_px": tag_edge_margin_px(det, shape),
                        "beta_px_deg": bearing_px_deg(det, res.intrinsics),
                        "center_px": [float(x) for x in det.center],
                        "decision_margin": float(getattr(det, "decision_margin", 0.0)),
                        "hamming": int(getattr(det, "hamming", 0)),
                        "reproj_rms_px": q.get("reproj_rms_px"),
                        "quality_ok": q.get("ok"),
                        "pnp2": pnp2_solutions(det, res.intrinsics, self.tag_size)})
            self.last_seen_t = self.now()
        if doc is not None:
            row.update({"lateral": doc["lateral"], "forward": doc["forward"],
                        "vertical": doc["vertical"], "heading_deg": doc["heading_deg"],
                        "tilt_deg": doc["tilt_deg"], "distance": doc["distance"],
                        "heading_sigma_deg": doc.get("heading_sigma_deg")})
            self.last_doc = doc
        stamps = getattr(res, "stamps", None)
        if stamps is not None and stamps.row_px is None and det is not None:
            stamps.row_px = float(det.center[1])      # 태그 중심 행 (T_line 보정용)
        self.logger.frame(stamps=stamps, **row)

    async def wait(self, sec, note=None):
        """sec 초 동안 프레임을 읽으며 기다린다(실시간이면 카메라가 박자를 준다)."""
        t0 = self.now()
        while self.now() - t0 < float(sec):
            r = await self.pump(note)
            if r is None:
                break
            if not self.dry and self.pipe is None:
                break
        return self.now() - t0

    def tag_lost(self):
        if self.last_seen_t is None:
            return False
        return (self.now() - self.last_seen_t) > TAG_LOST_ABORT_S

    # ── 명령 ───────────────────────────────────────────────────────────────
    def cmd(self, movement, why=""):
        self.t_decide = self.now()
        if self.tx is None:
            return None
        return self.tx.set_movement(movement, why=why)

    def stop(self, why="stop"):
        return self.cmd("stop", why)

    def _gaps_now(self):
        """자이로 누적 유실 구간 수. 비교는 **차분으로만** 할 것."""
        try:
            return int((self.gyro.stats() or {}).get("gaps") or 0)
        except Exception:
            return 0

    def _last_tx_t(self):
        """가장 최근에 **CAN 버스로 나간** 시각. 없으면 None(그러면 결정 시각을 쓴다)."""
        try:
            if self.tx is not None and self.tx.probe is not None and self.tx.probe.tx_marks:
                return float(self.tx.probe.tx_marks[-1][0])
        except Exception:
            pass
        return None

    def _say_onset(self, what, onset_s, ref_s, lat_ms):
        """현장에서 눈으로 확인하라고 두 값을 9/7 실측과 나란히 찍는다."""
        flag = ""
        if ref_s and abs(onset_s - ref_s) > 0.35 * max(ref_s, 1e-6):
            flag = "   ← 9/7 과 30% 넘게 다르다. 확인할 것"
        self.say("    명령→움직임 %s: **%.3f s** (9/7 %.2f s) | 명령 지연 %.1f ms%s"
                 % (what, onset_s, ref_s, lat_ms, flag))

    async def hold(self, movement, sec, why="", settle=1.5):
        """movement 를 sec 초 보내고 stop. 반환 = 기록용 dict."""
        sec = max(0.0, min(MAX_CMD_SEC, float(sec)))
        t_set = self.cmd(movement, why)
        self.event("cmd_start", movement=movement, sec=sec, why=why, t_cmd_set=t_set,
                   forward=(self.last_doc or {}).get("forward"),
                   lateral=(self.last_doc or {}).get("lateral"),
                   heading_deg=(self.last_doc or {}).get("heading_deg"),
                   gyro_deg=(self.gyro.angle_deg if self.gyro else None))
        f0 = (self.last_doc or {}).get("forward")
        g0 = self.gyro.angle_deg if self.gyro else None
        # **출발 지연을 현장에서 바로 본다**(분석을 기다리지 않는다).
        # 원점은 명령이 CAN 에 나간 시각(t_tx). 없으면 결정 시각(t_set).
        t_org = self._last_tx_t() or t_set
        onset, onset_by = None, None
        t_deadline = self.now() + sec
        while self.now() < t_deadline:
            await self.pump(movement)
            if onset is None:
                d = (self.last_doc or {}).get("forward")
                if f0 is not None and d is not None and abs(float(d) - float(f0)) > ONSET_FWD_M:
                    onset, onset_by = self.now() - t_org, "camera"
                elif self.gyro is not None and g0 is not None and \
                        abs(self.gyro.angle_deg - g0) > ONSET_ROT_DEG:
                    onset, onset_by = self.now() - t_org, "gyro"
        f_at_stop = (self.last_doc or {}).get("forward")
        t_stop = self.now()
        self.stop("end of %s" % movement)
        t_tx = None
        if self.tx is not None and self.tx.probe is not None and self.tx.probe.tx_marks:
            t_tx = self.tx.probe.tx_marks[-1][0]
        await self.wait(settle, note="settle")
        self.event("cmd_end", movement=movement, sec=sec,
                   forward_at_stop=f_at_stop, t_stop=t_stop, t_cmd_tx=t_tx,
                   forward_after=(self.last_doc or {}).get("forward"),
                   gyro_deg=(self.gyro.angle_deg if self.gyro else None),
                   onset_s=onset, onset_by=onset_by, t_onset_org=t_org,
                   cmd_latency_ms=((t_org - t_set) * 1000.0) if t_org else None)
        if onset is not None:
            self._say_onset("전진", onset, REF_ONSET_FWD_S, (t_org - t_set) * 1000.0)
        elif not self.dry:
            self.say("    !! %.1fs 안에 움직임을 못 봤다 — 차가 안 갔거나 문턱(%.0f mm) 미만"
                     % (sec, ONSET_FWD_M * 1000))
        return {"movement": movement, "sec": sec, "t_stop": t_stop,
                "forward_at_stop": f_at_stop, "onset_s": onset,
                "forward_after": (self.last_doc or {}).get("forward")}

    async def stage_gap(self, why=""):
        """단계 사이 2 s 정지 + 자이로 바이어스 재추정."""
        self.stop("stage gap")
        await self.wait(self.dur(STAGE_GAP_S), note="gap")
        if self.gyro is not None and hasattr(self.gyro, "calibrate") and not self.dry:
            try:
                rep = await asyncio.to_thread(self.gyro.calibrate, 1.0)
                self.event("gyro_rebias", why=why,
                           bias_dps=rep["bias_dps"], moving=rep["moving"])
            except Exception as exc:
                self.event("gyro_rebias", why=why, error=str(exc))

    # ── 측정 ───────────────────────────────────────────────────────────────
    async def measure(self, n=30, note="measure"):
        """정지 상태 n 프레임 중앙값. 태그가 안 보이면 None."""
        lat, fwd, head, tilt, beta, marg = [], [], [], [], [], []
        for _ in range(int(n) * 3):
            res = await self.pump(note)
            if res is None:
                break
            d = res.docking.get(self.tag_id)
            if d is not None:
                lat.append(d["lateral"])
                fwd.append(d["forward"])
                head.append(d["heading_deg"])
                tilt.append(d["tilt_deg"])
                for det in res.detections:
                    if int(det.tag_id) == int(self.tag_id):
                        beta.append(bearing_px_deg(det, res.intrinsics))
                        marg.append(tag_edge_margin_px(det, res.image.shape))
            if len(fwd) >= n:
                break
        if not fwd:
            return None
        out = {"n": len(fwd), "lateral": _median(lat), "forward": _median(fwd),
               "heading_deg": _median(head), "tilt_deg": _median(tilt),
               "beta_px_deg": _median(beta), "margin_px": _median(marg),
               "t": self.now()}
        self.event("measure", **out)
        return out

    # ── 자동 복귀 ──────────────────────────────────────────────────────────
    async def return_to(self, target_m, tries=8):
        """카메라 forward 를 보며 시작 거리로 되돌아간다(벽에서 멀어지는 쪽이 안전)."""
        for k in range(tries):
            m = await self.measure(10, note="return")
            if m is None:
                self.event("return_abort", why="태그 안 보임")
                return False
            err = m["forward"] - float(target_m)
            if abs(err) <= RETURN_TOL_M:
                self.event("return_ok", forward=m["forward"], target=target_m, tries=k)
                return True
            # 남은 거리 / 정속(0.28 m/s) — 모자라게 간다(RETURN_SAFETY)
            sec = min(MAX_CMD_SEC, max(1.2, abs(err) / 0.28 * RETURN_SAFETY + 1.0))
            mv = "backward" if err < 0 else "forward"
            self.event("return_step", err_m=err, movement=mv, sec=sec)
            await self.hold(mv, sec, why="자동 복귀 %.2f m" % err)
        self.event("return_giveup", target=target_m)
        return False

    async def return_heading(self, target_deg=0.0, tries=6):
        """자이로로 원래 heading 으로 돌아온다."""
        if self.gyro is None:
            return False
        for k in range(tries):
            err = self.gyro.angle_deg - float(target_deg)
            if abs(err) <= 1.0:
                self.event("return_heading_ok", gyro_deg=self.gyro.angle_deg, tries=k)
                return True
            await self.rotate_closed(-err, why="heading 복귀")
        return False

    # ── 회전(측정용 폐루프) ────────────────────────────────────────────────
    async def rotate_closed(self, deg, why="", lead_deg=0.0):
        """자이로를 보며 deg 만큼 돈다. **리드 0** — 오버슈트를 그대로 재려는 것이다.

        (실행용 정지 규칙 θ_rem ≤ τ_r·ω 는 G_code-B 다. 여기서는 그 τ_r 을 **재는** 중.)
        """
        deg = float(deg)
        if abs(deg) < 0.2 or self.gyro is None:
            return None
        mv = "rotate_ccw" if deg > 0 else "rotate_cw"
        start = self.gyro.angle_deg
        cap = min(30.0, 2.0 * (abs(deg) / 8.0 + 1.5))      # 시간 캡 = 예상 ×2
        t_set = self.cmd(mv, why)
        self.event("rotate_start", target_deg=deg, movement=mv, why=why,
                   gyro_deg=start, t_cmd_set=t_set, cap_s=cap)
        t0 = self.now()
        t_org = self._last_tx_t() or t_set
        gaps0 = self._gaps_now()          # 이 회전 시작 시점의 누적 gaps
        onset = None
        reason = "goal"
        while True:
            await self.pump(mv)
            prog = (self.gyro.angle_deg - start) * (1.0 if deg > 0 else -1.0)
            if onset is None and abs(self.gyro.angle_deg - start) > ONSET_ROT_DEG:
                onset = self.now() - t_org
            if prog >= abs(deg) - abs(lead_deg):
                break
            if self.now() - t0 > cap:
                reason = "cap"
                break
            if not self.gyro.alive:
                reason = "gyro-dead"
                break
            # **이 회전 동안 늘어난** gaps 만 본다. 누적값을 보면 세션 초반에 한 번
            # 튄 1회가 래치돼 이후 모든 회전이 33 ms 만에 죽는다
            # (2026-09-21 현장: ±12° 3회가 전부 turned 0.01° 로 끝났다).
            # rot_control.py 는 원래 gaps_before 차분을 본다 — 같은 방식으로 맞춘다.
            if not self.dry and (self._gaps_now() - gaps0) > 0:
                reason = "gyro-gaps"
                break
        t_stop = self.now()
        self.stop("rotate end")
        at_stop = self.gyro.angle_deg
        await self.wait(self.dur(2.0), note="rot settle")
        turned = self.gyro.angle_deg - start
        out = {"target_deg": deg, "turned_deg": turned,
               "at_stop_deg": at_stop - start, "coast_deg": self.gyro.angle_deg - at_stop,
               "t_cmd": t_set, "t_cmd_tx": t_org, "t_stop": t_stop,
               "elapsed_s": t_stop - t0, "onset_s": onset,
               "cmd_latency_ms": ((t_org - t_set) * 1000.0) if t_org else None,
               "reason": reason, "dir": "L" if deg > 0 else "R"}
        self.event("rotate_end", **out)
        if onset is not None:
            self._say_onset("회전", onset, REF_ONSET_ROT_S, (t_org - t_set) * 1000.0)
        return out

    # ── 사람 ───────────────────────────────────────────────────────────────
    async def ask(self, text, key=None):
        """한 줄 프롬프트. --yes/dry-run 이면 자동으로 넘어간다(그 사실도 기록)."""
        if self.args.yes or self.dry:
            self.say("[자동] %s" % text)
            self.event("prompt", text=text, answer="auto", key=key)
            return ""
        self.say("")
        ans = await asyncio.to_thread(input, "  >> %s  (Enter) " % text)
        self.event("prompt", text=text, answer=ans, key=key)
        return ans

    async def ask_value(self, text, key, default=None):
        """숫자를 받아 적는다(줄자 입력). 빈 줄이면 default."""
        if self.args.yes or self.dry:
            self.event("value", key=key, value=default, source="auto")
            return default
        ans = await asyncio.to_thread(input, "  >> %s [%s] " % (text, default))
        try:
            val = float(ans.strip())
        except Exception:
            val = default
        self.event("value", key=key, value=val, raw=ans, source="human")
        return val


# ═══════════════════════════════════════════════════════════════════════════
# 단계들
# ═══════════════════════════════════════════════════════════════════════════

async def mode_timing(s):
    """(0) 타이밍 게이트 30 min — L·ε·Δ_FS·TXACK·gaps. **이게 통과해야 캘리브를 만든다.**"""
    s.say("=== timing: 타이밍 게이트 ===")
    await s.ask("태그가 화면 **중앙 높이**에 보이게 차를 세우고 Enter", key="row_center")
    s.block = "still_center"
    s.event("timing_block", block=s.block, bag=False)
    await s.wait(s.dur(60.0), note="still_center")

    # 8 도/s 제자리회전 ×3 — ψ_cam(t_capture) vs ψ_gyro 상호상관용 (plan 4-1 ε)
    for k in range(3):
        s.block = "rot_center_%d" % k
        s.event("timing_block", block=s.block, bag=False, row="center")
        await s.rotate_closed(+12.0 if k % 2 == 0 else -12.0, why="ε 측정 회전")
    await s.return_heading(0.0)

    if not _want(s, "upper"):
        # center 만 돌아도 **게이트는 찍는다** — 이게 이 단계의 통과/불통과 신호다.
        # (upper 는 T_line 을 하나 더 얻는 것뿐이고, 게이트 자체는 여기서 판정된다.)
        _timing_gate(s)
        return {"gate_passed": s.gate_passed, "parts": "center only"}
    await s.ask("태그가 화면 **위쪽 행**에 오게 (차를 태그 쪽으로 조금 붙여) Enter",
                key="row_upper")
    s.block = "still_upper"
    s.event("timing_block", block=s.block, bag=False)
    await s.wait(s.dur(60.0), note="still_upper")
    for k in range(3):
        s.block = "rot_upper_%d" % k
        s.event("timing_block", block=s.block, bag=False, row="upper")
        await s.rotate_closed(+12.0 if k % 2 == 0 else -12.0, why="ε 측정 회전(윗행)")
    await s.return_heading(0.0)

    # bag on/off 비교 — arrival p99 차 < 30 ms 면 bag 을 켠 채로 전 블록을 돌아도 된다
    if not s.dry and not s.args.no_bag:
        bag = os.path.join(s.dir, "timing_bag.bag")
        s.block = "still_bag"
        s.event("timing_block", block=s.block, bag=True, path=bag)
        if await s.reopen_camera(record=bag):
            await s.wait(60.0, note="still_bag")
            await s.reopen_camera(record=None)

    _timing_gate(s)
    return {"gate_passed": s.gate_passed}


def _timing_gate(s):
    """타이밍 게이트 판정·출력. center 만 돌아도 여기서 찍는다."""
    summary = s.timing.summary()
    can = s.tx.stats() if s.tx is not None else {}
    # **단계가 시작된 뒤** 늘어난 gaps 만 센다. 스트림 기동 직후 1회는 흔하고 무해한데,
    # 누적값을 쓰면 그 1회로 게이트가 영영 미통과가 된다(2026-09-21 현장).
    gaps = max(0, s._gaps_now() - getattr(s, "gaps_at_open", 0))
    first10 = can.get("txack_first10_max_ms")
    passed = (gaps == 0) and (first10 is not None and first10 < GATE_TXACK_MS)
    if first10 is None:
        s.say("!! TXACK 를 못 받았다 — 게이트를 '미확인' 으로 둔다(분석에서 판정)")
        passed = None if s.fake else False        # 가짜 장비에서는 판정 자체를 안 한다
    s.gate_passed = passed
    s.block = None
    s.event("gate", passed=passed, txack_first10_max_ms=first10,
            gyro_gaps=gaps, timing=summary, can=can,
            loop_tick=s.loop_tick.summary(),
            rule="TXACK 첫 10건 max < %.0f ms ∧ gyro gaps = 0" % GATE_TXACK_MS)
    s.say("게이트: TXACK 첫10 max %s ms / gaps %s → %s"
          % (first10, gaps,
             "통과" if passed else ("미확인(가짜 장비)" if passed is None
                                 else "미통과(이후 단계는 원시 기록만)")))
    s.say("L 중앙값 %s ms / p99 %s ms" % (summary["L_ms_median"], summary["L_ms_p99"]))
    return passed


async def mode_safety(s):
    """(1) 안전 45 min — 차단·복구·emergency·데드맨·kill -STOP·조이스틱·이중 송신원."""
    s.say("=== safety: CAN 안전 시험 ===")
    s.say("!! 이 단계는 차가 **실제로 기어간다**. 아래를 확인하고 진행할 것")
    await s.ask("사람이 **제동(비상정지) 위치**에 서 있는가? 확인하고 Enter", key="brake_ready")
    await s.ask("차 앞 5 m 가 비어 있는가? 확인하고 Enter", key="path_clear")

    probe = s.tx.probe if s.tx is not None else None
    n = max(1, min(3, s.args.n))

    # ① listen-only 60 s — 리모컨 수신기가 살아 있나(FM11)
    # **try/finally 로 감싼다.** 예외나 Ctrl+C 가 이 구간에서 나면 차단이 안 풀리고,
    # 그 뒤 데드맨·SIGINT·close 의 stop 프레임이 전부 프로브에서 버려진다
    # (차는 마지막 forward_slow 로 계속 간다). ChannelProbe 가 stop 템플릿만은
    # 절대 안 버리도록 고쳤지만, 차단을 푸는 책임은 여기 있다.
    s.event("safety_block", block="listen_only")
    try:
        if probe is not None:
            probe.blocked_all = True
        await s.wait(s.dur(60.0), note="listen_only")
    finally:
        if probe is not None:
            probe.blocked = set()
            probe.blocked_all = False
        s.stop("listen_only 끝")
    s.event("safety_block", block="listen_only_end",
            rx_note="can.jsonl 의 dir=rx 줄이 차량/리모컨 프레임이다")

    # ② 97 크립 중 차단 4종 × n (plan 4-4)
    CF = _cf()
    hb = CF.HEARTBEAT_ID if CF else CAN_IDS["heartbeat"]
    mov = CF.CAN_MOVEMENT_ID if CF else CAN_IDS["movement"]
    ctl = CF.CAN_CONTROL_ID if CF else CAN_IDS["control"]
    blocks = [("heartbeat", {hb}), ("movement_0x1E3", {mov}),
              ("control_0x2E3", {ctl}), ("all", None)]
    # 이 단계는 시행마다 6 s(2 s 크립 + 4 s 차단) 를 앞으로 간다. 복귀가 없으면
    # 12 시행이 누적돼 벽까지 간다 → 시행마다 시작 자리로 되돌리고, 그래도 모자라면
    # 포크 끝 기준 바닥(SAFETY_FLOOR_M)에서 단계를 멈춘다.
    start = (await s.measure(30, note="safety_start") or {}).get("forward", s.start_m)
    floor = C.CAM_TO_REF_M + SAFETY_FLOOR_M
    s.say("시작 %.2f m · 바닥 %.2f m (포크 끝이 태그면 %.2f m 앞)"
          % (start, floor, SAFETY_FLOOR_M))
    for name, ids in blocks:
        for k in range(n):
            now_m = (s.last_doc or {}).get("forward")
            if now_m is not None and float(now_m) <= floor:
                s.say("!! %.2f m — 안전 바닥 %.2f m 이하다. safety 를 여기서 멈춘다"
                      % (float(now_m), floor))
                s.event("safety_floor", forward=float(now_m), floor=floor)
                s.stop("안전 바닥")
                return {"aborted": "floor", "forward": float(now_m)}
            s.event("safety_block", block=name, trial=k, phase="start")
            s.cmd("forward_slow", "차단 시험 크립")
            try:
                await s.wait(s.dur(2.0), note="creep")
                if probe is not None:
                    if ids is None:
                        probe.blocked_all = True
                    else:
                        probe.blocked = set(ids)
                t_block = s.now()
                s.event("safety_cut", block=name, trial=k, t_cut=t_block)
                await s.wait(s.dur(4.0), note="cut_%s" % name)
            finally:
                # 어느 길로 빠져나가든 **차단부터 푼다** — 그 다음에 stop.
                if probe is not None:
                    probe.blocked = set()
                    probe.blocked_all = False
                s.stop("차단 시험 끝")
            await s.wait(s.dur(3.0), note="cut_recover")
            s.event("safety_block", block=name, trial=k, phase="end",
                    forward=(s.last_doc or {}).get("forward"))
            await s.return_to(start)          # 누적 금지 — 매번 시작 자리로
            if s.tag_lost():
                s.say("!! 태그를 5 s 넘게 못 봤다 — 이 단계를 중단한다")
                return {"aborted": "tag_lost"}
        await s.ask("차단(%s) 뒤 차가 어떻게 됐나 눈으로 확인했으면 Enter" % name,
                    key="after_%s" % name)

    # ③ 복구 순서 — 어디서 살아나는지가 래치 유형을 가른다
    await s.ask("복구 ①: 프레임만 재개해서 움직이나? 확인 후 Enter", key="recover_frames")
    s.cmd("forward_slow", "복구 확인")
    await s.wait(s.dur(3.0), note="recover_frames")
    s.stop()
    await s.ask("복구 ②: mode+heartbeat 버스트가 필요한가? 확인 후 Enter", key="recover_burst")
    if s.ctrl is not None and hasattr(s.ctrl, "_startup_burst_sequence") and not s.dry:
        try:
            await s.ctrl._startup_burst_sequence()
        except Exception as exc:
            s.event("recover_burst_error", error=str(exc))
    await s.ask("복구 ③: 키 사이클(전원)까지 필요했나? 답을 적었으면 Enter", key="recover_key")

    # ④ emergency 0x80 — **정지 상태에서 1회만**
    await s.ask("emergency(0x80) 를 **정지 상태에서 1회** 보낸다. 준비되면 Enter",
                key="emergency_ready")
    if s.ctrl is not None:
        s.ctrl.emergency_stop = True
        s.event("emergency", phase="on")
        await s.wait(s.dur(3.0), note="emergency")
        s.ctrl.emergency_stop = False
        s.event("emergency", phase="off")
    await s.wait(s.dur(3.0), note="emergency_release")
    await s.ask("emergency 해제가 됐나(다시 움직이나)? 확인 후 Enter", key="emergency_release")

    # ⑤ 송신 데드맨 — 판단 루프가 멈추면 stop 이 나가는가 (FM8a)
    s.event("safety_block", block="deadman")
    s.cmd("forward_slow", "데드맨 시험")
    await s.wait(s.dur(1.5), note="deadman_pre")
    t0 = s.now()
    # **`check()` 의 반환값을 보지 않는다.** 실주행에는 watchdog 태스크가 20 ms 마다
    # 같은 check 를 돌아서, 그쪽이 먼저 트립하면 여기 check 는 movement=="stop" 이라
    # 영영 False 를 돌려준다(위상에 따라 13/20 확률로 '안 걸림' 오판). trips 증가분과
    # 컨트롤러가 실제로 stop 으로 바뀌었는지를 본다.
    trips0 = s.tx.deadman_trips
    tripped = False
    while s.now() - t0 < 1.5:                 # 일부러 feed 를 멈춘다
        if s.dry:
            s.plant.step(0.05)
        else:
            await asyncio.sleep(0.05)
        s.tx.check()
        if (s.tx.deadman_trips > trips0
                or getattr(s.ctrl, "current_movement", None) == "stop"):
            tripped = True
            break
    s.stop("데드맨 시험 끝")
    s.event("deadman_test", tripped=bool(tripped), trips0=trips0,
            trips=s.tx.deadman_trips if s.tx else None,
            movement=getattr(s.ctrl, "current_movement", None))
    s.say("데드맨: %s" % ("stop 강제됨(정상)" if tripped else "안 걸림 — 확인 필요"))
    await s.wait(s.dur(2.0), note="deadman_after")
    await s.return_to(start)

    # ⑥ kill -STOP 2 s — 호스트 동결(FM8b). 소프트웨어 대응이 없다는 걸 확인하는 시험
    if s.args.kill_stop and not s.dry:
        await s.ask("호스트 동결 시험(kill -STOP 2 s)을 한다. 제동 준비됐으면 Enter",
                    key="kill_stop_ready")
        import signal
        import subprocess
        pid = os.getpid()
        s.cmd("forward_slow", "kill -STOP 시험")
        await s.wait(2.0, note="pre_freeze")
        subprocess.Popen(["bash", "-c", "sleep 2; kill -CONT %d" % pid])
        s.event("kill_stop", phase="before", t=time.time())
        os.kill(pid, signal.SIGSTOP)
        s.event("kill_stop", phase="after", t=time.time())
        s.stop("freeze 끝")
        await s.wait(3.0, note="post_freeze")
        await s.return_to(start)
    else:
        s.event("kill_stop", skipped=True,
                why="--kill-stop 을 안 줬거나 dry-run")

    # ⑦ 조이스틱 개입 ×3 (FM11) / ⑧ 이중 송신원 / ⑨ heartbeat 0x05
    for k in range(n):
        await s.ask("PC 가 전진 중일 때 조이스틱으로 정지를 시도해 보라 (%d/%d). 끝나면 Enter"
                    % (k + 1, n), key="joystick_%d" % k)
        s.cmd("forward_slow", "조이스틱 개입 시험")
        await s.wait(s.dur(4.0), note="joystick")
        s.stop()
        await s.wait(s.dur(2.0), note="joystick_after")
        await s.return_to(start)
    await s.ask("리모컨 수신기를 켠 채로 두면 bus-off 가 나는가? 확인했으면 Enter",
                key="dual_source")
    try:
        CF = _cf()
        if CF is None:
            raise RuntimeError("canlib 없음 — heartbeat 데이터 시험 생략")
        old = list(CF.HEARTBEAT_DATA)
        CF.HEARTBEAT_DATA[0] = 0x05           # CiA 301 operational (기본 0x00 은 boot-up)
        s.event("heartbeat_data", value=5)
        await s.wait(s.dur(10.0), note="hb_0x05")
        CF.HEARTBEAT_DATA[:] = old
        s.event("heartbeat_data", value=old[0], restored=True)
    except Exception as exc:
        s.event("heartbeat_data", error=str(exc))
    return {}


async def mode_mount(s):
    """(2) 장착 종속 perception 30 min — 줄자 입력 + 두 셀 재검 + 태그 자기보정."""
    s.say("=== mount: 장착 기하 ===")
    s.say("줄자로 재서 숫자를 적는다. 빈 줄이면 기본값이 들어간다(나중에 고칠 수 있다)")
    vals = {}
    vals["cam_height_m"] = await s.ask_value("카메라 높이(바닥→렌즈) [m]", "cam_height_m", 0.50)
    vals["tag_height_m"] = await s.ask_value("태그 중심 높이 [m]", "tag_height_m", 1.60)
    vals["cam_pitch_deg"] = await s.ask_value("카메라 장착 pitch [도] (위로 +)", "cam_pitch_deg", 0.0)
    vals["cam_roll_deg"] = await s.ask_value("카메라 장착 roll [도]", "cam_roll_deg", 0.0)
    vals["x_off_m"] = await s.ask_value("카메라 ↔ 차체 중심 횡오프셋 [m] (오른쪽 +)", "x_off_m", 0.0)
    vals["cam_yaw_deg"] = await s.ask_value(
        "카메라 장착 yaw [도] (광축이 차체 정면보다 **반시계**면 +)", "cam_yaw_deg", 0.0)
    # δ̂. 태그 겨냥 목표가 β* = −δ̂ 라(계약 §1.3 (I5)) 부호를 반대로 넣으면 그 두 배만큼
    # 틀어진 채 블라인드로 들어간다. 줄자로 못 재면 0 으로 두고 첫 도킹의 치우침으로 잡는다.
    vals["cam_to_ref_m"] = await s.ask_value("카메라 → 접힌 포크 끝 [m]", "cam_to_ref_m",
                                             C.CAM_TO_REF_M)
    s.event("mount_measured", **vals)

    for cell, text in (("tilt20", "3.5 m·tilt 20도(비스듬)에 차를 세우고 Enter"),
                       ("front", "3.5 m·정면(tilt≈0)에 차를 세우고 Enter")):
        await s.ask(text, key="mount_%s" % cell)
        s.event("mount_cell", cell=cell, phase="start")
        for _ in range(100):
            if await s.pump("mount_%s" % cell) is None:
                break
        m = await s.measure(30, note="mount_%s" % cell)
        s.event("mount_cell", cell=cell, phase="end", **(m or {}))
    return vals


async def mode_rotate(s):
    """(3a) 회전 — 개루프 격자 → 폐루프 → 카메라 동반(A·s) → 사후 일관성 → 저속 β."""
    s.say("=== rotate: 회전 ===")
    await s.ask("차 주변이 비었는가? 제자리 회전을 여러 번 한다. Enter", key="rotate_ready")
    n = max(1, s.args.n)
    start_heading = s.gyro.angle_deg if s.gyro else 0.0

    # 개루프 펄스 L/R × {1.0, 1.5, 2.0, 3.0 s} × n — ω(t) 에서 τ_r·α_up·α_r 를 뽑는다.
    # plan 2-4 가 지정한 격자다. {1.5, 3.0} 두 점만으로는 램프를 못 잡아 α 가 영영 None
    # 이었고, 회전 정지 규칙이 순수지연 1파라미터로 되돌아갔다(σ_θ ≤ 0.5° 요건 미달).
    _os = (s.args.sec,) if s.args.sec_given else (1.0, 1.5, 2.0, 3.0)
    for sec in (_os if _want(s, "open") else ()):
        _pairs = (("rotate_ccw", "L"), ("rotate_cw", "R"))
        if s.args.side:
            _pairs = tuple(x for x in _pairs if x[1] == s.args.side.upper())
        for mv, side in _pairs:
            for k in range(n if s.args.n_given else min(4, n * 2)):
                s.block = "rot_open_%s_%.1f_%d" % (side, sec, k)
                s.event("rotate_open", side=side, sec=sec, trial=k, block=s.block)
                await s.hold(mv, sec, why="개루프 %s %.1fs" % (side, sec), settle=s.settle_sec())
                await s.return_heading(start_heading)
    # 5 s 자이로 전용 ×2 (200 Hz 원시만 본다)
    for k in range((n if s.args.n_given else min(2, n)) if _want(s, "gyro") else 0):
        s.event("rotate_open_long", trial=k)
        await s.hold("rotate_ccw", 5.0, why="5 s 자이로 전용", settle=s.settle_sec())
        await s.return_heading(start_heading)

    # 폐루프 15도 × 6 — σ_θ 요건 0.5도 (plan 2-4)
    for k in range((n if s.args.n_given else min(6, n * 3)) if _want(s, "closed") else 0):
        s.event("rotate_closed", target=15.0, trial=k)
        await s.rotate_closed(+15.0 if k % 2 == 0 else -15.0, why="폐루프 15도")
    await s.return_heading(start_heading)

    # tilt ≥ 20도, 3.5/6 m × n — Δβ_px vs Δψ_gyro 로 A·s 분리 (plan 1-1)
    # --start-m 을 주면 그 거리 하나만 — 거리를 바꾸려면 어차피 수동으로 옮겨야 한다
    dists = ((s.args.start_m,) if s.args.start_m else (3.5, 6.0)) if _want(s, "arm") else ()
    for dist in dists:
        # tilt 는 눈으로 못 잰다 — 재서 알려주고, 모자라면 옮겨 다시 재게 한다.
        # 모자란 채로 하면 회전팔 A 측정이 통째로 무의미해진다(각도를 못 믿는 자세).
        for _try in range(6):
            await s.ask(
                "태그에서 **%.1f m**(수직거리) 떨어져, 옆으로 **약 %.1f m** 비켜서서 "
                "몸을 태그 쪽으로 돌려 세우고 Enter   (tilt %d도 이상이면 된다)"
                % (dist, dist * math.tan(math.radians(ARM_TILT_WANT_DEG)),
                   ARM_TILT_MIN_DEG),
                key="rotate_cam_%.1f_%d" % (dist, _try))
            m = await s.measure(20, note="arm_pose_check")
            t = (m or {}).get("tilt_deg")
            d_now = (m or {}).get("forward")
            if t is None:
                s.say("  !! 태그가 안 보인다 — 몸을 태그 쪽으로 더 돌려라")
                continue
            s.say("  지금 자세: tilt **%.1f도** · 거리 %.2f m" % (float(t), float(d_now or 0)))
            if float(t) >= ARM_TILT_MIN_DEG:
                s.say("  좋다 — 이 자세로 간다")
                break
            need = dist * math.tan(math.radians(ARM_TILT_WANT_DEG))
            s.say("  !! tilt 가 %.0f도 미만이다. **옆으로 더 비켜서고**(약 %.1f m) "
                  "몸을 태그 쪽으로 더 돌려라" % (ARM_TILT_MIN_DEG, need))
        else:
            s.say("  !! tilt 를 못 맞췄다 — 이 거리는 건너뛴다(A 측정 신뢰 불가)")
            s.event("arm_skipped", dist=dist, why="tilt_too_low")
            continue
        for k in range(n if s.args.n_given else min(5, n * 2)):
            before = await s.measure(30, note="A_before")
            r = await s.rotate_closed(+20.0 if k % 2 == 0 else -20.0, why="A 측정 회전")
            await s.wait(s.dur(3.0), note="A_settle")      # 자이로 +3 s 적분
            after = await s.measure(30, note="A_after")
            s.event("rotate_camera", dist=dist, trial=k, before=before, after=after,
                    rotate=r)
            if s.tag_lost():
                s.say("!! 태그를 놓쳤다 — 이 거리는 중단")
                break
        await s.return_heading(start_heading)

    # 3.5 m tilt 25도 ±5도 ×10 — 사후 일관성(회전 뒤 카메라-자이로 차)
    for k in range((n if s.args.n_given else min(10, n * 5)) if _want(s, "small") else 0):
        r = await s.rotate_closed(+5.0 if k % 2 == 0 else -5.0, why="소각 일관성")
        m = await s.measure(20, note="small_rot")
        s.event("rotate_small", trial=k, rotate=r, after=m)
    await s.return_heading(start_heading)

    # 저속 회전으로 검출이 끊기는 β 좌우 ×3 (가시성 한계)
    for k in range((n if s.args.n_given else min(3, n * 2)) if _want(s, "beta") else 0):
        for mv, side in (("rotate_ccw", "L"), ("rotate_cw", "R")):
            s.event("rotate_beta_limit", side=side, trial=k, phase="start",
                    beta=(s.last_doc or {}).get("heading_deg"))
            s.cmd(mv, "β 한계 탐색")
            t0 = s.now()
            while s.now() - t0 < 6.0:
                res = await s.pump("beta_limit")
                if res is None or s.tag_lost():
                    break
            s.stop()
            await s.wait(s.dur(2.0), note="beta_limit_stop")
            s.event("rotate_beta_limit", side=side, trial=k, phase="end")
            await s.return_heading(start_heading)
    return {}


async def mode_forward(s):
    """(4a) 67 정속정지 n=20 (정면 10 + 사각 10). **여기까지가 필수.**"""
    s.say("=== forward: 정속 정지 ===")
    level = s.args.level
    mv = "forward_slow" if int(level) == 97 else "forward"
    n = s.args.n if s.args.n else 10
    geoms = tuple(g for g in ("front", "oblique") if _want(s, g, "cruise"))
    for geom in geoms:
        await s.ask("%s 자세(%s)로 %.1f m 에 세우고 Enter"
                    % ("정면" if geom == "front" else "사각 30도",
                       "tilt<10도" if geom == "front" else "tilt≈30도", s.start_m),
                    key="forward_%s" % geom)
        start = (await s.measure(30, note="forward_start") or {}).get("forward", s.start_m)
        for k in range(n):
            if s.tag_lost():
                s.say("!! 태그 실종 — 이 단계 중단")
                return {"aborted": "tag_lost"}
            s.block = "forward_%s_%d_%d" % (geom, level, k)
            s.event("forward_run", geom=geom, level=level, trial=k, phase="start",
                    start_forward=start, block=s.block)
            r = await s.hold(mv, s.args.sec, why="%s %d 정속정지 %d" % (geom, level, k),
                             settle=s.settle_sec())
            m = await s.measure(30, note="forward_stopped")
            s.event("forward_run", geom=geom, level=level, trial=k, phase="end",
                    cmd=r, stopped=m)
            await s.return_to(start)
    # **짧은 명령 격자** — δ_x(최소 신뢰 증분)와 S(T) 표는 길이가 **여러 개**라야 나온다.
    # 전부 --sec 하나로만 돌면 analyze 가 "더 짧은 걸 안 해 봤다" 며 None 을 내고,
    # δ_q 가 가정값 0.30 m 로 남아 상태기계가 그보다 작은 보정을 영영 거부한다(Tier 3).
    if not _want(s, "short"):
        return {"parts": "cruise only"}
    await s.ask("짧은 명령 격자(%s)를 돈다. 시작 자리로 세우고 Enter"
                % ", ".join("%.1fs" % t for t in SHORT_CMD_S), key="forward_short")
    start = (await s.measure(30, note="short_start") or {}).get("forward", s.start_m)
    _ss = (s.args.sec,) if s.args.sec_given else SHORT_CMD_S
    for sec in _ss:
        # 버킷당 3회 이상이라야 CV 가 뜻이 있다. 단 --n 을 직접 주면 그 값을 따른다
        # (한 번에 한 시행만 하고 나가야 하는 현장 흐름 때문).
        per = n if getattr(s.args, "n_given", False) else max(3, min(4, n))
        for k in range(max(1, per)):
            if s.tag_lost():
                s.say("!! 태그 실종 — 짧은 명령 격자 중단")
                return {"aborted": "tag_lost"}
            s.block = "forward_short_%d_%.1f_%d" % (level, sec, k)
            s.event("forward_run", geom="short", level=level, trial=k, phase="start",
                    sec=sec, start_forward=start, block=s.block)
            r = await s.hold(mv, sec, why="%d 짧은 명령 %.1fs #%d" % (level, sec, k),
                             settle=s.settle_sec())
            m = await s.measure(20, note="short_stopped")
            s.event("forward_run", geom="short", level=level, trial=k, phase="end",
                    sec=sec, cmd=r, stopped=m)
            await s.return_to(start)
    return {}


async def mode_creep(s):
    """97 stiction 4 s ×10 → (선택) 편향 스윕 → 펄스 {1,1.5,2,2.5} ×5."""
    s.say("=== creep: 저속 97 ===")
    n = s.args.n if s.args.n else 10
    start = (await s.measure(30, note="creep_start") or {}).get("forward", s.start_m)
    for k in range(n if _want(s, "stiction") else 0):
        s.block = "creep_%d" % k
        s.event("creep_stiction", trial=k, phase="start", block=s.block)
        r = await s.hold("forward_slow", 4.0, why="97 stiction %d" % k, settle=s.settle_sec())
        m = await s.measure(20, note="creep_stopped")
        s.event("creep_stiction", trial=k, phase="end", cmd=r, stopped=m)
        await s.return_to(start)

    CF = _cf()
    if s.args.sweep and CF is None:
        s.say("(canlib 없음 — 편향 스윕 생략)")
    elif s.args.sweep:
        base = list(CF.MOVEMENT_TEMPLATES["forward_slow"])
        try:
            for bias in ((10, 20, 30, 45) if _want(s, "sweep") else ()):
                CF.MOVEMENT_TEMPLATES["forward_slow"][2] = CF.AN_NEUTRAL - bias
                for k in range(n if s.args.n_given else min(3, n)):
                    s.event("creep_sweep", bias=bias, byte2=CF.AN_NEUTRAL - bias, trial=k,
                            phase="start")
                    r = await s.hold("forward_slow", 4.0, why="편향 %d" % bias,
                                     settle=s.settle_sec())
                    m = await s.measure(20, note="sweep_stopped")
                    s.event("creep_sweep", bias=bias, trial=k, phase="end", cmd=r,
                            stopped=m)
                    await s.return_to(start)
        finally:
            CF.MOVEMENT_TEMPLATES["forward_slow"][:] = base
            s.event("creep_sweep", restored=True,
                    byte2=CF.MOVEMENT_TEMPLATES["forward_slow"][2])

    _ps = (s.args.sec,) if s.args.sec_given else (1.0, 1.5, 2.0, 2.5)
    for sec in (_ps if _want(s, "pulse") else ()):
        for k in range(n if s.args.n_given else min(5, n)):
            s.event("creep_pulse", sec=sec, trial=k, phase="start")
            r = await s.hold("forward_slow", sec, why="97 펄스 %.1fs" % sec,
                             settle=s.settle_sec())
            m = await s.measure(20, note="pulse_stopped")
            s.event("creep_pulse", sec=sec, trial=k, phase="end", cmd=r, stopped=m)
            await s.return_to(start)
    return {}


async def mode_tagcut(s):
    """태그가 잘리는 거리 실측 + depth 포함 bag (블라인드 구간 자료)."""
    s.say("=== tagcut: 태그가 잘리는 거리 ===")
    await s.ask("태그를 정면으로 보는 자리(%.1f m)에 세우고 Enter" % s.start_m,
                key="tagcut_ready")
    start = (await s.measure(30, note="tagcut_start") or {}).get("forward", s.start_m)
    bag = os.path.join(s.dir, "tagcut_depth.bag")
    reopened = False
    if not s.dry and not s.args.no_bag:
        reopened = await s.reopen_camera(record=bag, with_depth=True)
        s.event("tagcut_bag", path=bag, ok=reopened)
    level = "forward_slow" if s.args.level == 97 else "forward"
    s.cmd(level, "태그 컷까지 접근")
    t0 = s.now()
    cut = None
    last = {}
    # 이 모드만 전진을 **켜 놓고** 폴링한다(다른 모드는 시간이 정해진 hold 를 쓴다).
    # 그래서 컷 판정이 안 뜨면 멈출 근거가 시간뿐이라 벽까지 간다 → 거리 바닥과
    # 주행량 상한을 둔다. 정상 경로(3.5 m 출발, 3.3 m 컷)는 0.2 m 라 절대 안 걸린다.
    floor = C.CAM_TO_REF_M + TAGCUT_FLOOR_M      # 포크 끝이 태그면에서 이만큼 앞
    cap = start + TAGCUT_MAX_TRAVEL_M            # 여기까지 오면 무조건 선다
    why_stop = "태그 컷"
    while s.now() - t0 < 25.0:
        res = await s.pump("tagcut")
        if res is None:
            break
        doc = res.docking.get(s.tag_id)
        det = None
        for d in getattr(res, "detections", []):
            if int(d.tag_id) == int(s.tag_id):
                det = d
        if det is not None and doc is not None:
            mg = tag_edge_margin_px(det, res.image.shape)
            fwd = float(doc["forward"])
            last = {"forward": fwd, "margin_px": mg}
            if fwd <= floor:
                cut = dict(last, t=s.now(), why="거리 바닥 %.2f m — 컷 판정 전에 멈췄다" % floor)
                why_stop = "!! 거리 바닥 %.2f m (컷 판정이 안 떴다)" % floor
                break
            if start - fwd >= TAGCUT_MAX_TRAVEL_M:
                cut = dict(last, t=s.now(), why="주행량 상한 %.2f m" % TAGCUT_MAX_TRAVEL_M)
                why_stop = "!! 주행량 상한 %.2f m (컷 판정이 안 떴다)" % TAGCUT_MAX_TRAVEL_M
                break
            if mg < CUT_MARGIN_PX:
                cut = dict(last, t=s.now())
                break
        elif s.tag_lost():
            cut = dict(last, t=s.now(), why="검출 끊김")
            break
    else:
        why_stop = "!! 25 s 타임아웃 (컷 판정이 안 떴다)"
    s.stop(why_stop)
    if why_stop != "태그 컷":
        s.say(why_stop)
    await s.wait(s.dur(3.0), note="tagcut_stop")
    m = await s.measure(20, note="tagcut_after")
    s.event("tagcut", cut=cut, after=m, margin_rule_px=CUT_MARGIN_PX,
            second_tag="2번째 태그가 있으면 각 태그의 마지막 검출 거리는 frame.jsonl 에 있다")
    s.say("태그 컷: %s" % cut)
    if reopened:
        await s.reopen_camera(record=None, with_depth=False)
    await s.return_to(start)
    return {"cut": cut}


def _dry_pose(args, mode, start_m):
    """dry-run 가짜 리그의 시작 (lateral, heading). grid 는 **비스듬**해야 앵커가 선다.

    tilt(태그면을 얼마나 비스듬히 보나) ≈ atan(lateral / forward) 라 lateral 만 키우면
    tilt 는 커지지만 차가 정면을 보고 있어 태그가 화면 밖으로 나간다 → heading 도 같이
    틀어 태그를 보게 한다.
    """
    lat = float(args.lat) if getattr(args, "lat", None) is not None else 0.05
    psi = getattr(args, "psi", None)
    if psi is None:
        # 태그를 보게 — 자리가 옆이면 그만큼 틀어야 화면에 남는다
        psi = -math.degrees(math.atan2(lat, max(0.5, start_m))) if abs(lat) > 0.3 else 2.0
    return lat, float(psi)


def _want(s, *names):
    """`--part` 로 고른 부분만 돈다. 안 주면 전부.

    한 단계 안에서 **차를 옮기라고 시키는 곳**이 있으면 그 단계는 한 번에 못 끝낸다
    (프로그램이 CAN 을 잡고 있어 수동 전환이 안 되므로 반드시 종료해야 한다).
    그래서 자리가 바뀌는 지점마다 부분을 나눠, 부분 하나씩 돌리고 나가게 한다.
    결과는 같은 세션 폴더에 쌓이고 analyze 가 전부 모아서 본다.
    """
    p = (getattr(s.args, "part", "") or "").strip().lower()
    if not p:
        return True
    want = {x.strip() for x in p.split(",") if x.strip()}
    return any(n in want for n in names)


def _f(v):
    """ask_value 가 준 것을 float 로. 못 바꾸면 None."""
    try:
        return float(v)
    except (TypeError, ValueError):
        return None


async def mode_grid(s):
    """heading 바이어스 격자 — **사람은 자리만 대충 잡고, 각도는 차가 잰다.**

    무엇을 재나
    ------------
    "정면에서 카메라 heading 이 부드럽게 치우친다"(9/7) 를 tilt 별로 확인해 θ_min
    (정면 불신 경계)을 낸다.

    왜 줄자도, 큰 회전도 아닌가
    ----------------------------
    · 줄자로 각도를 재면 오차가 재려는 바이어스(1~2도)보다 크다 → 무의미.
    · **제자리 큰 회전도 안 된다**: tilt 는 차의 **위치**가 정하지 회전이 안 바꾼다.
      돌리면 tilt 는 그대로인 채 태그만 화면 밖으로 나간다(3.5 m 에서 36도면 FOV 밖).
    그래서 이렇게 한다:
      (1) 사람은 **자리만** 옮긴다 — "왼쪽으로 대략 0.5 m" 수준. 정확할 필요 없다.
          자리가 tilt 를 정하고, **tilt 는 카메라가 정확히 잰다**(tilt 는 정면에서도 안 흔들린다).
      (2) 그 자리에서 차가 **작게 ±흔든다**(태그가 화면에 남는 각도).
          자이로가 실제로 돈 양 Δψ_gyro 를 재고, 카메라가 읽은 변화 Δψ_cam 과 비교한다.
      (3) **ratio = Δψ_cam / Δψ_gyro.** 1 이면 카메라가 정직하고, 1 에서 벗어난 만큼이
          그 tilt 에서의 왜곡이다. 9/7 정면쌍이 정확히 이것이었다: 7.02 / 5.61 = 1.25.

    ratio 는 **차이(증분)로만** 나오므로 절대 기준(앵커·줄자)이 아예 필요 없다.
    """
    s.say("=== grid: heading 바이어스 vs tilt ===")
    s.say("각도는 안 재도 된다. 자리만 대충 옮기면 차가 스스로 흔들어 잰다")
    cells = []
    lats = GRID_LATERALS_M
    if getattr(s.args, "pos", None) is not None:      # --pos 0.5 → 그 자리만
        lats = tuple(float(x) for x in str(s.args.pos).split(",") if x.strip())
    for lat in lats:
        await s.ask("%.1f m 에서 도킹축 기준 **왼쪽으로 대략 %.1f m** 되게 세우고 Enter "
                    "(자 없이 걸음으로 재도 된다 — 정확할 필요 없다)"
                    % (s.start_m, lat), key="grid_lat_%.1f" % lat)
        base = await s.measure(30, note="grid_base_%.1f" % lat)
        if not base or base.get("heading_deg") is None:
            s.say("  태그를 못 봤다 — 이 자리는 건너뛴다")
            continue
        tilt = float(base.get("tilt_deg") or 0.0)
        s.say("  자리 확인: tilt %.1f 도 · %.2f m · 카메라 heading %+.2f 도"
              % (tilt, base.get("forward") or 0.0, float(base["heading_deg"])))
        for k in range(s.args.n if s.args.n_given else max(2, min(6, s.args.n * 2))):
            deg = GRID_WIGGLE_DEG * (1.0 if k % 2 == 0 else -1.0)
            before = await s.measure(20, note="wig_before")
            r = await s.rotate_closed(deg, why="grid 흔들기 %+.1f" % deg)
            after = await s.measure(20, note="wig_after")
            if not (before and after and r):
                continue
            if before.get("heading_deg") is None or after.get("heading_deg") is None:
                s.say("    태그를 놓쳤다 — 이 흔들기는 버린다")
                continue
            d_cam = float(after["heading_deg"]) - float(before["heading_deg"])
            d_gyro = float(r.get("turned_deg") or 0.0)
            if abs(d_gyro) < 0.5:
                continue
            ratio = d_cam / d_gyro
            cells.append({"tilt_deg": tilt, "ratio": ratio,
                          "d_cam_deg": d_cam, "d_gyro_deg": d_gyro,
                          "forward": after.get("forward")})
            s.say("    Δ카메라 %+.2f · Δ자이로 %+.2f → **비 %.3f**%s"
                  % (d_cam, d_gyro, ratio, "  ← 1 에서 멀다" if abs(ratio - 1) > 0.10 else ""))
            s.event("grid_wiggle", tilt_deg=tilt, ratio=ratio, d_cam_deg=d_cam,
                    d_gyro_deg=d_gyro, forward=after.get("forward"),
                    lateral_cmd=lat, trial=k)
        # 이 자리의 요약 — analyze 가 tilt 별 바이어스로 읽는다
        mine = [c["ratio"] for c in cells if c["tilt_deg"] == tilt]
        if mine:
            med = statistics.median(mine)
            # 바이어스 등가값: 비가 1 에서 벗어난 만큼을 흔든 각도에 곱한다
            bias_equiv = (med - 1.0) * GRID_WIGGLE_DEG
            s.say("  tilt %.1f 도 → 비 중앙값 %.3f (바이어스 등가 %+.2f 도, n=%d)"
                  % (tilt, med, bias_equiv, len(mine)))
            s.event("grid_cell", heading_cmd=lat, truth_deg=0.0,
                    truth_source="wiggle-ratio", tilt_deg=tilt,
                    ratio_median=med, bias_deg=bias_equiv, n=len(mine),
                    forward=base.get("forward"), heading_deg=bias_equiv, phase="end")
    if cells:
        lo = min(c["ratio"] for c in cells); hi = max(c["ratio"] for c in cells)
        s.say("비 범위 %.3f ~ %.3f — 정면(tilt 작을 때)에서 1 에서 멀어지면 그게 찾던 것"
              % (lo, hi))
    return {"n": len(cells)}


async def mode_oblique(s):
    """비스듬히 5 → 3.5 m 직진 ×2 — Δlat/Δs 로 CAM_YAW_OFFSET(δ) 를 뽑는다(plan 1-5)."""
    s.say("=== oblique: 비스듬 직진 ===")
    for k in range(s.args.n if s.args.n_given else min(2, max(1, s.args.n))):
        await s.ask("태그를 비스듬히(heading ≈ 25도) 보는 5 m 자리에 세우고 Enter",
                    key="oblique_%d" % k)
        before = await s.measure(30, note="oblique_before")
        s.event("oblique", trial=k, phase="start", before=before)
        r = await s.hold("forward", 5.0, why="oblique 5→3.5 m", settle=s.settle_sec())
        after = await s.measure(30, note="oblique_after")
        s.event("oblique", trial=k, phase="end", cmd=r, after=after)
        await s.return_to((before or {}).get("forward", 5.0))
    return {}


async def mode_search(s):
    """태그가 안 보이는 방향에서 시작 → 회전 탐색 → 찾으면 기록·정지."""
    s.say("=== search: 탐색 ===")
    await s.ask("태그가 **안 보이는** 방향을 보게 차를 돌려놓고 Enter", key="search_ready")
    start = s.gyro.angle_deg if s.gyro else 0.0
    s.event("search", phase="start", gyro_deg=start)
    s.cmd("rotate_ccw", "탐색 회전")
    t0 = s.now()
    found = None
    while s.now() - t0 < 50.0:
        res = await s.pump("search")
        if res is None:
            break
        if res.docking.get(s.tag_id) is not None:
            found = {"gyro_deg": s.gyro.angle_deg if s.gyro else None,
                     "t": s.now() - t0, **res.docking[s.tag_id]}
            break
    s.stop("탐색 끝")
    await s.wait(s.dur(3.0), note="search_stop")
    s.event("search", phase="end", found=found)
    s.say("탐색: %s" % ("찾음 %.1f 도" % found["gyro_deg"] if found else "못 찾음"))
    return {"found": bool(found)}


MODES = {"timing": mode_timing, "safety": mode_safety, "mount": mode_mount,
         "rotate": mode_rotate, "forward": mode_forward, "creep": mode_creep,
         "tagcut": mode_tagcut, "grid": mode_grid, "oblique": mode_oblique,
         "search": mode_search}


# ── 카메라 다시 열기 (bag/depth 구성 바꾸기) ────────────────────────────────

async def _reopen_camera(self, record=None, with_depth=False):
    """bag 녹화·depth 구성을 바꿔 카메라만 다시 연다. 실패하면 False 이고 그대로 간다."""
    if self.dry:
        self.event("camera_reopen", record=record, with_depth=with_depth, fake=True)
        return False
    from src.models import TagPipeline
    from src.utils.camera import CameraSettings
    try:
        self.pipe.close()
    except Exception:
        pass
    try:
        tune = CameraSettings.docking()
        self.pipe = TagPipeline.from_realsense(self.tag_size, tune=tune,
                                               timing=self.timing, label="first_run",
                                               record=record, with_depth=with_depth)
    except Exception as exc:
        self.say("!! 카메라 재개 실패: %s" % exc)
        self.event("camera_reopen", error=str(exc))
        return False
    self.event("camera_reopen", record=record, with_depth=with_depth, ok=True)
    return True


Session.reopen_camera = _reopen_camera


# ═══════════════════════════════════════════════════════════════════════════
# 실행기 (한 단계 / all / resume)
# ═══════════════════════════════════════════════════════════════════════════

async def run_stage(args, mode, root=None):
    s = Session(args, mode, root=root)
    out, err = {}, None
    try:
        await s.open()
        s.stop("단계 시작 stop 프레임")
        await s.wait(s.dur(1.0), note="stage_begin")
        out = await MODES[mode](s) or {}
        s.stop("단계 끝 stop 프레임")
        await s.wait(s.dur(1.0), note="stage_end")
    except (KeyboardInterrupt, asyncio.CancelledError):
        # asyncio.run 은 Ctrl+C 에 태스크를 취소해서 CancelledError 로 온다(3.11+)
        s.say("!! Ctrl+C — 정지 프레임을 보내고 나간다")
        s.event("interrupt")
        err = "interrupt"
    except SystemExit as exc:
        s.event("abort", error=str(exc))
        err = str(exc)
        raise
    except Exception as exc:
        s.say("!! 오류: %r" % (exc,))
        s.event("error", error=repr(exc))
        err = repr(exc)
    finally:
        await s.close()
    return {"mode": mode, "dir": s.dir, "error": err,
            "gate_passed": s.gate_passed, **out}


def _state_path(root):
    return os.path.join(root, "state.json")


def _load_state(root):
    try:
        with open(_state_path(root), encoding="utf-8") as f:
            return json.load(f)
    except Exception:
        return None


def _save_state(root, state):
    os.makedirs(root, exist_ok=True)
    with open(_state_path(root), "w", encoding="utf-8") as f:
        json.dump(state, f, ensure_ascii=False, indent=1)


def _latest_all_root():
    if not os.path.isdir(WORK):
        return None
    cands = [os.path.join(WORK, d) for d in sorted(os.listdir(WORK))
             if d.endswith("_all") and os.path.exists(os.path.join(WORK, d, "state.json"))]
    return cands[-1] if cands else None


async def run_all(args):
    root = None
    state = None
    if args.resume:
        root = args.root or _latest_all_root()
        state = _load_state(root) if root else None
        if state is None:
            print("  !! --resume 인데 이어갈 state.json 이 없다 — 새로 시작한다")
    if state is None:
        root = args.root or os.path.join(WORK, "%s_all" % _now_str())
        state = {"root": root, "started": _now_str(), "stages": {}, "argv": sys.argv}
        _save_state(root, state)
    print("  세션 폴더: %s" % root)

    skip = set(x.strip() for x in (args.skip or "").split(",") if x.strip())
    order = list(ORDER)
    if args.from_mode:
        if args.from_mode not in order:
            raise SystemExit("--from 은 %s 중 하나" % ", ".join(order))
        order = order[order.index(args.from_mode):]

    for mode in order:
        done = state["stages"].get(mode, {})
        if done.get("status") == "done" and args.resume:
            print("  [건너뜀] %s — 이미 끝났다 (%s)" % (mode, done.get("dir")))
            continue
        if mode in skip:
            state["stages"][mode] = {"status": "skipped"}
            _save_state(root, state)
            print("  [건너뜀] %s — --skip" % mode)
            continue
        print("\n" + "=" * 70)
        print("  단계 %d/%d: %s" % (order.index(mode) + 1, len(order), mode))
        print("=" * 70)
        try:
            res = await run_stage(args, mode, root=root)
        except (KeyboardInterrupt, asyncio.CancelledError):
            state["stages"][mode] = {"status": "interrupted"}
            _save_state(root, state)
            print("\n  중단됨. 이어서 하려면:  python tools/first_run.py all --resume")
            return state
        if res.get("error") == "interrupt":
            state["stages"][mode] = {"status": "interrupted", "dir": res.get("dir")}
            _save_state(root, state)
            print("\n  중단됨. 이어서 하려면:  python tools/first_run.py all --resume")
            return state
        status = "done" if not res.get("error") else "error"
        state["stages"][mode] = {"status": status, "dir": res.get("dir"),
                                 "error": res.get("error"),
                                 "gate_passed": res.get("gate_passed")}
        if mode == "timing":
            state["gate_passed"] = res.get("gate_passed")   # None = 미확인(가짜 장비)
        _save_state(root, state)
        if status == "error" and mode in REQUIRED and not args.keep_going:
            print("  !! 필수 단계 %s 가 실패했다 — 멈춘다 (--keep-going 이면 계속)" % mode)
            return state

    # 마지막에 분석을 자동으로 돌려 캘리브 3종을 만든다
    print("\n" + "=" * 70)
    print("  분석 → 캘리브 3종")
    print("=" * 70)
    import subprocess
    cmd = [sys.executable, os.path.join(ROOT, "tools", "analyze_first_run.py"),
           root, "--install"]
    if state.get("gate_passed") is False:
        cmd.append("--gate-failed")
    print("  %s" % " ".join(cmd))
    try:
        subprocess.run(cmd, check=False)
    except Exception as exc:
        print("  !! 분석 실행 실패: %s" % exc)
    state["analyzed"] = True
    _save_state(root, state)
    return state


def _resolve_mode(text):
    """'3' · 'rotate' · '3.rotate' 를 단계 이름으로. 잘못 주면 목록을 보여준다."""
    t = (text or "").strip()
    if t == "all":
        return "all"
    if t in MODES:
        return t
    head = t.split(".")[0]
    if head.isdigit():
        i = int(head)
        if 1 <= i <= len(ORDER):
            return ORDER[i - 1]
    raise SystemExit("단계를 못 알아들었다: %r\n%s" % (text, _stage_list()))


def _tally(root):
    """세션 폴더에 쌓인 단계별 **실행 횟수와 차량 동작 수**. 나눠 돌린 것을 다 센다."""
    out = {}
    if not root or not os.path.isdir(root):
        return out
    for d in sorted(glob.glob(os.path.join(root, "*"))):
        if not os.path.isdir(d):
            continue
        name = os.path.basename(d).split("_")[-1]
        ev = os.path.join(d, "events.jsonl")
        moves = 0
        try:
            with io.open(ev, encoding="utf-8") as f:
                for line in f:
                    if ('"cmd_start"' in line or '"rotate_start"' in line
                            or '"safety_block"' in line or '"grid_wiggle"' in line):
                        moves += 1
        except Exception:
            pass
        r = out.setdefault(name, {"runs": 0, "moves": 0})
        r["runs"] += 1
        r["moves"] += moves
    return out


def _stage_list(state=None, root=None):
    """번호가 붙은 단계 목록. state 가 있으면 끝낸 것에 표시."""
    done = (state or {}).get("stages", {})
    tal = _tally(root or (state or {}).get("root"))
    out = ["  단계 목록 (번호로도 된다: python tools/first_run.py 4)",
           "        단계      시작    돌린횟수  차량동작"]
    for i, m in enumerate(ORDER, 1):
        st = done.get(m, {}).get("status")
        mark = {"done": "✔", "interrupted": "…", "error": "✗",
                "skipped": "-"}.get(st, " ")
        need = "필수" if m in REQUIRED else "  "
        start = START_M.get(m, START_M_DEFAULT)
        t = tal.get(m, {})
        out.append("   %s %2d. %-8s %s %5.1f m %6d 회 %7d 번"
                   % (mark, i, m, need, start, t.get("runs", 0), t.get("moves", 0)))
    out.append("  (✔ 끝남 · … 중단됨 · ✗ 실패 · - 건너뜀 | 나눠 돌린 것도 다 세어 합친다)")
    return "\n".join(out)


def _shared_root(args, make=True):
    """단계를 따로 돌려도 **한 세션 폴더**에 쌓이게 한다(끝에 분석을 한 번에 하려고)."""
    if args.root:
        return args.root
    if getattr(args, "solo", False):
        return None                      # 옛 동작 — 단계마다 제 폴더
    root = _latest_all_root()
    if root:
        return root
    if not make:
        return None
    return os.path.join(WORK, "%s_all" % _now_str())


def _note_stage(root, mode, res):
    """단계 하나의 결과를 세션 state.json 에 적는다(--resume·분석이 이걸 본다)."""
    if not root:
        return None
    state = _load_state(root) or {"root": root, "started": _now_str(),
                                  "stages": {}, "argv": sys.argv}
    status = ("interrupted" if res.get("error") == "interrupt"
              else "error" if res.get("error") else "done")
    state["stages"][mode] = {"status": status, "dir": res.get("dir"),
                             "error": res.get("error"),
                             "gate_passed": res.get("gate_passed")}
    if mode == "timing":
        state["gate_passed"] = res.get("gate_passed")
    _save_state(root, state)
    return state



# ═══════════════════════════════════════════════════════════════════════════
# --plan : 현장에서 그대로 복붙할 명령 목록 (한 줄 = 한 시행)
# ═══════════════════════════════════════════════════════════════════════════
#: (단계번호, 부분이름, 추가인자, 반복횟수, 자리 설명)
#: 한 줄이 시행 하나다. 줄 사이에 **프로그램이 종료되어 CAN 을 놓으므로** 수동으로
#: 차를 옮길 수 있다. 자리가 같으면 연달아 쳐도 된다(차가 알아서 복귀한다).
PLAN = [
    # (번호, 부분, 추가인자, 반복, 자세dict)
    #   d   = 태그면까지 **수직거리** [m]
    #   lat = 도킹축에서 옆으로 [m] (0 = 축 위)
    #   face= 차가 어디를 보나
    #   car = 차가 **스스로** 하는 동작 (사람이 안 해도 되는 것)
    ("1", "center", "", 1, dict(d=3.5, lat=0.0, face="태그 정면(축 위)",
         extra="태그가 화면 **중앙 높이**에 오게",
         car="±12° 제자리 회전 3번 → 스스로 0°로 복귀")),
    ("1", "upper", "", 1, dict(d=None, lat=0.0, face="태그 정면(축 위)",
         extra="태그가 화면 **위쪽 행**에 오게 차를 조금 앞으로 (잘리지는 않을 만큼)",
         car="±12° 제자리 회전 3번 → 스스로 0°로 복귀")),
    ("2", "", "", 3, dict(d=8.0, lat=0.0, face="태그 정면(축 위)",
         extra="앞 5 m 비움 · **사람 제동 위치** (포크는 건드리지 않는다)",
         car="저속 전진 6초 × 4종 → 매번 스스로 8 m 로 복귀")),
    ("3", "", "", 1, dict(d=3.5, lat=1.6, face="태그 쪽으로 약 25° 틀어서",
         extra="이 단계는 자리 **2곳**: ①비스듬(위 자세) ②정면(옆 0 m). 줄자 7개 입력",
         car="없음 — 차는 가만히 있는다")),
    ("4", "open", "--sec 1.0 --side L", 4, dict(d=3.5, lat=1.6,
         face="태그 쪽으로 약 25° 틀어서 (tilt ≥ 20°)",
         extra="차 **주변이 원형으로** 비어야 한다 (앞뒤로는 안 간다)",
         car="제자리 회전 → 스스로 원래 각도로 복귀")),
    ("4", "open", "--sec 1.0 --side R", 4, None),
    ("4", "open", "--sec 1.5 --side L", 4, None),
    ("4", "open", "--sec 1.5 --side R", 4, None),
    ("4", "open", "--sec 2.0 --side L", 4, None),
    ("4", "open", "--sec 2.0 --side R", 4, None),
    ("4", "open", "--sec 3.0 --side L", 4, None),
    ("4", "open", "--sec 3.0 --side R", 4, None),
    ("4", "gyro", "", 2, None),
    ("4", "closed", "", 6, dict(same=True, star="σ_θ — 절대 못 버림")),
    ("4", "arm", "--start-m 3.5", 5, dict(d=3.5, lat=1.6,
         face="태그 쪽으로 약 25° 틀어서 (**tilt ≥ 20° 아니면 코드가 다시 세우라고 한다**)",
         extra="★ 회전팔 A",
         car="±20° 제자리 회전 → 스스로 복귀")),
    ("4", "arm", "--start-m 6", 5, dict(d=6.0, lat=2.8,
         face="태그 쪽으로 약 25° 틀어서 (tilt ≥ 20°)",
         extra="★★ **6 m 로 옮긴다** — 이걸 빼면 회전팔 A 를 원리적으로 못 구한다",
         car="±20° 제자리 회전 → 스스로 복귀")),
    ("4", "small", "", 10, dict(d=3.5, lat=1.6, face="태그 쪽으로 약 25° 틀어서",
         extra="3.5 m 자리로 되돌아온다",
         car="±5° 제자리 회전 → 스스로 복귀")),
    ("4", "beta", "", 3, dict(same=True, extra="태그가 안 보일 때까지 돌려 본다",
         car="좌우로 천천히 회전(태그 놓치면 스스로 멈춤)")),
    ("5", "front", "", 10, dict(d=5.5, lat=0.0, face="태그 정면(축 위) · tilt < 10°",
         extra="★ τ_eff · 앞 2 m 여유",
         car="4초 전진(약 1.3 m) → **스스로 5.5 m 로 후진 복귀**")),
    ("5", "oblique", "", 10, dict(d=5.5, lat=2.6,
         face="태그 쪽으로 약 25° 틀어서 (tilt 25~30°)",
         extra="옆으로 2.6 m 가 어려우면 2.0 m(tilt 20°)도 된다",
         car="4초 전진 → 스스로 복귀")),
    ("5", "short", "--sec 1.5", 4, dict(d=5.5, lat=0.0, face="태그 정면(축 위)",
         extra="짧은 명령 — 거리 변동을 본다", car="짧게 전진 → 스스로 복귀")),
    ("5", "short", "--sec 2.0", 4, None),
    ("5", "short", "--sec 3.0", 4, None),
    ("6", "stiction", "", 10, dict(d=4.5, lat=0.0, face="태그 정면(축 위)",
         extra="★ byte2=97 이 실제로 움직이나 (내일 처음 보내는 값)",
         car="저속 4초 전진(약 0.5 m) → 스스로 복귀")),
    ("6", "pulse", "--sec 1.0", 5, dict(same=True, car="저속 짧게 전진 → 스스로 복귀")),
    ("6", "pulse", "--sec 1.5", 5, None),
    ("6", "pulse", "--sec 2.0", 5, None),
    ("6", "pulse", "--sec 2.5", 5, None),
    ("7", "", "", 1, dict(d=3.5, lat=0.0, face="태그 정면(축 위)",
         extra="앞 1.5 m 여유 (태그가 잘릴 때까지 조금 전진)",
         car="태그 컷까지 전진 → 스스로 복귀. 안 멈추면 포크끝 0.40 m 앞에서 강제 정지")),
    ("8", "", "--pos 0", 6, dict(d=3.5, lat=0.0, face="태그 쪽(태그가 화면 가운데 오게)",
         extra="각도는 **안 재도 된다**. 옆 거리도 걸음으로 충분",
         car="제자리 ±3° 흔들기 → 앞뒤로는 안 간다")),
    ("8", "", "--pos 0.5", 6, dict(d=3.5, lat=0.5, face="태그 쪽(태그가 화면 가운데 오게)",
         extra="옆으로 **대략** 0.5 m", car="제자리 ±3° 흔들기")),
    ("8", "", "--pos 1.0", 6, dict(d=3.5, lat=1.0, face="태그 쪽",
         extra="옆으로 대략 1 m", car="제자리 ±3° 흔들기")),
    ("8", "", "--pos 1.5", 6, dict(d=3.5, lat=1.5, face="태그 쪽",
         extra="옆으로 대략 1.5 m", car="제자리 ±3° 흔들기")),
    ("8", "", "--pos 2.0", 6, dict(d=3.5, lat=2.0, face="태그 쪽",
         extra="옆으로 대략 2 m", car="제자리 ±3° 흔들기")),
    ("9", "", "", 2, dict(d=5.0, lat=2.3, face="태그 쪽으로 약 25° 틀어서",
         extra="앞 2 m 여유", car="5 m → 3.5 m 전진 → 스스로 복귀")),
    ("10", "", "", 1, dict(d=None, lat=None, face="**태그가 안 보이는** 방향",
         extra="어디서 해도 된다 · 차 주변 원형으로 비움",
         car="태그를 찾을 때까지 제자리 회전 — 찾으면 스스로 멈춤")),
]


def print_plan(out=print):
    """현장용 명령 목록 — 한 줄이 한 시행. 자세까지 적는다."""
    L = []
    L += ["# " + "=" * 68,
          "# first_run 현장 명령 목록 — 한 줄이 **한 시행**이다",
          "#",
          "#  · 한 줄 치면 실험 1회 하고 **종료하면서 CAN 을 놓는다**",
          "#    → 그때 수동 컨트롤러로 차를 옮길 수 있다",
          "#  · [사람] 표시가 나올 때만 차를 옮긴다. 그 아래 줄들은 같은 자리다",
          "#  · [차]  는 지게차가 **스스로** 하는 것 — 사람이 안 건드려도 된다",
          "#  · 거리(d)는 태그면까지 **수직거리**, 옆(lat)은 도킹축에서 옆으로",
          "#  · 각도는 눈대중이면 된다. 정확히 맞출 필요 없다",
          "#  · Ctrl+C = 즉시 정지 프레임 송신 후 종료 · 진행확인: --list",
          "# " + "=" * 68,
          "",
          "cd ~/krri/apriltag_v3",
          "",
          "# 장치 확인",
          "python tools/check/device_check.py --no-gui",
          'python -c "from canlib import canlib; print(canlib.getNumberOfChannels())"',
          ""]
    cur, last_pose, total = None, None, 0
    for num, part, extra_args, rep, pose in PLAN:
        name = ORDER[int(num) - 1]
        if num != cur:
            cur = num
            last_pose = None
            need = "필수" if name in REQUIRED else "선택"
            L += ["", "# " + "-" * 68,
                  "# %s. %s   (%s)" % (num, name, need),
                  "# " + "-" * 68]
        if pose and not pose.get("same"):
            last_pose = pose
            d, lat = pose.get("d"), pose.get("lat")
            if d is None:
                where = "아무 데나"
            elif not lat:
                where = "태그에서 **%.1f m**, 도킹축 **위**(옆 0 m)" % d
            else:
                where = "태그에서 **%.1f m**, 옆으로 **%.1f m**" % (d, lat)
            L += ["",
                  "#  [사람] 차를 세운다 ─────────────────────────────",
                  "#     어디:  %s" % where,
                  "#     자세:  %s" % pose.get("face", "-")]
            if pose.get("extra"):
                L.append("#     메모:  %s" % pose["extra"])
            if pose.get("car"):
                L.append("#  [차]   스스로 함: %s" % pose["car"])
        elif pose and pose.get("same"):
            note = pose.get("extra") or "바로 위와 **같은 자리**"
            L += ["", "#  [사람] 옮기지 않는다 — %s" % note]
            if pose.get("car"):
                L.append("#  [차]   스스로 함: %s" % pose["car"])
            if pose.get("star"):
                L.append("#     ★ %s" % pose["star"])
        elif pose is None and last_pose is not None:
            L += ["", "#  [사람] 옮기지 않는다 — 바로 위와 같은 자리"]
        args = " ".join(x for x in ("--part %s" % part if part else "", extra_args) if x)
        cmd = "python tools/first_run.py %s %s--n 1" % (num, (args + " ") if args else "")
        for i in range(rep):
            L.append("%-64s # %d/%d" % (cmd, i + 1, rep))
            total += 1
    L += ["", "", "# " + "-" * 68,
          "# 분석 → 캘리브 3종",
          "# " + "-" * 68,
          "python tools/first_run.py --list          # 세션 폴더 이름·진행 확인",
          "python tools/analyze_first_run.py work_dirs/first_run/<세션> --install",
          "cat work_dirs/first_run/<세션>/report.md",
          "",
          "# " + "-" * 68,
          "# 도킹 주행",
          "# " + "-" * 68,
          "python tools/dock.py --dry-run --show                 # CAN 안 보냄",
          "python tools/dock.py --show --record-events           # 실주행",
          "# 캘리브 없이 먼저 해보려면:",
          "python tools/dock.py --show --record-events --assume-calib --final-anyway",
          "",
          "# 총 %d 줄 = %d 시행" % (total, total),
          "# 시간 모자라면 버리는 순서: 10 → 9 → 7 → 8 → 6 → 4의 6 m 자리",
          "# 절대 못 버림: 4 closed(σ_θ) · 4 arm 3.5+6 m(A) · 5 front(τ_eff) · 3 mount(줄자)"]
    out("\n".join(L))
    return total


def _preflight(args):
    """시작 전 사람 확인 — VM 이 멈추면 차가 안 선다."""
    print("\n" + "=" * 70)
    print("  first_run — 내일 현장 측정 도구")
    print("=" * 70)
    if args.dry_run:
        print("  DRY-RUN: 가짜 장비로 코드 경로만 돈다. CAN 으로 아무것도 안 보낸다.")
        return
    print("  호스트가 맥북 VM 이면 **절전이 곧 사고**다:")
    print("    · 맥에서  caffeinate -s  를 켜 두고 뚜껑을 열어 둘 것")
    print("    · RealSense·Kvaser 는 허브 없이 직결")
    print("    · 사람이 제동 위치에 설 것 (호스트 동결에는 소프트웨어 대응이 없다)")
    if args.yes:
        print("  --yes — 확인 프롬프트를 건너뛴다")
        return
    ans = input("  위 세 가지를 확인했는가? (y/Enter=y, n=중단) ").strip().lower()
    if ans.startswith("n"):
        raise SystemExit("중단")


def main():
    ap = argparse.ArgumentParser(description="first_run — 현장 측정(원시 기록만)")
    ap.add_argument("mode", nargs="?", default="all",
                    help="단계 이름(timing·safety·…) 또는 **번호 1~10**, 또는 all. "
                         "목록은 --list")
    ap.add_argument("--list", action="store_true", help="번호가 붙은 단계 목록만 찍고 끝")
    ap.add_argument("--plan", action="store_true",
                    help="현장에서 그대로 복붙할 **전체 명령 목록**을 찍는다(한 줄 = 한 시행)")
    ap.add_argument("--lat", type=float, default=None,
                    help="[dry-run] 가짜 리그 시작 lateral [m]. grid 는 기본 1.6(tilt 25도)")
    ap.add_argument("--part", default="",
                    help="단계의 일부만 돈다(쉼표). 자리를 옮겨야 하는 단계를 나눠 돌 때. "
                         "1 timing: center,upper / 4 rotate: open,gyro,closed,arm,small,beta / "
                         "5 forward: front,oblique,short / 6 creep: stiction,pulse,sweep")
    ap.add_argument("--side", default=None, choices=("L", "R", "l", "r"),
                    help="4 rotate 개루프에서 한쪽만 (L=좌/ccw, R=우/cw)")
    ap.add_argument("--pos", default=None,
                    help="8 grid 에서 그 자리만 [m] (쉼표 가능). 예: --pos 0.5")
    ap.add_argument("--psi", type=float, default=None,
                    help="[dry-run] 가짜 리그 시작 heading [도]")
    ap.add_argument("--solo", action="store_true",
                    help="단계를 공유 세션에 안 넣고 제 폴더에 따로 남긴다")
    ap.add_argument("--dry-run", action="store_true",
                    help="가짜 카메라·자이로·CAN 으로 코드 경로만 돈다")
    ap.add_argument("--yes", action="store_true", help="프롬프트를 자동으로 넘긴다")
    ap.add_argument("--n", type=int, default=0, help="반복 횟수(줄여서 빨리 돌 때)")
    ap.add_argument("--sec", type=float, default=4.0, help="forward 명령 길이 [s]")
    ap.add_argument("--level", type=int, default=67, choices=(67, 97),
                    help="전진 레벨 (97 은 config FORWARD_SLOW 템플릿)")
    ap.add_argument("--start-m", type=float, default=None,
                    help="시작 거리 [m]. 안 주면 단계별 기본값(forward 5.5 / creep 4.5 / "
                         "oblique 5.0 / 나머지 3.5) — 태그 컷 3.3 m 위에 남도록 잡은 값")
    ap.add_argument("--tag-id", type=int, default=D.TAG_ID)
    ap.add_argument("--tag-size", type=float, default=D.TAG_SIZE_M)
    ap.add_argument("--resume", action="store_true", help="state.json 다음 단계부터")
    ap.add_argument("--from", dest="from_mode", default=None, help="이 단계부터")
    ap.add_argument("--skip", default="", help="건너뛸 단계들 (쉼표)")
    ap.add_argument("--root", default=None, help="세션 폴더를 직접 지정")
    ap.add_argument("--keep-going", action="store_true", help="필수 단계가 실패해도 계속")
    ap.add_argument("--no-bag", action="store_true", help="bag 녹화를 하지 않는다")
    ap.add_argument("--no-can", action="store_true", help="CAN 을 안 연다(카메라·IMU 만)")
    ap.add_argument("--no-reset", action="store_true", help="hardware_reset 생략")
    ap.add_argument("--reset", action="store_true",
                    help="가상머신에서도 hardware_reset 을 강행한다 "
                         "(UTM USB 전달이 끊겨 사람이 다시 넘겨야 할 수 있다)")
    ap.add_argument("--auto-exposure", action="store_true", help="노출 고정 대신 자동노출")
    ap.add_argument("--sweep", action="store_true", help="creep 에서 편향 스윕까지")
    ap.add_argument("--kill-stop", action="store_true",
                    help="safety 에서 kill -STOP 2 s(호스트 동결) 시험까지")
    ap.add_argument("--force", action="store_true",
                    help="GLOBAL 전환 실패에도 강행(기록 전용)")
    args = ap.parse_args()
    if getattr(args, "plan", False):
        print_plan()
        return
    if args.list:
        root = _shared_root(args, make=False)
        print(_stage_list(_load_state(root) if root else None, root=root))
        if root:
            print("  세션 폴더: %s" % root)
        return
    args.mode = _resolve_mode(args.mode)
    args.n_given = args.n > 0          # 사람이 --n 을 직접 줬나(바닥을 풀지 판단)
    args.sec_given = any(a == "--sec" or a.startswith("--sec=") for a in sys.argv)
    if args.n <= 0:
        args.n = 10 if args.mode in ("forward", "creep") else 3
    _preflight(args)
    try:
        import signal
        signal.signal(signal.SIGINT, _sigint)     # Ctrl+C = 먼저 정지, 그 다음 종료
    except Exception:
        pass
    try:
        if args.mode == "all":
            asyncio.run(run_all(args))
        else:
            root = _shared_root(args)
            if root:
                print("  세션 폴더: %s  (단계를 따로 돌려도 여기 쌓인다)" % root)
            res = asyncio.run(run_stage(args, args.mode, root=root)) or {}
            state = _note_stage(root, args.mode, res)
            print("\n" + _stage_list(state, root=root))
            print("\n  ━━ CAN 을 놓았다. 이제 **수동 컨트롤러로 차를 옮겨도 된다** ━━")
            i = ORDER.index(args.mode) + 1
            if args.part or args.pos:
                print("  같은 단계의 다른 부분이 남았으면 --part/--pos 를 바꿔 다시 실행")
            if i < len(ORDER):
                print("  다음 단계:  python tools/first_run.py %d      # %s" % (i + 1, ORDER[i]))
            left = [m for m in REQUIRED
                    if (state or {}).get("stages", {}).get(m, {}).get("status") != "done"]
            if left:
                print("  필수인데 아직 안 한 것: %s" % ", ".join(left))
            else:
                print("  필수 단계 전부 끝났다 → 분석:")
                print("    python tools/analyze_first_run.py %s --install" % (root or "<폴더>"))
    except KeyboardInterrupt:
        print("\n  중단. 이어서 하려면:  python tools/first_run.py all --resume")


if __name__ == "__main__":
    main()
