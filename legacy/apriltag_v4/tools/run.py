"""실주행 — 태그를 보며 짧게 가고·서고·돌고·다시 재서 탑재부 입구(포크 끝)까지 (plan 3절 · 2026-09-30 결정 12·13).

    python tools/run.py                       기록 o · 화면 x    실주행·실증 (기본)
    python tools/run.py --show                화면 o             보고 싶을 때
    python tools/run.py --no-record           기록 x             기록이 의심될 때만
    python tools/run.py --video               화면에 그린 그대로 영상으로 (960x540 · 15 fps · video.jsonl 곁줄)
    python tools/run.py --out /Volumes/EXT    기록 루트 (외장 등)
    python tools/run.py --dry-run             CAN 안 보냄. 위와 조합 가능
    python tools/run.py --reuse-sigma         직전 주행이 잰 σ 기준선·태그 기울기를 그대로 (60초 생략). 조명·태그가 바뀌었으면 쓰지 마라

출발 전 게이트 (하나라도 걸리면 출발하지 않는다):
    ① 캘리브 값 (config)               CAM_YAW_OFFSET_DEG 가 None 이면 거부 — 정면의 기준. 회전중심(CAM_TO_ROT_CENTER_M)은 None 이면 회전 상한 5도로
    ② 조향 강도 30                      광운대 씨앗(learn.KWU_SEED)이 30 짜리다 (결정 1)
    ③ CAN 동결 테이블                   6개 프레임 8바이트 (Driver.open → frames.apply_and_verify)
    ④ 시계 검사 clock.check()           카메라 기준점이 잡혔나 (WARMUP_FRAMES)
    ⑤ 자이로 보정                       IMU_BIAS_SEC 정지. 흔들리면 다시
    ⑥ σ 기준선 (출발 뒤, 태그가 보이면)  SIGMA_STILL_S 초 정지 → 밀림 기준선·태그 기울기. 이게 있어야 추정기가 ok 를 낸다 (2026-10-02: 매 주행, 그 자리에서)
SPACE 로 출발 (터미널에서 직접 읽는다 — SSH 터미널도 된다). Ctrl+C · q · ESC 는 **즉시 정지**(먼저 CAN 정지, 그다음 기록).

스레드 (plan 4-2-2): 메인 = 화면·키·감시 / 카메라 = 프레임→검출→추정기 / 제어 = 계획·명령·학습 / SDK 콜백 = 자이로 적분·회전 정지 판정
/ 기록 = 파일 쓰기(Recorder) / CANBusOwner = 송신(광운대 control.py). 제어가 죽으면 driver.stop + 알림.
★카메라를 제어에서 뺀 이유: forward() 가 look_fn 을 5 ms 마다 부르는 폐루프라 직진 중에도 누군가 프레임을 먹어야 한다
(calibrate 의 Rig 와 같은 구조). 회전 중엔 카메라 자세를 안 믿고(결정 7) 끝나면 추정기를 비운다.

마지막 직진 구간(2026-10-02): 접근 계획은 **정렬선**(태그컷 + FINAL_STRAIGHT_M)까지 좌우·정면을 끝내고, 그 안쪽은 태그를 보며
곧장 간다 — 약한 회전(강도 20)으로만 조금씩 고치고, 판단은 PnP 방향·좌우가 아니라 조준 빗나감(planning/aim.py: β·거리)으로 한다.
직진 다리마다 그 프레임들로 조준각 c 를 다시 뽑는다(_aim_leg).
상태: SEARCH(태그 없으면 plan 3-8 50도 훑기) → BASELINE(σ 기준선, 처음 한 번) → DECIDE(plan.decide) → SIDESTEP / STEP(돌고 → 보며 직진) / BACKUP / COMMIT(눈 감고) / 끝.
실패 7종은 전부 정지 + 사람 호출 (plan 3-10): timeout · tag_lost · no_converge(모르겠다·경로 없음·마지막 상한·걸음 초과) ·
gyro_stale · exception(CAN·시계·카메라) · deadman. 기록은 주행을 절대 막지 않는다 (plan 6-6).
"""
import argparse
import json
import math
import os
import select
import signal
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))        # tools/ 의 부모. bootstrap 이 다시 확인한다
from src.bootstrap import ROOT, check_writable, setup, work_root     # noqa: E402

setup()
sys.path.insert(0, str(ROOT / "tools"))

from config import control as C                                      # noqa: E402
from config import detection as D                                    # noqa: E402
from config import imu as I                                          # noqa: E402
from src import limits                                               # noqa: E402
from src.models.control import forward as F                          # noqa: E402
from src.models.control import rotate as R                           # noqa: E402
from src.models.control import sidestep as SS                        # noqa: E402
from src.models.control import learn as LN                           # noqa: E402
from src.models.control.driver import Driver                         # noqa: E402
from src.models.control.learn import Learner                         # noqa: E402
from src.models.detection import estimate as E                       # noqa: E402
from src.models.detection import tag as T                            # noqa: E402
from src.models.planning import aim as AIM                           # noqa: E402
from src.models.planning import plan as PL                           # noqa: E402
from src.utils import clock, hud                                     # noqa: E402
from src.utils.gyro import Gyro, max_rate_dps                        # noqa: E402
from src.utils.record import STOP_REASONS, Recorder                  # noqa: E402

# ── 구현 세부 (config 에 올리지 않는다 — plan 12-4) ─────────────────────
STILL_S = 2 * E.WINDOW_S          # 회전·사이드스텝 뒤 추정 창이 통째로 새 프레임으로 찰 때까지 (calibrate STILL_S 와 같은 근거)
LOST_S = 2 * E.WINDOW_S           # 태그를 이만큼 못 보면 잃은 것 → 탐색 (forward 의 BLIND_MAX_S 1.0 과 같은 크기)
UNCERTAIN_S = 4 * E.WINDOW_S      # "모르겠다" 가 이만큼 이어지면 문턱을 낮추고(3-6 ③), 또 이어지면 사람(④). 밀림은 더 봐도 안 준다
CLOCK_BAD_S = 2.0                 # stale/clock 이 이만큼 이어지면 카메라·시계 이상 → 정지 (MAX_AGE_S 0.3 의 여섯 배)
CLOCK_WAIT_S = 5.0                # 기준점(WARMUP_FRAMES 30장) 대기 상한. 30 fps 면 1초
CALIB_TRIES = 3                   # 보정 중 움직임이면 다시. 이 횟수 안에 안 되면 출발 거부
SEARCH_STEP_DEG = 50.0            # plan 3-8. 가로 화각 70도라 50도씩이면 겹쳐서 안 놓친다
SEARCH_STEPS = int(math.ceil(360.0 / SEARCH_STEP_DEG))   # 한 바퀴 8번. 반대 방향으로 한 바퀴 더 (plan 3-8 ④)
DISPLAY_SIZE = (960, 540)         # 화면·영상은 축소해서 (plan 12-0-1)
DISPLAY_FPS = 15.0
DEADMAN_ABORT_S = -1.0            # 비상정지 때 lease 를 과거로 — rotate/forward 의 대기 루프가 "lease" 로 즉시 빠져나온다
CONTROL_JOIN_S = 15.0             # 제어 스레드가 동작을 접고 나올 때까지 (SETTLE_S 2 + STILL_S 1 + 여유)
FRAME_KEYS = ("lateral", "vertical", "forward", "heading_deg", "distance", "beta_deg", "edge_px", "top_px", "tag_px",
              "row_px", "angle_ok", "reproj_px", "tilt_deg", "tag_roll_deg", "gyro_deg", "err_ratio")
STATE_LEVEL = {"BOOT": "warn", "GATE": "warn", "WAIT": "warn", "SEARCH": "warn", "BASELINE": "warn", "DECIDE": "txt", "ROTATE": "ok",
               "DRIVE": "ok", "SIDESTEP": "ok", "BACKUP": "warn", "COMMIT": "ok", "DONE": "ok", "FAIL": "bad",
               "STOP": "bad"}


class Log:
    """화면 + (기록 폴더가 있으면) log.txt."""

    def __init__(self, path=None):
        self.f = open(path, "a", encoding="utf-8") if path else None

    def __call__(self, text=""):
        print(text, flush=True)
        if self.f is not None:
            try:
                self.f.write("%s %s\n" % (datetime.now().strftime("%H:%M:%S"), text))
                self.f.flush()
            except Exception:
                pass


class _Fail(Exception):
    """제어 스레드가 "정지·사람" 으로 끝낼 때. reason 은 record.STOP_REASONS 중 하나."""

    def __init__(self, reason, why=""):
        super().__init__(why)
        self.reason = reason if reason in STOP_REASONS else "exception"
        self.why = why


# ═══════════════════════════════════════════════════════════════════════
# 장비
# ═══════════════════════════════════════════════════════════════════════
class RealSource:
    """실물: 자이로 먼저, 컬러 나중 (RSUSB 는 먼저 연 쪽이 IMU 를 갖는다 — CLAUDE.md)."""

    def __init__(self, log, raw=True):
        from src.models.detection.image import open_realsense, to_gray
        from src.utils.camera import set_global_time
        from src.utils.tune import camera_tune
        self.gyro = Gyro().start()
        if raw:
            self.gyro.enable_raw()
        try:
            # 9/21 실측(v3 dock.py:403)과 같은 설정 — 자동노출 끔·노출 8.3 ms(60 Hz 반주기)·게인 64·
            # 최신 프레임만(queue 1)·global_time. 빼먹으면 AE 가 프레임마다 흔들리고 큐 16장이 낡은 프레임을 준다
            self.frames, self.intr = open_realsense(stream="color", meta=True, tune=camera_tune())
        except BaseException:
            self.gyro.close()
            raise
        try:
            set_global_time(self.frames.profile)
        except Exception:
            pass
        self.shape = (int(self.intr.height), int(self.intr.width))
        self.detector = T.make_detector(quad_decimate=D.QUAD_DECIMATE)
        self._gray = to_gray
        self.stats = getattr(self.frames, "stats", None)
        log("  카메라 %dx%d · fx %.1f fy %.1f cx %.1f cy %.1f · 자이로 %s" % (
            self.shape[1], self.shape[0], self.intr.fx, self.intr.fy, self.intr.cx, self.intr.cy, self.gyro.axis_note()))

    def detect(self, img):
        return T.detect(self.detector, self._gray(img))

    def close(self):
        for obj in (self.frames, self.gyro):
            try:
                obj.close()
            except Exception:
                pass


def _intr_dict(intr):
    """계획기·기록용 intrinsics dict — limits._intr 이 읽는 이름 (calibrate._intr_dict 와 같다)."""
    d = {"fx": float(intr.fx), "fy": float(intr.fy), "cx": float(intr.cx), "cy": float(intr.cy),
         "w": int(intr.width), "h": int(intr.height),
         "distortion": [float(x) for x in (getattr(intr, "distortion", None) or ())]}
    d["half_fov_deg"] = limits.half_fov_deg(d)
    return d


def _last_baseline(root, skip=None):
    """가장 최근 runs/*/baseline.json (수정 시각 기준, skip 폴더 제외). (dict, path) — 없거나 깨졌으면 (None, None).
    --reuse-sigma 가 쓴다. 언제 것까지 쓸지는 사람이 정한다 — 여기엔 나이 기준이 없다."""
    cands = [c for c in (Path(root) / "runs").glob("*/baseline.json") if skip is None or c.parent != Path(skip)]
    for c in sorted(cands, key=lambda x: x.stat().st_mtime, reverse=True):
        try:
            body = json.loads(c.read_text(encoding="utf-8"))
            n = body.get("noise") or {}
            if all(n.get(k) is not None for k in ("sigma_ref_distance_m", "sigma_drift_lateral_m", "sigma_drift_heading_deg")):
                return body, c
        except Exception:
            continue
    return None, None


def _json_safe(v):
    if hasattr(v, "_asdict"):
        return _json_safe(v._asdict())
    if isinstance(v, dict):
        return {str(k): _json_safe(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_json_safe(x) for x in v]
    if isinstance(v, bool) or v is None or isinstance(v, str):
        return v
    if isinstance(v, (int, float)):
        f = float(v)
        return (int(v) if isinstance(v, int) else f) if math.isfinite(f) else None
    try:
        return float(v)
    except Exception:
        return str(v)


# ═══════════════════════════════════════════════════════════════════════
# 키 — SPACE 출발 · q/ESC 비상정지. 터미널에서 직접 읽는다 (v3 run.py _read_key_tty 와 같은 방식)
# ═══════════════════════════════════════════════════════════════════════
class Keys(threading.Thread):
    def __init__(self):
        super().__init__(name="Keys", daemon=True)
        self.tty = os.name != "nt" and sys.stdin.isatty()
        self.space, self.quit = threading.Event(), threading.Event()
        self._done = threading.Event()               # Thread._stop() 을 가리면 안 된다 — 이름을 피한다

    def run(self):
        if not self.tty:
            return
        import termios
        import tty
        fd = sys.stdin.fileno()
        old = termios.tcgetattr(fd)
        try:
            tty.setcbreak(fd)                                    # 줄 단위·에코 없이. Ctrl+C 는 그대로 SIGINT 다
            while not self._done.is_set():
                r, _, _ = select.select([sys.stdin], [], [], 0.1)
                if not r:
                    continue
                ch = os.read(fd, 1).decode(errors="ignore")
                if ch == " ":
                    self.space.set()
                elif ch in ("\x1b", "q", "Q"):                   # 화살표 키도 \x1b 로 시작한다 — 누르지 말 것
                    self.quit.set()
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old)

    def close(self):
        self._done.set()
        if self.is_alive():
            self.join(timeout=0.5)


# ═══════════════════════════════════════════════════════════════════════
# 도킹 한 판
# ═══════════════════════════════════════════════════════════════════════
class Docking:
    def __init__(self, args, root, run_dir, rec, log):
        self.args, self.root, self.dir, self.rec, self.log = args, root, run_dir, rec, log
        self.dry = bool(args.dry_run)
        self.mode = "dry" if self.dry else ("record" if rec is not None else "no-record")
        self.present = bool(args.show or args.video)
        self.stop_req = threading.Event()        # 모든 루프가 매 바퀴 본다 (plan 4-2-2 ③)
        self.closing = threading.Event()
        self._lock = threading.RLock()           # 추정기 · 최신 화면 패킷
        self._olock = threading.Lock()           # outcome 은 먼저 쓴 쪽이 이긴다
        self.outcome = None
        self._stop_written = False
        self.seeds_from = None                   # 물려받은 seeds.json 경로 (없으면 None)
        self._collect = None                     # 켜져 있으면 카메라 스레드가 프레임 상태를 여기 모은다 (σ 기준선)
        self.driver = self.src = self.gyro = self.est = self.learner = self.planner = self.aim = None
        self.intr = self.shape = None
        self.camclock = clock.CameraClock()
        self.keys = Keys()
        self.state = "BOOT"
        self.decision = None
        self.act = None                          # 진행바용 {"kind": "turn"/"drive", ...}
        self.latest = None                       # 화면 패킷 한 칸 — 큐가 아니라 덮어쓰기 (광운대 runtime_threads)
        self.last_seen_t = None
        self.last_beta = None
        self.n_frames = self.n_seen = 0
        self.fps = self.loop_ms = self.detect_ms = None
        self.t_start = None
        self.started = threading.Event()
        self.ctrl = self.cam = None
        self.cam_error = None
        self.warn_once = set()
        self.video = None
        self.video_side = None
        self.video_k = 0
        self.display_ok = bool(args.show)

    # ── 열기 · 게이트 ──────────────────────────────────────────────────
    def open(self):
        L = self.log
        # ① 캘리브 값 (config) — 카메라 어긋난 각은 정면의 기준이라 없으면 출발하지 않는다. 회전중심은 없으면 회전 상한 5도 (결정 ③)
        if C.CAM_YAW_OFFSET_DEG is None:
            raise SystemExit("!! config CAM_YAW_OFFSET_DEG 가 None — tools/calibrate.py camyaw 로 재서 config/control.py 에 적어라")
        if C.CAM_TO_ROT_CENTER_M is None:
            L("  !! 회전중심을 모른다 (config CAM_TO_ROT_CENTER_M=None) — 회전 상한 %.0f도로 간다. tools/calibrate.py rotcenter 로 재라"
              % C.TURN_MAX_UNKNOWN_CENTER_DEG)
        # ② 조향 강도 30 (결정 1) — 씨앗이 30 짜리다
        if C.ROTATE_JOYSTICK_DEFLECTION != LN.KWU_DEFLECTION:
            raise SystemExit("!! 조향 강도가 %d 다 — 30 이 아니면 출발하지 않는다 (결정 1: 씨앗이 30 짜리)"
                             % C.ROTATE_JOYSTICK_DEFLECTION)
        # ③ CAN 동결 테이블 → 버스. dry 면 안 보낸다
        self.driver = Driver(dry_run=self.dry, recorder=self.rec, log=L)
        self.driver.open()
        # 장비 — 진짜만. 가짜 리그는 없다 (2026-10-01: 가짜 데이터로 판단하지 않는다)
        self.src = RealSource(L, raw=self.rec is not None)
        self.gyro, self.intr, self.shape = self.src.gyro, self.src.intr, self.src.shape
        live = _intr_dict(self.intr)                             # intrinsics 는 카메라에서 읽은 것 (결정 4)
        # 추정기 — σ 기준선·태그 기울기는 출발 뒤 _baseline() 이 그 자리에서 재서 넣는다. 그 전엔 fix 가 no_sigma 다
        self.est = E.Estimator(self.intr, D.TAG_SIZE_M, cam_yaw_offset_deg=C.CAM_YAW_OFFSET_DEG,
                               tag_roll_correction_deg=0.0, noise=None)
        # 학습기 — 광운대 씨앗 위에 지난 주행(또는 calibrate)이 남긴 seeds.json. 회전 하한은 config 실측이 있으면 그걸로
        learned, self.seeds_from = LN.last_seeds(self.root)
        seeds = LN.seeds(learned)
        if C.ROT_FLOOR_DEG is not None:
            seeds["rot_floor_deg"] = C.ROT_FLOOR_DEG
        if C.ROT_FLOOR_FINE_DEG is not None:
            seeds["rot_floor_fine_deg"] = C.ROT_FLOOR_FINE_DEG
        self.learner = Learner(mode=self.mode, seeds=seeds)
        # 계획기 — 기하는 config 의 calibrate 값 + 카메라 intrinsics
        self.planner = PL.Planner(self.learner, intr=live, A=C.CAM_TO_ROT_CENTER_M, b=C.ROT_CENTER_LATERAL_M,
                                  rms_mm=C.ROT_CENTER_RMS_MM, final_m=C.FINAL_STRAIGHT_M)
        self.aim = AIM.Aim()                                     # 직진 다리마다 조준각 c 를 모은다 — 마지막 구간의 판단 근거
        g = self.planner.geo
        L("  씨앗: 광운대 KWU_SEED%s" % ((" + %s" % self.seeds_from) if self.seeds_from else " (지난 seeds.json 없음)"))
        L("  회전중심 %s · 회전 하한 %s · 최소걸음 %.3f m · 뒤공간 %.2f m · 태그컷 폴백 %.2f m (실행 중엔 프레임마다 다시 잰다)"
          % (("앞 %.2f m (RMS %s mm)" % (-g.A, C.ROT_CENTER_RMS_MM)) if g.center_known else "미확정(회전 5도)",
             ("%.2f도" % g.floor) if g.floor else "없음", g.min_step, g.back_space, g.tagcut0))
        L("  마지막 직진 %.2f m (정렬선 = 태그컷 + %.2f) · 약한 회전 강도 %d · 하한 %s · 카메라 좌우 치우침 %+.3f m%s"
          % (C.FINAL_STRAIGHT_M, C.FINAL_STRAIGHT_M, C.ROTATE_FINE_JOYSTICK_DEFLECTION,
             ("%.2f도" % self.learner.rot_floor_fine_deg) if self.learner.rot_floor_fine_deg else "없음",
             C.CAM_LATERAL_OFFSET_M, " (★0 = 가운데라는 가정 — 줄자로 확인)" if C.CAM_LATERAL_OFFSET_M == 0 else ""))
        if self.rec is not None:
            self.rec.snapshot(sys.argv, extra={"tool": "run", "mode": self.mode, "intrinsics": live,
                                               "seeds": _json_safe(seeds),
                                               "seeds_from": str(self.seeds_from) if self.seeds_from else None})
        self.cam = threading.Thread(target=self._cam_loop, name="Camera", daemon=True)
        self.cam.start()
        self.keys.start()
        if self.args.show:
            try:
                cv2.namedWindow("run", cv2.WINDOW_AUTOSIZE)          # 창이 없으면 waitKey 가 안 기다린다
            except Exception as e:
                self.display_ok = False
                L("  !! 화면을 못 띄운다 (%s) — 화면 없이 간다" % e)

    def gates(self):
        """④ 시계 ⑤ 자이로 + 장비 살아 있나. 하나라도 걸리면 SystemExit."""
        L = self.log
        self.state = "GATE"
        t0 = clock.now()
        while not self.camclock.ready and clock.now() - t0 < CLOCK_WAIT_S and self.cam_error is None:
            time.sleep(0.05)
        if self.cam_error is not None:
            raise SystemExit("!! 카메라 스레드가 죽었다: %s" % self.cam_error)
        err = self.camclock.check(L)
        if err:
            raise SystemExit("!! %s (프레임 %d 장)" % (err, self.n_frames))
        L("  카메라 %d 장 · 태그 %d 장" % (self.n_frames, self.n_seen))
        f0 = self.fix()
        if self.n_seen < E.MIN_N or f0.why == "no_tag":
            L("  !! 태그가 안 보인다 (카메라 %d 장 중 태그 %d 장). SPACE 를 누르면 **%d도씩 탐색 회전**부터 시작한다 —"
              % (self.n_frames, self.n_seen, SEARCH_STEP_DEG))
            L("     회전중심이 카메라 앞이라 카메라가 반지름 ~1.5 m 호를 그린다. 주변을 비우고 누를 것 (plan 3-8)")
        if f0.why in ("stale", "clock"):
            raise SystemExit("!! 측정 %s (나이 %.3f s) — 시계·카메라부터" % (f0.why, f0.age_s))
        if self.gyro is None:
            raise SystemExit("!! 자이로가 없다 — 회전 폐루프에 필수 (plan 4-1)")
        rep = None
        sec = I.IMU_BIAS_SEC
        for attempt in range(CALIB_TRIES):
            if self.stop_req.is_set():
                raise SystemExit("중단")
            L("  자이로 보정 %.0f초 — 차를 세워 두고 기다린다%s" % (sec, "" if not attempt else " (다시)"))
            rep = self.gyro.calibrate(sec)
            if self.rec is not None:
                self.rec.event("gyro_calibrate", attempt=attempt,
                               **_json_safe({k: v for k, v in rep.items() if k != "accel_mean"}))
            if not rep.get("moving"):
                break
            L("  !! 보정 중 움직임 (평균 %.2f 도/s) — 세우고 다시" % rep.get("mean_dps", 0.0))
        else:
            raise SystemExit("!! 자이로 보정 %d 번 다 흔들렸다 — 차가 움직이거나 자이로가 이상하다" % CALIB_TRIES)
        L("  자이로: 바이어스 (%s) 도/s · 잡음 %.3f · 드리프트 %.2f 도/분 · 축 %s · %.0f Hz · 유실 %d"
          % (", ".join("%.3f" % b for b in rep["bias_dps"]), rep["noise_dps"], rep["drift_dpm"], rep["axis_src"],
             self.gyro.stats().get("hz", 0.0), self.gyro.stats().get("gaps", 0)))
        if not self.gyro.alive:
            raise SystemExit("!! 자이로 샘플이 안 온다")
        if not self.dry:
            st = self.driver.status()
            if not st.get("ready"):
                raise SystemExit("!! CAN 이 준비되지 않았다: %s" % st.get("last_error"))
        if self.args.show and not hud.hangul_ok():
            L("  (CJK 폰트 없음 — 화면의 한글은 '?' 로 나온다)")

    def wait_start(self):
        """SPACE. 사람이 출발시킨다 — 터미널도 화면도 없으면 거부."""
        L = self.log
        self.state = "WAIT"
        if not self.keys.tty and not self.args.show:
            raise SystemExit("!! 터미널이 없어 SPACE 를 읽을 수 없다 — 실차는 사람이 출발시킨다")
        L("")
        L("  SPACE 를 누르면 출발. q / ESC / Ctrl+C 는 즉시 정지.  (%s%s)"
          % (self.mode, ", 화면 창에서도 된다" if self.args.show else ""))
        while not self.stop_req.is_set():
            if self.keys.space.is_set():
                if self.fix().why == "no_tag":
                    L("  (태그 없음 — 탐색 회전부터 시작)")
                return True
            if self.keys.quit.is_set():
                self.emergency("emergency")
                return False
            k = self._pump_display()
            if k == 32:
                return True
            if k in (27, ord("q")):
                self.emergency("emergency")
                return False
            if not self._pump_display_sleeps:
                time.sleep(1.0 / DISPLAY_FPS)
        return False

    # ── 스레드: 카메라 ─────────────────────────────────────────────────
    def _cam_loop(self):
        """프레임 → 시계 변환 → 검출 → 추정기 → 기록 · 화면 패킷. 추정기는 이 스레드와 제어 스레드가 락으로 나눠 쓴다."""
        last_t = None
        try:
            for i, ts, img in self.src.frames:
                if self.closing.is_set():
                    break
                t_arr = clock.now()
                t0 = time.perf_counter()
                try:
                    dets = self.src.detect(img)
                except Exception as e:
                    dets = []
                    self._warn("detect", "검출 예외: %s" % e)
                det = next((d for d in dets if int(d.tag_id) == D.TAG_ID), None)
                detect_ms = (time.perf_counter() - t0) * 1000.0
                t_cap = self.camclock.see(ts, t_arr)
                accel = self.gyro.accel if self.gyro is not None else None
                moving = self.driver.movement in ("forward", "forward_slow", "backward")
                with self._lock:
                    self.n_frames += 1
                    st = self.est.observe(t_cap, det, self.shape, accel=accel) if det is not None else None
                    if st is not None:
                        self.n_seen += 1
                        self.last_seen_t, self.last_beta = t_cap, st.get("beta_deg")
                        if self._collect is not None:
                            self._collect.append(dict(st, t=t_cap, gyro_deg=(self.gyro.angle_deg if self.gyro is not None else None)))
                    fix = self.est.fix(moving=moving)
                if last_t is not None:
                    self.loop_ms = (t_arr - last_t) * 1000.0
                    self.fps = 1.0 / max(1e-3, t_arr - last_t) if self.fps is None else 0.9 * self.fps + 0.1 / max(1e-3, t_arr - last_t)
                last_t = t_arr
                self.detect_ms = detect_ms
                if self.rec is not None and not self._stop_written:
                    row = {"t": t_cap, "wall": clock.wall_from(t_cap), "i": i, "state": self.state,
                           "seen": det is not None, "lat_ms": round((t_arr - t_cap) * 1000.0, 1),
                           "detect_ms": round(detect_ms, 1), "loop_ms": None if self.loop_ms is None else round(self.loop_ms, 1),
                           "why": fix.why}
                    if st is not None:
                        row.update({k: st.get(k) for k in FRAME_KEYS})
                        row.update(tag_px=T.tag_pixel_size(det), row_px=float(det.center[1]),
                                   gyro_deg=(self.gyro.angle_deg if self.gyro is not None else None))
                    self.rec.frame(**row)
                    if self.gyro is not None:
                        self.rec.imu(self.gyro.drain_raw())
                if self.present:
                    self._publish(img, det, fix)
        except Exception as e:
            self.cam_error = "%s: %s" % (type(e).__name__, e)
            self.log("  !! 카메라 스레드가 죽었다: %s" % self.cam_error)
            if self.started.is_set():
                self.fail("exception", "카메라 스레드: " + self.cam_error)

    def _publish(self, img, det, fix):
        """화면 패킷 한 칸. 축소본을 여기서 만든다 — SDK 버퍼를 메인 스레드까지 들고 가지 않는다."""
        try:
            small = cv2.resize(img, DISPLAY_SIZE, interpolation=cv2.INTER_AREA)
            if det is not None:
                s = DISPLAY_SIZE[0] / float(self.shape[1])
                pts = np.asarray(det.corners, dtype=np.float64) * s
                cv2.polylines(small, [np.round(pts).astype(np.int32).reshape(-1, 1, 2)], True, (0, 0, 255), 2, 16)
            with self._lock:
                self.latest = (small, self._info(fix), getattr(img, "frame_number", None), fix.t_capture)
        except Exception as e:
            self._warn("publish", "화면 패킷 실패: %s" % e)

    def _info(self, fix):
        """hud.draw 에 줄 것. 계산은 통로 반폭 하나뿐 (표시용)."""
        p, geo = self.planner.progress, self.planner.geo
        run = {"mode": self.mode, "steps": p.steps,
               "elapsed_s": (clock.now() - self.t_start) if self.t_start else None}
        if p.fine_since is not None:
            run["corrections"], run["fine_elapsed_s"] = p.corrections, self.planner.fine_elapsed_s()
        corridor = None
        if fix.ok:
            half = limits.corridor_half_m(fix.forward_m, geo.intr, geo.cut)
            corridor = {"half_m": half, "inside": abs(fix.lateral_m) <= half, "lateral_m": fix.lateral_m,
                        "cut_m": geo.tagcut, "cut_from": geo.cut_from}
        d = self.decision
        plan = None if d is None else {"kind": d.kind, "turn_deg": d.turn_deg, "drive_m": d.drive_m, "why": d.why}
        prog, act = None, self.act
        if act is not None:
            if act["kind"] == "turn":
                prog = {"kind": "turn", "done": self.gyro.angle_deg - act["start"], "target": act["target"], "unit": "deg"}
            elif fix.ok:
                prog = {"kind": "drive", "done": act["start_fwd"] - fix.forward_m, "target": act["target"], "unit": "m"}
        warnings = []
        if self.rec is not None and self.rec.log_errors:
            warnings.append(("warn", "record errors %d: %s" % (self.rec.log_errors, self.rec.last_error)))
        if self.gyro is not None and not self.gyro.alive:
            warnings.append(("bad", "GYRO STALE"))
        if self.outcome is not None:
            warnings.append(("bad" if self.outcome["outcome"] != "normal" else "ok",
                             "%s %s" % (self.outcome["outcome"], self.outcome.get("why") or "")))
        return {"state": (self.state, STATE_LEVEL.get(self.state, "txt")), "fix": fix, "corridor": corridor, "plan": plan,
                "progress": prog, "learner": self.learner, "clock": self.camclock, "gyro": self.gyro,
                "can": self.driver, "warnings": warnings, "fps": self.fps, "latency_ms": self.detect_ms,
                "loop_ms": self.loop_ms, "run": run}

    # ── 스레드: 메인 (화면·키·감시) ────────────────────────────────────
    _pump_display_sleeps = False

    def _pump_display(self):
        """패킷이 있으면 그리고(화면·영상) 키를 돌려준다. 화면이 없으면 -1."""
        packet = None
        with self._lock:
            packet, self.latest = self.latest, None
        if packet is not None and self.present:
            small, info, fn, t_cap = packet
            img = hud.draw(small, info)
            if self.video is not None:
                try:
                    self.video.write(img)
                    self.video_side.write(json.dumps({"k": self.video_k, "i": fn, "t": t_cap,
                                                      "wall": clock.wall_from(t_cap)}) + "\n")
                    self.video_k += 1
                except Exception as e:
                    self._warn("video", "영상 쓰기 실패: %s" % e)
            if self.display_ok:
                try:
                    cv2.imshow("run", img)
                except Exception as e:
                    self.display_ok = False
                    self.log("  !! 화면을 못 띄운다 (%s) — 화면 없이 간다" % e)
        if self.display_ok:
            self._pump_display_sleeps = True
            return cv2.waitKey(max(1, int(1000.0 / DISPLAY_FPS))) & 0xFF
        self._pump_display_sleeps = False
        return -1

    def loop(self):
        """출발 뒤 메인 스레드. 제어가 끝나거나 정지 요청이 올 때까지."""
        while not self.stop_req.is_set():
            if self.keys.quit.is_set():
                self.emergency("emergency")
                break
            k = self._pump_display()
            if k in (27, ord("q")):
                self.emergency("emergency")
                break
            if self.ctrl is not None and not self.ctrl.is_alive():
                break
            if not self._pump_display_sleeps:
                time.sleep(1.0 / DISPLAY_FPS)

    # ── 정지 ───────────────────────────────────────────────────────────
    def emergency(self, reason):
        """사람이 세움 (Ctrl+C · q · ESC). **CAN 정지가 먼저**, 기록은 나중 (plan 6-6 ④). 어느 스레드에서 불러도 된다."""
        try:
            self.driver.abort(reason)                       # CAN 정지 + 이후 모든 set/lease 차단 (검토 지적)
        except Exception:
            pass
        with self._olock:
            if self.outcome is None:
                self.outcome = {"outcome": reason, "fail_reason": reason, "why": "사람이 세웠다"}
                self.log("!! %s — 정지" % reason)
        self.state = "STOP"
        self.stop_req.set()

    def fail(self, reason, why=""):
        """실패 7종 → 정지 + 사람 호출 (plan 3-10). 제어 스레드가 부른다."""
        try:
            self.driver.stop(why or reason)
        except Exception:
            pass
        with self._olock:
            first = self.outcome is None
            if first:
                self.outcome = {"outcome": "fail", "fail_reason": reason, "why": why}
        if first:
            self.state = "FAIL"
            self.log("")
            self.log("!! 정지 (%s): %s — **사람을 부른다**" % (reason, why))
            self._write_stop(reason, why)
        self.stop_req.set()

    def _write_stop(self, reason, why):
        if self.rec is None or self._stop_written:
            return
        self._stop_written = True
        try:
            self.rec.stop(reason, note=why, state=self.state, t=clock.now(), wall=time.time(),
                          elapsed_s=(clock.now() - self.t_start) if self.t_start else None,
                          steps=self.planner.progress.steps if self.planner else None)
        except Exception as e:
            self.log("  !! 정지 기록 실패: %s" % e)

    # ── 제어 스레드 ───────────────────────────────────────────────────
    def start_control(self):
        self.started.set()
        self.t_start = clock.now()
        self.ctrl = threading.Thread(target=self._control, name="Control", daemon=True)
        self.ctrl.start()

    def _control(self):
        """어떤 예외든 잡아서 전체 정지를 건다 — 스레드가 조용히 죽으면 차가 계속 간다 (plan 4-2-2 ①)."""
        try:
            self._dock()
        except _Fail as e:
            self.fail(e.reason, e.why)
        except BaseException as e:
            self.log(traceback.format_exc())
            self.fail("exception", "%s: %s" % (type(e).__name__, e))
        finally:
            try:
                self.driver.stop("control end")
            except Exception:
                pass
            self.stop_req.set()

    def fix(self, moving=False):
        with self._lock:
            return self.est.fix(moving=moving)

    def _set_state(self, s):
        if s != self.state:
            self.state = s
            if self.rec is not None and not self._stop_written:
                try:
                    self.rec.event("state", state=s, t=clock.now(), thread=threading.current_thread().name)
                except Exception:
                    pass

    def _warn(self, key, text):
        if key not in self.warn_once:
            self.warn_once.add(key)
            self.log("  !! " + text)

    def _dock(self):
        L = self.log
        L("")
        L("== 출발 (%s)  시간 상한 %.0f s · 걸음 %d · 마지막 단계 보정 %d회 / %.0f s"
          % (self.mode, C.TIME_LIMIT_S, C.MAX_STEPS, C.MAX_CORRECTIONS, C.FINE_TIME_LIMIT_S))
        if self.rec is not None:
            self.rec.event("start", t=self.t_start, wall=time.time(), mode=self.mode)
        uncertain_since, relaxed, bad_since = None, False, None
        while not self.stop_req.is_set():
            self._limits()
            f = self.fix()
            now = clock.now()
            # 태그가 없다 — 잠깐이면 기다리고, 오래면 훑는다 (plan 3-8)
            if f.why == "no_tag":
                seen = self.last_seen_t
                if (seen is None and now - self.t_start > LOST_S) or (seen is not None and now - seen > LOST_S):
                    if not self._search():
                        raise _Fail("tag_lost", "태그를 영영 못 찾음 (%d도 × %d × 2 바퀴)" % (SEARCH_STEP_DEG, SEARCH_STEPS))
                    uncertain_since = None
                    continue
                time.sleep(F.POLL_S)
                continue
            # 시계·카메라 이상 — 잠깐은 넘기고, 이어지면 정지 (실패 ⑤ 급)
            if f.why in ("stale", "clock"):
                bad_since = bad_since or now
                if now - bad_since > CLOCK_BAD_S:
                    raise _Fail("exception", "측정 %s 가 %.1f s 이어짐 (나이 %.3f s) — 카메라·시계 이상" % (f.why, now - bad_since, f.age_s))
                time.sleep(F.POLL_S)
                continue
            bad_since = None
            if not self.est.drift.ok:
                self._baseline()                                  # ⑥ 출발 뒤 처음 한 번 — 그 자리에서 σ 기준선 (2026-10-02)
                continue
            if f.why == "no_sigma":
                raise _Fail("no_converge", "σ 기준선이 없다 — 출발 전 기준선 측정이 안 됐다")

            self._set_state("DECIDE")
            d = self.planner.decide(f, aim=self.aim)
            self.decision = d
            self._log_decision(f, d)
            if d.kind == "stop":
                raise _Fail(d.stop_reason or PL.STOP_REASON, d.why)
            if d.kind == "commit":
                self._commit(d)
                return
            if d.kind == "uncertain":
                if d.drive_m > 0 and self._advance(f, d):
                    uncertain_since = None
                    continue
                # 더 본다 → 기준을 낮춘다 → 사람 (plan 3-6 ②③④). 밀림은 더 봐도 안 주니 오래 끌지 않는다
                uncertain_since = uncertain_since or now
                waited = now - uncertain_since
                if waited > 2 * UNCERTAIN_S:
                    raise _Fail("no_converge", "'모르겠다' 가 %.0f s 넘게 안 풀림 (%s)" % (waited, d.why))
                if waited > UNCERTAIN_S and not relaxed:
                    self.planner.relax()
                    relaxed = True
                    L("       기준을 낮춘다: %.1fσ → %.1fσ (plan 3-6 ③)" % (C.UNCERTAIN_FACTOR, PL.RELAXED_FACTOR))
                time.sleep(E.WINDOW_S)
                continue
            uncertain_since = None
            if d.kind == "sidestep":
                ok = self._sidestep(d)
            elif d.kind == "step":
                ok = self._step(d)
            elif d.kind == "backup":
                ok = self._backup(d)
            else:
                raise _Fail("exception", "모르는 결정 %s" % d.kind)
            self.planner.executed(d, ok)
        # 정지 요청으로 나왔다 (사람). outcome 은 emergency() 가 적었다

    def _baseline(self):
        """⑥ σ 기준선 — 출발 뒤 태그가 보이는 자리에서 SIGMA_STILL_S 초 가만히. 0.5초 창 평균의 흔들림(밀림)이 추정기의 σ 가 되고
        (plan 3-6 · 결정 8), 가속도계로 본 태그 기울기도 같이 넣는다. 2026-10-02 부터 before_run 대신 매 주행 여기서 — 그 자리·
        조명·거리의 값이라 더 맞다. 재는 동안 차가 움직였으면(자이로 각속도 > IMU_MOVING_DPS) 한 번 더. 그래도 안 되면 정지·사람. 끝나면 시간 상한이 다시 돈다."""
        L = self.log
        self._set_state("BASELINE")
        if self.args.reuse_sigma:
            saved, path = _last_baseline(self.root, skip=self.dir)
            if saved is not None:
                noise = E.Noise(**{k: saved["noise"].get(k) for k in E.Noise.__dataclass_fields__})
                roll = saved.get("tag_roll_deg")
                with self._lock:
                    self.est.set_noise(noise)
                    if roll is not None:
                        self.est.set_tag_roll(roll)
                    else:
                        self.est.reset()
                L("       σ 기준선 재사용 (--reuse-sigma): %s 에 잰 것 — 기준 거리 %.2f m · 밀림 좌우 %.1f mm · 방향 %.3f도 · 태그 기울기 %s"
                  % (saved.get("date") or "?", noise.sigma_ref_distance_m, noise.sigma_drift_lateral_m * 1e3,
                     noise.sigma_drift_heading_deg, ("%.2f도" % roll) if roll is not None else "-"))
                L("       (%s. 조명·태그가 바뀌었으면 스위치 없이 다시 돌려라)" % path)
                self._save_baseline(dict(saved, reused_from=str(path)))
                if self.rec is not None and not self._stop_written:
                    self.rec.event("sigma_baseline", reused=True, source=str(path), date=saved.get("date"),
                                   **_json_safe({"noise": noise.__dict__, "tag_roll_deg": roll}))
                return
            L("       --reuse-sigma 인데 지난 기준선(runs/*/baseline.json)이 없다 — 지금 잰다")
        sec = C.SIGMA_STILL_S
        b = rows = None
        moved = 0.0
        for attempt in range(2):
            L("       σ 기준선 — %.0f초 가만히 (차·사람 모두. 카메라 앞을 지나가지 마라)%s" % (sec, " (다시)" if attempt else ""))
            self.driver.stop("baseline")
            with self._lock:
                self.est.reset()
                self._collect = []
            t0 = clock.now()
            while clock.now() - t0 < sec:
                if self.stop_req.is_set():
                    with self._lock:
                        self._collect = None
                    return
                time.sleep(0.1)
            with self._lock:
                rows, self._collect = self._collect, None
            # 움직였나는 각속도로 (보정과 같은 기준 IMU_MOVING_DPS). 적분 각도 차는 드리프트(실측 0.6도/분)라 60초 창엔 못 쓴다
            moved = max_rate_dps([r["t"] for r in rows], [r.get("gyro_deg") for r in rows], E.WINDOW_S)
            b = E.baseline_from_rows(rows)
            if self.rec is not None and not self._stop_written:
                try:
                    self.rec.event("sigma_baseline", attempt=attempt, n=len(rows), max_rate_dps=moved, ok=b is not None,
                                   **(_json_safe({"noise": b.noise.__dict__, "lateral": b.lateral._asdict(),
                                                  "heading": b.heading._asdict(),
                                                  "distance": b.distance._asdict() if b.distance else None,
                                                  "n_clean": b.n_clean, "ambiguous_rate": b.ambiguous_rate,
                                                  "tag_roll_deg": b.tag_roll_deg}) if b else {}))
                except Exception as e:
                    self._warn("baseline_rec", "기준선 기록 실패: %s" % e)
            if b is None:
                raise _Fail("no_converge", "σ 기준선을 못 냈다 — %.0f초에 프레임 %d 장 (태그가 안 보였거나 두 해가 헷갈렸다)" % (sec, len(rows)))
            if moved > I.IMU_MOVING_DPS:
                L("       !! 재는 동안 %.2f 도/s 로 움직였다 (기준 %.1f) — 차가 흔들렸다. 다시" % (moved, I.IMU_MOVING_DPS))
                continue
            break
        else:
            raise _Fail("no_converge", "σ 기준선 — 두 번 다 차가 움직였다 (%.2f 도/s)" % moved)
        with self._lock:
            self.est.set_noise(b.noise)
            if b.tag_roll_deg is not None:
                self.est.set_tag_roll(b.tag_roll_deg)              # reset 까지 한다
            else:
                self.est.reset()
        n = b.noise
        L("       %d 장 (헷갈린 장 %.0f %%) · 기준 거리 %.2f m · 밀림 좌우 %.1f mm · 방향 %.3f도 · 거리 %s · 태그 기울기 %s"
          % (b.n, b.ambiguous_rate * 100, n.sigma_ref_distance_m, n.sigma_drift_lateral_m * 1e3, n.sigma_drift_heading_deg,
             ("%.1f mm" % (n.sigma_drift_distance_m * 1e3)) if n.sigma_drift_distance_m is not None else "-",
             ("%.2f도" % b.tag_roll_deg) if b.tag_roll_deg is not None else "- (가속도계 없음)"))
        L("       (좌우 블록평균 sd %.1f mm · 떨림 %.1f mm. 9/21 실측은 밀림 31 mm — 많이 다르면 조명·태그 판을 의심)"
          % (b.lateral.block_sd * 1e3, b.lateral.resid_sd * 1e3))
        self._save_baseline({"noise": n.__dict__, "tag_roll_deg": b.tag_roll_deg, "n": b.n, "n_clean": b.n_clean,
                             "ambiguous_rate": b.ambiguous_rate, "seconds": sec, "reused_from": None,
                             "date": datetime.now().isoformat(timespec="seconds")})
        self.t_start = clock.now()                               # 시간 상한은 여기서부터 — 기준선 재는 1분은 주행이 아니다
        if self.rec is not None and not self._stop_written:
            self.rec.event("start", t=self.t_start, wall=time.time(), mode=self.mode, after="baseline")

    def _save_baseline(self, body):
        """기준선을 이 주행 폴더에 baseline.json 으로 — 다음 주행의 --reuse-sigma 가 읽는다. 기록을 껐으면 안 남는다."""
        if self.dir is None:
            return
        try:
            (self.dir / "baseline.json").write_text(json.dumps(_json_safe(body), ensure_ascii=False, indent=1))
        except Exception as e:
            self._warn("baseline_save", "baseline.json 저장 실패: %s" % e)

    def _limits(self):
        """루프마다. 시간·걸음·자이로·CAN·카메라."""
        now = clock.now()
        p = self.planner.progress
        if now - self.t_start > C.TIME_LIMIT_S:
            raise _Fail("timeout", "%.0f s 초과" % C.TIME_LIMIT_S)
        if p.steps >= C.MAX_STEPS:
            raise _Fail("no_converge", "걸음 %d 회 초과 — 수렴 못 함" % C.MAX_STEPS)
        if self.gyro is None or not self.gyro.alive:
            raise _Fail("gyro_stale", "자이로 샘플이 %.1f s 넘게 없다" % I.IMU_STALE_SEC)
        if self.cam_error is not None:
            raise _Fail("exception", "카메라 스레드: " + self.cam_error)
        if not self.dry:
            st = self.driver.status()
            if not st.get("ready") or st.get("errors"):
                raise _Fail("exception", "CAN: ready %s · errors %s · %s" % (st.get("ready"), st.get("errors"), st.get("last_error")))
        if self.rec is not None and self.rec.log_errors:
            self._warn("rec", "기록 오류 %d (주행은 계속): %s" % (self.rec.log_errors, self.rec.last_error))

    def _log_decision(self, f, d):
        p = self.planner.progress
        self.log("  [%5.1fs] 좌우 %+.3f±%.3f · 방향 %+.2f±%.2f · 거리 %.2f (컷 %.2f%s) · β %+.2f · n %d%s | %s%s"
                 % (clock.now() - self.t_start, f.lateral_m, f.lateral_sigma_m, f.heading_deg, f.heading_sigma_deg,
                    f.forward_m, d.cut_m, "" if d.cut_from == "live" else "?", f.beta_deg, f.n, (" amb %d" % f.ambiguous) if f.ambiguous else "", d.summary(),
                    (" [보정 %d/%d · %.0fs/%.0f]" % (p.corrections, C.MAX_CORRECTIONS, self.planner.fine_elapsed_s(),
                                                    C.FINE_TIME_LIMIT_S)) if p.fine_since is not None else ""))
        if d.waypoints:                                              # 계획한 점들 — 다음 판단 줄의 좌우·거리·방향과 견주면 된다
            self.log("            경로점(좌우 m, 거리 m, 방향 도): " + " → ".join("(%+.2f, %.2f, %+.0f)" % tuple(w) for w in d.waypoints))
        if self.rec is not None and not self._stop_written:
            try:
                self.rec.event("decision", decision=d.kind, why=d.why, turn_deg=d.turn_deg, drive_m=d.drive_m,
                               movement=d.movement, fwd_target_m=d.fwd_target_m, at_cut=d.at_cut,
                               to_ref=d.to_ref, ref_m=d.ref_m, cut_m=d.cut_m, line_m=d.line_m, cut_from=d.cut_from,
                               final=d.final, fine=d.fine, miss_m=d.miss_m, miss_sigma_m=d.miss_sigma_m,
                               aim_c_deg=self.aim.c()[0], aim_c_sigma_deg=self.aim.c()[1], aim_legs=len(self.aim.fits),
                               corridor_half_m=d.corridor_half_m, inside=d.inside, margin_m=d.margin_m,
                               predicted_m=d.predicted_m, route=_json_safe(d.route), waypoints=_json_safe(d.waypoints), complete=d.complete,
                               correction=d.correction, stop_reason=d.stop_reason, steps=p.steps,
                               corrections=p.corrections, k_sigma=p.k_sigma, t=clock.now(),
                               fix=_json_safe({k: getattr(f, k) for k in (
                                   "why", "lateral_m", "heading_deg", "forward_m", "distance_m", "beta_deg",
                                   "lateral_sigma_m", "heading_sigma_deg", "lateral_drift_sigma_m", "lateral_fast_sigma_m",
                                   "heading_drift_sigma_deg", "heading_fast_sigma_deg", "drift_from", "n", "n_fit",
                                   "ambiguous", "age_s", "t_capture", "edge_px", "top_px", "vertical_m", "closing_mps")}))
            except Exception as e:
                self._warn("decision", "결정 기록 실패: %s" % e)

    # ── 동작들 ────────────────────────────────────────────────────────
    def _after_rotation(self):
        """회전 중 프레임은 안 믿는다 (결정 7) — 창을 비우고 새 창이 찰 때까지."""
        with self._lock:
            self.est.reset()
        time.sleep(STILL_S)

    def _check_rotation(self, res):
        """회전 결과 중 정지·사람 감인 것. 그 외는 부르는 쪽이 재계획한다."""
        if res.reason == "wrong_way":
            raise _Fail("exception", "CAN 회전 부호가 뒤집혔다 — driver.MOVES 의 rotate 두 줄을 맞바꿔라")
        if res.reason == "gyro_stale":
            raise _Fail("gyro_stale", "회전 중 자이로 끊김")
        if res.reason == "lease":
            if self.stop_req.is_set():
                return
            raise _Fail("deadman", "회전 중 데드맨이 세웠다")
        if res.reason in ("watchdog", "max_angle", "settle_timeout"):
            self.log("       !! 회전이 %s 로 끝났다 — 다시 재고 계획한다" % res.reason)

    def _check_leg(self, res):
        if res.reason == "lease":
            if self.stop_req.is_set():
                return
            raise _Fail("deadman", "직진 중 데드맨이 세웠다")
        if res.reason == "watchdog":
            self.log("       !! 직진이 워치독으로 끝났다 — 다시 재고 계획한다")

    def _rotate(self, deg, **kw):
        self._set_state("ROTATE")
        self.act = {"kind": "turn", "target": deg, "start": self.gyro.angle_deg}
        try:
            res = R.rotate(self.driver, self.gyro, self.learner, deg, self.rec, self.log, **kw)
        finally:
            self.act = None
        self._after_rotation()
        self._check_rotation(res)
        return res

    def _look_step(self, d, sign=1.0):
        """forward() 가 볼 것 — 남은 거리 = (법선 거리 − 목표 법선 거리) ÷ cos(방향), 가는 방향으로 +.
        정렬선·태그컷에서 끝나는 다리(d.to_ref)는 목표를 **프레임마다 다시 잰 그 선**에 붙인다 — 오르막·요철이면 목표가 따라 움직인다."""
        geo = self.planner.geo

        def look():
            f = self.fix(moving=True)
            if not f.ok:
                return F.Look(ok=False, t_capture=f.t_capture)
            fwd_target = d.fwd_target_m
            if d.to_ref:
                geo.see(f)                                            # 태그컷(과 정렬선)을 이 프레임으로
                ref_now = geo.cut if d.to_ref == "line" else geo.tagcut
                fwd_target = ref_now + (d.fwd_target_m - d.ref_m)     # 결정 때의 "그 선 + 조금" 을 지금 선 기준으로
            c = max(0.5, math.cos(math.radians(f.heading_deg)))       # 60도 넘게 비스듬하면 그 이상 늘리지 않는다
            sig = f.distance_sigma_m if math.isfinite(f.distance_sigma_m) else 0.0   # 밀림 기준선 있으면 hypot, 없으면 한 장 흔들림 — /√n 값은 가짜 출발을 잡는다 (검토 지적)
            return F.Look(remaining_m=sign * (f.forward_m - fwd_target) / c, t_capture=f.t_capture, ok=True, sigma_m=sig)
        return look

    def _drive(self, d, look, movement="forward"):
        self._set_state("BACKUP" if movement == "backward" else "DRIVE")
        f0 = self.fix()
        self.act = {"kind": "drive", "target": d.drive_m, "start_fwd": f0.forward_m if f0.ok else 0.0}
        with self._lock:
            self._collect = []                                        # 이 다리의 프레임들 — 조준각 c 를 뽑는다
        rows = []
        try:
            res = F.forward(self.driver, self.learner, look, d.drive_m, movement=movement, rec=self.rec, log=self.log)
        finally:
            self.act = None
            with self._lock:
                rows, self._collect = (self._collect or []), None
        self._aim_leg(rows)
        self._check_leg(res)
        return res

    def _aim_leg(self, rows):
        """직진 다리 하나의 (거리, β, 자이로)에 직선을 맞춰 조준각 c 를 모은다 (planning/aim.py). 계산이 틀어져도 주행은 간다."""
        try:
            fit = self.aim.add_leg(rows, self.planner.geo.dz)
            if fit is None:
                return
            c, sc = self.aim.c()
            self.log("          조준각: 이 다리 c %+.2f±%.2f도 (%d 장 · %.2f m · 잔차 %.1f mm) → 전체 %+.2f±%.2f도 (다리 %d개)"
                     % (fit["c_deg"], fit["c_sigma_deg"], fit["n"], fit["span_m"], fit["resid_mm"], c, sc, len(self.aim.fits)))
            if self.rec is not None and not self._stop_written:
                self.rec.event("aim_leg", t=clock.now(), c_all_deg=c, c_all_sigma_deg=sc, legs=len(self.aim.fits), **_json_safe(fit))
        except Exception as e:
            self._warn("aim_leg", "조준각 계산 실패: %s" % e)

    def _step(self, d):
        ok = True
        if abs(d.turn_deg) > 1e-9:
            res = self._rotate(d.turn_deg, fine=d.fine)               # 마지막 구간의 회전은 약한 강도
            if not res.ok and res.reason not in ("zero", "too_small"):
                ok = False
        if ok and d.drive_m > 0 and not self.stop_req.is_set():
            res = self._drive(d, self._look_step(d))
            ok = res.done and res.reason in ("predicted", "settle_timeout", "settle_blind")
        return ok

    def _advance(self, f, d):
        """못 믿을 때(few) β·거리만으로 곧장 조금 (plan 3-6 ①). 남은 거리는 태그까지의 수평 거리로 잰다."""
        dz = self.planner.geo.dz                                 # 잰 높이차 — planner 와 같은 출처

        def rng_of(fx):
            return math.sqrt(max(0.0, fx.distance_m ** 2 - dz * dz))
        target = rng_of(f) - d.drive_m

        def look():
            fx = self.fix(moving=True)
            ok = fx.why in ("ok", "few") and math.isfinite(fx.distance_fast_sigma_m)
            return F.Look(remaining_m=rng_of(fx) - target if ok else 0.0, t_capture=fx.t_capture, ok=ok,
                          sigma_m=fx.distance_fast_sigma_m if ok else 0.0)
        self._set_state("DRIVE")
        self.act = {"kind": "drive", "target": d.drive_m, "start_fwd": f.forward_m}
        try:
            res = F.forward(self.driver, self.learner, look, d.drive_m, movement="forward", rec=self.rec, log=self.log)
        finally:
            self.act = None
        self._check_leg(res)
        return res.done

    def _sidestep(self, d):
        self._set_state("SIDESTEP")
        res = SS.execute(self.driver, self.gyro, self.learner, d.sidestep, self.rec, self.log)
        self._after_rotation()                                        # 끝난 뒤 estimator.reset() 은 부르는 쪽이 (sidestep.py)
        for r in (res.turn1, res.turn2):
            if r is not None:
                self._check_rotation(r)
        if res.leg is not None:
            self._check_leg(res.leg)
        if not res.ok and not self.stop_req.is_set():
            raise _Fail("no_converge", "사이드스텝 실패 (%s) — 그 자리 정지·사람" % res.why)
        return res.ok

    def _backup(self, d):
        """마지막 단계의 최후의 보루 — 한 번 (결정 6). 태그가 보이면 보며, 아니면 시간으로. 후진 모델 없으면 사람."""
        res = self._drive(d, self._look_step(d, sign=-1.0), movement="backward")
        if res.reason == "no_tag":
            self._set_state("BACKUP")
            res = F.forward_timed(self.driver, self.learner, d.drive_m, "backward", self.rec, self.log)
            self._check_leg(res)
        if res.reason == "no_model":
            raise _Fail("no_converge", "후진 속도를 모른다 — 후진 못 함, 정지·사람 (learn.KWU_SEED 에 187 씨앗이 없다)")
        return res.done

    def _commit(self, d):
        """눈 감고 BLIND (포크 끝이 입구까지). 그 뒤 정지 = 1차 목표."""
        self._set_state("COMMIT")
        self.log("       == 눈 감고 %.2f m — 포크 끝이 태그면 앞 %.2f m 에서 선다 (%s)"
                 % (d.drive_m, C.STANDOFF_M, d.why))
        res = F.forward_timed(self.driver, self.learner, d.drive_m, "forward", self.rec, self.log)
        self._check_leg(res)
        if not res.timed_ok:
            if self.stop_req.is_set():
                return
            raise _Fail("exception", "눈 감고 직진이 %s 로 끝났다" % res.reason)
        self.driver.stop("docked")
        with self._olock:
            if self.outcome is None:
                self.outcome = {"outcome": "normal", "fail_reason": None,
                                "why": "포크 끝이 입구 (1차 목표). 진입 여유 %s" % (("%+.0f mm" % (d.margin_m * 1e3)) if d.margin_m is not None else "-")}
        self._set_state("DONE")
        self.log("")
        self.log("== 도킹 끝 — %s (%.0f s, 걸음 %d, 보정 %d)"
                 % (self.outcome["why"], clock.now() - self.t_start, self.planner.progress.steps, self.planner.progress.corrections))
        self._write_stop("normal", self.outcome["why"])

    def _search(self):
        """태그를 놓쳤을 때 — 마지막에 보이던 쪽으로 50도씩 돌고 서서 확인, 한 바퀴. 반대로 한 바퀴 더 (plan 3-8)."""
        self._set_state("SEARCH")
        sign = -1.0 if (self.last_beta or 0.0) > 0 else 1.0          # β + 는 화면 오른쪽 → 오른쪽(시계, −)으로
        self.log("       == 태그 없음 — %d도씩 %s부터 훑는다. 회전중심이 앞이라 카메라가 반지름 ~1.5 m 원을 그린다 — 주변을 비워라"
                 % (SEARCH_STEP_DEG, "오른쪽" if sign < 0 else "왼쪽"))
        for rnd in range(2):
            for k in range(SEARCH_STEPS):
                if self.stop_req.is_set():
                    return False
                self._limits()
                res = self._rotate(sign * SEARCH_STEP_DEG, cap_deg=SEARCH_STEP_DEG, safety_s=C.SIDESTEP_ROT_SAFETY_S)
                if not res.done and res.reason in ("gyro_uncalibrated", "too_big", "too_small", "zero"):
                    raise _Fail("exception", "탐색 회전 거부 (%s)" % res.reason)
                f = self.fix()
                if f.why != "no_tag" and f.n >= E.MIN_N:
                    self.log("       태그 찾음 (β %+.1f도, %d 장) — 처음부터 다시 계획" % (f.beta_deg, f.n))
                    return True
            sign = -sign
            self.log("       한 바퀴 돌았는데 없다 — 반대 방향" if rnd == 0 else "")
        return False

    # ── 닫기 ───────────────────────────────────────────────────────────
    def summary(self):
        p = self.planner.progress if self.planner else None
        out = dict(self.outcome or {"outcome": "refused", "fail_reason": None, "why": "출발 전"})
        out.update({"mode": self.mode,
                    "t_start": self.t_start, "elapsed_s": (clock.now() - self.t_start) if self.t_start else None,
                    "steps": p.steps if p else None, "corrections": p.corrections if p else None,
                    "sidestepped": p.sidestepped if p else None, "backed_up": p.backed_up if p else None,
                    "fine_s": self.planner.fine_elapsed_s() if self.planner else None,
                    "frames": self.n_frames, "seen": self.n_seen,
                    "rejected": dict(self.est.rejected) if self.est else None,
                    "learner": self.learner.dump() if self.learner else None,
                    "learned_seeds": self.learner.seeds() if self.learner else None,
                    "gyro": self.gyro.quality() if self.gyro is not None else None,
                    "clock": self.camclock.report(),
                    "camera": (self.src.stats.summary() if getattr(self.src, "stats", None) else None),
                    "seeds_from": str(self.seeds_from) if self.seeds_from else None,
                    "aim": {"c_deg": self.aim.c()[0], "c_sigma_deg": self.aim.c()[1], "legs": self.aim.fits} if getattr(self, "aim", None) else None,
                    "video_frames": self.video_k if self.video is not None else None})
        return _json_safe(out)

    def close(self):
        self.stop_req.set()
        if self.ctrl is not None and self.ctrl.is_alive():
            self.ctrl.join(timeout=CONTROL_JOIN_S)                    # 카메라보다 먼저 — 마지막 동작이 정지 확인까지 보게
            if self.ctrl.is_alive():
                self.log("  !! 제어 스레드가 %.0f s 안에 안 끝났다 — 그냥 닫는다" % CONTROL_JOIN_S)
        self.closing.set()
        if self.cam is not None and self.cam.is_alive():
            self.cam.join(timeout=3.0)                                # 다음 프레임에서 나온다. 실행 중인 제너레이터는 못 닫는다
        if self.src is not None:
            try:
                self.src.frames.close()
            except Exception:
                pass
        if self.driver is not None:
            try:
                self.driver.close()                                   # stop 을 보내고 버스를 닫는다
            except Exception as e:
                self.log("  !! CAN 닫기 실패: %s" % e)
        if self.src is not None:
            try:
                self.src.close()
            except Exception:
                pass
        if self.outcome is not None and self.outcome["outcome"] in STOP_REASONS:
            self._write_stop(self.outcome["outcome"], self.outcome.get("why") or "")
        summary = self.summary()
        if self.rec is not None:
            try:
                (self.dir / "learned.json").write_text(json.dumps(summary.get("learner"), ensure_ascii=False, indent=1))
                # 다음 주행이 물려받는다 (learn.last_seeds — 수렴한 값만 들어 있다)
                (self.dir / "seeds.json").write_text(json.dumps(summary.get("learned_seeds"), ensure_ascii=False, indent=1))
            except Exception:
                pass
            rs = self.rec.close(outcome=summary)
            summary["record"] = rs
            self.log("  기록 %s → %s" % (rs, self.dir))
        if self.video is not None:
            try:
                self.video.release()
                self.video_side.close()
            except Exception:
                pass
        if self.display_ok:
            try:
                cv2.destroyAllWindows()
            except Exception:
                pass
        self.keys.close()
        return summary

    def open_video(self):
        path = self.dir / "video.mp4"
        self.video = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), DISPLAY_FPS, DISPLAY_SIZE)
        if not self.video.isOpened():
            self.log("  !! 영상 파일을 못 연다: %s — 영상 없이 간다" % path)
            self.video = None
            return
        self.video_side = open(self.dir / "video.jsonl", "a", encoding="utf-8")
        self.log("  영상 %s (%dx%d, %.0f fps) + video.jsonl" % (path, DISPLAY_SIZE[0], DISPLAY_SIZE[1], DISPLAY_FPS))


# ═══════════════════════════════════════════════════════════════════════
def main():
    ap = argparse.ArgumentParser(description="실주행 — 태그를 보며 탑재부 입구까지 (기본: 기록 o · 화면 x)")
    ap.add_argument("--show", action="store_true", help="화면")
    ap.add_argument("--no-record", action="store_true", help="기록 끔 (기록이 의심될 때만)")
    ap.add_argument("--video", action="store_true", help="화면에 그린 그대로 영상으로 (960x540 · 15 fps)")
    ap.add_argument("--out", default=None, help="기록 루트 (외장 등). 기본 work_dirs/ (KRRI_WORK_ROOT)")
    ap.add_argument("--dry-run", action="store_true", help="CAN 안 보냄")
    ap.add_argument("--reuse-sigma", action="store_true",
                    help="직전 주행이 잰 σ 기준선·태그 기울기를 그대로 쓴다 (60초 생략). 조명·태그가 바뀌었으면 쓰지 마라")
    args = ap.parse_args()

    root = work_root(args.out)
    record = not args.no_record
    run_dir = None
    if record or args.video:
        base = check_writable(root / "runs", need_mb=200)             # 출발 전에 쓸 수 있는지 (plan 6-6 ⑤)
        run_dir = base / datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
    log = Log(run_dir / "log.txt" if run_dir is not None else None)
    log("== run %s  %s%s" % (run_dir.name if run_dir else "(기록 없음)",
                            "dry-run " if args.dry_run else "",
                            "화면" if args.show else ""))
    rec = Recorder(run_dir) if record else None
    dock = Docking(args, root, run_dir, rec, log)

    sig_n = [0]

    def on_sigint(signum, frame):
        sig_n[0] += 1
        if sig_n[0] >= 3:
            raise KeyboardInterrupt                                    # 세 번째면 그냥 죽는다
        dock.emergency("ctrl_c")
    signal.signal(signal.SIGINT, on_sigint)

    code = 1
    try:
        dock.open()
        if args.video:
            dock.open_video()
        dock.gates()
        if dock.wait_start():
            dock.start_control()
            dock.loop()
        code = 0
    except SystemExit as e:
        log("!! %s" % e)
        with dock._olock:
            if dock.outcome is None:
                dock.outcome = {"outcome": "refused", "fail_reason": None, "why": str(e)}
        code = 2
    except KeyboardInterrupt:
        dock.emergency("ctrl_c")
    except Exception as e:
        log("!! %s: %s" % (type(e).__name__, e))
        log(traceback.format_exc())
        dock.emergency("exception")
    finally:
        summary = dock.close()
    log("== 결과: %s (%s) %s" % (summary["outcome"], summary.get("fail_reason") or "-", summary.get("why") or ""))
    if summary["outcome"] == "normal":
        return 0
    return code if code == 2 else 1


if __name__ == "__main__":
    sys.exit(main())
