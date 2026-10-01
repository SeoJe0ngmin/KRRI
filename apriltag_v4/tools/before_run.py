"""출발 전 측정 → work_dirs/before_run/<시각>/measured.json  (plan 12-1 · 2026-09-30 결정 11).

    python tools/before_run.py                    # 전 단계 순서대로. 단계마다 무엇을 할지 말하고 Enter 를 기다린다
    python tools/before_run.py --resume           # 최신 폴더에 이어서 (끝난 단계는 건너뜀)
    python tools/before_run.py --only rotcenter,veer
    python tools/before_run.py --activate         # 끝나면 config/measured.ACTIVE_RUN 을 이 폴더로 (가짜 기록은 거부)
    python tools/before_run.py --list             # 단계 목록

여기서는 **원시 기록을 남기고 값을 낸다.** 계산식은 tools/analyze_before_run.py 에 있고 같은 폴더에서
사후에 다시 낼 수 있다(판정을 기록 시점에 박지 않는다 — plan 6-3 ⑤). σ 밀림은 estimate.drift_baseline.

단계 순서는 plan 12-1 의 번호와 다르다 — **한 자리에서 할 수 있는 것부터, 눈이 머는 태그컷은 마지막에**:

    device       ① 장비: 가속도계·자이로·카메라·CAN. intrinsics 를 읽어 참고값(D435I_COLOR_REF)과 차이를 보인다
    human        ② 줄자 6개: 태그높이·카메라높이·포크끝·태그좌우·탑재부길이·뒤공간 (+ 조건 메모)
    camcheck     ②-b 줄자 4.0 m 에서 카메라 대조 3종 — 태그 px · 자세 거리 · 태그 중심 행 (예측 vs 실제)
    camyaw       ⑤ 법선 위에 세운 채 heading 을 읽는다 = cam_yaw_offset
    tagroll      ③ 태그 액자 기울기 (중력)
    sigma_still  ⑨ 정지 60초 → 0.5초 창 평균의 흔들림(밀림). **이게 있어야 추정기가 ok 를 낸다** (estimate: no_sigma)
    rotfloor     ⑧ 움직이자마자 정지 × 좌5·우5 → 최대각 = 회전 하한
    rotresp      ⑪ 강도 30 으로 ±12도 × 3 → Learner 가 τ·각속도·출발지연을 낸다
    rotcenter    ④ 5도씩 스윙 → 동심원 → 회전중심 + RMS
    backspeed    ⑬ 후진(187) 짧게 2회, 카메라로 거리 변화 → 이 뒤의 되돌아오기에 쓴다
    fwdspeed     ⑫ 직진(67) 폐루프 2회 → Learner
    sigma_drive  ⑩ 직진 60초 (다리를 이어서) → 같은 계산
    veer         ⑥ 3 m 직진 × 3, 자이로가 휜 시행은 버린다 → 쏠림
    tagcut       ⑦ 천천히 다가가 태그가 안 보이게 되는 거리. 끝나면 눈이 멀어 있다

안전: 움직이는 단계는 시작 전에 "사람이 제동 위치에 있나" 를 한 번 확인한다. Ctrl+C 는 **먼저 CAN 정지**, 그다음 기록.
조향 강도가 30 이 아니면 움직이는 단계를 거부한다 (결정 1 — 씨앗이 30 짜리다). 실패한 단계는 기록만 남기고 다음으로 간다.
"""
import argparse
import json
import math
import random
import re
import signal
import statistics as S
import sys
import threading
import time
import traceback
from collections import deque
from datetime import datetime
from pathlib import Path
from types import SimpleNamespace

import cv2                                                           # 가짜 카메라의 투영에만. 실물은 image.py 가 연다
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))        # tools/ 의 부모. bootstrap 이 다시 확인한다
from src.bootstrap import ROOT, check_writable, setup, work_root     # noqa: E402

setup()
sys.path.insert(0, str(ROOT / "tools"))
import analyze_before_run as A                                       # noqa: E402  현장 계산 = 사후 재계산

from config import control as C                                      # noqa: E402
from config import detection as D                                    # noqa: E402
from config import imu as I                                          # noqa: E402
from config import measured as M                                     # noqa: E402
from src import limits                                               # noqa: E402
from src.models.control import forward as F                          # noqa: E402
from src.models.control import rotate as R                           # noqa: E402
from src.models.control.driver import Driver                         # noqa: E402
from src.models.control.frames import SAFE, apply_and_verify         # noqa: E402
from src.models.control.learn import Ema, Learner                    # noqa: E402
from src.models.detection import estimate as E                       # noqa: E402
from src.models.detection import pose as P                           # noqa: E402
from src.models.detection import tag as T                            # noqa: E402
from src.models.detection.image import CameraIntrinsics, intrinsics_from_ref   # noqa: E402
from src.utils import clock                                          # noqa: E402
from src.utils.gyro import Gyro, Rotation, RotationResult            # noqa: E402
from src.utils.record import Recorder                                # noqa: E402

# ── 구현 세부 (config 에 올리지 않는다 — plan 12-4) ─────────────────────
TAPE_CHECK_M = 4.0            # 카메라 대조를 하는 줄자 거리 (결정 11)
ROT_RESPONSE_DEG = 12.0       # 회전 응답 확인 각 (plan 12-1 ⑪). 상한 17.1 아래, 하한 위
ROT_RESPONSE_N = 3            # 방향당 횟수 (plan 12-1 ⑪ "수회")
ROT_FLOOR_N = 5               # 방향당 횟수 (plan 4-5 "양방향 5회")
SWING_MAX_DEG = 40.0          # 동심원 스윙 반폭. 중심이 앞이면 ±40, 뒤면 절반 (plan 4-7 ③)
SIGMA_S = 60.0                # σ 기준선 기록 시간 (결정 8·11 "정지 60초 · 직진 60초")
VEER_M = 3.0                  # 쏠림 측정 직진 거리 (plan 5-4 "3 m 직진")
VEER_N = 3                    # 시행 수 (plan 5-4 "3~5회")
GYRO_STRAIGHT_TOL_DEG = 0.3   # 이보다 돌았으면 휜 시행 — plan 5-6 의 "자이로가 0.3도 넘게 돌았으면 버린다"
STILL_S = 2 * E.WINDOW_S      # 정지 확인 창. 추정 창의 두 배라야 창 하나가 통째로 새 프레임이다
NEAR_MARGIN_M = 0.3           # 직진 다리는 태그컷 + 이만큼 앞에서 선다. 정지 관성 12~14 cm(9/7 실측, CLAUDE.md)의 2배
TAG_LOST_N = F.ONSET_N        # 태그컷: 연속 이만큼 못 보면 선다 (forward 의 "연속 3" 과 같은 생각)
FRAME_KEYS = ("lateral", "vertical", "forward", "heading_deg", "distance", "beta_deg", "edge_px", "top_px", "tag_px",
              "row_px", "angle_ok", "reproj_px", "tilt_deg", "tag_roll_deg", "gyro_deg", "err_ratio")
ANGLE_KEYS = E.Estimator.ANGLE_KEYS


class Log:
    """화면 + 폴더의 log.txt."""

    def __init__(self, path):
        self.f = open(path, "a", encoding="utf-8")

    def __call__(self, text=""):
        print(text, flush=True)
        try:
            self.f.write("%s %s\n" % (datetime.now().strftime("%H:%M:%S"), text))
            self.f.flush()
        except Exception:
            pass


def _json_safe(v):
    """NaN·inf·numpy·NamedTuple 을 JSON 이 받는 꼴로."""
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


def _intr_dict(intr):
    """Measured.intrinsics — limits._intr 이 읽는 이름(fx fy cx cy w h) + 반화각 + 왜곡."""
    d = {"fx": float(intr.fx), "fy": float(intr.fy), "cx": float(intr.cx), "cy": float(intr.cy),
         "w": int(intr.width), "h": int(intr.height),
         "distortion": [float(x) for x in (intr.distortion or ())]}
    d["half_fov_deg"] = limits.half_fov_deg(d)
    return d


def _ensure_back_key(lrn):
    """후진(187) 열쇠가 learner 에 있게 한다 (씨앗은 없음 — backspeed 단계가 잰다)."""
    k = str(F.strength_of("backward"))
    for dct, (lo, hi) in ((lrn.fwd_speed, lrn.SPEED_FWD), (lrn.fwd_startup, lrn.STARTUP_FWD),
                          (lrn.fwd_tau, lrn.TAU_FWD)):
        dct.setdefault(k, Ema(lo=lo, hi=hi))


def _seed(e, v):
    """Ema 에 씨앗을 심는다 (값·seed 둘 다). before_run 실측 → 다음 단계가 바로 쓴다."""
    if v is not None and math.isfinite(v):
        e.value, e.seed = float(v), float(v)


# ═══════════════════════════════════════════════════════════════════════
# 장비 한 벌
# ═══════════════════════════════════════════════════════════════════════
class Rig:
    """카메라·자이로·CAN·추정기·학습기 + 단계들이 같이 쓰는 동작. 실물 장비만 — 가짜 리그는 없다 (2026-10-01)."""

    def __init__(self, args, run_dir, rec, log):
        self.args, self.dir, self.rec, self.log = args, Path(run_dir), rec, log
        self.m = M.Measured()
        self.learner = Learner(mode="before_run", seeds=M.seeds())
        _ensure_back_key(self.learner)
        self.camclock = clock.CameraClock()
        self.intr = self.shape = self.est = None
        self.gyro = self.driver = self.frames = self.detector = self.cam = None
        self.gyro_report = None
        self.can_ok = self.cam_ok = False
        self.deflection_ok = (C.ROTATE_JOYSTICK_DEFLECTION == M.KWU_DEFLECTION)
        self.armed = False
        self.stage = ""
        self.conditions = {}
        self._lock = threading.RLock()
        self._collect = None            # 켜져 있으면 프레임 상태를 여기 모은다
        self._last = None
        self.n_frames = self.n_seen = self.miss = 0
        self._stop = threading.Event()
        self._thread = None

    # ── 수명 ──────────────────────────────────────────────────────────
    def open(self):
        self.log("== 장비 열기 (실물)")
        self._open_real()
        self.rec.snapshot(sys.argv, extra={"tool": "before_run", "intrinsics": self.m.intrinsics})

    def _open_real(self):
        from src.models.detection.image import open_realsense, to_gray
        from src.utils.camera import set_global_time, CameraSettings
        # 자이로 먼저, 컬러 나중 — RSUSB 는 먼저 연 쪽이 IMU 를 갖는다 (CLAUDE.md)
        try:
            self.gyro = Gyro().start().enable_raw()
        except Exception as e:
            self.log("  !! 자이로를 못 열었다: %s — 회전 단계는 못 한다" % e)
            self.gyro = None
        self.frames, self.intr = open_realsense(stream="color", meta=True,
                                                tune=CameraSettings.docking())   # 9/21 과 같은 노출 설정. 못 열면 예외 → 끝
        try:
            set_global_time(self.frames.profile)
        except Exception:
            pass
        self.detector = T.make_detector(quad_decimate=D.QUAD_DECIMATE)
        self._to_gray = to_gray
        self.cam_ok = True
        self.m.intrinsics = _intr_dict(self.intr)
        self.rebuild_estimator()                     # 첫 프레임부터 받는다 — 보정 중에도 태그를 센다
        self._thread = threading.Thread(target=self._cam_loop, name="Camera", daemon=True)
        self._thread.start()
        if self.gyro is not None:
            self.gyro_report = self.recalibrate()
        self.driver = Driver(dry_run=False, recorder=self.rec, log=self.log)
        try:
            self.driver.open()
            self.can_ok = True
        except SystemExit as e:
            self.log("  !! %s — 움직이는 단계는 못 한다" % e)
            self.can_ok = False

    def close(self):
        self._stop.set()
        if self.driver is not None:
            try:
                self.driver.close()
            except Exception as e:
                self.log("  !! CAN 닫기 실패: %s" % e)
        if self._thread is not None:
            self._thread.join(timeout=2.0)
        for obj in (self.frames, self.gyro):
            if obj is not None:
                try:
                    obj.close()
                except Exception:
                    pass

    # ── 카메라 ────────────────────────────────────────────────────────
    def _cam_loop(self):
        try:
            for i, ts, img in self.frames:
                if self._stop.is_set():
                    break
                t_arr = clock.now()
                if self.shape is None:
                    self.shape = tuple(img.shape[:2])
                dets = T.detect(self.detector, self._to_gray(img))
                det = next((d for d in dets if int(d.tag_id) == D.TAG_ID), None)
                self.ingest(i, ts, t_arr, det, self.shape)
        except Exception as e:
            self.cam_ok = False
            self.log("  !! 카메라 스레드가 죽었다: %s" % e)

    def ingest(self, i, ts_camera, t_arrival, det, shape):
        """프레임 하나. 시계 변환 → 추정기 → 기록. 진짜(스레드)와 가짜(가상 시계 걸음)가 같이 부른다."""
        t_cap = self.camclock.see(ts_camera, t_arrival)
        accel = self.gyro.accel if self.gyro is not None else None
        st = None
        with self._lock:
            self.n_frames += 1
            if det is not None and self.est is not None:
                st = self.est.observe(t_cap, det, shape, accel=accel)
            if st is not None:
                self.n_seen += 1
                self.miss = 0
                st = dict(st)
                st.update(t=t_cap, tag_px=T.tag_pixel_size(det), row_px=float(det.center[1]),
                          col_px=float(det.center[0]), angle_ok=bool(st.get("angle_ok", True)),
                          gyro_deg=(self.gyro.angle_deg if self.gyro is not None else None))
                self._last = st
                if self._collect is not None:
                    self._collect.append(st)
            else:
                self.miss += 1
        row = {"t": t_cap, "wall": clock.wall_from(t_cap), "i": i, "stage": self.stage,
               "seen": det is not None, "lat_ms": round((t_arrival - t_cap) * 1000.0, 1)}
        if st is not None:
            row.update({k: st.get(k) for k in FRAME_KEYS})
        self.rec.frame(**row)
        if self.gyro is not None:
            self.rec.imu(self.gyro.drain_raw())

    def fix(self, moving=False):
        with self._lock:
            return self.est.fix(moving=moving)

    def look(self, sign=1.0):
        """forward() 가 볼 것. 남은거리 = 부호 × 법선거리 (뒤로 갈 땐 −). 밀림 기준선이 없어도(no_sigma) 값은 쓴다 —
        여기서는 그 기준선을 재는 중이고, forward 는 거리 떨림(sigma_m)만 본다."""
        f = self.fix(moving=True)
        ok = f.why in ("ok", "no_sigma") and math.isfinite(f.forward_m) and f.forward_m > 0
        sig = f.distance_sigma_m if math.isfinite(f.distance_sigma_m) else 0.0   # 밀림 기준선 있으면 hypot, 없으면 한 장 흔들림 — /√n 값은 가짜 출발을 잡는다 (검토 지적)
        return F.Look(remaining_m=sign * f.forward_m, t_capture=f.t_capture, ok=ok, sigma_m=sig)

    def rebuild_estimator(self):
        """측정한 cam_yaw·태그 기울기·σ 기준선을 추정기에 넣는다. 그 뒤 값은 보정된 값이다."""
        with self._lock:
            self.est = E.Estimator(self.intr, D.TAG_SIZE_M,
                                   cam_yaw_offset_deg=self.m.cam_yaw_offset_deg or 0.0,
                                   tag_roll_correction_deg=self.m.tag_roll_deg or 0.0, noise=self.m)

    def collect(self, seconds, why):
        """seconds 동안의 프레임 상태(dict) 전부. 추정 창은 비우고 시작한다."""
        with self._lock:
            self.est.reset()
            self._collect = []
        self.stage_note = why
        time.sleep(seconds)
        with self._lock:
            rows, self._collect = self._collect, None
        return rows

    def measure_still(self, seconds, why):
        """정지 확인 — 깨끗한 프레임의 중앙값 (v2 의 '정지 후 확인용' 과 같은 자리). ok 는 MIN_N 장 이상일 때."""
        rows = self.collect(seconds, why)
        clean = [r for r in rows if r["angle_ok"]]
        out = {"why": why, "n": len(rows), "n_clean": len(clean), "t": clock.now(),
               "ok": len(clean) >= E.MIN_N}
        for k in ("lateral", "vertical", "forward", "heading_deg", "distance", "beta_deg", "tilt_deg",
                  "tag_px", "row_px", "edge_px", "reproj_px", "gyro_deg", "tag_roll_deg"):
            src = clean if (k in ANGLE_KEYS or k == "tag_roll_deg") else rows
            vals = [r[k] for r in src if r.get(k) is not None and math.isfinite(r[k])]
            out[k] = S.median(vals) if vals else None
            out[k + "_sd"] = S.pstdev(vals) if len(vals) > 1 else None
        self.rec.event("still", **_json_safe(out))
        return out

    # ── 자이로 ────────────────────────────────────────────────────────
    def recalibrate(self):
        """정지 상태에서 바이어스·회전축·영점. 움직였다고 하면 한 번 더."""
        rep = None
        for attempt in range(2):
            rep = self.gyro.calibrate()
            self.rec.event("gyro_calibrate", attempt=attempt,
                           **_json_safe({k: v for k, v in rep.items() if k != "accel_mean"}))
            if not rep.get("moving"):
                break
            self.log("  !! 보정 중 움직임(평균 %.2f 도/s) — 차를 세우고 다시" % rep.get("mean_dps", 0.0))
        self.log("  자이로 보정 %ds: 바이어스 (%s) 도/s · 잡음 %.3f 도/s · 드리프트 %.2f 도/분 · 축 %s"
                 % (rep["sec"], ", ".join("%.3f" % b for b in rep["bias_dps"]), rep["noise_dps"],
                    rep["drift_dpm"], rep["axis_src"]))
        return rep

    # ── 사람 ──────────────────────────────────────────────────────────
    def ask(self, text, default=None, cast=float):
        if self.args.yes:
            self.log("  %s → %s (자동, --yes)" % (text, default))
            return default
        while True:
            s = input("  %s [%s]: " % (text, "" if default is None else default)).strip()
            if not s:
                return default
            try:
                return cast(s)
            except ValueError:
                print("  다시 (%s)" % cast.__name__)

    def confirm(self, text):
        """'go' / 'skip' / 'quit'."""
        if self.args.yes:
            return "go"
        s = input("  %s  [Enter=진행  s=건너뜀  q=끝]: " % text).strip().lower()
        return "quit" if s == "q" else "skip" if s == "s" else "go"

    def ready_to_move(self, need_gyro=True):
        """움직여도 되나. 비어 있으면 된다, 아니면 이유."""
        if self.driver is None or not self.can_ok:
            return "CAN 이 없다"
        if not self.cam_ok:
            return "카메라가 죽었다"
        if not self.deflection_ok:
            return ("조향 강도가 %d 다 — 30 이 아니면 거부 (결정 1: 광운대 씨앗이 30 짜리)"
                    % C.ROTATE_JOYSTICK_DEFLECTION)
        if need_gyro:
            if self.gyro is None or not self.gyro.calibrated:
                return "자이로가 없거나 보정 전"
            if not self.gyro.alive:
                return "자이로가 끊겼다"
        if not self.armed:
            if self.confirm("차가 움직인다. 사람이 제동 위치에 있고 주변이 비었나") != "go":
                return "사람이 준비 안 됨"
            self.armed = True
        return ""

    # ── 동작 ──────────────────────────────────────────────────────────
    def rotate(self, deg, **kw):
        return R.rotate(self.driver, self.gyro, self.learner, deg, self.rec, self.log, **kw)

    def leg(self, target_m, movement="forward"):
        """카메라를 보며 한 다리 (forward.forward). 뒤로 갈 땐 남은거리 부호를 뒤집는다."""
        sign = -1.0 if movement == "backward" else 1.0
        return F.forward(self.driver, self.learner, lambda: self.look(sign), target_m, movement,
                         rec=self.rec, log=self.log)

    def back_model(self):
        k = F.strength_of("backward")
        return bool(self.learner.fwd_speed_mps(k)) and self.learner.fwd_startup_s(k) is not None

    def near_m(self):
        """직진 다리가 서야 하는 법선거리 하한 — 지금 프레임으로 다시 잰 태그컷(실행과 같은 식) + 정지 여유.
        태그가 없으면 피치 0 계산값(높이차: sigma_still 것, 없으면 config)."""
        f = self.fix()
        cut = limits.tag_cut_m(self.m.intrinsics, self.m.height_diff_m)
        if math.isfinite(f.vertical_m) and math.isfinite(f.top_px) and f.forward_m > 0:
            cut = limits.tag_cut_live_m(self.m.intrinsics, -f.vertical_m, f.forward_m, f.top_px)
        return cut + NEAR_MARGIN_M

    def reposition(self, target_fwd_m, why):
        """자리가 모자라면 **사람이** 차를 옮긴다 — 방금 지나온 길 밖으로 스스로 물러나지 않는다 (뒤공간을 모른다).

        수동 조작은 CAN 을 끊어야 하므로 **닫았다가 옮긴 뒤 다시 연다**. 재연결이 안 되면 Ctrl+C → --resume.
        """
        if self.driver is not None and self.can_ok:
            try:
                self.driver.close()                 # 수동 조작 동안 우리 프레임이 나가면 안 된다
            except Exception as e:
                self.log("  !! CAN 닫기 실패: %s" % e)
            self.can_ok = False
            self.log("  CAN 닫음 — 수동으로 옮기고 Enter 하면 다시 연다")
        if self.confirm("%s: 차를 손으로 태그 앞 %.1f m, 법선 위에 세우고" % (why, target_fwd_m)) != "go":
            return None
        if self.driver is not None:
            try:
                self.driver.open()
                self.can_ok = True
                self.log("  CAN 다시 열림")
            except SystemExit as e:
                self.can_ok = False
                self.log("  !! CAN 재연결 실패: %s — Ctrl+C 하고 `before_run.py --resume` 으로 이어서" % e)
                return None
        return self.measure_still(STILL_S, why + " 위치")

    def go_to(self, target_fwd_m, why):
        """법선거리 target 까지 다리를 이어 간다 — 방금 지나온 길을 되돌아올 때. 뒤로는 후진 모델이 있을 때만."""
        st = self.measure_still(STILL_S, why + " 위치")
        if not st["ok"]:
            self.log("  !! 태그가 안 보여 못 움직인다")
            return False
        legs = int(math.ceil(abs(st["forward"] - target_fwd_m) / C.STEP_FORWARD_HARD_MAX_M)) + 2
        for _ in range(legs):
            delta = st["forward"] - target_fwd_m
            if abs(delta) <= C.FWD_TOL_M:
                return True
            if delta > 0:
                res = self.leg(min(delta, C.STEP_FORWARD_HARD_MAX_M), "forward")
            elif self.back_model():
                res = self.leg(min(-delta, C.STEP_FORWARD_HARD_MAX_M), "backward")
            else:
                st = self.reposition(target_fwd_m, why + " (후진 모델 없음)")
                return st is not None and st["ok"] and abs(st["forward"] - target_fwd_m) <= C.FWD_TOL_M
            if not res.done:
                self.log("  !! 이동 실패(%s)" % res.reason)
                return False
            st = self.measure_still(STILL_S, why + " 위치")
            if not st["ok"]:
                return False
        return abs(st["forward"] - target_fwd_m) <= C.FWD_TOL_M

    def hold(self, movement, hold_s, why, stop_when=None):
        """시간으로 명령을 쥐고 카메라로 본다 (후진 속도·태그컷 전용). 안전망: 시간 상한 + 데드맨 lease."""
        hold_s = min(float(hold_s), C.FWD_SAFETY_MAX_S)
        drv = self.driver
        f0 = self.fix()                               # 출발 판정의 σ — 창을 비우기 **전에** 읽는다
        with self._lock:
            self.est.reset()
            self._collect = []
        self.miss = 0
        drv.lease(F.LEASE_S)
        t_cmd = drv.set(movement, why=why)
        reason = "timed"
        try:
            while clock.now() - t_cmd < hold_s:
                if drv.lease_expired():
                    reason = "lease"
                    break
                if stop_when is not None and stop_when():
                    reason = "stop_when"
                    break
                time.sleep(F.POLL_S)
                drv.lease(F.LEASE_S)
        finally:
            drv.stop("hold " + reason)                # 정지가 먼저 (plan 6-6 ④)
            drv.clear_lease()
        t_stop = clock.now()
        time.sleep(C.SETTLE_S)
        with self._lock:
            rows, self._collect = self._collect, None
        out = {"movement": movement, "why": why, "hold_s": hold_s, "reason": reason,
               "t_cmd": t_cmd, "t_stop_cmd": t_stop, "t_settled": clock.now(),
               "sigma_m": (f0.distance_fast_sigma_m if math.isfinite(f0.distance_fast_sigma_m) else None),
               # (t, 법선거리, 가장자리 px, vertical, 윗변 행) — 뒤 둘은 태그컷 실시간 식 대조용 (tagcut 단계)
               "track": [(round(r["t"] - t_cmd, 3), round(r["forward"], 4), round(r["edge_px"], 1),
                          None if r.get("vertical") is None else round(r["vertical"], 4),
                          None if r.get("top_px") is None else round(r["top_px"], 1))
                         for r in rows if r.get("forward") is not None]}
        self.rec.event("hold", **_json_safe(out))
        return out

    def rotate_until_onset(self, sign):
        """"자이로가 움직였다고 하면 즉시 끊어라" (plan 4-5 하한). 안전망은 rotate() 와 같은 세 겹."""
        g, drv = self.gyro, self.driver
        movement = "rotate_left_slow" if sign > 0 else "rotate_right_slow"
        startup = self.learner.rot_startup_s(sign) or M.KWU_SEED["rot_startup_s"]["L"]
        drv.arm_rotation_timeout(movement, C.ROT_SAFETY_MAX_S)
        # 목표 = 출발 문턱. 출발을 본 순간 남은각 ≤ 0 이라 콜백이 바로 끊는다 (tau·잔여 0)
        rot = Rotation(target_deg=sign * g.ONSET_DEG, tau_s=0.0, residual_deg=0.0,
                       period_s=R._period_s(g), start_angle=g.angle_deg, t_cmd=clock.now(),
                       max_deg=limits.turn_cap_deg(False))
        g.arm(rot, drv.stop_now)
        drv.lease(R.LEASE_S)
        try:
            drv.set(movement, why="rot_floor %+d" % sign)
        except BaseException:
            g.disarm()
            drv.stop("rot_floor 명령 실패")
            drv.clear_lease()
            raise
        deadline = rot.t_cmd + R.ABORT_FACTOR * startup + g.SETTLE_MAX_S + R.LEASE_S
        fault = ""
        while True:
            res = g.result()
            if res is None or res.done:
                break
            if clock.now() > deadline:
                fault = "watchdog"
                break
            if not g.alive:
                fault = "gyro_stale"
                break
            if drv.lease_expired():
                fault = "lease"
                break
            drv.lease(R.LEASE_S)
            time.sleep(R.POLL_S)
        res = g.result() or RotationResult(target_deg=rot.target_deg)
        if fault:
            drv.stop("rot_floor " + fault)
            g.disarm()
            res.reason = res.reason or fault
            if not res.done:
                res.turned_deg = g.angle_deg - rot.start_angle
        drv.clear_lease()
        ok = res.done and res.reason == "predicted"
        self.rec.event("rotfloor", sign=sign, ok=ok, reason=res.reason, turned_deg=res.turned_deg,
                       turned_at_stop=res.turned_at_stop, omega_at_stop=res.omega_at_stop,
                       t_cmd=res.t_cmd, t_onset=res.t_onset, t_stop_cmd=res.t_stop_cmd,
                       t_settled=res.t_settled)
        return res

    # ── 학습값 → Measured ─────────────────────────────────────────────
    def harvest_learner(self):
        """Learner 의 지금 값을 Measured 에. 한 번도 못 본 값은 씨앗(광운대) 그대로 — 파일이 자립하게."""
        L, m = self.learner, self.m

        def pick(e):
            return round(e.value, 5) if (e.n > 0 or e.seed) else None
        for side in ("L", "R"):
            m.rot_tau_s[side] = pick(L.rot_tau[side])
            m.rot_rate_dps[side] = pick(L.rot_rate[side])
            m.rot_startup_s[side] = pick(L.rot_startup[side])
            m.rot_residual_deg[side] = pick(L.rot_residual[side])
        k = str(F.strength_of("forward"))
        m.fwd_tau_s[k] = pick(L.fwd_tau[k])
        m.fwd_speed_mps[k] = pick(L.fwd_speed[k])
        m.fwd_startup_s[k] = pick(L.fwd_startup[k])
        m.fwd_residual_m = pick(L.fwd_residual)
        kb = str(F.strength_of("backward"))
        # 후진은 backspeed 단계의 직접 측정(시간 유지 + 카메라)이 우선. 그게 없을 때만 폐루프 후진에서 배운 값
        if m.back_speed_mps is None and (L.fwd_speed[kb].n > 0 or L.fwd_speed[kb].seed):
            m.back_speed_mps = pick(L.fwd_speed[kb])
            m.back_startup_s = pick(L.fwd_startup[kb])


# ═══════════════════════════════════════════════════════════════════════
# 단계
# ═══════════════════════════════════════════════════════════════════════
def stage_device(rig):
    out = {}
    g = rig.gyro
    # 가속도계 — 태그 기울기(5-3)와 회전축의 전제. 9/21 처럼 자이로만 살고 이게 죽을 수 있다
    acc = None
    if g is not None:
        for _ in range(40):
            acc = g.accel
            if acc is not None:
                break
            time.sleep(0.05)
    mag = math.sqrt(sum(a * a for a in acc)) if acc else None
    out["accel"] = {"xyz": acc, "mag": mag}
    rig.log("  가속도계: %s" % ("없음 — 태그 기울기·회전축 못 잰다" if acc is None
                              else "(%.2f, %.2f, %.2f) |g| %.2f %s" % (*acc, mag, "ok" if 7.0 < mag < 12.5 else "!! 중력 크기가 아니다")))
    # 자이로
    if g is not None:
        q = g.quality()
        out["gyro"] = _json_safe({"report": {k: v for k, v in (rig.gyro_report or {}).items() if k != "accel_mean"},
                                  "quality": q, "alive": g.alive})
        rig.log("  자이로: %s Hz 실측 %.1f · 유실 %d · 처리간격 p99 %s ms (5 를 크게 넘으면 GIL 대기)"
                % ("살아있음" if g.alive else "!! 끊김", q.get("hz", 0.0), q.get("gaps", 0),
                   q.get("process_interval_ms", {}).get("p99")))
    else:
        out["gyro"] = None
    # 카메라 + 시계 + intrinsics
    for _ in range(100):
        if rig.camclock.ready or not rig.cam_ok:
            break
        time.sleep(0.05)
    err = rig.camclock.check(rig.log)
    if err:
        rig.log("  !! %s" % err)
    ref = intrinsics_from_ref(rig.shape) if rig.shape else None
    w, h, fx, fy, cx, cy = limits._intr(rig.m.intrinsics)
    rig.log("  카메라 %dx%d · 프레임 %d 장, 태그 %d 장 (%.0f %%)%s"
            % (w, h, rig.n_frames, rig.n_seen, 100.0 * rig.n_seen / max(1, rig.n_frames),
               " · " + rig.frames.stats.summary()))
    if rig.n_seen == 0:
        rig.log("  !! 태그가 안 보인다 — 태그 id %d, 조명·거리를 확인하라" % D.TAG_ID)
    rig.log("  intrinsics       카메라        참고값(D435I_COLOR_REF)   차이")
    diff = {}
    for name, val in (("fx", fx), ("fy", fy), ("cx", cx), ("cy", cy)):
        rv = getattr(ref, name) if ref is not None else float("nan")
        diff[name] = val - rv
        rig.log("    %-4s %12.2f %14.2f %14s" % (name, val, rv, "%+.2f" % (val - rv)))
    out["intrinsics"] = rig.m.intrinsics
    out["intrinsics_diff"] = _json_safe(diff)
    out["clock"] = _json_safe(rig.camclock.report())
    # CAN
    out["can"] = _json_safe(rig.driver.status() if rig.driver is not None else None)
    out["can_ok"] = rig.can_ok
    rig.log("  CAN: %s" % ("열림 (동결 테이블 확인 끝)" if rig.can_ok else "!! 없음 — 움직이는 단계는 못 한다"))
    # 강도 — 이 값들을 잰 강도로 각인된다. 30 아니면 회전 단계 거부 (결정 1)
    rig.m.rotate_deflection = C.ROTATE_JOYSTICK_DEFLECTION
    out["deflection_ok"] = rig.deflection_ok
    rig.log("  조향 강도 %d (byte1 %d/%d) %s" % (C.ROTATE_JOYSTICK_DEFLECTION, 127 + C.ROTATE_JOYSTICK_DEFLECTION,
                                             127 - C.ROTATE_JOYSTICK_DEFLECTION,
                                             "" if rig.deflection_ok else "!! 30 이 아니다 — 움직이는 단계 거부"))
    rig.rec.event("device", **out)
    return out


def stage_human(rig):
    m = rig.m
    rig.log("  조건만 묻는다 (줄자 값은 config).")
    rig.conditions = {
        "camera_remounted": rig.ask("카메라를 다시 달았나 (1/0)", 1, cast=int) == 1,
        "tag_remounted": rig.ask("태그를 다시 붙였나 (1/0)", 1, cast=int) == 1,
        "note": rig.ask("메모 (노면·적재·날씨)", "", cast=str),
    }
    rig.log("  줄자 값은 전부 config: 태그 %.2f · 카메라 %.2f (높이차 %.2f, 대조용 — 실행은 sigma_still 의 카메라값) · "
            "포크끝 %.2f · 태그좌우 %+.2f · 뒤공간 %.1f" % (D.TAG_HEIGHT_M, D.CAMERA_HEIGHT_M, D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M,
                                                      C.CAM_TO_FORK_TIP_M, C.TAG_LATERAL_OFFSET_M, C.BACK_MAX_M))
    out = {}
    out["conditions"] = rig.conditions
    rig.rec.event("human", **out)
    return out


def stage_camcheck(rig):
    tape = rig.ask("카메라 렌즈 → 태그면 줄자 [m]", TAPE_CHECK_M)
    st = rig.measure_still(STILL_S, "camcheck")
    if not st["ok"]:
        raise RuntimeError("태그가 안 보인다 (깨끗한 프레임 %d)" % st["n_clean"])
    r = A.camcheck(rig.m.intrinsics, st["tag_px"], st["forward"], st["row_px"], tape,
                   D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M)                 # 줄자 높이차 = config
    rig.log("  (a) 태그 px   예측 %.1f  실제 %.1f  → 거리로 %.3f m (줄자와 %+.3f m)"
            % (r["px_pred"], r["px_meas"], r["dist_from_px_m"], r["err_px_m"]))
    rig.log("  (b) 자세 거리 %.3f m  줄자 %.2f m  (%+.3f m)" % (r["forward_meas_m"], tape, r["err_dist_m"]))
    rig.log("  (c) 태그 중심 행  예측 %.1f  실제 %.1f  → 높이차로 %.3f m (입력 %.3f)  태그컷 %.2f ↔ %.2f m (%+.3f m)"
            % (r["row_pred"], r["row_meas"], r["height_diff_from_row_m"], r["height_diff_in_m"],
               r["cut_from_row_m"], r["cut_from_input_m"], r["err_cut_m"]))
    bad = [k for k in ("err_px_m", "err_dist_m", "err_cut_m") if abs(r[k] or 0.0) > C.FWD_TOL_M]
    if bad:
        rig.log("  !! %s 가 앞뒤 허용 %.2f m 를 넘는다 — 통로·태그컷 입력이 전부 흔들린다. "
                "줄자·높이 입력·태그 크기(%.3f m)·intrinsics 를 다시 보라" % (bad, C.FWD_TOL_M, D.TAG_SIZE_M))
    r["warn"] = bad
    rig.rec.event("camcheck", **_json_safe(r))
    return r


def stage_camyaw(rig):
    st = rig.measure_still(STILL_S, "camyaw")
    if not st["ok"]:
        raise RuntimeError("태그가 안 보인다")
    raw = st["heading_deg"] + rig.est.cam_yaw_offset_deg      # 추정기가 이미 뺀 게 있으면 되돌린다
    rig.m.cam_yaw_offset_deg = raw
    rig.log("  법선 위 heading %+.2f도 (1초 sd %.2f, n %d) → cam_yaw_offset. 좌우 %+.3f m (0 이어야) · β %+.2f도"
            % (raw, st["heading_deg_sd"] or 0.0, st["n_clean"], st["lateral"], st["beta_deg"]))
    if rig.m.sigma_drift_heading_deg:
        rig.log("  (방향 밀림 σ %.2f도 급이 이 값에 그대로 들어 있다 — 5-4 의 쏠림·정답자세와 같이 본다)"
                % rig.m.sigma_drift_heading_deg)
    rig.rebuild_estimator()
    out = {"cam_yaw_offset_deg": raw, "lateral_m": st["lateral"], "beta_deg": st["beta_deg"], "n": st["n_clean"]}
    rig.rec.event("camyaw", **_json_safe(out))
    return out


def stage_tagroll(rig):
    st = rig.measure_still(STILL_S, "tagroll")
    if not st["ok"]:
        raise RuntimeError("태그가 안 보인다")
    if st["tag_roll_deg"] is None:
        raise RuntimeError("가속도계 값이 없다 — 기울기를 못 잰다 (5-3)")
    rig.m.tag_roll_deg = st["tag_roll_deg"]
    rig.log("  태그 액자 기울기 %+.2f도 (sd %.2f) — 높이차 %.2f m 면 좌우로 %.0f mm 샌다"
            % (st["tag_roll_deg"], st["tag_roll_deg_sd"] or 0.0,
               D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M,
               abs(math.sin(math.radians(st["tag_roll_deg"])) * (D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M) * 1e3)))
    rig.rebuild_estimator()
    out = {"tag_roll_deg": st["tag_roll_deg"], "sd": st["tag_roll_deg_sd"], "n": st["n_clean"]}
    rig.rec.event("tagroll", **_json_safe(out))
    return out


def stage_sigma_still(rig):
    sec = rig.args.sigma_s
    rig.log("  %.0f초 동안 가만히. 사람도 카메라 앞을 지나가지 마라." % sec)
    rows = rig.collect(sec, "sigma_still")
    s = A.still_sigma(rows)
    if s is None:
        raise RuntimeError("깨끗한 프레임이 모자란다 (%d 장)" % len(rows))
    m = rig.m
    m.sigma_ref_distance_m = s["sigma_ref_distance_m"]
    m.sigma_drift_lateral_m = s["sigma_drift_lateral_m"]
    m.sigma_drift_heading_deg = s["sigma_drift_heading_deg"]
    # 높이차 = 카메라가 잰 vertical(태그 기준 카메라 높이, 아래가 −) 의 중앙값. 줄자와 달리 **그 자리의 경사·차체
    # 기울기가 들어간다** (2026-10-01). 줄자값은 대조용. 두 값의 태그컷 차이가 앞뒤 허용을 넘으면 경고
    vert = [r["vertical"] for r in rows if r.get("vertical") is not None and math.isfinite(r["vertical"]) and r.get("angle_ok")]
    if len(vert) >= E.MIN_N:
        m.height_diff_m = -S.median(vert)
        tape = D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M                      # 줄자 높이차 = config
        rig.log("  높이차: 카메라 %.3f m · config %.3f m" % (m.height_diff_m, tape))
        d_cut = limits.tag_cut_m(m.intrinsics, m.height_diff_m) - limits.tag_cut_m(m.intrinsics, tape)
        if abs(d_cut) > C.FWD_TOL_M:
            rig.log("  !! 카메라·줄자 높이차가 태그컷 기준 %.2f m 다르다 — 경사·카메라 기울기·줄자 입력을 의심" % d_cut)
        # 실행이 쓰는 실시간 식(높이 + 윗변 행 → 피치 포함)을 이 자리에서 한 번 보여 준다. 피치 0 가정과의 차 = 카메라 피치 몫
        tops = [r["top_px"] for r in rows if r.get("top_px") is not None and math.isfinite(r["top_px"])]
        fwds = [r["forward"] for r in rows if r.get("angle_ok") and r.get("forward") is not None and math.isfinite(r["forward"])]
        if tops and fwds:
            live = limits.tag_cut_live_m(m.intrinsics, m.height_diff_m, S.median(fwds), S.median(tops))
            flat = limits.tag_cut_m(m.intrinsics, m.height_diff_m)
            rig.log("  태그컷 (이 자리): 실시간 식 %.2f m · 피치 0 가정 %.2f m (차 %+.2f = 카메라 피치 몫). 실행은 실시간 식을 프레임마다 쓴다"
                    % (live, flat, live - flat))
    L, H, Dd = s["lateral"], s["heading"], s["distance"]
    rig.log("  %d 장 (헷갈린 장 %.0f %%) · 기준 거리(3D) %.3f m" % (s["n"], s["ambiguous_rate"] * 100, s["sigma_ref_distance_m"]))
    rig.log("             한 장 sd    블록평균 sd   떨림       밀림(→Measured)")
    rig.log("    좌우   %7.1f mm  %8.1f mm  %7.1f mm  %7.1f mm" % (L["frame_sd"] * 1e3, L["block_sd"] * 1e3, L["resid_sd"] * 1e3, L["drift"] * 1e3))
    rig.log("    방향   %7.3f도  %8.3f도  %7.3f도  %7.3f도" % (H["frame_sd"], H["block_sd"], H["resid_sd"], H["drift"]))
    if Dd:
        rig.log("    거리   %7.1f mm  %8.1f mm  %7.1f mm  %7.1f mm  (기준선 필드가 아직 없다 — 진단만)"
                % (Dd["frame_sd"] * 1e3, Dd["block_sd"] * 1e3, Dd["resid_sd"] * 1e3, Dd["drift"] * 1e3))
    if L["drift"] == 0.0 or H["drift"] == 0.0:
        rig.log("  (밀림 0 = 블록평균의 흔들림이 떨림으로 다 설명된다. 9/21 실측(밀림 31 mm)과 다르면 조명·태그 판을 의심)")
    rig.rebuild_estimator()
    rig.rec.event("sigma_still", **_json_safe(s))
    return {k: s[k] for k in ("sigma_ref_distance_m", "sigma_drift_lateral_m", "sigma_drift_heading_deg", "n", "ambiguous_rate")}


def stage_rotfloor(rig):
    why = rig.ready_to_move()
    if why:
        return {"skip": why}
    rig.recalibrate()
    turned, rows = [], []
    for k in range(ROT_FLOOR_N):
        for sign in (+1, -1):
            res = rig.rotate_until_onset(sign)
            ok = res.done and res.reason == "predicted"
            rig.log("  %s %d: %s %+.2f도 (출발 %.2fs, 끊을 때 %.1f도/s)"
                    % ("좌" if sign > 0 else "우", k + 1, "ok" if ok else "!! " + res.reason, res.turned_deg,
                       (res.t_onset - res.t_cmd) if res.t_onset else float("nan"), res.omega_at_stop))
            rows.append({"sign": sign, "ok": ok, "turned_deg": res.turned_deg, "reason": res.reason})
            if ok:
                turned.append(res.turned_deg)
    r = A.rot_floor(turned)
    if r["rot_floor_deg"] is None:
        raise RuntimeError("성공한 시행이 없다")
    rig.m.rot_floor_deg = r["rot_floor_deg"]
    rig.learner.rot_floor_deg = r["rot_floor_deg"]
    rig.log("  회전 하한 = 최대 %.2f도 (중앙 %.2f, sd %s, n %d). 씨앗 0.81 은 유도값이었다"
            % (r["rot_floor_deg"], r["median_deg"], "%.2f" % r["sd_deg"] if r["sd_deg"] is not None else "-", r["n"]))
    r["rows"] = rows
    rig.rec.event("rotfloor_summary", **_json_safe(r))
    return r


def stage_rotresp(rig):
    why = rig.ready_to_move()
    if why:
        return {"skip": why}
    rig.recalibrate()
    rows = []
    for k in range(ROT_RESPONSE_N):
        for sign in (+1, -1):
            s0 = rig.measure_still(STILL_S, "rotresp")
            res = rig.rotate(sign * ROT_RESPONSE_DEG)
            s1 = rig.measure_still(STILL_S, "rotresp")
            row = {"k": k, "sign": sign, "ok": res.ok, "reason": res.reason, "turned_deg": res.turned_deg,
                   "coast_deg": res.coast_deg, "omega_at_stop": res.omega_at_stop, "tau_observed": res.tau_observed,
                   "startup_s": (res.t_onset - res.t_cmd) if res.t_onset else None,
                   "beta0": s0["beta_deg"], "beta1": s1["beta_deg"], "heading0": s0["heading_deg"],
                   "heading1": s1["heading_deg"], "gyro0": s0["gyro_deg"], "gyro1": s1["gyro_deg"]}
            rows.append(row)
            rig.rec.event("rotresp", **_json_safe(row))
            if s0["ok"] and s1["ok"] and res.ok:
                rig.log("     β 변화 %+.2f도 / 카메라 방향 변화 %+.2f도 / 자이로 %+.2f도"
                        % (s1["beta_deg"] - s0["beta_deg"], s1["heading_deg"] - s0["heading_deg"], res.turned_deg))
    rig.harvest_learner()
    L = rig.learner
    for side in ("L", "R"):
        rig.log("  %s: τ %.3f s (n %d) · 각속도 %.2f 도/s (n %d) · 출발 %.2f s · 잔여 %+.2f도"
                % (side, L.rot_tau[side].value, L.rot_tau[side].n, L.rot_rate[side].value, L.rot_rate[side].n,
                   L.rot_startup[side].value, L.rot_residual[side].value))
    return {"rows": rows, "learned": _json_safe(L.dump())}


def _stop_row(st, direction, psi_rel):
    return {"lateral": st["lateral"], "forward": st["forward"], "heading_deg": st["heading_deg"],
            "beta_deg": st["beta_deg"], "distance": st["distance"], "gyro_deg": st["gyro_deg"],
            "psi_rel_deg": psi_rel, "dir": direction, "n": st["n_clean"]}


def stage_rotcenter(rig):
    why = rig.ready_to_move()
    if why:
        return {"skip": why}
    rig.recalibrate()
    step = limits.turn_cap_deg(False)             # 중심을 모르니 5도씩 (plan 4-6)
    s0 = rig.measure_still(STILL_S, "rotcenter")
    if not s0["ok"]:
        raise RuntimeError("태그가 안 보인다")
    psi0 = rig.gyro.angle_deg
    stops = [_stop_row(s0, 0, 0.0)]
    rig.rec.event("rotcenter_stop", **_json_safe(stops[0]))
    beta_max = limits.path_angle_max_deg(s0["forward"], rig.m.intrinsics)   # 태그가 화면에 남는 β 한계
    state = {"gain": None, "swing": SWING_MAX_DEG}

    def can_step(sign):
        last = stops[-1]
        g = 1.0 if state["gain"] is None else state["gain"]          # 아직 모르면 1배로 본다
        nxt_beta = last["beta_deg"] + g * step * sign
        return abs(nxt_beta) < beta_max and abs(last["psi_rel_deg"] + sign * step) <= state["swing"] + 1e-6

    def do_step(sign):
        res = rig.rotate(sign * step)
        if not res.ok:
            rig.log("  !! 스텝 실패(%s) — 여기까지" % res.reason)
            return False
        st = rig.measure_still(STILL_S, "rotcenter")
        if not st["ok"]:
            rig.log("  !! 태그를 잃었다 — 여기까지")
            return False
        row = _stop_row(st, sign, rig.gyro.angle_deg - psi0)
        stops.append(row)
        rig.rec.event("rotcenter_stop", **_json_safe(row))
        rig.log("     점 %2d: ψ %+6.1f도  β %+6.2f  좌우 %+.3f  앞 %.3f  방향 %+.2f"
                % (len(stops) - 1, row["psi_rel_deg"], row["beta_deg"], row["lateral"], row["forward"], row["heading_deg"]))
        return True

    # ① 한쪽으로 3 걸음 → 태그가 화면에서 몇 배로 움직이나 (plan 4-7 ③)
    for _ in range(3):
        if not (can_step(+1) and do_step(+1)):
            break
    if len(stops) >= 3:
        state["gain"] = A.gain_of(stops)
        if abs(state["gain"]) >= 1.0:
            state["swing"] = SWING_MAX_DEG / 2      # 중심이 뒤 — 태그가 더 빨리 나간다
        rig.log("  배율 dβ/dψ = %.2f → 중심이 %s. 스윙 ±%.0f도" % (state["gain"], "앞" if abs(state["gain"]) < 1 else "뒤", state["swing"]))
    # ② 끝까지 → ③ 반대쪽 끝까지 (가운데 통과) → ④ 가운데로
    while can_step(+1) and do_step(+1):
        pass
    while can_step(-1) and do_step(-1):
        pass
    while stops[-1]["psi_rel_deg"] < -step / 2 and can_step(+1) and do_step(+1):
        pass
    fit = A.rot_center_fit(stops)
    if fit.get("why"):
        raise RuntimeError("정지점 %d 개 — 적합 불가" % fit["n"])
    m = rig.m
    m.cam_to_rot_center_m = fit["cam_to_rot_center_m"]
    m.rot_center_lateral_m = fit["rot_center_lateral_m"]
    m.circle_rms_mm = fit["circle_rms_mm"]
    rig.log("  회전중심 (위치+방향, n %d, 스윙 %.0f도): 앞뒤 %+.3f m (음수 = 카메라 앞) · 좌우 %+.3f m · 잔차 %.0f mm"
            % (fit["n"], fit["swing_deg"], fit["cam_to_rot_center_m"], fit["rot_center_lateral_m"], fit["ls_rms_mm"]))
    rig.log("  검산 (위치만 동심원): 반지름 %.3f m · RMS %.1f mm · 연속 쌍 %d개 중앙 %s 흩어짐 %s"
            % (fit["circle_radius_m"], fit["circle_rms_mm"], fit["pairs_n"],
               "%+.2f" % fit["pairs_cam_to_rot_center_m"] if fit["pairs_cam_to_rot_center_m"] is not None else "-",
               "%.2f m" % fit["pairs_spread_m"] if fit["pairs_spread_m"] is not None else "-"))
    for side, v in fit["sides"].items():
        rig.log("     %s 스텝만: %+.3f m (n %d, rms %.0f mm)" % (side, v["cam_to_rot_center_m"], v["n"], v["rms_mm"]))
    if not (-1.8 <= fit["cam_to_rot_center_m"] <= -0.4):
        rig.log("  !! 예상 범위(카메라 앞 0.4~1.8 m) 밖이다 — 측정을 다시 본다 (plan 4-7 검증 기준)")
    if m.sigma_drift_lateral_m and m.sigma_ref_distance_m:
        unit = m.sigma_drift_lateral_m * (s0["distance"] / m.sigma_ref_distance_m) ** 2 * 1e3
        ratio = fit["circle_rms_mm"] / unit if unit > 0 else float("inf")
        rig.log("  RMS / 좌우 밀림 σ(%.0f mm) = %.1f → %s" % (unit, ratio,
                "중심 고정 (1배 이내)" if ratio <= 1 else "조금 흔들린다 (3배 이내) — 주행 중 갱신" if ratio <= 3
                else "원이 아니다 — 좌·우 따로 보고, 회전 상한을 줄여라"))
    rig.harvest_learner()
    out = {k: fit.get(k) for k in ("n", "swing_deg", "cam_to_rot_center_m", "rot_center_lateral_m", "circle_rms_mm",
                                   "ls_rms_mm", "circle_radius_m", "gain", "pairs_n", "pairs_spread_m", "sides")}
    rig.rec.event("rotcenter", stops=stops, **_json_safe(out))
    return out


def stage_backspeed(rig):
    why = rig.ready_to_move(need_gyro=False)
    if why:
        return {"skip": why}
    pulse_m = C.BACK_MAX_M                                    # 한 번에 물러나는 거리 = 후진 대체값과 같은 0.5 m
    space = C.BACK_MAX_M                                      # 뒤공간은 config (현장 값)
    if 2 * pulse_m > space:
        pulse_m = space / 2.0
    # 후진 속도는 아직 모른다 — 유지시간은 직진 씨앗(광운대 67)으로 잡는다. 9/7 후진 7회가 직진과 비슷했다
    v0, su0 = M.KWU_SEED["fwd_speed_mps"]["67"], M.KWU_SEED["fwd_startup_s"]["67"]
    hold_s = su0 + pulse_m / v0
    if rig.confirm("뒤에 %.1f m 이상 비어 있나 (0.5 m 씩 두 번 물러난다)" % (2 * pulse_m)) != "go":
        return {"skip": "뒤공간 미확인"}
    fits = []
    for k in range(2):
        out = rig.hold("backward", hold_s, "backspeed")
        f = A.speed_from_track(out["track"], out["sigma_m"], out["t_stop_cmd"] - out["t_cmd"])
        rig.log("  %d: %s 속도 %s m/s · 출발 %s s · %d 점 · 간 거리 %s m"
                % (k + 1, out["reason"], "%.3f" % f["speed_mps"] if f.get("speed_mps") else "-",
                   "%.2f" % f["onset_s"] if f.get("onset_s") else "-", f.get("n", 0),
                   "%.3f" % f["moved_m"] if f.get("moved_m") is not None else "-"))
        fits.append(f)
        rig.rec.event("backspeed", k=k, **_json_safe(f))
    speeds = [f["speed_mps"] for f in fits if f.get("speed_mps")]
    starts = [f["onset_s"] for f in fits if f.get("onset_s")]
    if not speeds:
        raise RuntimeError("후진 속도를 못 냈다 (%s)" % [f.get("why") for f in fits])
    v, su = S.median(speeds), S.median(starts)
    rig.m.back_speed_mps, rig.m.back_startup_s = v, su
    kb = str(F.strength_of("backward"))
    _seed(rig.learner.fwd_speed[kb], v)
    _seed(rig.learner.fwd_startup[kb], su)
    rig.log("  후진(187) %.3f m/s · 출발 %.2f s → 이제부터 되돌아오기를 폐루프 후진으로 한다" % (v, su))
    return {"back_speed_mps": v, "back_startup_s": su, "fits": fits}


def stage_fwdspeed(rig):
    why = rig.ready_to_move(need_gyro=False)
    if why:
        return {"skip": why}
    st = rig.measure_still(STILL_S, "fwdspeed")
    if not st["ok"]:
        raise RuntimeError("태그가 안 보인다")
    want = rig.near_m() + C.STEP_FORWARD_HARD_MAX_M + C.FWD_TOL_M
    if st["forward"] < want:
        st = rig.reposition(want, "fwdspeed") or st
    room = st["forward"] - rig.near_m()
    leg = min(C.STEP_FORWARD_HARD_MAX_M, room)
    floor = F.min_step_m(rig.learner, F.strength_of("forward")) or 0.0
    if leg <= max(floor, 0.0):
        raise RuntimeError("자리가 없다 — 태그 앞 %.1f m 이상에 세워라" % want)
    rows = []
    for k in range(2):
        res = rig.leg(leg, "forward")
        rows.append({"k": k, "ok": res.ok, "reason": res.reason, "travelled_m": res.travelled_m, "speed_mps": res.speed_mps})
        if k == 0:
            if rig.back_model():
                rig.leg(leg, "backward")               # 되돌아온다 — 후진도 같이 배운다
            elif room < 2 * leg + C.FWD_TOL_M:
                rig.go_to(st["forward"], "fwdspeed 복귀")
    rig.harvest_learner()
    L, k67 = rig.learner, str(F.strength_of("forward"))
    rig.log("  직진(67): 정속 %.3f m/s (n %d) · 출발 %.2f s · τ %.3f s · 잔여 %+.3f m"
            % (L.fwd_speed[k67].value, L.fwd_speed[k67].n, L.fwd_startup[k67].value, L.fwd_tau[k67].value,
               L.fwd_residual.value))
    return {"rows": rows, "leg_m": leg, "learned": _json_safe(L.dump())}


def stage_sigma_drive(rig):
    why = rig.ready_to_move(need_gyro=False)
    if why:
        return {"skip": why}
    st = rig.measure_still(STILL_S, "sigma_drive")
    if not st["ok"]:
        raise RuntimeError("태그가 안 보인다")
    near = rig.near_m()
    want = near + C.STEP_FORWARD_HARD_MAX_M + C.FWD_TOL_M
    if st["forward"] < want:
        st = rig.reposition(want, "sigma_drive") or st
    far = st["forward"]
    leg = min(C.STEP_FORWARD_HARD_MAX_M, far - near)
    if leg <= (F.min_step_m(rig.learner, F.strength_of("forward")) or 0.0):
        raise RuntimeError("자리가 없다 — 태그 앞 %.1f m 이상에 세워라" % want)
    rig.log("  %.2f m 다리를 앞뒤로 이어 움직인 시간 %.0f초를 모은다 (%.1f ~ %.1f m)" % (leg, rig.args.sigma_s, near, far))
    moving_s, legs, fails, direction = 0.0, [], 0, "forward"
    ref = rig.m.sigma_ref_distance_m
    while moving_s < rig.args.sigma_s and fails < 2:
        with rig._lock:
            rig._collect = []
        res = rig.leg(leg, direction)
        with rig._lock:
            rows, rig._collect = rig._collect, None
        if not res.done or res.reason not in ("predicted", "settle_timeout", "settle_blind"):
            fails += 1
            rig.log("  !! 다리 실패(%s)" % res.reason)
            if direction == "backward":
                direction = "forward"
            continue
        t0, t1 = (res.t_onset or res.t_cmd), res.t_stop_cmd
        moving = [r for r in rows if t0 <= r["t"] <= t1]
        moving_s += max(0.0, t1 - t0)
        d = A.drive_sigma(moving, ref)
        leg_row = {"k": len(legs), "movement": direction, "t_onset": t0, "t_stop_cmd": t1, "n": len(moving), "sigma": d}
        legs.append(leg_row)
        rig.rec.event("sigma_drive_leg", **_json_safe(leg_row))
        if d:
            rig.log("     다리 %d %s: 좌우 밀림 %.1f mm · 방향 %.3f도 (거리 %.2f m, %d 장, 누적 %.0f s)"
                    % (len(legs), direction, d["lateral_drift_m"] * 1e3, d["heading_drift_deg"], d["distance_med_m"], d["n"], moving_s))
        if direction == "forward":
            if rig.back_model():
                direction = "backward"
            elif not rig.go_to(far, "sigma_drive 복귀"):
                break
        else:
            direction = "forward"
    good = [l["sigma"] for l in legs if l["sigma"]]
    if not good:
        raise RuntimeError("쓸 만한 다리가 없다")
    rig.m.sigma_drive_lateral_m = S.median(x["lateral_drift_m"] for x in good)
    rig.m.sigma_drive_heading_deg = S.median(x["heading_drift_deg"] for x in good)
    rig.log("  직진 σ (기준 %.2f m 로 옮김): 좌우 밀림 %.1f mm · 방향 %.3f도  ← 정지 %.1f mm · %.3f도"
            % (ref or 0.0, rig.m.sigma_drive_lateral_m * 1e3, rig.m.sigma_drive_heading_deg,
               (rig.m.sigma_drift_lateral_m or 0.0) * 1e3, rig.m.sigma_drift_heading_deg or 0.0))
    rig.rebuild_estimator()
    rig.harvest_learner()
    return {"legs": len(legs), "moving_s": moving_s, "sigma_drive_lateral_m": rig.m.sigma_drive_lateral_m,
            "sigma_drive_heading_deg": rig.m.sigma_drive_heading_deg}


def stage_veer(rig):
    why = rig.ready_to_move()
    if why:
        return {"skip": why}
    st = rig.measure_still(STILL_S, "veer")
    if not st["ok"]:
        raise RuntimeError("태그가 안 보인다")
    near, run = rig.near_m(), rig.args.veer_m
    if st["forward"] < near + run:
        st = rig.reposition(near + run + C.FWD_TOL_M, "veer") or st
    if st["forward"] < near + run:
        run = st["forward"] - near
        rig.log("  !! 태그 앞 %.1f m 이상이라야 %.1f m 를 달린다 — 지금 %.2f m, %.2f m 만 간다"
                % (near + rig.args.veer_m, rig.args.veer_m, st["forward"], run))
    if run <= (F.min_step_m(rig.learner, F.strength_of("forward")) or 0.0):
        raise RuntimeError("달릴 자리가 없다")
    n_legs = max(1, int(math.ceil(run / C.STEP_FORWARD_HARD_MAX_M)))
    trials = []
    start_fwd = st["forward"]
    for k in range(VEER_N):
        rig.recalibrate()                                  # 자이로가 "휘었나" 를 판정한다 — 시행마다 영점
        s0 = rig.measure_still(STILL_S, "veer_start")
        g0 = rig.gyro.angle_deg
        with rig._lock:
            rig._collect = []
        oks = []
        for _ in range(n_legs):
            res = rig.leg(run / n_legs, "forward")
            oks.append(res.ok)
            if not res.done:
                break
        with rig._lock:
            rows, rig._collect = rig._collect, None
        s1 = rig.measure_still(STILL_S, "veer_end")
        g1 = rig.gyro.angle_deg
        heads = [r["heading_deg"] for r in rows if r["angle_ok"] and r.get("heading_deg") is not None]
        still_ok = s0["ok"] and s1["ok"]
        trial = {"k": k, "lat0": s0["lateral"], "fwd0": s0["forward"], "lat1": s1["lateral"], "fwd1": s1["forward"],
                 "heading_deg": S.median(heads) if heads else s0["heading_deg"],
                 "gyro_delta_deg": g1 - g0, "legs_ok": all(oks) and len(oks) == n_legs and still_ok,
                 "n_legs": n_legs, "ok_still": still_ok}
        trials.append(trial)
        rig.rec.event("veer_trial", **_json_safe(trial))
        one = A.veer_from_trials([trial], GYRO_STRAIGHT_TOL_DEG)["rows"][0]
        if still_ok:
            rig.log("  시행 %d: 좌우 %+.3f → %+.3f m, %.2f m 감 · 진행각 %s · 카메라 방향 %+.2f · 자이로 %+.2f도 → %s"
                    % (k + 1, trial["lat0"], trial["lat1"], one["run_m"] or 0.0,
                       "%+.2f" % one["travel_deg"] if one["travel_deg"] is not None else "-", trial["heading_deg"] or 0.0,
                       trial["gyro_delta_deg"], "쓴다 %+.2f도" % one["veer_deg"] if one["used"] else "버림"))
        else:
            rig.log("  시행 %d: 시작/끝 정지 측정이 안 됐다 (다리 %s) — 버림" % (k + 1, ["ok" if o else "실패" for o in oks]))
        if k < VEER_N - 1 and not rig.go_to(start_fwd, "veer 복귀"):
            rig.log("  !! 출발점으로 못 돌아갔다 — 여기까지")
            break
    r = A.veer_from_trials(trials, GYRO_STRAIGHT_TOL_DEG)
    if r["veer_deg"] is None:
        raise RuntimeError("쓸 만한 시행이 없다 (자이로가 %.1f도 넘게 돌았거나 다리가 실패)" % GYRO_STRAIGHT_TOL_DEG)
    rig.m.veer_deg = r["veer_deg"]
    rig.log("  쏠림 %+.2f도 (흩어짐 %s, %d/%d 시행) — 1.78 m 눈 감고 가면 %.0f mm"
            % (r["veer_deg"], "%.2f" % r["spread_deg"] if r["spread_deg"] is not None else "-", r["n_used"], r["n"],
               abs(limits.lateral_leak_m(C.BLIND_M, r["veer_deg"])) * 1e3))
    rig.harvest_learner()
    rig.rec.event("veer", **_json_safe(r))
    return {k: r[k] for k in ("veer_deg", "spread_deg", "n_used", "n")}


def stage_tagcut(rig):
    why = rig.ready_to_move(need_gyro=False)
    if why:
        return {"skip": why}
    st = rig.measure_still(STILL_S, "tagcut")
    if not st["ok"]:
        raise RuntimeError("태그가 안 보인다")
    cut_calc = limits.tag_cut_m(rig.m.intrinsics, rig.m.height_diff_m)     # 피치 0 가정 (높이차: sigma_still 것, 없으면 config)
    if st["forward"] < cut_calc + NEAR_MARGIN_M:
        st = rig.reposition(TAPE_CHECK_M, "tagcut") or st
    k67 = F.strength_of("forward")
    v = rig.learner.fwd_speed_mps(k67) or M.KWU_SEED["fwd_speed_mps"]["67"]
    su = rig.learner.fwd_startup_s(k67)
    su = M.KWU_SEED["fwd_startup_s"]["67"] if su is None else su
    hold_s = F.ABORT_FACTOR * (su + max(0.0, st["forward"] - cut_calc + D.TAG_SIZE_M) / v)
    rig.log("  태그 앞 %.2f m 에서 출발. 계산 태그컷 %.2f m. 태그를 %d 장 연속 놓치면 선다 (최대 %.1fs). 끝나면 눈이 먼 채다"
            % (st["forward"], cut_calc, TAG_LOST_N, min(hold_s, C.FWD_SAFETY_MAX_S)))
    out = rig.hold("forward", hold_s, "tagcut", stop_when=lambda: rig.miss >= TAG_LOST_N)
    r = A.tag_cut_from_track(out["track"])
    rig.log("  %s · 가장자리 %.0f → %.0f px · 마지막으로 본 거리 %.3f m · 여유 %.0f px 지점 %s"
            % (out["reason"], r.get("edge_start_px") or 0.0, r.get("edge_min_px") or 0.0, r["last_seen_m"] or 0.0,
               D.TAG_EDGE_MARGIN_PX, "%.3f m" % r["tag_cut_m"] if r["tag_cut_m"] else "못 지남"))
    if r["tag_cut_m"] is None:
        raise RuntimeError("가장자리가 %.0f px 를 안 지났다 — 더 가까이서 다시 (%s)" % (D.TAG_EDGE_MARGIN_PX, out["reason"]))
    rig.m.tag_cut_m = r["tag_cut_m"]                       # 대조용 — 실행은 안 쓴다 (프레임마다 limits.tag_cut_live_m)
    rig.log("  태그컷 실측 %.3f m (피치 0 가정 계산 %.2f, 차이 %+.3f m). 눈 감는 거리 = 태그컷 − 포크끝 = %.2f m"
            % (r["tag_cut_m"], cut_calc, r["tag_cut_m"] - cut_calc, r["tag_cut_m"] - C.CAM_TO_FORK_TIP_M))
    # 실행이 쓰는 실시간 식을 이 다리의 프레임마다 돌려 실측과 견준다 — 이게 안 맞으면 run 의 "도착" 이 틀린다
    chk = A.tag_cut_live_check(out["track"], rig.m.intrinsics, r["tag_cut_m"])
    r.update({"live_" + k: v for k, v in chk.items()})
    if chk["n"]:
        rig.log("  실시간 식(프레임 %d 장): 예측 중앙값 %.3f m · 흩어짐 ±%.3f · 멀리서(%.1f m) %.3f → 가까이서 %.3f · 실측과 차 %+.3f m"
                % (chk["n"], chk["pred_median_m"], chk["pred_sd_m"], chk["far_m"], chk["pred_far_m"], chk["pred_near_m"], chk["err_m"]))
        if abs(chk["err_m"]) > C.FWD_TOL_M:
            rig.log("  !! 실시간 식이 실측과 %.2f m 넘게 다르다 — 윗변 행·vertical·intrinsics 를 의심 (run 의 도착 판정이 이 식이다)" % C.FWD_TOL_M)
    else:
        rig.log("  !! 실시간 식을 대조할 프레임이 없다 (vertical·윗변 행이 기록에 없다)")
    if abs(r["tag_cut_m"] - cut_calc) > C.FWD_TOL_M:
        rig.log("  (피치 0 가정 계산과 %.2f m 넘게 다르다 — 경사·카메라 피치면 실시간 식이 맞아야 정상. camcheck 와 같이 보라)" % C.FWD_TOL_M)
    r["cut_calc_m"] = cut_calc
    rig.rec.event("tagcut", **_json_safe(r))
    return r


#: 이름 → (함수, 제목, 무엇을 하나, 안전 안내, 움직이나)
STAGES = {
    "device": (stage_device, "장비", "가속도계·자이로·카메라·CAN 을 보고 intrinsics 를 참고값과 대조한다", "", False),
    "human": (stage_human, "조건 3개", "카메라·태그를 다시 달았나, 메모. 줄자값은 전부 config (묻지 않는다)", "", False),
    "camcheck": (stage_camcheck, "카메라 대조 3종", "줄자 %.1f m 법선 위에 세우고 태그 px·자세 거리·태그 행을 예측과 견준다" % TAPE_CHECK_M,
                 "차는 정지. 카메라 렌즈에서 태그면까지 줄자로 %.1f m" % TAPE_CHECK_M, False),
    "camyaw": (stage_camyaw, "카메라 틀어진 각", "법선 위에 차체를 평행하게 세운 채 heading 을 읽는다 = cam_yaw_offset",
               "차체 양옆 같은 지점에서 태그 벽까지 줄자 거리가 같게 (차체 ∥ 법선), 카메라는 태그 정면", False),
    "tagroll": (stage_tagroll, "태그 액자 기울기", "중력(가속도계)과 태그 가로축의 각. 정지 중에만 맞다", "차는 정지", False),
    "sigma_still": (stage_sigma_still, "σ 정지 %.0f초" % SIGMA_S, "0.5초 창 평균의 흔들림(밀림) → 추정기의 σ 기준선", "차는 정지. 카메라 앞을 지나가지 마라", False),
    "rotfloor": (stage_rotfloor, "회전 하한", "움직이자마자 끊기를 좌·우 %d회씩 → 최대각" % ROT_FLOOR_N,
                 "제자리에서 조금씩 돈다. 사람은 제동 위치", True),
    "rotresp": (stage_rotresp, "회전 응답 (강도 30)", "±%.0f도 %d회씩 → τ·각속도·출발지연" % (ROT_RESPONSE_DEG, ROT_RESPONSE_N),
                "좌우로 12도씩 돈다. 회전중심이 앞에 있으면 카메라가 30 cm 씩 옆으로 간다", True),
    "rotcenter": (stage_rotcenter, "회전중심 (동심원)", "5도씩 ±%.0f도 스윙, 점마다 정지 자세 → 중심 + RMS" % SWING_MAX_DEG,
                  "카메라가 반지름 ~1.5 m 원을 그린다 — 양옆 2 m 를 비워라. 태그 3.5~4 m 정면에서", True),
    "backspeed": (stage_backspeed, "후진 속도 (187)", "%.1f m 씩 두 번 물러나며 카메라로 거리 변화" % C.BACK_MAX_M, "뒤 %.1f m 이상 비워라" % (2 * C.BACK_MAX_M), True),
    "fwdspeed": (stage_fwdspeed, "직진 속도 (67)", "카메라 폐루프로 ≤1.5 m 두 번 → 정속·출발·τ", "앞이 비어야 한다. 태그컷 앞에서 선다", True),
    "sigma_drive": (stage_sigma_drive, "σ 직진 %.0f초" % SIGMA_S, "다리를 앞뒤로 이어 움직인 시간을 모아 같은 계산", "태그 앞 ~5 m 에서. 앞뒤가 비어야 한다", True),
    "veer": (stage_veer, "쏠림", "%.0f m 직진 %d회, 시작·끝 좌우로 진행각 − 카메라 방향. 자이로가 돈 시행은 버린다" % (VEER_M, VEER_N),
             "태그 앞 %.1f m 이상에서 출발. 앞 4 m 를 비워라" % (VEER_M + 3.6), True),
    "tagcut": (stage_tagcut, "태그컷 대조", "천천히 다가가 실제로 안 보이게 되는 거리를 재고 실시간 식과 견준다 (실행은 프레임마다 다시 잰다). 끝나면 태그가 안 보이는 자리에 선다",
               "태그 앞 4 m 쯤에서 출발. 벽까지 3 m 는 남는다. 마지막 단계", True),
}
ORDER = ("device", "human", "camcheck", "camyaw", "tagroll", "sigma_still",
         "rotfloor", "rotresp", "rotcenter", "backspeed", "fwdspeed", "sigma_drive", "veer", "tagcut")


def run_stage(rig, name, state):
    fn, title, what, safety, motion = STAGES[name]
    rig.log("")
    rig.log("== [%s] %s" % (name, title))
    rig.log("   %s" % what)
    if safety:
        rig.log("   안전: %s" % safety)
    a = rig.confirm("시작")
    if a == "quit":
        raise KeyboardInterrupt
    if a == "skip":
        state["skipped"][name] = "사람이 건너뜀"
        rig.log("   건너뜀")
        return
    rig.stage = name
    t0 = clock.now()
    rig.rec.event("stage", name=name, status="start", t=t0)
    status, out = "done", {}
    try:
        out = fn(rig) or {}
        if out.get("skip"):
            status = "skipped"
            rig.log("   건너뜀: %s" % out["skip"])
    except KeyboardInterrupt:
        raise
    except Exception as e:
        status, out = "failed", {"error": "%s: %s" % (type(e).__name__, e)}
        rig.log("   !! 실패: %s (다음 단계로 간다)" % out["error"])
        try:
            rig.log_file_only(traceback.format_exc())
        except Exception:
            pass
    finally:
        if rig.driver is not None:
            try:
                rig.driver.stop("stage %s end" % name)
            except Exception:
                pass
        rig.stage = ""
    state[status][name] = _json_safe(out)
    if status == "done":
        for k in ("failed", "skipped"):
            state[k].pop(name, None)
    rig.rec.event("stage", name=name, status=status, t=clock.now(), elapsed_s=clock.now() - t0,
                  **({"error": out["error"]} if "error" in out else {}))


def save(rig, state):
    """measured.json · state.json · learned.json. 단계마다 부른다 — 죽어도 여기까지는 남는다."""
    rig.harvest_learner()
    m = rig.m
    m.source = "before_run"
    m.date = datetime.now().isoformat(timespec="seconds")
    for k in M.Measured.__dataclass_fields__:                   # NaN 은 JSON 에 없다
        setattr(m, k, _json_safe(getattr(m, k)))
    path = M.write(rig.root, rig.run_name, m)
    state["updated"] = m.date
    state["conditions"] = rig.conditions
    (rig.dir / "state.json").write_text(json.dumps(_json_safe(state), ensure_ascii=False, indent=1))
    (rig.dir / "learned.json").write_text(json.dumps(_json_safe(rig.learner.dump()), ensure_ascii=False, indent=1))
    return path


def report(rig, state):
    m = rig.m
    rig.log("")
    rig.log("== 결과 %s" % (rig.dir / "measured.json"))
    rig.log("   끝난 단계 %s" % list(state["done"]))
    if state["failed"]:
        rig.log("   실패 %s" % {k: v.get("error") for k, v in state["failed"].items()})
    if state["skipped"]:
        rig.log("   건너뜀 %s" % list(state["skipped"]))
    missing = [n for n in M.MUST_MEASURE if getattr(m, n, None) is None]
    if missing:
        rig.log("   !! MUST_MEASURE 가 비었다: %s — run.py 가 출발을 막는다" % missing)
    else:
        rig.log("   MUST_MEASURE 전부 있음")
    for name in ("cam_to_rot_center_m", "rot_center_lateral_m", "circle_rms_mm", "cam_yaw_offset_deg", "tag_roll_deg",
                 "tag_cut_m", "rot_floor_deg", "veer_deg", "sigma_ref_distance_m", "sigma_drift_lateral_m",
                 "sigma_drift_heading_deg", "sigma_drive_lateral_m", "sigma_drive_heading_deg", "back_speed_mps",
                 "back_startup_s", "fwd_residual_m"):
        v = getattr(m, name)
        rig.log("   %-26s %s" % (name, "-" if v is None else "%.4f" % v))
    for name in ("rot_tau_s", "rot_rate_dps", "rot_startup_s", "rot_residual_deg", "fwd_speed_mps", "fwd_startup_s", "fwd_tau_s"):
        rig.log("   %-26s %s" % (name, getattr(m, name)))
    kb = str(F.strength_of("backward"))
    eb = rig.learner.fwd_speed.get(kb)
    if eb is not None and eb.n > 0:
        rig.log("   (폐루프 후진에서 배운 값: %.3f m/s · 출발 %.2f s, n %d — 위 back_* 는 직접 측정)"
                % (eb.value, rig.learner.fwd_startup[kb].value, eb.n))
    rig.log("   원시 기록 %s · 학습 %s · 다시 내기: python tools/analyze_before_run.py %s"
            % (rig.dir / "event.jsonl", rig.dir / "learned.json", rig.dir))


def activate(run_name):
    """config/measured.py 의 ACTIVE_RUN 한 줄만 바꾼다."""
    path = ROOT / "config" / "measured.py"
    src = path.read_text(encoding="utf-8")
    new, n = re.subn(r'^ACTIVE_RUN = .*$', 'ACTIVE_RUN = "%s"' % run_name, src, count=1, flags=re.M)
    if n != 1:
        print("!! config/measured.py 에서 ACTIVE_RUN 줄을 못 찾았다 — 손으로 넣어라: ACTIVE_RUN = \"%s\"" % run_name)
        return 1
    path.write_text(new, encoding="utf-8")
    print("config/measured.py: ACTIVE_RUN = \"%s\"" % run_name)
    return 0


def _latest_run(base):
    runs = sorted(d for d in Path(base).glob("*") if d.is_dir() and (d / "state.json").exists())
    return runs[-1] if runs else None


def _load_measured(rig, path):
    """--resume: 이미 잰 값과 학습 씨앗을 이어받는다."""
    raw = json.loads(Path(path).read_text())
    fields = M.Measured.__dataclass_fields__
    for k, v in raw.items():
        if k in fields and v is not None and v != {} and k not in ("source", "date"):
            setattr(rig.m, k, v)
    seeds = M.seeds()
    for k in ("rot_tau_s", "rot_rate_dps", "rot_startup_s", "rot_residual_deg", "fwd_tau_s", "fwd_speed_mps", "fwd_startup_s"):
        for kk, vv in (raw.get(k) or {}).items():
            if vv is not None:
                seeds.setdefault(k, {})[kk] = vv
    if raw.get("fwd_residual_m") is not None:
        seeds["fwd_residual_m"] = raw["fwd_residual_m"]
    if raw.get("rot_floor_deg") is not None:
        seeds["rot_floor_deg"] = raw["rot_floor_deg"]
    rig.learner = Learner(mode="before_run", seeds=seeds)
    _ensure_back_key(rig.learner)
    kb = str(F.strength_of("backward"))
    _seed(rig.learner.fwd_speed[kb], raw.get("back_speed_mps"))
    _seed(rig.learner.fwd_startup[kb], raw.get("back_startup_s"))


def main():
    ap = argparse.ArgumentParser(description="출발 전 측정 → work_dirs/before_run/<시각>/measured.json")
    ap.add_argument("--resume", action="store_true", help="최신 폴더에 이어서 (끝난 단계는 건너뜀)")
    ap.add_argument("--only", default="", help="이 단계만 (쉼표로 여럿). --resume 과 같이 쓰면 그 폴더에")
    ap.add_argument("--activate", action="store_true", help="끝나면 config/measured.ACTIVE_RUN 을 이 폴더로")
    ap.add_argument("--out", default=None, help="기록 루트 (외장 등). 기본 work_dirs/")
    ap.add_argument("--sigma-s", type=float, default=SIGMA_S, help="σ 기록 시간 [s]")
    ap.add_argument("--veer-m", type=float, default=VEER_M, help="쏠림 직진 거리 [m]")
    ap.add_argument("--yes", action="store_true", help="프롬프트에 기본값으로 답한다 (무인 시험용 — 실차엔 쓰지 마라)")
    ap.add_argument("--list", action="store_true", help="단계 목록")
    args = ap.parse_args()
    if args.list:
        for n in ORDER:
            fn, title, what, safety, motion = STAGES[n]
            print("  %-12s %-18s %s%s" % (n, title, what, "  [움직임]" if motion else ""))
        return 0
    only = [s.strip() for s in args.only.split(",") if s.strip()]
    bad = [s for s in only if s not in STAGES]
    if bad:
        print("!! 모르는 단계 %s. 있는 것: %s" % (bad, ", ".join(ORDER)))
        return 2
    root = work_root(args.out)
    base = check_writable(root / "before_run", need_mb=200)
    run_dir, state = None, None
    if args.resume:
        run_dir = _latest_run(base)
        if run_dir is None:
            print("!! 이어갈 폴더가 없다: %s" % base)
            return 2
        state = json.loads((run_dir / "state.json").read_text())
    if run_dir is None:
        run_dir = base / datetime.now().strftime("%Y%m%d_%H%M%S")
        run_dir.mkdir(parents=True, exist_ok=True)
        state = {"done": {}, "failed": {}, "skipped": {}, "order": list(ORDER), "argv": sys.argv[1:]}
    log = Log(run_dir / "log.txt")
    rig = None
    log("== before_run %s" % run_dir.name)
    rec = Recorder(run_dir)
    rig = Rig(args, run_dir, rec, log)
    rig.root, rig.run_name = root, run_dir.name
    rig.log_file_only = lambda text: log.f.write(text + "\n")
    if args.resume and (run_dir / "measured.json").exists():
        _load_measured(rig, run_dir / "measured.json")
        log("   이어받음: 끝난 단계 %s" % list(state["done"]))
    todo = [s for s in ORDER if (not only or s in only) and not (args.resume and s in state["done"] and not only)]

    def on_sigint(signum, frame):
        try:
            if rig.driver is not None:
                rig.driver.stop_now()                  # 먼저 세운다. 기록은 그다음
        except Exception:
            pass
        raise KeyboardInterrupt
    signal.signal(signal.SIGINT, on_sigint)

    outcome = "normal"
    try:
        rig.open()
        for name in todo:
            run_stage(rig, name, state)
            save(rig, state)
    except KeyboardInterrupt:
        outcome = "ctrl_c"
        log("\n!! Ctrl+C — 정지")
    except SystemExit as e:
        outcome = "exception"
        log("!! %s" % e)
    except Exception as e:
        outcome = "exception"
        log("!! %s: %s" % (type(e).__name__, e))
        log(traceback.format_exc())
    finally:
        try:
            rig.close()
        finally:
            try:
                save(rig, state)
            except Exception as e:
                log("!! 저장 실패: %s" % e)
            summary = rec.close(outcome={"outcome": outcome,
                                         "done": list(state["done"]), "failed": list(state["failed"]),
                                         "skipped": list(state["skipped"])})
            log("   기록 %s" % summary)
    report(rig, state)
    if args.activate and outcome == "normal":
        return activate(run_dir.name)
    log("   쓰려면: config/measured.py 의 ACTIVE_RUN = \"%s\"  (또는 --activate)" % run_dir.name)
    return 0 if outcome == "normal" else 1


if __name__ == "__main__":
    sys.exit(main())
