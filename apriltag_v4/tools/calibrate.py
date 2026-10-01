"""한 번 재서 config/control.py 에 적는 값 — 회전중심 · 카메라 어긋난 각 · 회전 하한 (2026-10-02: measured.json 폐지).

    python tools/calibrate.py                 # device → camyaw → rotfloor → rotcenter. 단계마다 설명하고 Enter 를 기다린다
    python tools/calibrate.py --only rotcenter
    python tools/calibrate.py --write         # 끝나면 config/control.py 의 해당 줄을 새 값으로 바꾼다 (기본은 찍어만 준다)
    python tools/calibrate.py --list

언제: 카메라를 다시 달았을 때(camyaw·rotcenter), 차가 바뀌었을 때(전부). 날마다가 아니다.
날마다 바뀌는 것(σ 기준선·태그 기울기)은 run.py 가 출발 뒤 그 자리에서 스스로 잰다. 회전 응답·직진 속도는 주행 중
학습이 쌓는다 — 여기서 도는 회전들도 learner 가 배워 seeds.json 으로 넘긴다 (learn.last_seeds).

    device     장비: 가속도계·자이로·카메라·CAN. intrinsics 를 참고값(D435I_COLOR_REF)과 대조
    camyaw     법선 위에 차체를 평행하게 세운 채 60초 heading 중앙값          = CAM_YAW_OFFSET_DEG
    rotfloor   움직이자마자 끊기를 좌·우 5회씩 → 최대각                        = ROT_FLOOR_DEG
    rotcenter  5도씩 ±40도 스윙, 점마다 정지 자세 → 원 맞춤 (plan 4-7)         = CAM_TO_ROT_CENTER_M · ROT_CENTER_LATERAL_M · ROT_CENTER_RMS_MM

안전: 움직이는 단계는 시작 전에 "사람이 제동 위치에 있나" 를 한 번 확인한다. Ctrl+C 는 **먼저 CAN 정지**, 그다음 기록.
조향 강도가 30 이 아니면 움직이는 단계를 거부한다 (결정 1 — 씨앗이 30 짜리다). 계산식은 tools/analyze_calibrate.py —
같은 폴더(work_dirs/calibrate/<시각>/)에서 사후 재계산. 실물 장비만 — 가짜 리그는 없다 (2026-10-01).
"""
import argparse
import json
import math
import re
import signal
import statistics as S
import sys
import threading
import time
import traceback
from datetime import datetime
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))        # tools/ 의 부모. bootstrap 이 다시 확인한다
from src.bootstrap import ROOT, check_writable, setup, work_root     # noqa: E402

setup()
sys.path.insert(0, str(ROOT / "tools"))
import analyze_calibrate as A                                        # noqa: E402  현장 계산 = 사후 재계산
from analyze_calibrate import to_vehicle_frame                       # noqa: E402

from config import control as C                                      # noqa: E402
from config import detection as D                                    # noqa: E402
from config import imu as I                                          # noqa: E402
from src import limits                                               # noqa: E402
from src.models.control import learn as LN                           # noqa: E402
from src.models.control import rotate as R                           # noqa: E402
from src.models.control.driver import Driver                         # noqa: E402
from src.models.control.learn import Learner                         # noqa: E402
from src.models.detection import estimate as E                       # noqa: E402
from src.models.detection import tag as T                            # noqa: E402
from src.models.detection.image import intrinsics_from_ref           # noqa: E402
from src.utils import clock                                          # noqa: E402
from src.utils.gyro import Gyro, Rotation, RotationResult, max_rate_dps   # noqa: E402
from src.utils.record import Recorder                                # noqa: E402

# ── 구현 세부 (config 에 올리지 않는다 — plan 12-4) ─────────────────────
STILL_S = 2 * E.WINDOW_S      # 정지 확인 창. 추정 창의 두 배라야 창 하나가 통째로 새 프레임이다
ROT_FLOOR_N = 5               # 방향당 횟수 (plan 4-5 "양방향 5회")
SWING_MAX_DEG = 40.0          # 동심원 스윙 반폭. 중심이 앞이면 ±40, 뒤면 절반 (plan 4-7 ③)
FRAME_KEYS = ("lateral", "vertical", "forward", "heading_deg", "distance", "beta_deg", "edge_px", "top_px", "tag_px",
              "row_px", "angle_ok", "reproj_px", "tilt_deg", "tag_roll_deg", "gyro_deg", "err_ratio")
ANGLE_KEYS = E.Estimator.ANGLE_KEYS
#: results["config"] 의 이름 → config/control.py 의 줄. 값은 이 순서로 찍는다
CONFIG_KEYS = A.CONFIG_KEYS


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
    """limits._intr 이 읽는 이름(fx fy cx cy w h) + 반화각 + 왜곡."""
    d = {"fx": float(intr.fx), "fy": float(intr.fy), "cx": float(intr.cx), "cy": float(intr.cy),
         "w": int(intr.width), "h": int(intr.height),
         "distortion": [float(x) for x in (intr.distortion or ())]}
    d["half_fov_deg"] = limits.half_fov_deg(d)
    return d


# ═══════════════════════════════════════════════════════════════════════
# 장비 한 벌
# ═══════════════════════════════════════════════════════════════════════
class Rig:
    """카메라·자이로·CAN·추정기·학습기 + 단계들이 같이 쓰는 동작. run.py 와 같은 열기 순서(자이로 먼저)."""

    def __init__(self, args, run_dir, rec, log):
        self.args, self.dir, self.rec, self.log = args, Path(run_dir), rec, log
        self.results = {"config": {}, "stages": {}}     # config: CONFIG_KEYS → 값. stages: 단계별 출력
        learned, self.seeds_from = LN.last_seeds(work_root(args.out))
        self.learner = Learner(mode="calibrate", seeds=LN.seeds(learned))
        self.camclock = clock.CameraClock()
        self.intr = self.intr_d = self.shape = self.est = None
        self.gyro = self.driver = self.frames = self.detector = None
        self.gyro_report = None
        self.can_ok = self.cam_ok = False
        self.deflection_ok = (C.ROTATE_JOYSTICK_DEFLECTION == LN.KWU_DEFLECTION)
        self.armed = False
        self.stage = ""
        self._lock = threading.RLock()
        self._collect = None            # 켜져 있으면 프레임 상태를 여기 모은다
        self.n_frames = self.n_seen = self.miss = 0
        self._stop = threading.Event()
        self._thread = None

    # ── 수명 ──────────────────────────────────────────────────────────
    def open(self):
        from src.models.detection.image import open_realsense, to_gray
        from src.utils.camera import set_global_time, CameraSettings
        self.log("== 장비 열기 (실물)")
        # 자이로 먼저, 컬러 나중 — RSUSB 는 먼저 연 쪽이 IMU 를 갖는다 (CLAUDE.md)
        try:
            self.gyro = Gyro().start().enable_raw()
        except Exception as e:
            self.log("  !! 자이로를 못 열었다: %s — 회전 단계는 못 한다" % e)
            self.gyro = None
        self.frames, self.intr = open_realsense(stream="color", meta=True,
                                                tune=CameraSettings.docking())   # run.py 와 같은 노출 설정. 못 열면 예외 → 끝
        try:
            set_global_time(self.frames.profile)
        except Exception:
            pass
        self.detector = T.make_detector(quad_decimate=D.QUAD_DECIMATE)
        self._to_gray = to_gray
        self.cam_ok = True
        self.intr_d = _intr_dict(self.intr)
        # 추정기 — cam_yaw 0 으로 연다: camyaw 단계가 **날것** heading 을 읽어야 한다. σ 기준선은 없다(fix 는 no_sigma) —
        # 여기선 fix 를 안 쓰고 프레임 중앙값(measure_still)만 쓴다
        self.est = E.Estimator(self.intr, D.TAG_SIZE_M, cam_yaw_offset_deg=0.0, noise=None)
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
        self.rec.snapshot(sys.argv, extra={"tool": "calibrate", "intrinsics": self.intr_d,
                                           "seeds_from": str(self.seeds_from) if self.seeds_from else None})

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
        """프레임 하나. 시계 변환 → 추정기 → 기록."""
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

    def collect(self, seconds, why):
        """seconds 동안의 프레임 상태(dict) 전부. 추정 창은 비우고 시작한다."""
        with self._lock:
            self.est.reset()
            self._collect = []
        time.sleep(seconds)
        with self._lock:
            rows, self._collect = self._collect, None
        return rows

    def measure_still(self, seconds, why):
        """정지 확인 — 깨끗한 프레임의 중앙값. ok 는 MIN_N 장 이상일 때."""
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
    def confirm(self, text):
        """'go' / 'skip' / 'quit'."""
        if self.args.yes:
            return "go"
        s = input("  %s  [Enter=진행  s=건너뜀  q=끝]: " % text).strip().lower()
        return "quit" if s == "q" else "skip" if s == "s" else "go"

    def ready_to_move(self):
        """움직여도 되나. 비어 있으면 된다, 아니면 이유."""
        if self.driver is None or not self.can_ok:
            return "CAN 이 없다"
        if not self.cam_ok:
            return "카메라가 죽었다"
        if not self.deflection_ok:
            return ("조향 강도가 %d 다 — 30 이 아니면 거부 (결정 1: 광운대 씨앗이 30 짜리)"
                    % C.ROTATE_JOYSTICK_DEFLECTION)
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

    def rotate_until_onset(self, sign):
        """"자이로가 움직였다고 하면 즉시 끊어라" (plan 4-5 하한). 안전망은 rotate() 와 같은 세 겹."""
        g, drv = self.gyro, self.driver
        movement = "rotate_left_slow" if sign > 0 else "rotate_right_slow"
        startup = self.learner.rot_startup_s(sign) or LN.KWU_SEED["rot_startup_s"]["L"]
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


# ═══════════════════════════════════════════════════════════════════════
# 단계
# ═══════════════════════════════════════════════════════════════════════
def stage_device(rig):
    out = {}
    g = rig.gyro
    # 가속도계 — 태그 기울기(run 출발 전)와 회전축의 전제. 9/21 처럼 자이로만 살고 이게 죽을 수 있다
    acc = None
    if g is not None:
        for _ in range(40):
            acc = g.accel
            if acc is not None:
                break
            time.sleep(0.05)
    mag = math.sqrt(sum(a * a for a in acc)) if acc else None
    out["accel"] = {"xyz": acc, "mag": mag}
    rig.log("  가속도계: %s" % ("없음 — run 이 태그 기울기를 못 잰다·회전축 못 잰다" if acc is None
                              else "(%.2f, %.2f, %.2f) |g| %.2f %s" % (*acc, mag, "ok" if 7.0 < mag < 12.5 else "!! 중력 크기가 아니다")))
    if g is not None:
        q = g.quality()
        out["gyro"] = _json_safe({"report": {k: v for k, v in (rig.gyro_report or {}).items() if k != "accel_mean"},
                                  "quality": q, "alive": g.alive})
        rig.log("  자이로: %s Hz 실측 %.1f · 유실 %d · 처리간격 p99 %s ms (5 를 크게 넘으면 GIL 대기)"
                % ("살아있음" if g.alive else "!! 끊김", q.get("hz", 0.0), q.get("gaps", 0),
                   q.get("process_interval_ms", {}).get("p99")))
    else:
        out["gyro"] = None
    for _ in range(100):
        if rig.camclock.ready or not rig.cam_ok:
            break
        time.sleep(0.05)
    err = rig.camclock.check(rig.log)
    if err:
        rig.log("  !! %s" % err)
    ref = intrinsics_from_ref(rig.shape) if rig.shape else None
    w, h, fx, fy, cx, cy = limits._intr(rig.intr_d)
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
    out["intrinsics"] = rig.intr_d
    out["intrinsics_diff"] = _json_safe(diff)
    out["clock"] = _json_safe(rig.camclock.report())
    out["can"] = _json_safe(rig.driver.status() if rig.driver is not None else None)
    out["can_ok"] = rig.can_ok
    rig.log("  CAN: %s" % ("열림 (동결 테이블 확인 끝)" if rig.can_ok else "!! 없음 — 움직이는 단계는 못 한다"))
    out["deflection_ok"] = rig.deflection_ok
    rig.log("  조향 강도 %d (byte1 %d/%d) %s" % (C.ROTATE_JOYSTICK_DEFLECTION, 127 + C.ROTATE_JOYSTICK_DEFLECTION,
                                             127 - C.ROTATE_JOYSTICK_DEFLECTION,
                                             "" if rig.deflection_ok else "!! 30 이 아니다 — 움직이는 단계 거부"))
    rig.rec.event("device", **out)
    return out


def stage_camyaw(rig):
    # 방향값은 가만히 있어도 수십 초에 걸쳐 천천히 출렁인다(9/21: 0.5초 블록평균 sd 0.46도). 1초만 읽으면 그 오차가 그대로
    # config 에 박혀 매 주행 같은 쪽으로 샌다 — run 의 σ 기준선과 같은 시간(SIGMA_STILL_S) 동안 본다 (2026-10-02)
    sec = C.SIGMA_STILL_S
    rig.log("  %.0f초 동안 가만히 (차·사람 모두. 카메라 앞을 지나가지 마라)" % sec)
    rows = rig.collect(sec, "camyaw")
    # 움직였나는 **각속도**로 본다 (보정과 같은 기준 IMU_MOVING_DPS). 적분 각도 차는 드리프트라 못 쓴다
    gy = [r.get("gyro_deg") for r in rows]
    rate = max_rate_dps([r["t"] for r in rows], gy, E.WINDOW_S) if rig.gyro is not None else None
    gy = [x for x in gy if x is not None]
    drifted = (gy[-1] - gy[0]) if gy else None
    clean = [r for r in rows if r["angle_ok"] and r.get("heading_deg") is not None and math.isfinite(r["heading_deg"])]
    if len(clean) < 2 * E.MIN_N:
        raise RuntimeError("깨끗한 프레임이 모자란다 (%d / %d 장) — 태그가 안 보이거나 두 해가 헷갈린다" % (len(clean), len(rows)))
    if rate is not None and rate > I.IMU_MOVING_DPS:
        raise RuntimeError("재는 동안 %.2f 도/s 로 움직였다 (기준 %.1f) — 차가 흔들렸다. 다시 재라" % (rate, I.IMU_MOVING_DPS))
    heads = [r["heading_deg"] for r in clean]
    raw = S.median(heads)                                      # 추정기가 cam_yaw 0 으로 열려 있어 날것이다
    b = E.drift_baseline([r["t"] for r in clean], heads)       # 0.5초 블록평균의 흔들림 = 짧게 읽었을 때 틀리는 크기
    lat = S.median([r["lateral"] for r in clean])
    betas = [r["beta_deg"] for r in rows if r.get("beta_deg") is not None and math.isfinite(r["beta_deg"])]
    beta = S.median(betas) if betas else float("nan")
    rig.results["config"]["CAM_YAW_OFFSET_DEG"] = raw
    rig.log("  법선 위 heading 중앙값 %+.2f도 (%.0f초, 깨끗한 %d / %d 장) → CAM_YAW_OFFSET_DEG (지금 config %s)"
            % (raw, sec, len(clean), len(rows), C.CAM_YAW_OFFSET_DEG))
    if b is not None:
        rig.log("  출렁임: 한 장 sd %.2f도 · 0.5초 블록평균 sd %.2f도 (= 1초만 읽었다면 틀렸을 크기) · 블록 %d 개"
                % (S.pstdev(heads), b.block_sd, b.n_blocks))
    rig.log("  좌우 %+.3f m (0 이어야 — 카메라가 태그 정면) · β %+.2f도. 차체 ∥ 법선이 전제다 — 그 자세 오차는 여기서 못 본다" % (lat, beta))
    bb = E.drift_baseline([r["t"] for r in rows if r.get("beta_deg") is not None], betas)
    rig.log("  정지 확인: 가장 빠른 0.5초 %s 도/s (기준 %.1f) · 자이로 흐름 %s도 (드리프트 — 판정엔 안 쓴다) · β 블록평균 sd %s도 (화면 속 태그가 움직인 정도)"
            % ("%.3f" % rate if rate is not None else "-", I.IMU_MOVING_DPS,
               "%+.2f" % drifted if drifted is not None else "-", "%.4f" % bb.block_sd if bb else "-"))
    out = {"cam_yaw_offset_deg": raw, "seconds": sec, "n": len(rows), "n_clean": len(clean),
           "frame_sd_deg": S.pstdev(heads), "block_sd_deg": b.block_sd if b else None, "drift_deg": b.drift if b else None,
           "lateral_m": lat, "beta_deg": beta, "max_rate_dps": rate, "gyro_drift_deg": drifted,
           "beta_block_sd_deg": bb.block_sd if bb else None}
    rig.rec.event("camyaw", **_json_safe(out))
    return out


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
    rig.results["config"]["ROT_FLOOR_DEG"] = r["rot_floor_deg"]
    rig.learner.rot_floor_deg = r["rot_floor_deg"]
    rig.log("  회전 하한 = 최대 %.2f도 (중앙 %.2f, sd %s, n %d) → ROT_FLOOR_DEG (지금 config %s. 씨앗 0.81 은 유도값)"
            % (r["rot_floor_deg"], r["median_deg"], "%.2f" % r["sd_deg"] if r["sd_deg"] is not None else "-", r["n"],
               C.ROT_FLOOR_DEG))
    r["rows"] = rows
    rig.rec.event("rotfloor_summary", **_json_safe(r))
    return r


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
    beta_max = limits.path_angle_max_deg(s0["forward"], rig.intr_d)   # 태그가 화면에 남는 β 한계
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
    # 주 적합 = 거리 + 화면위치(β) 원 맞춤. PnP 방향·좌우는 이 거리에서 자세마다 몇 도씩 치우쳐서 못 쓴다 (analyze 의 설명)
    dz = -s0["vertical"] if s0.get("vertical") is not None else (D.TAG_HEIGHT_M - D.CAMERA_HEIGHT_M)
    fit = A.rot_center_from_bearing(stops, dz)
    if fit.get("why"):
        raise RuntimeError("정지점 %d 개 — 적합 불가" % fit["n"])
    cfg = rig.results["config"]
    yaw = cfg.get("CAM_YAW_OFFSET_DEG", C.CAM_YAW_OFFSET_DEG)      # 이번에 잰 값 먼저, 없으면 config
    ctr_a, ctr_b = fit["cam_to_rot_center_m"], fit["rot_center_lateral_m"]      # 카메라 축 (A 는 이 파일에서 analyze_calibrate 모듈이다)
    rig.log("  회전중심 (거리+화면위치 원 맞춤, n %d, β 폭 %.0f도, 높이차 %.2f m): 앞뒤 %+.3f ±%s m (음수 = 카메라 앞) · 좌우 %+.3f ±%s m · RMS %.1f mm"
            % (fit["n"], fit["beta_span_deg"], dz, ctr_a, "%.3f" % fit["se_forward_m"] if fit["se_forward_m"] is not None else "-",
               ctr_b, "%.3f" % fit["se_lateral_m"] if fit["se_lateral_m"] is not None else "-", fit["circle_rms_mm"]))
    if yaw is not None:
        ctr_a, ctr_b = to_vehicle_frame(ctr_a, ctr_b, yaw)
        rig.log("  차체 축으로 (cam_yaw %+.2f도 만큼 돌림): 앞뒤 %+.3f m · 좌우 %+.3f m  ← config 에 적는 값 (지금 config %s / %s)"
                % (yaw, ctr_a, ctr_b, C.CAM_TO_ROT_CENTER_M, C.ROT_CENTER_LATERAL_M))
    else:
        rig.log("  !! cam_yaw 를 모른다(이번에 안 쟀고 config 도 None) — 위 값은 **카메라 축** 그대로다. camyaw 를 잰 뒤 "
                "analyze_calibrate.to_vehicle_frame 으로 돌리거나 rotcenter 를 다시 하라 (|A|·sin(yaw) 만큼 옆으로 틀린다)")
    cfg["CAM_TO_ROT_CENTER_M"] = ctr_a
    cfg["ROT_CENTER_LATERAL_M"] = ctr_b
    cfg["ROT_CENTER_RMS_MM"] = fit["circle_rms_mm"]
    if not (-1.8 <= ctr_a <= -0.2):
        rig.log("  !! 예상 범위(카메라 앞 0.2~1.8 m) 밖이다 — 측정을 다시 본다 (첫 실측 0.46 m, 광운대 차 0.68 m)")
    # 진단: PnP 위치+방향 적합 (옛 방식). 크게 다르면 PnP 방향이 이 스윙에서 얼마나 치우쳤는지 보여 준다
    old = A.rot_center_fit(stops)
    if not old.get("why"):
        rig.log("  (진단 — PnP 위치+방향 적합: 앞뒤 %+.3f · 좌우 %+.3f · 잔차 %.0f mm · 연속 쌍 흩어짐 %s. 주 적합과 다르면 PnP 방향이 치우친 것)"
                % (old["cam_to_rot_center_m"], old["rot_center_lateral_m"], old["ls_rms_mm"],
                   "%.2f m" % old["pairs_spread_m"] if old["pairs_spread_m"] is not None else "-"))
        dh = [s["heading_deg"] - s0["heading_deg"] - s["psi_rel_deg"] for s in stops]
        rig.log("  (진단 — PnP 방향 − (시작 방향 + 자이로): %+.1f ~ %+.1f도. 0 근처여야 PnP 방향을 믿을 수 있다)" % (min(dh), max(dh)))
    out = {k: fit.get(k) for k in ("n", "cam_to_rot_center_m", "rot_center_lateral_m", "circle_rms_mm", "circle_radius_m",
                                   "se_forward_m", "se_lateral_m", "beta_span_deg", "height_diff_m")}
    out.update(gain=A.gain_of(stops), cam_yaw_used_deg=yaw, config_cam_to_rot_center_m=ctr_a, config_rot_center_lateral_m=ctr_b,
               pnp_fit={k: old.get(k) for k in ("cam_to_rot_center_m", "rot_center_lateral_m", "ls_rms_mm", "pairs_spread_m")})
    rig.rec.event("rotcenter", stops=stops, **_json_safe(out))
    return out


#: 이름 → (함수, 제목, 무엇을 하나, 안전 안내, 움직이나)
STAGES = {
    "device": (stage_device, "장비", "가속도계·자이로·카메라·CAN 을 보고 intrinsics 를 참고값과 대조한다", "", False),
    "camyaw": (stage_camyaw, "카메라 어긋난 각", "법선 위에 차체를 평행하게 세운 채 %.0f초 heading 중앙값 = CAM_YAW_OFFSET_DEG" % C.SIGMA_STILL_S,
               "차체 양옆 같은 지점에서 태그 벽까지 줄자 거리가 같게 (차체 ∥ 법선), 카메라는 태그 정면. 차는 정지", False),
    "rotfloor": (stage_rotfloor, "회전 하한", "움직이자마자 끊기를 좌·우 %d회씩 → 최대각 = ROT_FLOOR_DEG" % ROT_FLOOR_N,
                 "제자리에서 조금씩 돈다. 사람은 제동 위치", True),
    "rotcenter": (stage_rotcenter, "회전중심 (동심원)", "5도씩 ±%.0f도 스윙, 점마다 정지 자세 → 원 맞춤 = CAM_TO_ROT_CENTER_M 등 3개" % SWING_MAX_DEG,
                  "카메라가 반지름 ~1.5 m 원을 그린다 — 양옆 2 m 를 비워라. 태그 3.5~4 m 정면에서", True),
}
ORDER = ("device", "camyaw", "rotfloor", "rotcenter")


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
    rig.results["stages"][name] = _json_safe(out)
    rig.rec.event("stage", name=name, status=status, t=clock.now(), elapsed_s=clock.now() - t0,
                  **({"error": out["error"]} if "error" in out else {}))


def save(rig, state):
    """results.json · state.json · learned.json · seeds.json. 단계마다 부른다 — 죽어도 여기까지는 남는다."""
    state["updated"] = datetime.now().isoformat(timespec="seconds")
    rig.results["date"] = state["updated"]
    (rig.dir / "results.json").write_text(json.dumps(_json_safe(rig.results), ensure_ascii=False, indent=1))
    (rig.dir / "state.json").write_text(json.dumps(_json_safe(state), ensure_ascii=False, indent=1))
    (rig.dir / "learned.json").write_text(json.dumps(_json_safe(rig.learner.dump()), ensure_ascii=False, indent=1))
    # 여기서 돈 회전들이 가르친 값 — run.py 가 learn.last_seeds 로 물려받는다
    (rig.dir / "seeds.json").write_text(json.dumps(_json_safe(rig.learner.seeds()), ensure_ascii=False, indent=1))


def config_lines(values, run_name):
    """config/control.py 에 적을 줄들. 값 뒤에 출처(calibrate 폴더)를 남긴다."""
    out = []
    for k in CONFIG_KEYS:
        v = values.get(k)
        if v is None:
            continue
        fmt = "%.2f" if k in ("CAM_YAW_OFFSET_DEG", "ROT_FLOOR_DEG") else ("%.1f" if k == "ROT_CENTER_RMS_MM" else "%.3f")
        out.append((k, fmt % v))
    return out


def write_config(values, run_name):
    """config/control.py 의 해당 줄 값을 바꾼다. 설명 주석은 두고 끝에 [calibrate <폴더>] 를 붙인다."""
    path = ROOT / "config" / "control.py"
    src = path.read_text(encoding="utf-8")
    changed = []
    for k, v in config_lines(values, run_name):
        pat = re.compile(r"^(%s\s*=\s*)([^#\n]*?)(\s*#[^\n]*)?$" % k, re.M)
        m = pat.search(src)
        if m is None:
            print("!! config/control.py 에 %s 줄이 없다 — 손으로 넣어라: %s = %s" % (k, k, v))
            continue
        comment = m.group(3) or ""
        comment = re.sub(r"\s*\[calibrate [^\]]*\]", "", comment)
        comment = (comment.rstrip() + " [calibrate %s]" % run_name) if comment.strip() else "   # [calibrate %s]" % run_name
        src = src[:m.start()] + m.group(1) + v + comment + src[m.end():]
        changed.append(k)
    path.write_text(src, encoding="utf-8")
    return changed


def report(rig, state, run_name):
    rig.log("")
    rig.log("== 결과 %s" % (rig.dir / "results.json"))
    rig.log("   끝난 단계 %s" % list(state["done"]))
    if state["failed"]:
        rig.log("   실패 %s" % {k: v.get("error") for k, v in state["failed"].items()})
    if state["skipped"]:
        rig.log("   건너뜀 %s" % list(state["skipped"]))
    lines = config_lines(rig.results["config"], run_name)
    if lines:
        rig.log("")
        rig.log("== config/control.py 에 적을 값 (--write 면 자동):")
        for k, v in lines:
            rig.log("   %s = %s" % (k, v))
    else:
        rig.log("   (config 에 적을 값이 없다 — 단계가 안 끝났다)")
    rig.log("   원시 기록 %s · 학습 씨앗 %s · 다시 내기: python tools/analyze_calibrate.py %s"
            % (rig.dir / "event.jsonl", rig.dir / "seeds.json", rig.dir))


def main():
    ap = argparse.ArgumentParser(description="한 번 재서 config 에 적는 값 → work_dirs/calibrate/<시각>/results.json")
    ap.add_argument("--only", default="", help="이 단계만 (쉼표로 여럿)")
    ap.add_argument("--write", action="store_true", help="끝나면 config/control.py 의 해당 줄을 새 값으로")
    ap.add_argument("--out", default=None, help="기록 루트 (외장 등). 기본 work_dirs/")
    ap.add_argument("--yes", action="store_true", help="프롬프트에 기본값으로 답한다 (무인 시험용 — 실차엔 쓰지 마라)")
    ap.add_argument("--list", action="store_true", help="단계 목록")
    args = ap.parse_args()
    if args.list:
        for n in ORDER:
            fn, title, what, safety, motion = STAGES[n]
            print("  %-10s %-14s %s%s" % (n, title, what, "  [움직임]" if motion else ""))
        return 0
    only = [s.strip() for s in args.only.split(",") if s.strip()]
    bad = [s for s in only if s not in STAGES]
    if bad:
        print("!! 모르는 단계 %s. 있는 것: %s" % (bad, ", ".join(ORDER)))
        return 2
    root = work_root(args.out)
    base = check_writable(root / "calibrate", need_mb=200)
    run_dir = base / datetime.now().strftime("%Y%m%d_%H%M%S")
    run_dir.mkdir(parents=True, exist_ok=True)
    state = {"done": {}, "failed": {}, "skipped": {}, "order": list(ORDER), "argv": sys.argv[1:]}
    log = Log(run_dir / "log.txt")
    log("== calibrate %s" % run_dir.name)
    rec = Recorder(run_dir)
    rig = Rig(args, run_dir, rec, log)
    rig.log_file_only = lambda text: log.f.write(text + "\n")
    todo = [s for s in ORDER if (not only or s in only)]

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
    report(rig, state, run_dir.name)
    if args.write and outcome == "normal" and rig.results["config"]:
        changed = write_config(rig.results["config"], run_dir.name)
        log("   config/control.py 갱신: %s" % changed)
    return 0 if outcome == "normal" else 1


if __name__ == "__main__":
    sys.exit(main())
