"""1. 이미지 얻기 — 카메라/bag/영상에서 프레임과 내부파라미터를 냄.

어느 소스든 `for i, ts, img in frames:` 3-튜플로 통일함.
"""
from config import detection as D
from dataclasses import dataclass
import json
from pathlib import Path
import subprocess
import sys
import time

import cv2
import numpy as np

from ...utils.camera import COLOR_EXPOSURE_UNIT_US

@dataclass
class CameraIntrinsics:
    """detection_pose() 에 넘길 카메라 값."""
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0
    height: int = 0
    distortion: tuple = ()          # radtan (k1,k2,p1,p2,k3). 비어 있으면 왜곡 없음으로 봄

    @property
    def params(self):
        """apriltag 가 받는 (fx, fy, cx, cy) 튜플."""
        return (self.fx, self.fy, self.cx, self.cy)

    @property
    def K(self):
        """3x3 카메라 행렬."""
        return np.array([[self.fx, 0, self.cx],
                         [0, self.fy, self.cy],
                         [0, 0, 1]], dtype=np.float64)

    @classmethod
    def from_realsense(cls, profile, stream="color", index=1):
        """RealSense 가 들고 있는 공장 캘리브레이션 값을 그대로 읽음."""
        import pyrealsense2 as rs

        kinds = {"color": rs.stream.color, "depth": rs.stream.depth,
                 "infrared": rs.stream.infrared}
        if stream not in kinds:
            raise ValueError("모르는 스트림: %s" % stream)
        if stream == "infrared":
            vsp = profile.get_stream(kinds[stream], index).as_video_stream_profile()
        else:
            vsp = profile.get_stream(kinds[stream]).as_video_stream_profile()
        i = vsp.get_intrinsics()
        # RealSense 는 주점을 ppx, ppy 로 부름 (= cx, cy)
        return cls(fx=i.fx, fy=i.fy, cx=i.ppx, cy=i.ppy,
                   width=i.width, height=i.height,
                   distortion=tuple(i.coeffs))

    def undistort(self, img):
        """렌즈 왜곡을 폄. 왜곡계수가 없으면 그대로 돌려줌."""
        if not self.distortion:
            return img
        d = np.array(self.distortion, dtype=np.float64)
        return cv2.undistort(img, self.K, d)

def intrinsics_from_ref(shape, ref=D.D435I_COLOR_REF):
    """해상도만 알 때 기준 보정값에서 역산함. 화각 가정보다 정확함.

    D435i 는 세로 비율로만 스케일됨(4:3 은 가로를 잘라냄). 실측 대조 결과
    1280x720 / 640x480 모두 0.03px 안에서 맞았음.
    """
    h, w = shape[:2]
    W0, H0, fx0, fy0, cx0, cy0 = ref
    s = float(h) / float(H0)
    return CameraIntrinsics(fx0 * s, fy0 * s,
                            w / 2.0 + (cx0 - W0 / 2.0) * s,
                            h / 2.0 + (cy0 - H0 / 2.0) * s, w, h)

def intrinsics_from_hfov(shape, hfov_deg=None):
    """화면 크기 + 수평화각으로 CameraIntrinsics 를 지어냄.

    hfov_deg 를 안 주면 화각을 안 쓰고 intrinsics_from_ref() 로 역산함.
    해상도마다 화각이 달라서(16:9 70.5도, 4:3 55.8도) 상수 하나로는 못 맞춤.
    """
    if hfov_deg is None:
        return intrinsics_from_ref(shape)
    h, w = shape[:2]
    fx = (w / 2.0) / np.tan(np.deg2rad(float(hfov_deg)) / 2.0)
    return CameraIntrinsics(fx, fx, w / 2.0, h / 2.0, w, h)


def fov_edges_deg(intr):
    """광축에서 화면 네 끝까지의 각 [도]. (위, 아래, 왼쪽, 오른쪽).

    **위아래가 다르다.** cy 가 화면 정중앙이 아니기 때문이다 — D435i 1080p 는
    cy=571.3 이라 중앙(540)보다 31줄 아래에 있고, 그만큼 위를 더 본다
    (위 22.8도 / 아래 20.5도, 합이 세로화각 43.3도).
    태그를 올려다보는 우리 배치에서는 이 차이가 그대로 근접 한계에 들어간다.
    """
    h = int(intr.height) or int(round(intr.cy * 2))
    w = int(intr.width) or int(round(intr.cx * 2))
    up = np.degrees(np.arctan(intr.cy / intr.fy))
    down = np.degrees(np.arctan((h - intr.cy) / intr.fy))
    left = np.degrees(np.arctan(intr.cx / intr.fx))
    right = np.degrees(np.arctan((w - intr.cx) / intr.fx))
    return float(up), float(down), float(left), float(right)


def tag_visible_near_m(intr, tag_height_m, cam_height_m, tag_size_m):
    """태그 전체가 화면에 들어오는 **가장 가까운 거리** [m].

    가까이 갈수록 태그를 가파르게 올려다보게 되어, 어느 지점부터 윗변이
    화면 위로 넘어간다. AprilTag 는 네 모서리가 다 있어야 하므로 그때부터
    검출이 아예 안 된다. **회전으로는 절대 복구가 안 되는 실종**이라
    STOP_M 은 반드시 이 값보다 커야 한다.

        높이차 0.40m / 20cm 태그  ->  1.19m
        높이차 0.92m / 30cm 태그  ->  2.55m

    태그가 카메라보다 낮으면 아래쪽 화각으로 같은 계산을 한다.
    """
    up, down, _, _ = fov_edges_deg(intr)
    dh = float(tag_height_m) - float(cam_height_m)
    half = float(tag_size_m) / 2.0
    edge = (dh + half) if dh >= 0 else -(dh - half)      # 광축에서 먼 쪽 변까지
    limit = up if dh >= 0 else down
    return float(edge / np.tan(np.radians(limit)))


class Frame(np.ndarray):
    """BGR/흑백 이미지이면서 depth·휘도·메타데이터를 함께 들고 다니는 ndarray."""
    depth = None
    depth_scale = 0.0
    luma = None
    frame_number = None
    dropped_before = 0
    # meta 는 **클래스 속성으로 dict 를 두면 안 된다** — 모든 Frame 이 같은 dict 를
    meta = None
    exposure_us = None
    gain = None
    # 타이밍층(src/utils/timing.FrameStamps). 절대 캡처시각을 여기 실어 흘린다.
    stamps = None

    def __array_finalize__(self, obj):
        # 뷰/슬라이스로 파생될 때도 속성이 따라가게 함.
        if obj is None:
            return
        self.stamps = getattr(obj, "stamps", None)
        self.depth = getattr(obj, "depth", None)
        self.depth_scale = getattr(obj, "depth_scale", 0.0)
        self.luma = getattr(obj, "luma", None)
        self.frame_number = getattr(obj, "frame_number", None)
        self.dropped_before = getattr(obj, "dropped_before", 0)
        self.meta = getattr(obj, "meta", None)
        self.exposure_us = getattr(obj, "exposure_us", None)
        self.gain = getattr(obj, "gain", None)


# 컬러 센서의 노출 메타데이터 눈금. 컬러의 노출값은 UVC 원값이라 100us 눈금이고,
_EXPOSURE_UNIT_US = {"color": COLOR_EXPOSURE_UNIT_US, "infrared": 1.0}


def _attach(img, depth=None, depth_scale=0.0, luma=None, frame_number=None,
            meta=None, dropped_before=0, exposure_unit_us=1.0, stamps=None):
    """이미지에 곁다리 정보를 붙여 Frame 으로 만듦."""
    f = img.view(Frame)
    f.stamps = stamps
    f.depth = depth
    f.depth_scale = float(depth_scale)
    f.luma = luma
    f.frame_number = frame_number
    f.dropped_before = int(dropped_before or 0)
    f.meta = meta if meta is not None else {}
    exp = f.meta.get("actual_exposure")
    f.exposure_us = None if exp is None else float(exp) * float(exposure_unit_us)
    g = f.meta.get("gain_level")
    f.gain = None if g is None else int(g)
    return f


class _FrameStream:
    """프레임 제너레이터 + 곁다리 정보(ae_roi, profile)."""

    def __init__(self, gen, ae_roi=None, profile=None, stats=None,
                 tuning=None, settings_before=None, cleanup=None):
        self._gen = gen
        self.ae_roi = ae_roi          # camera.ExposureROI 또는 None
        self.profile = profile        # pipeline.start() 가 준 것. 센서 옵션을 만질 때 씀
        self.stats = stats            # camera.FrameStats — **항상 있다**
        self.tuning = tuning          # tune= 을 줬을 때 {옵션: (전, 후)}. 아니면 None
        # 손대기 전 CameraSettings. **되돌리려면 이게 있어야 한다** —
        self.settings_before = settings_before
        # 파이프라인 정지 + 카메라 상태 복원. **제너레이터가 아니라 여기가 들고 있음.**
        self._cleanup = cleanup

    def __iter__(self):
        return self

    def __next__(self):
        return next(self._gen)

    def close(self):
        """스트림을 닫고 카메라를 원래대로 돌려놓음. 몇 번 불러도 안전함."""
        try:
            self._gen.close()
        finally:
            c, self._cleanup = self._cleanup, None
            if c is not None:
                c()

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def __del__(self):
        try:
            self.close()
        except Exception:
            pass

    def send(self, value):
        return self._gen.send(value)

    def throw(self, *a, **kw):
        return self._gen.throw(*a, **kw)


def _who_has_camera():
    """/dev/video* 를 잡고 있는 프로세스를 이름까지 찾아 줌.

    EBUSY 는 원인이 둘인데 메시지가 같음 — 우리가 안 닫았거나, 남이 잡고 있거나.
    누가 잡고 있는지 이름이 나오면 바로 갈림(realsense-viewer 가 흔함).
    """
    import glob
    import subprocess
    devs = glob.glob("/dev/video*")
    if not devs:
        return " (/dev/video* 가 없다 — UTM 이면 USB 로 RealSense 를 VM 에 넘겼나 확인, WSL 이면 usbipd 로 붙일 것)"
    try:
        out = subprocess.run(["fuser"] + devs, capture_output=True, text=True, timeout=3)
        pids = sorted(set(out.stdout.split()))
    except Exception:
        return ""
    if not pids:
        return ""
    try:
        ps = subprocess.run(["ps", "-o", "pid=,cmd=", "-p", ",".join(pids)],
                            capture_output=True, text=True, timeout=3).stdout.strip()
    except Exception:
        ps = " ".join(pids)
    return "\n  지금 잡고 있는 프로세스:\n" + "\n".join("    " + l.strip()
                                                       for l in ps.splitlines())


def open_realsense(stream="color", width=None, height=None, fps=30,
                   depth=False, emitter=None, ir_index=1,
                   with_depth=False, depth_size=(1280, 720), depth_fps=None,
                   color_format="bgr8", ae_roi=False, stats=None, meta=True,
                   exposure_us=None, ae_priority=None, tune=False, restore=True,
                   record=None, timing=None):
    """RealSense 를 열고 (프레임 제너레이터, CameraIntrinsics) 를 돌려줌.

    timing: src/utils/timing.TimingSession 을 주면 프레임마다 절대 시각을 계산해
        Frame.stamps 로 실어 보낸다(plan 4-1). 3-튜플 계약(i, ts, img)은 그대로다 —
        ts 는 예전처럼 첫 프레임 기준 상대시각이고, **절대 시각은 stamps 에만** 있다.
        queue=1·global_time 은 tune= 경로에서 걸린다(CameraSettings.docking).
    """
    import pyrealsense2 as rs

    want_depth = bool(depth or with_depth)

    if width is None or height is None:
        width, height = D.COLOR_SIZE if stream == "color" else D.IR_SIZE

    pipeline = rs.pipeline()
    config = rs.config()
    if stream == "color":
        if color_format not in ("bgr8", "yuyv"):
            raise ValueError("color_format 은 bgr8 또는 yuyv")
        # D400 컬러 센서가 실제로 내줄 수 있는 포맷은 RGB8/RGBA8/BGR8/BGRA8/YUYV 뿐
        cfmt = rs.format.bgr8 if color_format == "bgr8" else rs.format.yuyv
        config.enable_stream(rs.stream.color, width, height, cfmt, fps)
    elif stream == "infrared":
        config.enable_stream(rs.stream.infrared, ir_index, width, height, rs.format.y8, fps)
    else:
        raise ValueError("stream 은 color 또는 infrared")
    if want_depth:
        dw, dh = depth_size
        config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16,
                             int(depth_fps or min(fps, 30)))

    if record:
        rec = Path(record)
        rec.parent.mkdir(parents=True, exist_ok=True)
        config.enable_record_to_file(str(rec))

    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        if "busy" in str(exc).lower():
            raise RuntimeError(
                "%s — 카메라는 한 프로세스만 연다.%s\n"
                "  같은 프로세스 안이면: 루프에서 break 했을 때 frames.close() 를 부르거나 "
                "`with open_realsense()[0] as frames:` 로 열어라."
                % (exc, _who_has_camera())) from exc
        raise

    # IR 점 프로젝터 제어 + depth 눈금 읽기 (둘 다 depth 센서에 달려 있음)
    want_emitter = (stream != "infrared") if emitter is None else bool(emitter)
    depth_scale = 0.0
    try:
        ds = profile.get_device().first_depth_sensor()
        if ds.supports(rs.option.emitter_enabled):
            ds.set_option(rs.option.emitter_enabled, 1 if want_emitter else 0)
        if want_depth:
            # 원시 uint16 을 미터로 바꾸는 눈금. 0.001 로 박아두지 말고 장치에서 읽음.
            depth_scale = ds.get_depth_scale()
    except Exception:
        pass                                    # 장치에 따라 없을 수 있음

    intr = CameraIntrinsics.from_realsense(profile, stream, ir_index)
    align = rs.align(rs.stream.color) if (want_depth and stream == "color") else None

    # 컬러 자동노출 ROI. 태그를 찾은 뒤 호출자가 roi.follow(...) 를 불러 줌.
    roi = None
    if ae_roi:
        from ...utils.camera import ExposureROI
        roi = ExposureROI(profile, stream=stream)

    # 묶음 튜닝. **pipeline.start() 뒤, 첫 wait_for_frames 전에** 걺 —
    want_tune = (tune is not False and tune is not None and stream == "color")
    want_exposure = ((exposure_us is not None or ae_priority is not None)
                     and stream == "color")

    # **카메라를 만지기 전에 지금 상태를 뜸.**
    tuning = None
    settings_before = None
    if want_tune or want_exposure:
        try:
            from ...utils.camera import CameraSettings as _CS
            settings_before = _CS.from_sensor(profile)
        except Exception:
            settings_before = None            # 못 뜨면 복원도 포기함(아래 경고)

    if want_tune:
        from ...utils.camera import CameraSettings, tune_for_tags
        if isinstance(tune, CameraSettings):
            _b, tuning = tune_for_tags(profile, settings=tune)
        elif tune is True:
            _b, tuning = tune_for_tags(profile)
        else:
            # 숫자 = 접근속도 [m/s]. 그 속도에서 블러가 10px 를 넘지 않게 노출을 잡음.
            _b, tuning = tune_for_tags(profile, speed_mps=float(tune), fx=intr.fx)
        settings_before = settings_before or _b

    # 노출/AE 우선순위. **pipeline.start() 뒤, 첫 wait_for_frames 전에** 걸어야 함.
    if want_exposure:
        from ...utils.camera import set_color_exposure
        applied = set_color_exposure(profile, exposure_us=exposure_us,
                                     ae_priority=ae_priority)
        if applied["errors"]:
            # 조용히 실패하면 "걸었다고 믿는" 상태가 됨. 그게 제일 나쁨.
            import warnings
            warnings.warn("컬러 노출 설정 실패: %s" % ", ".join(applied["errors"]))

    if restore and (want_tune or want_exposure) and settings_before is None:
        import warnings
        warnings.warn("카메라 상태를 뜨지 못해 원상복구를 못 한다 — "
                      "끝난 뒤 realsense-viewer 가 우리 노출을 물려받는다")

    # AE ROI 도 펌웨어에 남는 상태. 걸기 전 상자를 기억해 둠.
    roi_before = None
    if roi is not None and roi.supported:
        try:
            from ...utils.camera import ae_roi_of
            roi_before = ae_roi_of(profile)
        except Exception:
            roi_before = None

    # 드롭 회계는 호출자가 안 줘도 항상 돎 — 정수 뺄셈 하나 값.
    if stats is None:
        from ...utils.camera import FrameStats
        stats = FrameStats()
    reader = None
    if meta:
        from ...utils.camera import MetaReader
        reader = MetaReader()
    exp_unit = _EXPOSURE_UNIT_US.get(stream, 1.0)

    want_yuyv = (stream == "color" and color_format == "yuyv")

    # ── 정리(정지 + 원상복구). 어느 경로로 끝나든 **정확히 한 번** 돈다 ──────────
    _done = []

    def _cleanup():
        if _done:
            return                            # close() 와 제너레이터 finally 가 둘 다 부름
        _done.append(True)
        if restore:
            if roi_before is not None:
                try:
                    from ...utils.camera import aim_ae_at_bbox
                    aim_ae_at_bbox(profile, roi_before, (height, width), pad=0.0)
                except Exception:
                    pass                      # 되돌리기 실패로 종료를 막지는 않음
            if settings_before is not None:
                try:
                    settings_before.apply(profile)
                except Exception:
                    pass
        try:
            pipeline.stop()
        except Exception:
            pass

    def frames():
        t0 = None
        i = 0
        try:
            while True:
                # wait_for_frames 는 절대 밀리지 않음 — pipeline 의 출력 큐가
                fs = pipeline.wait_for_frames()
                t_arrival = time.time()          # 도착시각은 **기다림이 끝난 바로 그 자리**에서
                if align is not None:
                    fs = align.process(fs)
                f = fs.get_color_frame() if stream == "color" \
                    else fs.get_infrared_frame(ir_index)
                if not f:
                    continue
                ts = f.get_timestamp() / 1000.0
                if t0 is None:
                    t0 = ts
                buf = np.asanyarray(f.get_data())
                luma = None
                if want_yuyv:
                    # 파이썬 래퍼는 YUYV 를 (H, W) uint16 으로 줌 —
                    from ...utils.camera import yuyv_to_luma
                    luma = yuyv_to_luma(buf)
                    # 표시/그리기용 BGR 은 여기서 만듦. 소비자 눈에는 예전과 같은
                    img = cv2.cvtColor(
                        buf.view(np.uint8).reshape(buf.shape[0], buf.shape[1], 2),
                        cv2.COLOR_YUV2BGR_YUY2)
                else:
                    img = buf                         # color=BGR, infrared=흑백
                dm = None
                if want_depth:
                    df = fs.get_depth_frame()
                    dm = np.asanyarray(df.get_data()) if df else None
                # 드롭 회계를 먼저 돌려 "이 프레임 앞에서 몇 장 사라졌나"를 받아옴.
                fn = f.get_frame_number()
                missed = stats.update(fn, ts - t0)
                md = reader.read(f) if reader is not None else None
                stamps = None
                if timing is not None:
                    try:
                        stamps = timing.stamp(ts, t_arrival=t_arrival, meta=md,
                                              domain=str(f.get_frame_timestamp_domain()),
                                              frame_number=fn, dropped_before=missed)
                    except Exception:
                        stamps = None            # 기록 때문에 주행이 멈추면 안 된다
                img = _attach(img, dm, depth_scale, luma=luma, frame_number=fn,
                              meta=md, stamps=stamps,
                              dropped_before=missed, exposure_unit_us=exp_unit)
                # depth 를 켜든 말든 항상 3-튜플. 소비자(tools/live_pose.py)가
                yield i, ts - t0, img
                i += 1
        finally:
            _cleanup()

    # 반환 튜플 모양(gen, intr)과 `for i, t, img in gen:` / gen.close() 는 그대로 두고,
    return _FrameStream(frames(), ae_roi=roi, profile=profile, stats=stats,
                        tuning=tuning, settings_before=settings_before,
                        cleanup=_cleanup), intr


def open_bag(path, loop=False, realtime=False, stream="color",
             with_depth=False, ir_index=1, timeout_ms=2000, meta=True, tune=False):
    """녹화 파일(.db3)을 재생함. open_realsense 와 같은 (frames, intrinsics) 를 줌."""
    import pyrealsense2 as rs

    p = Path(path)
    if not p.exists():
        raise FileNotFoundError("bag 이 없다: %s" % p)
    if stream not in ("color", "infrared"):
        raise ValueError("stream 은 color 또는 infrared")

    pipeline = rs.pipeline()
    config = rs.config()
    # 스트림을 enable_stream 으로 지정하지 않음 — 녹화에 들어 있는 조합을
    config.enable_device_from_file(str(p), repeat_playback=bool(loop))

    profile = pipeline.start(config)
    playback = profile.get_device().as_playback()
    # 기본이 False 인 게 핵심. True 면 벽시계로 밀어붙여 프레임을 버림.
    playback.set_real_time(bool(realtime))

    try:
        intr = CameraIntrinsics.from_realsense(profile, stream, ir_index)
    except Exception:
        pipeline.stop()
        raise KeyError("bag 에 %s 스트림이 없다: %s" % (stream, p))

    depth_scale = 0.0
    if with_depth:
        try:
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        except Exception:
            depth_scale = 0.001                 # 녹화에 depth 센서 정보가 없을 때의 최후값
    align = rs.align(rs.stream.color) if (with_depth and stream == "color") else None

    from ...utils.camera import FrameStats
    stats = FrameStats()
    reader = None
    if meta:
        from ...utils.camera import MetaReader
        reader = MetaReader()
    exp_unit = _EXPOSURE_UNIT_US.get(stream, 1.0)

    # open_realsense 와 같은 이유로 껍데기가 정리를 들고 있음 —
    _done = []

    def _cleanup():
        if _done:
            return
        _done.append(True)
        try:
            pipeline.stop()
        except Exception:
            pass

    def frames():
        t0 = None
        i = 0
        try:
            while True:
                # 파일 끝은 예외가 아니라 상태로도 옴. 둘 다 봄.
                try:
                    ok, fs = pipeline.try_wait_for_frames(timeout_ms)
                except RuntimeError:
                    break                       # wait 계열이 던지는 EOF/타임아웃
                if not ok:
                    break                       # 더 줄 프레임이 없다 = 파일 끝
                if not loop and playback.current_status() == rs.playback_status.stopped:
                    break
                if align is not None:
                    fs = align.process(fs)
                f = fs.get_color_frame() if stream == "color" \
                    else fs.get_infrared_frame(ir_index)
                if not f:
                    continue
                ts = f.get_timestamp() / 1000.0
                if t0 is None:
                    t0 = ts
                img = np.asanyarray(f.get_data())
                dm = None
                if with_depth:
                    df = fs.get_depth_frame()
                    dm = np.asanyarray(df.get_data()) if df else None
                fn = f.get_frame_number()
                missed = stats.update(fn, ts - t0)
                img = _attach(img, dm, depth_scale, frame_number=fn,
                              meta=(reader.read(f) if reader is not None else None),
                              dropped_before=missed, exposure_unit_us=exp_unit)
                # open_realsense 와 같은 3-튜플. 소스를 바꿔 끼워도 소비자가 그대로.
                yield i, ts - t0, img
                i += 1
        finally:
            _cleanup()

    # open_realsense 와 같은 껍데기로 감쌈 — frames.stats 를 밖에서 볼 수 있게.
    settings_before = None
    if tune is not False and tune is not None:
        try:
            from ...utils.camera import CameraSettings
            settings_before = CameraSettings.from_sensor(profile)
        except Exception:
            settings_before = None              # 녹화에 컬러 센서 정보가 없을 수 있음

    return _FrameStream(frames(), profile=profile, stats=stats,
                        settings_before=settings_before, cleanup=_cleanup), intr


def from_video(path, loop=False):
    """영상 파일을 3-튜플로 흘림. 돌려주는 건 (frames, None) 임."""
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError("영상이 없다: %s" % p)
    cap = cv2.VideoCapture(str(p))
    if not cap.isOpened():
        raise RuntimeError("영상을 열 수 없다(코덱?): %s" % p)

    fps = cap.get(cv2.CAP_PROP_FPS) or 0.0
    _done = []

    def _cleanup():
        if _done:
            return
        _done.append(True)
        try:
            cap.release()
        except Exception:
            pass

    def frames():
        i = 0
        try:
            while True:
                ms = cap.get(cv2.CAP_PROP_POS_MSEC)     # read() 전이 이 프레임의 시각
                ok, bgr = cap.read()
                if not ok:
                    if not loop:
                        break
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ms = 0.0
                    ok, bgr = cap.read()
                    if not ok:
                        break                            # 되감아도 안 나오면 진짜 끝
                ts = (ms / 1000.0) if ms and ms > 0 else (i / fps if fps > 0 else float(i))
                yield i, ts, bgr
                i += 1
        finally:
            _cleanup()

    return _FrameStream(frames(), cleanup=_cleanup), None


# ----------------------------------------------------------------- 웹캠(UVC)
def list_cameras():
    """OpenCV 번호 순서대로 [(번호, 이름)]. 지금은 macOS 만 안다(다른 OS 는 []).

    OpenCV 의 AVFoundation 백엔드는 장치 목록을 uniqueID 문자열 순으로 정렬해
    번호를 매긴다(cap_avfoundation_mac.mm). system_profiler 가 주는 같은
    uniqueID 로 똑같이 정렬하면 번호가 맞는다. 이름 순서와는 다르다 —
    맥북에서 RealSense("0x2000…")가 FaceTime("EBB0…")보다 앞이라 0번이다.
    """
    if sys.platform != "darwin":
        return []
    try:
        out = subprocess.run(["system_profiler", "SPCameraDataType", "-json"],
                             capture_output=True, text=True, timeout=30).stdout
        items = json.loads(out).get("SPCameraDataType", [])
    except Exception:
        return []
    cams = sorted((str(it.get("spcamera_unique-id", "")), str(it.get("_name", "")))
                  for it in items)
    return [(i, name) for i, (_, name) in enumerate(cams)]


def camera_index(spec="realsense"):
    """'0' 같은 번호는 그대로, 'realsense' 같은 이름 조각은 목록에서 찾음.

    돌려주는 건 (번호, 이름). 번호로 줬는데 목록을 모르면 이름은 ''.
    """
    s = str(spec).strip()
    cams = list_cameras()
    if s.isdigit():
        i = int(s)
        return i, dict(cams).get(i, "")
    hits = [(i, n) for i, n in cams if s.lower() in n.lower()]
    if len(hits) == 1:
        return hits[0]
    if not cams:
        raise RuntimeError("카메라를 이름으로 고르는 건 macOS 에서만 된다. 번호를 줘라 (예: 0)")
    listing = ", ".join("%d=%s" % c for c in cams)
    if not hits:
        raise RuntimeError("'%s' 인 카메라가 없다. 있는 것: %s" % (spec, listing))
    raise RuntimeError("'%s' 가 여럿이다: %s. 번호로 골라라" % (spec, listing))


def open_webcam(index=0, width=None, height=None, fps=30):
    """UVC 웹캠을 OpenCV 로 엶. from_video 처럼 (frames, None) — 내부파라미터는 모름.

    macOS 에서 RealSense 컬러를 여는 유일한 길이다. librealsense 는 macOS 12
    이후 카메라를 못 잡지만(시스템 UVCAssistant 가 UVC 인터페이스를 선점, sudo 로
    뺏어도 2.56.5 는 IMU 초기화에서 죽는다 — librealsense #14302) 컬러 센서
    자체는 표준 UVC 라 macOS 가 일반 웹캠으로 띄워 준다. 2026-09-06 맥북 실측
    1920x1080/1280x720 @30fps.
    이 길로는 depth/IR/IMU/노출 메타데이터/bag 녹화가 없다. 내부파라미터를 안 주면
    파이프라인이 D435i 기준값에서 역산한다 — 같은 센서·같은 모드라 RealSense 로
    열었을 때와 같은 값이다(intrinsics_from_ref 참고).
    """
    backend = cv2.CAP_AVFOUNDATION if sys.platform == "darwin" else cv2.CAP_ANY
    cap = cv2.VideoCapture(int(index), backend)
    if not cap.isOpened():
        raise RuntimeError("웹캠 %s 를 열 수 없다 (카메라 권한? 다른 앱이 쓰는 중?)" % index)
    if width:
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, int(width))
    if height:
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, int(height))
    if fps:
        cap.set(cv2.CAP_PROP_FPS, int(fps))

    _done = []

    def _cleanup():
        if _done:
            return
        _done.append(True)
        try:
            cap.release()
        except Exception:
            pass

    def frames():
        i = 0
        t0 = None
        try:
            while True:
                ok, bgr = cap.read()
                if not ok:
                    break
                now = time.perf_counter()        # 도착 시각. 센서 시계는 못 본다
                if t0 is None:
                    t0 = now
                yield i, now - t0, bgr
                i += 1
        finally:
            _cleanup()

    # stats 를 안 단다 — 유실을 셀 프레임 번호가 없어서 0 으로 찍히면 거짓말이 된다.
    return _FrameStream(frames(), cleanup=_cleanup), None


def depth_at(depth, u, v, patch=5, scale=None):
    """(u, v) 픽셀의 깊이 [m]. 없으면 None."""
    dm = getattr(depth, "depth", None)
    if dm is not None:                       # Frame 이 들어온 경우
        if scale is None:
            scale = getattr(depth, "depth_scale", 0.0) or 0.0
    else:
        dm = depth
    if dm is None:
        return None
    dm = np.asanyarray(dm)
    if dm.ndim != 2 or dm.size == 0:
        return None
    if scale is None:
        scale = 1.0 if np.issubdtype(dm.dtype, np.floating) else 0.001
    if not scale:
        return None                          # depth_scale=0 == depth 없음

    h, w = dm.shape
    u, v = int(round(float(u))), int(round(float(v)))
    r = max(0, int(patch) // 2)
    x0, x1 = max(0, u - r), min(w, u + r + 1)
    y0, y1 = max(0, v - r), min(h, v + r + 1)
    if x0 >= x1 or y0 >= y1:
        return None                          # 화면 밖

    win = dm[y0:y1, x0:x1]
    nz = win[win > 0]                        # 0 = 측정 실패. 평균에 섞으면 거리가 당겨짐
    if nz.size == 0:
        return None
    return float(np.median(nz.astype(np.float64)) * scale)


#: to_gray(channel=...) 에서 쓰는 BGR 채널 번호.
_BGR_CHANNEL = {"blue": 0, "green": 1, "red": 2}


def to_gray(img, channel=None):
    """검출기에 넣을 흑백 이미지. 컬러면 변환하고 이미 흑백이면 그대로."""
    if channel is not None:
        c = _BGR_CHANNEL.get(channel)
        if c is None:
            raise ValueError("channel 은 red/green/blue: %r" % (channel,))
        if img.ndim != 3:
            return np.asarray(img)              # 이미 흑백이면 고를 채널이 없음
        # 뷰가 아니라 복사여야 함 — 검출기는 C-contiguous 버퍼를 요구함.
        return np.ascontiguousarray(np.asarray(img)[:, :, c])
    luma = getattr(img, "luma", None)
    if luma is not None and luma.shape[:2] == img.shape[:2]:
        return luma
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img
