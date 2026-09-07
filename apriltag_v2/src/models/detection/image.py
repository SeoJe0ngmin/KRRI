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
    fx: float
    fy: float
    cx: float
    cy: float
    width: int = 0
    height: int = 0
    distortion: tuple = ()

    @property
    def params(self):
        return (self.fx, self.fy, self.cx, self.cy)

    @property
    def K(self):
        return np.array([[self.fx, 0, self.cx], [0, self.fy, self.cy], [0, 0, 1]], dtype=np.float64)

    @classmethod
    def from_realsense(cls, profile, stream='color', index=1):
        import pyrealsense2 as rs
        kinds = {'color': rs.stream.color, 'depth': rs.stream.depth, 'infrared': rs.stream.infrared}
        if stream not in kinds:
            raise ValueError('모르는 스트림: %s' % stream)
        if stream == 'infrared':
            vsp = profile.get_stream(kinds[stream], index).as_video_stream_profile()
        else:
            vsp = profile.get_stream(kinds[stream]).as_video_stream_profile()
        i = vsp.get_intrinsics()
        return cls(fx=i.fx, fy=i.fy, cx=i.ppx, cy=i.ppy, width=i.width, height=i.height, distortion=tuple(i.coeffs))

    def undistort(self, img):
        if not self.distortion:
            return img
        d = np.array(self.distortion, dtype=np.float64)
        return cv2.undistort(img, self.K, d)

def intrinsics_from_ref(shape, ref=D.D435I_COLOR_REF):
    h, w = shape[:2]
    W0, H0, fx0, fy0, cx0, cy0 = ref
    s = float(h) / float(H0)
    return CameraIntrinsics(fx0 * s, fy0 * s, w / 2.0 + (cx0 - W0 / 2.0) * s, h / 2.0 + (cy0 - H0 / 2.0) * s, w, h)

def intrinsics_from_hfov(shape, hfov_deg=None):
    if hfov_deg is None:
        return intrinsics_from_ref(shape)
    h, w = shape[:2]
    fx = w / 2.0 / np.tan(np.deg2rad(float(hfov_deg)) / 2.0)
    return CameraIntrinsics(fx, fx, w / 2.0, h / 2.0, w, h)

def fov_edges_deg(intr):
    h = int(intr.height) or int(round(intr.cy * 2))
    w = int(intr.width) or int(round(intr.cx * 2))
    up = np.degrees(np.arctan(intr.cy / intr.fy))
    down = np.degrees(np.arctan((h - intr.cy) / intr.fy))
    left = np.degrees(np.arctan(intr.cx / intr.fx))
    right = np.degrees(np.arctan((w - intr.cx) / intr.fx))
    return (float(up), float(down), float(left), float(right))

def tag_visible_near_m(intr, tag_height_m, cam_height_m, tag_size_m):
    up, down, _, _ = fov_edges_deg(intr)
    dh = float(tag_height_m) - float(cam_height_m)
    half = float(tag_size_m) / 2.0
    edge = dh + half if dh >= 0 else -(dh - half)
    limit = up if dh >= 0 else down
    return float(edge / np.tan(np.radians(limit)))

class Frame(np.ndarray):
    depth = None
    depth_scale = 0.0
    luma = None
    frame_number = None
    dropped_before = 0
    meta = None
    exposure_us = None
    gain = None

    def __array_finalize__(self, obj):
        if obj is None:
            return
        self.depth = getattr(obj, 'depth', None)
        self.depth_scale = getattr(obj, 'depth_scale', 0.0)
        self.luma = getattr(obj, 'luma', None)
        self.frame_number = getattr(obj, 'frame_number', None)
        self.dropped_before = getattr(obj, 'dropped_before', 0)
        self.meta = getattr(obj, 'meta', None)
        self.exposure_us = getattr(obj, 'exposure_us', None)
        self.gain = getattr(obj, 'gain', None)
_EXPOSURE_UNIT_US = {'color': COLOR_EXPOSURE_UNIT_US, 'infrared': 1.0}

def _attach(img, depth=None, depth_scale=0.0, luma=None, frame_number=None, meta=None, dropped_before=0, exposure_unit_us=1.0):
    f = img.view(Frame)
    f.depth = depth
    f.depth_scale = float(depth_scale)
    f.luma = luma
    f.frame_number = frame_number
    f.dropped_before = int(dropped_before or 0)
    f.meta = meta if meta is not None else {}
    exp = f.meta.get('actual_exposure')
    f.exposure_us = None if exp is None else float(exp) * float(exposure_unit_us)
    g = f.meta.get('gain_level')
    f.gain = None if g is None else int(g)
    return f

class _FrameStream:

    def __init__(self, gen, ae_roi=None, profile=None, stats=None, tuning=None, settings_before=None, cleanup=None):
        self._gen = gen
        self.ae_roi = ae_roi
        self.profile = profile
        self.stats = stats
        self.tuning = tuning
        self.settings_before = settings_before
        self._cleanup = cleanup

    def __iter__(self):
        return self

    def __next__(self):
        return next(self._gen)

    def close(self):
        try:
            self._gen.close()
        finally:
            c, self._cleanup = (self._cleanup, None)
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
    import glob
    import subprocess
    devs = glob.glob('/dev/video*')
    if not devs:
        return ' (/dev/video* 가 없다 — UTM 이면 USB 로 RealSense 를 VM 에 넘겼나 확인, WSL 이면 usbipd 로 붙일 것)'
    try:
        out = subprocess.run(['fuser'] + devs, capture_output=True, text=True, timeout=3)
        pids = sorted(set(out.stdout.split()))
    except Exception:
        return ''
    if not pids:
        return ''
    try:
        ps = subprocess.run(['ps', '-o', 'pid=,cmd=', '-p', ','.join(pids)], capture_output=True, text=True, timeout=3).stdout.strip()
    except Exception:
        ps = ' '.join(pids)
    return '\n  지금 잡고 있는 프로세스:\n' + '\n'.join(('    ' + l.strip() for l in ps.splitlines()))

def open_realsense(stream='color', width=None, height=None, fps=30, depth=False, emitter=None, ir_index=1, with_depth=False, depth_size=(1280, 720), depth_fps=None, color_format='bgr8', ae_roi=False, stats=None, meta=True, exposure_us=None, ae_priority=None, tune=False, restore=True, record=None):
    import pyrealsense2 as rs
    want_depth = bool(depth or with_depth)
    if width is None or height is None:
        width, height = D.COLOR_SIZE if stream == 'color' else D.IR_SIZE
    pipeline = rs.pipeline()
    config = rs.config()
    if stream == 'color':
        if color_format not in ('bgr8', 'yuyv'):
            raise ValueError('color_format 은 bgr8 또는 yuyv')
        cfmt = rs.format.bgr8 if color_format == 'bgr8' else rs.format.yuyv
        config.enable_stream(rs.stream.color, width, height, cfmt, fps)
    elif stream == 'infrared':
        config.enable_stream(rs.stream.infrared, ir_index, width, height, rs.format.y8, fps)
    else:
        raise ValueError('stream 은 color 또는 infrared')
    if want_depth:
        dw, dh = depth_size
        config.enable_stream(rs.stream.depth, dw, dh, rs.format.z16, int(depth_fps or min(fps, 30)))
    if record:
        rec = Path(record)
        rec.parent.mkdir(parents=True, exist_ok=True)
        config.enable_record_to_file(str(rec))
    try:
        profile = pipeline.start(config)
    except RuntimeError as exc:
        if 'busy' in str(exc).lower():
            raise RuntimeError('%s — 카메라는 한 프로세스만 연다.%s\n  같은 프로세스 안이면: 루프에서 break 했을 때 frames.close() 를 부르거나 `with open_realsense()[0] as frames:` 로 열어라.' % (exc, _who_has_camera())) from exc
        raise
    want_emitter = stream != 'infrared' if emitter is None else bool(emitter)
    depth_scale = 0.0
    try:
        ds = profile.get_device().first_depth_sensor()
        if ds.supports(rs.option.emitter_enabled):
            ds.set_option(rs.option.emitter_enabled, 1 if want_emitter else 0)
        if want_depth:
            depth_scale = ds.get_depth_scale()
    except Exception:
        pass
    intr = CameraIntrinsics.from_realsense(profile, stream, ir_index)
    align = rs.align(rs.stream.color) if want_depth and stream == 'color' else None
    roi = None
    if ae_roi:
        from ...utils.camera import ExposureROI
        roi = ExposureROI(profile, stream=stream)
    want_tune = tune is not False and tune is not None and (stream == 'color')
    want_exposure = (exposure_us is not None or ae_priority is not None) and stream == 'color'
    tuning = None
    settings_before = None
    if want_tune or want_exposure:
        try:
            from ...utils.camera import CameraSettings as _CS
            settings_before = _CS.from_sensor(profile)
        except Exception:
            settings_before = None
    if want_tune:
        from ...utils.camera import CameraSettings, tune_for_tags
        if isinstance(tune, CameraSettings):
            _b, tuning = tune_for_tags(profile, settings=tune)
        elif tune is True:
            _b, tuning = tune_for_tags(profile)
        else:
            _b, tuning = tune_for_tags(profile, speed_mps=float(tune), fx=intr.fx)
        settings_before = settings_before or _b
    if want_exposure:
        from ...utils.camera import set_color_exposure
        applied = set_color_exposure(profile, exposure_us=exposure_us, ae_priority=ae_priority)
        if applied['errors']:
            import warnings
            warnings.warn('컬러 노출 설정 실패: %s' % ', '.join(applied['errors']))
    if restore and (want_tune or want_exposure) and (settings_before is None):
        import warnings
        warnings.warn('카메라 상태를 뜨지 못해 원상복구를 못 한다 — 끝난 뒤 realsense-viewer 가 우리 노출을 물려받는다')
    roi_before = None
    if roi is not None and roi.supported:
        try:
            from ...utils.camera import ae_roi_of
            roi_before = ae_roi_of(profile)
        except Exception:
            roi_before = None
    if stats is None:
        from ...utils.camera import FrameStats
        stats = FrameStats()
    reader = None
    if meta:
        from ...utils.camera import MetaReader
        reader = MetaReader()
    exp_unit = _EXPOSURE_UNIT_US.get(stream, 1.0)
    want_yuyv = stream == 'color' and color_format == 'yuyv'
    _done = []

    def _cleanup():
        if _done:
            return
        _done.append(True)
        if restore:
            if roi_before is not None:
                try:
                    from ...utils.camera import aim_ae_at_bbox
                    aim_ae_at_bbox(profile, roi_before, (height, width), pad=0.0)
                except Exception:
                    pass
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
                fs = pipeline.wait_for_frames()
                if align is not None:
                    fs = align.process(fs)
                f = fs.get_color_frame() if stream == 'color' else fs.get_infrared_frame(ir_index)
                if not f:
                    continue
                ts = f.get_timestamp() / 1000.0
                if t0 is None:
                    t0 = ts
                buf = np.asanyarray(f.get_data())
                luma = None
                if want_yuyv:
                    from ...utils.camera import yuyv_to_luma
                    luma = yuyv_to_luma(buf)
                    img = cv2.cvtColor(buf.view(np.uint8).reshape(buf.shape[0], buf.shape[1], 2), cv2.COLOR_YUV2BGR_YUY2)
                else:
                    img = buf
                dm = None
                if want_depth:
                    df = fs.get_depth_frame()
                    dm = np.asanyarray(df.get_data()) if df else None
                fn = f.get_frame_number()
                missed = stats.update(fn, ts - t0)
                img = _attach(img, dm, depth_scale, luma=luma, frame_number=fn, meta=reader.read(f) if reader is not None else None, dropped_before=missed, exposure_unit_us=exp_unit)
                yield (i, ts - t0, img)
                i += 1
        finally:
            _cleanup()
    return (_FrameStream(frames(), ae_roi=roi, profile=profile, stats=stats, tuning=tuning, settings_before=settings_before, cleanup=_cleanup), intr)

def open_bag(path, loop=False, realtime=False, stream='color', with_depth=False, ir_index=1, timeout_ms=2000, meta=True, tune=False):
    import pyrealsense2 as rs
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError('bag 이 없다: %s' % p)
    if stream not in ('color', 'infrared'):
        raise ValueError('stream 은 color 또는 infrared')
    pipeline = rs.pipeline()
    config = rs.config()
    config.enable_device_from_file(str(p), repeat_playback=bool(loop))
    profile = pipeline.start(config)
    playback = profile.get_device().as_playback()
    playback.set_real_time(bool(realtime))
    try:
        intr = CameraIntrinsics.from_realsense(profile, stream, ir_index)
    except Exception:
        pipeline.stop()
        raise KeyError('bag 에 %s 스트림이 없다: %s' % (stream, p))
    depth_scale = 0.0
    if with_depth:
        try:
            depth_scale = profile.get_device().first_depth_sensor().get_depth_scale()
        except Exception:
            depth_scale = 0.001
    align = rs.align(rs.stream.color) if with_depth and stream == 'color' else None
    from ...utils.camera import FrameStats
    stats = FrameStats()
    reader = None
    if meta:
        from ...utils.camera import MetaReader
        reader = MetaReader()
    exp_unit = _EXPOSURE_UNIT_US.get(stream, 1.0)
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
                try:
                    ok, fs = pipeline.try_wait_for_frames(timeout_ms)
                except RuntimeError:
                    break
                if not ok:
                    break
                if not loop and playback.current_status() == rs.playback_status.stopped:
                    break
                if align is not None:
                    fs = align.process(fs)
                f = fs.get_color_frame() if stream == 'color' else fs.get_infrared_frame(ir_index)
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
                img = _attach(img, dm, depth_scale, frame_number=fn, meta=reader.read(f) if reader is not None else None, dropped_before=missed, exposure_unit_us=exp_unit)
                yield (i, ts - t0, img)
                i += 1
        finally:
            _cleanup()
    settings_before = None
    if tune is not False and tune is not None:
        try:
            from ...utils.camera import CameraSettings
            settings_before = CameraSettings.from_sensor(profile)
        except Exception:
            settings_before = None
    return (_FrameStream(frames(), profile=profile, stats=stats, settings_before=settings_before, cleanup=_cleanup), intr)

def from_video(path, loop=False):
    p = Path(path)
    if not p.exists():
        raise FileNotFoundError('영상이 없다: %s' % p)
    cap = cv2.VideoCapture(str(p))
    if not cap.isOpened():
        raise RuntimeError('영상을 열 수 없다(코덱?): %s' % p)
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
                ms = cap.get(cv2.CAP_PROP_POS_MSEC)
                ok, bgr = cap.read()
                if not ok:
                    if not loop:
                        break
                    cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                    ms = 0.0
                    ok, bgr = cap.read()
                    if not ok:
                        break
                ts = ms / 1000.0 if ms and ms > 0 else i / fps if fps > 0 else float(i)
                yield (i, ts, bgr)
                i += 1
        finally:
            _cleanup()
    return (_FrameStream(frames(), cleanup=_cleanup), None)

def list_cameras():
    if sys.platform != 'darwin':
        return []
    try:
        out = subprocess.run(['system_profiler', 'SPCameraDataType', '-json'], capture_output=True, text=True, timeout=30).stdout
        items = json.loads(out).get('SPCameraDataType', [])
    except Exception:
        return []
    cams = sorted(((str(it.get('spcamera_unique-id', '')), str(it.get('_name', ''))) for it in items))
    return [(i, name) for i, (_, name) in enumerate(cams)]

def camera_index(spec='realsense'):
    s = str(spec).strip()
    cams = list_cameras()
    if s.isdigit():
        i = int(s)
        return (i, dict(cams).get(i, ''))
    hits = [(i, n) for i, n in cams if s.lower() in n.lower()]
    if len(hits) == 1:
        return hits[0]
    if not cams:
        raise RuntimeError('카메라를 이름으로 고르는 건 macOS 에서만 된다. 번호를 줘라 (예: 0)')
    listing = ', '.join(('%d=%s' % c for c in cams))
    if not hits:
        raise RuntimeError("'%s' 인 카메라가 없다. 있는 것: %s" % (spec, listing))
    raise RuntimeError("'%s' 가 여럿이다: %s. 번호로 골라라" % (spec, listing))

def open_webcam(index=0, width=None, height=None, fps=30):
    backend = cv2.CAP_AVFOUNDATION if sys.platform == 'darwin' else cv2.CAP_ANY
    cap = cv2.VideoCapture(int(index), backend)
    if not cap.isOpened():
        raise RuntimeError('웹캠 %s 를 열 수 없다 (카메라 권한? 다른 앱이 쓰는 중?)' % index)
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
                now = time.perf_counter()
                if t0 is None:
                    t0 = now
                yield (i, now - t0, bgr)
                i += 1
        finally:
            _cleanup()
    return (_FrameStream(frames(), cleanup=_cleanup), None)

def depth_at(depth, u, v, patch=5, scale=None):
    dm = getattr(depth, 'depth', None)
    if dm is not None:
        if scale is None:
            scale = getattr(depth, 'depth_scale', 0.0) or 0.0
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
        return None
    h, w = dm.shape
    u, v = (int(round(float(u))), int(round(float(v))))
    r = max(0, int(patch) // 2)
    x0, x1 = (max(0, u - r), min(w, u + r + 1))
    y0, y1 = (max(0, v - r), min(h, v + r + 1))
    if x0 >= x1 or y0 >= y1:
        return None
    win = dm[y0:y1, x0:x1]
    nz = win[win > 0]
    if nz.size == 0:
        return None
    return float(np.median(nz.astype(np.float64)) * scale)
_BGR_CHANNEL = {'blue': 0, 'green': 1, 'red': 2}

def to_gray(img, channel=None):
    if channel is not None:
        c = _BGR_CHANNEL.get(channel)
        if c is None:
            raise ValueError('channel 은 red/green/blue: %r' % (channel,))
        if img.ndim != 3:
            return np.asarray(img)
        return np.ascontiguousarray(np.asarray(img)[:, :, c])
    luma = getattr(img, 'luma', None)
    if luma is not None and luma.shape[:2] == img.shape[:2]:
        return luma
    if img.ndim == 3:
        return cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
    return img
