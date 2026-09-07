from dataclasses import dataclass, asdict, replace, fields
import numpy as np
from config.detection import BLUR_CLEAN_PX, BLUR_DEAD_PX, COLOR_EXPOSURE_UNIT_US

def yuyv_to_luma(buf):
    a = np.asanyarray(buf)
    if a.ndim != 2:
        raise ValueError('YUYV 버퍼는 2-D 여야 한다: %r' % (a.shape,))
    if a.dtype != np.uint8:
        a = a.view(np.uint8)
    return np.ascontiguousarray(a[:, 0::2])

class ExposureROI:
    MIN_SIDE_PX = 32

    def __init__(self, profile, pad=0.35, stream='color'):
        self.pad = float(pad)
        self.sensor = None
        self.last = None
        self.supported = False
        try:
            import pyrealsense2 as rs
            dev = profile.get_device()
            for s in dev.query_sensors():
                name = s.get_info(rs.camera_info.name).lower()
                want_rgb = stream == 'color'
                if want_rgb != name.startswith('rgb'):
                    continue
                if s.is_roi_sensor():
                    self.sensor = s.as_roi_sensor()
                    self.supported = True
                break
        except Exception:
            pass

    def follow(self, detections, shape):
        if not self.supported or not detections:
            return False
        h, w = (shape[0], shape[1])
        pts = np.concatenate([np.asarray(d.corners, dtype=np.float64) for d in detections], axis=0)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        px, py = ((x1 - x0) * self.pad, (y1 - y0) * self.pad)
        return self.set_box(x0 - px, y0 - py, x1 + px, y1 + py, w, h)

    def set_box(self, x0, y0, x1, y1, w, h):
        if not self.supported:
            return False
        import pyrealsense2 as rs
        cx, cy = ((x0 + x1) / 2.0, (y0 + y1) / 2.0)
        half = max(self.MIN_SIDE_PX / 2.0, (x1 - x0) / 2.0)
        x0, x1 = (cx - half, cx + half)
        half = max(self.MIN_SIDE_PX / 2.0, (y1 - y0) / 2.0)
        y0, y1 = (cy - half, cy + half)
        box = (max(0, int(x0)), max(0, int(y0)), min(w - 1, int(round(x1))), min(h - 1, int(round(y1))))
        if box[2] - box[0] < 4 or box[3] - box[1] < 4:
            return False
        if box == self.last:
            return False
        roi = rs.region_of_interest()
        roi.min_x, roi.min_y, roi.max_x, roi.max_y = box
        try:
            self.sensor.set_region_of_interest(roi)
        except Exception:
            return False
        self.last = box
        return True

    def reset(self, w, h):
        return self.set_box(0, 0, w - 1, h - 1, w, h)

class FrameStats:

    def __init__(self):
        self.received = 0
        self.dropped = 0
        self.gaps = 0
        self.first_number = None
        self.last_number = None
        self._t_first = None
        self._t_last = None

    def update(self, frame_or_number, t=None):
        n = getattr(frame_or_number, 'frame_number', None)
        if n is None and isinstance(frame_or_number, (int, np.integer)):
            n = int(frame_or_number)
        self.received += 1
        if t is not None:
            if self._t_first is None:
                self._t_first = float(t)
            self._t_last = float(t)
        if n is None:
            return 0
        n = int(n)
        miss = 0
        if self.last_number is not None:
            miss = n - self.last_number - 1
            if miss > 0:
                self.dropped += miss
                self.gaps += 1
            else:
                miss = 0
        if self.first_number is None:
            self.first_number = n
        self.last_number = n
        return miss

    @property
    def fps(self):
        if self._t_first is None or self._t_last is None:
            return 0.0
        dt = self._t_last - self._t_first
        return (self.received - 1) / dt if dt > 0 else 0.0

    @property
    def sent(self):
        return self.received + self.dropped

    @property
    def drop_rate(self):
        return self.dropped / self.sent if self.sent else 0.0

    def summary(self):
        s = '받음 %d / 보냄 %d, 버림 %d (%.1f%%), 구멍 %d회' % (self.received, self.sent, self.dropped, self.drop_rate * 100.0, self.gaps)
        if self.fps:
            s += ', 소비 %.1f fps' % self.fps
        if self.last_number is None:
            s += '  [frame_number 없음 — 개수만 셈]'
        return s
_META_KEYS = ('actual_exposure', 'gain_level', 'frame_counter', 'sensor_timestamp', 'time_of_arrival', 'backend_timestamp', 'actual_fps', 'auto_exposure', 'white_balance', 'frame_laser_power_mode')

def frame_meta(f, keys=_META_KEYS):
    try:
        import pyrealsense2 as rs
    except Exception:
        return {}
    out = {}
    for k in keys:
        mv = getattr(rs.frame_metadata_value, k, None)
        if mv is None:
            continue
        try:
            if f.supports_frame_metadata(mv):
                out[k] = int(f.get_frame_metadata(mv))
        except Exception:
            pass
    return out

def describe_metadata(f):
    try:
        import pyrealsense2 as rs
    except Exception:
        return 'pyrealsense2 를 못 불러왔다'
    names = [a for a in dir(rs.frame_metadata_value) if not a.startswith('_') and a not in ('name', 'value')]
    got = []
    for n in names:
        mv = getattr(rs.frame_metadata_value, n)
        try:
            if f.supports_frame_metadata(mv):
                got.append('  %-28s %s' % (n, f.get_frame_metadata(mv)))
        except Exception:
            pass
    head = 'timestamp=%.3f ms  domain=%s  frame_number=%d' % (f.get_timestamp(), f.get_frame_timestamp_domain(), f.get_frame_number())
    if not got:
        return head + '\n  (메타데이터 없음 — 리눅스면 커널 패치 미적용)'
    return head + '\n' + '\n'.join(got)

def timestamp_domain(f):
    return str(f.get_frame_timestamp_domain())

def set_global_time(profile, enabled=True):
    out = {}
    try:
        import pyrealsense2 as rs
    except Exception:
        return out
    for s in profile.get_device().query_sensors():
        name = s.get_info(rs.camera_info.name)
        try:
            if s.supports(rs.option.global_time_enabled):
                s.set_option(rs.option.global_time_enabled, 1.0 if enabled else 0.0)
                out[name] = True
            else:
                out[name] = False
        except Exception:
            out[name] = False
    return out

class MetaReader:

    def __init__(self, keys=_META_KEYS):
        self._keys = tuple(keys)
        self._probed = None
        self.supported = ()

    def probe(self, f):
        try:
            import pyrealsense2 as rs
        except Exception:
            self._probed, self.supported = ((), ())
            return self.supported
        live = []
        for k in self._keys:
            mv = getattr(rs.frame_metadata_value, k, None)
            if mv is None:
                continue
            try:
                if f.supports_frame_metadata(mv):
                    live.append((k, mv))
            except Exception:
                pass
        self._probed = tuple(live)
        self.supported = tuple((k for k, _ in live))
        return self.supported

    def read(self, f):
        if self._probed is None:
            self.probe(f)
        if not self._probed:
            return {}
        out = {}
        for k, mv in self._probed:
            try:
                out[k] = int(f.get_frame_metadata(mv))
            except Exception:
                pass
        return out
BLUR_PX_100PCT = BLUR_CLEAN_PX
BLUR_PX_ZERO = BLUR_DEAD_PX

def motion_blur_px(exposure_us, speed_mps, fx, z_m):
    if not z_m or z_m <= 0:
        return float('inf')
    return float(fx) * float(speed_mps) * (float(exposure_us) * 1e-06) / float(z_m)

def exposure_budget_us(speed_mps, fx, z_m, blur_px=BLUR_CLEAN_PX):
    v = abs(float(speed_mps))
    if v <= 0:
        return float('inf')
    return float(blur_px) * float(z_m) / (float(fx) * v) * 1000000.0

def diagnose_frame(img, fx=None, z_m=None, speed_mps=None):
    parts = []
    n = getattr(img, 'dropped_before', 0) or 0
    if n:
        parts.append('직전 %d장 유실(파이프라인 큐가 버림)' % n)
    exp = getattr(img, 'exposure_us', None)
    gain = getattr(img, 'gain', None)
    if exp is None and gain is None:
        if getattr(img, 'meta', None) is None:
            why = 'RealSense 프레임이 아니다(영상파일/웹캠) — 노출 정보가 원래 없다'
        else:
            why = '프레임 메타데이터가 안 온다(리눅스면 커널 패치 미적용)'
        if not parts:
            return '판단 재료 없음 — ' + why
        parts.append('노출/게인 불명 — ' + why)
        return ', '.join(parts)
    if exp is not None:
        s = '노출 %.1fms' % (exp / 1000.0)
        if fx and z_m and speed_mps:
            b = motion_blur_px(exp, speed_mps, fx, z_m)
            if b >= BLUR_DEAD_PX:
                s += ' -> 블러 %.1fpx: 실측 0%% 구간(%.0fpx 에서 검출이 끊긴다)' % (b, BLUR_DEAD_PX)
            elif b > BLUR_CLEAN_PX:
                s += ' -> 블러 %.1fpx: 실측 열화 구간(%.0fpx 넘음)' % (b, BLUR_CLEAN_PX)
            else:
                s += ' -> 블러 %.1fpx: 블러는 무죄' % b
            s += ' (노출을 %.1fms 로 끊으면 100%%)' % (exposure_budget_us(speed_mps, fx, z_m) / 1000.0)
        parts.append(s)
    if gain is not None:
        parts.append('게인 %d' % gain)
    return ', '.join(parts)

def set_color_exposure(profile, exposure_us=None, ae_priority=None, gain=None):
    out = {'exposure_us': None, 'ae_priority': None, 'gain': None, 'auto_exposure': None, 'errors': []}
    try:
        import pyrealsense2 as rs
    except Exception:
        out['errors'].append('pyrealsense2')
        return out
    sensor = None
    for s in profile.get_device().query_sensors():
        try:
            if s.get_info(rs.camera_info.name).lower().startswith('rgb'):
                sensor = s
                break
        except Exception:
            pass
    if sensor is None:
        out['errors'].append('rgb sensor 없음')
        return out

    def _set(opt, value, name):
        try:
            if not sensor.supports(opt):
                out['errors'].append(name + '(미지원)')
                return None
            rng = sensor.get_option_range(opt)
            v = min(max(float(value), rng.min), rng.max)
            sensor.set_option(opt, v)
            return sensor.get_option(opt)
        except Exception as exc:
            out['errors'].append('%s(%s)' % (name, exc.__class__.__name__))
            return None
    if ae_priority is not None:
        v = _set(rs.option.auto_exposure_priority, float(ae_priority), 'ae_priority')
        out['ae_priority'] = None if v is None else int(v)
    if exposure_us is not None:
        v = _set(rs.option.exposure, float(exposure_us) / COLOR_EXPOSURE_UNIT_US, 'exposure')
        out['exposure_us'] = None if v is None else float(v) * COLOR_EXPOSURE_UNIT_US
    if gain is not None:
        v = _set(rs.option.gain, float(gain), 'gain')
        out['gain'] = None if v is None else float(v)
    try:
        if sensor.supports(rs.option.enable_auto_exposure):
            out['auto_exposure'] = bool(sensor.get_option(rs.option.enable_auto_exposure))
    except Exception:
        pass
    return out
MAINS_HALF_CYCLE_MS = 1000.0 / 120.0
AE_ROI_MARGIN_FRACTION = 1.0 / 8.0

def exposure_ms_for_motion(speed_mps, range_m, fx, blur_px=BLUR_PX_100PCT):
    speed_mps = abs(float(speed_mps))
    if speed_mps <= 0.0:
        return float('inf')
    return 1000.0 * float(blur_px) * float(range_m) / (float(fx) * speed_mps)

def exposure_units(ms, unit_us=COLOR_EXPOSURE_UNIT_US):
    return max(1, int(round(float(ms) * 1000.0 / float(unit_us))))

def exposure_ms(units, unit_us=COLOR_EXPOSURE_UNIT_US):
    return float(units) * float(unit_us) / 1000.0
DOCKING_EXPOSURE_UNITS = exposure_units(MAINS_HALF_CYCLE_MS)

@dataclass
class CameraSettings:
    enable_auto_exposure: bool = None
    exposure: float = None
    gain: float = None
    power_line_frequency: int = None
    auto_exposure_priority: int = None
    enable_auto_white_balance: bool = None
    frames_queue_size: int = None
    global_time_enabled: bool = None

    @classmethod
    def docking(cls, speed_mps=None, range_m=1.0, fx=None):
        units = DOCKING_EXPOSURE_UNITS
        if speed_mps:
            if not fx:
                raise ValueError('speed_mps 를 주면 fx 도 줘야 한다 (intr.fx)')
            ms = exposure_ms_for_motion(speed_mps, range_m, fx)
            n = int(ms / MAINS_HALF_CYCLE_MS)
            units = exposure_units(n * MAINS_HALF_CYCLE_MS) if n >= 1 else exposure_units(ms)
        return cls(enable_auto_exposure=False, exposure=float(units), gain=64.0, power_line_frequency=2, auto_exposure_priority=0, enable_auto_white_balance=False, frames_queue_size=1, global_time_enabled=True)

    @classmethod
    def from_sensor(cls, sensor):
        import pyrealsense2 as rs
        s = color_sensor(sensor)
        out = {}
        for f in fields(cls):
            opt = getattr(rs.option, f.name, None)
            if opt is None:
                continue
            try:
                if not s.supports(opt):
                    continue
                v = s.get_option(opt)
            except Exception:
                continue
            if f.type is bool or f.name.startswith(('enable_', 'global_time')):
                out[f.name] = bool(round(v))
            elif f.type is int:
                out[f.name] = int(round(v))
            else:
                out[f.name] = float(v)
        return cls(**out)

    def apply(self, sensor, strict=False):
        import pyrealsense2 as rs
        if self.enable_auto_exposure is True and strict:
            self.validate()
        s = color_sensor(sensor)
        report = {}
        common = ('power_line_frequency', 'auto_exposure_priority', 'enable_auto_white_balance', 'frames_queue_size', 'global_time_enabled')
        if self.enable_auto_exposure is True:
            order = common + ('exposure', 'gain', 'enable_auto_exposure')
        else:
            order = common + ('enable_auto_exposure', 'exposure', 'gain')
        for name in order:
            want = getattr(self, name)
            if want is None:
                continue
            opt = getattr(rs.option, name, None)
            if opt is None:
                if strict:
                    raise AttributeError('pyrealsense2 에 rs.option.%s 가 없다' % name)
                continue
            try:
                if not s.supports(opt):
                    if strict:
                        raise RuntimeError('센서가 %s 를 지원하지 않는다' % name)
                    report[name] = (None, None)
                    continue
                before = s.get_option(opt)
            except Exception:
                report[name] = (None, None)
                continue
            val = float(want)
            try:
                r = s.get_option_range(opt)
                val = min(max(val, r.min), r.max)
            except Exception:
                pass
            try:
                s.set_option(opt, val)
                after = s.get_option(opt)
            except Exception:
                if strict:
                    raise
                after = None
            report[name] = (before, after)
        return report

    def validate(self):
        if self.enable_auto_exposure is True:
            bad = [n for n in ('exposure', 'gain') if getattr(self, n) is not None]
            if bad:
                raise ValueError('enable_auto_exposure=True 와 %s 를 같이 줄 수 없다 — 쓰는 순간 SDK 가 AE 를 꺼버린다(ds-color-common.cpp:90-93 + option.cpp:84-105). 노출을 고정하려면 enable_auto_exposure=False 를, 자동노출을 쓰려면 %s 를 None 으로 둘 것.' % (' / '.join(bad), ' / '.join(bad)))
        return self

    @property
    def exposure_ms(self):
        return None if self.exposure is None else exposure_ms(self.exposure)

    def describe(self):
        lines = []
        for f in fields(self):
            v = getattr(self, f.name)
            if v is None:
                lines.append('  %-26s -            (건드리지 않음)' % f.name)
            elif f.name == 'exposure':
                lines.append('  %-26s %-12g (%.2f ms)' % (f.name, v, exposure_ms(v)))
            else:
                lines.append('  %-26s %s' % (f.name, v))
        return '\n'.join(lines)
DOCKING_SETTINGS = CameraSettings.docking()

def color_sensor(obj):
    import pyrealsense2 as rs
    if isinstance(obj, rs.sensor) or hasattr(obj, 'get_option_range'):
        return obj
    dev = obj.get_device() if hasattr(obj, 'get_device') else obj
    for s in dev.query_sensors():
        try:
            if s.get_info(rs.camera_info.name).lower().startswith('rgb'):
                return s
        except Exception:
            pass
    return dev.first_color_sensor()

def tune_for_tags(target, settings=None, speed_mps=None, range_m=1.0, fx=None, verbose=False):
    s = color_sensor(target)
    want = settings or CameraSettings.docking(speed_mps=speed_mps, range_m=range_m, fx=fx)
    before = CameraSettings.from_sensor(s)
    report = want.apply(s)
    if verbose:
        print('[tune_for_tags] 컬러 센서 설정')
        for k, (b, a) in report.items():
            if a is None:
                print('  %-26s %-12s -> (못 씀 / 미지원)' % (k, b))
            else:
                extra = '  (%.2f ms)' % exposure_ms(a) if k == 'exposure' else ''
                mark = '' if b == a else '  *'
                print('  %-26s %-12g -> %-12g%s%s' % (k, b if b is not None else float('nan'), a, extra, mark))
    return (before, report)

def ae_limit_supported(target):
    import pyrealsense2 as rs
    dev = target.get_device() if hasattr(target, 'get_device') else target
    out = {}
    opt = getattr(rs.option, 'auto_exposure_limit', None)
    for key, getter in (('color', 'first_color_sensor'), ('depth', 'first_depth_sensor')):
        try:
            s = color_sensor(dev) if key == 'color' else getattr(dev, getter)()
            out[key] = bool(opt is not None and s.supports(opt))
        except Exception:
            out[key] = False
    return out

def describe_color_options(target):
    import pyrealsense2 as rs
    s = color_sensor(target)
    lines = []
    for name in sorted((a for a in dir(rs.option) if not a.startswith('_') and a not in ('name', 'value'))):
        opt = getattr(rs.option, name)
        try:
            if not s.supports(opt):
                continue
            v = s.get_option(opt)
        except Exception:
            continue
        try:
            r = s.get_option_range(opt)
            rng = '[%g .. %g] step %g def %g' % (r.min, r.max, r.step, r.default)
        except Exception:
            rng = '(범위 없음)'
        lines.append('  %-28s %-12g %s' % (name, v, rng))
    return '\n'.join(lines) if lines else '  (읽을 수 있는 옵션이 없다)'

def ae_roi_supported(target):
    try:
        return bool(color_sensor(target).is_roi_sensor())
    except Exception:
        return False

def aim_ae_at_bbox(target, bbox, shape, pad=0.35):
    import pyrealsense2 as rs
    try:
        s = color_sensor(target)
        if not s.is_roi_sensor():
            return False
        r = s.as_roi_sensor()
    except Exception:
        return False
    h, w = (int(shape[0]), int(shape[1]))
    x0, y0, x1, y1 = (float(v) for v in bbox)
    px, py = ((x1 - x0) * pad, (y1 - y0) * pad)
    x0, y0, x1, y1 = (x0 - px, y0 - py, x1 + px, y1 + py)
    box = (max(0, int(x0)), max(0, int(y0)), min(w - 1, int(round(x1))), min(h - 1, int(round(y1))))
    if box[2] - box[0] < 8 or box[3] - box[1] < 8:
        return False
    roi = rs.region_of_interest()
    roi.min_x, roi.min_y, roi.max_x, roi.max_y = box
    try:
        r.set_region_of_interest(roi)
    except Exception:
        return False
    return True

def center_ae_roi(target, width, height):
    mx = int(width * AE_ROI_MARGIN_FRACTION)
    my = int(height * AE_ROI_MARGIN_FRACTION)
    return aim_ae_at_bbox(target, (mx, my, width - 1 - mx, height - 1 - my), (height, width), pad=0.0)

def ae_roi_of(target):
    try:
        r = color_sensor(target).as_roi_sensor().get_region_of_interest()
        return (r.min_x, r.min_y, r.max_x, r.max_y)
    except Exception:
        return None
_WHY_NOT = {'sharpness': "위험해서 뺐다. 언샤프 마스크는 흑백 경계에 오버슈트 링잉을 얹는데, AprilTag 는 바로 그 경계에 직선을 맞춰 모서리를 잡는다. 링잉은 밝은 쪽에서 선을 바깥으로, 어두운 쪽에서 안으로 민다 — 거리에 따라 달라지는 계통오차이고 그대로 estimate_pose 와 MAX_REPROJ_RMS_PX 로 들어간다. 흐린 영상에서 검출 '개수'는 늘려 놓고 자세 '정확도'는 망칠 수 있어 도킹에서 최악의 실패 방식이다.", 'contrast': '무의미해서 뺐다. 단조 톤커브인데 AprilTag 의 임계화는 국소 적응형이라 타일마다 min/max 를 재고 중간에서 자른다. 단조 변환은 그 판정을 안 바꾼다. 할 수 있는 일이라곤 흰칸을 클리핑시키거나 검은칸을 0 으로 눌러 정보를 없애는 것뿐이다.', 'gamma': 'contrast 와 같은 이유. 흑백 두 무리가 0~255 중 어디 앉는지를 바꿀 뿐 얼마나 잘 갈라지는지를 못 바꾼다. 흑백 타깃이니 감마가 도움이 될 것 같다는 직관이 강해서 굳이 적어 둔다.', 'brightness': "노출 뒤에 더해지는 DC 오프셋이라 검은칸과 흰칸을 똑같이 민다. 임계화가 보는 것은 둘의 '차이'라 한쪽이 클리핑될 때까지 아무 변화가 없다. 오프셋 말고 신호를 바꾸는 노출이 언제나 낫다.", 'saturation': '색차 전용. 실측으로 상쇄가 확인됐다 — U,V 를 키워도 BGR2GRAY 결과는 클리핑 안 된 픽셀에서 1 단계 안이다(40048 샘플). 진짜 무의미.', 'hue': 'saturation 과 같다. U/V 회전이라 회색에는 안 남는다.', 'white_balance': 'AWB 를 잠그는 것(enable_auto_white_balance)은 넣었지만 켈빈 값 자체는 뺐다. 위 상쇄 결과 때문에 범위 안 어떤 값을 골라도 회색 1 단계 안이다. 뷰어에서 이걸 튜닝하는 데 시간을 쓰지 말 것.', 'backlight_compensation': '역광 도크에서 제일 먼저 손이 가는 물건인데 틀린 답이다. 0/1 두 단계뿐인 문서화 안 된 전체화면 계측 재가중이고, 수동노출로 가면 아예 아무 일도 안 한다. 같은 문제를 aim_ae_at_bbox() 가 정확히, 우리 통제 아래 푼다. (이 개체에서 픽셀이 실제로 바뀌는지는 못 재봤다 — 카메라 미연결.)', 'auto_exposure_limit / auto_exposure_limit_toggle': "이게 진짜로 원했던 물건이라 특히 분명히 해 둔다: **컬러 센서에는 없다.** d400-device.cpp:1060-1070 이 get_depth_sensor() 에만, 그것도 CAP_GLOBAL_SHUTTER 일 때만 등록한다. 주석이 대놓고 'ae / gain limit feature is not supported on rolling-shutter' 라고 적혀 있고 D435i 의 RGB 는 롤링셔터다. pyrealsense2 에 열거값이 있다는 것은 지원한다는 뜻이 아니다(열거값은 장치와 무관하게 늘 있다). 확인은 ae_limit_supported() 한 줄이면 된다. 그래서 수동노출로 간다."}
