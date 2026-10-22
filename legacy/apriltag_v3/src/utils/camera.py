"""RealSense 카메라를 **읽고 쓰는** 것 전부. 태그 검출과 무관한 카메라 쪽 일은 여기 모음."""
from dataclasses import dataclass, asdict, replace, fields
import numpy as np

from config.detection import BLUR_CLEAN_PX, BLUR_DEAD_PX, COLOR_EXPOSURE_UNIT_US


# 1. YUYV 원본 휘도
def yuyv_to_luma(buf):
    """pyrealsense2 가 준 YUYV 버퍼에서 **센서 원본 휘도**를 뽑음."""
    a = np.asanyarray(buf)
    if a.ndim != 2:
        raise ValueError("YUYV 버퍼는 2-D 여야 한다: %r" % (a.shape,))
    if a.dtype != np.uint8:
        a = a.view(np.uint8)                 # (H, W) uint16 -> (H, 2W) uint8
    # 짝수 바이트가 Y, 홀수 바이트가 U/V 가 번갈아 듦. Y 만 걷어냄.
    return np.ascontiguousarray(a[:, 0::2])


# 2. 컬러 자동노출 ROI

class ExposureROI:
    """컬러 센서의 자동노출을 태그 사각형에만 걺."""

    #: ROI 한 변의 최소 픽셀. 이보다 작으면 펌웨어가 받지 않는 경우가 있어 넓혀 줌.
    MIN_SIDE_PX = 32

    def __init__(self, profile, pad=0.35, stream="color"):
        """Args:
        profile: pipeline.start(config) 가 준 객체
        pad: 태그 바운딩박스를 이 비율만큼 사방으로 넓혀 잡음.
        """
        self.pad = float(pad)
        self.sensor = None
        self.last = None
        self.supported = False
        try:
            import pyrealsense2 as rs
            dev = profile.get_device()
            for s in dev.query_sensors():
                name = s.get_info(rs.camera_info.name).lower()
                want_rgb = (stream == "color")
                if want_rgb != name.startswith("rgb"):
                    continue
                if s.is_roi_sensor():
                    self.sensor = s.as_roi_sensor()
                    self.supported = True
                break
        except Exception:
            pass                            

    def follow(self, detections, shape):
        """검출된 태그들을 덮는 사각형으로 AE ROI 를 옮김."""
        if not self.supported or not detections:
            return False
        h, w = shape[0], shape[1]
        pts = np.concatenate([np.asarray(d.corners, dtype=np.float64)
                              for d in detections], axis=0)
        x0, y0 = pts.min(axis=0)
        x1, y1 = pts.max(axis=0)
        px, py = (x1 - x0) * self.pad, (y1 - y0) * self.pad
        return self.set_box(x0 - px, y0 - py, x1 + px, y1 + py, w, h)

    def set_box(self, x0, y0, x1, y1, w, h):
        """픽셀 사각형으로 직접 걺. 화면 밖은 잘라내고 최소 크기를 보장함."""
        if not self.supported:
            return False
        import pyrealsense2 as rs
        # 최소 크기 확보 — 중심을 유지한 채 벌림
        cx, cy = (x0 + x1) / 2.0, (y0 + y1) / 2.0
        half = max(self.MIN_SIDE_PX / 2.0, (x1 - x0) / 2.0)
        x0, x1 = cx - half, cx + half
        half = max(self.MIN_SIDE_PX / 2.0, (y1 - y0) / 2.0)
        y0, y1 = cy - half, cy + half

        box = (max(0, int(x0)), max(0, int(y0)),
               min(w - 1, int(round(x1))), min(h - 1, int(round(y1))))
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


# 3. 프레임 드롭 회계

class FrameStats:
    """프레임을 몇 장 흘렸는지 셈."""

    def __init__(self):
        self.received = 0
        self.dropped = 0
        self.gaps = 0          
        self.first_number = None
        self.last_number = None
        self._t_first = None
        self._t_last = None

    def update(self, frame_or_number, t=None):
        """프레임 하나를 반영함."""
        n = getattr(frame_or_number, "frame_number", None)
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
        """소비 fps. 타임스탬프를 안 줬으면 0.0."""
        if self._t_first is None or self._t_last is None:
            return 0.0
        dt = self._t_last - self._t_first
        return (self.received - 1) / dt if dt > 0 else 0.0

    @property
    def sent(self):
        """카메라가 보낸 장 수 = 받은 것 + 버려진 것. "300 중 12 버림"의 300."""
        return self.received + self.dropped

    @property
    def drop_rate(self):
        """버려진 비율 0.0~1.0. 카메라가 보낸 것 대비."""
        return (self.dropped / self.sent) if self.sent else 0.0

    def summary(self):
        """한 줄 요약."""
        s = "받음 %d / 보냄 %d, 버림 %d (%.1f%%), 구멍 %d회" % (
            self.received, self.sent, self.dropped, self.drop_rate * 100.0, self.gaps)
        if self.fps:
            s += ", 소비 %.1f fps" % self.fps
        if self.last_number is None:
            s += "  [frame_number 없음 — 개수만 셈]"
        return s


# 4. 프레임 메타데이터 / 타임스탬프

# 값이 있을 때 자세 실패 원인 규명에 실제로 쓰이는 것들만 추렸음.
# frame_timestamp - sensor_timestamp = Δ_FS (USB 전송 시작 ↔ 노출중심 차, plan 4-1).
# 이 둘이 있어야 t_capture 를 노출중심으로 되돌릴 수 있다.
_META_KEYS = ("actual_exposure", "gain_level", "frame_counter", "sensor_timestamp",
              "frame_timestamp", "time_of_arrival", "backend_timestamp", "actual_fps",
              "auto_exposure", "white_balance", "frame_laser_power_mode")


def frame_meta(f, keys=_META_KEYS):
    """프레임 하나의 메타데이터를 dict 로. 없는 항목은 아예 넣지 않음."""
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
    """이 장치/커널에서 **실제로** 살아 있는 메타데이터를 전부 훑어 문자열로."""
    try:
        import pyrealsense2 as rs
    except Exception:
        return "pyrealsense2 를 못 불러왔다"
    names = [a for a in dir(rs.frame_metadata_value)
             if not a.startswith("_") and a not in ("name", "value")]
    got = []
    for n in names:
        mv = getattr(rs.frame_metadata_value, n)
        try:
            if f.supports_frame_metadata(mv):
                got.append("  %-28s %s" % (n, f.get_frame_metadata(mv)))
        except Exception:
            pass
    head = "timestamp=%.3f ms  domain=%s  frame_number=%d" % (
        f.get_timestamp(), f.get_frame_timestamp_domain(), f.get_frame_number())
    if not got:
        return head + "\n  (메타데이터 없음 — 리눅스면 커널 패치 미적용)"
    return head + "\n" + "\n".join(got)


def timestamp_domain(f):
    """이 프레임의 타임스탬프가 무슨 시계인지 문자열로."""
    return str(f.get_frame_timestamp_domain())


def set_global_time(profile, enabled=True):
    """모든 센서의 global_time_enabled 를 켜고/끔."""
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


# 5. 프레임별 메타데이터 수집기

class MetaReader:
    """프레임마다 노출/게인/센서시각을 뽑아 Frame 에 실어 보내기 위한 수집기."""

    def __init__(self, keys=_META_KEYS):
        self._keys = tuple(keys)
        self._probed = None        
        self.supported = ()       

    def probe(self, f):
        """첫 프레임에서 살아 있는 키를 한 번만 조사함."""
        try:
            import pyrealsense2 as rs
        except Exception:
            self._probed, self.supported = (), ()
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
        self.supported = tuple(k for k, _ in live)
        return self.supported

    def read(self, f):
        """이 프레임의 메타데이터 dict. 없으면 {} (예외 없음)."""
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


# 6. 검출 실패 원인 규명

# 모션블러 절벽. 1920x1080, 태그 238px(=tag36h11 한 칸 23.8px)에서 실측한 값.

# 아래 두 이름은 병합 전 camera_control.py 가 rs_tuning 에서 별칭으로 끌어 쓰던 것.
BLUR_PX_100PCT = BLUR_CLEAN_PX     # 검출 100% 를 지키는 블러 상한 [px]
BLUR_PX_ZERO = BLUR_DEAD_PX        # 검출이 0% 로 무너지는 블러 [px]

def motion_blur_px(exposure_us, speed_mps, fx, z_m):
    """노출시간 동안 태그가 화면에서 몇 픽셀 밀리는가."""
    if not z_m or z_m <= 0:
        return float("inf")
    return float(fx) * float(speed_mps) * (float(exposure_us) * 1e-6) / float(z_m)


def exposure_budget_us(speed_mps, fx, z_m, blur_px=BLUR_CLEAN_PX):
    """검출률 100% 를 지키려면 노출을 몇 us 안으로 끊어야 하는가."""
    v = abs(float(speed_mps))
    if v <= 0:
        return float("inf")
    return float(blur_px) * float(z_m) / (float(fx) * v) * 1e6


def diagnose_frame(img, fx=None, z_m=None, speed_mps=None):
    """이 프레임에서 태그를 못 찾았다면 **왜 못 찾았는지** 한 줄로."""
    parts = []
    n = getattr(img, "dropped_before", 0) or 0
    if n:
        parts.append("직전 %d장 유실(파이프라인 큐가 버림)" % n)

    exp = getattr(img, "exposure_us", None)
    gain = getattr(img, "gain", None)
    if exp is None and gain is None:
        if getattr(img, "meta", None) is None:
            why = "RealSense 프레임이 아니다(영상파일/웹캠) — 노출 정보가 원래 없다"
        else:
            why = "프레임 메타데이터가 안 온다(리눅스면 커널 패치 미적용)"
        if not parts:
            return "판단 재료 없음 — " + why
        parts.append("노출/게인 불명 — " + why)
        return ", ".join(parts)

    if exp is not None:
        s = "노출 %.1fms" % (exp / 1000.0)
        if fx and z_m and speed_mps:
            b = motion_blur_px(exp, speed_mps, fx, z_m)
            if b >= BLUR_DEAD_PX:
                s += " -> 블러 %.1fpx: 실측 0%% 구간(%.0fpx 에서 검출이 끊긴다)" % (b, BLUR_DEAD_PX)
            elif b > BLUR_CLEAN_PX:
                s += " -> 블러 %.1fpx: 실측 열화 구간(%.0fpx 넘음)" % (b, BLUR_CLEAN_PX)
            else:
                s += " -> 블러 %.1fpx: 블러는 무죄" % b
            s += " (노출을 %.1fms 로 끊으면 100%%)" % (
                exposure_budget_us(speed_mps, fx, z_m) / 1000.0)
        parts.append(s)
    if gain is not None:
        parts.append("게인 %d" % gain)
    return ", ".join(parts)


# 7. 컬러 센서 노출 제어

def set_color_exposure(profile, exposure_us=None, ae_priority=None, gain=None):
    """컬러 센서의 노출을 직접 잡음. 모션블러를 끊는 **유일한** 방법."""
    out = {"exposure_us": None, "ae_priority": None, "gain": None,
           "auto_exposure": None, "errors": []}
    try:
        import pyrealsense2 as rs
    except Exception:
        out["errors"].append("pyrealsense2")
        return out

    sensor = None
    for s in profile.get_device().query_sensors():
        try:
            if s.get_info(rs.camera_info.name).lower().startswith("rgb"):
                sensor = s
                break
        except Exception:
            pass
    if sensor is None:
        out["errors"].append("rgb sensor 없음")
        return out

    def _set(opt, value, name):
        try:
            if not sensor.supports(opt):
                out["errors"].append(name + "(미지원)")
                return None
            rng = sensor.get_option_range(opt)
            v = min(max(float(value), rng.min), rng.max)  
            sensor.set_option(opt, v)
            return sensor.get_option(opt)          
        except Exception as exc:
            out["errors"].append("%s(%s)" % (name, exc.__class__.__name__))
            return None

    if ae_priority is not None:
        v = _set(rs.option.auto_exposure_priority, float(ae_priority), "ae_priority")
        out["ae_priority"] = None if v is None else int(v)
    if exposure_us is not None:
        v = _set(rs.option.exposure, float(exposure_us) / COLOR_EXPOSURE_UNIT_US, "exposure")
        out["exposure_us"] = None if v is None else float(v) * COLOR_EXPOSURE_UNIT_US
    if gain is not None:
        v = _set(rs.option.gain, float(gain), "gain")
        out["gain"] = None if v is None else float(v)
    try:
        if sensor.supports(rs.option.enable_auto_exposure):
            out["auto_exposure"] = bool(sensor.get_option(rs.option.enable_auto_exposure))
    except Exception:
        pass
    return out


#: 한국 상용전원 60 Hz. 형광등/저가 LED 는 그 **두 배**인 120 Hz 로 깜빡임.
MAINS_HALF_CYCLE_MS = 1000.0 / 120.0

# BLUR_PX_100PCT(10) / BLUR_PX_ZERO(32) 는 위에서 위쪽 읽기 절에서 정의한 값을 그대로 씀.

#: realsense-viewer 의 AE ROI "reset" 이 쓰는 상자 = 화면 가운데 3/4
AE_ROI_MARGIN_FRACTION = 1.0 / 8.0


# ── 노출 시간을 숫자로 정하는 근거 ───────────────────────────────────────────

def exposure_ms_for_motion(speed_mps, range_m, fx, blur_px=BLUR_PX_100PCT):
    """이 속도/거리에서 블러를 blur_px 안에 묶는 노출시간 [ms]."""
    speed_mps = abs(float(speed_mps))
    if speed_mps <= 0.0:
        return float("inf")                  # 정지 상태면 블러 상한이 없음
    return 1000.0 * float(blur_px) * float(range_m) / (float(fx) * speed_mps)


def exposure_units(ms, unit_us=COLOR_EXPOSURE_UNIT_US):
    """밀리초 -> 컬러 exposure 옵션 값. 최소 1 칸은 보장함."""
    return max(1, int(round(float(ms) * 1000.0 / float(unit_us))))


def exposure_ms(units, unit_us=COLOR_EXPOSURE_UNIT_US):
    """컬러 exposure 옵션 값 -> 밀리초."""
    return float(units) * float(unit_us) / 1000.0


#: 도킹 기본 노출값. 왜 하필 83 인가 — 두 가지 제약이 같은 곳에서 만남.
DOCKING_EXPOSURE_UNITS = exposure_units(MAINS_HALF_CYCLE_MS)   # = 83


# ── 설정 묶음 ────────────────────────────────────────────────────────────────

@dataclass
class CameraSettings:
    """컬러 센서에서 **태그 검출에 실제로 영향이 있는** 옵션만 담은 묶음."""

    #: 자동노출. False 면 노출이 프레임 사이에 안 움직임.
    enable_auto_exposure: bool = None

    #: 노출값. **단위는 100us** (COLOR_EXPOSURE_UNIT_US). 83 = 8.3 ms.
    exposure: float = None

    #: 아날로그 게인. 녹화된 D435 기본값 64.
    gain: float = None

    #: 0=끔 1=50Hz 2=60Hz 3=자동 (d400-color.cpp:205-212 의 값 매핑 그대로).
    power_line_frequency: int = None

    #: 1 이면 어두울 때 AE 가 **프레임률을 떨어뜨려서** 노출을 더 벌 수 있음. 기본값이 1.
    auto_exposure_priority: int = None

    #: 자동 화이트밸런스. 잠금.
    enable_auto_white_balance: bool = None

    #: SDK 쪽 프레임 큐 깊이(펌웨어 아님). 녹화된 기본값 16.
    frames_queue_size: int = None

    #: 프레임 타임스탬프를 호스트 시계에 맞춤. 검출 품질과는 무관함.
    global_time_enabled: bool = None

    # ── 만들기 ──────────────────────────────────────────────────────────────

    @classmethod
    def docking(cls, speed_mps=None, range_m=1.0, fx=None):
        """도킹용 기본 묶음."""
        units = DOCKING_EXPOSURE_UNITS
        if speed_mps:
            if not fx:
                # 기본값을 두면 해상도가 달라도 조용히 1080p 로 계산해 버림.
                # 부르는 쪽은 CameraIntrinsics.fx 를 이미 갖고 있음.
                raise ValueError("speed_mps 를 주면 fx 도 줘야 한다 (intr.fx)")
            ms = exposure_ms_for_motion(speed_mps, range_m, fx)
            # 깜빡임 때문에 반주기(8.333ms)의 정수배로 내림. 한 주기 밑으로는 못 감 —
            n = int(ms / MAINS_HALF_CYCLE_MS)
            units = exposure_units(n * MAINS_HALF_CYCLE_MS) if n >= 1 else exposure_units(ms)
        return cls(
            enable_auto_exposure=False,
            exposure=float(units),
            gain=64.0,                        # 공장 기본값 그대로. 위 gain 주석 참고
            power_line_frequency=2,           # 60Hz (한국)
            auto_exposure_priority=0,         # 프레임률 고정
            enable_auto_white_balance=False,
            frames_queue_size=1,              # 최신 프레임만
            global_time_enabled=True,
        )

    @classmethod
    def from_sensor(cls, sensor):
        """지금 센서 상태를 그대로 뜸. 지원 안 하는 옵션은 None 으로 남음."""
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
            if f.type is bool or f.name.startswith(("enable_", "global_time")):
                out[f.name] = bool(round(v))
            elif f.type is int:
                out[f.name] = int(round(v))
            else:
                out[f.name] = float(v)
        return cls(**out)

    # ── 쓰기 ────────────────────────────────────────────────────────────────

    def apply(self, sensor, strict=False):
        """센서에 씀. **순서가 전부.**"""
        import pyrealsense2 as rs
        # **아무것도 쓰기 전에** 모순부터 봄. 순서가 중요한 만큼 중간에 터지면
        if self.enable_auto_exposure is True and strict:
            self.validate()
        s = color_sensor(sensor)
        report = {}

        common = ("power_line_frequency", "auto_exposure_priority",
                  "enable_auto_white_balance", "frames_queue_size",
                  "global_time_enabled")
        if self.enable_auto_exposure is True:
            # **자동노출을 켜는 요청(주로 되돌리기).**
            order = common + ("exposure", "gain", "enable_auto_exposure")
        else:
            # **수동으로 고정하는 요청(도킹).**
            order = common + ("enable_auto_exposure", "exposure", "gain")

        for name in order:
            want = getattr(self, name)
            if want is None:
                continue                      # None = 건드리지 않음
            opt = getattr(rs.option, name, None)
            if opt is None:
                if strict:
                    raise AttributeError("pyrealsense2 에 rs.option.%s 가 없다" % name)
                continue
            try:
                if not s.supports(opt):
                    if strict:
                        raise RuntimeError("센서가 %s 를 지원하지 않는다" % name)
                    report[name] = (None, None)
                    continue
                before = s.get_option(opt)
            except Exception:
                report[name] = (None, None)
                continue

            val = float(want)
            try:                              # 장치가 아는 범위로 자름
                r = s.get_option_range(opt)
                val = min(max(val, r.min), r.max)
            except Exception:
                pass
            try:
                s.set_option(opt, val)
                after = s.get_option(opt)     # 쓴 값이 아니라 읽은 값을 보고함
            except Exception:
                if strict:
                    raise
                after = None
            report[name] = (before, after)
        return report

    def validate(self):
        """모순된 조합을 걸러냄. apply(strict=True) 가 **쓰기 전에** 부름."""
        if self.enable_auto_exposure is True:
            bad = [n for n in ("exposure", "gain") if getattr(self, n) is not None]
            if bad:
                raise ValueError(
                    "enable_auto_exposure=True 와 %s 를 같이 줄 수 없다 — "
                    "쓰는 순간 SDK 가 AE 를 꺼버린다"
                    "(ds-color-common.cpp:90-93 + option.cpp:84-105). "
                    "노출을 고정하려면 enable_auto_exposure=False 를, "
                    "자동노출을 쓰려면 %s 를 None 으로 둘 것."
                    % (" / ".join(bad), " / ".join(bad)))
        return self

    # ── 곁다리 ──────────────────────────────────────────────────────────────

    @property
    def exposure_ms(self):
        """노출을 밀리초로. None 이면 None."""
        return None if self.exposure is None else exposure_ms(self.exposure)

    def describe(self):
        """사람이 읽을 여러 줄 문자열. 로그에 한 번 찍어 두면 나중에 살아남."""
        lines = []
        for f in fields(self):
            v = getattr(self, f.name)
            if v is None:
                lines.append("  %-26s -            (건드리지 않음)" % f.name)
            elif f.name == "exposure":
                lines.append("  %-26s %-12g (%.2f ms)" % (f.name, v, exposure_ms(v)))
            else:
                lines.append("  %-26s %s" % (f.name, v))
        return "\n".join(lines)


#: 도킹 기본값. 모듈 상수로도 하나 놔둠 — open_realsense(tune=True) 가 이걸 씀.
DOCKING_SETTINGS = CameraSettings.docking()


# ── 센서 찾기 ────────────────────────────────────────────────────────────────

def color_sensor(obj):
    """무엇을 주든 컬러 센서를 찾아 줌."""
    import pyrealsense2 as rs
    if isinstance(obj, rs.sensor) or hasattr(obj, "get_option_range"):
        return obj
    dev = obj.get_device() if hasattr(obj, "get_device") else obj
    for s in dev.query_sensors():
        try:
            if s.get_info(rs.camera_info.name).lower().startswith("rgb"):
                return s
        except Exception:
            pass
    return dev.first_color_sensor()           # 이름을 못 찾았을 때의 최후수단


# ── 한 방에 정리 ─────────────────────────────────────────────────────────────

def tune_for_tags(target, settings=None, speed_mps=None, range_m=1.0, fx=None,
                  verbose=False):
    """카메라를 태그 검출하기 좋은 상태로 만듦. 한 번만 부르면 됨."""
    s = color_sensor(target)
    want = settings or CameraSettings.docking(speed_mps=speed_mps, range_m=range_m, fx=fx)
    before = CameraSettings.from_sensor(s)
    report = want.apply(s)
    if verbose:
        print("[tune_for_tags] 컬러 센서 설정")
        for k, (b, a) in report.items():
            if a is None:
                print("  %-26s %-12s -> (못 씀 / 미지원)" % (k, b))
            else:
                extra = "  (%.2f ms)" % exposure_ms(a) if k == "exposure" else ""
                mark = "" if b == a else "  *"
                print("  %-26s %-12g -> %-12g%s%s" % (k, b if b is not None else float("nan"), a, extra, mark))
    return before, report


def ae_limit_supported(target):
    """"자동노출 상한" 이 이 센서에 정말 있는지 카메라에 직접 물어봄."""
    import pyrealsense2 as rs
    dev = target.get_device() if hasattr(target, "get_device") else target
    out = {}
    opt = getattr(rs.option, "auto_exposure_limit", None)
    for key, getter in (("color", "first_color_sensor"), ("depth", "first_depth_sensor")):
        try:
            s = color_sensor(dev) if key == "color" else getattr(dev, getter)()
            out[key] = bool(opt is not None and s.supports(opt))
        except Exception:
            out[key] = False
    return out


def describe_color_options(target):
    """컬러 센서의 모든 옵션을 값/범위와 함께 훑어 문자열로."""
    import pyrealsense2 as rs
    s = color_sensor(target)
    lines = []
    for name in sorted(a for a in dir(rs.option) if not a.startswith("_")
                       and a not in ("name", "value")):
        opt = getattr(rs.option, name)
        try:
            if not s.supports(opt):
                continue
            v = s.get_option(opt)
        except Exception:
            continue
        try:
            r = s.get_option_range(opt)
            rng = "[%g .. %g] step %g def %g" % (r.min, r.max, r.step, r.default)
        except Exception:
            rng = "(범위 없음)"
        lines.append("  %-28s %-12g %s" % (name, v, rng))
    return "\n".join(lines) if lines else "  (읽을 수 있는 옵션이 없다)"


# ── 자동노출 ROI ─────────────────────────────────────────────────────────────

def ae_roi_supported(target):
    """이 장치의 컬러 센서가 AE ROI 를 받는지."""
    try:
        return bool(color_sensor(target).is_roi_sensor())
    except Exception:
        return False


def aim_ae_at_bbox(target, bbox, shape, pad=0.35):
    """자동노출 계측창을 태그 자리에 걺."""
    import pyrealsense2 as rs
    try:
        s = color_sensor(target)
        if not s.is_roi_sensor():
            return False
        r = s.as_roi_sensor()
    except Exception:
        return False

    h, w = int(shape[0]), int(shape[1])
    x0, y0, x1, y1 = (float(v) for v in bbox)
    px, py = (x1 - x0) * pad, (y1 - y0) * pad
    x0, y0, x1, y1 = x0 - px, y0 - py, x1 + px, y1 + py

    box = (max(0, int(x0)), max(0, int(y0)),
           min(w - 1, int(round(x1))), min(h - 1, int(round(y1))))
    if box[2] - box[0] < 8 or box[3] - box[1] < 8:
        return False                          # 너무 작으면 펌웨어가 거부함
    roi = rs.region_of_interest()
    roi.min_x, roi.min_y, roi.max_x, roi.max_y = box
    try:
        r.set_region_of_interest(roi)
    except Exception:
        return False                          # rs.cpp:1793 은 min<=max 만 보고
    return True                               # 실제 거부는 펌웨어가 함


def center_ae_roi(target, width, height):
    """AE ROI 를 기본 상자(가운데 3/4)로 되돌림."""
    mx = int(width * AE_ROI_MARGIN_FRACTION)
    my = int(height * AE_ROI_MARGIN_FRACTION)
    return aim_ae_at_bbox(target, (mx, my, width - 1 - mx, height - 1 - my),
                          (height, width), pad=0.0)


def ae_roi_of(target):
    """지금 걸려 있는 AE ROI 를 (x0, y0, x1, y1) 로. 못 읽으면 None."""
    try:
        r = color_sensor(target).as_roi_sensor().get_region_of_interest()
        return (r.min_x, r.min_y, r.max_x, r.max_y)
    except Exception:
        return None


# ── 일부러 뺀 것들 ───────────────────────────────────────────────────────────

#: 나중에 "이것도 넣으면 낫지 않나" 하고 돌아오는 것을 막으려고 남김.
_WHY_NOT = {
    "sharpness":
        "위험해서 뺐다. 언샤프 마스크는 흑백 경계에 오버슈트 링잉을 얹는데, "
        "AprilTag 는 바로 그 경계에 직선을 맞춰 모서리를 잡는다. 링잉은 밝은 쪽에서 "
        "선을 바깥으로, 어두운 쪽에서 안으로 민다 — 거리에 따라 달라지는 계통오차이고 "
        "그대로 estimate_pose 와 MAX_REPROJ_RMS_PX 로 들어간다. 흐린 영상에서 "
        "검출 '개수'는 늘려 놓고 자세 '정확도'는 망칠 수 있어 도킹에서 최악의 실패 방식이다.",
    "contrast":
        "무의미해서 뺐다. 단조 톤커브인데 AprilTag 의 임계화는 국소 적응형이라 "
        "타일마다 min/max 를 재고 중간에서 자른다. 단조 변환은 그 판정을 안 바꾼다. "
        "할 수 있는 일이라곤 흰칸을 클리핑시키거나 검은칸을 0 으로 눌러 정보를 "
        "없애는 것뿐이다.",
    "gamma":
        "contrast 와 같은 이유. 흑백 두 무리가 0~255 중 어디 앉는지를 바꿀 뿐 "
        "얼마나 잘 갈라지는지를 못 바꾼다. 흑백 타깃이니 감마가 도움이 될 것 같다는 "
        "직관이 강해서 굳이 적어 둔다.",
    "brightness":
        "노출 뒤에 더해지는 DC 오프셋이라 검은칸과 흰칸을 똑같이 민다. "
        "임계화가 보는 것은 둘의 '차이'라 한쪽이 클리핑될 때까지 아무 변화가 없다. "
        "오프셋 말고 신호를 바꾸는 노출이 언제나 낫다.",
    "saturation":
        "색차 전용. 실측으로 상쇄가 확인됐다 — U,V 를 키워도 BGR2GRAY 결과는 "
        "클리핑 안 된 픽셀에서 1 단계 안이다(40048 샘플). 진짜 무의미.",
    "hue":
        "saturation 과 같다. U/V 회전이라 회색에는 안 남는다.",
    "white_balance":
        "AWB 를 잠그는 것(enable_auto_white_balance)은 넣었지만 켈빈 값 자체는 뺐다. "
        "위 상쇄 결과 때문에 범위 안 어떤 값을 골라도 회색 1 단계 안이다. "
        "뷰어에서 이걸 튜닝하는 데 시간을 쓰지 말 것.",
    "backlight_compensation":
        "역광 도크에서 제일 먼저 손이 가는 물건인데 틀린 답이다. 0/1 두 단계뿐인 "
        "문서화 안 된 전체화면 계측 재가중이고, 수동노출로 가면 아예 아무 일도 안 한다. "
        "같은 문제를 aim_ae_at_bbox() 가 정확히, 우리 통제 아래 푼다. "
        "(이 개체에서 픽셀이 실제로 바뀌는지는 못 재봤다 — 카메라 미연결.)",
    "auto_exposure_limit / auto_exposure_limit_toggle":
        "이게 진짜로 원했던 물건이라 특히 분명히 해 둔다: **컬러 센서에는 없다.** "
        "d400-device.cpp:1060-1070 이 get_depth_sensor() 에만, 그것도 "
        "CAP_GLOBAL_SHUTTER 일 때만 등록한다. 주석이 대놓고 "
        "'ae / gain limit feature is not supported on rolling-shutter' 라고 적혀 있고 "
        "D435i 의 RGB 는 롤링셔터다. pyrealsense2 에 열거값이 있다는 것은 "
        "지원한다는 뜻이 아니다(열거값은 장치와 무관하게 늘 있다). "
        "확인은 ae_limit_supported() 한 줄이면 된다. 그래서 수동노출로 간다.",
}
