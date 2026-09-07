import math
import threading
import time
from config.imu import IMU_BIAS_SEC, IMU_CALIB_MIN_RATIO, IMU_DT_GAP_SAMPLES, IMU_GYRO_HZ, IMU_MOVING_DPS, IMU_STALE_SEC, IMU_YAW_SIGN
ACCEL_HZ_CANDIDATES = (63, 100, 200, 250)

class GyroYaw:

    def __init__(self, hz=IMU_GYRO_HZ, sign=IMU_YAW_SIGN, use_accel=True):
        self.hz = int(hz)
        self.sign = float(sign)
        self.use_accel = bool(use_accel)
        self._pipe = None
        self._lock = threading.Lock()
        self._angle_rad = 0.0
        self._bias = (0.0, 0.0, 0.0)
        self._last_w = (0.0, 0.0, 0.0)
        self._t_last = None
        self._wall_last = None
        self._n = 0
        self._gaps = 0
        self._t_first = None
        self._calib = None
        self._calib_a = None
        self._calibrated = False
        self._noise_dps = None
        self._axis = (0.0, -1.0, 0.0)
        self._axis_src = '가정(카메라 수평)'
        self._dt_max = float(IMU_DT_GAP_SAMPLES) / self.hz

    def start(self):
        import pyrealsense2 as rs
        pipe = rs.pipeline()
        cfg = rs.config()
        cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.hz)
        if self.use_accel:
            for accel_hz in ACCEL_HZ_CANDIDATES:
                cfg_try = rs.config()
                cfg_try.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.hz)
                cfg_try.enable_stream(rs.stream.accel, rs.format.motion_xyz32f, accel_hz)
                if cfg_try.can_resolve(pipe):
                    cfg = cfg_try
                    break
            else:
                self.use_accel = False
        try:
            pipe.start(cfg, self._on_frame)
        except RuntimeError as exc:
            if self.use_accel:
                self.use_accel = False
                cfg = rs.config()
                cfg.enable_stream(rs.stream.gyro, rs.format.motion_xyz32f, self.hz)
                try:
                    pipe.start(cfg, self._on_frame)
                except RuntimeError as exc2:
                    raise RuntimeError('gyro %dHz 스트림을 못 열었다 (%s) — 이 장치의 유효값은 200/400 뿐이다. 장치 확인: tools/realsense_check.py' % (self.hz, exc2)) from exc2
            else:
                raise RuntimeError('gyro %dHz 스트림을 못 열었다 (%s) — 이 장치의 유효값은 200/400 뿐이다. 장치 확인: tools/realsense_check.py' % (self.hz, exc)) from exc
        self._pipe = pipe
        return self

    def close(self):
        p, self._pipe = (self._pipe, None)
        if p is not None:
            try:
                p.stop()
            except Exception:
                pass

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False

    def _on_frame(self, f):
        if not f.is_motion_frame():
            return
        m = f.as_motion_frame()
        name = m.get_profile().stream_type().name
        v = m.get_motion_data()
        if name == 'accel':
            with self._lock:
                if self._calib_a is not None:
                    self._calib_a.append((v.x, v.y, v.z))
            return
        if name != 'gyro':
            return
        ts = m.get_timestamp() / 1000.0
        with self._lock:
            self._n += 1
            self._last_w = (v.x, v.y, v.z)
            if self._t_first is None:
                self._t_first = ts
            if self._t_last is not None:
                dt = ts - self._t_last
                if 0.0 < dt <= self._dt_max:
                    self._angle_rad += self._project(v) * dt
                else:
                    self._gaps += 1
            self._t_last = ts
            self._wall_last = time.monotonic()
            if self._calib is not None:
                self._calib.append((v.x, v.y, v.z))

    def _project(self, w):
        ax, ay, az = self._axis
        bx, by, bz = self._bias
        return self.sign * ((w.x - bx) * ax + (w.y - by) * ay + (w.z - bz) * az)

    def calibrate(self, sec=IMU_BIAS_SEC):
        if self._pipe is None:
            raise RuntimeError('start() 전에 calibrate() 를 불렀다')
        with self._lock:
            self._calib = []
            self._calib_a = [] if self.use_accel else None
        time.sleep(float(sec))
        with self._lock:
            got, self._calib = (self._calib, None)
            got_a, self._calib_a = (self._calib_a, None)
        need = self.hz * sec * IMU_CALIB_MIN_RATIO
        if len(got) < need:
            raise RuntimeError('보정 샘플이 %d개뿐이다(기대 %d) — gyro 가 안 들어온다' % (len(got), int(self.hz * sec)))
        n = len(got)
        mean = [sum((v[i] for v in got)) / n for i in range(3)]
        if not self.use_accel:
            axis, src = ((0.0, -1.0, 0.0), '가정(카메라 수평) — accel 스트림이 안 열림 (동시 스트림 거부, 자이로 단독)')
        elif got_a is not None and len(got_a) < 3:
            axis, src = ((0.0, -1.0, 0.0), '가정(카메라 수평) — accel 스트림은 열렸는데 샘플이 %d개뿐' % len(got_a))
        else:
            axis, src = self._axis_from_gravity(got_a)
        proj = [sum(((v[i] - mean[i]) * axis[i] for i in range(3))) for v in got]
        var = sum((p * p for p in proj)) / max(n - 1, 1)
        sd = math.sqrt(var)
        deg = math.degrees
        mean_dps = abs(deg(sum((mean[i] * axis[i] for i in range(3)))))
        moving = deg(sd) > IMU_MOVING_DPS or mean_dps > IMU_MOVING_DPS
        if not moving:
            with self._lock:
                self._bias = tuple(mean)
                self._axis, self._axis_src = (axis, src)
                self._angle_rad = 0.0
                self._calibrated = True
                self._noise_dps = math.degrees(sd)
        accel_mean = tuple((sum((v[i] for v in got_a)) / len(got_a) for i in range(3))) if got_a else None
        return {'n': n, 'sec': float(sec), 'accel_mean': accel_mean, 'bias_dps': tuple((deg(b) for b in mean)), 'noise_dps': deg(sd), 'drift_dpm': deg(sd / math.sqrt(n)) * 60.0, 'moving': moving, 'mean_dps': mean_dps, 'axis': axis, 'axis_src': src}

    def _axis_from_gravity(self, samples):
        default = (0.0, -1.0, 0.0)
        if not samples or len(samples) < 3:
            return (default, '가정(카메라 수평) — 가속도 없음')
        n = len(samples)
        g = [sum((v[i] for v in samples)) / n for i in range(3)]
        mag = math.sqrt(sum((c * c for c in g)))
        if not 7.0 < mag < 12.5:
            return (default, '가정(카메라 수평) — 중력 크기 %.1f' % mag)
        up = tuple((c / mag for c in g))
        if up[1] > 0:
            up = tuple((-c for c in up))
        tilt = math.degrees(math.acos(max(-1.0, min(1.0, -up[1]))))
        if tilt > 60.0:
            return (default, '가정(카메라 수평) — 중력축 기울기 %.0f도, 못 믿음' % tilt)
        return (up, '중력(기울기 %.1f도)' % tilt)

    def zero(self):
        with self._lock:
            self._angle_rad = 0.0

    @property
    def angle_deg(self):
        with self._lock:
            return math.degrees(self._angle_rad)

    @property
    def rate_dps(self):
        with self._lock:
            w = _V(*self._last_w)
            return math.degrees(self._project(w))

    @property
    def age_sec(self):
        with self._lock:
            w = self._wall_last
        return float('inf') if w is None else time.monotonic() - w

    @property
    def alive(self):
        return self.age_sec <= IMU_STALE_SEC

    @property
    def calibrated(self):
        return self._calibrated

    @property
    def noise_dps(self):
        return self._noise_dps

    def axis_note(self):
        return self._axis_src

    def stats(self):
        with self._lock:
            n, gaps = (self._n, self._gaps)
            span = self._t_last - self._t_first if self._t_first is not None and self._t_last is not None else 0.0
        return {'n': n, 'gaps': gaps, 'hz': (n - 1) / span if span > 0 else 0.0}

class _V:
    __slots__ = ('x', 'y', 'z')

    def __init__(self, x, y, z):
        self.x, self.y, self.z = (x, y, z)

def imu_panel_lines(yaw):
    if yaw is None:
        return [('kv', 'IMU', '없음 — 회전이 개루프', 'bad')]
    alive = yaw.alive
    gaps = yaw.stats().get('gaps', 0)
    out = [('kv', 'IMU 각도', '%+.2f 도' % yaw.angle_deg, 'ok' if alive else 'bad'), ('kv', 'IMU 속도', '%+.2f 도/s' % yaw.rate_dps, 'ok' if alive else 'dim')]
    if not alive:
        out.append(('kv', '', '끊김 %.1fs 째 — 회전 금지' % yaw.age_sec, 'bad'))
    elif not yaw.calibrated:
        out.append(('kv', '', '보정 전 — 5.5도/분 흘러간다', 'warn'))
    elif gaps:
        out.append(('kv', '', '샘플 누락 %d회' % gaps, 'warn'))
    return out
