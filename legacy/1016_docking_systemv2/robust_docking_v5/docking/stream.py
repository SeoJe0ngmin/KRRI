"""Small, bounded stream utilities. IMU integration is separate from images."""
import collections
import math
import threading
import time
import copy
import bisect
from .calibration import StationaryCalibration


class Latest:
    def __init__(self):
        self.cv = threading.Condition()
        self.item = None
        self.replaced = 0
        self.closed = False

    def put(self, value):
        with self.cv:
            if self.item is not None:
                self.replaced += 1
            self.item = value
            self.cv.notify()

    def get(self, timeout=.2):
        with self.cv:
            if self.item is None and not self.closed:
                self.cv.wait(timeout)
            item, self.item = self.item, None
            return item

    def close(self):
        with self.cv:
            self.closed = True
            self.cv.notify_all()


class Imu:
    def __init__(self, cfg):
        self.cfg, self.lock = cfg, threading.Lock()
        self.last = self.arrival = None
        self.angle = self.rate = 0.
        self.count = self.gaps = self.reversed = 0
        self.max_gap = 0.
        self.history = collections.deque(maxlen=4000)
        self.accel = None
        self.calibration = StationaryCalibration(cfg) if cfg.get('startup_calibration') else None
        self.calibration_applied = False
        self.bias, self.axis = cfg['gyro_bias_rps'][:], cfg['yaw_axis'][:]
        self.created = time.monotonic()
        self.vector_rate = 0.
        self.raw_history = collections.deque(maxlen=12000)
        self.raw_last = None
        # Corrected 3-axis rates for short image-to-image rotation prediction.
        # No integration or NumPy work runs on the SDK callback thread.
        self.gyro_vectors = collections.deque(maxlen=2000)
        self.stationary_candidate = None
        self.stationary_cursor = self.stationary_frame = None
        self.stationary_updates = 0
        self.stationary_result = None
        self.stationary_reason = 'not_observed_stationary'
        self.stationary_initial_bias = None

    def _calibrate(self, kind, stamp, xyz):
        if self.calibration:
            self.calibration.add(kind, stamp, xyz)
            if self.calibration.state == 'READY' and not self.calibration_applied:
                r = self.calibration.result
                self.bias, self.axis = r['gyro_bias_rps'][:], r['yaw_axis'][:]
                self.history.clear()
                self.gyro_vectors.clear()
                self.last = None
                self.angle = self.rate = 0.
                self.calibration_applied = True

    def add_accel(self, stamp, xyz):
        with self.lock:
            self.accel = tuple(xyz)
            self.raw_history.append(('accel', stamp, tuple(xyz)))
            self._calibrate('accel', stamp, xyz)

    def calibration_error(self, reason):
        with self.lock:
            if self.calibration:
                self.calibration.state, self.calibration.reason = 'FAILED', reason

    def add(self, stamp, xyz, arrival=None):
        with self.lock:
            self._calibrate('gyro', stamp, xyz)
            rate = sum((v-b)*a for v, b, a in zip(xyz, self.bias, self.axis))
            rate *= self.cfg['gyro_scale']
            self.vector_rate = math.sqrt(sum((v-b)**2 for v, b in zip(xyz, self.bias)))*abs(self.cfg['gyro_scale'])
            if self.last is not None:
                dt = stamp-self.last
                if dt <= 0:
                    self.reversed += 1
                    return
                self.max_gap = max(self.max_gap, dt)
                if dt > self.cfg['max_gap_s']:
                    self.gaps += 1
                else:
                    self.angle += (rate+self.rate)*dt/2
            self.rate, self.last = rate, stamp
            self.raw_last = tuple(xyz)
            self.gyro_vectors.append((stamp, tuple((v-b)*self.cfg['gyro_scale']
                                                  for v, b in zip(xyz, self.bias))))
            self.raw_history.append(('gyro', stamp, tuple(xyz)))
            self.arrival = time.monotonic() if arrival is None else arrival
            self.count += 1
            self.history.append((stamp, self.angle, self.rate, self.gaps))

    def snapshot(self, now=None):
        with self.lock:
            t = time.monotonic() if now is None else now
            if self.calibration and self.calibration.state == 'COLLECTING' and t-self.created > self.calibration.cfg['timeout_s']:
                self.calibration.state, self.calibration.reason = 'FAILED', 'host_calibration_timeout'
            return {'count': self.count, 'gaps': self.gaps, 'clock_reversals': self.reversed,
                    'max_gap_s': self.max_gap, 'yaw_rad': self.angle, 'device_stamp_s': self.last, 'rate_rps': self.rate,
                    'vector_rate_rps': self.vector_rate, 'accel_mps2': self.accel,
                    'effective_bias_rps': self.bias[:], 'effective_yaw_axis': self.axis[:],
                    'ready': self.calibration is None or self.calibration.state in ('READY', 'DISABLED'),
                    'stationary_recalibration': {'updates': self.stationary_updates,
                        'collecting': self.stationary_candidate is not None and self.stationary_candidate.state == 'COLLECTING',
                        'reason': self.stationary_reason, 'result': self.stationary_result},
                    'calibration': self.calibration.status() if self.calibration else {'state': 'DISABLED'},
                    'age_s': None if self.arrival is None else t-self.arrival}

    def publish_fast(self, shared):
        """Only scalar copying on the gyro callback; no video or queue feeder."""
        with self.lock:
            ready = self.calibration is None or self.calibration.state in ('READY', 'DISABLED')
            values = [self.count, self.arrival or 0., self.last or 0., self.angle,
                      self.rate, self.gaps, self.reversed, float(ready)]
        with shared.get_lock():
            shared[:] = values

    def at(self, stamp):
        """Heading at image exposure time, without mixing callback/publication times."""
        with self.lock:
            h = list(self.history)
        return self._at(h, stamp)

    def rotation_samples(self, a, b):
        """Corrected gyro samples with interpolated exposure endpoints.

        Missing coverage, clock reversal and gaps are failures, never zero
        rotation. Startup/recalibration clears this separate short history.
        """
        if a is None or b is None or not all(math.isfinite(t) for t in (a, b)) or b <= a:
            return None, 'invalid_interval'
        with self.lock:
            if self.calibration and self.calibration.state not in ('READY', 'DISABLED'):
                return None, 'imu_not_ready'
            if self.reversed:
                return None, 'gyro_clock_reversal'
            h = list(self.gyro_vectors)
        times = [s[0] for s in h]
        i, j = bisect.bisect_right(times, a)-1, bisect.bisect_left(times, b)
        if i < 0 or j >= len(h) or j <= i:
            return None, 'gyro_interval_not_covered'
        segment = h[i:j+1]
        if any(not 0 < q[0]-p[0] <= self.cfg['max_gap_s'] for p, q in zip(segment, segment[1:])):
            return None, 'gyro_sample_gap'
        if any(not all(math.isfinite(v) for v in xyz) for _, xyz in segment):
            return None, 'nonfinite_gyro'
        def endpoint(t, p, q):
            u = (t-p[0])/(q[0]-p[0])
            return (t, tuple(x+u*(y-x) for x, y in zip(p[1], q[1])))
        first = endpoint(a, segment[0], segment[1])
        last = endpoint(b, segment[-2], segment[-1])
        return [first]+[s for s in segment if a < s[0] < b]+[last], 'covered'

    def _at(self, history, stamp):
        for p, q in zip(history, history[1:]):
            if p[0] <= stamp <= q[0] and q[0]-p[0] <= self.cfg['max_gap_s'] and p[3] == q[3]:
                u = (stamp-p[0])/(q[0]-p[0])
                return dict(yaw_rad=p[1]+u*(q[1]-p[1]), rate_rps=p[2]+u*(q[2]-p[2]),
                            gaps=p[3], device_stamp_s=stamp)
        return None

    def between(self, a, b):
        """Device-clock interpolation; missing coverage is NOT zero rotation."""
        with self.lock:
            h = list(self.history)
        x, y = self._at(h, a), self._at(h, b)
        return None if x is None or y is None or x['gaps'] != y['gaps'] else y['yaw_rad']-x['yaw_rad']


class Metrics:
    def __init__(self, capacity=20000):
        self.values, self.counts = {}, collections.Counter()
        self.capacity = capacity

    def add(self, key, value):
        if value is not None and math.isfinite(value):
            self.values.setdefault(key, collections.deque(maxlen=self.capacity)).append(value)

    def report(self):
        out = {'counts': dict(self.counts), 'quantiles_window_max_samples': self.capacity}
        for key, values in self.values.items():
            v = sorted(values)
            def q(p):
                return v[min(len(v)-1, int((len(v)-1)*p))]
            out[key] = {'n': len(v), 'p50': q(.5), 'p95': q(.95), 'p99': q(.99), 'max': v[-1]}
        return out
