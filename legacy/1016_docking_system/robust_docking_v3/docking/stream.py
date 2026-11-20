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

    def observe_stationary(self, stamp, eligible):
        """Re-estimate bias using IMU samples covered by fresh visual stillness.

        Does not freeze/reset integrated angle, clear history, or reduce any
        absolute pose/error bound. Each update is relative to the startup bias
        envelope, preventing repeated small updates from ratcheting unchecked.
        """
        c = self.cfg.get('stationary_recalibration', {})
        with self.lock:
            if not c.get('enabled'):
                return
            ready = self.calibration is None or self.calibration.state == 'READY'
            if not c.get('verified') or not ready or not eligible:
                self.stationary_candidate = None
                self.stationary_frame = self.stationary_cursor = None
                self.stationary_reason = 'unverified_or_not_visually_stationary'
                return
            if self.stationary_frame is not None and stamp <= self.stationary_frame:
                return  # Duplicate imagery cannot supply new evidence.
            gap = self.stationary_frame is None or stamp-self.stationary_frame > c['max_visual_gap_s']
            self.stationary_frame = stamp
            if self.stationary_candidate is None or gap:
                cfg = copy.deepcopy(self.cfg)
                cfg['gyro_bias_rps'] = self.bias[:]
                cfg['startup_calibration'].update(enabled=True, duration_s=c['duration_s'],
                    timeout_s=c['duration_s']*4, align_axis_to_gravity=False)
                self.stationary_candidate = StationaryCalibration(cfg)
                self.stationary_cursor = stamp
                self.stationary_reason = 'collecting_visually_stationary_imu'
                return
            samples = sorted((s for s in self.raw_history if self.stationary_cursor < s[1] <= stamp), key=lambda s: s[1])
            candidate = self.stationary_candidate
            for kind, t, xyz in samples:
                candidate.add(kind, t, xyz)
            self.stationary_cursor = stamp
            if candidate.state == 'FAILED':
                self.stationary_candidate = None
                self.stationary_reason = candidate.reason
                return
            if candidate.state != 'READY':
                self.stationary_reason = candidate.reason
                return
            new_bias = candidate.result['gyro_bias_rps']
            if self.stationary_initial_bias is None:
                self.stationary_initial_bias = self.bias[:]
            change = math.sqrt(sum((a-b)**2 for a, b in zip(new_bias, self.stationary_initial_bias)))
            self.stationary_candidate = None
            if change > c['max_bias_change_rps']:
                self.stationary_reason = 'bias_change_outside_validated_envelope'
                return
            self.bias = new_bias[:]
            # Do not construct an image interval across a bias change.
            self.gyro_vectors.clear()
            # Preserve past integrated angles. Only the rate used for future
            # integration changes at this boundary, including trapezoid start.
            if self.raw_last is not None:
                self.rate = sum((v-b)*a for v, b, a in zip(self.raw_last, self.bias, self.axis))*self.cfg['gyro_scale']
                self.vector_rate = math.sqrt(sum((v-b)**2 for v, b in zip(self.raw_last, self.bias)))*abs(self.cfg['gyro_scale'])
            self.stationary_updates += 1
            self.stationary_result = dict(candidate.result, applied_at_device_s=self.last)
            self.stationary_reason = 'bias_updated_heading_and_uncertainty_preserved'

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
                return dict(yaw_rad=p[1]+u*(q[1]-p[1]), gaps=p[3], device_stamp_s=stamp)
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
