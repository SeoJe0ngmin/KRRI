"""Stationary IMU calibration; observations are not accuracy certificates."""
import collections
import math

def norm(v):
    return math.sqrt(sum(x*x for x in v))


class Moments:
    def __init__(self):
        self.n = 0
        self.mean, self.m2 = [0.]*3, [0.]*3

    def add(self, values):
        self.n += 1
        for i, value in enumerate(values):
            d = value-self.mean[i]
            self.mean[i] += d/self.n
            self.m2[i] += d*(value-self.mean[i])

    def std(self):
        return [math.sqrt(x/max(1, self.n-1)) for x in self.m2]


class StationaryCalibration:
    def __init__(self, cfg):
        self.imu, self.cfg = cfg, cfg['startup_calibration']
        self.enabled = self.cfg['enabled']
        self.state = 'COLLECTING' if self.enabled else 'DISABLED'
        self.reason = 'keep_stationary' if self.enabled else 'configured_bias'
        self.first = self.start = None
        self.last = {'gyro': None, 'accel': None}
        self.gyro, self.accel = Moments(), Moments()
        self.gyro_window = collections.deque()
        self.window_means = Moments()
        self.motion_window_s = self.cfg.get('motion_window_s', 0.)
        self.restarts = self.duplicates = 0
        self.result = None

    def reset_window(self, reason):
        self.start = None
        self.gyro, self.accel = Moments(), Moments()
        self.gyro_window.clear()
        self.window_means = Moments()
        self.reason = reason
        self.restarts += 1

    def gyro_motion(self, stamp, xyz):
        """Optional window gate; bias and integration still use raw samples.

        A small raw peak is not by itself proof of motion. Check sustained
        rate, raw peak, raw variance and variability of window means separately.
        None of these distinguish a small constant turn from a constant bias;
        the operator's stationary setup remains a prerequisite.
        """
        residual = [x-b for x, b in zip(xyz, self.imu['gyro_bias_rps'])]
        if not self.motion_window_s:
            return norm(residual) > math.radians(self.cfg['moving_dps'])
        if norm(residual) > math.radians(self.cfg['max_gyro_peak_dps']):
            return True
        self.gyro_window.append((stamp, residual))
        cutoff = stamp-self.motion_window_s
        # Retain the sample immediately before the cutoff to establish coverage.
        while len(self.gyro_window) > 1 and self.gyro_window[1][0] <= cutoff:
            self.gyro_window.popleft()
        if self.gyro_window[0][0] > cutoff+1e-9:
            return False
        mean = [sum(v[i] for _, v in self.gyro_window)/len(self.gyro_window) for i in range(3)]
        if norm(mean) > math.radians(self.cfg['moving_dps']):
            return True
        self.window_means.add(mean)
        return False

    def add(self, kind, stamp, xyz):
        if self.state != 'COLLECTING':
            return
        if not math.isfinite(stamp) or len(xyz) != 3 or not all(math.isfinite(x) for x in xyz):
            self.reset_window('nonfinite_sample')
            return
        prev = self.last[kind]
        if prev is not None and stamp <= prev:
            self.duplicates += 1
            if stamp < prev:
                self.state, self.reason = 'FAILED', 'device_clock_reversed'
            return
        self.last[kind] = stamp
        if self.first is None:
            self.first = stamp
        if stamp-self.first > self.cfg['timeout_s']:
            self.state, self.reason = 'FAILED', 'stationary_calibration_timeout'
            return
        if prev is not None and stamp-prev > self.cfg['max_gap_s']:
            self.reset_window('sample_gap')
        if kind == 'gyro':
            if self.gyro_motion(stamp, xyz):
                self.reset_window('motion_during_calibration')
                return
        elif abs(norm(xyz)-self.cfg['gravity_mps2']) > self.cfg['gravity_tolerance_mps2']:
            self.reset_window('acceleration_during_calibration')
            return
        if self.start is None:
            self.start = stamp
        (self.gyro if kind == 'gyro' else self.accel).add(xyz)
        duration = stamp-self.start
        if duration < self.cfg['duration_s']:
            return
        minimum = self.cfg['min_sample_ratio']
        if (self.gyro.n < self.imu['gyro_hz']*duration*minimum or
                self.accel.n < self.imu['accel_hz']*duration*minimum):
            self.reason = 'insufficient_unique_samples'
            return
        if any(stamp-self.last[k] > self.cfg['max_gap_s'] for k in ('gyro', 'accel')):
            self.reason = 'sensor_stream_stale'
            return
        if norm(self.gyro.std()) > math.radians(self.cfg['max_gyro_std_dps']):
            self.reset_window('gyro_not_quiet')
            return
        if self.motion_window_s:
            if self.window_means.n < 2:
                self.reason = 'insufficient_gyro_windows'
                return
            if norm(self.window_means.std()) > math.radians(self.cfg['max_gyro_window_std_dps']):
                self.reset_window('gyro_mean_not_stable')
                return
        if norm(self.accel.std()) > self.cfg['max_accel_std_mps2']:
            self.reset_window('accel_not_quiet')
            return
        gravity = [x/norm(self.accel.mean) for x in self.accel.mean]
        dot = sum(a*b for a, b in zip(gravity, self.imu['yaw_axis']))
        if abs(dot) < math.cos(math.radians(self.cfg['max_axis_tilt_deg'])):
            self.state, self.reason = 'FAILED', 'configured_yaw_axis_inconsistent_with_gravity'
            return
        axis = [x*(1 if dot >= 0 else -1) for x in gravity] if self.cfg['align_axis_to_gravity'] else self.imu['yaw_axis'][:]
        self.result = {'gyro_bias_rps': self.gyro.mean[:], 'yaw_axis': axis,
                       'gravity_mean_mps2': self.accel.mean[:], 'gyro_std_rps': self.gyro.std(),
                       'accel_std_mps2': self.accel.std(), 'duration_s': duration,
                       'gyro_samples': self.gyro.n, 'accel_samples': self.accel.n,
                       'yaw_sign_verified': self.imu.get('yaw_sign_verified', False)}
        if self.motion_window_s:
            self.result.update(motion_window_s=self.motion_window_s,
                               gyro_window_std_rps=self.window_means.std())
        self.state, self.reason = 'READY', 'stationary_bias_measured'

    def status(self):
        return {'state': self.state, 'reason': self.reason, 'restarts': self.restarts,
                'gyro_samples': self.gyro.n, 'accel_samples': self.accel.n,
                'duplicates': self.duplicates, 'result': self.result}
