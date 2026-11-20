"""Finite-window diagnostics, never used as certified pose-error bounds."""
import collections
from .calibration import Moments


class StillnessReport:
    def __init__(self, cfg):
        self.cfg = cfg
        self.samples = collections.deque(maxlen=cfg.get('max_samples', 6000))
        self.first = self.last = None
        self.cached = None

    def add(self, stamp, position):
        if self.last is not None and stamp <= self.last:
            return
        self.last = stamp
        if self.first is None:
            self.first = stamp
        self.samples.append((stamp, tuple(position)))
        while self.samples and stamp-self.samples[0][0] > self.cfg.get('window_s', 60.):
            self.samples.popleft()

    def report(self):
        m = Moments()
        for _, xyz in self.samples:
            m.add(xyz)
        span = self.samples[-1][0]-self.samples[0][0] if self.samples else 0.
        return {'n': m.n, 'span_s': span, 'mean_m': m.mean if m.n else None,
                'std_m': m.std() if m.n > 1 else None,
                'operator_must_hold_stationary': True,
                'certified_error_bound': False}


def validate_sensor_options(sensor):
    import math
    def positive(v, name):
        if isinstance(v, bool) or not isinstance(v, (int, float)) or not math.isfinite(v) or v <= 0:
            raise ValueError('invalid sensor option: '+name)
    tag = sensor['tag']
    positive(tag.get('min_tag_px', 20), 'min_tag_px')
    sigma = tag.get('quad_sigma', 0)
    if isinstance(sigma, bool) or not isinstance(sigma, (int, float)) or not math.isfinite(sigma) or sigma < 0:
        raise ValueError('quad_sigma must be nonnegative')
    log = sensor.get('logging', {})
    for key, default in (('queue_size', 4096), ('flush_s', 1.), ('max_bytes', 500*1024*1024)):
        positive(log.get(key, default), 'logging.'+key)
    if not isinstance(log.get('queue_size', 4096), int):
        raise ValueError('logging.queue_size must be an integer')
    diagnostics = sensor.get('diagnostics', {})
    positive(diagnostics.get('window_s', 60.), 'diagnostics.window_s')
    positive(diagnostics.get('max_samples', 6000), 'diagnostics.max_samples')
    if not isinstance(diagnostics.get('max_samples', 6000), int):
        raise ValueError('diagnostics.max_samples must be an integer')
    if diagnostics.get('reject_rotational_blur_px') is not None:
        positive(diagnostics['reject_rotational_blur_px'], 'diagnostics.reject_rotational_blur_px')
    cal = sensor['imu'].get('startup_calibration')
    if cal and cal.get('enabled'):
        for key in ('duration_s', 'timeout_s', 'min_sample_ratio', 'max_gap_s', 'moving_dps',
                    'gravity_mps2', 'gravity_tolerance_mps2', 'max_gyro_std_dps',
                    'max_accel_std_mps2', 'max_axis_tilt_deg'):
            positive(cal.get(key), 'startup_calibration.'+key)
        if cal['min_sample_ratio'] > 1 or cal['duration_s'] >= cal['timeout_s'] or cal['max_axis_tilt_deg'] >= 90:
            raise ValueError('invalid startup calibration window/ratio/axis limit')
        if 'motion_window_s' in cal:
            for key in ('motion_window_s', 'max_gyro_peak_dps', 'max_gyro_window_std_dps'):
                positive(cal.get(key), 'startup_calibration.'+key)
            if (cal['motion_window_s'] >= cal['duration_s'] or
                    cal['motion_window_s']*sensor['imu']['gyro_hz'] < 2 or
                    cal['max_gyro_peak_dps'] < cal['moving_dps']):
                raise ValueError('invalid startup calibration gyro motion window/peak')
    axis = sensor['imu']['yaw_axis']
    if len(axis) != 3 or not all(isinstance(v, (int, float)) and not isinstance(v, bool) and math.isfinite(v) for v in axis) or abs(sum(v*v for v in axis)-1) > 1e-5:
        raise ValueError('IMU yaw_axis must be a unit vector')
