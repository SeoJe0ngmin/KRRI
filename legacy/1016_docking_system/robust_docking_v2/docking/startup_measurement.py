"""Per-run stationary observations before any search or mission motion.

These measurements describe the current stationary setup. They do not certify
absolute accuracy, braking response, transport latency, or moving VO drift.
The operator must hold the vehicle stationary; quiet IMU alone is not proof.
"""
import copy
import math
from collections import Counter

from .core import finite
from .motion import stopped


def validate_startup_measurement(cfg):
    c = cfg.get('startup_measurement', {})
    if not isinstance(c, dict) or not isinstance(c.get('enabled', False), bool):
        raise ValueError('invalid startup_measurement.enabled')
    if not c.get('enabled'):
        return
    for key in ('duration_s', 'timeout_s', 'max_sample_gap_s'):
        value = c.get(key)
        if isinstance(value, bool) or not finite([value]) or value <= 0:
            raise ValueError('invalid startup_measurement.'+key)
    if not c['max_sample_gap_s'] < c['duration_s'] < c['timeout_s']:
        raise ValueError('startup measurement needs gap < duration < timeout')


class Series:
    """Constant memory statistics; no fabricated samples when data is absent."""
    def __init__(self, width):
        self.width, self.n = width, 0
        self.mean, self.m2 = [0.]*width, [0.]*width
        self.first = self.last = self.low = self.high = None

    def add(self, values):
        if not isinstance(values, (list, tuple)) or len(values) != self.width or not finite(values):
            return
        values = list(values)
        if self.n == 0:
            self.first, self.low, self.high = values[:], values[:], values[:]
        self.last, self.n = values, self.n+1
        for i, value in enumerate(values):
            delta = value-self.mean[i]
            self.mean[i] += delta/self.n
            self.m2[i] += delta*(value-self.mean[i])
            self.low[i], self.high[i] = min(self.low[i], value), max(self.high[i], value)

    def report(self):
        if not self.n:
            return {'samples': 0}
        return dict(samples=self.n, mean=self.mean[:],
                    std=[math.sqrt(max(0., v)/max(1, self.n-1)) for v in self.m2],
                    min=self.low[:], max=self.high[:],
                    last_minus_first=[b-a for a, b in zip(self.first, self.last)])


class StartupMeasurement:
    def __init__(self, cfg):
        validate_startup_measurement(cfg)
        self.cfg, self.c = cfg, cfg.get('startup_measurement', {})
        self.enabled = self.c.get('enabled', False)
        self.complete = not self.enabled
        self.state = 'STARTUP_WAIT' if self.enabled else 'DISABLED'
        self.reason = 'keep_vehicle_stationary'
        self.started = self.now = self.last_capture = self.last_host = None
        self.generation = self.imu_gaps = None
        self.resets = Counter()
        self._clear_window()

    def _clear_window(self):
        self.first_host = self.first_capture = self.window_last = None
        self.samples = 0
        self.tag_reasons = Counter()
        self.tag_position = Series(3)
        self.local_xy = Series(2)
        self.gyro_rate = Series(1)
        self.accel = Series(3)
        self.imu_yaw = Series(1)
        self.processing_ms = Series(1)
        self.result_age_ms = Series(1)
        self.performance = self.imu_result = None
        self.can_diagnostics = None

    def reset(self, reason):
        if self.first_host is not None:
            self.resets[reason] += 1
            self._clear_window()
        self.state, self.reason = 'STARTUP_WAIT', reason

    def fail(self, reason):
        self.state, self.reason = 'FAULT', reason

    def status(self):
        span = (min(self.now-self.first_host, self.window_last-self.first_capture)
                if self.first_host is not None else 0.)
        return dict(enabled=self.enabled, complete=self.complete, state=self.state,
                    reason=self.reason, required_s=self.c.get('duration_s', 0.),
                    stationary_s=max(0., span), samples=self.samples,
                    resets=dict(self.resets), command='stop')

    def step(self, now, obs, angular_ok=None, can_status=None):
        if self.complete or self.state == 'FAULT':
            return
        if not finite([now]) or (self.now is not None and now < self.now):
            self.fail('startup_measurement_host_clock_reversed')
            return
        self.now = now
        if self.started is None:
            self.started = now
        if now-self.started >= self.c['timeout_s']:
            self.fail('startup_measurement_timeout')
            return
        if obs is None:
            self.reset('waiting_for_camera')
            return
        imu = obs.get('imu', {})
        if obs.get('clock_fault') or imu.get('clock_reversals'):
            self.fail('startup_measurement_clock_fault')
            return
        if imu.get('calibration', {}).get('state') == 'FAILED':
            self.fail('imu_startup_calibration_failed')
            return
        stamp, lower = obs.get('capture_id'), obs.get('capture_lower')
        if finite([stamp]) and self.last_capture is not None and stamp < self.last_capture:
            self.fail('startup_measurement_capture_clock_reversed')
            return
        limits = self.cfg['limits']
        if (not finite([stamp, lower]) or not 0 <= now-lower <= limits['max_age_s'] or
                not obs.get('sensor_ready') or obs.get('imu_bad') or not imu.get('ready') or
                imu.get('calibration', {}).get('state') != 'READY'):
            self.reset('waiting_for_fresh_calibrated_sensors')
            return
        age, age_limit = imu.get('age_s'), self.cfg['sensor']['imu'].get('max_age_s')
        if not finite([age, age_limit]) or not 0 <= age <= age_limit:
            self.reset('waiting_for_fresh_imu')
            return
        if (not obs.get('odom', {}).get('ok') or
                not stopped(obs, now, limits, angular_ok)):
            self.reset('stationary_evidence_unavailable')
            return
        if stamp == self.last_capture:
            return  # Replayed frames/time passing must not complete the window.
        generation, gaps = obs.get('tracking_generation'), imu.get('gaps')
        if self.last_capture is not None and (
                stamp-self.last_capture > self.c['max_sample_gap_s'] or
                now-self.last_host > self.c['max_sample_gap_s'] or
                generation != self.generation or gaps != self.imu_gaps):
            self.reset('sensor_gap_or_tracking_reset')
        self.last_capture, self.generation, self.imu_gaps = stamp, generation, gaps
        self.last_host = now
        if self.first_host is None:
            self.first_host, self.first_capture = now, stamp
        self.window_last = stamp
        self.samples += 1
        self.state, self.reason = 'STARTUP_MEASURE', 'collecting_stationary_measurements'
        tag = obs.get('tag', {})
        self.tag_reasons[tag.get('reason', 'unavailable')] += 1
        if tag.get('ok') and tag.get('id') == self.cfg['sensor']['tag']['id']:
            self.tag_position.add(tag.get('camera_translation_m'))
        self.local_xy.add(obs.get('local_motion', {}).get('xy'))
        self.gyro_rate.add([imu.get('rate_rps')])
        self.imu_yaw.add([imu.get('yaw_rad')])
        self.accel.add(imu.get('accel_mps2'))
        self.performance = copy.deepcopy(obs.get('performance', {}))
        self.processing_ms.add([self.performance.get('arrival_to_result_ms')])
        self.result_age_ms.add([1000*(now-lower)])
        self.imu_result = copy.deepcopy(imu)
        if can_status is not None:
            self.can_diagnostics = copy.deepcopy(can_status.get('diagnostics'))
        if self.status()['stationary_s'] >= self.c['duration_s']:
            self.complete, self.state, self.reason = True, 'STARTUP_READY', 'stationary_measurement_complete'

    def report(self):
        return dict(schema=1, status=self.status(),
                    elapsed_s=0. if self.started is None else self.now-self.started,
                    config_changed=False, certified_error_bound=False,
                    operator_must_hold_stationary=True,
                    sample_source='unique video observations received by supervisor; IMU snapshots are not raw 200 Hz samples',
                    tag_reasons=dict(self.tag_reasons), tag_camera_position_m=self.tag_position.report(),
                    local_xy_m=self.local_xy.report(), gyro_rate_snapshot_rps=self.gyro_rate.report(),
                    accel_snapshot_mps2=self.accel.report(), imu_yaw_snapshot_rad=self.imu_yaw.report(),
                    callback_to_result_ms=self.processing_ms.report(),
                    capture_to_supervisor_age_upper_ms=self.result_age_ms.report(),
                    performance_at_end=copy.deepcopy(self.performance), imu_at_end=copy.deepcopy(self.imu_result),
                    can_diagnostics_at_end=copy.deepcopy(self.can_diagnostics),
                    notes=['Tag scatter is repeatability, not absolute accuracy.',
                           'Local displacement is observed stationary drift, not a moving odometry bound.',
                           'CAN notification delay is not vehicle actuation latency.',
                           'Missing tag data stays missing; normal tag acquisition still follows this gate.'])
