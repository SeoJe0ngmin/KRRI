"""One-shot, pre-mission tag acquisition. Never used for in-mission tag loss.

Before the tag is found, dock-relative free-space checks are unavailable.
Search therefore requires a separately qualified free disk around the initial
body reference, including full body rotation, drift and STOP/coast. Relative
vision/IMU are required to monitor motion; this is not a timed blind spin.
"""
import math
from .core import Box, finite, safe
from .recovery import within_budget
from .feedback import imu_ready
from .motion import AngularStopGate, stopped as measured_stop


def validate_startup(cfg, live=False):
    c = cfg.get('startup_search', {})
    if not c.get('enabled'):
        return
    for k in ('timeout_s', 'initial_wait_s', 'stable_s', 'max_sample_gap_s',
              'min_missing_samples', 'min_tag_samples', 'pose_error_ratio',
              'max_sweep_rad', 'max_rotation_s', 'max_start_delay_s',
              'max_stop_response_s', 'brake_reserve_rad', 'max_center_drift_m',
              'stop_drift_reserve_m', 'free_radius_m'):
        if not finite([c.get(k)]) or isinstance(c[k], bool) or c[k] <= 0:
            raise ValueError('missing/invalid startup_search.'+k)
    for k in ('min_missing_samples', 'min_tag_samples'):
        if not isinstance(c[k], int) or c[k] < 2:
            raise ValueError('startup search requires multiple unique observations: '+k)
    if c.get('direction') not in ('left', 'right'):
        raise ValueError('startup_search.direction must be left/right')
    if not 0 < c['pose_error_ratio'] <= 1 or c['stable_s'] >= c['timeout_s']:
        raise ValueError('invalid startup measurement window')
    if not c['brake_reserve_rad'] < c['max_sweep_rad'] <= 2*math.pi:
        raise ValueError('startup search allows at most one full turn including stop reserve')
    if c['stop_drift_reserve_m'] >= c['max_center_drift_m']:
        raise ValueError('startup drift budget must exceed STOP reserve')
    b = cfg['body']
    radius = math.hypot(max(b['front_m'], b['rear_m']), b['half_width_m'])
    if radius+c['max_center_drift_m']+cfg['scene']['clearance_m'] > c['free_radius_m']:
        raise ValueError('startup free disk does not contain full body rotation/drift')
    if live and not cfg.get('validation', {}).get('startup_search'):
        raise ValueError('missing validation evidence: startup_search')


    improve = c.get('improvement', {})
    if improve.get('enabled'):
        for key in ('retry_wait_s', 'max_attempts', 'center_tolerance_rad', 'max_center_turn_rad',
                    'max_saturated_fraction', 'max_dark_fraction'):
            if not finite([improve.get(key)]) or isinstance(improve[key], bool) or improve[key] <= 0:
                raise ValueError('invalid startup improvement.'+key)
        if not isinstance(improve['max_attempts'], int) or improve['max_attempts'] > 20:
            raise ValueError('invalid startup improvement attempt budget')
        if not improve['center_tolerance_rad'] < improve['max_center_turn_rad'] < math.pi/2:
            raise ValueError('invalid startup centering angles')
        if max(improve['max_saturated_fraction'], improve['max_dark_fraction']) > 1:
            raise ValueError('invalid startup lighting fractions')


class StartupSearch:
    def __init__(self, cfg):
        self.cfg, self.c = cfg, cfg.get('startup_search', {})
        self.complete = not self.c.get('enabled', False)
        self.state, self.command, self.reason = 'INITIAL_WAIT', 'stop', 'initial_sensor_readiness'
        self.started = self.last_capture = self.still_since = None
        self.ready_since = None
        self.missing_samples = self.tag_samples = 0
        self.ever_saw_tag = False
        self.scan_started = self.motion_started = self.braked = None
        self.scan_yaw = self.scan_stamp = self.scan_local = self.scan_generation = None
        self.last_imu_stamp = None
        self.scan_gaps = None
        self.angle_upper = self.drift_upper = 0.
        self.stable_first = self.stable_last = self.stable_box = None
        self.last_tag_stamp = self.measure_after = None
        self.improvement_since = None
        self.retries = 0
        self.exposure_request = 0
        self.scan_direction = self.c.get('direction', 'left')
        self.leg_yaw = self.leg_target = self.last_scan_yaw = None
        self.sweep_travel = 0.
        self.angular_stop = AngularStopGate(cfg)

    def status(self):
        return {'complete': self.complete, 'state': self.state, 'reason': self.reason,
                'command': self.command, 'ever_saw_tag': self.ever_saw_tag,
                'rotation_started': self.scan_started is not None,
                'angle_upper_rad': self.angle_upper, 'center_drift_upper_m': self.drift_upper,
                'tag_samples': self.tag_samples, 'missing_samples': self.missing_samples,
                'retries': self.retries, 'exposure_request': self.exposure_request}

    def fault(self, reason):
        self.state, self.command, self.reason = 'FAULT', 'stop', 'initial_'+reason

    def reset_measurement(self):
        self.stable_first = self.stable_last = self.stable_box = None
        self.last_tag_stamp = None
        self.tag_samples = 0

    def brake(self, now, stamp, reason):
        self.state, self.command, self.reason = 'INITIAL_BRAKING', 'stop', reason
        self.braked, self.measure_after, self.still_since = now, stamp, None
        self.reset_measurement()

    def unavailable(self):
        self.still_since = self.ready_since = None
        self.missing_samples = 0
        self.reset_measurement()
        if self.state in ('INITIAL_SEARCH', 'INITIAL_CENTERING'):
            self.fault('search_sensor_unavailable')
        else:
            self.command, self.reason = 'stop', 'initial_waiting_for_fresh_sensors'

    def step(self, now, obs, fast_imu=None, fast_imu_required=False):
        if self.complete or self.state == 'FAULT':
            return
        c, lim = self.c, self.cfg['limits']
        angular_ok = self.angular_stop.update(fast_imu, fast_imu_required)
        if angular_ok is False:
            self.still_since = self.ready_since = None
            self.missing_samples = 0
            self.reset_measurement()
        if self.started is None:
            self.started = now
        if now-self.started > c['timeout_s']:
            self.fault('acquisition_timeout')
            return
        if self.state in ('INITIAL_SEARCH', 'INITIAL_CENTERING'):
            if self.motion_started is None and now-self.scan_started > c['max_start_delay_s']:
                self.fault('rotation_start_timeout')
                return
            if self.motion_started is not None and now-self.motion_started > c['max_rotation_s']:
                self.fault('rotation_timeout')
                return
        if self.state == 'INITIAL_BRAKING' and now-self.braked > c['max_stop_response_s']+lim['settle_timeout_s']:
            self.fault('stop_not_observed')
            return
        if obs is None:
            self.unavailable()
            return
        imu = obs.get('imu', {})
        if obs.get('clock_fault') or imu.get('clock_reversals', 0) or (fast_imu or {}).get('clock_reversals', 0):
            self.fault('clock_reversed')
            return
        if imu.get('calibration', {}).get('state') == 'FAILED':
            self.fault('imu_calibration_failed')
            return
        if fast_imu_required:
            if not imu_ready(fast_imu, self.cfg):
                self.unavailable()
                return
            imu = dict(imu, **fast_imu)
        stamp, lower = obs.get('capture_id'), obs.get('capture_lower')
        if finite([stamp]) and self.last_capture is not None and stamp < self.last_capture:
            self.fault('clock_reversed')
            return
        if finite([lower]) and lower > now:
            self.fault('capture_clock_invalid')
            return
        if (not finite([stamp, lower]) or now-lower > lim['max_age_s'] or
                not obs.get('sensor_ready') or obs.get('imu_bad') or not imu.get('ready') or
                imu.get('calibration', {}).get('state') != 'READY' or
                not finite([imu.get('yaw_rad'), imu.get('device_stamp_s'), imu.get('age_s')]) or
                not 0 <= imu['age_s'] <= self.cfg['sensor']['imu']['max_age_s']):
            self.unavailable()
            return
        if self.scan_started is not None and self.state in ('INITIAL_SEARCH', 'INITIAL_CENTERING', 'INITIAL_BRAKING'):
            if ((self.last_imu_stamp is not None and imu['device_stamp_s'] < self.last_imu_stamp) or
                    imu.get('gaps', 0) != self.scan_gaps):
                self.fault('imu_clock_or_gap')
                return
            self.last_imu_stamp = imu['device_stamp_s']
            dt = imu['device_stamp_s']-self.scan_stamp
            gyro_error = self.cfg['sensor']['bounds']['gyro_rate_error_rps']*dt
            delta = imu['yaw_rad']-self.leg_yaw
            if self.last_scan_yaw is not None:
                self.sweep_travel += abs(imu['yaw_rad']-self.last_scan_yaw)
            self.last_scan_yaw = imu['yaw_rad']
            self.angle_upper = self.sweep_travel+gyro_error
            direction = 1 if self.scan_direction == 'left' else -1
            if self.state != 'INITIAL_BRAKING' and direction*delta+gyro_error < 0:
                self.fault('wrong_rotation_direction')
                return
            if self.motion_started is None and direction*delta-gyro_error > 0:
                self.motion_started = now
            if self.angle_upper > c['max_sweep_rad']:
                self.fault('rotation_envelope_exceeded')
                return
            if self.state in ('INITIAL_SEARCH', 'INITIAL_CENTERING') and self.angle_upper >= c['max_sweep_rad']-c['brake_reserve_rad']:
                self.fault('search_budget_exhausted')
                return
        if self.state == 'INITIAL_CENTERING' and (abs(imu['yaw_rad']-self.leg_yaw)+gyro_error+
                lim['max_yaw_rate_rps']*imu['age_s'] >= self.leg_target-c['brake_reserve_rad']):
            self.brake(now, stamp, 'initial_centering_complete')
            return
        if stamp == self.last_capture:
            return
        # Missing frames cannot count as sustained stillness or tag absence.
        if self.last_capture is not None and stamp-self.last_capture > c['max_sample_gap_s']:
            self.still_since = self.ready_since = None
            self.missing_samples = 0
            self.reset_measurement()
            if self.state in ('INITIAL_SEARCH', 'INITIAL_CENTERING'):
                self.fault('search_frame_gap')
                return
        self.last_capture = stamp
        if self.last_imu_stamp is not None and imu['device_stamp_s'] < self.last_imu_stamp:
            self.fault('imu_clock_reversed')
            return
        self.last_imu_stamp = imu['device_stamp_s']
        tag = obs.get('tag', {})
        tag_ok = bool(tag.get('ok') and tag.get('id') == self.cfg['sensor']['tag']['id'])
        tag_seen = bool(tag.get('detected', tag.get('ok')) and tag.get('id') == self.cfg['sensor']['tag']['id'])
        local = obs.get('local_motion', {})
        local_ok = (obs.get('odom', {}).get('ok') and local.get('bounded') and
                    isinstance(local.get('xy'), list) and len(local['xy']) == 2 and finite(local['xy']) and
                    isinstance(local.get('error_xy'), list) and len(local['error_xy']) == 2 and
                    finite(local['error_xy']) and min(local['error_xy']) >= 0)
        if self.scan_started is not None and self.state in ('INITIAL_SEARCH', 'INITIAL_CENTERING', 'INITIAL_BRAKING'):
            if not local_ok or obs.get('tracking_generation') != self.scan_generation:
                self.fault('relative_tracking_lost')
                return
            displacement = math.hypot(*(a-b for a, b in zip(local['xy'], self.scan_local['xy'])))
            error = math.hypot(*(a+b for a, b in zip(local['error_xy'], self.scan_local['error_xy'])))
            self.drift_upper = displacement+error+lim['max_speed_mps']*(now-lower)
            if self.drift_upper > c['max_center_drift_m'] or self.angle_upper > c['max_sweep_rad']:
                self.fault('rotation_envelope_exceeded')
                return
        # Any decoded target detection ends search immediately. A one-frame sighting
        # is enough to STOP, but cannot establish the initial mission pose.
        if tag_seen and self.state in ('INITIAL_WAIT', 'INITIAL_SEARCH'):
            self.ever_saw_tag = True
            self.brake(now, stamp, 'initial_tag_found_stop')
            return
        if self.state in ('INITIAL_SEARCH', 'INITIAL_CENTERING'):
            if (self.angle_upper >= c['max_sweep_rad']-c['brake_reserve_rad'] or
                    self.drift_upper >= c['max_center_drift_m']-c['stop_drift_reserve_m']):
                self.fault('search_budget_exhausted')
            return
        stopped = measured_stop(obs, now, lim, angular_ok)
        if not stopped:
            self.still_since = None
            if self.state == 'INITIAL_WAIT':
                self.ready_since, self.missing_samples = None, 0
            self.reset_measurement()
            return
        if self.still_since is None:
            self.still_since = now
        if self.state == 'INITIAL_WAIT':
            if self.ready_since is None:
                self.ready_since = now
            if tag.get('reason') not in (None, 'not_scheduled'):
                self.missing_samples += 1
            if (now-self.still_since < lim['stop_dwell_s'] or now-self.ready_since < c['initial_wait_s'] or
                    self.missing_samples < c['min_missing_samples'] or not local_ok):
                self.reason = 'initial_confirming_stop_and_tag_absence'
                return
            initial_error = 2*math.hypot(*local['error_xy'])+lim['max_speed_mps']*(now-lower)
            if initial_error >= c['max_center_drift_m']-c['stop_drift_reserve_m']:
                self.reason = 'initial_relative_position_error_too_large'
                return
            self.scan_yaw, self.scan_stamp = imu['yaw_rad'], imu['device_stamp_s']
            self.leg_yaw = self.last_scan_yaw = imu['yaw_rad']
            self.scan_gaps = imu.get('gaps', 0)
            self.scan_local, self.scan_generation = local, obs.get('tracking_generation')
            if self.scan_generation is None:
                return
            self.scan_started, self.motion_started = now, None
            self.state, self.reason = 'INITIAL_SEARCH', 'initial_searching_for_tag'
            self.command = 'rotate_left_slow' if c['direction'] == 'left' else 'rotate_right_slow'
            return
        if now-self.still_since < lim['stop_dwell_s']:
            return
        self.state, self.command = 'INITIAL_MEASURE', 'stop'
        if self._improve(now, obs, imu, tag_seen, tag_ok, local_ok):
            return
        if not obs.get('valid') or not obs.get('bounded'):
            self.reset_measurement()
            self.reason = 'initial_waiting_for_absolute_tag_pose'
            return
        age = now-lower
        b = Box.around(obs['pose'], obs['error']).inflate((lim['max_speed_mps']*age,
                    lim['max_speed_mps']*age, lim['max_yaw_rate_rps']*age))
        if not safe(b, self.cfg):
            self.fault('pose_outside_free_space')
            return
        if not within_budget(b, self.cfg, c['pose_error_ratio']):
            self.reset_measurement()
            self.reason = 'initial_waiting_for_smaller_pose_error'
            return
        if (self.stable_last is None or stamp-self.stable_last > c['max_sample_gap_s'] or
                self.stable_box.intersection(b) is None):
            self.reset_measurement()
            self.stable_first, self.stable_box = stamp, b
        else:
            self.stable_box = self.stable_box.intersection(b)  # Consistency only.
        self.stable_last = stamp
        anchor = obs.get('anchor_stamp')
        if tag_ok and finite([anchor]) and anchor > self.measure_after and anchor != self.last_tag_stamp:
            self.tag_samples += 1
            self.last_tag_stamp = anchor
        self.reason = 'initial_measuring_fresh_tag_pose'
        if self.tag_samples < c['min_tag_samples'] or stamp-self.stable_first < c['stable_s']:
            return
        # Do not finish on an old tag followed by an indefinitely propagated pose.
        if self.last_tag_stamp is None or stamp-self.last_tag_stamp > c['max_sample_gap_s']:
            return
        recal = imu.get('stationary_recalibration', {})
        applied = (recal.get('result') or {}).get('applied_at_device_s')
        if recal.get('collecting') and (applied is None or applied <= self.measure_after):
            self.reason = 'initial_waiting_for_stationary_gyro_calibration'
            return
        self.complete = True
        self.state, self.reason = 'INITIAL_READY', 'initial_pose_and_stop_verified'


    def _improve(self, now, obs, imu, tag_seen, tag_ok, local_ok):
        c = self.c.get('improvement', {})
        if not c.get('enabled'):
            return False
        if tag_ok and obs.get('bounded'):
            self.improvement_since = None
            return False
        stamp = obs['capture_id']
        if self.improvement_since is None:
            self.improvement_since = stamp
        tag = obs.get('tag', {})
        reason = tag.get('reason', 'missing')
        quality = obs.get('camera', {}).get('quality', {})
        lighting = (quality.get('saturated_fraction', 0.) > c['max_saturated_fraction'] or
                    quality.get('dark_fraction', 0.) > c['max_dark_fraction'])
        self.reason = 'initial_improving_'+('exposure' if lighting else reason)
        if stamp-self.improvement_since < c['retry_wait_s']:
            return True
        if self.retries >= c['max_attempts']:
            self.fault('observation_unresolved_'+('exposure' if lighting else reason))
            return True
        self.retries += 1
        self.improvement_since = stamp
        if lighting:
            self.exposure_request += 1
            return True
        bearing = tag.get('bearing_body_rad')
        center = tag_seen and reason == 'clipped' and finite([bearing]) and abs(bearing) > c['center_tolerance_rad']
        missing = not tag_seen and reason in ('missing_or_duplicate_id', 'missing')
        # Ambiguous pose, tiny tag, vertical clipping, bad reprojection:
        # retry fresh images, retain the quality gate, report reason on expiry.
        if not (center or missing) or not local_ok:
            return True
        if self.scan_started is None:
            self.scan_yaw, self.scan_stamp = imu['yaw_rad'], imu['device_stamp_s']
            self.scan_local, self.scan_generation = obs['local_motion'], obs.get('tracking_generation')
            self.scan_gaps = imu.get('gaps', 0)
            self.last_scan_yaw = imu['yaw_rad']
        if obs.get('tracking_generation') != self.scan_generation or self.scan_generation is None:
            self.reason = 'initial_reacquire_needs_continuous_local_tracking'
            return True
        if imu.get('gaps', 0) != self.scan_gaps or imu['device_stamp_s'] < self.scan_stamp:
            self.fault('imu_clock_or_gap')
            return True
        # Include motion/drift during INITIAL_MEASURE before issuing a retry,
        # not one supervisor tick after the new rotation has already started.
        if self.last_scan_yaw is not None:
            self.sweep_travel += abs(imu['yaw_rad']-self.last_scan_yaw)
        self.last_scan_yaw = imu['yaw_rad']
        self.angle_upper = (self.sweep_travel+self.cfg['sensor']['bounds']['gyro_rate_error_rps']*
                            (imu['device_stamp_s']-self.scan_stamp))
        # Retain the ORIGINAL drift reference across all retries.
        local = obs['local_motion']
        drift = math.hypot(*(a-b for a, b in zip(local['xy'], self.scan_local['xy'])))
        drift += math.hypot(*(a+b for a, b in zip(local['error_xy'], self.scan_local['error_xy'])))
        drift += self.cfg['limits']['max_speed_mps']*(now-obs['capture_lower'])
        if drift >= self.c['max_center_drift_m']-self.c['stop_drift_reserve_m']:
            self.fault('reacquire_drift_budget_exhausted')
            return True
        self.leg_target = min(abs(bearing), c['max_center_turn_rad']) if center else None
        if center and self.leg_target <= self.c['brake_reserve_rad']:
            self.reason = 'initial_centering_below_executable_turn'
            return True
        if self.angle_upper+self.c['brake_reserve_rad'] >= self.c['max_sweep_rad']:
            self.fault('search_budget_exhausted')
            return True
        self.scan_direction = ('left' if bearing > 0 else 'right') if center else self.c['direction']
        self.leg_yaw = imu['yaw_rad']
        self.scan_started, self.motion_started = now, None
        self.state = 'INITIAL_CENTERING' if center else 'INITIAL_SEARCH'
        self.command = 'rotate_'+self.scan_direction+'_slow'
        self.reason = 'initial_centering_tag' if center else 'initial_resuming_limited_search'
        self.reset_measurement()
        return True
