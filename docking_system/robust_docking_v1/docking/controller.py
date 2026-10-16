"""R5/R6/R7: measured settling, bounded recovery, and latched hard faults."""
import math
from .core import Box, action_box, transition, safe, project_xy, finite
from .recovery import within_budget, return_rotation, return_config, tag_heading, return_observation, return_space_available
from .startup import StartupSearch
from .startup_measurement import StartupMeasurement
from .feedback import RotationFeedback, imu_ready
from .motion import AngularStopGate, stopped as measured_stop


RETURN_REASONS = frozenset(('sidestep_tag_absence', 'observation_error_growing',
    'tracking_bridge_exhausted', 'tracking_chain_changed', 'observation_unavailable',
    'sidestep_reacquire', 'observation_error_budget_exceeded',
    'outside_winning_belief_domain', 'sidestep_return_space_unavailable', 'sidestep_tag_detected'))

class Controller:
    def __init__(self, cfg, policy, fast_imu_required=False):
        self.cfg, self.policy = cfg, policy
        self.state = 'WAITING'
        self.reason = 'need_fresh_bounded_pose_and_measured_stop'
        self.command = 'stop'
        self.action = self.start_box = self.start_pose = None
        self.started = self.braked = self.still_since = None
        self.last_capture = None
        self.rank = None
        self.expected = None
        self.transitions = 0
        self.run_started = None
        self.phase = None
        self.phase_started = None
        self.phase_actions = 0
        self.motion_started = None
        self.start_observed_box = None
        self.recovery = cfg.get('recovery', {})
        self.recovery_started = self.recovery_capture = None
        self.recovery_attempts = self.return_actions = 0
        self.recovery_needs_tag = self.returning = False
        self.pause_envelope = None
        self.stable_start = self.stable_last = self.stable_box = None
        self.stable_count = 0
        self.sidestep_heading = self.last_tag_seen = None
        self.last_box = None
        self.return_reference = None
        self.sidestep_leg = None
        self.last_generation = self.recovery_generation = None
        self.recovery_tags = 0
        self.recovery_last_anchor = None
        self.return_allowed = False
        self.initial = StartupSearch(cfg)
        self.preparation = StartupMeasurement(cfg)
        self.fast_imu_required = fast_imu_required
        self.fast_imu = self.rotation_feedback = None
        self.stop_sample = self.stop_generation = self.stop_since = None
        self.stop_proof = None
        self.angular_stop = AngularStopGate(cfg)
        self.angular_stop_ok = None
        self.recovery_last_seen = None
        self.return_seeking_tag = False

    def fault(self, reason):
        self.state, self.reason, self.command = 'FAULT', reason, 'stop'
        return self.status()

    def status(self):
        return {'state': self.state, 'reason': self.reason, 'command': self.command,
                'rank': self.rank, 'action': self.action['name'] if self.action else None,
                'transitions': self.transitions, 'phase': self.phase, 'phase_actions': self.phase_actions,
                'motion_observed': self.motion_started is not None,
                'recovery_attempts': self.recovery_attempts, 'return_actions': self.return_actions,
                'recovery_needs_tag': self.recovery_needs_tag, 'returning_to_tag': self.returning,
                'recovery_samples': self.stable_count, 'recovery_tag_samples': self.recovery_tags,
                'stop_proof': self.stop_proof, 'latest_gyro_stop_ok': self.angular_stop_ok,
                'sidestep_leg': self.sidestep_leg, 'initial_acquisition': self.initial.status(),
                'startup_measurement': self.preparation.status()}

    def _reset_stability(self):
        self.stable_start = self.stable_last = self.stable_box = None
        self.stable_count = 0

    def _lost_stop_evidence(self, now):
        self.still_since = self.stop_sample = self.stop_generation = self.stop_since = None
        self.stop_proof = None
        self.recovery_tags, self.recovery_last_anchor = 0, None
        self._reset_stability()
        if self.state == 'REOBSERVING':
            # STOP remains active. Loss of a previously confirmed stop starts
            # one new stop-confirmation deadline, never a deadline per frame.
            self.state, self.braked = 'PAUSING', now

    def _recovery_stopped(self, now, obs, stopped):
        stamp, generation = obs['capture_id'], obs.get('tracking_generation')
        proof = 'model_return' if obs.get('return_only') else 'visual'
        if not stopped:
            self._lost_stop_evidence(now)
            return False
        if (self.stop_sample is None or proof != self.stop_proof or
                (proof == 'visual' and generation != self.stop_generation) or
                stamp-self.stop_sample > self.recovery['max_sample_gap_s']):
            self._lost_stop_evidence(now)
        self.stop_sample, self.stop_generation = stamp, generation
        self.stop_proof = proof
        if self.stop_since is None:
            self.stop_since = stamp
        if stamp-self.stop_since < self.cfg['limits']['stop_dwell_s']:
            return False
        self.state = 'REOBSERVING'
        return True

    def _arm_rotation(self, now, obs):
        self.rotation_feedback = None
        if self.action['kind'] != 'rotate' or not self.fast_imu_required:
            return None
        try:
            self.rotation_feedback = RotationFeedback(self.cfg, self.action, self.start_pose[2], obs, self.fast_imu)
        except ValueError as exc:
            return self.pause(now, str(exc), True)
        return None

    def _fast_rotation(self, now):
        if self.state != 'MOVING' or self.rotation_feedback is None:
            return None
        decision, moved = self.rotation_feedback.update(self.fast_imu)
        if decision == 'clock_reversed':
            return self.fault('imu_clock_reversed')
        if decision in ('gap', 'unavailable'):
            return self.pause(now, 'imu_gap_or_stale', True)
        if moved and self.motion_started is None:
            self.motion_started = now
        if decision == 'brake':
            if self.motion_started is None:
                return self.pause(now, 'brake_threshold_without_observed_motion', True)
            self.state, self.command, self.reason = 'BRAKING', 'stop', 'gyro_target_braking_reserve'
            self.braked, self.still_since = now, None
            return self.status()
        return None

    def pause(self, now, reason, require_tag=False):
        if not self.recovery.get('enabled'):
            return self.fault(reason)
        if self.recovery_started is None:
            if self.recovery_attempts >= self.recovery['max_attempts']:
                return self.fault('recovery_attempt_limit')
            self.recovery_attempts += 1
            self.recovery_started, self.recovery_capture = now, self.last_capture
            self.return_actions = 0
            self.recovery_needs_tag = False
            self.recovery_generation = self.last_generation
            self.recovery_tags, self.recovery_last_anchor = 0, None
            self.recovery_last_seen = None
            self.return_allowed = reason in RETURN_REASONS
        elif reason not in RETURN_REASONS:
            self.return_allowed = False
        self.recovery_needs_tag |= require_tag or (self.phase == 'SIDESTEP' and reason in RETURN_REASONS)
        if self.state in ('MOVING', 'BRAKING') and self.action is not None:
            self.pause_envelope = action_box(self.start_box, self.action, (0., 1.), True)
        elif self.pause_envelope is None:
            self.pause_envelope = self.last_box
        # The completed-action endpoint no longer describes an interrupted leg.
        self.expected = None
        self.state, self.command, self.reason = 'PAUSING', 'stop', reason
        self.braked, self.still_since, self.returning = now, None, False
        self.return_seeking_tag = False
        self.stop_sample = self.stop_generation = self.stop_since = None
        self.stop_proof = None
        self.recovery_tags, self.recovery_last_anchor = 0, None
        self._reset_stability()
        return self.status()

    def _recover(self, now, obs, box, stopped):
        if self.pause_envelope is not None:
            if self.pause_envelope.intersection(box) is None:
                return self.fault('recovery_pose_conflicts_with_stop_envelope')
        stamp = obs['capture_id']
        tag = obs.get('tag', {})
        target_seen = (tag.get('detected', tag.get('ok')) and
                       tag.get('id') == self.cfg['sensor']['tag']['id'])
        if target_seen:
            self.recovery_last_seen = stamp
        if (self.recovery_last_anchor is not None and
                stamp-self.recovery_last_anchor > self.recovery['max_sample_gap_s']):
            self.recovery_tags, self.recovery_last_anchor = 0, None
            self._reset_stability()
        if not self._recovery_stopped(now, obs, stopped):
            return self.status()
        reanchored_chain = (obs.get('anchor_stamp') is not None and self.recovery_capture is not None
                            and self.recovery_capture < obs['anchor_stamp'] <= stamp)
        fresh_anchor = (reanchored_chain and
                        stamp-obs['anchor_stamp'] <= self.recovery['max_sample_gap_s'])
        if (fresh_anchor and target_seen and tag.get('ok') and not obs.get('return_only')
                and obs['anchor_stamp'] != self.recovery_last_anchor):
            self.recovery_tags += 1
            self.recovery_last_anchor = obs['anchor_stamp']
        recent_sighting = (self.recovery_last_seen is not None and
                          stamp-self.recovery_last_seen <= self.recovery['max_sample_gap_s'])
        if self.recovery_needs_tag and not fresh_anchor and not recent_sighting:
            # A bounded chain may also originate at a recently accepted tag
            # that has since disappeared. This supports returning only, not
            # normal restart. A bare reseeded local origin is never sufficient.
            if (self.return_allowed and self.phase == 'SIDESTEP' and self.sidestep_heading is not None
                    and obs.get('anchor_stamp') is not None
                    and ((obs.get('odom', {}).get('ok') and
                          (reanchored_chain or (self.recovery_generation is not None and
                           obs.get('tracking_generation') == self.recovery_generation))) or
                         obs.get('return_only') or obs.get('tracking_recovery', {}).get('coarse'))):
                active = return_config(getattr(self.policy, 'active_cfg', self.cfg))
                heading = tag_heading(box, active, self.sidestep_heading)
                candidate = return_rotation(box, heading, active)
                if candidate is None:
                    # Coarse lateral position may make the bearing uncertain.
                    # First restore the previously tag-facing heading if executable.
                    candidate = return_rotation(box, self.sidestep_heading, active)
                if candidate and self.return_actions < self.recovery['max_return_actions']:
                    if self.transitions >= self.cfg['limits'].get('max_steps', 20):
                        return self.fault('maximum_actions_reached')
                    if self.phase_actions >= self.cfg['prealignment']['max_actions']:
                        return self.fault('maximum_sidestep_actions_reached')
                    self.action, self.expected = candidate
                    self.sidestep_leg = 'RETURN_TO_TAG'
                    self.start_box, self.start_pose, self.start_observed_box = box, box.center, box
                    refused = self._arm_rotation(now, obs)
                    if refused is not None:
                        return refused
                    self.started, self.motion_started, self.still_since = now, None, None
                    self.returning = True
                    self.return_seeking_tag = True
                    self.recovery_tags, self.recovery_last_anchor = 0, None
                    self.return_actions += 1
                    self.transitions += 1
                    self.phase_actions += 1
                    self._reset_stability()
                    self.state, self.command, self.reason = 'MOVING', self.action['command'], 'return_to_tag_heading'
                    return self.status()
            self.reason = 'waiting_for_absolute_tag_reanchor'
            self._reset_stability()
            return self.status()
        if self.recovery_needs_tag and self.recovery_tags < self.recovery['min_tag_samples']:
            self.reason = 'waiting_for_multiple_tag_observations'
            self._reset_stability()
            return self.status()
        if not within_budget(box, self.cfg, self.recovery['resume_error_ratio']):
            self.reason = 'waiting_for_smaller_observation_error'
            self._reset_stability()
            return self.status()
        stamp = obs['capture_id']
        if (self.stable_last is None or stamp-self.stable_last > self.recovery['max_sample_gap_s']
                or self.stable_box.intersection(box) is None):
            self.stable_start, self.stable_box, self.stable_count = stamp, box, 0
        else:
            # Used ONLY for a consistency check, never to shrink the live bound.
            self.stable_box = self.stable_box.intersection(box)
        self.stable_last, self.stable_count = stamp, self.stable_count+1
        self.reason = 'waiting_for_stable_fresh_observations'
        if self.stable_count < self.recovery['min_samples'] or stamp-self.stable_start < self.recovery['stable_s']:
            return self.status()
        recal_cfg = self.cfg.get('sensor', {}).get('imu', {}).get('stationary_recalibration', {})
        recal = obs.get('imu', {}).get('stationary_recalibration', {})
        applied = (recal.get('result') or {}).get('applied_at_device_s')
        recal_done = applied is not None and self.recovery_capture is not None and applied > self.recovery_capture
        if recal_cfg.get('enabled') and recal_cfg.get('verified') and recal.get('collecting') and not recal_done:
            # Give an eligible stationary window time to finish. An older bias
            # update (or duplicate snapshot) cannot release this wait. If the
            # window is rejected, the old bias/error bounds remain in force.
            self.reason = 'waiting_for_stationary_gyro_calibration'
            return self.status()
        # Recovery may interrupt progress. Replan at the measured stop rather
        # than replaying the old command. Mission/time/attempt budgets survive.
        self.rank, self.expected, self.action = None, None, None
        self.pause_envelope = None
        self.recovery_started = None
        self.recovery_needs_tag = False
        self.still_since = now-(obs['capture_id']-self.stop_since)
        self.state, self.reason = 'WAITING', 'recovered_replanning'
        return None  # Continue into the normal policy lookup on this frame.

    def step(self, now, obs, fast_imu=None, can_status=None):
        if self.state in ('FAULT', 'DONE'):
            return self.status()
        lim = self.cfg['limits']
        self.fast_imu = fast_imu
        if fast_imu is not None and 'host_arrival_s' in fast_imu:
            self.fast_imu = dict(fast_imu, age_s=now-fast_imu['host_arrival_s'])
        self.angular_stop_ok = self.angular_stop.update(self.fast_imu, self.fast_imu_required)
        if self.angular_stop_ok is False:
            self._lost_stop_evidence(now)
        # No search, centering, or mission motion until the stationary window
        # completes. Its deadline is separate from the actual mission budget.
        if not self.preparation.complete:
            if self.fast_imu and self.fast_imu.get('clock_reversals'):
                return self.fault('imu_clock_reversed')
            self.preparation.step(now, obs, self.angular_stop_ok, can_status)
            self.state, self.command, self.reason = (self.preparation.state, 'stop', self.preparation.reason)
            if self.preparation.complete:
                self.state = 'WAITING'
                self._lost_stop_evidence(now)
            return self.status()
        if self.run_started is None:
            self.run_started = now
        if now-self.run_started > lim.get('run_timeout_s', 300.):
            return self.fault('run_timeout')
        # This one-shot gate is independent of recovery/mission phase. Once
        # completed it is never re-entered by tag clipping or a sidestep.
        if not self.initial.complete:
            self.initial.step(now, obs, self.fast_imu, self.fast_imu_required)
            self.state, self.command, self.reason = self.initial.state, self.initial.command, self.initial.reason
            if self.initial.complete:
                self.state, self.command = 'WAITING', 'stop'
                self.last_capture = obs['capture_id']
                self.last_generation = obs.get('tracking_generation')
                self.still_since = None
            return self.status()
        if self.recovery_started is not None and now-self.recovery_started > self.recovery['timeout_s']:
            return self.fault('recovery_timeout')
        # Deadlines are independent of sensor arrival and unique frame count.
        if self.state == 'MOVING':
            start_limit = self.action.get('max_start_delay_s')
            if start_limit is not None and self.motion_started is None:
                if now-self.started > start_limit:
                    return self.fault('command_start_timeout')
            elif now-(self.motion_started if self.motion_started is not None else self.started) > self.action['max_duration_s']:
                return self.fault('primitive_timeout')
        stop_timeout = lim['settle_timeout_s']+(self.action.get('max_stop_response_s', 0.) if self.action else 0.)
        blind = self.recovery.get('blind_return', {})
        if blind.get('verified') and self.return_allowed and self.recovery_started is not None:
            stop_timeout = max(stop_timeout, blind['stop_rest_after_s']+lim['stop_dwell_s']+self.recovery['max_sample_gap_s'])
        if self.state in ('BRAKING', 'PAUSING') and now-self.braked > stop_timeout:
            return self.fault('stop_not_observed')
        if self.phase == 'SIDESTEP' and now-self.phase_started > self.cfg['prealignment']['max_duration_s']:
            return self.fault('sidestep_timeout')
        if obs is not None and obs.get('imu', {}).get('calibration', {}).get('state') == 'FAILED':
            return self.fault('imu_startup_calibration_failed')
        if obs is not None and (obs.get('clock_fault') or obs.get('imu', {}).get('clock_reversals', 0)):
            return self.fault('capture_clock_reversed')
        # This path runs on every supervisor tick, including repeated video
        # frames. Once it brakes, stale images cannot restore MOVING.
        fast_result = self._fast_rotation(now)
        if fast_result is not None:
            return fast_result
        if self.fast_imu_required and not imu_ready(self.fast_imu, self.cfg):
            if self.fast_imu and self.fast_imu.get('clock_reversals'):
                return self.fault('imu_clock_reversed')
            if self.state in ('MOVING', 'BRAKING'):
                return self.pause(now, 'imu_gap_or_stale', True)
            self._lost_stop_evidence(now)
            self.reason = 'waiting_for_fresh_imu_feedback'
            return self.status()
        if obs is not None and not obs.get('sensor_ready', True):
            if self.state in ('MOVING', 'BRAKING'):
                return self.pause(now, 'sensor_not_ready', True)
            self._lost_stop_evidence(now)
            if self.recovery_started is not None:
                self.recovery_needs_tag = True
                self.recovery_tags = 0
                self.return_allowed = False
            self.reason = ('waiting_for_ready_sensor_interval' if obs.get('imu', {}).get('ready')
                           else 'waiting_for_stationary_imu_calibration')
            return self.status()
        # capture_lower is a conservative bound on host-monotonic exposure
        # time, derived from an externally bounded transport latency.
        if obs is None:
            if self.state in ('MOVING', 'BRAKING'):
                return self.pause(now, 'observation_unavailable', True)
            self._lost_stop_evidence(now)
            if self.recovery_started is not None:
                self.recovery_needs_tag = True
                self.recovery_tags = 0
                self.return_allowed = False
            return self.status()
        lower = obs.get('capture_lower')
        if lower is None or lower > now:
            return self.fault('observation_too_old_or_clock_invalid')
        if now-lower > lim['max_age_s']:
            if self.state not in ('PAUSING', 'REOBSERVING'):
                return self.pause(now, 'observation_too_old_or_clock_invalid', True)
            self._lost_stop_evidence(now)
            self.recovery_needs_tag = True
            self.recovery_tags = 0
            self.return_allowed = False
            return self.status()
        stamp = obs['capture_id']
        if self.last_capture is not None and stamp < self.last_capture:
            return self.fault('capture_clock_reversed')
        if stamp == self.last_capture:
            return self.status()  # Never count repeated copies toward dwell.
        self.last_capture = stamp
        if obs.get('imu_bad'):
            if self.state not in ('PAUSING', 'REOBSERVING'):
                return self.pause(now, 'imu_gap_or_stale', True)
            self._lost_stop_evidence(now)
            self.recovery_needs_tag = True
            self.recovery_tags = 0
            self.return_allowed = False
            return self.status()
        obs = return_observation(self, now, obs)
        generation = obs.get('tracking_generation')
        previous_generation, self.last_generation = self.last_generation, generation
        if (previous_generation is not None and generation != previous_generation
                and self.state in ('MOVING', 'BRAKING') and not obs.get('return_only')):
            return self.pause(now, 'tracking_chain_changed', True)
        stopped = (obs.get('qualified_return_stop', False) if obs.get('return_only')
                   else measured_stop(obs, now, lim, self.angular_stop_ok))
        if not obs.get('bounded') or not obs.get('valid'):
            if self.state in ('MOVING', 'BRAKING'):
                return self.pause(now, 'observation_unavailable', True)
            self._reset_stability()
            if self.recovery_started is not None:
                self.recovery_needs_tag = True
                if not (blind.get('enabled') and blind.get('verified')):
                    self.return_allowed = False
                self.recovery_tags = 0
                # A restarted local feature chain can prove stationary motion,
                # but supplies NO dock position and cannot authorize a turn.
                local_ok = (obs.get('motion_bounded') and obs.get('odom', {}).get('ok') and
                            obs.get('local_motion', {}).get('bounded') and generation is not None)
                if self._recovery_stopped(now, obs, stopped and local_ok):
                    self.reason = 'waiting_for_absolute_tag_reanchor'
            else:
                self._lost_stop_evidence(now)
            return self.status()
        pose, error = obs['pose'], obs['error']
        # Propagate from capture to NOW with physical rate bounds. Publication
        # at a later time must not make an old pose look newly measured.
        age = now-lower
        b = Box.around(pose, error).inflate((lim['max_speed_mps']*age,
                                           lim['max_speed_mps']*age,
                                           lim['max_yaw_rate_rps']*age))
        active = getattr(self.policy, 'active_cfg', self.cfg)
        safety_cfg = return_config(active) if self.returning or obs.get('return_only') or self.recovery_started is not None else active
        if not safe(b, safety_cfg):
            return self.fault('body_uncertainty_outside_free_space')
        self.last_box = b
        if obs.get('bounded') and not obs.get('return_only') and not obs.get('tracking_recovery', {}).get('predicting'):
            if obs.get('capture_imu'):
                self.return_reference = dict(pose=pose[:], error=error[:], imu=dict(obs['capture_imu']),
                                             anchor_stamp=obs.get('anchor_stamp'))
        if obs.get('tag', {}).get('ok'):
            self.last_tag_seen = now
        if self.state in ('PAUSING', 'REOBSERVING'):
            recovered = self._recover(now, obs, b, stopped)
            if recovered is not None:
                return recovered
        if self.state == 'MOVING':
            # A conservative emergency STOP must remain inside the prechecked
            # sweep contract. A measured contract violation latches failure.
            sweep = action_box(self.start_box, self.action, (0., 1.), True)
            if sweep.intersection(b) is None:
                return self.fault('primitive_sweep_contract_exceeded')
            tag = obs.get('tag', {})
            target_seen = (tag.get('detected', tag.get('ok')) and
                           tag.get('id') == self.cfg['sensor']['tag']['id'])
            return_leg = (self.phase == 'SIDESTEP' and self.action['kind'] == 'rotate' and
                          (self.returning or self.sidestep_leg == 'RETURN_TO_TAG'))
            if return_leg and not target_seen and tag.get('reason') != 'not_scheduled':
                self.return_seeking_tag = True
            if return_leg and self.return_seeking_tag and target_seen:
                result = self.pause(now, 'sidestep_tag_detected', True)
                self.recovery_last_seen = stamp
                return result
            if self.recovery.get('enabled'):
                if not self.returning and not within_budget(b, self.cfg, self.recovery['pause_error_ratio']):
                    return self.pause(now, 'observation_error_growing')
                tracking = obs.get('tracking_recovery', {})
                if tracking.get('predicting') and not self.returning:
                    # No guessed translation progress, stop proof or new action while blind.
                    # Continue only inside the checked prefix/STOP contract and stop early
                    # if the worst possible position has reached the braking threshold.
                    if not tracking.get('bridge_ok') or not sweep.contains(b):
                        return self.pause(now, 'tracking_bridge_exhausted', True)
                    if self.action['kind'] == 'translate':
                        h = self.start_pose[2]
                        direction = (math.cos(h), math.sin(h))
                        extent = project_xy(b, direction)
                        origin = self.start_pose[0]*direction[0]+self.start_pose[1]*direction[1]
                        upper = extent[1]-origin if self.action['target'] > 0 else origin-extent[0]
                        if upper >= abs(self.action['target'])-self.action['brake_reserve']:
                            return self.pause(now, 'tracking_bridge_exhausted', True)
                    self.reason = 'bounded_tracking_bridge'
                    return self.status()
            sign = 1 if self.action['target'] > 0 else -1
            if self.action['kind'] == 'rotate':
                progress = sign*(pose[2]-self.start_pose[2])
                motion_lower = b.lo[2]-self.start_observed_box.hi[2] if sign > 0 else self.start_observed_box.lo[2]-b.hi[2]
            else:
                h = self.start_pose[2]
                progress = sign*((pose[0]-self.start_pose[0])*math.cos(h)
                                 +(pose[1]-self.start_pose[1])*math.sin(h))
                previous = project_xy(self.start_observed_box, (math.cos(h), math.sin(h)))
                current = project_xy(b, (math.cos(h), math.sin(h)))
                motion_lower = current[0]-previous[1] if sign > 0 else previous[0]-current[1]
            if self.motion_started is None and motion_lower > 0:
                self.motion_started = now
            # reserve is a verified upper bound covering sampling, age,
            # command delay and coast for this entire speed envelope.
            if progress >= abs(self.action['target'])-self.action['brake_reserve']:
                if self.motion_started is None:
                    return self.pause(now, 'brake_threshold_without_observed_motion', True)
                self.state, self.command, self.reason = 'BRAKING', 'stop', 'target_braking_reserve'
                self.braked, self.still_since = now, None
            return self.status()
        if not stopped:
            self.still_since = None
            return self.status()
        if self.still_since is None:
            self.still_since = now
        if now-self.still_since < lim['stop_dwell_s']:
            return self.status()
        if not self.returning and not within_budget(b, self.cfg):
            return self.pause(now, 'observation_error_budget_exceeded')
        if self.expected is not None:
            intersection = b.intersection(self.expected)
            if intersection is None:
                return self.fault('terminal_motion_contract_exceeded')
            if not self.returning:
                b = intersection
            # Returning does not re-observe the dock. Preserve the full live
            # gyro/position uncertainty instead of narrowing it at a STOP.
        if self.returning:
            self.returning = False
            self.pause_envelope = b
            self.expected, self.action = None, None
            self.state, self.command, self.reason = 'REOBSERVING', 'stop', 'return_rotation_stopped'
            self.stop_sample, self.stop_generation = stamp, generation
            self.stop_proof = 'model_return' if obs.get('return_only') else 'visual'
            # Normal braking already confirmed a stop. Translate that fact
            # into the device-time dwell field; never subtract a host clock
            # from a camera timestamp on the next recovery observation.
            self.stop_since = stamp-lim['stop_dwell_s']
            self._reset_stability()
            return self.status()
        tag_visible = obs.get('tag', {}).get('ok') or (
            obs.get('tag', {}).get('reason') == 'not_scheduled' and self.last_tag_seen is not None and
            now-self.last_tag_seen <= self.cfg['sensor']['tag']['period_s']+lim['max_age_s'])
        if (self.phase == 'SIDESTEP' and self.action is not None and
                not tag_visible and
                (self.action['kind'] == 'translate' or self.sidestep_leg == 'RETURN_TO_TAG')):
            return self.pause(now, 'sidestep_reacquire', True)
        choice = self.policy.lookup(pose, b)
        if choice is None:
            return self.pause(now, 'outside_winning_belief_domain')
        phase = choice.get('phase', 'APPROACH')
        if self.phase != phase:
            if self.phase is not None and (self.phase, phase) != ('SIDESTEP', 'APPROACH'):
                return self.fault('invalid_mission_phase_transition')
            self.phase, self.phase_started, self.phase_actions = phase, now, 0
            if phase == 'SIDESTEP':
                self.sidestep_heading = pose[2]
                self.last_tag_seen = now if obs.get('tag', {}).get('ok') else None
            self.rank = None
        if self.rank is not None and choice['rank'] >= self.rank:
            return self.fault('robust_progress_not_verified')
        self.rank = choice['rank']
        if self.rank == 0:
            if not Box(self.cfg['goal']['lo'], self.cfg['goal']['hi']).contains(b):
                return self.fault('goal_set_not_contained')
            self.state, self.command, self.reason = 'DONE', 'stop', 'full_goal_set_and_stop_dwell_verified'
            return self.status()
        self.action = choice['action']
        if self.phase == 'SIDESTEP':
            if self.action['kind'] == 'translate':
                self.sidestep_leg = 'LATERAL_MOVE'
            else:
                desired = tag_heading(b, self.cfg, self.sidestep_heading)
                delta = (desired-pose[2]+math.pi) % (2*math.pi)-math.pi
                self.sidestep_leg = 'RETURN_TO_TAG' if delta*self.action['target'] > 0 else 'TURN_OUT'
        # A visible tag does not interrupt an ordinary heading correction.
        # Early STOP applies to reacquiring a lost tag during the return leg.
        self.return_seeking_tag = (self.phase == 'SIDESTEP' and
                                   self.sidestep_leg == 'RETURN_TO_TAG' and not tag_visible)
        if self.transitions >= lim.get('max_steps', 20):
            return self.fault('maximum_actions_reached')
        if self.phase == 'SIDESTEP' and self.phase_actions >= self.cfg['prealignment']['max_actions']:
            return self.fault('maximum_sidestep_actions_reached')
        # Cover the complete abstraction belief, not only the actual midpoint.
        self.start_box = self.policy.grid.belief(choice['key'])
        if self.phase == 'SIDESTEP' and not return_space_available(self.start_box, self.action, getattr(self.policy, 'active_cfg', self.cfg)):
            return self.pause(now, 'sidestep_return_space_unavailable', True)
        self.expected = transition(self.start_box, self.action, getattr(self.policy, 'active_cfg', self.cfg))
        if self.expected is None:
            return self.fault('sweep_invalid')
        # Use the SAME absolute target as synthesis. Moving the target to the
        # latest measured midpoint would invalidate its endpoint contract.
        self.start_pose = self.start_box.center
        self.start_observed_box = b
        refused = self._arm_rotation(now, obs)
        if refused is not None:
            return refused
        self.motion_started = None
        self.started, self.still_since = now, None
        self.state, self.command, self.reason = 'MOVING', self.action['command'], 'winning_action'
        self.transitions += 1
        self.phase_actions += 1
        return self.status()
