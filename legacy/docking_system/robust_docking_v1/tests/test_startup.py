"""Synthetic state/geometry tests; not measured forklift search performance."""
import copy
import math
import unittest
from docking.controller import Controller
from docking.core import Policy, synthesize
from docking.startup import StartupSearch, validate_startup
from test_core import config
from test_recovery import options


def configured():
    c = config()
    c['startup_search'] = dict(enabled=True, direction='left', timeout_s=180., initial_wait_s=.6,
        stable_s=.6, max_sample_gap_s=.3, min_missing_samples=3, min_tag_samples=3,
        pose_error_ratio=.6, max_sweep_rad=2*math.pi, max_rotation_s=120.,
        max_start_delay_s=12., max_stop_response_s=8., brake_reserve_rad=.1,
        max_center_drift_m=.5, stop_drift_reserve_m=.1, free_radius_m=2.)
    c['sensor']['imu']['max_age_s'] = .15
    c['sensor']['bounds']['gyro_rate_error_rps'] = .001
    c['recovery'] = options()
    return c


def sample(t, yaw=0., tag=False, pose=(0., 0., 0.), speed=0., yaw_rate=0.):
    return dict(capture_id=t, capture_lower=t, sensor_ready=True, imu_bad=False,
        imu=dict(ready=True, calibration={'state': 'READY'}, yaw_rad=yaw, device_stamp_s=t, age_s=0.),
        pose=list(pose) if tag else None, error=[0., 0., 0.] if tag else None, valid=tag, bounded=tag,
        tag=dict(ok=tag, detected=tag, id=1 if tag else None, reason='accepted' if tag else 'missing_or_duplicate_id'),
        anchor_stamp=t if tag else None, speed_upper=speed, yaw_rate_upper=yaw_rate,
        odom={'ok': True}, tracking_generation=0,
        local_motion=dict(bounded=True, xy=[0., 0.], error_xy=[0., 0.]))


class StartupTests(unittest.TestCase):
    def search(self, direction='left'):
        c = configured()
        c['startup_search']['direction'] = direction
        s = StartupSearch(c)
        for t in (1., 1.2, 1.4, 1.6, 1.8): s.step(t, sample(t))
        self.assertEqual(s.state, 'INITIAL_SEARCH', s.status())
        return s

    def test_no_rotation_before_initial_bias_calibration_and_measured_stop(self):
        s = StartupSearch(configured())
        for t in (1., 2., 3.):
            o = sample(t)
            o['sensor_ready'], o['imu']['ready'] = False, False
            o['imu']['calibration']['state'] = 'COLLECTING'
            s.step(t, o)
            self.assertEqual(s.command, 'stop')
        for t in (3.2, 3.4, 3.6, 3.8, 4.): s.step(t, sample(t, speed=.1))
        self.assertEqual(s.command, 'stop')
        for t in (4.2, 4.4, 4.6, 4.8, 5.): s.step(t, sample(t))
        self.assertEqual(s.state, 'INITIAL_SEARCH')

    def test_visible_at_start_skips_rotation_and_measures_after_stop(self):
        s = StartupSearch(configured())
        s.step(1., sample(1., tag=True))
        self.assertEqual(s.state, 'INITIAL_BRAKING')
        for t in (1.2, 1.4, 1.6, 1.8, 2., 2.2, 2.4): s.step(t, sample(t, tag=True))
        self.assertTrue(s.complete, s.status())
        self.assertIsNone(s.scan_started)
        self.assertEqual(s.command, 'stop')

    def test_can_search_past_ninety_and_stop_at_tag_near_180_both_directions(self):
        for direction, sign in (('left', 1), ('right', -1)):
            s = self.search(direction)
            self.assertEqual(s.command, 'rotate_'+direction+'_slow')
            for i in range(1, 41):
                t, yaw = 1.8+i*.2, sign*i*.08
                s.step(t, sample(t, yaw=yaw, tag=i == 40, yaw_rate=.4))
                self.assertEqual(s.command, 'stop' if i == 40 else 'rotate_'+direction+'_slow', s.status())
            self.assertGreater(s.angle_upper, math.pi)
            self.assertEqual(s.state, 'INITIAL_BRAKING')
            self.assertFalse(s.complete)
            for i in range(1, 4): s.step(t+i*.2, sample(t+i*.2, yaw=yaw, tag=True, yaw_rate=.1))
            self.assertEqual(s.state, 'INITIAL_BRAKING')
            for i in range(4, 12): s.step(t+i*.2, sample(t+i*.2, yaw=yaw, tag=True))
            self.assertTrue(s.complete, s.status())

    def test_decoded_target_stops_even_when_moving_pose_is_ambiguous(self):
        s = self.search()
        o = sample(2., yaw=.05)
        o['tag'] = dict(id=1, detected=True, ok=False, reason='reprojection_or_planar_ambiguity')
        s.step(2., o)
        self.assertEqual(s.state, 'INITIAL_BRAKING')
        self.assertEqual(s.command, 'stop')
        self.assertFalse(s.complete)

    def test_wrong_id_and_unscheduled_detection_do_not_establish_tag(self):
        s = self.search()
        o = sample(2., tag=True)
        o['tag']['id'] = 99
        s.step(2., o)
        self.assertEqual(s.state, 'INITIAL_SEARCH')
        s = StartupSearch(configured())
        for i in range(10):
            t = 1.+i*.2
            o = sample(t)
            o['tag']['reason'] = 'not_scheduled'
            s.step(t, o)
        self.assertEqual(s.command, 'stop')
        self.assertEqual(s.missing_samples, 0)

    def test_tag_that_coasts_out_of_view_does_not_restart_search(self):
        s = self.search()
        s.step(2., sample(2., tag=True))
        for i in range(1, 12):
            t = 2.+i*.2
            s.step(t, sample(t))
            self.assertEqual(s.command, 'stop', s.status())
        self.assertFalse(s.complete)
        self.assertEqual(s.state, 'INITIAL_MEASURE')

    def test_duplicate_frame_and_single_tag_do_not_finish_initial_measurement(self):
        s = self.search()
        s.step(2., sample(2., tag=True))
        for t in (2.2, 2.4, 2.6): s.step(t, sample(t, tag=True))
        o = sample(2.8, tag=True)
        s.step(2.8, o)
        count = s.tag_samples
        for t in (2.81, 2.9, 3.): s.step(t, o)
        self.assertEqual(s.tag_samples, count)
        self.assertFalse(s.complete)
        for t in (3., 3.2, 3.4, 3.6):
            o = sample(t)
            o.update(valid=True, bounded=True, pose=[0, 0, 0], error=[0, 0, 0], anchor_stamp=2.8)
            s.step(t, o)
        self.assertFalse(s.complete)

    def test_search_sensor_loss_clock_reversal_and_reseed_latch_stop(self):
        for failure in ('no_frame', 'stale', 'imu', 'clock', 'reseed'):
            s = self.search()
            o = sample(2.)
            if failure == 'no_frame': o = None
            elif failure == 'stale': o['capture_lower'] = 1.
            elif failure == 'imu': o['imu_bad'] = True
            elif failure == 'clock': o['capture_id'] = 1.
            elif failure == 'reseed': o['tracking_generation'] = 1
            s.step(2., o)
            self.assertEqual((s.state, s.command), ('FAULT', 'stop'), (failure, s.status()))
            s.step(2.2, sample(2.2, tag=True))
            self.assertEqual(s.state, 'FAULT')

    def test_delayed_start_does_not_use_up_rotation_duration(self):
        s = self.search()
        s.c['max_rotation_s'] = 1.
        for i in range(1, 31):
            t = 1.8+i*.2
            s.step(t, sample(t))
        self.assertEqual(s.state, 'INITIAL_SEARCH')
        self.assertIsNone(s.motion_started)
        s.step(14., None)
        self.assertEqual(s.reason, 'initial_rotation_start_timeout')

    def test_time_and_angle_budgets_stop_unproductive_search(self):
        s = self.search()
        for i in range(1, 81):
            t = 1.8+i*.2
            s.step(t, sample(t, yaw=i*.08))
            if s.state == 'FAULT': break
        self.assertEqual(s.reason, 'initial_search_budget_exhausted')
        self.assertLess(s.angle_upper, 2*math.pi)
        s = self.search()
        s.step(200., None)
        self.assertEqual(s.reason, 'initial_acquisition_timeout')

    def test_center_drift_has_stop_reserve(self):
        s = self.search()
        o = sample(2., yaw=.02)
        o['local_motion']['xy'] = [.41, 0.]
        s.step(2., o)
        self.assertEqual(s.reason, 'initial_search_budget_exhausted')
        self.assertEqual(s.command, 'stop')

    def test_configuration_requires_full_vehicle_clearance_and_evidence(self):
        c = configured()
        validate_startup(c)
        with self.assertRaisesRegex(ValueError, 'validation'): validate_startup(c, live=True)
        c['validation']['startup_search'] = 'synthetic test only'
        validate_startup(c, live=True)
        c['startup_search']['free_radius_m'] = .1
        with self.assertRaisesRegex(ValueError, 'free disk'): validate_startup(c)
        c = configured()
        c['startup_search']['max_start_delay_s'] = None
        with self.assertRaisesRegex(ValueError, 'max_start_delay'): validate_startup(c)


class StartupHandoverTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = configured()
        cls.policy = Policy(cls.cfg, synthesize(cls.cfg))
        key = tuple(map(int, next(k for k, v in cls.policy.table['nodes'].items() if v[0] == 1).split(',')))
        cls.pose = cls.policy.grid.belief(key).center

    def start_mission(self):
        c = Controller(copy.deepcopy(self.cfg), self.policy)
        for i in range(1, 25):
            t = i*.2
            c.step(t, sample(t, tag=True, pose=self.pose))
            if c.state == 'MOVING': break
        self.assertEqual(c.state, 'MOVING', c.status())
        self.assertTrue(c.initial.complete)
        return c, t

    def test_initial_handover_then_dropout_uses_recovery_never_initial_search(self):
        c, t = self.start_mission()
        c.step(t+.1, None)
        self.assertEqual(c.state, 'PAUSING')
        self.assertTrue(c.initial.complete)
        for i in range(1, 4): c.step(t+.1+i*.2, sample(t+.1+i*.2))
        self.assertEqual(c.command, 'stop')
        self.assertFalse(c.state.startswith('INITIAL'))
        self.assertIsNone(c.initial.scan_started)

    def test_search_behind_vehicle_hands_over_to_real_policy_and_finishes(self):
        c = Controller(copy.deepcopy(self.cfg), self.policy)
        for t in (1., 1.2, 1.4, 1.6, 1.8): c.step(t, sample(t))
        self.assertEqual(c.state, 'INITIAL_SEARCH')
        self.assertEqual(c.transitions, 0)
        for i in range(1, 41):
            t = 1.8+i*.2
            c.step(t, sample(t, yaw=i*.08, tag=i == 40, pose=self.pose, yaw_rate=.4))
        self.assertEqual(c.state, 'INITIAL_BRAKING')
        for i in range(1, 20):
            t = 9.8+i*.2
            c.step(t, sample(t, yaw=3.2, tag=True, pose=self.pose))
            if c.state == 'MOVING': break
        self.assertEqual(c.state, 'MOVING', c.status())
        self.assertEqual(c.phase, 'APPROACH')
        self.assertEqual(c.transitions, 1)
        self.assertEqual(c.recovery_attempts, 0)
        self.assertEqual(c.run_started, 1.)
        target = c.expected.center
        for dt in (.2, .2, .5):
            t += dt
            c.step(t, sample(t, yaw=3.2, tag=True, pose=target))
        self.assertEqual(c.state, 'DONE', c.status())

    def test_clipping_with_continuous_tracking_keeps_normal_motion(self):
        c, t = self.start_mission()
        o = sample(t+.2, tag=True, pose=self.pose)
        o['tag'] = dict(ok=False, reason='clipped')
        o['anchor_stamp'] = t
        c.step(t+.2, o)
        self.assertEqual(c.state, 'MOVING')
        self.assertTrue(c.initial.complete)

    def test_sidestep_absence_uses_sidestep_recovery_not_initial_search(self):
        c, t = self.start_mission()
        c.cfg['prealignment'] = dict(max_duration_s=120.)
        c.phase, c.phase_started, c.last_tag_seen = 'SIDESTEP', t, t
        o = sample(t+2.1, tag=True, pose=self.pose)
        o['tag'] = dict(ok=False, reason='missing_or_duplicate_id')
        o['anchor_stamp'] = t
        c.step(t+2.1, o)
        self.assertEqual(c.state, 'MOVING')
        o.update(capture_id=t+2.2, capture_lower=t+2.2, valid=False, bounded=False)
        c.step(t+2.2, o)
        self.assertEqual(c.reason, 'observation_unavailable')
        self.assertEqual(c.state, 'PAUSING')
        self.assertTrue(c.initial.complete)

    def test_operator_fault_cannot_restart_initial_search(self):
        c = Controller(self.cfg, self.policy)
        c.fault('operator_stop_or_ssh_eof')
        for t in (1., 2., 3.): c.step(t, sample(t))
        self.assertEqual(c.state, 'FAULT')
        self.assertEqual(c.command, 'stop')
