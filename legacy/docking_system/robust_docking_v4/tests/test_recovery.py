"""Recovery uses the real controller/policy and synthetic bounded observations.

These check software behavior, not physical bounds or forklift stopping accuracy.
"""
import copy
import math
import unittest
from docking.controller import Controller
from docking.core import Box, Policy, synthesize
from docking.recovery import return_rotation, validate_recovery
from test_core import config, observation


def options():
    return dict(enabled=True, timeout_s=30., stable_s=.6, max_sample_gap_s=.4,
                min_samples=3, min_tag_samples=2, max_attempts=2, pause_error_ratio=.9, resume_error_ratio=.5,
                tag_return_after_s=2., return_heading_tolerance_rad=.04, max_return_actions=3)


def seen(pose, t, error=(0, 0, 0), tag=False, anchor=1., speed=0.):
    o = observation(pose, t, error, speed)
    o.update(tag={'ok': tag, 'id': 1 if tag else None}, anchor_stamp=anchor, tracking_generation=1,
             odom={'ok': True}, sensor_ready=True)
    return o


class RecoveryTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = config()
        cls.cfg['recovery'] = options()
        cls.policy = Policy(cls.cfg, synthesize(cls.cfg))
        key = tuple(map(int, next(k for k, v in cls.policy.table['nodes'].items() if v[0] == 1).split(',')))
        cls.pose = cls.policy.grid.belief(key).center

    def moving(self):
        c = Controller(copy.deepcopy(self.cfg), self.policy)
        c.step(1., seen(self.pose, 1., tag=True))
        c.step(1.5, seen(self.pose, 1.5, tag=True, anchor=1.5))
        self.assertEqual(c.state, 'MOVING')
        return c

    def settle(self, c, pose=None, anchor=None, start=2.):
        pose = self.pose if pose is None else pose
        for t in (start, start+.2, start+.45, start+.65, start+.85, start+1.1, start+1.3):
            c.step(t, seen(pose, t, anchor=t if anchor is None else anchor, tag=anchor is None))

    def test_dropout_stop_then_fresh_reanchor_replans_without_resetting_budgets(self):
        c = self.moving()
        c.step(1.6, None)
        self.assertEqual((c.state, c.command), ('PAUSING', 'stop'))
        self.assertIsNone(c.expected)
        self.settle(c)
        self.assertEqual(c.state, 'MOVING', c.status())
        self.assertEqual(c.recovery_attempts, 1)
        self.assertEqual(c.transitions, 2)
        self.assertEqual(c.run_started, 1.)
        self.assertGreater(c.started, 2.)
        self.assertIsNotNone(c.expected)

    def test_new_local_feature_chain_without_tag_cannot_restart(self):
        c = self.moving()
        c.step(1.6, None)
        for t in (2., 2.5, 3., 3.5):
            o = seen(self.pose, t, anchor=None)
            o.update(valid=False, bounded=False, pose=None, error=None, tracking_generation=2)
            c.step(t, o)
        self.assertEqual(c.command, 'stop')
        self.assertNotEqual(c.state, 'MOVING')

    def test_same_anchor_before_dropout_is_not_relocalization(self):
        c = self.moving()
        c.step(1.6, None)
        self.settle(c, anchor=1.)
        self.assertEqual(c.state, 'REOBSERVING')
        self.assertEqual(c.reason, 'waiting_for_absolute_tag_reanchor')

    def test_hysteresis_does_not_resume_at_pause_threshold(self):
        c = self.moving()
        error = [x*.95 for x in self.cfg['observation_error']]
        c.step(1.6, seen(self.pose, 1.6, error))
        self.assertEqual(c.state, 'PAUSING')
        for t in (2., 2.2, 2.45, 2.7, 3., 3.2):
            c.step(t, seen(self.pose, t, [x*.7 for x in self.cfg['observation_error']]))
        self.assertEqual(c.state, 'REOBSERVING')
        self.assertEqual(c.command, 'stop')
        self.settle(c, start=3.4)
        self.assertEqual(c.state, 'MOVING', c.status())

    def test_duplicates_and_long_camera_gap_do_not_fill_stability_window(self):
        c = self.moving()
        c.step(1.6, None)
        c.step(2., seen(self.pose, 2., anchor=2.))
        c.step(2.2, seen(self.pose, 2.2, anchor=2.))
        c.step(2.5, seen(self.pose, 2.5, anchor=2.5, tag=True))
        o = seen(self.pose, 2.6, anchor=2.6, tag=True)
        c.step(2.6, o)
        for t in (2.61, 2.7, 2.8): c.step(t, o)
        self.assertEqual(c.stable_count, 1)
        c.step(3.1, seen(self.pose, 3.1, anchor=3.1, tag=True))
        self.assertEqual(c.stable_count, 0)  # Gap also breaks observed-stop dwell.
        self.assertEqual(c.command, 'stop')

    def test_single_tag_detection_followed_by_odometry_is_insufficient(self):
        c = self.moving()
        c.step(1.6, None)
        c.step(2., seen(self.pose, 2., anchor=2., tag=True))
        c.step(2.2, seen(self.pose, 2.2, anchor=2., tag=False))
        c.step(2.5, seen(self.pose, 2.5, anchor=2.5, tag=True))
        for t in (2.7, 2.9, 3.1, 3.3):
            c.step(t, seen(self.pose, t, anchor=2.5))
        self.assertEqual(c.recovery_tags, 0)  # The lone sighting expires.
        self.assertEqual(c.command, 'stop')
        self.assertEqual(c.reason, 'waiting_for_absolute_tag_reanchor')

    def test_residual_motion_blocks_restart(self):
        c = self.moving()
        c.step(1.6, None)
        for t in (2., 2.3, 2.6, 2.9):
            c.step(t, seen(self.pose, t, anchor=t, speed=.1))
        self.assertEqual(c.state, 'PAUSING')
        self.assertEqual(c.stable_count, 0)

    def test_automatic_resume_allows_pending_stationary_bias_window_to_finish(self):
        c = self.moving()
        c.cfg['sensor']['imu']['stationary_recalibration'] = dict(enabled=True, verified=True)
        c.step(1.6, None)
        for t in (2., 2.2, 2.45, 2.65, 2.85, 3.1, 3.3):
            o = seen(self.pose, t, tag=True, anchor=t)
            o['imu'] = {'stationary_recalibration': {'collecting': True,
                        'result': {'applied_at_device_s': 1.}}}
            c.step(t, o)
        self.assertEqual(c.state, 'REOBSERVING', c.status())
        self.assertEqual(c.reason, 'waiting_for_stationary_gyro_calibration')
        o = seen(self.pose, 3.5, tag=True, anchor=3.5)
        # A fresh completed update remains visible even if the next window
        # has already started between camera/controller polls.
        o['imu'] = {'stationary_recalibration': {'collecting': True,
                    'result': {'applied_at_device_s': 3.4}}}
        c.step(3.5, o)
        self.assertEqual(c.state, 'MOVING', c.status())

    def test_conflicting_relocalization_is_hard_fault(self):
        c = self.moving()
        c.step(1.6, None)
        c.step(2., seen([-1., -.8, .5], 2., anchor=2.))
        self.assertEqual(c.state, 'FAULT')
        self.assertIn(c.reason, ('recovery_pose_conflicts_with_stop_envelope', 'body_uncertainty_outside_free_space'))

    def test_timeout_and_operator_fault_remain_latched(self):
        c = self.moving()
        c.step(1.6, None)
        c.step(32., None)
        self.assertEqual(c.reason, 'recovery_timeout')
        self.settle(c, start=33., anchor=33.)
        self.assertEqual(c.state, 'FAULT')
        c = self.moving()
        c.fault('operator_stop_or_ssh_eof')
        self.settle(c)
        self.assertEqual(c.state, 'FAULT')
        self.assertEqual(c.recovery_attempts, 0)

    def test_attempt_limit_and_clock_reversal_are_not_reset(self):
        c = self.moving()
        c.recovery_attempts = 2
        c.step(1.6, None)
        self.assertEqual(c.reason, 'recovery_attempt_limit')
        c = self.moving()
        c.step(1.6, seen(self.pose, 1.4))
        self.assertEqual(c.reason, 'capture_clock_reversed')

    def test_bad_hysteresis_is_rejected(self):
        c = copy.deepcopy(self.cfg)
        c['recovery']['resume_error_ratio'] = .95
        with self.assertRaises(ValueError): validate_recovery(c)

    def test_short_tag_loss_keeps_normal_motion(self):
        c = self.moving()
        c.cfg['prealignment'] = dict(max_duration_s=120.)
        c.phase, c.phase_started, c.last_tag_seen = 'SIDESTEP', 1., 1.5
        c.step(1.7, seen(self.pose, 1.7))
        self.assertEqual(c.state, 'MOVING')
        c.step(3.6, seen(self.pose, 3.6))
        self.assertEqual(c.state, 'MOVING')  # Healthy background tracking: no fixed tag timer.
        lost = seen(self.pose, 3.7)
        lost.update(valid=False, bounded=False, pose=None, error=None)
        c.step(3.7, lost)
        self.assertEqual(c.state, 'PAUSING')
        self.assertEqual(c.reason, 'observation_unavailable')


class TagReturnTests(unittest.TestCase):
    def cfg(self):
        c = config()
        c['recovery'] = options()
        c['observation_error'] = [.05, .05, .02]
        c['scene'] = dict(clearance_m=.02, free_rectangles=[[-5, 5, -5, 5]])
        c['prealignment'] = dict(enabled=True, entry_x_m=3., center_y_m=0.,
            alignment_y_m=.1, alignment_yaw_rad=.1, max_actions=8, max_duration_s=120.)
        base = copy.deepcopy(c['actions'][-1])
        base.update(name='return_right_90', target=-math.pi/2, max_start_delay_s=10.,
                    max_duration_s=20., max_stop_response_s=5., sidestep_only=True)
        c['actions'] = [base]
        c['_phase'] = 'SIDESTEP'
        return c

    def controller(self, heading=math.pi/2):
        c = self.cfg()
        if heading < 0:
            c['actions'][0].update(name='return_left_90', target=math.pi/2,
                                   command='rotate_left_slow')
        class PolicyStub:
            active_cfg = c
        ctl = Controller(c, PolicyStub())
        ctl.phase, ctl.phase_started, ctl.sidestep_heading = 'SIDESTEP', 0., 0.
        ctl.last_capture = 1.
        ctl.last_generation = 1
        ctl.last_box = Box.around([0., 0., heading], [.005, .005, .005])
        ctl.pause(1., 'sidestep_tag_absence', True)
        return ctl

    def test_lost_tag_but_live_odometry_can_return_ninety_degrees(self):
        c = self.controller()
        p = [0., 0., math.pi/2]
        c.step(1.1, seen(p, 1.1))
        c.step(1.3, seen(p, 1.3))
        c.step(1.6, seen(p, 1.6))
        self.assertEqual(c.reason, 'return_to_tag_heading', c.status())
        self.assertEqual(c.command, 'rotate_right_slow')
        self.assertAlmostEqual(c.action['target'], -math.pi/2)
        self.assertTrue(c.returning)
        # Even after rotation finishes, missing tag keeps the vehicle stopped.
        c.step(2., seen([0., 0., 0.], 2.))
        self.assertEqual(c.state, 'BRAKING')
        c.step(2.1, seen([0., 0., 0.], 2.1))
        c.step(2.6, seen([0., 0., 0.], 2.6))
        self.assertEqual(c.state, 'REOBSERVING')
        c.step(2.7, seen([0., 0., 0.], 2.7))
        self.assertEqual(c.command, 'stop')
        self.assertEqual(c.reason, 'waiting_for_absolute_tag_reanchor')

    def test_rightward_sidestep_returns_left(self):
        c = self.controller(-math.pi/2)
        for t in (1.1, 1.3, 1.6): c.step(t, seen([0., 0., -math.pi/2], t))
        self.assertEqual(c.command, 'rotate_left_slow')
        self.assertAlmostEqual(c.action['target'], math.pi/2)
        self.assertTrue(c.returning)

    def test_return_stop_keeps_host_and_device_clocks_separate(self):
        c = self.controller()
        def update(t, heading):
            o = seen([0., 0., heading], t)
            o['capture_id'] += 1000.
            c.step(t, o)
        for t in (1.1, 1.3, 1.6): update(t, math.pi/2)
        self.assertEqual(c.command, 'rotate_right_slow')
        for t in (2., 2.1, 2.6, 2.7): update(t, 0.)
        self.assertEqual(c.state, 'REOBSERVING')
        self.assertEqual(c.reason, 'waiting_for_absolute_tag_reanchor')

    def test_no_anchor_or_invalid_imu_never_searches_blindly(self):
        for anchor, imu_bad in ((None, False), (1., True)):
            c = self.controller()
            for t in (1.1, 1.6, 2., 2.2):
                o = seen([0., 0., math.pi/2], t, anchor=anchor)
                o['imu_bad'] = imu_bad
                c.step(t, o)
            self.assertEqual(c.command, 'stop')
            self.assertEqual(c.return_actions, 0)

    def test_reseeded_generation_cannot_supply_return_rotation(self):
        c = self.controller()
        for t in (1.1, 1.6, 2., 2.2):
            o = seen([0., 0., math.pi/2], t)
            o['tracking_generation'] = 2
            c.step(t, o)
        self.assertEqual(c.command, 'stop')
        self.assertEqual(c.return_actions, 0)

    def test_rotation_that_sweeps_outside_space_is_rejected(self):
        c = self.cfg()
        c['body'] = dict(front_m=.5, rear_m=.5, half_width_m=.1)
        c['scene'] = dict(clearance_m=0., free_rectangles=[[-.2, .2, -2, 2]])
        self.assertIsNone(return_rotation(Box.around([0, 0, math.pi/2], [0, 0, 0]), 0., c))

