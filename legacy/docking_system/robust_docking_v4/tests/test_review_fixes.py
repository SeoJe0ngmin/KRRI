"""Regressions for review items 1-3, using real IMU/control/policy code.

All bounds and motion below are synthetic, not forklift calibration evidence.
"""
import copy
import math
import unittest

from docking.controller import Controller
from docking.core import Box, Grid, Policy, synthesize, transition
from docking.feedback import read_imu, RotationFeedback
from docking.stream import Imu
from docking.startup import StartupSearch
from test_core import config
from test_can_transport import Shared
from test_recovery import options, seen
from test_startup import configured, sample


def fast(t, yaw=0., **kw):
    result = dict(count=1, ready=True, device_stamp_s=t, yaw_rad=yaw,
                  rate_rps=0., gaps=0, clock_reversals=0, age_s=0.)
    result.update(kw)
    return result


class ReviewRegressionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = config()
        cls.cfg['recovery'] = options()
        cls.cfg['limits']['max_age_s'] = .5
        cls.cfg['sensor']['bounds']['gyro_rate_error_rps'] = .001
        for action in cls.cfg['actions']:
            action['max_start_delay_s'] = 20.
        cls.table = synthesize(cls.cfg)
        cls.policy = Policy(cls.cfg, cls.table)

    def moving(self, sign=1, use_fast=True):
        ai = 3 if sign > 0 else 4
        key = next(k for k, v in self.table['nodes'].items() if v[0] > 0 and v[1] == ai)
        pose = list(self.policy.grid.belief(tuple(map(int, key.split(',')))).center)
        pose[2] += sign*.015  # Measured pose differs from the policy bin centre.
        c = Controller(copy.deepcopy(self.cfg), self.policy, fast_imu_required=use_fast)
        for t in (1., 1.5):
            o = seen(pose, t, tag=True, anchor=t)
            o['capture_imu'] = dict(device_stamp_s=t, yaw_rad=0., gaps=0)
            c.step(t, o, fast(t) if use_fast else None)
        self.assertEqual(c.state, 'MOVING', c.status())
        return c, pose, o

    def local(self, t, **kw):
        o = seen(None, t, anchor=None)
        o.update(valid=False, bounded=False, pose=None, error=None, tracking_generation=2,
                 motion_bounded=True, local_motion={'bounded': True})
        o.update(kw)
        return o

    def waiting_for_anchor(self):
        c, pose, _ = self.moving(use_fast=False)
        c.step(1.6, None)
        for i in range(25):
            t = 2.+i*.2
            c.step(t, self.local(t))
        self.assertEqual(c.state, 'REOBSERVING', c.status())
        self.assertEqual(c.command, 'stop')
        return c, pose

    def test_200hz_gyro_stops_both_directions_without_a_new_image(self):
        for sign in (-1, 1):
            c, pose, image = self.moving(sign)
            imu, shared = Imu(c.cfg['sensor']['imu']), Shared([0.]*8)
            imu.add(1.5, (0, 0, 0), 1.5)
            for i in range(1, 81):
                t = 1.5+i/200.
                imu.add(t, (0, -sign*.5, 0), t)
                imu.publish_fast(shared)
                c.step(t, image, read_imu(shared, t))
                if c.state == 'BRAKING':
                    break
            self.assertEqual(c.reason, 'gyro_target_braking_reserve', c.status())
            self.assertEqual(c.last_capture, 1.5)
            self.assertEqual(c.command, 'stop')
            self.assertIsNotNone(c.motion_started)
            self.assertLess(t, 1.9)
            # The target remains bin-centred; it was not shifted to pose.
            cutoff = abs(c.action['target'])-c.action['brake_reserve']
            self.assertLess(abs(imu.snapshot(t)['yaw_rad']), cutoff)
            for j in range(1, 5):
                c.step(t+j*.005, image, fast(t+j*.005, sign*.14))
                self.assertEqual((c.state, c.command), ('BRAKING', 'stop'))

    def test_capture_time_gyro_is_interpolated_not_latest_publication_angle(self):
        imu = Imu(self.cfg['sensor']['imu'])
        for i in range(21): imu.add(1.+i*.005, (0, -1, 0), 1.+i*.005)
        ref = imu.at(1.0275)
        self.assertAlmostEqual(ref['yaw_rad'], .0275)
        self.assertAlmostEqual(imu.snapshot(1.1)['yaw_rad'], .1)
        self.assertIsNone(imu.at(.9))
        imu.add(1.3, (0, -1, 0), 1.3)
        self.assertIsNone(imu.at(1.2))

    def test_delayed_image_reference_preserves_absolute_heading_target(self):
        obs = seen([0., 0., .10], 5., error=[0., 0., .001])
        obs['capture_id'] = 1005.  # Device clock is independent of host clock.
        obs['capture_imu'] = dict(yaw_rad=1., device_stamp_s=1005., gaps=0)
        monitor = RotationFeedback(self.cfg, self.cfg['actions'][3], 0., obs,
                                   fast(1005.02, 1.01))
        decision, moved = monitor.update(fast(1005.1, 1.04))
        self.assertEqual(decision, 'brake')
        self.assertTrue(moved)
        # Using the image's publication-time gyro as its reference would
        # incorrectly subtract 1.01 and report only .13 rad here.
        self.assertGreaterEqual(monitor.progress_upper(fast(1005.1, 1.04)), .14)

    def test_gap_stale_and_reversed_fast_imu_stop_with_repeated_video(self):
        for bad in (fast(1.51, gaps=1), fast(1.51, age_s=.3),
                    fast(1.49), fast(1.51, clock_reversals=1)):
            c, _, image = self.moving()
            c.step(1.51, image, bad)
            self.assertEqual(c.command, 'stop')
            self.assertIn(c.state, ('PAUSING', 'FAULT'))

    def test_missing_capture_time_reference_never_arms_rotation(self):
        c, pose, _ = self.moving()
        c = Controller(c.cfg, self.policy, fast_imu_required=True)
        for t in (1., 1.5): c.step(t, seen(pose, t), fast(t))
        self.assertEqual(c.command, 'stop')
        self.assertEqual(c.reason, 'rotation_imu_reference_unavailable')
        self.assertEqual(c.transitions, 0)

    def test_start_delay_does_not_cause_premature_braking_or_motion_timeout(self):
        c, pose, _ = self.moving()
        c.action = dict(c.action, max_duration_s=2.)
        for i in range(1, 150):
            t = 1.5+i*.1
            c.step(t, seen(pose, t), fast(t))
            self.assertEqual(c.state, 'MOVING', c.status())
            self.assertIsNone(c.motion_started)
        c.step(21.6, seen(pose, 21.6), fast(21.6))
        self.assertEqual(c.reason, 'command_start_timeout')

    def test_brake_threshold_without_motion_is_recovery_not_completed_action(self):
        c, pose, _ = self.moving(use_fast=False)
        # Defensive runtime path for a threshold inconsistent with the arm.
        c.action = dict(c.action, brake_reserve=.17)
        c.step(1.6, seen(pose, 1.6))
        self.assertEqual(c.reason, 'brake_threshold_without_observed_motion')
        self.assertEqual((c.state, c.command), ('PAUSING', 'stop'))
        self.assertIsNone(c.expected)

    def test_local_stop_survives_old_stop_deadline_without_claiming_position(self):
        c, _ = self.waiting_for_anchor()
        self.assertTrue(c.recovery_needs_tag)
        self.assertFalse(c.return_allowed)
        self.assertEqual(c.return_actions, 0)
        self.assertEqual(c.reason, 'waiting_for_absolute_tag_reanchor')
        self.assertIsNotNone(c.pause_envelope)
        self.assertEqual(c.recovery_attempts, 1)

    def test_local_stop_reanchors_with_multiple_tags_then_replans(self):
        c, pose = self.waiting_for_anchor()
        transitions = c.transitions
        c.step(7., dict(seen(pose, 7., tag=True, anchor=7.), tracking_generation=2))
        self.assertEqual(c.command, 'stop')
        for i in range(1, 7):
            t = 7.+i*.2
            o = dict(seen(pose, t, tag=True, anchor=t), tracking_generation=2)
            c.step(t, o)
            if c.state == 'MOVING': break
        self.assertEqual(c.state, 'MOVING', c.status())
        self.assertEqual(c.transitions, transitions+1)
        self.assertEqual(c.recovery_attempts, 1)

    def test_local_stop_does_not_forgive_conflicting_new_absolute_pose(self):
        c, _ = self.waiting_for_anchor()
        c.step(7., seen([-1., -.8, .5], 7., tag=True, anchor=7.))
        self.assertEqual(c.state, 'FAULT')
        self.assertEqual(c.command, 'stop')

    def test_local_stop_still_expires_at_recovery_timeout(self):
        c, _ = self.waiting_for_anchor()
        c.step(32., self.local(32.))
        self.assertEqual(c.reason, 'recovery_timeout')

    def test_local_evidence_requires_bounded_visual_motion_and_valid_speed(self):
        for changes in ({'motion_bounded': False}, {'odom': {'ok': False}},
                        {'local_motion': {'bounded': False}}, {'imu_bad': True},
                        {'speed_upper': -.1}, {'speed_upper': float('nan')},
                        {'speed_upper': None}, {'speed_upper': .1}, {'yaw_rate_upper': -.1}):
            c, _, _ = self.moving(use_fast=False)
            c.step(1.6, None)
            for i in range(1, 18):
                t = 1.6+i*.2
                c.step(t, self.local(t, **changes))
            self.assertEqual((c.state, c.command), ('FAULT', 'stop'), changes)
            self.assertEqual(c.reason, 'stop_not_observed')

    def test_duplicates_gaps_and_generation_changes_do_not_prove_local_stop(self):
        c, _, _ = self.moving(use_fast=False)
        c.step(1.6, None)
        o = self.local(2.)
        for t in (2., 2.1, 2.2, 2.3, 2.45): c.step(t, o)
        self.assertEqual(c.state, 'PAUSING')
        for t in (2.6, 3.2, 3.8): c.step(t, self.local(t))
        self.assertEqual(c.state, 'PAUSING')
        c.step(4., self.local(4., tracking_generation=3))
        c.step(4.2, self.local(4.2, tracking_generation=4))
        self.assertEqual(c.state, 'PAUSING')

    def test_lost_or_moving_stop_evidence_restarts_one_bounded_stop_wait(self):
        for changed in ({'speed_upper': .1}, {'motion_bounded': False}):
            c, _ = self.waiting_for_anchor()
            c.step(7., self.local(7., **changed))
            self.assertEqual((c.state, c.braked), ('PAUSING', 7.))
            for t in (7.2, 8., 9., 10.1): c.step(t, self.local(t, **changed))
            self.assertEqual(c.reason, 'stop_not_observed')


class BrakingGeometryTests(unittest.TestCase):
    def test_both_signs_of_translation_and_rotation_require_starting_brake_room(self):
        c = config()
        start = Grid(c).belief((5, 5, 6))
        for action in c['actions']:
            a = dict(action, brake_reserve=abs(action['target'])-.01)
            self.assertIsNone(transition(start, a, c), a['name'])
            self.assertIsNotNone(transition(start, action, c), action['name'])

    def test_original_premature_rotation_case_is_removed_from_synthesized_policy(self):
        c = config()
        for a in c['actions']:
            if a['kind'] == 'rotate': a['brake_reserve'] = .16
        table = synthesize(c)
        Policy(c, table).verify()
        self.assertTrue(table['winning_nodes'] > 0)
        self.assertFalse(any(v[0] > 0 and v[1] in (3, 4) for v in table['nodes'].values()))
        old = dict(table, algorithm='observation-game-feedback-axis-v2')
        with self.assertRaisesRegex(ValueError, 'synthesize again'): Policy(c, old)

    def test_initial_search_gyro_budget_runs_even_on_duplicate_image(self):
        c = configured()
        c['startup_search'].update(max_sweep_rad=.15, brake_reserve_rad=.02)
        search = StartupSearch(c)
        for t in (1., 1.2, 1.4, 1.6, 1.8):
            o = sample(t)
            search.step(t, o, fast(t), True)
        self.assertEqual(search.state, 'INITIAL_SEARCH')
        search.step(1.9, o, fast(1.9, .14), True)
        self.assertEqual(search.reason, 'initial_search_budget_exhausted')
        self.assertEqual(search.command, 'stop')

    def test_failed_startup_calibration_is_not_masked_by_not_ready_fast_channel(self):
        search = StartupSearch(configured())
        o = sample(1.)
        o['imu']['calibration']['state'] = 'FAILED'
        search.step(1., o, fast(1., ready=False), True)
        self.assertEqual(search.reason, 'initial_imu_calibration_failed')
        self.assertEqual(search.state, 'FAULT')
