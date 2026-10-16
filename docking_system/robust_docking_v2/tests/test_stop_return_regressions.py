"""Second review 1-4: timed sensor/recovery regressions, without hardware.

Every numerical motion/stop bound here is synthetic, not vehicle evidence.
"""
import copy
import math
import unittest
import numpy as np

from docking.controller import Controller
from docking.core import Box
from docking.motion import AngularStopGate
from docking.startup import StartupSearch
from docking.vision import Observer, SparseOdom
from test_core import config, observation
import test_recovery
from test_recovery import seen
from test_review_fixes import fast
import test_review_4567
from test_review_4567 import sensor, window_options
from test_startup import configured, sample


class LatestGyroStopTests(unittest.TestCase):
    def controller(self):
        cfg = config()
        cfg['observation_error'] = [.06]*3
        cfg['sensor']['bounds']['gyro_rate_error_rps'] = .001
        cfg['sensor']['stop_window'] = window_options()
        class GoalPolicy:
            active_cfg = cfg
            def lookup(self, *args):
                return dict(rank=0)
        return Controller(cfg, GoalPolicy(), fast_imu_required=True)

    def obs(self, ctl, t):
        p = Box(ctl.cfg['goal']['lo'], ctl.cfg['goal']['hi']).center
        o = observation(p, t)
        o['stop_motion'] = dict(bounded=True, speed_upper=.001, acceleration_bound=.01)
        return o

    def test_latest_rotation_vetoes_delayed_quiet_image_and_goal(self):
        c = self.controller()
        c.step(1., self.obs(c, 1.), fast(1.))
        c.step(1.5, self.obs(c, 1.4), fast(1.5, rate_rps=.2))
        self.assertEqual(c.state, 'WAITING')
        self.assertIsNone(c.still_since)
        self.assertEqual(c.command, 'stop')
        for t in (1.6, 1.8, 2.1):
            c.step(t, self.obs(c, t), fast(t))
        self.assertEqual(c.state, 'DONE')

    def test_motion_on_duplicate_tick_cancels_dwell_before_new_image(self):
        for change in ('motion', 'gap', 'missing', 'stale'):
            with self.subTest(change=change):
                c = self.controller()
                o = self.obs(c, 1.)
                c.step(1., o, fast(1.))
                bad = dict(motion=fast(1.2, rate_rps=.2), gap=fast(1.2, gaps=1),
                           missing=None, stale=fast(1., age_s=.2))[change]
                c.step(1.2, o, bad)
                self.assertIsNone(c.still_since)
                for t in (1.3, 1.5):
                    c.step(t, self.obs(c, t), fast(t, gaps=int(change == 'gap')))
                self.assertNotEqual(c.state, 'DONE')
                c.step(1.8, self.obs(c, 1.8), fast(1.8, gaps=int(change == 'gap')))
                self.assertEqual(c.state, 'DONE')

    def test_aged_gyro_needs_acceleration_bound(self):
        c = self.controller()
        gate = AngularStopGate(c.cfg)
        self.assertTrue(gate.update(fast(1., age_s=.05), True))
        c.cfg['sensor']['stop_window']['max_angular_acceleration_rps2'] = 1.
        self.assertFalse(gate.update(fast(1., age_s=.05), True))
        c.cfg['sensor']['stop_window']['verified'] = False
        self.assertFalse(gate.update(fast(1., age_s=.001), True))

    def test_startup_repeated_image_cannot_hide_residual_rotation(self):
        s = StartupSearch(configured())
        for t in (1., 1.2, 1.4, 1.6, 1.8):
            s.step(t, sample(t, tag=True), fast(t), True)
        self.assertGreater(s.tag_samples, 0)
        s.step(1.85, sample(1.8, tag=True), fast(1.85, rate_rps=.2), True)
        self.assertEqual(s.tag_samples, 0)
        self.assertIsNone(s.still_since)
        for t in (2., 2.2, 2.4):
            s.step(t, sample(t, tag=True), fast(t), True)
        self.assertFalse(s.complete)
        for t in (2.6, 2.8, 3., 3.2):
            s.step(t, sample(t, tag=True), fast(t), True)
        self.assertTrue(s.complete)


class PairedReferenceTests(unittest.TestCase):
    class Odom:
        """Controllable match outcomes, with SparseOdom's keyframe lifecycle."""
        def __init__(self):
            self.prev = self.last_stamp = None
            self.generation = 0
            self.local = np.eye(4)
            self.mode, self.angle = 'track', 0.

        def seed(self, gray, depth, K, dist, stamp):
            self.prev, self.last_stamp = gray.copy(), stamp

        def update(self, gray, depth, K, dist, stamp):
            ref = self.last_stamp
            ok = self.prev is not None and self.mode == 'track'
            held = self.prev is not None and self.mode == 'hold'
            if self.prev is not None and self.mode == 'reseed':
                self.generation += 1
                self.local = np.eye(4)
            delta = np.eye(4)
            co, si = math.cos(self.angle), math.sin(self.angle)
            delta[:2, :2] = [[co, -si], [si, co]]
            if ok:
                self.local = self.local.dot(delta)
            if not held:
                self.seed(gray, depth, K, dist, stamp)
            return dict(ok=ok, held_reference=held, reference_stamp=ref,
                        reason='tracked' if ok else 'initializing', generation=self.generation,
                        dt=stamp-ref if ref is not None else None,
                        T_local_camera=self.local.copy(), T_previous_current=delta,
                        translation_m=0., rotation_rad=abs(self.angle) if ok else 0.)

    def observer(self):
        o = Observer(sensor())
        o.odom = self.Odom()
        return o

    def frame(self, o, t, yaw=0., delta=0., gaps=0, missing=False, quality=None, offset=0.):
        ref = None if missing else dict(device_stamp_s=t+offset, yaw_rad=yaw, gaps=gaps)
        return o.process(np.zeros((20, 20), np.uint8), np.ones((20, 20)),
                         np.eye(3), np.zeros(5), t, delta, quality, capture_imu=ref)

    def test_mismatch_then_reseed_does_not_reuse_discarded_rotation(self):
        o = self.observer()
        self.frame(o, 1.)
        bad = self.frame(o, 1.1, yaw=.2, delta=.2)
        self.assertEqual(bad['odom']['reason'], 'vision_gyro_disagree')
        self.assertIsNone(o.keyframe_imu)
        self.frame(o, 1.2, yaw=.2)  # Actual new seed, retaining global gyro yaw.
        recovered = self.frame(o, 1.3, yaw=.2)
        self.assertTrue(recovered['imu_consistent'])
        self.assertTrue(recovered['motion_bounded'])
        self.assertIsNone(recovered['anchor_stamp'])
        self.assertFalse(recovered['bounded'])  # Relative recovery isn't a new dock pose.
        self.assertEqual(o.keyframe_imu['yaw_rad'], .2)

    def test_held_visual_reference_uses_entire_gyro_interval(self):
        o = self.observer()
        self.frame(o, 1.)
        o.odom.mode = 'hold'
        self.frame(o, 1.1, yaw=.12, delta=.12)
        self.assertEqual(o.keyframe_imu['device_stamp_s'], 1.)
        o.odom.mode, o.odom.angle = 'track', .24
        r = self.frame(o, 1.2, yaw=.24, delta=.12)
        self.assertTrue(r['imu_consistent'])
        self.assertEqual(o.keyframe_imu['device_stamp_s'], 1.2)

    def test_blur_invalidates_both_references(self):
        o = self.observer()
        self.frame(o, 1.)
        self.frame(o, 1.1, yaw=.2, delta=.2, quality={'reject_motion_blur': True})
        self.assertIsNone(o.keyframe_imu)
        self.frame(o, 1.2, yaw=.2)
        self.assertTrue(self.frame(o, 1.3, yaw=.2)['imu_consistent'])

    def test_missing_misaligned_and_gap_endpoints_never_prove_motion(self):
        for kw in ({'missing': True}, {'offset': -.01}, {'gaps': 1}):
            with self.subTest(kw=kw):
                o = self.observer()
                self.frame(o, 1.)
                self.assertFalse(self.frame(o, 1.1, **kw)['motion_bounded'])

    def test_tag_on_held_reference_reseeds_matching_gyro_endpoint(self):
        o = self.observer()
        self.frame(o, 1.)
        o.odom.mode = 'hold'
        class Tag:
            def update(self, *args):
                return dict(ok=True, id=1, T_camera_tag=np.eye(4))
        o.tag = Tag()
        self.frame(o, 1.1, yaw=.2, delta=.2)
        self.assertEqual(o.keyframe_imu['device_stamp_s'], 1.1)
        o.tag, o.odom.mode = None, 'track'
        r = self.frame(o, 1.2, yaw=.2)
        self.assertTrue(r['imu_consistent'])
        self.assertEqual(r['anchor_stamp'], 1.1)


class TagReacquisitionTests(unittest.TestCase):
    def test_brief_absolute_tag_can_seed_a_new_chain_for_return_only(self):
        c = test_recovery.TagReturnTests().controller()
        p = [0., 0., math.pi/2]
        for t in (1.1, 1.3, 1.6, 1.8, 2.1):
            o = seen(p, t, tag=t == 1.6, anchor=1.6 if t >= 1.6 else 1.)
            o['tracking_generation'] = 2 if t >= 1.6 else 1
            c.step(t, o)
        self.assertTrue(c.returning, c.status())
        self.assertTrue(c.recovery_needs_tag)
        self.assertEqual(c.command, 'rotate_right_slow')

    def test_one_sighting_does_not_permanently_disable_return(self):
        c = test_recovery.TagReturnTests().controller()
        p = [0., 0., math.pi/2]
        for t in (1.1, 1.3, 1.6):
            c.step(t, seen(p, t, tag=t == 1.6, anchor=1.6 if t == 1.6 else 1.))
        self.assertEqual(c.command, 'stop')
        for t in (1.8, 2.1):
            c.step(t, seen(p, t, anchor=1.6))
        self.assertTrue(c.returning, c.status())
        self.assertEqual(c.recovery_tags, 0)
        self.assertEqual(c.recovery_attempts, 1)
        self.assertEqual(c.recovery_started, 1.)

    def moving_return(self):
        c = test_recovery.TagReturnTests().controller()
        for t in (1.1, 1.3, 1.6):
            c.step(t, seen([0., 0., math.pi/2], t))
        self.assertTrue(c.returning)
        return c

    def test_return_sighting_stops_early_then_can_resume_if_lost(self):
        c = self.moving_return()
        heading = math.pi/2-.05
        o = seen([0., 0., heading], 1.8, anchor=1.)
        # A decoded ID is enough to STOP, even when pose is ambiguous.
        o['tag'] = dict(ok=False, detected=True, id=1, reason='ambiguous')
        c.step(1.8, o)
        self.assertEqual(c.reason, 'sidestep_tag_detected')
        self.assertEqual(c.command, 'stop')
        self.assertIsNone(c.expected)  # Early STOP cannot use the old endpoint contract.
        for t in (2., 2.2, 2.5):
            c.step(t, seen([0., 0., heading], t))
        self.assertTrue(c.returning, c.status())
        self.assertEqual(c.return_actions, 2)
        self.assertEqual(c.recovery_attempts, 1)
        self.assertEqual(c.recovery_started, 1.)

    def test_other_tag_and_turn_out_do_not_trigger_return_stop(self):
        for mode in ('wrong_id', 'turn_out', 'lateral'):
            with self.subTest(mode=mode):
                c = self.moving_return()
                if mode != 'wrong_id':
                    c.returning = False
                    c.sidestep_leg = 'TURN_OUT' if mode == 'turn_out' else 'LATERAL_MOVE'
                    if mode == 'lateral':
                        c.action = copy.deepcopy(c.action)
                        c.action.update(kind='translate', target=.5, command='forward_slow')
                o = seen([0., 0., math.pi/2], 1.8, tag=True, anchor=1.8)
                o['tag']['id'] = 9 if mode == 'wrong_id' else 1
                c.step(1.8, o)
                self.assertEqual(c.state, 'MOVING', c.status())

    def test_intermittent_tags_cannot_accumulate_for_restart(self):
        c = test_recovery.TagReturnTests().controller()
        c.return_allowed = False  # Isolate the remeasurement gate.
        p = [0., 0., math.pi/2]
        anchor = 1.
        for i in range(25):
            t = 1.1+i*.2
            present = i in (3, 8, 13, 18, 23)
            if present:
                anchor = t
            c.step(t, seen(p, t, tag=present, anchor=anchor))
            self.assertLess(c.recovery_tags, c.recovery['min_tag_samples'])
            self.assertEqual(c.command, 'stop')

    def test_early_sighting_does_not_renew_return_or_time_budget(self):
        c = self.moving_return()
        c.recovery['max_return_actions'] = 1
        p = [0., 0., math.pi/2-.05]
        c.step(1.8, seen(p, 1.8, tag=True, anchor=1.8))
        for t in (2., 2.2, 2.5, 2.7, 3.):
            c.step(t, seen(p, t, anchor=1.8))
        self.assertEqual(c.return_actions, 1)
        self.assertEqual(c.command, 'stop')
        c.step(31.1, None)
        self.assertEqual(c.reason, 'recovery_timeout')


class ModelStopProofTests(unittest.TestCase):
    def controller(self):
        c = test_review_4567.BlindReturnTests().controller()
        c.fast_imu_required = True
        c.cfg['limits']['max_age_s'] = .5
        c.recovery['max_sample_gap_s'] = .5
        c.cfg['sensor']['stop_window'] = window_options()
        return c

    def test_actual_no_feature_reseeds_at_three_fps_do_not_starve_stop(self):
        c = self.controller()
        od = SparseOdom(dict(sensor()['odom'], hold_keyframe_s=.6))
        gray, depth = np.zeros((40, 40), np.uint8), np.ones((40, 40))
        latest, generations = None, set()
        frame = -1
        for tick in range(301):
            now = 1.+tick/100.
            f = int((now-1.)*3+1e-9)
            if f != frame:
                frame = f
                stamp = 1.+f/3.
                measured = od.update(gray, depth, np.eye(3), np.zeros(5), stamp)
                generations.add(measured['generation'])
                latest = test_review_4567.BlindReturnTests().lost(stamp)
                latest['odom'] = measured
                latest['tracking_generation'] = measured['generation']
            c.step(now, latest, fast(now))
            if c.returning or c.state == 'FAULT':
                break
        self.assertGreaterEqual(len(generations), 3)
        self.assertTrue(c.returning, c.status())
        self.assertLess(now, 4.)
        self.assertEqual(c.command, 'rotate_right_slow')
        self.assertTrue(c.recovery_needs_tag)

    def test_visual_proof_still_resets_on_generation_change(self):
        c = test_recovery.TagReturnTests().controller()
        for i in range(12):
            t = 1.1+i*.2
            o = seen([0., 0., math.pi/2], t)
            o['tracking_generation'] = i
            c.step(t, o)
        self.assertEqual(c.command, 'stop')
        self.assertEqual(c.return_actions, 0)

    def test_return_completion_with_reseeds_keeps_waiting_for_absolute_tag(self):
        c = self.controller()
        for i, t in enumerate((1.1, 1.4, 1.8, 2.1, 2.4)):
            o = test_review_4567.BlindReturnTests().lost(t)
            o['tracking_generation'] = i
            c.step(t, o, fast(t))
        self.assertTrue(c.returning, c.status())
        for i in range(8):
            t = 2.5+i/3.
            o = test_review_4567.BlindReturnTests().lost(t, -math.pi/2)
            o['tracking_generation'] = i+10
            c.step(t, o, fast(t, yaw=-math.pi/2))
        self.assertEqual(c.state, 'REOBSERVING', c.status())
        self.assertEqual(c.command, 'stop')
        self.assertTrue(c.recovery_needs_tag)
        self.assertEqual(c.return_actions, 1)

    def test_model_proof_obeys_latest_gyro_and_duplicate_tick_veto(self):
        c = self.controller()
        for t in (1.1, 1.4, 1.8, 2.):
            c.step(t, test_review_4567.BlindReturnTests().lost(t), fast(t))
        c.step(2.1, test_review_4567.BlindReturnTests().lost(2.), fast(2.1, rate_rps=.2))
        self.assertIsNone(c.stop_since)
        for t in (2.2, 2.4):
            c.step(t, test_review_4567.BlindReturnTests().lost(t), fast(t))
            self.assertEqual(c.command, 'stop')
        c.step(2.7, test_review_4567.BlindReturnTests().lost(2.7), fast(2.7))
        self.assertTrue(c.returning, c.status())

    def test_model_stop_never_releases_ordinary_approach(self):
        c = self.controller()
        c.phase = 'APPROACH'
        for t in (1.1, 1.4, 1.8, 2., 2.3, 2.6):
            c.step(t, test_review_4567.BlindReturnTests().lost(t), fast(t))
        self.assertEqual(c.command, 'stop')
        self.assertEqual(c.return_actions, 0)
        self.assertNotEqual(c.state, 'DONE')
