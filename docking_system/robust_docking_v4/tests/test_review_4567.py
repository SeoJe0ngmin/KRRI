"""Fault injection for review 4–7; synthetic bounds are NOT vehicle calibration."""
import copy
import math
import types
import unittest
from unittest.mock import patch
try:
    import numpy as np
    import cv2
    from docking.vision import Observer, SparseOdom, WindowMotion
except ImportError:
    np = None
from docking.motion import velocity_bound, validate_motion, stopped
from docking.recovery import return_observation, return_config, return_space_available, tag_heading, return_rotation
from docking.startup import StartupSearch
from docking.canhealth import Diagnostics, guard
from docking.exposure import recovery_camera_settings
from test_core import config, Box
from test_startup import configured, sample
from test_recovery import seen
import test_recovery


def sensor():
    c = config()['sensor']
    c['tag']['enabled'] = False
    c['calibration'].update(verified=True, T_body_camera=np.eye(4).tolist(), T_dock_tag=np.eye(4).tolist())
    c['bounds'].update(verified=True, tag_pose_error=[.01]*3, odom_error_per_m=[.001]*3,
        odom_error_per_frame=[.00001]*3, odom_error_per_rad=[.001]*3,
        gyro_rate_error_rps=.001, relative_translation_error_m=.003, max_acceleration_mps2=.01,
        capture_to_callback_max_s=.02)
    c['tracking_recovery'] = dict(enabled=True, verified=True, hold_keyframe_s=.6,
        max_bridge_s=.3, max_episode_s=4., max_unobserved_speed_mps=.5)
    return c


def window_options():
    return dict(enabled=True, verified=True, min_s=.15, max_s=.75, reference_period_s=.15,
                max_frame_gap_s=.3, translation_error_m=.001, yaw_error_rad=.0001,
                max_angular_acceleration_rps2=.001)


@unittest.skipIf(np is None, 'NumPy/OpenCV not installed')
class StopWindowTests(unittest.TestCase):
    def test_pair_noise_floor_and_window_acceleration_bound(self):
        self.assertGreater(velocity_bound(0., .003, 1/30, .01), .02)
        self.assertLess(velocity_bound(0., .003, .5, .01), .02)
        self.assertGreater(velocity_bound(.015, .003, .5, .01), .02)  # creep
        self.assertGreater(velocity_bound(0., .003, .5, 1.), .02)  # cancellation/acceleration

    def test_impossible_stop_criterion_is_rejected_before_drive(self):
        s = sensor(); s['stop_window'] = window_options()
        validate_motion(s, config()['limits'], live=True)
        s['stop_window']['translation_error_m'] = .2
        with self.assertRaisesRegex(ValueError, 'velocity floor'):
            validate_motion(s, config()['limits'], live=True)

    def test_missing_window_cannot_fall_back_and_age_expands_speed(self):
        o = sample(1., tag=True)
        o['stop_motion'] = dict(bounded=False, speed_upper=0., acceleration_bound=1.)
        self.assertFalse(stopped(o, 1., config()['limits']))
        o['stop_motion'].update(bounded=True, speed_upper=.001)
        self.assertTrue(stopped(o, 1., config()['limits']))
        self.assertFalse(stopped(o, 1.1, config()['limits']))

    def test_direct_baseline_not_sum_of_noisy_frame_pairs(self):
        s = sensor(); s['stop_window'] = window_options()
        w = WindowMotion(s)
        gray = np.zeros((20, 20), np.uint8)
        depth = np.ones((20, 20))
        def seed(ref, gray, depth, K, dist, t):
            ref.last_stamp = t
        def match(ref, gray, depth, K, dist, t):
            T = np.eye(4); T[0, 3] = .001  # Direct baseline error, not .001 per frame.
            return dict(ok=True, T_previous_current=T)
        with patch.object(SparseOdom, 'seed', seed), patch.object(SparseOdom, 'update', match):
            for i in range(31):
                result = w.update(gray, depth, np.eye(3), np.zeros(5), i/30., np.eye(4), True)
            self.assertTrue(result['bounded'])
            self.assertLess(result['speed_upper'], .02)
            self.assertLessEqual(len(w.references), 6)
            missing = w.update(gray, depth, np.eye(3), np.zeros(5), 1.1, np.eye(4), False)
            self.assertFalse(missing['bounded'])
            restart = w.update(gray, depth, np.eye(3), np.zeros(5), 1.2, np.eye(4), True)
            self.assertFalse(restart['bounded'])
            duplicate = w.update(gray, depth, np.eye(3), np.zeros(5), 1.2, np.eye(4), True)
            self.assertFalse(duplicate['bounded'])


@unittest.skipIf(np is None, 'NumPy/OpenCV not installed')
class TrackingRecoveryTests(unittest.TestCase):
    def observer(self):
        o = Observer(sensor())
        class Odom:
            ok, generation, local = True, 0, np.eye(4)
            last_stamp = None
            def update(self, *args):
                reference, self.last_stamp = self.last_stamp, args[-1]
                return dict(ok=self.ok, reference_stamp=reference, reason='tracked' if self.ok else 'too_few_depth_features',
                    generation=self.generation, dt=.1, T_local_camera=self.local.copy(),
                    T_previous_current=np.eye(4), translation_m=0., rotation_rad=0.)
        class Tag:
            ok = True
            def update(self, *args):
                return dict(ok=self.ok, reason='synthetic', T_camera_tag=np.eye(4))
        o.odom, o.tag = Odom(), Tag()
        return o

    def frame(self, o, t, dyaw=0.):
        o.test_yaw = getattr(o, 'test_yaw', 0.)+(dyaw or 0.)
        ref = dict(device_stamp_s=t, yaw_rad=o.test_yaw, gaps=0) if dyaw is not None else None
        return o.process(np.zeros((10, 10), np.uint8), np.ones((10, 10)), np.eye(3), np.zeros(5), t, dyaw, capture_imu=ref)

    def test_dropout_preserves_bounded_coordinates_and_reseed_retains_error(self):
        o = self.observer()
        first = self.frame(o, 1.)
        o.tag.ok = o.odom.ok = False
        held = self.frame(o, 1.1)
        self.assertTrue(held['bounded'])
        self.assertTrue(held['tracking_recovery']['bridge_ok'])
        self.assertGreaterEqual(held['error'][0], first['error'][0]+.049)
        o.odom.generation = 1
        reseed = self.frame(o, 1.2)
        self.assertTrue(reseed['tracking_recovery']['coarse'])
        o.odom.ok = True
        resumed = self.frame(o, 1.3)
        self.assertGreaterEqual(resumed['error'][0], reseed['error'][0])
        self.assertEqual(resumed['anchor_stamp'], 1.)
        expired = self.frame(o, 5.2)
        self.assertFalse(expired['bounded'])
        o.tag.ok = True
        fresh = self.frame(o, 5.4)
        self.assertTrue(fresh['bounded'])
        self.assertFalse(fresh['tracking_recovery']['coarse'])

    def test_lost_imu_cannot_bridge_visual_loss(self):
        o = self.observer(); self.frame(o, 1.)
        o.tag.ok = o.odom.ok = False
        self.assertFalse(self.frame(o, 1.1, None)['bounded'])

    def test_short_lk_failure_retains_keyframe_then_expires(self):
        s = sensor(); cfg = dict(s['odom'], hold_keyframe_s=.3)
        o = SparseOdom(cfg)
        gray = np.zeros((20, 20), np.uint8)
        o.prev, o.last_stamp = gray.copy(), 0.
        o.points, o.xyz = np.ones((20, 1, 2), np.float32), np.ones((20, 3))
        with patch('cv2.calcOpticalFlowPyrLK', return_value=(None, None, None)), patch.object(o, 'seed'):
            a = o.update(gray, None, np.eye(3), np.zeros(5), .1)
            self.assertTrue(a['held_reference'])
            self.assertEqual(o.last_stamp, 0.)
            self.assertEqual(o.generation, 0)
            b = o.update(gray, None, np.eye(3), np.zeros(5), .4)
            self.assertFalse(b['ok'])
            self.assertEqual(o.generation, 1)


class BlindReturnTests(unittest.TestCase):
    def controller(self):
        ctl = test_recovery.TagReturnTests().controller()
        ctl.cfg['sensor']['bounds']['gyro_rate_error_rps'] = .001
        ctl.recovery['blind_return'] = dict(enabled=True, verified=True, stop_rest_after_s=.8,
                                           rest_speed_upper_mps=.01, max_heading_error_rad=.15)
        ctl.return_reference = dict(pose=[0., 0., math.pi/2], error=[.005]*3,
            imu=dict(yaw_rad=0., device_stamp_s=1., gaps=0), anchor_stamp=1.)
        return ctl

    def lost(self, t, yaw=0., gaps=0):
        o = seen([0, 0, math.pi/2+yaw], t)
        o.update(valid=False, bounded=False, pose=None, error=None,
                 capture_imu=dict(yaw_rad=yaw, device_stamp_s=t, gaps=gaps))
        o['odom']['ok'] = False
        return o

    def test_returns_with_no_features_using_coarse_tube_then_waits_for_tag(self):
        ctl = self.controller()
        for t in (1.1, 1.3, 1.6): ctl.step(t, self.lost(t))
        self.assertEqual(ctl.command, 'stop')  # Stop contract has not elapsed.
        for t in (1.9, 2.1, 2.4): ctl.step(t, self.lost(t))
        self.assertTrue(ctl.returning, ctl.status())
        self.assertEqual(ctl.command, 'rotate_right_slow')
        ctl.step(2.6, self.lost(2.6, -math.pi/2))
        self.assertEqual(ctl.command, 'stop')
        for t in (2.9, 3.2, 3.5, 3.7, 4., 4.2): ctl.step(t, self.lost(t, -math.pi/2))
        self.assertEqual(ctl.command, 'stop')
        self.assertTrue(ctl.recovery_needs_tag)
        self.assertEqual(ctl.return_actions, 1)
        self.assertNotEqual(ctl.state, 'DONE')

    def test_both_directions_return_without_background_precision(self):
        for direction in (-1, 1):
            ctl = self.controller()
            heading = direction*math.pi/2
            ctl.return_reference['pose'][2] = heading
            if direction < 0:
                ctl.cfg['actions'][0].update(target=math.pi/2, command='rotate_left_slow')
                ctl.pause_envelope = Box.around([0., 0., heading], [.04, .04, .02])
            for t in (1.1, 1.3, 1.6, 1.9, 2.1, 2.4):
                ctl.step(t, self.lost(t))
            self.assertEqual(ctl.command, 'rotate_right_slow' if direction > 0 else 'rotate_left_slow')
            self.assertTrue(ctl.returning)

    def test_return_stop_preserves_large_gyro_error_until_tag_reacquisition(self):
        ctl = self.controller()
        ctl.return_reference['error'][2] = .12  # Larger than the docking yaw budget.
        for t in (1.1, 1.3, 1.6, 1.9, 2.1, 2.4): ctl.step(t, self.lost(t))
        self.assertTrue(ctl.returning, ctl.status())
        self.assertGreater(ctl.expected.radius[2], .12)
        ctl.step(2.6, self.lost(2.6, -math.pi/2))
        for t in (2.9, 3.2, 3.5, 3.7, 4., 4.2): ctl.step(t, self.lost(t, -math.pi/2))
        self.assertEqual(ctl.state, 'REOBSERVING', ctl.status())
        self.assertGreaterEqual(ctl.pause_envelope.radius[2], .12)
        self.assertTrue(ctl.recovery_needs_tag)
        self.assertEqual(ctl.command, 'stop')

    def test_return_widens_bounds_without_changing_configured_command(self):
        ctl = self.controller()
        original = copy.deepcopy(ctl.cfg['actions'][0])
        action, end = return_rotation(Box.around([0., 0., math.pi/2], [.01, .01, .12]), 0., ctl.cfg)
        self.assertEqual(ctl.cfg['actions'][0], original)
        for key in ('command', 'target', 'brake_reserve', 'max_duration_s'):
            self.assertEqual(action[key], original[key])
        self.assertGreater(end.radius[2], .12)
        self.assertIsNone(return_rotation(Box.around([0., 0., math.pi/2], [.01, .01, .16]), 0., ctl.cfg))

    def test_unmatched_imu_timestamp_does_not_authorize_return(self):
        ctl = self.controller()
        for t in (1.1, 1.3, 1.6, 1.9, 2.1, 2.4):
            o = self.lost(t)
            o['capture_imu']['device_stamp_s'] = t-.1
            ctl.step(t, o)
        self.assertEqual(ctl.command, 'stop')
        self.assertEqual(ctl.return_actions, 0)

    def test_return_space_is_reserved_before_the_lateral_leg(self):
        ctl = self.controller()
        ctl.recovery['reserve_return_sweep'] = True
        b = Box.around([0., 0., math.pi/2], [.005]*3)
        self.assertTrue(return_space_available(b, ctl.cfg['actions'][0], ctl.cfg))
        ctl.cfg['scene']['free_rectangles'] = [[-.01, .01, -.1, .1]]
        self.assertFalse(return_space_available(b, ctl.cfg['actions'][0], ctl.cfg))

    def test_gyro_gap_or_missing_qualification_prevents_blind_turn(self):
        for kind in ('gap', 'unverified'):
            ctl = self.controller()
            if kind == 'unverified': ctl.recovery['blind_return']['verified'] = False
            for t in (1.1, 1.3, 1.6, 1.9, 2.1, 2.4): ctl.step(t, self.lost(t, gaps=int(kind == 'gap')))
            self.assertEqual(ctl.command, 'stop')
            self.assertEqual(ctl.return_actions, 0)

    def test_tight_space_still_refuses_rotation(self):
        ctl = self.controller()
        ctl.cfg['scene']['free_rectangles'] = [[-.01, .01, -.01, .01]]
        for t in (1.1, 1.3, 1.6, 1.9, 2.1, 2.4): ctl.step(t, self.lost(t))
        self.assertEqual(ctl.command, 'stop')
        self.assertEqual(ctl.return_actions, 0)

    def test_return_bearing_changes_after_lateral_motion(self):
        c = config(); c['sensor']['calibration'].update(verified=True,
            T_body_camera=[[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, .5], [0, 0, 0, 1]],
            T_dock_tag=[[1, 0, 0, 2], [0, 1, 0, 1], [0, 0, 1, 1], [0, 0, 0, 1]])
        self.assertAlmostEqual(tag_heading(Box.around([0, 0, 0], [0]*3), c, 0), math.atan2(1, 2))
        self.assertAlmostEqual(tag_heading(Box.around([0, 1, 0], [0]*3), c, 0), 0.)


class StartupImprovementTests(unittest.TestCase):
    def search(self):
        c = configured()
        c['startup_search']['improvement'] = dict(enabled=True, retry_wait_s=.4, max_attempts=3,
            center_tolerance_rad=.05, max_center_turn_rad=.35, max_saturated_fraction=.4, max_dark_fraction=.8)
        return StartupSearch(c)

    def test_tag_coasts_out_then_search_resumes_with_existing_budget(self):
        s = self.search(); s.step(1., sample(1., tag=True))
        for t in (1.2, 1.4, 1.6, 1.8, 2., 2.2):
            s.step(t, sample(t))
        self.assertEqual(s.state, 'INITIAL_SEARCH', s.status())
        self.assertEqual(s.started, 1.)
        self.assertEqual(s.retries, 1)

    def test_imu_gap_while_remeasuring_prevents_a_new_search_command(self):
        s = self.search()
        for t in (1., 1.2, 1.4, 1.6, 1.8): s.step(t, sample(t))
        self.assertEqual(s.state, 'INITIAL_SEARCH')
        s.step(2., sample(2., tag=True))
        gap_in_measurement = False
        for i in range(1, 15):
            t = 2.+i*.2
            if s.state == 'INITIAL_MEASURE': gap_in_measurement = True
            o = sample(t)
            o['imu']['gaps'] = int(gap_in_measurement)
            s.step(t, o)
            self.assertEqual(s.command, 'stop', s.status())
        self.assertTrue(gap_in_measurement)
        self.assertEqual(s.reason, 'initial_imu_clock_or_gap')

    def test_edge_detection_centers_then_brakes_without_pose_acceptance(self):
        s = self.search()
        def edge(t, yaw=0.):
            o = sample(t, yaw=yaw)
            o['tag'] = dict(detected=True, ok=False, id=1, reason='clipped', bearing_body_rad=.3)
            return o
        s.step(1., edge(1.))
        for t in (1.2, 1.4, 1.6, 1.8, 2., 2.2): s.step(t, edge(t))
        self.assertEqual(s.state, 'INITIAL_CENTERING', s.status())
        self.assertEqual(s.command, 'rotate_left_slow')
        s.step(2.4, edge(2.4, .22))
        self.assertEqual(s.state, 'INITIAL_BRAKING', s.status())
        self.assertFalse(s.complete)

    def test_ambiguity_is_not_accepted_after_repeated_waiting(self):
        s = self.search()
        for i in range(30):
            t = 1.+i*.2
            o = sample(t)
            o['tag'] = dict(detected=True, ok=False, id=1, reason='reprojection_or_planar_ambiguity')
            s.step(t, o)
            self.assertEqual(s.command, 'stop')
        self.assertEqual(s.state, 'FAULT')
        self.assertIn('observation_unresolved', s.reason)

    def test_ae_request_requires_neutral_fresh_command_and_explicit_permission(self):
        camera = dict(allow_startup_auto_exposure=True, auto_exposure=False, exposure_us=100.)
        self.assertIsNotNone(recovery_camera_settings(camera, [1, 1.], [1., 1., .5], 1.1, 0))
        self.assertIsNone(recovery_camera_settings(camera, [1, 1.], [1., 0., .5], 1.1, 0))
        self.assertIsNone(recovery_camera_settings(camera, [1, 1.], [1., 1., .5], 2., 0))
        self.assertIsNone(recovery_camera_settings(camera, [1, 1.], [1., 1., .5], 1.1, 1))


class CanHealthTests(unittest.TestCase):
    def setup_probe(self):
        from enum import IntFlag
        class Flag(IntFlag):
            TXACK=64; ERROR_FRAME=32; OVERRUN=1536
        class Stat(IntFlag):
            BUS_OFF=1; ERROR_PASSIVE=2
        class NoMsg(Exception): pass
        api = types.SimpleNamespace(MessageFlag=Flag, Stat=Stat, CanNoMsg=NoMsg)
        class Channel:
            def __init__(self):
                self.rx, self.sent = [], []
                self.iocontrol = types.SimpleNamespace(flush_tx_buffer=lambda: self.sent.clear())
                self.status = 0
            def write(self, frame): self.sent.append(frame)
            def read(self, timeout=0):
                if not self.rx: raise NoMsg()
                return self.rx.pop(0)
            def requestChipStatus(self): pass
            def readStatus(self): return self.status
            def read_error_counters(self): return types.SimpleNamespace(tx=0, rx=0, overrun=0)
        c = dict(max_ack_age_s=1., max_write_gap_s=1., poll_budget_s=.001,
                 status_period_s=.1, max_rx_per_poll=4, max_pending=8)
        channel = Channel()
        return Diagnostics(channel, api, c), channel, Flag

    def frame(self, data=None, flags=0, id_=0x1E3):
        return types.SimpleNamespace(id=id_, data=data or [127]*8, flags=flags)

    def test_ack_is_not_actual_motion_and_rx_budget_is_bounded(self):
        d,ch,flags=self.setup_probe()
        with patch('docking.canhealth.time.monotonic', return_value=1.):
            d.write(self.frame())
            ch.rx = [self.frame(flags=flags.TXACK)] + [self.frame(id_=0x123)]*10
            r=d.poll()
            self.assertEqual(r['txack_count'],1)
            self.assertTrue(r['transmission_is_not_actuation'])
            self.assertEqual(len(ch.rx),7)

    def test_ack_timeout_foreign_sender_busoff_overrun_are_errors(self):
        for kind in ('timeout','foreign','busoff','overrun'):
            d,ch,flags=self.setup_probe()
            with patch('docking.canhealth.time.monotonic', return_value=1.): d.write(self.frame())
            if kind=='foreign': ch.rx=[self.frame()]
            if kind=='busoff': ch.status=1
            if kind=='overrun': ch.rx=[self.frame(flags=flags.OVERRUN)]
            with patch('docking.canhealth.time.monotonic',return_value=3. if kind=='timeout' else 1.1):
                with self.assertRaises(RuntimeError): d.poll()

    def test_runtime_guard_rejects_fork_and_other_modes(self):
        for frame in (self.frame([127,127,127,127,140,127,127,127]),
                      self.frame([127,147,97,127,127,127,127,127]),
                      self.frame([0x42,0,0,0x15,10,0x40,0x69,0x93], id_=0x2E3)):
            with self.assertRaises(RuntimeError): guard(frame)
        guard(self.frame())
        guard(self.frame([127,147,127,127,127,127,127,127]))


@unittest.skipIf(np is None, 'NumPy/OpenCV not installed')
class RealImageMotionTests(unittest.TestCase):
    def test_real_klt_pnp_stationary_and_creep_images(self):
        # Analytic pinhole plane at 2 m: 6 px/s * 2/300 = .04 m/s.
        # Actual OpenCV tracking and pose solvers, no motion estimator mock.
        gray = np.random.RandomState(73).randint(0, 255, (240, 320)).astype(np.uint8)
        gray = cv2.GaussianBlur(gray, (5, 5), 0)
        depth = np.full(gray.shape, 2., np.float32)
        K = np.array([[300., 0, 160], [0, 300., 120], [0, 0, 1]])
        for speed in (0., 6.):
            s = sensor(); s['stop_window'] = window_options()
            s['odom']['exclude_normalized_rects'] = []
            w = WindowMotion(s)
            for i in range(13):
                t = i*.1
                im = cv2.warpAffine(gray, np.array([[1., 0, t*speed], [0, 1, 0]]), (320, 240))
                result = w.update(im, depth, K, np.zeros(5), t, np.eye(4), True)
            self.assertTrue(result['bounded'], result)
            if speed == 0: self.assertLess(result['speed_upper'], .02)
            else: self.assertGreater(result['speed_upper'], .035)


class ControllerBridgeTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not hasattr(test_recovery.RecoveryTests, "cfg"):
            test_recovery.RecoveryTests.setUpClass()

    def test_short_gap_keeps_existing_action_but_does_not_invent_progress(self):
        helper = test_recovery.RecoveryTests()
        helper.setUp()
        c = helper.moving()
        o = seen(helper.pose, 1.7)
        o['tracking_recovery'] = dict(predicting=True, coarse=False, bridge_ok=True)
        c.step(1.7, o)
        self.assertEqual(c.state, 'MOVING')
        self.assertEqual(c.reason, 'bounded_tracking_bridge')
        self.assertIsNone(c.motion_started)
        o.update(capture_id=1.8, capture_lower=1.8)
        o['tracking_recovery']['bridge_ok'] = False
        c.step(1.8, o)
        self.assertEqual(c.state, 'PAUSING')
        self.assertEqual(c.reason, 'tracking_bridge_exhausted')
        self.assertEqual(c.command, 'stop')

    def test_short_gap_near_stop_target_brakes_early(self):
        helper = test_recovery.RecoveryTests(); helper.setUp(); c = helper.moving()
        o = seen(c.expected.center, 1.7)
        o['tracking_recovery'] = dict(predicting=True, coarse=False, bridge_ok=True)
        c.step(1.7, o)
        self.assertEqual(c.command, 'stop')
        self.assertEqual(c.state, 'PAUSING')

    def test_return_only_model_cannot_enable_stationary_gyro_calibration(self):
        from docking.calibration import stationary_evidence
        ctl = BlindReturnTests().controller()
        o = BlindReturnTests().lost(2.)
        o['sensor_ready'] = True
        o = return_observation(ctl, 2., o)
        c = dict(stationary_recalibration=dict(enabled=True, verified=True, control_max_age_s=1.,
                 max_visual_gap_s=1., max_speed_mps=.02, max_visual_yaw_rate_rps=.01))
        self.assertFalse(stationary_evidence(o, [2., 1., 0.], 2., c))
