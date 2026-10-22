"""Bias updates require independent fresh vision + neutral evidence."""
import copy
import unittest
from docking.calibration import stationary_evidence, validate_stationary_recalibration
from docking.configuration import load_config
from docking.stream import Imu
from test_core import ROOT
import os


def cfg():
    c = load_config(os.path.join(ROOT, 'configs/nano_measure.json'))['imu']
    c['startup_calibration'].update(duration_s=.1, timeout_s=2.)
    c['stationary_recalibration'] = dict(enabled=True, verified=True, duration_s=.2,
        max_visual_gap_s=.15, control_max_age_s=.3, max_speed_mps=.01,
        max_visual_yaw_rate_rps=.01, relative_yaw_error_rad=.0001, max_bias_change_rps=.005)
    return c


class StationaryUpdateTests(unittest.TestCase):
    def ready_imu(self):
        imu = Imu(cfg())
        self.feed(imu, 0., .15, (0., 0., 0.))
        self.assertTrue(imu.snapshot(.15)['ready'])
        return imu

    def feed(self, imu, start, end, gyro):
        for k in range(round(start*200), round(end*200)+1):
            t = k/200.
            imu.add(t, gyro, t)
            if k % 2 == 0: imu.add_accel(t, (0., -9.80665, 0.))

    def test_bias_reestimated_but_integrated_heading_not_zeroed(self):
        imu = self.ready_imu()
        self.feed(imu, .155, .2, (0., -.002, 0.))
        imu.observe_stationary(.2, True)
        for start, end in ((.205, .3), (.305, .4), (.405, .5)):
            self.feed(imu, start, end, (0., -.002, 0.))
            before = imu.snapshot(end)['yaw_rad']
            imu.observe_stationary(end, True)
            self.assertEqual(imu.snapshot(end)['yaw_rad'], before)
        result = imu.snapshot(.5)
        self.assertEqual(result['stationary_recalibration']['updates'], 1)
        self.assertAlmostEqual(result['effective_bias_rps'][1], -.002)
        self.assertGreater(result['yaw_rad'], 0.)
        self.feed(imu, .505, .6, (0., -.002, 0.))
        self.assertAlmostEqual(imu.between(.52, .58), 0.)

    def test_same_image_motion_or_gap_cannot_calibrate(self):
        for mode in ('duplicate', 'moving', 'gap'):
            imu = self.ready_imu()
            for i in range(2, 9):
                t = i/10.
                self.feed(imu, (i-1)/10.+.005 if i > 2 else .155, t, (0., -.002, 0.))
                stamp = .2 if mode == 'duplicate' else t
                eligible = mode != 'moving'
                if mode == 'gap' and i % 2: continue
                imu.observe_stationary(stamp, eligible)
            self.assertEqual(imu.snapshot(.8)['stationary_recalibration']['updates'], 0, mode)

    def test_bias_change_limit_is_total_not_per_update(self):
        imu = self.ready_imu()
        imu.cfg['stationary_recalibration']['max_bias_change_rps'] = .001
        for i in range(2, 7):
            self.feed(imu, .155 if i == 2 else (i-1)/10.+.005, i/10., (0., -.002, 0.))
            imu.observe_stationary(i/10., True)
        self.assertEqual(imu.snapshot(.6)['stationary_recalibration']['updates'], 0)
        self.assertEqual(imu.bias, [0., 0., 0.])

    def evidence(self):
        return dict(capture_lower=10., speed_upper=.001, visual_yaw_rate_upper=.001,
                    sensor_ready=True, imu_bad=False, odom={'ok': True})

    def test_visual_and_control_gates(self):
        c, o = cfg(), self.evidence()
        self.assertTrue(stationary_evidence(o, [10., 1., 9.], 10.1, c))
        for hint in (None, [9., 1., 8.], [10., 0., 9.], [10., 1., 10.05]):
            self.assertFalse(stationary_evidence(o, hint, 10.1, c))
        for key, value in (('speed_upper', .1), ('speed_upper', None), ('visual_yaw_rate_upper', .1),
                           ('imu_bad', True), ('sensor_ready', False), ('capture_lower', 9.)):
            bad = dict(o, **{key: value})
            self.assertFalse(stationary_evidence(bad, [10., 1., 8.], 10.1, c), key)
        bad = dict(o, odom={'ok': False})
        self.assertFalse(stationary_evidence(bad, [10., 1., 9.], 10.1, c))

    def test_unverified_bounds_cannot_arm_or_update(self):
        c = cfg()
        c['stationary_recalibration']['verified'] = False
        with self.assertRaises(ValueError): validate_stationary_recalibration(c, live=True)
        self.assertFalse(stationary_evidence(self.evidence(), [10., 1., 9.], 10.1, c))
        imu = self.ready_imu()
        imu.cfg['stationary_recalibration']['verified'] = False
        for i in range(2, 8):
            self.feed(imu, .155 if i == 2 else (i-1)/10.+.005, i/10., (0., -.002, 0.))
            imu.observe_stationary(i/10., True)
        self.assertEqual(imu.stationary_updates, 0)
