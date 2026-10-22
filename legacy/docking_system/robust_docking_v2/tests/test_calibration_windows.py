"""Window calibration must retain raw noise and actual motion rejection."""
import copy
import math
import os
import unittest

from docking.calibration import StationaryCalibration
from docking.configuration import load_config
from docking.diagnostics import validate_sensor_options

ROOT = os.path.dirname(os.path.dirname(__file__))


class WindowCalibrationTests(unittest.TestCase):
    def setUp(self):
        self.sensor = load_config(os.path.join(ROOT, 'configs/nano_measure.json'))
        self.cfg = self.sensor['imu']
        self.cfg['startup_calibration'].update(duration_s=2., timeout_s=5.)

    def feed(self, c, start=0, stop=450, signal=None):
        for k in range(start, stop+1):
            # A known bias plus high-frequency zero-mean variation. These
            # constructed samples test the detector, not physical accuracy.
            xyz = signal(k) if signal else (-.08+.21*(-1)**k, -.1+.19*(-1)**(k//2), 0.)
            c.add('gyro', k/200., tuple(math.radians(v) for v in xyz))
            if k % 2 == 0:
                c.add('accel', k/200., (0., -9.80665, 0.))

    def test_noise_does_not_erase_window_and_bias_uses_all_raw_values(self):
        c = StationaryCalibration(self.cfg)
        self.feed(c)
        self.assertEqual(c.state, 'READY', c.status())
        self.assertEqual(c.restarts, 0)
        self.assertGreater(math.degrees(math.sqrt(sum(v*v for v in c.result['gyro_std_rps']))), .15)
        n = c.result['gyro_samples']
        expected = [sum((-.08+.21*(-1)**k, -.1+.19*(-1)**(k//2), 0.)[i]
                        for k in range(n))/n for i in range(3)]
        for actual, value in zip(c.result['gyro_bias_rps'], expected):
            self.assertAlmostEqual(actual, math.radians(value), places=12)

    def test_single_small_peak_is_included_in_bias_not_clipped_away(self):
        c = StationaryCalibration(self.cfg)
        self.feed(c, signal=lambda k: (2. if k == 50 else 0., 0., 0.))
        self.assertEqual(c.state, 'READY', c.status())
        self.assertAlmostEqual(c.result['gyro_bias_rps'][0], math.radians(2.)/c.result['gyro_samples'])

    def test_sustained_turn_never_becomes_bias(self):
        c = StationaryCalibration(self.cfg)
        self.feed(c, stop=1100, signal=lambda k: (0., .6, 0.))
        self.assertEqual(c.state, 'FAILED')
        self.assertIsNone(c.result)
        self.assertGreater(c.restarts, 0)

    def test_large_alternating_motion_is_not_hidden_by_mean(self):
        c = StationaryCalibration(self.cfg)
        self.feed(c, signal=lambda k: ((-1)**k, 0., 0.))
        self.assertNotEqual(c.state, 'READY')
        self.assertEqual(c.reason, 'gyro_not_quiet')

    def test_slow_changes_below_rate_gate_fail_mean_stability(self):
        c = StationaryCalibration(self.cfg)
        self.feed(c, signal=lambda k: (.28*math.sin(2*math.pi*k/160.), 0., 0.))
        self.assertNotEqual(c.state, 'READY')
        self.assertEqual(c.reason, 'gyro_mean_not_stable')

    def test_large_peak_requires_a_new_full_window(self):
        c = StationaryCalibration(self.cfg)
        self.feed(c, stop=100)
        c.add('gyro', .505, (math.radians(6.), 0., 0.))
        self.assertEqual(c.reason, 'motion_during_calibration')
        self.assertEqual(c.window_means.n, 0)
        self.feed(c, start=102, stop=450)
        self.assertNotEqual(c.state, 'READY')
        self.feed(c, start=451, stop=530)
        self.assertEqual(c.state, 'READY', c.status())

    def test_gap_acceleration_and_clock_reversal_still_reject(self):
        for fault in ('gap', 'acceleration', 'reverse', 'nonfinite'):
            with self.subTest(fault=fault):
                c = StationaryCalibration(self.cfg)
                self.feed(c, stop=100)
                if fault == 'gap':
                    c.add('gyro', .6, (0., 0., 0.))
                    self.assertEqual(c.reason, 'sample_gap')
                elif fault == 'acceleration':
                    c.add('accel', .51, (0., -12., 0.))
                    self.assertEqual(c.reason, 'acceleration_during_calibration')
                elif fault == 'nonfinite':
                    c.add('gyro', .505, (float('nan'), 0., 0.))
                    self.assertEqual(c.reason, 'nonfinite_sample')
                else:
                    c.add('gyro', .49, (0., 0., 0.))
                    self.assertEqual(c.state, 'FAILED')
                if fault != 'reverse':
                    self.assertEqual(c.window_means.n, 0)
                self.assertIsNone(c.result)

    def test_duplicates_and_missing_accel_cannot_supply_evidence(self):
        c = StationaryCalibration(self.cfg)
        for _ in range(1000):
            c.add('gyro', 0., (0., 0., 0.))
        self.assertEqual(c.gyro.n, 1)
        self.assertEqual(c.window_means.n, 0)
        for k in range(1, 1100):
            c.add('gyro', k/200., (0., 0., 0.))
        self.assertEqual(c.state, 'FAILED')
        self.assertIsNone(c.result)

    def test_live_profiles_use_bench_window_without_claiming_drive_validation(self):
        from docking.profiles import readiness
        for name in ('hardware.json', 'hardware_vm.json'):
            cfg = load_config(os.path.join(ROOT, 'configs', name))
            sensor = cfg['sensor']
            c = StationaryCalibration(sensor['imu'])
            self.assertEqual(c.motion_window_s, .1)
            self.assertEqual(c.cfg['max_gyro_std_dps'], .3)
            self.feed(c, stop=100, signal=lambda k: (.6, 0., 0.))
            self.assertEqual(c.reason, 'motion_during_calibration')
            self.assertFalse(readiness(cfg)['drive_ready'])
            self.assertFalse(cfg['validation']['approved'])

    def test_invalid_window_settings_rejected(self):
        validate_sensor_options(self.sensor)
        for key, value in (('motion_window_s', 0.), ('motion_window_s', 2.),
                           ('motion_window_s', .001), ('max_gyro_peak_dps', .1),
                           ('max_gyro_peak_dps', None), ('max_gyro_window_std_dps', float('nan'))):
            with self.subTest(key=key, value=value):
                sensor = copy.deepcopy(self.sensor)
                sensor['imu']['startup_calibration'][key] = value
                with self.assertRaises(ValueError):
                    validate_sensor_options(sensor)


if __name__ == '__main__':
    unittest.main()
