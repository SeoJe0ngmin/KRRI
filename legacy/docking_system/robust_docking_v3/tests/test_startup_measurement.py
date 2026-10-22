"""Preparation state-machine tests; no real sensor or vehicle validation."""
import copy
import json
import unittest

from docking.controller import Controller
from docking.startup_measurement import StartupMeasurement, validate_startup_measurement
from test_startup import configured, sample


def config():
    c = configured()
    c['startup_measurement'] = dict(enabled=True, duration_s=.5, timeout_s=3., max_sample_gap_s=.25)
    return c


def observation(t, **kwargs):
    o = sample(t, **kwargs)
    o['imu'].update(gaps=0, rate_rps=.001, accel_mps2=[0., 9.81, 0.])
    o['performance'] = dict(arrival_to_result_ms=25., camera_fps=30., processed_fps=25.)
    if o['tag']['ok']:
        o['tag']['camera_translation_m'] = [0., 0., 3.]
    return o


class PreparationTests(unittest.TestCase):
    def test_requires_full_window_after_calibration_not_just_waiting(self):
        c = config()
        before = copy.deepcopy(c)
        s = StartupMeasurement(c)
        o = observation(1.)
        o['imu'].update(ready=False, calibration={'state': 'COLLECTING'})
        s.step(1., o)
        s.step(2., observation(2., tag=True))
        for t in (2.2, 2.4):
            s.step(t, observation(t, tag=True))
            self.assertFalse(s.complete)
        s.step(2.6, observation(2.6, tag=True))
        self.assertTrue(s.complete)
        report = s.report()
        self.assertFalse(report['certified_error_bound'])
        self.assertFalse(report['config_changed'])
        self.assertEqual(report['tag_camera_position_m']['samples'], 4)
        self.assertEqual(report['callback_to_result_ms']['mean'], [25.])
        json.dumps(report, allow_nan=False)
        self.assertEqual(c, before)

    def test_no_tag_does_not_invent_accuracy_or_block_later_acquisition(self):
        s = StartupMeasurement(config())
        for t in (1., 1.2, 1.4, 1.6):
            s.step(t, observation(t))
        self.assertTrue(s.complete)
        self.assertEqual(s.report()['tag_camera_position_m'], {'samples': 0})

    def test_motion_stale_sensor_gap_tracking_reset_restart_window(self):
        for fault in ('motion', 'stale', 'gyro', 'gap', 'tracking', 'odom'):
            with self.subTest(fault=fault):
                s = StartupMeasurement(config())
                s.step(1., observation(1., tag=True))
                s.step(1.2, observation(1.2, tag=True))
                o = observation(1.4)
                if fault == 'motion': o['speed_upper'] = .2
                if fault == 'stale': o['capture_lower'] = 0.
                if fault == 'gyro': o['imu']['ready'] = False
                if fault == 'gap': o['imu']['gaps'] = 1
                if fault == 'tracking': o['tracking_generation'] = 1
                if fault == 'odom': o['odom']['ok'] = False
                s.step(1.4, o)
                self.assertFalse(s.complete)
                self.assertEqual(sum(s.resets.values()), 1)
                self.assertEqual(s.report()['tag_camera_position_m']['samples'], 0)
                for t in (1.6, 1.8):
                    s.step(t, observation(t))
                    self.assertFalse(s.complete)
                for t in (2., 2.2): s.step(t, observation(t))
                self.assertTrue(s.complete, s.report())

    def test_no_evidence_no_camera_and_duplicates_cannot_release(self):
        for fault in ('missing_camera', 'no_stop_evidence', 'duplicates'):
            with self.subTest(fault=fault):
                s = StartupMeasurement(config())
                for i in range(11):
                    t = 1.+i*.1
                    o = observation(1. if fault == 'duplicates' else t, tag=True)
                    if fault == 'missing_camera': o = None
                    if fault == 'no_stop_evidence': o.pop('speed_upper')
                    s.step(t, o)
                    self.assertFalse(s.complete)
                    self.assertEqual(s.status()['command'], 'stop')

    def test_fast_imu_veto_applies_even_to_repeated_video(self):
        s = StartupMeasurement(config())
        o = observation(1.)
        s.step(1., o, angular_ok=True)
        s.step(1.01, o, angular_ok=False)
        self.assertEqual(s.status()['stationary_s'], 0.)
        self.assertEqual(s.samples, 0)

    def test_long_camera_or_supervisor_gap_starts_new_window(self):
        for device_delta in (.2, 1.):
            s = StartupMeasurement(config())
            s.step(1., observation(1.))
            o = observation(1.+device_delta)
            o['capture_lower'] = 2.
            s.step(2., o)
            self.assertEqual(s.samples, 1)
            self.assertEqual(s.status()['stationary_s'], 0.)

    def test_calibration_failure_clock_fault_timeout_are_latched(self):
        for fault in ('calibration', 'clock', 'backwards', 'host_backwards', 'timeout'):
            with self.subTest(fault=fault):
                s = StartupMeasurement(config())
                s.step(1., observation(1.))
                o, t = observation(1.2), 1.2
                if fault == 'calibration': o['imu']['calibration']['state'] = 'FAILED'
                if fault == 'clock': o['clock_fault'] = True
                if fault == 'backwards': o['capture_id'] = .9
                if fault == 'host_backwards': t = .9
                if fault == 'timeout': t, o = 4., None
                s.step(t, o)
                self.assertEqual(s.state, 'FAULT')
                s.step(4.2, observation(4.2))
                self.assertEqual(s.state, 'FAULT')
                self.assertFalse(s.complete)

    def test_report_snapshots_do_not_change_as_new_data_arrives(self):
        s = StartupMeasurement(config())
        o = observation(1., tag=True)
        can = dict(diagnostics={'max_notification_delay_s': .002})
        s.step(1., o, can_status=can)
        report = s.report()
        o['imu']['accel_mps2'][0] = 999.
        can['diagnostics']['max_notification_delay_s'] = 999.
        s.step(1.2, observation(1.2))
        self.assertEqual(report['imu_at_end']['accel_mps2'][0], 0.)
        self.assertEqual(report['can_diagnostics_at_end']['max_notification_delay_s'], .002)
        self.assertEqual(report['status']['samples'], 1)

    def test_controller_cannot_search_or_move_during_preparation(self):
        c = config()
        c['limits']['run_timeout_s'] = .1
        control = Controller(c, None)
        for t in (1., 1.2, 1.4):
            status = control.step(t, observation(t))
            self.assertEqual(status['command'], 'stop')
            self.assertIsNone(control.initial.started)
            self.assertIsNone(control.run_started)
        status = control.step(1.6, observation(1.6))
        self.assertTrue(status['startup_measurement']['complete'])
        self.assertEqual(status['command'], 'stop')
        control.step(1.8, observation(1.8))
        self.assertEqual(control.initial.started, 1.8)
        self.assertEqual(control.run_started, 1.8)

    def test_controller_failure_never_enters_search(self):
        control = Controller(config(), None)
        control.step(1., observation(1.))
        status = control.step(4., None)
        self.assertEqual((status['state'], status['command']), ('FAULT', 'stop'))
        self.assertIsNone(control.initial.started)

    def test_invalid_window_configuration_fails_before_hardware(self):
        for key, value in (('enabled', 1), ('duration_s', 0.), ('duration_s', float('nan')),
                           ('timeout_s', .5), ('max_sample_gap_s', .6)):
            c = config()
            c['startup_measurement'][key] = value
            with self.assertRaises(ValueError): validate_startup_measurement(c)


if __name__ == '__main__':
    unittest.main()
