import copy
import json
import os
import tempfile
import threading
import time
import unittest
from unittest.mock import patch

from docking.calibration import StationaryCalibration
from docking.configuration import load_config
from docking.diagnostics import StillnessReport
from docking.recording import Recorder
from docking.stream import Imu

ROOT = os.path.dirname(os.path.dirname(__file__))


class CalibrationTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(os.path.join(ROOT, 'configs/nano_measure.json'))['imu']
        self.cfg['startup_calibration'].update(duration_s=.2, timeout_s=2.)

    def samples(self, cal, start=0., end=.25, gyro=(.001, -.002, .0001), accel=(0., -9.80665, 0.)):
        for i in range(round(start*200), round(end*200)+1):
            t = i/200.
            cal.add('gyro', t, gyro)
            if i % 2 == 0:
                cal.add('accel', t, accel)

    def test_unique_timed_samples_calibrate_bias_not_driving_response(self):
        c = StationaryCalibration(self.cfg)
        self.samples(c)
        self.assertEqual(c.state, 'READY', c.status())
        for a, b in zip(c.result['gyro_bias_rps'], (.001, -.002, .0001)):
            self.assertAlmostEqual(a, b)
        self.assertEqual(c.result['yaw_axis'], [0., -1., 0.])
        self.assertFalse(c.result['yaw_sign_verified'])

    def test_repeated_samples_cannot_fill_window(self):
        c = StationaryCalibration(self.cfg)
        for _ in range(1000):
            c.add('gyro', 0., (0., 0., 0.))
            c.add('accel', 0., (0., -9.80665, 0.))
        self.assertEqual(c.state, 'COLLECTING')
        self.assertEqual(c.gyro.n, 1)

    def test_motion_restarts_window_and_bias_is_not_fit_during_turn(self):
        c = StationaryCalibration(self.cfg)
        self.samples(c, end=.1)
        c.add('gyro', .105, (0., -.1, 0.))
        self.assertEqual(c.reason, 'motion_during_calibration')
        self.samples(c, start=.11, end=.25)
        self.assertNotEqual(c.state, 'READY')
        self.samples(c, start=.255, end=.4)
        self.assertEqual(c.state, 'READY')

    def test_missing_accel_and_wrong_gravity_axis_never_ready(self):
        c = StationaryCalibration(self.cfg)
        for i in range(501):
            c.add('gyro', i/200., (0., 0., 0.))
        self.assertEqual(c.state, 'FAILED')
        c = StationaryCalibration(self.cfg)
        self.samples(c, accel=(9.80665, 0., 0.))
        self.assertEqual(c.reason, 'configured_yaw_axis_inconsistent_with_gravity')

    def test_imu_integrates_with_measured_bias_after_calibration(self):
        imu = Imu(self.cfg)
        for i in range(51):
            imu.add(i/200., (.001, -.002, .0001), i/200.)
            if i % 2 == 0:
                imu.add_accel(i/200., (0., -9.80665, 0.))
        self.assertTrue(imu.snapshot(.25)['ready'])
        self.assertAlmostEqual(imu.between(.22, .24), 0.)

    def test_jitter_report_is_not_an_error_bound(self):
        d = StillnessReport({'window_s': 1, 'max_samples': 20})
        for i in range(10):
            d.add(i/10., [1+i/1000., 2., 3.])
        d.add(.9, [99, 99, 99])
        r = d.report()
        self.assertEqual(r['n'], 10)
        self.assertGreater(r['std_m'][0], 0)
        self.assertFalse(r['certified_error_bound'])


class RecordingTests(unittest.TestCase):
    def wait_until(self, check):
        deadline = time.monotonic()+2
        while not check() and time.monotonic() < deadline:
            time.sleep(.005)
        self.assertTrue(check())

    def test_flush_depends_on_time_with_only_one_record(self):
        with tempfile.TemporaryDirectory() as folder:
            r = Recorder(folder, {'flush_s': .02})
            try:
                r.put({'one_frame': True})
                path = os.path.join(folder, 'observations.jsonl')
                self.wait_until(lambda: os.path.exists(path) and os.path.getsize(path) > 0)
                with open(path) as f:
                    self.assertEqual(json.loads(f.readline()), {'one_frame': True})
            finally:
                r.close()

    def test_capacity_stops_recording_without_throwing_at_producer(self):
        with tempfile.TemporaryDirectory() as folder:
            r = Recorder(folder, {'max_bytes': 30, 'flush_s': .01})
            r.put({'payload': 'x'*100})
            self.wait_until(lambda: r.error is not None)
            self.assertFalse(r.put({'next': 1}))
            self.assertIn('size_limit', r.close()['error'])

    def test_disk_error_is_visible_without_camera_exception(self):
        with patch('builtins.open', side_effect=OSError('disk failure')):
            r = Recorder('/unused', {})
            self.wait_until(lambda: r.error is not None)
            self.assertFalse(r.put({'frame': 1}))
            self.assertIn('disk failure', r.close()['error'])

    def test_slow_writer_cannot_block_producer(self):
        entered, release = threading.Event(), threading.Event()
        real_dumps = json.dumps
        def slow(*args, **kwargs):
            entered.set()
            release.wait(2)
            return real_dumps(*args, **kwargs)
        with tempfile.TemporaryDirectory() as folder, patch('docking.recording.json.dumps', side_effect=slow):
            r = Recorder(folder, {'queue_size': 1})
            try:
                r.put({'first': 1})
                self.assertTrue(entered.wait(1))
                self.assertTrue(r.put({'queued': 2}))
                self.assertFalse(r.put({'overflow': 3}))
            finally:
                release.set()
                r.close()
            self.assertEqual(r.status()['dropped'], 1)
