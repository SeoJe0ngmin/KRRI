import os
import threading
import unittest
from types import SimpleNamespace

from docking.camera import configure_device_clock, receive_motion
from docking.configuration import load_config
from docking.stream import Imu


ROOT = os.path.dirname(os.path.dirname(__file__))
RS = SimpleNamespace(
    stream=SimpleNamespace(gyro='gyro', accel='accel'),
    timestamp_domain=SimpleNamespace(hardware_clock='hardware_clock'),
    option=SimpleNamespace(global_time_enabled='global_time_enabled'),
    camera_info=SimpleNamespace(name='name'))


class Sensor:
    def __init__(self, streams, supported=True, ignores_write=False):
        self.streams, self.supported = streams, supported
        self.ignores_write, self.value, self.writes = ignores_write, 1., []

    def get_stream_profiles(self):
        return [SimpleNamespace(stream_type=lambda s=s: s) for s in self.streams]

    def supports(self, option):
        return self.supported

    def set_option(self, option, value):
        self.writes.append(value)
        if not self.ignores_write:
            self.value = value

    def get_option(self, option):
        return self.value

    def get_info(self, info):
        return '/'.join(self.streams)


class MotionFrame:
    def __init__(self, kind, stamp, domain='hardware_clock'):
        self.kind, self.stamp, self.domain = kind, stamp, domain
        self.timestamp_reads = 0

    def is_motion_frame(self):
        return True

    def as_motion_frame(self):
        return self

    def get_profile(self):
        return SimpleNamespace(stream_type=lambda: self.kind)

    def get_frame_timestamp_domain(self):
        return self.domain

    def get_motion_data(self):
        v = (.001, -.002, .0001) if self.kind == 'gyro' else (0., -9.80665, 0.)
        return SimpleNamespace(x=v[0], y=v[1], z=v[2])

    def get_timestamp(self):
        self.timestamp_reads += 1
        return self.stamp*1000.


class Shared(list):
    def __init__(self):
        super().__init__([0.]*8)
        self.lock = threading.Lock()

    def get_lock(self):
        return self.lock


class ClockTests(unittest.TestCase):
    def setUp(self):
        cfg = load_config(os.path.join(ROOT, 'configs/nano_measure.json'))['imu']
        cfg['startup_calibration'].update(duration_s=.2, timeout_s=2.)
        self.imu = Imu(cfg)
        self.domains = {'gyro': None, 'accel': None, 'fault': None}
        self.fast = Shared()

    def receive(self, kind, stamp, domain='hardware_clock'):
        frame = MotionFrame(kind, stamp, domain)
        receive_motion(frame, RS, self.imu, self.domains, self.fast)
        return frame

    def calibrate(self):
        for i in range(51):
            self.receive('gyro', i/200.)
            if i % 2 == 0:
                self.receive('accel', i/200.)

    def test_only_resolved_sensors_configured_and_read_back(self):
        motion, video = Sensor(['gyro', 'accel']), Sensor(['infrared', 'depth'])
        device = SimpleNamespace(query_sensors=lambda: [motion, video])
        profile = SimpleNamespace(get_device=lambda: device,
                                  get_streams=motion.get_stream_profiles)
        settings = configure_device_clock(profile, RS)
        self.assertEqual(motion.writes, [0.])
        self.assertEqual(video.writes, [])
        self.assertEqual(settings[0]['global_time_enabled'], 0.)
        # Configuring the video pipeline must not change a running IMU sensor.
        profile.get_streams = video.get_stream_profiles
        configure_device_clock(profile, RS)
        self.assertEqual(motion.writes, [0.])
        self.assertEqual(video.writes, [0.])

    def test_failed_clock_option_write_is_not_silently_accepted(self):
        sensor = Sensor(['gyro'], ignores_write=True)
        profile = SimpleNamespace(
            get_device=lambda: SimpleNamespace(query_sensors=lambda: [sensor]),
            get_streams=sensor.get_stream_profiles)
        with self.assertRaisesRegex(RuntimeError, 'timestamp conversion'):
            configure_device_clock(profile, RS)

    def test_sensor_without_option_still_requires_valid_frame_domain(self):
        sensor = Sensor(['gyro'], supported=False)
        profile = SimpleNamespace(
            get_device=lambda: SimpleNamespace(query_sensors=lambda: [sensor]),
            get_streams=sensor.get_stream_profiles)
        self.assertFalse(configure_device_clock(profile, RS)[0]['global_time_supported'])
        self.assertEqual(sensor.writes, [])
        self.receive('gyro', 1000000000., 'system_time')
        self.assertEqual(self.imu.count, 0)
        self.assertFalse(self.imu.snapshot()['ready'])

    def test_device_time_calibrates_with_two_different_sample_rates(self):
        self.calibrate()
        state = self.imu.snapshot()
        self.assertTrue(state['ready'], state)
        self.assertEqual(state['calibration']['state'], 'READY')
        self.assertAlmostEqual(self.imu.between(.22, .24), 0.)
        self.assertIsNone(self.domains['fault'])
        self.assertEqual(self.fast[7], 1.)

    def test_mixed_startup_domains_never_enter_calibration_window(self):
        self.receive('gyro', .005)
        first = self.imu.calibration.first
        bad = self.receive('accel', 1000000000., 'global_time')
        self.assertEqual(bad.timestamp_reads, 0)
        self.assertEqual(self.imu.calibration.first, first)
        self.assertEqual(self.imu.calibration.accel.n, 0)
        self.assertIn('accel=global_time', self.domains['fault'])
        self.assertEqual(self.imu.snapshot()['calibration']['state'], 'FAILED')
        # A transient mismatch cannot silently restore readiness later.
        self.receive('gyro', .01)
        self.receive('accel', .01)
        self.assertEqual(self.imu.count, 1)
        self.assertEqual(self.fast[7], 0.)

    def test_domain_change_after_ready_invalidates_fast_control_feedback(self):
        self.calibrate()
        history = list(self.imu.history)
        angle = self.imu.angle
        self.receive('accel', 1000000000., 'system_time')
        self.assertFalse(self.imu.snapshot()['ready'])
        self.assertEqual(self.fast[7], 0.)
        self.assertEqual(list(self.imu.history), history)
        self.assertEqual(self.imu.angle, angle)


if __name__ == '__main__':
    unittest.main()
