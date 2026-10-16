"""SDK doubles: selection/units/order/readback, without camera or CAN access."""
import unittest
from types import SimpleNamespace as NS

from docking.exposure import configure_exposure, frame_exposure, validate_exposure


rs = NS(stream=NS(infrared='ir', color='rgb'), camera_info=NS(name='name'),
        option=NS(**{k: k for k in ('enable_auto_exposure', 'exposure', 'gain', 'auto_exposure_priority')}),
        frame_metadata_value=NS(actual_exposure='actual', gain_level='gain', auto_exposure='auto'))


class Sensor:
    def __init__(self, stream):
        self.stream, self.writes = stream, []
        self.values = {'enable_auto_exposure': 1, 'exposure': 83, 'gain': 16, 'auto_exposure_priority': 0}
        self.read_only = set()
        self.ignore_writes = False

    def get_stream_profiles(self):
        return [NS(stream_type=lambda: self.stream)]

    def supports(self, key):
        return key in self.values

    def is_option_read_only(self, key):
        return key in self.read_only

    def get_option_range(self, key):
        limits = {'enable_auto_exposure': (0, 1), 'exposure': (1, 10000),
                  'gain': (0, 128), 'auto_exposure_priority': (0, 1)}
        lo, hi = limits[key]
        return NS(min=lo, max=hi, step=1)

    def set_option(self, key, value):
        self.writes.append((key, value))
        if not self.ignore_writes:
            self.values[key] = value

    def get_option(self, key):
        return self.values[key]


class ExposureTests(unittest.TestCase):
    def setUp(self):
        self.ir, self.rgb = Sensor('ir'), Sensor('rgb')
        self.device = NS(get_info=lambda _: 'Intel RealSense D435I',
                         query_sensors=lambda: [self.rgb, self.ir])

    def camera(self, stream='infrared', auto=False, exp=8300, gain=16):
        return dict(stream=stream, auto_exposure=auto, exposure_us=exp, gain=gain)

    def test_ir_manual_applies_to_stereo_and_disables_auto_first(self):
        report = configure_exposure(self.device, self.camera(), rs)
        self.assertEqual(self.ir.writes, [('enable_auto_exposure', 0), ('exposure', 8300), ('gain', 16)])
        self.assertEqual(self.rgb.writes, [])
        self.assertEqual(report['readback_exposure_us_at_start'], 8300)

    def test_color_option_has_100us_ticks_and_does_not_touch_stereo(self):
        report = configure_exposure(self.device, self.camera(stream='color'), rs)
        self.assertEqual(self.rgb.writes[1], ('exposure', 83))
        self.assertEqual(self.ir.writes, [])
        self.assertEqual(report['readback_exposure_us_at_start'], 8300)

    def test_auto_does_not_write_manual_controls(self):
        c = self.camera(auto=True, exp=None, gain=None)
        configure_exposure(self.device, c, rs)
        self.assertEqual(self.ir.writes, [('enable_auto_exposure', 1)])

    def test_bad_config_rejected_before_device_changes(self):
        for c in (self.camera(auto=True), self.camera(exp=None), self.camera(exp=float('nan')),
                  self.camera(gain=True), self.camera(exp=0), self.camera(auto=1)):
            with self.assertRaises(ValueError):
                configure_exposure(self.device, c, rs)
        self.assertEqual(self.ir.writes, [])

    def test_out_of_range_off_step_and_read_only_rejected_before_any_write(self):
        for c in (self.camera(exp=10001), self.camera(stream='color', exp=8350), self.camera(gain=200)):
            with self.assertRaises(ValueError):
                configure_exposure(self.device, c, rs)
        self.ir.read_only.add('gain')
        with self.assertRaises(ValueError):
            configure_exposure(self.device, self.camera(), rs)
        self.assertEqual(self.ir.writes, [])
        self.assertEqual(self.rgb.writes, [])

    def test_unapplied_setting_is_not_reported_as_success(self):
        self.ir.ignore_writes = True
        with self.assertRaises(RuntimeError):
            configure_exposure(self.device, self.camera(), rs)

    def test_priority_is_optional_and_unsupported_request_is_explicit_failure(self):
        c = self.camera(auto=True, exp=None, gain=None)
        c['auto_exposure_priority'] = False
        del self.ir.values['auto_exposure_priority']
        with self.assertRaises(ValueError):
            configure_exposure(self.device, c, rs)
        self.assertEqual(self.ir.writes, [])

    def test_unknown_model_not_assumed_to_use_d435_units(self):
        self.device.get_info = lambda _: 'another camera'
        with self.assertRaises(ValueError):
            configure_exposure(self.device, self.camera(), rs)

    def test_manual_gain_may_be_left_unchanged_but_is_reported(self):
        report = configure_exposure(self.device, self.camera(gain=None), rs)
        self.assertEqual(self.ir.writes, [('enable_auto_exposure', 0), ('exposure', 8300)])
        self.assertEqual(report['readback_sdk_units_at_start']['gain'], 16)

    def test_frame_metadata_is_already_microseconds(self):
        values = {'actual': 8300, 'gain': 16, 'auto': 1}
        frame = NS(supports_frame_metadata=lambda key: key in values,
                   get_frame_metadata=lambda key: values[key])
        self.assertEqual(frame_exposure(frame, rs),
                         {'actual_exposure_us': 8300, 'gain': 16, 'auto_exposure': True})
        values.clear()
        self.assertEqual(frame_exposure(frame, rs),
                         {'actual_exposure_us': None, 'gain': None, 'auto_exposure': None})

    def test_requires_explicit_mode(self):
        with self.assertRaises(ValueError):
            validate_exposure({'stream': 'infrared'})


if __name__ == '__main__':
    unittest.main()
