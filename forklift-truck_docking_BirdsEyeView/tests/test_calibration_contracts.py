import unittest
from types import SimpleNamespace
from unittest.mock import patch
from support import config
from docking.runtime import run
from docking.sensor_calibration import model


class CalibrationContracts(unittest.TestCase):
    def test_camera_only_never_enters_drive(self):
        args=SimpleNamespace(command='drive',camera_only=True,profile='core')
        with patch('docking.camera.worker') as hardware:
            with self.assertRaisesRegex(ValueError,'camera-only'):run(args,config())
            hardware.assert_not_called()

    def test_explicit_lens_override_can_replace_unsupported_sdk_model(self):
        class Profile:
            def as_video_stream_profile(self):return self
            def get_intrinsics(self):return SimpleNamespace(model='distortion.inverse_brown_conrady',width=640,height=480,coeffs=[.1]*5)
        override=dict(serial='A',width=640,height=480,fx=380.,fy=380.,cx=320.,cy=240.,coeffs=[0.]*5,validated=True)
        with self.assertRaisesRegex(ValueError,'unsupported'):model(Profile(),'A')
        self.assertEqual(model(Profile(),'A',override)['fx'],380.)
        override['validated']=False
        with self.assertRaisesRegex(ValueError,'validated'):model(Profile(),'A',override)


if __name__=='__main__':unittest.main()
