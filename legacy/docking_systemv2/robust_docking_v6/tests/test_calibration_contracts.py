"""Calibration validity and run-mode guards, without devices or ROS."""
import copy
import unittest
from types import SimpleNamespace
from unittest.mock import patch
import cv2
import numpy as np
from support import config
from docking.mount_calibration import estimate, R_REFERENCE_TAG, R_OPTICAL_TO_BODY
from docking.calibration_tools import intrinsics
from docking.evaluate import evaluate
from docking.runtime import run
from docking.sensor_calibration import model


class CalibrationContracts(unittest.TestCase):
    def test_fixed_reference_mount_and_vibration_rejection(self):
        cfg=config()['sensor'];model_info=cfg['calibration']['camera_model']
        Rct=R_OPTICAL_TO_BODY.T@R_REFERENCE_TAG
        rvec=cv2.Rodrigues(Rct)[0].ravel().tolist()
        rows=[dict(arrival=10+i/5,stationary=True,vio_gyro_norm=.002,
                   imu=dict(age_s=.01,calibration=dict(state='READY')),
                   tag=dict(ok=True,reason='accepted',camera_rotation_vector_rad=rvec,camera_translation_m=[0,1,3]))
              for i in range(306)]
        report=estimate(rows,cfg,model_info,60)
        self.assertTrue(report['usable'],report['reasons'])
        np.testing.assert_allclose(report['R_body_camera'],R_OPTICAL_TO_BODY,atol=1e-8)
        vibrating=copy.deepcopy(rows)
        for r in vibrating[20:80]:r['stationary']=False
        report=estimate(vibrating,cfg,model_info,60)
        self.assertFalse(report['usable']);self.assertIn('visual_stationarity_not_confirmed',report['reasons'])

    def test_bad_mount_interval_does_not_start_camera(self):
        from docking.preflight import mount
        with patch('docking.runtime.run') as hardware:
            with self.assertRaises(ValueError):mount(SimpleNamespace(seconds=float('nan')))
            hardware.assert_not_called()

    def test_camera_only_never_enters_drive(self):
        args=SimpleNamespace(command='drive',camera_only=True,profile='core')
        with patch('docking.camera.worker') as hardware:
            with self.assertRaisesRegex(ValueError,'camera-only'):run(args,config())
            hardware.assert_not_called()

    def test_constant_bias_is_not_hidden_by_trajectory_alignment(self):
        truth=[dict(device_stamp=i/10,x_m=i/10,y_m=0,yaw_rad=0) for i in range(10)]
        estimates=[dict(capture_id=i/10,pose=[i/10,.1,0],covariance=(np.eye(3)*.001).tolist()) for i in range(10)]
        report=evaluate(estimates,truth)
        self.assertAlmostEqual(report['ate_position_rmse_m'],.1)
        self.assertAlmostEqual(report['pose_repeatability_std'][1],0)
        self.assertAlmostEqual(report['fork_lateral_abs_p95_m'],.1)

    def test_explicit_lens_override_can_replace_unsupported_sdk_model(self):
        class Profile:
            def as_video_stream_profile(self):return self
            def get_intrinsics(self):return SimpleNamespace(model='distortion.inverse_brown_conrady',width=640,height=480,coeffs=[.1]*5)
        override=dict(serial='A',width=640,height=480,fx=380.,fy=380.,cx=320.,cy=240.,coeffs=[0.]*5,validated=True)
        with self.assertRaisesRegex(ValueError,'unsupported'):model(Profile(),'A')
        self.assertEqual(model(Profile(),'A',override)['fx'],380.)
        override['validated']=False
        with self.assertRaisesRegex(ValueError,'validated'):model(Profile(),'A',override)

    def test_invalid_checkerboard_geometry_is_not_calibrated(self):
        with self.assertRaisesRegex(ValueError,'square size'):
            intrinsics(SimpleNamespace(columns=9,rows=6,square_m=-.025))


if __name__=='__main__':unittest.main()
