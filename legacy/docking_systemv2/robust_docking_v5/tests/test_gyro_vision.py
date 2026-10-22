"""Coordinate/time/visual failures of gyro-aided VO, without hardware."""
import copy
import json
import os
import unittest
import importlib.util
from types import SimpleNamespace
from unittest.mock import patch

AVAILABLE = importlib.util.find_spec('numpy') is not None and importlib.util.find_spec('cv2') is not None
if AVAILABLE:
    import numpy as np
    import cv2
    from docking.gyro_vision import GyroPrediction, predict_pixels, sdk_rotation
    from docking.vision import SparseOdom

from docking.stream import Imu
from support import config, ROOT


def imu_config():
    c = config()['sensor']['imu']
    c['startup_calibration']['enabled'] = False
    c.update(gyro_bias_rps=[.01, -.02, .03], gyro_scale=1., max_gap_s=.05)
    return c


def predictor(imu, R=None):
    R = np.eye(3) if R is None else R
    gyro = SimpleNamespace(get_extrinsics_to=lambda _: SimpleNamespace(rotation=R.ravel(order='F')))
    p = GyroPrediction(imu, gyro, object())
    p.clock_valid = True
    return p


@unittest.skipUnless(AVAILABLE, 'NumPy/OpenCV not installed')
class GyroVisionTests(unittest.TestCase):
    def series(self, omega=(0., .4, 0.), seconds=.3):
        imu = Imu(imu_config())
        for i in range(int(seconds*200)+1):
            imu.add(i/200., np.asarray(omega)+imu.bias)
        return imu

    def test_sdk_column_major_and_nonidentity_axes(self):
        Rcg = cv2.Rodrigues(np.array([0., 0., np.pi/2]))[0]
        np.testing.assert_allclose(sdk_rotation(SimpleNamespace(rotation=Rcg.ravel(order='F'))), Rcg)
        imu = self.series((.4, 0., 0.))
        R, info = predictor(imu, Rcg)(.0025, .2075)
        expected = cv2.Rodrigues(Rcg.dot([.4*.205, 0., 0.]))[0]
        np.testing.assert_allclose(R, expected, atol=1e-12)
        self.assertAlmostEqual(info['interval_s'], .205)

    def test_pixel_prediction_uses_opposite_scene_rotation_and_distortion(self):
        K = np.array([[400., 0, 320], [0, 410, 240], [0, 0, 1]])
        dist = np.array([.05, -.02, .001, -.002, 0.])
        rays = np.array([[0., 0., 1.], [.2, .1, 1.]])
        pts = cv2.projectPoints(rays, np.zeros(3), np.zeros(3), K, dist)[0].astype(np.float32)
        R = cv2.Rodrigues(np.array([0., .1, 0.]))[0]
        guess, valid = predict_pixels(pts, K, dist, R)
        expected = cv2.projectPoints((R.T.dot(rays.T)).T, np.zeros(3), np.zeros(3), K, dist)[0]
        np.testing.assert_allclose(guess, expected, atol=2e-5)
        self.assertTrue(np.all(valid))
        self.assertLess(guess[0, 0, 0], pts[0, 0, 0])

    def test_missing_coverage_and_internal_gap_are_not_zero_rotation(self):
        imu = self.series()
        for a, b in ((-.001, .1), (.1, .301), (.1, .1), (None, .1)):
            self.assertIsNone(predictor(imu)(a, b)[0])
        imu.add(.4, np.asarray([0., .4, 0.])+imu.bias)
        R, info = predictor(imu)(.28, .4)
        self.assertIsNone(R)
        self.assertEqual(info['reason'], 'gyro_sample_gap')

    def test_changed_clock_and_bad_extrinsics_disable_hint(self):
        p = predictor(self.series())
        p.clock_valid = False
        self.assertEqual(p(0., .1)[1]['reason'], 'clock_domains_not_matched')
        bad = predictor(self.series(), np.diag([1., 1., -1.]))
        self.assertFalse(bad.settings['available'])
        self.assertIsNone(bad(0., .1)[0])

    def test_startup_not_ready_and_reversed_clock_reject_prediction(self):
        c = imu_config()
        c['startup_calibration']['enabled'] = True
        imu = Imu(c)
        imu.add(0., c['gyro_bias_rps'])
        imu.add(.005, c['gyro_bias_rps'])
        self.assertEqual(predictor(imu)(0., .005)[1]['reason'], 'imu_not_ready')
        imu = self.series()
        imu.add(.1, imu.bias)
        self.assertEqual(predictor(imu)(0., .1)[1]['reason'], 'gyro_clock_reversal')

    def test_calibration_completion_discards_precalibration_rates(self):
        c = imu_config()
        c['startup_calibration'].update(enabled=True, duration_s=.2, timeout_s=2.)
        imu = Imu(c)
        for i in range(61):
            imu.add(i/200., c['gyro_bias_rps'])
            if i % 2 == 0:
                imu.add_accel(i/200., (0., -9.80665, 0.))
        self.assertTrue(imu.snapshot()['ready'])
        self.assertIsNone(predictor(imu)(.1, .3)[0])
        R, _ = predictor(imu)(.25, .3)
        np.testing.assert_allclose(R, np.eye(3), atol=1e-12)

    def test_back_facing_rays_are_not_projected_as_visible(self):
        pts = np.array([[[320., 240.]]], np.float32)
        K = np.array([[400., 0, 320], [0, 400, 240], [0, 0, 1]])
        R = cv2.Rodrigues(np.array([0., np.pi, 0.]))[0]
        _, valid = predict_pixels(pts, K, np.zeros(5), R)
        self.assertFalse(valid[0])


@unittest.skipUnless(AVAILABLE, 'NumPy/OpenCV not installed')
class VisualMatchingTests(unittest.TestCase):
    def setUp(self):
        cv2.setNumThreads(1)
        cv2.setRNGSeed(42)
        rng = np.random.RandomState(42)
        self.gray = cv2.GaussianBlur(rng.randint(0, 256, (480, 640), dtype=np.uint8), (5, 5), 0)
        self.K = np.array([[400., 0, 320], [0, 400, 240], [0, 0, 1]])
        self.dist = np.zeros(5)
        self.depth = np.full(self.gray.shape, 3., np.float32)
        self.cfg = copy.deepcopy(config()['sensor']['odom'])
        self.cfg.update(gyro_aided_tracking=True, max_features=120, min_spatial_bins=4)

    def odom(self, rotation):
        od = SparseOdom(self.cfg)
        od.rotation_predictor = lambda a, b: (rotation, {'reason': 'test_interval', 'interval_s': b-a})
        od.update(self.gray, self.depth, self.K, self.dist, 0.)
        return od

    def test_real_lk_pnp_with_camera_rotation(self):
        R = cv2.Rodrigues(np.array([.015, .05, -.01]))[0]
        H = self.K.dot(R.T).dot(np.linalg.inv(self.K))
        frame = cv2.warpPerspective(self.gray, H, (640, 480))
        result = self.odom(R).update(frame, self.depth, self.K, self.dist, .2)
        self.assertTrue(result['ok'], result)
        self.assertTrue(result['gyro_aid']['used'])
        self.assertLess(result['gyro_aid']['visual_disagreement_rad'], .01)
        self.assertLess(result['translation_m'], .03)
        np.testing.assert_allclose(result['T_previous_current'][:3, :3], R, atol=.01)

    def test_translation_is_measured_visually_not_set_to_zero_by_gyro(self):
        shifted = cv2.warpAffine(self.gray, np.array([[1., 0., 12.], [0., 1., 0.]]), (640, 480))
        result = self.odom(np.eye(3)).update(shifted, self.depth, self.K, self.dist, .2)
        self.assertTrue(result['ok'], result)
        self.assertAlmostEqual(result['T_previous_current'][0, 3], -.09, delta=.01)

    def test_bad_gyro_cannot_certify_stationary_image_as_rotating(self):
        R = cv2.Rodrigues(np.array([0., .15, 0.]))[0]
        result = self.odom(R).update(self.gray, self.depth, self.K, self.dist, .2)
        self.assertFalse(result['ok'], result)
        self.assertNotIn('T_previous_current', result)

    def test_no_features_never_propagates_position_using_gyro_alone(self):
        gray = np.zeros_like(self.gray)
        od = SparseOdom(self.cfg)
        od.rotation_predictor = lambda *_: (np.eye(3), {})
        od.update(gray, self.depth, self.K, self.dist, 0.)
        result = od.update(gray, self.depth, self.K, self.dist, .2)
        self.assertFalse(result['ok'])
        self.assertEqual(result['reason'], 'too_few_depth_features')

    def test_unavailable_gyro_falls_back_to_image_matching(self):
        od = self.odom(None)
        result = od.update(self.gray, self.depth, self.K, self.dist, .2)
        self.assertTrue(result['ok'], result)
        self.assertFalse(result['gyro_aid']['used'])

    def test_held_keyframe_uses_original_exposure_for_rotation(self):
        od = self.odom(np.eye(3))
        od.hold_s = .6
        calls = []
        od.rotation_predictor = lambda a, b: (calls.append((a, b)) or np.eye(3), {})
        with patch('cv2.calcOpticalFlowPyrLK', return_value=(None, None, None)):
            self.assertTrue(od.update(self.gray, self.depth, self.K, self.dist, .1)['held_reference'])
        self.assertTrue(od.update(self.gray, self.depth, self.K, self.dist, .2)['ok'])
        self.assertEqual(calls, [(0., .1), (0., .2)])

    def test_vectorized_depth_seed_matches_original_selection(self):
        rng = np.random.RandomState(5)
        depth = self.depth.copy()
        depth[rng.rand(*depth.shape) < .25] = 0.
        depth[120:140, 200:300] = 4.
        depth[320:350, 400:450] = np.nan
        self.dist = np.array([.06, -.02, .001, 0., 0.])
        pts = cv2.goodFeaturesToTrack(self.gray, self.cfg['max_features'], .01, self.cfg['feature_spacing_px'],
                                     mask=np.full(self.gray.shape, 255, np.uint8))
        expected, xyz = [], []
        for p in pts[:, 0]:
            x, y = map(int, np.round(p))
            if not (2 <= x < 638 and 2 <= y < 478):
                continue
            patch_depth = depth[y-1:y+2, x-1:x+2]
            z = patch_depth[(patch_depth > self.cfg['min_depth_m']) & (patch_depth < self.cfg['max_depth_m'])]
            if len(z) < 6 or np.ptp(z) > self.cfg['depth_patch_spread_m']:
                continue
            z = float(np.median(z))
            uv = cv2.undistortPoints(np.array([[p]], np.float64), self.K, self.dist)[0, 0]
            expected.append(p)
            xyz.append([uv[0]*z, uv[1]*z, z])
        od = SparseOdom(self.cfg)
        od.seed(self.gray, depth, self.K, self.dist, 0.)
        np.testing.assert_allclose(od.points[:, 0], expected)
        np.testing.assert_allclose(od.xyz, xyz, atol=1e-12)

