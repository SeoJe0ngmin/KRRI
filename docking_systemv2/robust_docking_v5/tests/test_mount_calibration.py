"""Geometry/sign, unsuitable measurements, and application without hardware."""
import copy
import io
import json
from pathlib import Path
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch

import cv2
import numpy as np

from docking import mount_calibration as mc
from support import config


ROOT = Path(__file__).resolve().parents[1]



MODEL = dict(serial='TEST-CAMERA', stream='infrared', width=640, height=480,
             fx=380., fy=380., cx=320., cy=240., distortion='distortion.none', coeffs=[0.]*5)


def rotation(yaw=0., pitch=0., roll=0.):
    z, y, x = [cv2.Rodrigues(np.radians(v)*np.array(axis, float))[0]
               for v, axis in [(yaw, (0, 0, 1)), (pitch, (0, 1, 0)), (roll, (1, 0, 0))]]
    return z @ y @ x @ mc.R_OPTICAL_TO_BODY


def observations(R=None):
    R = rotation(1.14, -2., .8) if R is None else R
    rv = cv2.Rodrigues(R.T @ mc.R_REFERENCE_TAG)[0].ravel().tolist()
    rows = []
    for t in np.arange(0., 65.01, .1):
        rows.append(dict(arrival=float(t), clock_fault=False,
                         imu=dict(calibration={'state': 'READY'}, age_s=.01, gaps=0, clock_reversals=0,
                                  device_stamp_s=float(t), yaw_rad=0.),
                         tag=dict(ok=True, reason='accepted', camera_rotation_vector_rad=rv,
                                  camera_translation_m=[0., -1., 3.]),
                         recording=dict(error=None, dropped=0)))
    return rows


class MountEstimationTests(unittest.TestCase):
    def estimate(self, rows=None):
        return mc.estimate(observations() if rows is None else rows, config()['sensor'], MODEL)

    def test_signed_yaw_pitch_roll_and_transform_direction(self):
        for yaw, pitch, roll in [(1.14, -2., .8), (-3., 5., -4.), (0., 0., 0.)]:
            R = rotation(yaw, pitch, roll)
            out = self.estimate(observations(R))
            self.assertTrue(out['usable'], out['reasons'])
            np.testing.assert_allclose(out['R_body_camera'], R, atol=1e-12)
            self.assertAlmostEqual(out['yaw_left_deg'], yaw)
            self.assertAlmostEqual(out['pitch_down_deg'], pitch)
            self.assertAlmostEqual(out['roll_deg'], roll)
            # Runtime absolute pose = R_dock_tag * inv(R_camera_tag) * inv(R_body_camera).
            Rct = R.T @ mc.R_REFERENCE_TAG
            np.testing.assert_allclose(mc.R_REFERENCE_TAG @ Rct.T @ np.array(out['R_body_camera']).T,
                                       np.eye(3), atol=1e-12)
            self.assertFalse(out['certified_error_bound'])

    def test_reference_pose_misalignment_is_not_observable(self):
        # This intentionally passes: camera-only statistics cannot distinguish
        # a fixed body heading error from a mounting error. Document the limit.
        out = self.estimate(observations(rotation(4.)))
        self.assertTrue(out['usable'])
        self.assertAlmostEqual(out['yaw_left_deg'], 4.)
        self.assertTrue(out['reference_alignment_confirmed'])

    def test_small_jitter_and_isolated_pose_outliers_do_not_dominate_blocks(self):
        rows = observations()
        rng = np.random.default_rng(4)
        for index, row in enumerate(rows):
            yaw = 6. if index % 30 == 0 else 1.14+rng.normal(0., .1)
            R = rotation(yaw)
            row['tag']['camera_rotation_vector_rad'] = cv2.Rodrigues(R.T @ mc.R_REFERENCE_TAG)[0].ravel().tolist()
        report = self.estimate(rows)
        self.assertTrue(report['usable'], report['reasons'])
        self.assertAlmostEqual(report['yaw_left_deg'], 1.14, delta=.03)

    def test_missing_ambiguous_short_or_sparse_tags_are_not_applied(self):
        for case in ('missing', 'ambiguous', 'short', 'sparse', 'nan', 'clock'):
            rows = observations()
            if case == 'short':
                rows = rows[:150]
            elif case == 'sparse':
                rows = [r for r in rows if not 20 < r['arrival'] < 35]
            else:
                for row in rows:
                    if case in ('missing', 'ambiguous'):
                        row['tag'].update(ok=False, reason=case)
                    elif case == 'nan':
                        row['tag']['camera_rotation_vector_rad'] = [float('nan'), 0, 0]
                    else:
                        row['clock_fault'] = True
            with self.subTest(case=case):
                self.assertFalse(self.estimate(rows)['usable'])

    def test_rejects_unready_stale_gapped_or_rotating_imu(self):
        for key in ('unready', 'stale', 'gap', 'rotate'):
            rows = observations()
            for row in rows:
                imu = row['imu']
                if key == 'unready':
                    imu['calibration']['state'] = 'FAILED'
                elif key == 'stale':
                    imu['age_s'] = 1.
                elif key == 'gap':
                    imu['gaps'] = int(row['arrival'] > 30)
                else:
                    imu['yaw_rad'] = np.radians(row['arrival'])
            with self.subTest(key=key):
                self.assertFalse(self.estimate(rows)['usable'])

    def test_rejects_slow_orientation_drift_translation_and_wrong_reference(self):
        for key in ('drift', 'translation', 'upside_down'):
            rows = observations()
            for row in rows:
                if key == 'translation':
                    row['tag']['camera_translation_m'] = [0., 0., 2.+row['arrival']*.02]
                else:
                    R = rotation(row['arrival']*.06) if key == 'drift' else rotation(0., 0., 180.)
                    row['tag']['camera_rotation_vector_rad'] = cv2.Rodrigues(R.T @ mc.R_REFERENCE_TAG)[0].ravel().tolist()
            with self.subTest(key=key):
                self.assertFalse(self.estimate(rows)['usable'])

