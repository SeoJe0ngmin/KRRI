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
from docking.configuration import load_config
from docking.core import fingerprint

ROOT = Path(__file__).resolve().parents[1]


def config():
    return json.loads((ROOT/'configs/hardware.json').read_text())


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

    def test_does_not_use_previous_mounting_or_pnp_anchor(self):
        cfg = config()
        cfg['sensor']['calibration']['R_body_camera'] = rotation(9.).tolist()
        before = copy.deepcopy(cfg)
        sensor = mc.calibration_sensor(cfg)
        self.assertIsNone(sensor['calibration']['R_body_camera'])
        self.assertIsNone(sensor['calibration']['T_dock_tag'])
        self.assertFalse(sensor['odom']['enabled'])
        self.assertEqual(sensor['camera']['stream'], 'infrared')
        self.assertEqual(cfg, before)

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


class MountApplyTests(unittest.TestCase):
    def setUp(self):
        self.tmp = tempfile.TemporaryDirectory()
        self.addCleanup(self.tmp.cleanup)
        self.paths = [Path(self.tmp.name)/('v%d.json' % n) for n in (1, 2, 3)]
        self.report = mc.estimate(observations(), config()['sensor'], MODEL)
        for path in self.paths:
            cfg = config()
            cfg['experiment'] = {'folder': path.stem}
            cfg['validation'].update(approved=True, extrinsics='old evidence')
            cfg['sensor']['calibration'].update(verified=True, camera_lateral_m=.112)
            path.write_text(json.dumps(cfg))

    def test_applies_same_rotation_preserves_variants_and_invalidates_old_geometry(self):
        before = [json.loads(p.read_text()) for p in self.paths]
        written = mc.apply_report(self.report, self.paths)
        self.assertEqual(len(written), 3)
        for path, old in zip(self.paths, before):
            out = load_config(path)
            cal = out['sensor']['calibration']
            self.assertEqual(cal['R_body_camera'], self.report['R_body_camera'])
            self.assertIsNone(cal['camera_lateral_m'])
            self.assertIsNone(cal['T_body_camera'])
            self.assertIsNone(cal['R_dock_tag'])
            self.assertFalse(cal['verified'])
            self.assertFalse(out['validation']['approved'])
            self.assertIsNone(out['validation']['extrinsics'])
            self.assertEqual(out['experiment'], old['experiment'])
            self.assertEqual(out['sensor']['odom'], old['sensor']['odom'])
            self.assertEqual(out['can'], old['can'])
            self.assertEqual(out['sensor']['camera']['serial'], MODEL['serial'])
            self.assertNotEqual(fingerprint(out), fingerprint(old))

    def test_measured_lateral_builds_rotation_and_height_without_inventing_tag_pose(self):
        mc.apply_report(self.report, self.paths, lateral=-.025)
        cal = load_config(self.paths[0])['sensor']['calibration']
        self.assertEqual([r[3] for r in cal['T_body_camera'][:3]], [0., -.025, .57])
        self.assertIsNone(cal['T_dock_tag'])

    def test_stale_rotation_is_replaced_instead_of_conflicting(self):
        c = config()
        c['sensor']['calibration'].update(R_body_camera=rotation(-7).tolist(), camera_lateral_m=.01)
        c = mc.resolve_config(c)
        self.paths[0].write_text(json.dumps(c))
        mc.apply_report(self.report, self.paths, lateral=0.)
        out = load_config(self.paths[0])['sensor']['calibration']['T_body_camera']
        np.testing.assert_allclose(np.array(out)[:3, :3], self.report['R_body_camera'])

    def test_mismatched_stream_or_serial_leaves_every_file_unchanged(self):
        for key, value in [('stream', 'color'), ('serial', 'DIFFERENT')]:
            cfg = config()
            cfg['sensor']['camera'][key] = value
            self.paths[-1].write_text(json.dumps(cfg))
            before = [p.read_bytes() for p in self.paths]
            with self.assertRaises(ValueError):
                mc.apply_report(self.report, self.paths)
            self.assertEqual([p.read_bytes() for p in self.paths], before)

    def test_failed_measurement_never_writes(self):
        bad = dict(self.report, usable=False)
        before = [p.read_bytes() for p in self.paths]
        with self.assertRaises(ValueError):
            mc.apply_report(bad, self.paths)
        self.assertEqual([p.read_bytes() for p in self.paths], before)

    def test_write_failure_rolls_back_completed_files(self):
        before = [p.read_bytes() for p in self.paths]
        original = mc._atomic
        calls = 0
        def fail_second(path, data):
            nonlocal calls
            calls += 1
            if calls == 2:
                raise OSError('simulated storage failure')
            return original(path, data)
        with patch.object(mc, '_atomic', side_effect=fail_second), self.assertRaises(OSError):
            mc.apply_report(self.report, self.paths)
        self.assertEqual([p.read_bytes() for p in self.paths], before)

    def capture_run(self, interrupted):
        def measure(args):
            self.assertEqual(args.command, 'measure')
            self.assertFalse(args.execute)
            self.assertTrue(args.no_odom)
            self.assertFalse(args.no_depth)  # do not force a switch to RGB
            p = Path(args.output)
            p.mkdir()
            (p/'session.json').write_text(json.dumps(dict(elapsed_s=75., interrupted=interrupted, hardware_motion_enabled=False)))
            (p/'camera_model.json').write_text(json.dumps(MODEL))
            (p/'observations.jsonl').write_text('\n'.join(json.dumps(r) for r in observations()))
            return p
        args = SimpleNamespace(seconds=60., config=str(ROOT/'configs/nano_measure.json'),
                               aligned_reference=True, camera_lateral_m=None, apply=True,
                               apply_versions=False, view=False, port=8765, platform='nano')
        with patch.object(mc, 'WORK_ROOT', self.tmp.name), patch.object(mc, 'profile_paths', return_value=self.paths), \
                patch('docking.runtime.run', side_effect=measure), patch.object(mc, 'apply_report', return_value=[]) as apply, \
                patch('sys.stdout', new_callable=io.StringIO):
            result = mc.run(args)
            if interrupted:
                self.assertEqual(result, 2)
                apply.assert_not_called()
            else:
                self.assertEqual(result, 0)
                apply.assert_called_once()

    def test_capture_uses_measure_mode_and_interruption_does_not_apply(self):
        self.capture_run(True)

    def test_successful_capture_can_apply_with_the_same_no_CAN_path(self):
        self.capture_run(False)


if __name__ == '__main__':
    unittest.main()
