"""Synthetic sensor sequences; no camera, CAN, or physical accuracy claims."""
import itertools
import importlib.util
import json
import math
from pathlib import Path
import random
import unittest

from docking.estimation import drift_budget, fuse_tag, tag_quality, validate_estimation


def sensor_options():
    return {'camera': {'fps': 30}, 'tag': {'min_tag_px': 20, 'min_margin': 20,
                    'max_reproj_px': 2., 'max_ambiguity_ratio': .8},
            'tag_fusion': {'enabled': True, 'max_tag_weight': [.5]*3},
            'bounds': {'odom_error_per_m': [.01]*3, 'odom_error_per_rad': [.02]*3,
                       'odom_error_per_frame': [.001]*3}}


class BoundedFusionTests(unittest.TestCase):
    def test_recentered_box_contains_every_raw_tag_corner(self):
        rng = random.Random(7)
        for _ in range(100):
            prior = [rng.uniform(-1, 1) for _ in range(3)]
            z = [rng.uniform(-1, 1) for _ in range(3)]
            pe, ze = [rng.random() for _ in range(3)], [rng.random() for _ in range(3)]
            x, e, report = fuse_tag(prior, pe, z, ze, {}, sensor_options())
            for signs in itertools.product((-1, 1), repeat=3):
                for i, sign in enumerate(signs):
                    corner = z[i]+sign*ze[i]
                    self.assertLessEqual(abs(corner-x[i]), e[i]+1e-12)
            self.assertFalse(report['statistical_covariance'])

    def test_weak_tag_has_less_influence_but_not_a_smaller_error(self):
        cfg = sensor_options()
        strong = dict(min_edge_px=200, margin=200, reproj_px=.05, ambiguity_ratio=.05)
        weak = dict(min_edge_px=20, margin=20, reproj_px=2., ambiguity_ratio=.8)
        a, ae, _ = fuse_tag([0]*3, [.1]*3, [.1]*3, [.1]*3, strong, cfg)
        b, be, _ = fuse_tag([0]*3, [.1]*3, [.1]*3, [.1]*3, weak, cfg)
        self.assertGreater(tag_quality(strong, cfg['tag']), tag_quality(weak, cfg['tag']))
        self.assertGreater(a[0], b[0])
        self.assertGreater(be[0], ae[0])

    def test_axis_weights_use_prior_error_without_claiming_covariance(self):
        _, _, r = fuse_tag([0]*3, [.01, .1, 1.], [.1]*3, [.1]*3, {}, sensor_options())
        self.assertAlmostEqual(r['tag_weight'][0], .01/.11)
        self.assertEqual(r['tag_weight'][1:], [.5, .5])

    def test_repeated_correlated_tag_cannot_erase_error_floor(self):
        x, e = [0]*3, [.5]*3
        z, ze = [.1, .2, .03], [.02, .03, .01]
        for _ in range(100):
            x, e, _ = fuse_tag(x, e, z, ze, {}, sensor_options())
            self.assertTrue(all(a >= b for a, b in zip(e, ze)))

    def test_angle_wrap_takes_short_correction_and_preserves_bound(self):
        x, e, r = fuse_tag([0, 0, math.pi-.02], [.1]*3,
                          [0, 0, -math.pi+.02], [.1]*3, {}, sensor_options())
        self.assertAlmostEqual(x[2], math.pi)
        self.assertAlmostEqual(e[2], .12)
        self.assertAlmostEqual(r['innovation'][2], .04)

    def test_unknown_tag_error_stays_unknown_even_with_precise_prior(self):
        _, e, _ = fuse_tag([0]*3, [.001]*3, [.1]*3, None, {}, sensor_options())
        self.assertIsNone(e)

    def test_initialization_and_disabled_fusion_use_raw_tag(self):
        c = sensor_options()
        for prior, enabled in ((None, True), ([0]*3, False)):
            c['tag_fusion']['enabled'] = enabled
            x, e, _ = fuse_tag(prior, [.1]*3, [.2]*3, [.03]*3, {}, c)
            for v in x:
                self.assertAlmostEqual(v, .2)
            self.assertEqual(e, [.03]*3)

    def test_time_drift_grows_at_rest_and_terms_sum(self):
        b = sensor_options()['bounds']
        short, _ = drift_budget(b, 0., 0., 20, 1., 30)
        long, terms = drift_budget(b, 0., 0., 20, 11., 30)
        self.assertAlmostEqual(long[0]-short[0], .3)
        self.assertEqual(terms['distance'], [0]*3)
        self.assertAlmostEqual(long[0], sum(t[0] for t in terms.values()))

    def test_dropped_frames_do_not_reduce_the_stationary_drift_budget(self):
        b = sensor_options()['bounds']
        full, _ = drift_budget(b, 0., 0., 300, 10., 30)
        slow, _ = drift_budget(b, 0., 0., 100, 10., 30)
        for a, b in zip(full, slow):
            self.assertAlmostEqual(a, b)

    def test_real_profiles_do_not_add_a_new_physical_time_coefficient(self):
        for name in ('nano_measure', 'vm_measure', 'hardware', 'hardware_vm'):
            p = Path(__file__).resolve().parents[1]/'configs'/(name+'.json')
            cfg = json.loads(p.read_text())
            c = cfg.get('sensor', cfg)
            validate_estimation(c)
            expected = cfg.get('experiment', {}).get('variant') != 'a_baseline'
            self.assertEqual(c['tag_fusion']['enabled'], expected)
            self.assertNotIn('odom_error_per_s', c['bounds'])
            self.assertFalse(c['bounds']['verified'])

    def test_unknown_frame_coefficient_is_not_assumed_zero(self):
        b = sensor_options()['bounds']
        for missing in (False, True):
            if missing:
                b.pop('odom_error_per_frame')
            else:
                b['odom_error_per_frame'] = None
            total, terms = drift_budget(b, 0., 0., 0, 0., 30)
            self.assertIsNone(total)
            self.assertIsNone(terms['unprocessed_intervals'])

    def test_no_new_measured_time_coefficient_but_fps_and_weights_are_validated(self):
        c = sensor_options()
        validate_estimation(c, live=True)
        self.assertNotIn('odom_error_per_s', c['bounds'])
        for value in (-1., 0., float('nan'), True):
            c['camera']['fps'] = value
            with self.assertRaises(ValueError):
                validate_estimation(c)
        c = sensor_options()
        for value in ([0]*3, [2]*3, [float('inf')]*3, [True]*3):
            c['tag_fusion']['max_tag_weight'] = value
            with self.assertRaises(ValueError):
                validate_estimation(c)


@unittest.skipUnless(importlib.util.find_spec('numpy') and importlib.util.find_spec('cv2'),
                     'NumPy/OpenCV not installed in this interpreter')
class ObserverFusionTests(unittest.TestCase):
    def setUp(self):
        import numpy as np
        from docking.vision import Observer, inverse
        from test_core import config
        self.np, self.inverse = np, inverse
        self.c = config()['sensor']
        self.c['tag']['enabled'] = False  # Stub avoids a native detector dependency.
        self.c['tag']['period_s'] = .05
        self.c['tag_fusion'] = sensor_options()['tag_fusion']
        self.c['bounds'].update(verified=True, tag_pose_error=[.1]*3,
            odom_error_per_m=[0]*3, odom_error_per_frame=[.001]*3,
            odom_error_per_rad=[0]*3,
            gyro_rate_error_rps=.01)
        # Nonzero lever arm and rotation verify that the correction is in body/dock coordinates.
        self.bc = self.matrix([.4, -.2, .3])
        self.bc[2, 3] = .57
        self.c['calibration'].update(verified=True, T_body_camera=self.bc.tolist(),
                                     T_dock_tag=np.eye(4).tolist())
        self.o = Observer(self.c)
        self.tag_pose, self.tag_ok, self.body = [0]*3, True, np.eye(4)
        self.last_body, self.last_time = np.eye(4), None
        outer = self
        class Odom:
            ok, generation = True, 0
            def update(self, *args):
                t = args[-1]
                delta = inverse(outer.last_body).dot(outer.body)
                prior_t, outer.last_time = outer.last_time, t
                outer.last_body = outer.body.copy()
                return dict(ok=self.ok, reason='synthetic', generation=self.generation,
                    reference_stamp=prior_t, dt=t-prior_t if prior_t is not None else .1,
                    T_local_camera=inverse(outer.bc).dot(outer.body).dot(outer.bc),
                    T_previous_current=inverse(outer.bc).dot(delta).dot(outer.bc),
                    translation_m=float(np.linalg.norm(delta[:3, 3])),
                    rotation_rad=abs(math.atan2(delta[1, 0], delta[0, 0])))
        class Tag:
            def update(self, *args):
                return dict(ok=outer.tag_ok, reason='synthetic',
                            T_camera_tag=inverse(outer.matrix(outer.tag_pose).dot(outer.bc)))
        self.o.odom, self.o.tag = Odom(), Tag()

    def matrix(self, p):
        t = self.np.eye(4)
        c, s = math.cos(p[2]), math.sin(p[2])
        t[:2, :2] = [[c, -s], [s, c]]
        t[:2, 3] = p[:2]
        return t

    def frame(self, stamp, body=None, tag=None, dyaw=0.):
        if body is not None:
            self.body = self.matrix(body)
        if tag is not None:
            self.tag_pose = tag
        return self.o.process(self.np.zeros((10, 10), self.np.uint8), self.np.ones((10, 10)),
                              self.np.eye(3), self.np.zeros(5), stamp, dyaw)

    def test_tag_correction_uses_current_moving_prediction_not_last_pose(self):
        self.frame(1.)
        r = self.frame(1.1, [.1, 0, 0], [.12, 0, 0])
        self.assertAlmostEqual(r['tag_fusion']['prior_pose'][0], .1)
        self.assertAlmostEqual(r['pose'][0], .11)
        self.assertAlmostEqual(r['tag_fusion']['recenter_padding'][0], .01)
        self.assertGreaterEqual(r['error'][0], .11)

    def test_rotation_correction_handles_camera_lever_arm_and_preserves_local_motion(self):
        self.frame(1.)
        r = self.frame(1.1, [.1, .2, .1], [.12, .22, .14], dyaw=.1)
        self.np.testing.assert_allclose(r['pose'], [.11, .21, .12], atol=1e-12)
        self.np.testing.assert_allclose(r['local_motion']['xy'], [.1, .2], atol=1e-12)
        r = self.frame(1.2, [.1, .2, .1], [.12, .22, .14])
        self.np.testing.assert_allclose(r['local_motion']['xy'], [.1, .2], atol=1e-12)

    def test_drift_at_rest_between_visible_tag_updates_then_nonzero_reanchor(self):
        self.c['tag']['period_s'] = 1.
        first = self.frame(1.)
        second = self.frame(1.5)
        self.assertEqual(second['tag']['reason'], 'not_scheduled')
        self.assertTrue(all(b > a for a, b in zip(first['error'], second['error'])))
        corrected = self.frame(2.)
        self.assertGreaterEqual(corrected['error'][2], .1)
        self.assertGreater(corrected['local_motion']['error_xy'][0], second['local_motion']['error_xy'][0])

    def test_coast_and_rebound_still_change_pose_and_accumulate_absolute_travel(self):
        self.frame(1.)
        self.tag_ok = False
        forward = self.frame(1.1, [.04, 0, .02], dyaw=.02)
        rebound = self.frame(1.2, [.03, 0, .01], dyaw=-.01)
        self.assertAlmostEqual(forward['pose'][0], .04)
        self.assertAlmostEqual(rebound['pose'][0], .03)
        self.assertAlmostEqual(rebound['pose'][2], .01)
        self.assertAlmostEqual(self.o.travel, .05)
        self.assertAlmostEqual(self.o.angle, .03)
        self.assertGreater(rebound['error'][2], forward['error'][2])

    def test_conflicting_tag_is_rejected_instead_of_smoothed_into_validity(self):
        self.frame(1.)
        r = self.frame(1.1, tag=[2., 0, 0])
        self.assertFalse(r['tag']['ok'])
        self.assertEqual(r['tag_fusion']['reason'], 'tag_conflicts_with_prior_bound')
        self.assertAlmostEqual(r['pose'][0], 0.)
        self.assertEqual(r['anchor_stamp'], 1.)

    def test_tracking_reset_does_not_smooth_against_a_stale_anchor(self):
        self.frame(1.)
        self.o.odom.ok, self.o.odom.generation = False, 1
        self.tag_ok = False
        self.assertFalse(self.frame(1.1)['valid'])
        self.o.odom.ok, self.tag_ok = True, True
        r = self.frame(1.2, tag=[2., 0, 0])
        self.assertEqual(r['tag_fusion']['reason'], 'initialized_from_tag')
        self.assertAlmostEqual(r['pose'][0], 2.)

    def test_missing_geometry_keeps_pose_unavailable(self):
        self.c['calibration']['T_dock_tag'] = None
        r = self.frame(1.)
        self.assertIsNone(r['pose'])
        self.assertEqual(r['tag_fusion']['reason'], 'awaiting_calibrated_tag_pose')

    def test_missing_frame_bound_cannot_publish_bounded_smoothed_pose(self):
        self.c['bounds']['odom_error_per_frame'] = None
        r = self.frame(1.)
        self.assertFalse(r['bounded'])
        self.assertIsNone(r['error'])


if __name__ == '__main__':
    unittest.main()
