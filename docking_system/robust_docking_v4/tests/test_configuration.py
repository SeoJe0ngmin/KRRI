"""Measured geometry must affect transforms/footprint without inventing data."""
import copy
import json
import os
import unittest

from docking.configuration import load_config, resolve_config, rig_dimensions
from docking.core import validate

CONFIGS = os.path.join(os.path.dirname(__file__), '..', 'configs')
R_OPTICAL_TO_BODY = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
R_TAG_TO_DOCK = [[0, 0, -1], [-1, 0, 0], [0, 1, 0]]


class GeometryTests(unittest.TestCase):
    def test_measured_dimensions_survive_all_real_profiles(self):
        for name in ('nano_measure.json', 'vm_measure.json', 'hardware.json', 'hardware_vm.json'):
            cfg = load_config(os.path.join(CONFIGS, name))
            sensor = cfg.get('sensor', cfg)
            rig = rig_dimensions(sensor)
            self.assertEqual(rig['tag_height_m'], 1.58)
            self.assertEqual(rig['camera_height_m'], .57)
            self.assertEqual(rig['camera_to_fork_tip_m'], 1.5)
            self.assertAlmostEqual(rig['height_difference_m'], 1.01)
            # Heights alone must not create an unmeasured rotation or offset.
            self.assertIsNone(sensor['calibration']['T_body_camera'])
            self.assertIsNone(sensor['calibration']['T_dock_tag'])
            if 'sensor' in cfg:
                self.assertEqual(cfg['body']['front_m'], 1.5)
                with self.assertRaises(ValueError):
                    validate(cfg, live=True)

    def complete(self):
        cfg = load_config(os.path.join(CONFIGS, 'hardware.json'))
        cfg['sensor']['calibration'].update(
            R_body_camera=R_OPTICAL_TO_BODY, camera_lateral_m=.04,
            R_dock_tag=R_TAG_TO_DOCK, tag_position_xy_m=[0, .8])
        return cfg

    def test_heights_and_side_tag_offset_build_full_transforms(self):
        before = self.complete()
        cfg = resolve_config(before)
        cal = cfg['sensor']['calibration']
        self.assertEqual([row[3] for row in cal['T_body_camera'][:3]], [0, .04, .57])
        self.assertEqual([row[3] for row in cal['T_dock_tag'][:3]], [0, .8, 1.58])
        self.assertIsNone(before['sensor']['calibration']['T_body_camera'])
        self.assertEqual(resolve_config(cfg), cfg)
        # Aligned front at -0.5 m means the body/camera station is at -2 m.
        body_x = -.5-cfg['body']['front_m']
        self.assertEqual(body_x, -2.)
        # Camera sees tag centre 2 m forward, 0.76 m left, 1.01 m above.
        dock_delta = [0-body_x, .8-.04, 1.58-.57]
        camera_delta = [sum(R_OPTICAL_TO_BODY[k][i]*dock_delta[k] for k in range(3)) for i in range(3)]
        for actual, expected in zip(camera_delta, [-.76, -1.01, 2.]):
            self.assertAlmostEqual(actual, expected)

    def test_known_measurement_changes_flow_into_calculation(self):
        c = self.complete()
        c['sensor']['rig']['camera_height_m'] = .60
        c['sensor']['rig']['tag_height_m'] = 1.70
        c['sensor']['rig']['camera_to_fork_tip_m'] = 1.6
        c['body']['front_m'] = None
        c = resolve_config(c)
        self.assertEqual(c['sensor']['calibration']['T_body_camera'][2][3], .60)
        self.assertEqual(c['sensor']['calibration']['T_dock_tag'][2][3], 1.70)
        self.assertEqual(c['body']['front_m'], 1.6)

    def test_existing_matrices_cannot_override_new_measurements(self):
        c = resolve_config(self.complete())
        for field in ('camera_height_m', 'tag_height_m', 'camera_to_fork_tip_m'):
            changed = copy.deepcopy(c)
            changed['sensor']['rig'][field] += .1
            with self.assertRaises(ValueError):
                resolve_config(changed)

    def test_bad_rotations_are_rejected_even_before_full_calibration(self):
        for rotation in (5, [[1, 0, 0], [0, 1, 0], [0, 0, -1]], [[1, 0, 0]]):
            cfg = self.complete()
            cfg['sensor']['calibration']['R_body_camera'] = rotation
            with self.assertRaises(ValueError):
                resolve_config(cfg)

    def test_explicit_test_geometry_is_not_derived_from_rig(self):
        with open(os.path.join(os.path.dirname(__file__), 'fixtures', 'policy_contract.json')) as f:
            c = json.load(f)
        self.assertEqual(resolve_config(c), c)


if __name__ == '__main__':
    unittest.main()
