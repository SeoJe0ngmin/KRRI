import copy
import os
import unittest
from docking.configuration import load_config
from docking.profiles import build_trial, prepare, readiness
from test_core import ROOT


class ProfileTests(unittest.TestCase):
    def setUp(self):
        self.cfg = load_config(os.path.join(ROOT, 'configs', 'hardware.json'))

    def test_wall_front_not_camera_stops_half_metre_from_plane(self):
        self.cfg['trial'].update(lateral_tolerance_m=.02, yaw_tolerance_rad=.01)
        c = build_trial(self.cfg, 'wall_stop')
        self.assertEqual(c['goal']['lo'][0], -2.1)
        self.assertEqual(c['goal']['hi'][0], -1.9)
        self.assertFalse(c['prealignment']['enabled'])
        self.assertIsNone(self.cfg['goal'])

    def test_cone_goal_requires_whole_vehicle_inside(self):
        self.cfg['body']['rear_m'] = 1.
        self.cfg['body']['half_width_m'] = .5
        self.cfg['trial'].update(lateral_tolerance_m=.02, yaw_tolerance_rad=.01, insertion_front_m=2.)
        with self.assertRaises(ValueError):
            build_trial(self.cfg, 'cone_entry')
        self.cfg['trial']['insertion_front_m'] = 3.
        c = build_trial(self.cfg, 'cone_entry')
        self.assertAlmostEqual(c['goal']['lo'][0]-c['body']['rear_m'], .4)
        self.assertTrue(c['prealignment']['enabled'])

    def test_cone_depth_checks_rear_corner_at_nonzero_yaw(self):
        self.cfg['body'].update(rear_m=1., half_width_m=.5)
        self.cfg['trial'].update(lateral_tolerance_m=.02, yaw_tolerance_rad=.2, insertion_front_m=2.61)
        with self.assertRaisesRegex(ValueError, 'rotated'):
            build_trial(self.cfg, 'cone_entry')

    def test_even_incomplete_new_trial_clears_approval(self):
        self.cfg['validation']['approved'] = True
        c = build_trial(self.cfg, 'wall_stop')
        self.assertIsNone(c['goal'])
        self.assertFalse(c['validation']['approved'])
        self.assertFalse(c['prealignment']['enabled'])

    def test_preparation_preserves_platform_timing_and_known_heights(self):
        c = prepare(self.cfg, os.path.join(ROOT, 'configs', 'sites', 'krri.json'), 'cone_entry')
        self.assertEqual(c['limits'], self.cfg['limits'])
        self.assertEqual(c['sensor']['rig'], self.cfg['sensor']['rig'])
        self.assertEqual(c['body']['front_m'], 1.5)
        r = readiness(c)
        self.assertFalse(r['drive_ready'])
        self.assertFalse(r['hardware_opened'])
        self.assertIn('limits.lease_s', r['missing_inputs'])
        self.assertIn('startup_search.brake_reserve_rad', r['missing_inputs'])
        self.assertIn('startup_search.free_radius_m', r['missing_inputs'])

    def test_measurement_profile_does_not_claim_drive_ready(self):
        c = load_config(os.path.join(ROOT, 'configs', 'nano_measure.json'))
        self.assertFalse(readiness(c)['drive_ready'])

    def test_new_active_features_report_missing_inputs_and_verification(self):
        r = readiness(self.cfg)
        for field in ('sensor.stop_window.translation_error_m',
                      'sensor.tracking_recovery.max_unobserved_speed_mps',
                      'recovery.blind_return.stop_rest_after_s',
                      'can.diagnostics.max_ack_age_s', 'validation.motion_contracts'):
            self.assertIn(field, r['missing_inputs'])
        self.assertIn('sensor.imu.yaw_sign_verified', r['unverified_inputs'])
        self.assertIn('can.diagnostics.verified', r['unverified_inputs'])
        for field in ('sensor.camera.serial', 'sensor.camera.exposure_us', 'sensor.camera.gain'):
            self.assertNotIn(field, r['missing_inputs'])

    def test_disabled_optional_features_are_not_reported_as_required(self):
        self.cfg['sensor']['tracking_recovery']['enabled'] = False
        self.cfg['recovery']['blind_return']['enabled'] = False
        self.cfg['startup_search']['enabled'] = False
        r = readiness(self.cfg)
        self.assertNotIn('sensor.tracking_recovery.max_unobserved_speed_mps', r['missing_inputs'])
        self.assertNotIn('recovery.blind_return.verified', r['unverified_inputs'])
        self.assertNotIn('validation.startup_search', r['missing_inputs'])

    def test_vm_requires_host_specific_timing_verification(self):
        c = load_config(os.path.join(ROOT, 'configs', 'hardware_vm.json'))
        self.assertIn('deployment.timing_verified_on_this_host', readiness(c)['unverified_inputs'])
