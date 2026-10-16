"""Shared physical inputs must reach every estimator without stale overrides."""
import copy
import json
from pathlib import Path
import shutil
import tempfile
import unittest

from docking.configuration import load_config
from docking.core import fingerprint
from docking import mount_calibration as mc
from docking.shared import manager
from test_mount_calibration import MODEL, observations

SYSTEM = Path(__file__).resolve().parents[2]


def descriptions(text):
    doc = json.loads(text)
    result = {(): doc['description']}
    def walk(node, path):
        result[path] = node['description']
        for key, child in node.get('fields', {}).items():
            walk(child, path+(key,))
    for key, node in doc.items():
        if key not in ('schema', 'description'):
            walk(node, (key,))
    return result


class SharedConfigTests(unittest.TestCase):
    def setUp(self):
        self.temp = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp.cleanup)
        self.root = Path(self.temp.name)/'docking_system'
        (self.root/'configs').mkdir(parents=True)
        shutil.copyfile(SYSTEM/'configs/common.json', self.root/'configs/common.json')
        for n in range(1, 5):
            dst = self.root/f'robust_docking_v{n}'
            dst.mkdir()
            shutil.copyfile(SYSTEM/f'robust_docking_v{n}/version.json', dst/'version.json')
            shutil.copytree(SYSTEM/f'robust_docking_v{n}/configs', dst/'configs')
        self.path = self.root/'configs/common.json'
        self.shared = manager()

    def edit(self, fn):
        before = self.path.read_bytes()
        doc = self.shared.read(self.path)
        fn(doc)
        self.path.write_text(self.shared.encode(doc, previous=before))

    def config(self, n=1, name='hardware.json'):
        return load_config(self.root/f'robust_docking_v{n}/configs'/name)

    def test_one_height_change_reaches_all_versions_and_platforms_without_sync(self):
        self.edit(lambda d: d['values']['sensor']['rig'].update(camera_height_m=.63, camera_to_fork_tip_m=1.62))
        for n in range(1, 5):
            for name in ('hardware.json', 'hardware_vm.json', 'nano_measure.json', 'vm_measure.json', 'kw_wall_stop.json'):
                cfg = self.config(n, name)
                sensor = cfg.get('sensor', cfg)
                self.assertEqual(sensor['rig']['camera_height_m'], .63)
                if 'sensor' in cfg:
                    self.assertEqual(cfg['body']['front_m'], 1.62)

    def test_physical_settings_common_but_algorithm_flags_remain_distinct(self):
        for n, flags in enumerate(((True, True), (False, False), (True, False), (True, True)), 1):
            s = self.config(n)['sensor']
            self.assertEqual((s['tag_fusion']['enabled'], s['odom']['gyro_aided_tracking']), flags)
            for key in ('rig', 'imu', 'camera', 'calibration'):
                self.assertEqual(s[key], self.config(1)['sensor'][key])

    def test_stationary_startup_duration_is_shared_and_requires_new_policy(self):
        before = self.config()
        self.assertTrue(before['startup_measurement']['enabled'])
        self.assertEqual(before['startup_measurement']['duration_s'], 60.)
        self.edit(lambda d: d['values']['startup_measurement'].update(duration_s=30.))
        for n in range(1, 5):
            for name in ('hardware.json', 'hardware_vm.json', 'kw_wall_stop.json', 'kw_park_left_tag.json'):
                self.assertEqual(self.config(n, name)['startup_measurement']['duration_s'], 30.)
        self.assertNotEqual(fingerprint(before), fingerprint(self.config()))

    def test_missing_common_fails_instead_of_using_cached_profile(self):
        self.path.unlink()
        with self.assertRaises(FileNotFoundError):
            self.config()

    def test_mount_success_published_once_used_by_all_and_does_not_approve_drive(self):
        sensor = self.config()['sensor']
        for n in range(1, 5):
            for all_versions in (False, True):
                self.assertEqual(mc.profile_paths(self.root/f'robust_docking_v{n}', all_versions), [self.path])
        report = mc.estimate(observations(), sensor, MODEL)
        before = self.path.read_bytes()
        self.assertEqual(mc.apply_report(report, [self.path], lateral=-.02, expected=before), [str(self.path)])
        self.assertLessEqual(descriptions(before).items(), descriptions(self.path.read_bytes()).items())
        for n in range(1, 5):
            cfg = self.config(n)
            cal = cfg['sensor']['calibration']
            self.assertEqual(cal['R_body_camera'], report['R_body_camera'])
            self.assertEqual([row[3] for row in cal['T_body_camera'][:3]], [0., -.02, .57])
            self.assertEqual(cfg['sensor']['camera']['serial'], MODEL['serial'])
            self.assertFalse(cal['verified'])
            self.assertFalse(cfg['validation']['approved'])
            self.assertFalse(cfg['sensor']['imu']['yaw_sign_verified'])
            self.assertEqual(cfg['sensor']['imu']['gyro_bias_rps'], sensor['imu']['gyro_bias_rps'])
        self.edit(lambda d: d['values']['sensor']['rig'].update(camera_height_m=.64))
        self.assertEqual(self.config()['sensor']['calibration']['T_body_camera'][2][3], .64)

    def test_failed_or_stale_calibration_does_not_overwrite_common(self):
        report = mc.estimate(observations(), self.config()['sensor'], MODEL)
        before = self.path.read_bytes()
        with self.assertRaises(ValueError):
            mc.apply_report(dict(report, usable=False), [self.path])
        self.assertEqual(self.path.read_bytes(), before)
        self.edit(lambda d: d['values']['sensor']['rig'].update(tag_height_m=1.6))
        edited = self.path.read_bytes()
        with self.assertRaisesRegex(ValueError, 'changed during'):
            mc.apply_report(report, [self.path], expected=before)
        self.assertEqual(self.path.read_bytes(), edited)

    def test_diagnostic_success_is_record_only_and_does_not_invalidate_policy(self):
        before_cfg = self.config()
        before = self.path.read_bytes()
        self.shared.record_check(self.path, 'inspection_nano_v1', {'imu': {'ready': True}, 'fps': 30}, before)
        self.assertLessEqual(descriptions(before).items(), descriptions(self.path.read_bytes()).items())
        self.assertEqual(fingerprint(before_cfg), fingerprint(self.config()))
        self.assertFalse(self.config()['validation']['approved'])
        self.assertNotIn('automatic', self.config())

    def test_invalid_geometry_prevents_publish_and_sync(self):
        original = self.path.read_bytes()
        doc = self.shared.decode(original)
        doc['values']['sensor']['rig']['camera_height_m'] = -1
        with self.assertRaises(ValueError):
            self.shared.commit(self.path, doc, original)
        self.assertEqual(self.path.read_bytes(), original)

    def test_target_change_propagates_to_all_versions_without_changing_other_task(self):
        def change(d):
            d['sites']['kw_wall_stop']['trial'].update(standoff_m=.7, yaw_tolerance_rad=.01)
        self.edit(change)
        for n in range(1, 5):
            cfg = self.config(n, 'kw_wall_stop.json')
            self.assertAlmostEqual(sum([cfg['goal']['lo'][0], cfg['goal']['hi'][0]])/2, -2.2)
            self.assertEqual(self.config(n, 'kw_park_left_tag.json')['sensor']['calibration']['tag_position_xy_m'], [None, 1.])

    def test_platform_measurement_relaxation_does_not_change_driving_requirements(self):
        self.assertEqual(self.config(name='nano_measure.json')['imu']['max_age_s'], .15)
        self.assertIsNone(self.config()['sensor']['imu']['max_age_s'])
        self.assertFalse(self.config(name='hardware_vm.json')['deployment']['timing_verified_on_this_host'])

    def test_generated_snapshots_refresh_but_changes_to_them_are_not_another_source(self):
        self.edit(lambda d: d['values']['body'].update(half_width_m=.7))
        self.shared.sync(self.root)
        p = self.root/'robust_docking_v3/configs/hardware.json'
        snapshot = json.loads(p.read_text())
        self.assertEqual(snapshot['body']['half_width_m'], .7)
        snapshot['body']['half_width_m'] = 9
        p.write_text(json.dumps(snapshot))
        self.assertEqual(self.config(3)['body']['half_width_m'], .7)

    def test_description_changes_do_not_change_any_resolved_config(self):
        before = [self.config(n) for n in range(1, 5)]
        doc = json.loads(self.path.read_text())
        doc['description'] = '현장 메모: #은 문자열의 일부'
        doc['values']['fields']['body']['fields']['rear_m']['description'] = '뒤끝 거리 설명 수정'
        self.path.write_text(json.dumps(doc))
        self.assertEqual(before, [self.config(n) for n in range(1, 5)])
        self.shared.record_check(self.path, 'description_test', {}, self.path.read_bytes())
        saved = json.loads(self.path.read_text())
        self.assertEqual(saved['description'], doc['description'])
        self.assertEqual(saved['values']['fields']['body']['fields']['rear_m']['description'], '뒤끝 거리 설명 수정')

    def test_values_and_descriptions_survive_roundtrip_and_type_change(self):
        doc = {'schema': self.shared.SCHEMA, 'message': '태그 #1 / "인용" / \\ 경로',
               'left': {'enabled': True}, 'right': {'enabled': False}, 'camera_model': None}
        notes = {('message',): '문자열 내부 #은 값의 일부',
                 ('left', 'enabled'): '첫 번째 사용 여부', ('right', 'enabled'): '두 번째 사용 여부'}
        text = self.shared.encode(doc, descriptions=notes)
        json.loads(text)  # The user file is also valid for a standard JSON reader.
        self.assertEqual(self.shared.decode(text.encode()), doc)
        doc['left']['enabled'] = False
        doc['camera_model'] = {'width': 640, 'description': 'SDK model', 'value': [1, 2]}
        updated = self.shared.encode(doc, previous=text)
        self.assertEqual(self.shared.decode(updated), doc)
        raw = json.loads(updated)
        self.assertEqual(raw['camera_model']['value'], doc['camera_model'])
        self.assertEqual(raw['left']['fields']['enabled']['description'], '첫 번째 사용 여부')
        self.assertEqual(raw['right']['fields']['enabled']['description'], '두 번째 사용 여부')

    def test_non_json_comments_remain_invalid(self):
        for bad in ('{"fps": 30, # 주석\n}', '{"fps": 30 # 주석\n"width": 640}',
                    '{"fps": 30} // 다른 주석 형식'):
            with self.subTest(bad=bad), self.assertRaises(json.JSONDecodeError):
                self.shared.decode(bad)

    def test_ambiguous_or_missing_value_is_rejected(self):
        for node in ({'description': '설명'}, {'description': '설명', 'value': 1, 'fields': {}},
                     {'value': 1}, {'description': '설명', 'fields': []}):
            doc = {'schema': self.shared.DOCUMENT_SCHEMA, 'description': '공통', 'values': node}
            with self.subTest(node=node), self.assertRaises(ValueError):
                self.shared.decode(json.dumps(doc))

    def test_legacy_plain_common_remains_readable(self):
        before = self.config()
        self.path.write_text(json.dumps(self.shared.read(self.path)))
        self.assertEqual(self.config(), before)


if __name__ == '__main__':
    unittest.main()
