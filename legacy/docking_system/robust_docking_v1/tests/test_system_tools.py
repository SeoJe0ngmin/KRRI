"""Exercise shared field tools without a camera, CAN connection or SSH server."""
import copy
import importlib.util
import io
import json
import os
from pathlib import Path
import subprocess
import sys
import tempfile
import unittest
from unittest.mock import patch

import test_shared_config as fixtures

SYSTEM = fixtures.SYSTEM


class FieldToolTests(unittest.TestCase):
    setUp = fixtures.SharedConfigTests.setUp
    config = fixtures.SharedConfigTests.config

    def tool(self):
        spec = importlib.util.spec_from_file_location('field_tool_test', SYSTEM/'preflight.py')
        tool = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(tool)
        tool.ROOT = self.root
        return tool

    def test_device_report_does_not_turn_health_into_driving_approval(self):
        original = copy.deepcopy(self.config())
        result = dict(status='PASS', checks={}, command_frames_sent=False)
        with patch('docking.device_check.check_devices', return_value=result) as check, patch('sys.stdout', new_callable=io.StringIO):
            code = self.tool().main(['devices', '--seconds', '2'])
        self.assertEqual(code, 0)
        self.assertEqual(check.call_args.args[1], 2.)
        self.assertEqual(self.config(), original)
        self.assertEqual(self.shared.read(self.path)['automatic']['devices_nano']['status'], 'PASS')

    def test_inspection_records_per_run_imu_but_leaves_bias_and_limits_unchanged(self):
        original = copy.deepcopy(self.config())
        def capture(args):
            self.assertEqual(args.command, 'measure')
            self.assertFalse(args.execute)
            path = Path(args.output)
            path.mkdir(parents=True)
            (path/'metrics.json').write_text(json.dumps({'imu': {'ready': True, 'effective_bias_rps': [.1, .2, .3]}, 'performance': {'camera_fps': 30}}))
            (path/'session.json').write_text(json.dumps({'interrupted': False, 'hardware_motion_enabled': False}))
            return str(path)
        with patch('docking.runtime.run', side_effect=capture), patch('docking.paths.WORK_ROOT', str(self.root/'results')), patch('sys.stdout', new_callable=io.StringIO):
            code = self.tool().main(['--version', 'v3', 'inspect', '--seconds', '30'])
        self.assertEqual(code, 0)
        self.assertEqual(self.config(), original)
        report = self.shared.read(self.path)['automatic']['inspection_nano_v3']
        self.assertFalse(report['applied_startup_bias'])
        self.assertFalse(report['certified_error_bound'])
        self.assertEqual(report['metrics']['imu']['effective_bias_rps'], [.1, .2, .3])

    def test_updater_preserves_field_config_by_default_and_only_replaces_explicitly(self):
        # External command stubs capture transfer arguments; no network is used.
        bindir = self.root/'fake_bin'
        bindir.mkdir()
        log = self.root/'tool_calls.jsonl'
        script = '''#!/usr/bin/env python3
import json, os, sys
from pathlib import Path
name = Path(sys.argv[0]).name
if name == 'uname':
    print('Darwin')
else:
    with open(os.environ['DOCKING_TOOL_LOG'], 'a') as f:
        f.write(json.dumps([name]+sys.argv[1:])+'\\n')
    if name == 'ssh': sys.stdin.read()
'''
        for name in ('uname', 'ssh', 'rsync'):
            path = bindir/name
            path.write_text(script)
            path.chmod(0o755)
        env = dict(os.environ, PATH=str(bindir)+os.pathsep+os.environ['PATH'], DOCKING_TOOL_LOG=str(log))
        for extra in ([], ['--with-config']):
            log.write_text('')
            result = subprocess.run(['bash', str(SYSTEM/'update_jetson.sh'), '--dry-run', *extra],
                                    env=env, capture_output=True, text=True, timeout=10)
            self.assertEqual(result.returncode, 0, result.stderr)
            transfers = [row for row in map(json.loads, log.read_text().splitlines()) if row[0] == 'rsync']
            self.assertTrue(transfers)
            for row in transfers:
                self.assertIn('--dry-run', row)
                self.assertIn('--exclude=/calibration/', row)
                self.assertNotIn('--delete', row)
            if extra:
                self.assertEqual(len(transfers), 1)
                self.assertNotIn('--exclude=/configs/common.json', transfers[0])
            else:
                self.assertIn('--exclude=/configs/common.json', transfers[0])
                self.assertIn('--ignore-existing', transfers[1])

    def test_updater_prunes_only_named_retired_files_without_following_symlinks(self):
        text = (SYSTEM/'update_jetson.sh').read_text()
        body = text.split("python3 -B - <<'PY'\n", 1)[1].split('\nPY', 1)[0]
        project = self.root/'robust_docking_v1'
        retired = project/'configs/demo.json'
        retired.write_text('{}')
        custom = project/'configs/operator_notes.json'
        custom.write_text('{"note": "preserve"}')
        external = self.root/'operator_file.json'
        external.write_text('{"keep": true}')
        linked = project/'configs/demo_sidestep.json'
        linked.symlink_to(external)
        snapshot = self.path.read_bytes()
        logs = self.root/'work/observations.jsonl'
        logs.parent.mkdir()
        logs.write_text('{"capture": 1}\n')
        result = subprocess.run([sys.executable, '-c', body], cwd=self.root, capture_output=True, text=True, timeout=10)
        self.assertEqual(result.returncode, 0, result.stderr)
        self.assertFalse(retired.exists())
        self.assertTrue(custom.exists())
        self.assertTrue(linked.is_symlink())
        self.assertEqual(external.read_text(), '{"keep": true}')
        self.assertEqual(self.path.read_bytes(), snapshot)
        self.assertEqual(logs.read_text(), '{"capture": 1}\n')
