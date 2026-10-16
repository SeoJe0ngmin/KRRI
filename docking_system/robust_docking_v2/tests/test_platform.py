import json
import os
import unittest
from unittest.mock import patch

from docking.__main__ import main, default_config


class PlatformTests(unittest.TestCase):
    def test_vm_measure_uses_vm_profile(self):
        with patch('sys.argv', ['docking', 'measure', '--platform', 'vm']), \
             patch('docking.runtime.run') as run:
            self.assertEqual(main(), 0)
            self.assertEqual(run.call_args[0][0].config, default_config('measure', 'vm'))

    def test_explicit_config_is_preserved(self):
        with patch('sys.argv', ['docking', 'measure', '--platform', 'vm', '--config', 'my_site.json']), \
             patch('docking.runtime.run') as run:
            self.assertEqual(main(), 0)
            self.assertEqual(run.call_args[0][0].config, 'my_site.json')

    def test_platform_hardware_defaults_are_unverified_and_separate(self):
        self.assertNotEqual(default_config('drive', 'vm'), default_config('drive', 'nano'))
        for platform in ('vm', 'nano'):
            with open(default_config('drive', platform)) as f:
                c = json.load(f)
            self.assertFalse(c['validation']['approved'])
            self.assertEqual(c['body']['front_m'], 1.5)
            self.assertEqual(c['body']['rear_m'], 1.)
            self.assertEqual(c['body']['half_width_m'], .75)
            if platform == 'vm':
                self.assertFalse(c['deployment']['timing_verified_on_this_host'])


if __name__ == '__main__':
    unittest.main()
