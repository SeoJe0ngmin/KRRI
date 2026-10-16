import argparse
import json
import os
import tempfile
import unittest
from unittest.mock import patch
from docking.runtime import run
from test_core import ROOT


class MeasureModeTests(unittest.TestCase):
    def test_default_records_outside_code_even_from_different_working_directory(self):
        with tempfile.TemporaryDirectory() as folder:
            work = os.path.join(folder, 'krri', 'work')
            other = os.path.join(folder, 'elsewhere')
            os.mkdir(other)
            args = argparse.Namespace(
                command='measure', config=os.path.join(ROOT, 'configs', 'nano_measure.json'),
                no_tag=False, no_odom=True, no_depth=False, output=None)
            previous = os.getcwd()
            try:
                os.chdir(other)
                with patch('docking.runtime.WORK_ROOT', work), \
                     patch('docking.runtime.mp.get_context', side_effect=RuntimeError('before_any_devices')):
                    with self.assertRaisesRegex(RuntimeError, 'before_any_devices'):
                        run(args)
                sessions = os.listdir(work)
                self.assertEqual(len(sessions), 1)
                self.assertTrue(sessions[0].endswith('_measure'))
                self.assertTrue(os.path.isfile(os.path.join(work, sessions[0], 'config_used.json')))
                self.assertEqual(os.listdir(other), [])
            finally:
                os.chdir(previous)

    def prepare_without_devices(self, folder, no_depth=False, no_odom=False):
        args = argparse.Namespace(command='measure', config=os.path.join(ROOT, 'configs', 'nano_measure.json'),
                                  no_tag=False, no_odom=no_odom, no_depth=no_depth,
                                  output=os.path.join(folder, 'run'))
        # Stop before multiprocessing/device creation; inspect the exact
        # sensor profile that would be passed to the worker.
        with patch('docking.runtime.mp.get_context', side_effect=RuntimeError('before_any_devices')):
            with self.assertRaisesRegex(RuntimeError, 'before_any_devices'):
                run(args)
        with open(os.path.join(args.output, 'config_used.json')) as f:
            return json.load(f)

    def test_color_without_depth_cannot_reuse_ir_geometry(self):
        with tempfile.TemporaryDirectory() as folder:
            c = self.prepare_without_devices(folder, no_depth=True)
        self.assertEqual(c['camera']['stream'], 'color')
        self.assertFalse(c['camera']['depth_enabled'])
        self.assertFalse(c['odom']['enabled'])
        self.assertTrue(c['tag']['enabled'])
        self.assertFalse(c['calibration']['verified'])
        self.assertIsNone(c['calibration']['T_body_camera'])

    def test_no_odometry_preserves_depth_for_separate_workload_comparison(self):
        with tempfile.TemporaryDirectory() as folder:
            c = self.prepare_without_devices(folder, no_odom=True)
        self.assertEqual(c['camera']['stream'], 'infrared')
        self.assertTrue(c['camera']['depth_enabled'])
        self.assertFalse(c['odom']['enabled'])
