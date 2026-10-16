"""Camera/supervisor message regressions; no camera or CAN is opened."""
import argparse
import builtins
import json
import os
import queue
import tempfile
import threading
import unittest
from unittest.mock import patch

from docking.camera import worker
from docking.runtime import run
from test_core import ROOT


class SharedValues(list):
    def get_lock(self):
        return threading.Lock()


class FakeContext:
    """Deliver a child message either during capture or just after child exit."""
    Event = staticmethod(threading.Event)
    Queue = staticmethod(queue.Queue)

    def __init__(self, message, after_exit=False):
        self.message, self.after_exit = message, after_exit

    def Array(self, kind, values):
        return SharedValues(values)

    def Process(self, target, args, kwargs, daemon):
        context = self

        class Process:
            def start(self):
                self.stop, self.results = args[1:3]
                if not context.after_exit:
                    self.results.put(context.message)

            def is_alive(self):
                if self.stop.is_set():
                    return False
                if context.after_exit:
                    self.results.put(context.message)
                    return False
                return True

            def join(self, timeout=None):
                pass

        return Process()


class RuntimeMessageTests(unittest.TestCase):
    def measure(self, folder, message, after_exit=False):
        args = argparse.Namespace(
            command='measure', platform='nano',
            config=os.path.join(ROOT, 'configs', 'nano_measure.json'),
            no_tag=True, no_odom=True, no_depth=False, view=False,
            output=os.path.join(folder, 'run'), seconds=.01)
        context = FakeContext(message, after_exit)
        with patch('docking.runtime.mp.get_context', return_value=context), patch('builtins.print'):
            return run(args)

    def test_unknown_and_known_pose_error_do_not_abort_measurement(self):
        for error in (None, [0., 0., 0.], [.01, .02, .03]):
            with self.subTest(error=error), tempfile.TemporaryDirectory() as folder:
                output = self.measure(folder, {'error': error, 'pose': None})
                with open(os.path.join(output, 'session.json')) as handle:
                    session = json.load(handle)
                self.assertEqual(session['last_status']['state'], 'MEASURE')
                self.assertFalse(session['hardware_motion_enabled'])

    def test_worker_failures_keep_the_cause_in_both_receive_paths(self):
        for after_exit in (False, True):
            with self.subTest(after_exit=after_exit), tempfile.TemporaryDirectory() as folder:
                with self.assertRaisesRegex(RuntimeError, 'capture disconnected'):
                    self.measure(folder, {'worker_error': 'capture disconnected'}, after_exit)

    def test_normal_last_observation_does_not_mask_an_unexpected_child_exit(self):
        for error in (None, [.01, .02, .03]):
            with self.subTest(error=error), tempfile.TemporaryDirectory() as folder:
                with self.assertRaisesRegex(RuntimeError, 'camera process exited'):
                    self.measure(folder, {'error': error}, after_exit=True)

    def test_camera_exception_uses_separate_worker_error_message(self):
        original_import = builtins.__import__

        def fail_numpy(name, *args, **kwargs):
            if name == 'numpy':
                raise ImportError('test dependency unavailable')
            return original_import(name, *args, **kwargs)

        stop, results = threading.Event(), queue.Queue(1)
        with tempfile.TemporaryDirectory() as folder:
            with patch('docking.camera.signal.signal'), patch('builtins.__import__', side_effect=fail_numpy):
                worker({}, stop, results, queue.Queue(1), folder)
        self.assertTrue(stop.is_set())
        self.assertEqual(results.get_nowait(), {
            'worker_error': 'ImportError: test dependency unavailable'})


if __name__ == '__main__':
    unittest.main()
