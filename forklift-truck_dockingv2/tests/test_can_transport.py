"""Exercise the real transport loop against an in-memory Kvaser API double."""
import threading
import types
import unittest
import json
import os
from unittest.mock import patch

from docking.canbus import _worker, templates, Can
from support import config, ROOT


class Shared(list):
    def __init__(self, values):
        super().__init__(values)
        self.lock = threading.Lock()

    def get_lock(self):
        return self.lock


class TransportTests(unittest.TestCase):
    def test_lift_fold_reach_and_combined_motion_commands_remain_forbidden(self):
        for name in ('lift_up', 'lift_down', 'fold', 'unfold', 'reach_forward', 'reach_backward',
                     'forward_left', 'backward_right'):
            can = object.__new__(Can)
            can.abort_event = threading.Event()
            with self.assertRaises(ValueError): can.set(name)
            self.assertTrue(can.abort_event.is_set())

    def test_startup_active_expired_lease_and_shutdown_frames(self):
        clock = [10.]
        sent = []
        shared, health = Shared([0., 0.]), Shared([0., 0.])
        abort = threading.Event()
        class Close:
            def is_set(self):
                return clock[0] >= 10.25
            def wait(self, dt):
                clock[0] += dt
                return self.is_set()
        class Ready:
            def put(self, value):
                if value.get('ok'):
                    shared[0], shared[1] = 2, clock[0]+.04
            put_nowait = put
        class Frame:
            def __init__(self, id_, data, flags):
                self.id_, self.data, self.flags = id_, list(data), flags
                self.id = id_   # real canlib.Frame exposes .id; guard() reads it
        class Channel:
            def setBusParams(self, rate):
                self.rate = rate
            def busOn(self):
                pass
            def write(self, frame):
                sent.append((clock[0], frame))
                # Simulate a producer suspended while holding its mailbox.
                # The transmitter must keep checking the LAST lease deadline.
                if frame.id_==0x1E3 and frame.data[2]==97 and not shared.lock.locked():
                    shared.lock.acquire()
            def writeSync(self, timeout):
                pass
            def busOff(self):
                pass
            def close(self):
                pass
        backend = types.SimpleNamespace(openChannel=lambda _: Channel(),
            Bitrate=types.SimpleNamespace(BITRATE_125K=125000, BITRATE_250K=250000,
                                         BITRATE_500K=500000, BITRATE_1M=1000000))
        module = types.SimpleNamespace(canlib=backend, Frame=Frame)
        def sleep(dt):
            clock[0] += dt
        with patch.dict('sys.modules', {'canlib': module}), \
             patch('docking.canbus.time.monotonic', side_effect=lambda: clock[0]), \
             patch('docking.canbus.time.sleep', side_effect=sleep), \
             patch('docking.canbus.signal.signal'):
            _worker(config()['can'], shared, abort, Close(), Ready(), health)
        self.assertEqual([f.id_ for _,f in sent[:15]],[0x2E3,0x764,0x1E3]*5)
        first_forward=next(i for i,(_,f) in enumerate(sent) if f.id_==0x1E3 and f.data[2]==97)
        self.assertEqual([f.id_ for _,f in sent[first_forward-2:first_forward+1]],[0x2E3,0x764,0x1E3])
        self.assertEqual([f.id_ for _,f in sent[first_forward-2:first_forward+13]],[0x2E3,0x764,0x1E3]*5)
        movement = [(t, f) for t, f in sent if f.id_ == 0x1E3]
        self.assertTrue(all(f.data == [127]*8 for _, f in movement[:5]))
        self.assertTrue(any(f.data[2] == 97 for _, f in movement))
        self.assertTrue(all(f.data == [127]*8 for t, f in movement if t >= 10.066))
        self.assertTrue(all(f.data[4] == 127 for _, f in movement))
        self.assertEqual(movement[-1][1].data, [127]*8)
        self.assertTrue(any(f.id_ == 0x2E3 for _, f in sent))
        self.assertTrue(any(f.id_ == 0x764 for _, f in sent))
        for _, f in sent:
            self.assertEqual(f.flags, 0)
            self.assertIn(f.id_, (0x1E3, 0x2E3, 0x764))
            if f.id_ == 0x2E3:
                self.assertIn(f.data[4], range(16))
                self.assertEqual(f.data[:4]+f.data[5:], [0x42, 0, 0, 0x0A, 0x40, 0x69, 0x93])
            if f.id_ == 0x764:
                self.assertEqual(f.data, [0])
        self.assertTrue(abort.is_set())


if __name__ == '__main__':
    unittest.main()


class WorkerFaultTests(unittest.TestCase):
    def test_write_failure_emits_neutral_only_and_never_reads_the_bus(self):
        """Transmit-only (2026-10-18): a write failure latches abort, sends neutral immediately and at
        shutdown, and reports bus status; the receive queue is never read."""
        import queue
        clock, sent = [10.], []
        shared, health = Shared([0., 0., 0.]), Shared([0., 0.])
        abort = threading.Event()
        reports, faults = queue.Queue(4), queue.Queue(1)
        class Frame:
            def __init__(self, id_, data, flags=0):
                self.id, self.data, self.flags = id_, list(data), flags
        class Stat:
            ERROR_ACTIVE=8; TX_PENDING=16; ERROR_PASSIVE=2; BUS_OFF=1
        class Channel:
            def __init__(self): self.driving=False
            def setBusParams(self, rate): pass
            def busOn(self): pass
            def busOff(self): pass
            def close(self): pass
            def writeSync(self, timeout): pass
            def readStatus(self): return 24
            def read_error_counters(self): return types.SimpleNamespace(tx=0, rx=0, overrun=0)
            def write(self, frame):
                sent.append((clock[0], frame))
                if frame.id == 0x1E3 and frame.data[2] == 97:
                    self.driving = True
                    raise RuntimeError('simulated driver write failure')
            # No read(): the worker must not touch the receive queue.
        channel = Channel()
        class Close:
            def is_set(self): return clock[0] > 11.
            def wait(self, dt): clock[0] += dt
        class Ready:
            def put(self, value):
                if value.get('ok'): shared[:] = [2., 100., 0.]
            put_nowait = put
        api = types.SimpleNamespace(Stat=Stat, openChannel=lambda _: channel,
            Bitrate=types.SimpleNamespace(BITRATE_125K=125000, BITRATE_250K=250000, BITRATE_500K=500000, BITRATE_1M=1000000))
        cfg = config()['can']; cfg['diagnostics'] = dict(enabled=True, status_period_s=5.)
        with patch.dict('sys.modules', {'canlib': types.SimpleNamespace(canlib=api, Frame=Frame)}), \
             patch('time.monotonic', side_effect=lambda: clock[0]), \
             patch('time.sleep', side_effect=lambda dt: clock.__setitem__(0, clock[0]+dt)), \
             patch('docking.canbus.signal.signal'), patch('docking.canbus.mp.parent_process', return_value=None):
            _worker(cfg, shared, abort, Close(), Ready(), health, reports, faults)
        self.assertTrue(abort.is_set())
        failure = faults.get_nowait()
        self.assertIn('write failure', failure['fault']); self.assertEqual(failure['stage'], 'running')
        self.assertEqual(failure['bus_status'], 24); self.assertIn('ERROR_ACTIVE', failure['bus_status_names'])
        self.assertEqual(failure['receive_queue'], 'not_read_by_design')
        self.assertIsNotNone(failure['worker_cpu_percent'])
        # Immediate neutral after the failure, then three control/heartbeat/neutral bundles at shutdown.
        after = [f for _, f in sent][[i for i, (_, f) in enumerate(sent) if f.id == 0x1E3 and f.data[2] == 97][0]+1:]
        self.assertEqual(after[0].id, 0x1E3); self.assertEqual(after[0].data, [127]*8)
        self.assertEqual([f.id for f in after[-9:]], [0x2E3, 0x764, 0x1E3]*3)
        self.assertTrue(all(f.data == [127]*8 for f in after if f.id == 0x1E3))
        can = object.__new__(Can)
        can.abort_event = abort
        with self.assertRaisesRegex(RuntimeError, 'latched'): can.set('forward_slow')
