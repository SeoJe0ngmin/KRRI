import math
import queue
import threading
import types
import unittest
from unittest.mock import Mock, patch

import numpy as np

from docking.canbus import Can, SET_LOCK_WAIT_S
from docking.canhealth import BusStatus
from docking.controller import Controller
from docking.planner import worker as plan_worker
from docking.stream import ClockUnwrap
from support import config, observation


class ClockUnwrapTests(unittest.TestCase):
    def test_32bit_wrap_is_continuous_per_stream(self):
        u = ClockUnwrap(); period = 2**32/1e6
        self.assertAlmostEqual(u('gyro', period-.1), period-.1)
        self.assertAlmostEqual(u('gyro', .05), period+.05)
        self.assertAlmostEqual(u('color', 12.), 12.)
        self.assertAlmostEqual(u('gyro', .10), period+.10)
        self.assertEqual(u.wraps, 1)
        self.assertAlmostEqual(u('gyro', .09), period+.09)


class PlannerDrainTests(unittest.TestCase):
    def test_superseded_request_is_skipped(self):
        requests, results, stop = queue.Queue(), queue.Queue(), threading.Event()
        computed = []
        def fake_plan(cfg, pose, cut, responses):
            computed.append(pose); return dict(actions=[], complete=True, reason='x', expanded=0)
        requests.put(dict(id=1, pose=[1.], cut=-2.5, responses={}))
        requests.put(dict(id=2, pose=[2.], cut=-2.5, responses={}))
        with patch('docking.planner.plan', side_effect=fake_plan):
            t = threading.Thread(target=plan_worker, args=(config(), requests, results, stop)); t.start()
            reply = results.get(timeout=5); stop.set(); t.join(2)
        self.assertEqual(computed, [[2.]]); self.assertEqual(reply['id'], 2)


class CanSetLockTests(unittest.TestCase):
    def make(self, acquired, previous='forward'):
        can = Can.__new__(Can); can.lease_s = 1.; can.last_requested_command = previous
        can.abort_event = Mock(); can.abort_event.is_set.return_value = False
        can.process = Mock(); can.process.is_alive.return_value = True
        can.shared = Mock(); can.shared.get_lock.return_value.acquire.return_value = acquired
        can.shared.__setitem__ = Mock()
        return can

    def test_waits_for_the_mailbox_lock_before_giving_up(self):
        can = self.make(True); can.set('stop')
        can.shared.get_lock.return_value.acquire.assert_called_with(True, SET_LOCK_WAIT_S)
        self.assertEqual(SET_LOCK_WAIT_S, 1.0)
        can.abort_event.set.assert_not_called(); self.assertEqual(can.last_requested_command, 'stop')

    def test_moving_to_stop_still_latches_if_lock_never_frees(self):
        can = self.make(False); can.set('stop'); can.abort_event.set.assert_called_once()


class BusStatusTests(unittest.TestCase):
    def test_log_only_snapshot_and_errors_do_not_raise(self):
        class Stat: ERROR_ACTIVE = 8; TX_PENDING = 16; BUS_OFF = 1; ERROR_PASSIVE = 2
        ch = Mock(); ch.readStatus.return_value = 24
        ch.read_error_counters.return_value = types.SimpleNamespace(tx=3, rx=0, overrun=0)
        b = BusStatus(ch, types.SimpleNamespace(Stat=Stat), dict(status_period_s=5.))
        b.wrote(); r = b.poll(0.)
        self.assertEqual(r['tx_count'], 1); self.assertEqual(r['bus_status'], 24)
        self.assertEqual(r['bus_status_names'], ['ERROR_ACTIVE', 'TX_PENDING'])
        self.assertEqual(r['error_counters'], dict(tx=3, rx=0, overrun=0))
        self.assertIsNone(b.poll(1.))
        ch.readStatus.side_effect = RuntimeError('driver busy')
        r = b.poll(5.); self.assertIn('driver busy', r['status_error'])
        self.assertFalse(hasattr(ch, 'read') and ch.read.called)


class TransitionCorrectionTests(unittest.TestCase):
    def test_pure_turn_is_allowed_when_tag_margin_is_already_below_cut_margin(self):
        from docking.geometry import cut_position, tag_margin
        from docking.planner import plan
        c = config('left'); cut = cut_position(c)
        from docking.geometry import arrival_y, moved
        p = np.array([cut-.006, .15, -.031])
        self.assertLess(tag_margin(p, c), c['tuning']['cut_margin_px'])
        self.assertGreater(abs(arrival_y(p, c)), c['tuning']['position_tolerance_m'])
        r = plan(c, p, cut, {})
        self.assertNotEqual(r['reason'], 'no_feasible_progress', r)
        a = r['actions'][0]; self.assertTrue(a['turn'] != 0 and a['distance'] == 0, r)
        self.assertLess(abs(arrival_y(moved(p, a['turn'], 0, c['pivot']), c)), .01)


class EntryTests(unittest.TestCase):
    def started(self, mode):
        c = config(mode); c['tuning']['preparation_s'] = 1.
        from docking.geometry import cut_position
        x = Controller(c, cut_position(c), 0.)
        x.step(0., observation(0., mode=mode), True)
        for i in range(1, 12): x.step(i/10, observation(i/10, mode=mode))
        x.step(1.2, observation(1.2, mode=mode), True)
        return c, x

    def test_one_leg_for_the_whole_remaining_distance(self):
        c, x = self.started('left')
        from docking.geometry import remaining
        p = (x.cut-.006, 0., 0.)
        x.begin_entry(2, observation(2, mode='left', world=p), remaining(p, c), 'test')
        self.assertTrue(x.blind and x.state == 'MOVING' and x.action['blind']); self.assertEqual(x.phase, 'ENTRY')
        self.assertAlmostEqual(x.action['target'], remaining(p, c), places=6)
        self.assertGreater(x.action['target'], c['tuning']['forward_max_m'])
        r = x.responses.get('forward')
        self.assertAlmostEqual(x.action['hold_s'], r['startup']+x.action['target']/r['speed'])

    def test_vio_loss_finishes_on_time_model_and_completes_without_judgment(self):
        c, x = self.started('left')
        from docking.geometry import remaining
        p = (x.cut-.006, 0., 0.)
        x.begin_entry(2, observation(2, mode='left', world=p), remaining(p, c), 'test'); hold = x.action['hold_s']
        x.step(2.5, observation(2.5, mode='left', world=p, local=(.1, 0, 0)))
        lost = dict(local_valid=False, stationary=False, usable=False, valid=False, pose=None)
        x.step(3., observation(3., mode='left', **lost))
        self.assertEqual(x.state, 'MOVING'); self.assertEqual(x.command, 'forward'); self.assertTrue(x.action['timed'])
        x.step(2+hold-.1, observation(2+hold-.1, mode='left', **lost))
        self.assertEqual(x.command, 'forward')
        x.step(2+hold+.1, observation(2+hold+.1, mode='left', **lost))
        self.assertEqual(x.state, 'SETTLE'); self.assertEqual(x.command, 'stop')
        t = 2+hold+.1+c['tuning']['stable_s']+.1
        x.step(t, observation(t, mode='left', **lost))
        self.assertEqual(x.state, 'COMPLETE'); self.assertEqual(x.final_report['ended_by'], 'timed_fallback')
        self.assertFalse(x.final_report['pose_available'])

    def test_vio_progress_ends_the_leg_and_records_the_error(self):
        c, x = self.started('left')
        from docking.geometry import remaining
        p = (x.cut-.006, 0., 0.)
        x.begin_entry(2, observation(2, mode='left', world=p), remaining(p, c), 'test'); target = x.action['target']
        for k in range(1, 100):
            t = 2+k*.1; d = min(target+.05, .3*(t-2))
            x.step(t, observation(t, mode='left', world=(p[0]+d, 0, 0), local=(d, 0, 0), stationary=False))
            if x.state != 'MOVING': break
        self.assertEqual(x.state, 'SETTLE')
        x.step(t+2.5, observation(t+2.5, mode='left', world=(p[0]+d, 0, 0), local=(d, 0, 0)))
        self.assertEqual(x.state, 'COMPLETE'); self.assertEqual(x.final_report['ended_by'], 'vio_progress')
        self.assertIsNotNone(x.final_report['remaining_m'])

    def test_entry_no_progress_switches_to_the_time_model_instead_of_stopping(self):
        c, x = self.started('left')
        from docking.geometry import remaining
        p = (x.cut-.006, 0., 0.)
        x.begin_entry(2, observation(2, mode='left', world=p), remaining(p, c), 'test')
        x.step(2.5, observation(2.5, mode='left', world=(p[0]+.1, 0, 0), local=(.1, 0, 0), stationary=False))
        for k in range(30): x.step(2.6+k*.1, observation(2.6+k*.1, mode='left', world=(p[0]+.1, 0, 0), local=(.1, 0, 0), stationary=False))
        self.assertEqual(x.state, 'MOVING'); self.assertTrue(x.action['timed']); self.assertIn('entry_no_progress_timed', [e['event'] for e in x.events])


class ResponsesStartupTests(unittest.TestCase):
    def test_speed_uses_moving_time_not_command_time(self):
        from docking.planner import Responses
        r = Responses()
        r.observe('forward', progress=1., active_s=5., residual=.05, dead_s=1.2)
        self.assertAlmostEqual(r.get('forward')['speed'], .5*.28973+.5*(1/3.8))
        self.assertAlmostEqual(r.get('forward')['startup'], .5*1.498+.5*1.2)


class MotionFreshnessTests(unittest.TestCase):
    def test_moving_tolerates_1s_old_observation_but_rest_decisions_do_not(self):
        c = config(); c['tuning']['preparation_s'] = 1.
        x = Controller(c, -2.5, 0.)
        x.step(0., observation(0.), True)
        for i in range(1, 12): x.step(i/10, observation(i/10))
        x.step(1.2, observation(1.2), True)
        o = observation(2); x.set_legs(2, o, [dict(kind='drive', value=.5, label='t')], 't'); x.next_leg(2, o); self.assertEqual(x.state, 'MOVING')
        x.step(3.8, observation(2.1, local=(.05, 0, 0), stationary=False))
        self.assertEqual(x.state, 'MOVING'); self.assertEqual(x.command, 'forward')
        x.step(5.2, observation(2.1, local=(.05, 0, 0), stationary=False))
        self.assertEqual(x.state, 'RECOVER')
        self.assertFalse(x.fresh(5., observation(4.5)))
        self.assertFalse(hasattr(x.t, 'first_forward_m'))


class FineTurnFloorTests(unittest.TestCase):
    def test_final_zone_turn_floor_is_0_84_deg(self):
        from docking.geometry import cut_position
        from docking.planner import plan
        c = config('left'); cut = cut_position(c)
        self.assertEqual(c['tuning']['minimum_turn_fine_deg'], .84)
        p = np.array([cut-.006, .02, -.0005])
        r = plan(c, p, cut, {})
        for a in r['actions']:
            if a['turn']: self.assertGreaterEqual(abs(a['turn']), math.radians(.84)-1e-9)
