import copy
import math
import unittest
from docking.controller import Controller
from docking.core import Policy, synthesize
from test_core import config, observation


class DeadlineTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = config()
        cls.policy = Policy(cls.cfg, synthesize(cls.cfg))

    def moving(self, delayed=False):
        c = Controller(self.cfg, self.policy)
        key = tuple(map(int, next(k for k, v in self.policy.table['nodes'].items() if v[0] == 1).split(',')))
        pose = self.policy.grid.belief(key).center
        c.step(1., observation(pose, 1.))
        c.step(1.5, observation(pose, 1.5))
        self.assertEqual(c.state, 'MOVING')
        c.action = copy.deepcopy(c.action)
        if delayed:
            c.action.update(max_start_delay_s=20., max_duration_s=2., max_stop_response_s=10.)
        return c, pose

    def test_waiting_without_frames_still_has_global_deadline(self):
        c = Controller(self.cfg, self.policy)
        c.step(0., None)
        c.step(301., None)
        self.assertEqual(c.reason, 'run_timeout')

    def test_first_frame_after_bias_reset_waits_for_timed_imu_interval(self):
        c = Controller(self.cfg, self.policy)
        o = observation([0, 0, 0], 1.)
        o.update(sensor_ready=False, imu_bad=True, imu={'ready': True})
        c.step(1., o)
        self.assertEqual(c.state, 'WAITING')
        self.assertEqual(c.reason, 'waiting_for_ready_sensor_interval')

    def test_primitive_timeout_checked_even_on_repeated_frame(self):
        c, pose = self.moving()
        c.step(c.started+c.action['max_duration_s']+.01, observation(pose, 1.5))
        self.assertEqual(c.reason, 'primitive_timeout')

    def test_start_latency_does_not_consume_motion_deadline(self):
        c, pose = self.moving(delayed=True)
        c.step(10., observation(pose, 10.))
        self.assertEqual(c.state, 'MOVING')
        self.assertIsNone(c.motion_started)
        c.step(22., observation(pose, 22.))
        self.assertEqual(c.reason, 'command_start_timeout')

    def test_stop_latency_has_separate_budget(self):
        c, pose = self.moving(delayed=True)
        c.state, c.braked, c.command = 'BRAKING', 2., 'stop'
        c.step(6., observation(pose, 6., speed=.1))
        self.assertEqual(c.state, 'BRAKING')
        c.step(16., observation(pose, 16., speed=.1))
        self.assertEqual(c.reason, 'stop_not_observed')

    def test_motion_deadline_begins_only_after_observed_displacement(self):
        c, pose = self.moving(delayed=True)
        p = list(pose)
        sign = 1 if c.action['target'] > 0 else -1
        if c.action['kind'] == 'rotate':
            p[2] += sign*.03
        else:
            p[0] += sign*.03*math.cos(p[2])
            p[1] += sign*.03*math.sin(p[2])
        c.step(10., observation(p, 10.))
        self.assertEqual(c.motion_started, 10.)
        self.assertEqual(c.state, 'MOVING')
        c.step(11.5, observation(p, 11.5))
        self.assertEqual(c.state, 'MOVING')
        c.step(12.1, observation(p, 12.1))
        self.assertEqual(c.reason, 'primitive_timeout')

    def test_maximum_action_count_latches_stop(self):
        c, pose = self.moving()
        c.transitions = self.cfg['limits'].get('max_steps', 20)
        c.state, c.rank, c.expected, c.still_since = 'WAITING', None, None, None
        c.step(2., observation(pose, 2.))
        c.step(2.5, observation(pose, 2.5))
        self.assertEqual(c.reason, 'maximum_actions_reached')
