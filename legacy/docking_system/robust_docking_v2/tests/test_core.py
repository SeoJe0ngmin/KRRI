import copy
import json
import math
import os
import random
import unittest

from docking.core import (Box, Grid, Policy, action_box, body_aabb, fingerprint,
                          safe, synthesize, transition, trig_bounds, validate)
from docking.controller import Controller
from docking.canbus import templates, effective, NAMES
from docking.stream import Latest, Imu
from docking.runtime import check_sensor
from contract_checks import simulate

ROOT = os.path.dirname(os.path.dirname(__file__))


def config():
    with open(os.path.join(ROOT, 'tests', 'fixtures', 'policy_contract.json')) as f:
        return json.load(f)


def observation(pose, t, error=(0, 0, 0), speed=0., yaw=0.):
    return {'pose': pose, 'capture_lower': t, 'capture_id': t, 'published': t,
            'bounded': True, 'valid': True, 'error': error, 'speed_upper': speed,
            'yaw_rate_upper': yaw, 'imu_bad': False}


class GeometryTests(unittest.TestCase):
    def test_interval_trig_contains_interior_extrema(self):
        self.assertEqual(trig_bounds(0, math.pi), (0., 1.))
        self.assertEqual(trig_bounds(-math.pi, math.pi, True), (-1., 1.))

    def test_full_rectangle_rotation_envelope(self):
        c = config()
        box = Box([0, 0, -.2], [.1, .1, .4])
        a = body_aabb(box, c['body'])
        rng = random.Random(3)
        for _ in range(1000):
            x, y, h = [rng.uniform(l, u) for l, u in zip(box.lo, box.hi)]
            for dx in (-c['body']['rear_m'], c['body']['front_m']):
                for dy in (-c['body']['half_width_m'], c['body']['half_width_m']):
                    px, py = x+dx*math.cos(h)-dy*math.sin(h), y+dx*math.sin(h)+dy*math.cos(h)
                    self.assertTrue(a[0] <= px <= a[1] and a[2] <= py <= a[3])

    def test_endpoint_safe_but_rotation_sweep_unsafe(self):
        c = config()
        c['body'] = {'front_m': .5, 'rear_m': .5, 'half_width_m': .1}
        c['scene'] = {'clearance_m': 0., 'free_rectangles': [[-2, 2, -.2, .2]]}
        b = Box.around([0, 0, 0], [0, 0, 0])
        a = dict(c['actions'][3], target=math.pi, terminal_error=[0, 0, 0], sweep_error=[0, 0, 0])
        self.assertTrue(safe(b, c))
        self.assertTrue(safe(action_box(b, a), c))
        self.assertIsNone(transition(b, a, c))

    def test_feedback_target_contracts_heading(self):
        c = config()
        b = Box.around([0, 0, 0], [.01, .01, .03])
        end = action_box(b, c['actions'][3])
        self.assertAlmostEqual(end.radius[2], .004)

    def test_heading_uncertainty_changes_translation_lateral_extent(self):
        a = config()['actions'][1]
        narrow = action_box(Box.around([0, 0, 0], [.02, .01, 0]), a)
        wide = action_box(Box.around([0, 0, 0], [.02, .01, .1]), a)
        self.assertGreater(wide.radius[1], narrow.radius[1]+.07)

    def test_invalid_boxes_and_unknown_profile_rejected(self):
        for lo, hi in [([1, 0, 0], [0, 0, 0]), ([float('nan'), 0, 0], [1, 1, 1])]:
            with self.assertRaises(ValueError):
                Box(lo, hi)
        with open(os.path.join(ROOT, 'configs', 'hardware.json')) as f:
            with self.assertRaises(ValueError):
                validate(json.load(f), live=True)


class PolicyTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cfg = config()
        cls.table = synthesize(cls.cfg)
        cls.policy = Policy(cls.cfg, cls.table)

    def test_nontrivial_finite_winning_region(self):
        self.assertGreater(self.table['winning_nodes'], 100)
        self.assertLess(self.table['winning_nodes'], self.table['total_nodes'])
        self.assertGreater(self.table['layers'], 2)

    def test_all_successors_have_strictly_lower_rank(self):
        self.policy.verify()
        for name, (rank, ai) in self.table['nodes'].items():
            if rank == 0:
                continue
            key = tuple(map(int, name.split(',')))
            end = transition(self.policy.grid.belief(key), self.cfg['actions'][ai], self.cfg)
            for s in self.policy.grid.possible_observations(end):
                r = self.table['nodes'][','.join(map(str, s))][0]
                self.assertLess(r, rank)

    def test_nominal_only_or_corrupted_certificate_rejected(self):
        p = copy.deepcopy(self.table)
        key = next(k for k, v in p['nodes'].items() if v[0] > 0)
        p['nodes'][key] = [0, None]
        with self.assertRaisesRegex(ValueError, 'terminal'):
            Policy(self.cfg, p)

    def test_policy_hash_covers_sensor_and_geometry(self):
        c = copy.deepcopy(self.cfg)
        c['sensor']['camera']['width'] = 1280
        with self.assertRaises(ValueError):
            Policy(c, self.table)

    def test_nearest_centre_is_not_enough(self):
        k = tuple(map(int, next(iter(self.table['nodes'])).split(',')))
        b = self.policy.grid.belief(k)
        self.assertIsNone(self.policy.lookup(b.center, b.inflate([1, 1, 1])))

    def test_1000_random_all_outcome_contract_runs(self):
        r = simulate(self.cfg, self.table, 1000, 11)
        self.assertTrue(r['all_reached_goal'])

    def test_observation_bins_at_domain_boundary_fail_closed(self):
        g = self.policy.grid
        self.assertIsNone(g.possible_observations(Box.around(self.cfg['grid']['lo'], [0, 0, 0])))

    def test_controller_observed_stop_to_goal(self):
        c = Controller(self.cfg, self.policy)
        # A rank-one action provides a complete end-to-end controller episode.
        key = tuple(map(int, next(k for k, v in self.table['nodes'].items() if v[0] == 1).split(',')))
        pose = self.policy.grid.belief(key).center
        t = 10.
        c.step(t, observation(pose, t))
        t += .5
        c.step(t, observation(pose, t))
        self.assertEqual(c.state, 'MOVING')
        target = action_box(c.start_box, c.action).center
        t += 1.
        c.step(t, observation(target, t, speed=.1))
        self.assertEqual(c.state, 'BRAKING')
        t += .1
        c.step(t, observation(target, t))
        t += .5
        c.step(t, observation(target, t))
        self.assertEqual(c.state, 'DONE', c.status())
        self.assertEqual(c.command, 'stop')

    def moving_controller(self):
        key = tuple(map(int, next(k for k, v in self.table['nodes'].items() if v[0] == 1).split(',')))
        pose = self.policy.grid.belief(key).center
        c = Controller(self.cfg, self.policy)
        c.step(1., observation(pose, 1.))
        c.step(1.5, observation(pose, 1.5))
        self.assertEqual(c.state, 'MOVING')
        return c, pose

    def test_dropout_latches_stop_and_cannot_restart(self):
        c, pose = self.moving_controller()
        c.step(1.6, None)
        self.assertEqual(c.state, 'FAULT')
        c.step(2., observation(pose, 2.))
        self.assertEqual(c.command, 'stop')
        self.assertEqual(c.state, 'FAULT')

    def test_stale_repeated_frame_is_not_fresh(self):
        c, pose = self.moving_controller()
        c.step(2., observation(pose, 1.5))
        self.assertEqual(c.reason, 'observation_too_old_or_clock_invalid')

    def test_repeated_frame_cannot_satisfy_stop_dwell(self):
        c = Controller(self.cfg, self.policy)
        p = [2.3, .04, .03]
        o = observation(p, 1.)
        c.step(1., o)
        for i in range(10):
            c.step(1.01+i*.01, o)
        self.assertEqual(c.state, 'WAITING')

    def test_acceleration_quiet_alone_is_not_stopped(self):
        c = Controller(self.cfg, self.policy)
        for t in (1., 1.5, 2.):
            o = observation([2.3, .04, .03], t, speed=None)
            c.step(t, o)
        self.assertEqual(c.state, 'WAITING')

    def test_goal_midpoint_with_large_error_cannot_finish(self):
        c = Controller(self.cfg, self.policy)
        for t in (1., 1.5):
            c.step(t, observation([2.3, 0, 0], t, error=[.8, .2, .2]))
        self.assertNotEqual(c.state, 'DONE')


class StreamAndCanTests(unittest.TestCase):
    def test_camera_buffer_is_latest_only(self):
        q = Latest()
        for i in range(30):
            q.put(i)
        self.assertEqual(q.get(), 29)
        self.assertEqual(q.replaced, 29)
        self.assertIsNone(q.get(timeout=0))

    def test_imu_integrates_all_samples_and_flags_gap(self):
        c = config()['sensor']['imu']
        imu = Imu(c)
        for i in range(201):
            imu.add(i/200, (0, -1, 0), i/200)
        self.assertAlmostEqual(imu.snapshot(1.)['yaw_rad'], 1.)
        self.assertAlmostEqual(imu.between(.1, .9), .8)
        imu.add(1.2, (0, -1, 0), 1.2)
        self.assertEqual(imu.snapshot(1.2)['gaps'], 1)
        self.assertIsNone(imu.between(.9, 1.2))

    def test_can_fork_bytes_stay_neutral(self):
        frames = templates(config()['can'])
        self.assertEqual(frames['forward'][2], 67)
        self.assertEqual(frames['forward_slow'][2], 97)
        self.assertEqual(frames['backward'][2], 187)
        for frame in frames.values():
            self.assertTrue(all(frame[i] == 127 for i in (0, 3, 4, 5, 6, 7)))

    def test_can_lease_expiry_and_latch(self):
        self.assertEqual(effective(1, 2., False, 1.), (1, False))
        self.assertEqual(effective(1, 2., False, 2.), (0, True))
        self.assertEqual(effective(1, 100., True, 3.), (0, True))
        self.assertEqual(effective(0, 0., False, 3.), (0, False))

    def test_measurement_profile_cannot_arm_hardware(self):
        sensor = config()['sensor']
        check_sensor(sensor)
        with self.assertRaises(ValueError):
            check_sensor(sensor, live=True)


if __name__ == '__main__':
    unittest.main()
