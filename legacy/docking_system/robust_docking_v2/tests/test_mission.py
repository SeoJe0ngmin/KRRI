import copy
import json
import os
import unittest
from docking.core import Box, Grid, safe
from docking.mission import build, load_policy
from docking.controller import Controller
from contract_checks import simulate
from test_core import ROOT, observation
from test_recovery import options, seen


class MissionTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        with open(os.path.join(ROOT, 'tests', 'fixtures', 'sidestep_contract.json')) as f:
            cls.cfg = json.load(f)
        cls.cfg['recovery'] = options()
        cls.table = build(cls.cfg)
        cls.prototype = load_policy(cls.cfg, cls.table)

    def test_conditional_turn_translate_restore_handover_and_stop(self):
        policy = copy.copy(self.prototype)
        c = Controller(self.cfg, policy)
        pose, t = [-1.5, .8, 0.], 1.
        c.step(t, seen(pose, t, tag=True, anchor=t))
        t += .5
        c.step(t, seen(pose, t, tag=True, anchor=t))
        phases, kinds = [], []
        for _ in range(20):
            if c.state != 'MOVING':
                break
            phases.append(c.phase)
            kinds.append(c.action['kind'])
            pose = c.expected.center
            t += .1
            c.step(t, seen(pose, t, tag=True, anchor=t))
            self.assertEqual(c.state, 'BRAKING', c.status())
            t += .1
            c.step(t, seen(pose, t, tag=True, anchor=t))
            t += .5
            c.step(t, seen(pose, t, tag=True, anchor=t))
        self.assertEqual(c.state, 'DONE', c.status())
        self.assertEqual(kinds[:3], ['rotate', 'translate', 'rotate'])
        self.assertEqual(phases[:3], ['SIDESTEP']*3)
        self.assertEqual(phases[-1], 'APPROACH')

    def test_all_outcome_certificates_and_1000_random_missions(self):
        result = simulate(self.cfg, self.table, 1000, 41)
        self.assertTrue(result['all_reached_goal'])

    def test_normal_approach_has_priority(self):
        grid = Grid(self.cfg)
        key = tuple(map(int, next(k for k, v in self.table['approach']['nodes'].items() if v[0] > 0).split(',')))
        b = grid.belief(key)
        q = copy.copy(self.prototype).lookup(b.center, b)
        self.assertEqual(q['phase'], 'APPROACH')

    def test_interrupted_sidestep_returns_reanchors_and_finishes_real_mission(self):
        c = Controller(self.cfg, copy.copy(self.prototype))
        pose = [-1.5, .8, 0.]
        c.step(1., seen(pose, 1., tag=True))
        c.step(1.5, seen(pose, 1.5, tag=True, anchor=1.5))
        self.assertEqual(c.phase, 'SIDESTEP')
        # Complete the outward turn; the planner starts the lateral leg.
        pose = c.expected.center
        for t in (1.6, 1.7, 2.2): c.step(t, seen(pose, t, anchor=1.5))
        self.assertEqual(c.action['kind'], 'translate')
        pose = c.expected.center
        lost = seen(pose, 4., anchor=1.5)
        lost.update(valid=False, bounded=False, pose=None, error=None)
        c.step(4., lost)
        self.assertEqual(c.state, 'PAUSING', c.status())
        for t in (4.1, 4.3, 4.6): c.step(t, seen(pose, t, anchor=1.5))
        self.assertTrue(c.returning, c.status())
        pose = c.expected.center
        for t in (4.7, 4.8, 5.3): c.step(t, seen(pose, t, anchor=1.5))
        self.assertEqual(c.state, 'REOBSERVING', c.status())
        self.assertEqual(c.command, 'stop')
        # Fresh repeated tag poses release recovery into the real MissionPolicy.
        for t in (5.4, 5.6, 5.8, 6., 6.3): c.step(t, seen(pose, t, tag=True, anchor=t))
        self.assertEqual(c.state, 'MOVING', c.status())
        self.assertEqual(c.phase, 'APPROACH')
        self.assertEqual(c.recovery_attempts, 1)
        self.assertEqual(c.return_actions, 1)
        self.assertEqual(c.run_started, 1.)
        t = 6.3
        for _ in range(20):
            if c.state != 'MOVING': break
            pose = c.expected.center
            for dt in (.1, .1, .5):
                t += dt
                c.step(t, seen(pose, t, tag=True, anchor=t))
        self.assertEqual(c.state, 'DONE', c.status())

    def test_small_lateral_error_does_not_trigger_fallback(self):
        policy = copy.copy(self.prototype)
        # The side policy can recover some large heading errors, but that
        # alone must not activate the conditional lateral correction.
        pose = [-.7, 0., -.6]
        b = Box.around(pose, [0, 0, 0])
        self.assertIsNone(policy.normal.lookup(pose, b))
        self.assertIsNotNone(policy.side.lookup(pose, b))
        self.assertIsNone(policy.lookup(pose, b))

    def test_side_phase_never_enters_corridor(self):
        b = Box.around([1.5, 0., 0.], [0, 0, 0])
        self.assertTrue(safe(b, self.prototype.normal.cfg))
        self.assertFalse(safe(b, self.prototype.side.cfg))

    def test_tampered_handover_cannot_claim_success(self):
        table = copy.deepcopy(self.table)
        k = next(k for k, v in table['sidestep']['nodes'].items() if v[0] > 0)
        table['sidestep']['nodes'][k] = [0, None]
        with self.assertRaises(ValueError):
            load_policy(self.cfg, table)

    def test_sidestep_deadline_does_not_depend_on_new_video(self):
        c = Controller(self.cfg, copy.copy(self.prototype))
        pose = [-1.5, .8, 0.]
        c.step(1., observation(pose, 1.))
        c.step(1.5, observation(pose, 1.5))
        self.assertEqual(c.phase, 'SIDESTEP')
        c.state = 'WAITING'  # Isolate mission limit from primitive timeout.
        c.step(122., None)
        self.assertEqual(c.reason, 'sidestep_timeout')
