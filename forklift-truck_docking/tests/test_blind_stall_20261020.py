"""2026-10-20 KW: the operator e-stopped at the entrance and the blind leg kept commanding forward for 18 s until the
time limit (FAULT). Now 3 s without VIO progress while driving finishes the leg and records it (user decision: no fault)."""
import unittest
from docking.controller import Controller
from support import config,observation

class BlindStallTests(unittest.TestCase):
    def test_no_progress_for_3s_finishes_the_blind_leg(self):
        c=config();x=Controller(c,-2.5,0.);x.state='PLAN'
        o=observation(10.,world=(-2.4,0.,0.));x.begin_blind(10.,o,2.0);self.assertTrue(x.action['blind'])
        t=10.;xl=0.
        for k in range(15):t+=.1;xl+=.03;x.step(t,observation(t,world=(-2.4+xl,0.,0.),local=(xl,0.,0.),stationary=False))   # 1.5 s of motion
        self.assertEqual(x.state,'MOVING')
        for k in range(40):   # held: no VIO progress
            t+=.1;x.step(t,observation(t,world=(-2.4+xl,0.,0.),local=(xl,0.,0.),stationary=False))
            if x.state!='MOVING':break
        self.assertEqual(x.state,'SETTLE');self.assertEqual(x.post_settle,'COMPLETE');self.assertLess(t-11.5,3.6)
        self.assertIn('blind_entry_stalled_recorded',[e['event'] for e in x.events])

if __name__=='__main__':unittest.main()
