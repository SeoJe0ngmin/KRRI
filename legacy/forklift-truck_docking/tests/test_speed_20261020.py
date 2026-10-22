"""2026-10-20 KW (user decision): plan acceptance tolerance follows the measured spread, far steps may be 1.5 m, and a late
planner reply re-requests instead of ending the run."""
import math,unittest
from docking.controller import Controller
from support import config,observation

def started():
    c=config();c['tuning']['preparation_s']=1.;x=Controller(c,-2.5,0.)
    x.step(0.,observation(0.),True)
    for i in range(1,12):x.step(i/10,observation(i/10))
    x.step(1.2,observation(1.2),True);return x

class SpeedTests(unittest.TestCase):
    def test_plan_accepted_when_wobble_is_within_measured_spread(self):
        x=started();x.state='PLAN';x.cut=-3.3;x.step(5.,observation(5.,world=(-8.,0.,0.)));self.assertEqual(x.state,'PLANNING')
        reply=dict(id=x.plan_id,plan=dict(actions=[dict(turn=0.,distance=.8,fine=False)],complete=False,reason='progress_within_horizon'))
        x.accept_plan(reply,7.4,observation(7.4,world=(-8.,.08,math.radians(1.)),confidence_radius=[.2,.2,math.radians(2.)]))
        self.assertEqual(x.state,'MOVING')          # 8 cm / 1 deg wobble inside a 20 cm / 2 deg spread: accepted
        y=started();y.state='PLAN';y.cut=-3.3;y.step(5.,observation(5.,world=(-8.,0.,0.)))
        y.accept_plan(dict(reply,id=y.plan_id),7.4,observation(7.4,world=(-8.,.08,0.),confidence_radius=[.01,.01,.001]))
        self.assertEqual(y.state,'PLAN')            # same wobble with a tight spread: re-plan as before
    def test_far_step_is_1_5m_and_near_step_0_8m(self):
        x=started();x.cut=-3.3
        x.begin_drive(5.,observation(5.,world=(-9.,0.,0.)),5.,False);self.assertAlmostEqual(x.action['target'],1.5)
        y=started();y.cut=-3.3
        y.begin_drive(5.,observation(5.,world=(-5.,0.,0.)),5.,False);self.assertAlmostEqual(y.action['target'],.8)
    def test_late_planner_rerequests_then_faults(self):
        x=started();x.state='PLAN';x.cut=-3.3;x.step(5.,observation(5.,world=(-8.,0.,0.)));self.assertEqual(x.state,'PLANNING')
        for n in range(3):
            x.await_plan['at']=5.;x.step(12.+n,observation(12.+n,world=(-8.,0.,0.)));self.assertEqual(x.state,'PLAN',n)   # re-requested on the next step
            self.assertEqual(x.planner_timeouts,n+1);x.step(12.5+n,observation(12.5+n,world=(-8.,0.,0.)));self.assertEqual(x.state,'PLANNING')
        x.await_plan['at']=5.;x.step(20.,observation(20.,world=(-8.,0.,0.)));self.assertEqual(x.state,'FAULT')

if __name__=='__main__':unittest.main()
