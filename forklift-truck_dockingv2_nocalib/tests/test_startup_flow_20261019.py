"""Start-up order (2026-10-19 user decision): short still check -> Enter -> search if no tag -> 30 s baseline with the
tag in view -> plan. Plus the online mount yaw learned from good straight legs and the no-good-leg rule."""
import math
import unittest
import numpy as np
from docking.controller import Controller
from docking.fusion import fit_mount_yaw
from docking.observer import Observer
from support import config,observation,synthetic_tag,VioSource


class StartupFlowTests(unittest.TestCase):
    def ready(self,c):
        x=Controller(c,-2.5,0.);x.step(0.,observation(0.),True)
        for i in range(1,12):x.step(i/10,observation(i/10))
        self.assertEqual(x.state,'WAIT_ENTER');return x
    def test_tag_visible_at_enter_then_30s_baseline_then_plan(self):
        c=config();c['tuning']['preparation_s']=1.;c['tuning']['baseline_s']=3.
        x=self.ready(c);x.step(1.2,observation(1.2),True);x.step(1.3,observation(1.3))
        self.assertEqual(x.state,'BASELINE');self.assertFalse(x.baseline_done)
        for i in range(14,46):x.step(i/10,observation(i/10))
        self.assertTrue(x.baseline_done);self.assertEqual(x.state,'PLANNING')   # baseline over, first plan requested
    def test_no_tag_at_enter_searches_first_then_baseline(self):
        c=config();c['tuning']['preparation_s']=1.;c['tuning']['baseline_s']=3.
        x=self.ready(c);lost=dict(valid=False,usable=False,pose=None,tag=dict(reason='missing_or_duplicate_id',detected=False))
        x.step(1.2,observation(1.2,**lost),True);x.step(4.,observation(4.,**lost))
        self.assertEqual(x.state,'MOVING');self.assertEqual(x.action['after'],'SEARCH');self.assertFalse(x.baseline_done)
        x.step(4.2,observation(4.2,local=(0,0,.1),stationary=False,**dict(lost,tag=dict(reason='accepted',detected=True))))
        self.assertEqual(x.state,'SETTLE')
        x.step(7.,observation(7.,local=(0,0,.3)));x.step(7.1,observation(7.1,local=(0,0,.3)))
        self.assertEqual(x.state,'BASELINE')                                   # found: now the 30 s (3 s here) at rest
        for i in range(72,105):x.step(i/10,observation(i/10,local=(0,0,.3)))
        self.assertTrue(x.baseline_done);self.assertEqual(x.state,'PLANNING')
    def test_baseline_waits_through_a_vio_blip_but_restarts_the_search_when_the_tag_is_gone(self):
        c=config();c['tuning']['preparation_s']=1.;c['tuning']['baseline_s']=3.
        x=self.ready(c);x.step(1.2,observation(1.2),True);x.step(1.3,observation(1.3));self.assertEqual(x.state,'BASELINE')
        x.step(2.,observation(2.,valid=False,usable=False,pose=None))                      # VIO reset, tag still detected
        self.assertEqual(x.state,'BASELINE');self.assertEqual(x.reason,'baseline_waiting_for_pose')
        lost=dict(valid=False,usable=False,pose=None,tag=dict(reason='missing',detected=False))
        x.step(3.,observation(3.,**lost));self.assertEqual(x.state,'BASELINE')             # tag gone 0 s
        x.step(5.5,observation(5.5,**lost));self.assertEqual(x.state,'PLAN');self.assertFalse(x.baseline_done)   # gone > 2 s: search
    def test_no_good_straight_leg_still_enters_and_is_recorded(self):
        # 2026-10-19 user decision: never stop for this; a bad entry is a result to accept, so only record it.
        c=config();x=Controller(c,-2.5,0.);x.state='PLAN';x.mission_start=0.;x.baseline_done=True
        x.step(1,observation(1,world=(-2.5,0,0),good_legs=0))
        self.assertTrue(x.blind);self.assertEqual(x.command,'forward');self.assertTrue(x.status()['mount_yaw_unlearned'])
        self.assertEqual(x.status()['events'][0]['event'],'entering_with_unlearned_mount_yaw')
        x=Controller(c,-2.5,0.);x.state='PLAN';x.mission_start=0.;x.baseline_done=True
        x.step(1,observation(1,world=(-2.5,0,0),good_legs=2));self.assertTrue(x.blind);self.assertFalse(x.status()['mount_yaw_unlearned'])


class OnlineMountYawTests(unittest.TestCase):
    def test_fit_recovers_a_mount_error_from_a_straight_leg(self):
        tag=np.array([0.,.3]);eps=math.radians(2.);frames=[]
        for k in range(12):
            p=np.array([-4.+.08*k,0.]);v=tag-p;d=np.hypot(*v);b_true=math.atan2(v[1],v[0])
            frames.append((b_true-eps,d,0.,p))                                # measured bearing is off by -eps
        e,res=fit_mount_yaw(frames);self.assertAlmostEqual(e,eps,places=6);self.assertLess(res,1e-6)
    def test_observer_learns_the_mount_yaw_and_fixes_the_heading(self):
        """Config mount yaw 0, real camera yawed +2 deg: heading is 2 deg off until one good straight leg."""
        import cv2
        c=config();s=c['sensor'];s['calibration']['T_body_imu']=np.eye(4).tolist()
        true_sensor=__import__('copy').deepcopy(s);T=np.array(true_sensor['calibration']['T_body_camera'])
        Rz=cv2.Rodrigues(np.array([0,0,math.radians(2.)]))[0];T[:3,:3]=Rz@T[:3,:3];true_sensor['calibration']['T_body_camera']=T.tolist()
        o=Observer(s);src=VioSource();t=0.
        def rest(pose,n):
            nonlocal t;r=None
            for _ in range(n):
                for k in range(7):o.add_gyro(t+k/200,[0.,0.,0.])
                r=o.add_vio(src.packet(t,pose));o.add_tag(synthetic_tag(true_sensor,pose,t));t+=.1
            return r
        r=rest((-4.,0.,0.),12);self.assertAlmostEqual(math.degrees(r['pose'][2]),2.,delta=.3)   # wrong by the mount error
        self.assertEqual(r['good_legs'],0)
        for k in range(12):                                                        # 0.96 m straight leg, moving
            o.add_vio(src.packet(t,(-4.+.08*(k+1),0.,0.),gyro_norm=.05));o.add_tag(synthetic_tag(true_sensor,(-4.+.08*(k+1),0.,0.),t));t+=.1
        r=rest((-3.04,0.,0.),12)
        self.assertEqual(r['good_legs'],1);self.assertAlmostEqual(r['mount_yaw_online_deg'],2.,delta=.1)
        self.assertAlmostEqual(math.degrees(r['pose'][2]),0.,delta=.3)             # heading corrected
        self.assertAlmostEqual(r['pose'][0],-3.04,delta=.02);self.assertAlmostEqual(r['pose'][1],0.,delta=.02)

if __name__=='__main__':unittest.main()


class OvershootTests(unittest.TestCase):
    def test_starting_past_the_target_backs_out_once_or_finishes(self):
        c=config();x=Controller(c,-2.5,0.);x.state='PLAN';x.mission_start=0.;x.baseline_done=True
        x.step(1,observation(1,world=(-1.7,0,0)))                                  # 30 cm past: one backward step
        self.assertEqual(x.state,'MOVING');self.assertEqual(x.command,'backward');self.assertAlmostEqual(x.action['target'],.3,places=6)
        x=Controller(c,-2.5,0.);x.state='PLAN';x.mission_start=0.;x.baseline_done=True
        x.step(1,observation(1,world=(-1.9,0,0)))                                  # 10 cm past: below the minimum step -> finish
        self.assertEqual(x.state,'COMPLETE');self.assertEqual(x.reason,'already_past_target_recorded_not_judged')
