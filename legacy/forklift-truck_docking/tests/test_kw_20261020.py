"""2026-10-20 KW field decisions: (1) straight steps stop on the VIO or the tag-anchored displacement, whichever is
larger; (2) one sidestep when the start is far off the centreline; (3) one backward step when the final zone has no
feasible candidate."""
import math,unittest
from docking.controller import Controller
from support import config,observation

def started(c=None):
    c=c or config();c['tuning']['preparation_s']=1.;x=Controller(c,-2.5,0.)
    x.step(0.,observation(0.),True)
    for i in range(1,12):x.step(i/10,observation(i/10))
    x.step(1.2,observation(1.2),True);return x

class TagRulerTests(unittest.TestCase):
    def test_step_stops_on_tag_motion_when_vio_is_frozen(self):
        x=started();x.begin_drive(2.,observation(2.),.8,False);self.assertEqual(x.state,'MOVING')
        t=2.;xw=-5.
        for k in range(40):
            t+=.1;xw+=.029;x.step(t,observation(t,world=(xw,0.,0.),local=(0.,0.,0.),stationary=False))
            if x.state!='MOVING':break
        self.assertEqual(x.state,'SETTLE');self.assertLess(xw+5.,1.0)
        parts=x.brake['progress_parts'];self.assertAlmostEqual(parts[0],0.,places=6);self.assertGreater(parts[1],.5)
    def test_vio_alone_still_stops_when_the_tag_is_lost(self):
        x=started();x.begin_drive(2.,observation(2.),.5,False)
        t=2.;xl=0.
        for k in range(40):
            t+=.1;xl+=.029
            x.step(t,observation(t,local=(xl,0.,0.),stationary=False,usable=False,pose=None,tag=dict(ok=False,detected=False,reason='no_tag')))
            if x.state!='MOVING':break
        self.assertIn(x.state,('SETTLE','RECOVER'),x.status()['reason'])

class LastResortBackwardTests(unittest.TestCase):
    def test_no_feasible_progress_backs_once_then_faults(self):
        x=started();x.state='PLAN';o=observation(5.,world=(-3.3,-.2,0.))
        x.step(5.,o);self.assertEqual(x.state,'PLANNING')
        x.accept_plan(dict(id=x.plan_id,plan=dict(actions=[],complete=False,reason='no_feasible_progress')),5.1,observation(5.1,world=(-3.3,-.2,0.)))
        self.assertEqual(x.state,'MOVING');self.assertEqual(x.command,'backward');self.assertAlmostEqual(x.action['target'],.5)
        self.assertIn('no_feasible_progress_backing_once',[e['event'] for e in x.events])
        x.stop(6.,'PLAN');x.step(8.,observation(8.,world=(-3.8,-.2,0.)));x.step(8.1,observation(8.1,world=(-3.8,-.2,0.)))
        self.assertEqual(x.state,'PLANNING')
        x.accept_plan(dict(id=x.plan_id,plan=dict(actions=[],complete=False,reason='no_feasible_progress')),8.2,observation(8.2,world=(-3.8,-.2,0.)))
        self.assertEqual(x.state,'FAULT')

class SidestepTests(unittest.TestCase):
    def test_far_off_centreline_start_sidesteps_once(self):
        x=started();x.state='PLAN';x.cut=-3.3
        p=(-7.,-1.,0.)                                  # 1 m right of the centreline, 3.7 m of room
        x.step(5.,observation(5.,world=p));self.assertEqual(x.state,'MOVING');self.assertEqual(x.command,'rotate_left_slow')   # face +90 deg
        self.assertAlmostEqual(x.action['target'],math.pi/2,places=6);self.assertEqual(x.action['after'],'SIDESTEP')
        self.assertEqual([k for k,_ in x.sidestep_legs],['drive','turn']);self.assertAlmostEqual(x.sidestep_legs[0][1],1.,places=6)   # heading 0: the swings cancel
        self.assertIn('sidestep',[e['event'] for e in x.events])
        # a second plan request on the same spot does not sidestep again
        x.sidestep_legs=[];x.action=None;x.state='PLAN';x.step(20.,observation(20.,world=p));self.assertEqual(x.state,'PLANNING')
    def test_no_sidestep_when_close_or_near_centreline(self):
        for p in ((-7.,-.3,0.),(-5.,-1.,0.)):
            x=started();x.state='PLAN';x.cut=-3.3;x.step(5.,observation(5.,world=p));self.assertEqual(x.state,'PLANNING',p)
    def test_sidestep_legs_tolerate_a_lost_tag(self):
        x=started();x.state='PLAN';x.cut=-3.3;x.step(5.,observation(5.,world=(-7.,-1.,0.)))
        t=5.;yaw=0.
        for k in range(100):   # turning with the tag out of view: must not recover/fault
            t+=.1;yaw=min(math.pi/2,yaw+math.radians(12)*.1)
            x.step(t,observation(t,world=(-7.,-1.,yaw),local=(0.,0.,yaw),stationary=False,tag=dict(ok=False,detected=False,reason='no_tag')))
            if x.state!='MOVING':break
        self.assertEqual(x.state,'SETTLE',x.status()['reason'])

class SidestepGeometryTests(unittest.TestCase):
    """2026-10-21: the drive covers the pivot swing of both in-place turns, so the body ends on the centreline from any start heading."""
    def run_sidestep(self,p):
        from docking.geometry import moved,wrap
        x=started();x.state='PLAN';x.cut=-3.3;x.step(5.,observation(5.,world=p))
        self.assertEqual(x.action['after'],'SIDESTEP')
        face=x.sidestep_legs and None
        ev=[e for e in x.events if e['event']=='sidestep'][-1]
        # replay the three legs with the planner's kinematics
        q=moved(p,wrap((math.pi/2 if p[1]<0 else -math.pi/2)-p[2]),0,x.cfg['pivot'])
        q=moved(q,0,x.sidestep_legs[0][1],x.cfg['pivot']);q=moved(q,wrap(0.-q[2]),0,x.cfg['pivot'])
        return x,ev,q
    def test_lands_on_the_centreline_from_oblique_starts(self):
        for p in ((-7.,-1.,math.radians(34.)),(-7.,.66,math.radians(-22.)),(-7.,-.8,math.radians(-30.))):
            x,ev,q=self.run_sidestep(p)
            self.assertLess(abs(q[1]),1e-6,(p,ev));self.assertAlmostEqual(ev['predicted_end_lateral_m'],q[1],places=6)
            self.assertGreater(abs(ev['turn_swing_m']),.1)          # a real swing is corrected (size depends on the pinned test pivot)
    def test_drive_is_capped_and_retry_happens_once(self):
        x,ev,q=self.run_sidestep((-7.,2.,math.radians(-25.)))
        self.assertAlmostEqual(x.sidestep_legs[0][1],1.5)            # capped
        x.sidestep_legs=[];x.action=None;x.state='PLAN';x.reason='sidestep_done'
        x.step(30.,observation(30.,world=(-7.,.6,0.)));self.assertEqual(x.action['after'],'SIDESTEP');self.assertEqual(x.sidesteps,2)   # retry
        x.sidestep_legs=[];x.action=None;x.state='PLAN';x.reason='sidestep_done'
        x.step(60.,observation(60.,world=(-7.,.3,0.)));self.assertEqual(x.state,'PLANNING')                                    # no third
    def test_no_retry_when_close_enough(self):
        x,ev,q=self.run_sidestep((-7.,-1.,0.))
        x.sidestep_legs=[];x.action=None;x.state='PLAN';x.reason='sidestep_done'
        x.step(30.,observation(30.,world=(-7.,-.1,0.)));self.assertEqual(x.state,'PLANNING')

class CameraPositionTests(unittest.TestCase):
    def test_rear_extent_from_vehicle_length_and_pivot_follows_the_camera(self):
        import json,tempfile
        from pathlib import Path
        from docking import config
        doc=json.loads(Path(config.COMMON).read_text());f=doc['fields']['site']['fields']
        f['vehicle_length_m']=dict(description='',value=2.17);f['rear_extent_m']=dict(description='',value=None)
        f['camera_to_fork_tip_m']=dict(description='',value=.5);f['camera_lateral_m']=dict(description='',value=-.45)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'common.json';p.write_text(json.dumps(doc));c=config.load(p,mode='left')
        self.assertAlmostEqual(c['site']['rear_extent_m'],1.67)
        self.assertAlmostEqual(c['pivot'][0],.5-c['site']['rotation_center_behind_fork_tip_m'])
        self.assertAlmostEqual(c['pivot'][1],c['site']['rotation_center_lateral_m'])   # centreline-based: unchanged by the camera's side
        self.assertAlmostEqual(c['sensor']['calibration']['T_body_camera'][1][3],-.45)

if __name__=='__main__':unittest.main()
