"""2026-10-20/22 KW field decisions: (1) straight steps stop on the VIO or the tag-anchored displacement, whichever is larger;
(2) sidestep whenever the lateral is at least one real step (v2: no 0.5 m threshold), at most twice, with the pivot swing of
both turns in the drive; (3) camera position inputs recompute the rotation centre."""
import math,unittest
from docking.controller import Controller
from support import config,observation

def started(c=None):
    c=c or config();c['tuning']['preparation_s']=1.;x=Controller(c,-2.5,0.)
    x.step(0.,observation(0.),True)
    for i in range(1,12):x.step(i/10,observation(i/10))
    x.step(1.2,observation(1.2),True);return x

def drive(x,t,d):
    o=observation(t);x.set_legs(t,o,[dict(kind='drive',value=d,label='test')],'test');x.next_leg(t,o);return x

class TagRulerTests(unittest.TestCase):
    def test_step_stops_on_tag_motion_when_vio_is_frozen(self):
        x=started();x.cut=-1.5;drive(x,2.,.8);self.assertEqual(x.state,'MOVING')
        t=2.;xw=-5.
        for k in range(40):
            t+=.1;xw+=.029;x.step(t,observation(t,world=(xw,0.,0.),local=(0.,0.,0.),stationary=False))
            if x.state!='MOVING':break
        self.assertEqual(x.state,'SETTLE');self.assertLess(xw+5.,1.0)
        parts=x.brake['progress_parts'];self.assertAlmostEqual(parts[0],0.,places=6);self.assertGreater(parts[1],.5)
    def test_vio_alone_still_stops_when_the_tag_is_lost(self):
        x=started();drive(x,2.,.5)
        t=2.;xl=0.
        for k in range(40):
            t+=.1;xl+=.029
            x.step(t,observation(t,local=(xl,0.,0.),stationary=False,usable=False,pose=None,tag=dict(ok=False,detected=False,reason='no_tag')))
            if x.state!='MOVING':break
        self.assertIn(x.state,('SETTLE','RECOVER'),x.status()['reason'])

class SidestepTests(unittest.TestCase):
    def test_far_off_centreline_start_sidesteps(self):
        x=started();x.state='PLAN';x.cut=-3.3
        p=(-7.,-1.,0.)                                  # 1 m right of the centreline, 3.7 m of room
        x.step(5.,observation(5.,world=p));self.assertEqual(x.state,'MOVING');self.assertEqual(x.command,'rotate_left_slow')   # face +90 deg, strength 30
        self.assertAlmostEqual(x.action['target'],math.pi/2,places=6);self.assertEqual(x.action['after'],'LEGS')
        self.assertEqual([l['kind'] for l in x.legs],['drive','turn']);self.assertAlmostEqual(x.legs[0]['value'],1.,places=6)   # heading 0: the swings cancel
        self.assertEqual(x.legs[1]['strength'],'slow');self.assertIn('sidestep',[e['event'] for e in x.events]);self.assertEqual(x.sidesteps,1)
    def test_any_lateral_of_one_step_or_more_sidesteps_smaller_does_not(self):
        x=started();x.state='PLAN';x.cut=-3.3;x.step(5.,observation(5.,world=(-7.,-.3,0.)))
        self.assertEqual(x.command,'rotate_left_slow');self.assertEqual(x.sidesteps,1)                  # 0.3 m: sidestep (v2 'always')
        x=started();x.state='PLAN';x.cut=-3.3;x.step(5.,observation(5.,world=(-7.,-.1,0.)))
        self.assertEqual(x.command,'forward');self.assertEqual(x.action['label'],'rough_drive');self.assertEqual(x.sidesteps,0)   # below one real step: left to the fine stage
    def test_no_sidestep_without_room_goes_to_the_fine_stage(self):
        x=started();x.state='PLAN';x.cut=-3.3;x.step(5.,observation(5.,world=(-3.8,-1.,0.)))   # 0.5 m before the cut: no room for the two turns
        self.assertEqual(x.sidesteps,0);self.assertEqual(x.phase,'FINE_ENTER');self.assertEqual(x.command,'stop');self.assertEqual(x.rezero_request,1)
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
        self.assertEqual(x.action['after'],'LEGS');self.assertEqual(x.action['label'],'sidestep_face')
        ev=[e for e in x.events if e['event']=='sidestep'][-1]
        q=moved(p,wrap((math.pi/2 if p[1]<0 else -math.pi/2)-p[2]),0,x.cfg['pivot'])
        q=moved(q,0,x.legs[0]['value'],x.cfg['pivot']);q=moved(q,wrap(0.-q[2]),0,x.cfg['pivot'])
        return x,ev,q
    def test_lands_on_the_centreline_from_oblique_starts(self):
        for p in ((-7.,-1.,math.radians(34.)),(-7.,.66,math.radians(-22.)),(-7.,-.8,math.radians(-30.))):
            x,ev,q=self.run_sidestep(p)
            self.assertLess(abs(q[1]),1e-6,(p,ev));self.assertAlmostEqual(ev['predicted_end_lateral_m'],q[1],places=6)
            self.assertGreater(abs(ev['turn_swing_m']),.1)
    def test_drive_is_capped_and_retry_happens_once(self):
        x,ev,q=self.run_sidestep((-7.,2.,math.radians(-25.)))
        self.assertAlmostEqual(x.legs[0]['value'],1.5)                 # capped
        x.legs=[];x.action=None;x.state='PLAN'
        x.step(30.,observation(30.,world=(-7.,.6,0.)));self.assertEqual(x.action['label'],'sidestep_face');self.assertEqual(x.sidesteps,2)   # retry
        x.legs=[];x.action=None;x.state='PLAN'
        x.step(60.,observation(60.,world=(-7.,.3,0.)));self.assertEqual(x.action['label'],'rough_drive')                              # no third: rough drive
    def test_no_retry_when_within_one_step(self):
        x,ev,q=self.run_sidestep((-7.,-1.,0.))
        x.legs=[];x.action=None;x.state='PLAN'
        x.step(30.,observation(30.,world=(-7.,-.1,0.)));self.assertEqual(x.action['label'],'rough_drive')

class CameraPositionTests(unittest.TestCase):
    def test_rear_extent_from_vehicle_length_and_pivot_follows_the_camera(self):
        import json,tempfile
        from pathlib import Path
        from docking import config as cfgmod
        doc=json.loads(Path(cfgmod.COMMON).read_text());f=doc['fields']['site']['fields']
        f['vehicle_length_m']=dict(description='',value=2.17);f['rear_extent_m']=dict(description='',value=None)
        f['camera_to_fork_tip_m']=dict(description='',value=.5);f['camera_lateral_m']=dict(description='',value=-.45)
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'common.json';p.write_text(json.dumps(doc));c=cfgmod.load(p,mode='left')
        self.assertAlmostEqual(c['site']['rear_extent_m'],1.67)
        self.assertAlmostEqual(c['pivot'][0],.5-c['site']['rotation_center_behind_fork_tip_m'])
        self.assertAlmostEqual(c['pivot'][1],c['site']['rotation_center_lateral_m'])   # centreline-based: unchanged by the camera's side
        self.assertAlmostEqual(c['sensor']['calibration']['T_body_camera'][1][3],-.45)
    def test_truck_naming_and_depth_check(self):
        import json,tempfile
        from pathlib import Path
        from docking import config as cfgmod
        doc=json.loads(Path(cfgmod.COMMON).read_text());f=doc['fields']['site']['fields']
        self.assertIn('truck_inner_width_m',f);self.assertNotIn('bay_width_m',f);self.assertIn('truck_depth_m',f)
        f['truck_depth_m']=dict(description='',value=.8)   # shallower than the 1.0 m left-mode fork target
        with tempfile.TemporaryDirectory() as d:
            p=Path(d)/'common.json';p.write_text(json.dumps(doc))
            c=cfgmod.load(p,mode='left');self.assertEqual(c['site']['truck_depth_m'],.8)            # measure: loads
            with self.assertRaises(ValueError):cfgmod.load(p,mode='left',require_mount=True)        # drive: refused

if __name__=='__main__':unittest.main()
