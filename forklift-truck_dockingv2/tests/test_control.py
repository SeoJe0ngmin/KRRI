import math
import unittest
import numpy as np
from docking.controller import Controller
from docking.planner import plan
from docking.geometry import front,remaining,arrival_y,inside_truck,cut_position,moved
from support import config,observation

class GeometryPlanning(unittest.TestCase):
    def test_front_reference_central_and_left(self):
        for mode,x in [('central',-2.),('left',-.5)]:
            c=config(mode);self.assertAlmostEqual(remaining([x,0,0],c),0.)
        self.assertAlmostEqual(front([-2,0,0],config())[0],-.5)
        self.assertAlmostEqual(front([-.5,0,0],config('left'))[0],1.)
    def test_final_lateral_projects_heading_and_fork(self):
        c=config('left');p=[-3.,0.,math.radians(2)]
        self.assertGreater(abs(arrival_y(p,c)),.03)
        self.assertAlmostEqual(arrival_y([-3,-4*math.tan(p[2]),p[2]],c),0.)
    def test_truck_walls_apply_only_inside_entrance(self):
        c=config('left')
        self.assertTrue(inside_truck([-5,2,1.],c))
        self.assertTrue(inside_truck([-.5,0,0],c))
        self.assertFalse(inside_truck([-.5,.2,0],c))
    def test_cut_uses_actual_height_and_left_tag_offset(self):
        a=config();b=config('left');self.assertLess(cut_position(a),-2.)
        b['sensor']['calibration']['T_dock_tag'][1][3]=2.
        self.assertLess(cut_position(b),cut_position(a))
    def test_legacy_planner_still_runs_but_is_not_used(self):
        c=config();p=[-10.,.2,math.radians(4)];cut=cut_position(c)
        result=plan(c,p,cut);self.assertTrue(result['actions'],result);self.assertLessEqual(len(result['actions']),6)
        x=Controller(c,cut,0.);self.assertIsNone(x.request);x.accept_plan(dict(id=1,plan=result),1.,observation(1.));self.assertIsNone(x.request)

class Control(unittest.TestCase):
    def setUp(self):
        self.c=config();self.c['tuning']['preparation_s']=1.
        self.ctrl=Controller(self.c,-2.5,0.)
    def started(self):
        x=self.ctrl;x.step(0.,observation(0.),True)
        for i in range(1,12):x.step(i/10,observation(i/10))
        self.assertEqual(x.state,'WAIT_ENTER');x.step(1.2,observation(1.2),True)
        return x
    def drive(self,x,t,d,o=None):
        o=o or observation(t);x.set_legs(t,o,[dict(kind='drive',value=d,label='test')],'test');x.next_leg(t,o);return x
    def test_enter_before_ready_is_not_latched(self):
        self.ctrl.step(0.,observation(0),True)
        for i in range(1,12):self.ctrl.step(i/10,observation(i/10))
        self.assertEqual(self.ctrl.state,'WAIT_ENTER');self.assertIsNone(self.ctrl.mission_start)
    def test_waiting_motion_restarts_preparation(self):
        x=self.started();x.state='WAIT_ENTER'
        x.step(2,observation(2,stationary=False),True)
        self.assertEqual(x.state,'PREPARING');self.assertEqual(x.command,'stop')
    def test_coasting_distance_retained_and_step_recorded(self):
        x=self.started();x.cut=-1.5;self.drive(x,2,.8)
        self.assertEqual(x.action['target'],.8);self.assertEqual(x.state,'MOVING')
        x.step(5.5,observation(5.5,world=(-4.2,0,0),local=(.8,0,0),stationary=False))
        self.assertEqual(x.state,'SETTLE');self.assertEqual(x.command,'stop')
        x.step(8,observation(8,world=(-3.4,0,0),local=(1.6,0,0)))
        self.assertEqual(x.state,'PLAN');self.assertGreater(x.responses.data['forward']['coast'],.4)
        ev=[e for e in x.status()['events'] if e['event']=='step_done'];self.assertEqual(len(ev),1)
        e=ev[0];self.assertEqual(e['command'],'forward');self.assertEqual(e['unit'],'m');self.assertAlmostEqual(e['coast'],.8,places=6)
        self.assertAlmostEqual(e['stop_progress'],.8,places=6);self.assertIsNotNone(e['stop_reserve']);self.assertTrue(e['vio_ok']);self.assertIn('learned',e)
        x.step(8.1,observation(8.1,world=(-3.4,0,0),local=(1.6,0,0)))
        self.assertEqual(x.state,'MOVING');self.assertEqual(x.command,'forward');self.assertEqual(x.action['label'],'rough_drive')
    def test_stale_and_generation_changes_stop_without_integrating_commands(self):
        for fault in ['stale','generation','epoch','imu','unusable','nonfinite']:
            with self.subTest(fault=fault):
                x=Controller(self.c,-2.5,0);self.drive(x,1,.2)
                o=observation(1.1,stationary=False)
                if fault=='stale':o['arrival']=-2.
                if fault=='generation':o['tracking_generation']=1
                if fault=='epoch':o['motion_epoch']=1
                if fault=='imu':o['imu_bad']=True
                if fault=='unusable':o['usable']=False
                if fault=='nonfinite':o['local_pose'][0]=float('nan')
                x.step(1.1,o);self.assertEqual(x.command,'stop');self.assertEqual(x.state,'RECOVER')
    def test_wrong_motion_polarity_latches_fault(self):
        x=self.started();x.begin(2,observation(2),'rotate_left_slow',.1,'SETTLE')
        x.step(3,observation(3,local=(0,0,-.1),stationary=False))
        self.assertEqual(x.state,'FAULT');self.assertEqual(x.command,'stop')
    def test_sensor_loss_during_search_cannot_restart_360_budget(self):
        x=self.started();x.begin(2,observation(2),'rotate_left_slow',1.,'SEARCH')
        x.step(3,observation(3,local_valid=False))
        self.assertEqual(x.state,'FAULT');self.assertEqual(x.command,'stop')
    def test_search_is_bounded_and_detected_tag_stops_now(self):
        x=self.started();o=observation(2,valid=False,usable=False,pose=None,tag=dict(reason='missing_or_duplicate_id',detected=False))
        x.step(2,o);self.assertIsNone(x.action)
        o['capture_id']=o['arrival']=4.;x.step(4,o);self.assertAlmostEqual(x.action['target'],math.radians(self.c['tuning']['search_step_deg']))
        self.assertTrue(x.status()['search_active'])
        o=observation(4.2,local=(0,0,.1),usable=False,valid=False,pose=None,stationary=False)
        x.step(4.2,o);self.assertEqual(x.state,'SETTLE');self.assertEqual(x.command,'stop')
    def test_search_exhausted_no_seventh_turn(self):
        x=self.started();x.sweep=2*math.pi
        x.step(2,observation(2,valid=False,usable=False,pose=None,tag=dict(reason='missing',detected=False)))
        self.assertEqual(x.state,'FAULT');self.assertIsNone(x.action)
    def test_recovery_and_settle_timeout_differ(self):
        x=self.started();self.drive(x,2,.2);x.stop(3)
        x.step(24,observation(24,local_valid=False));self.assertEqual(x.state,'RECOVER')
        x.step(62,observation(62,local_valid=False));self.assertEqual(x.state,'RECOVER')
        x.step(64,observation(64,local_valid=False));self.assertEqual(x.state,'FAULT')
    def test_rough_phase_drives_to_the_stop_point_before_the_cut(self):
        x=self.started();x.step(2,observation(2,world=(-5,0,0)))
        self.assertEqual(x.phase,'ROUGH');self.assertEqual(x.state,'MOVING');self.assertEqual(x.command,'forward')
        self.assertEqual(x.action['label'],'rough_drive');self.assertAlmostEqual(x.action['stop_x'],-2.5-self.c['tuning']['rough_stop_before_cut_m'])
        self.assertAlmostEqual(x.action['target'],1.5,places=6)
        self.assertGreaterEqual(x.action['cap_s'],self.c['tuning']['translation_timeout_s'])
    def test_rough_phase_straightens_first(self):
        x=self.started();x.step(2,observation(2,world=(-5,0,math.radians(11.5))))
        self.assertEqual(x.state,'MOVING');self.assertEqual(x.command,'rotate_right_fine');self.assertAlmostEqual(x.action['target'],math.radians(11.5),places=6)
        x=Controller(self.c,-2.5,0.);x.state='PLAN';x.mission_start=0.;x.baseline_done=True
        x.step(1,observation(1,world=(-5,0,math.radians(25))));self.assertEqual(x.command,'rotate_right_slow')
    def test_rough_leg_stops_on_the_dock_ruler_at_the_stop_point(self):
        x=self.started();x.step(2,observation(2,world=(-5,0,0)));stop_x=x.action['stop_x']
        t=2.;xw=-5.;xl=0.
        for k in range(80):
            t+=.1;xw+=.029;xl+=.01
            x.step(t,observation(t,world=(xw,0,0),local=(xl,0,0),stationary=False))
            if x.state!='MOVING':break
        self.assertEqual(x.state,'SETTLE');self.assertLess(xw,stop_x+.05);self.assertGreater(xw,stop_x-.4)
    def test_target_stop_central_and_left(self):
        for mode,body_x in [('central',-2.),('left',-.5)]:
            x=Controller(config(mode),-2.5,0.);x.state='PLAN';x.mission_start=0.;x.baseline_done=True
            x.step(1,observation(1,world=(body_x,0,0)))
            self.assertEqual(x.state,'COMPLETE');self.assertEqual(x.command,'stop')
    def test_total_timer_excludes_preparation(self):
        x=self.started();x.mission_start=1000
        x.step(1500,observation(1500));self.assertNotEqual(x.reason,'mission_timeout')
        x.step(1601,observation(1601));self.assertEqual(x.state,'FAULT')

if __name__=='__main__':unittest.main()
