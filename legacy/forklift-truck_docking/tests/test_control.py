import math
import unittest
from unittest.mock import patch
import numpy as np
from docking.controller import Controller
from docking.planner import plan
from docking.geometry import front,remaining,arrival_y,inside_bay,cut_position,moved
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
    def test_bay_applies_only_inside_entrance(self):
        c=config('left')
        self.assertTrue(inside_bay([-5,2,1.],c))
        self.assertTrue(inside_bay([-.5,0,0],c))
        self.assertFalse(inside_bay([-.5,.2,0],c))
    def test_cut_uses_actual_height_and_left_tag_offset(self):
        a=config();b=config('left');self.assertLess(cut_position(a),-2.)
        b['sensor']['calibration']['T_dock_tag'][1][3]=2.
        self.assertLess(cut_position(b),cut_position(a))
    def test_plan_farther_than_4point8m_makes_partial_progress(self):
        c=config();p=[-10.,.2,math.radians(4)];cut=cut_position(c)
        result=plan(c,p,cut)
        self.assertTrue(result['actions'],result)
        self.assertLessEqual(len(result['actions']),6)
        self.assertLessEqual(result['expanded'],48)
        self.assertFalse(result['complete'])
        self.assertTrue(any(a['distance']>0 for a in result['actions']))
        self.assertTrue(all(not a['fine'] for a in result['actions']))
    def test_final_correction_uses_fine_and_no_independent_yaw_gate(self):
        c=config('left');cut=cut_position(c);yaw=math.radians(.5)
        p=[cut,-(1-cut)*math.tan(yaw),yaw]
        result=plan(c,p,cut)
        self.assertTrue(result['complete'],result)
        self.assertEqual(result['actions'],[])
    def test_heading_and_lateral_are_reduced_on_approach(self):
        c=config();p=np.array([-7.,.3,math.radians(8)]);cut=cut_position(c)
        result=plan(c,p,cut);self.assertTrue(result['actions'],result)
        q=p.copy()
        for a in result['actions']:q=moved(q,a['turn'],a['distance'],c['pivot'])
        self.assertLess(abs(q[1])+abs(q[2]),abs(p[1])+abs(p[2]))

class Control(unittest.TestCase):
    def setUp(self):
        self.c=config();self.c['tuning']['preparation_s']=1.
        self.ctrl=Controller(self.c,-2.5,0.)
    def started(self):
        x=self.ctrl;x.step(0.,observation(0.),True)
        for i in range(1,12):x.step(i/10,observation(i/10))
        self.assertEqual(x.state,'WAIT_ENTER');x.step(1.2,observation(1.2),True)
        return x
    def test_enter_before_ready_is_not_latched(self):
        self.ctrl.step(0.,observation(0),True)
        for i in range(1,12):self.ctrl.step(i/10,observation(i/10))
        self.assertEqual(self.ctrl.state,'WAIT_ENTER');self.assertIsNone(self.ctrl.mission_start)
    def test_waiting_motion_restarts_preparation(self):
        x=self.started();x.state='WAIT_ENTER'
        x.step(2,observation(2,stationary=False),True)
        self.assertEqual(x.state,'PREPARING');self.assertEqual(x.command,'stop')
    def test_coasting_distance_retained_and_actual_pose_replans(self):
        x=self.started();x.begin_drive(2,observation(2),.8,False)   # 0.8 m leg: a cruising leg the learner accepts (2026-10-20)
        self.assertEqual(x.action['target'],.8)
        x.step(5.5,observation(5.5,world=(-4.2,0,0),local=(.8,0,0),stationary=False))   # 3.5 s of motion: a cruising leg (the learner ignores shorter ones)
        self.assertEqual(x.state,'SETTLE');self.assertEqual(x.command,'stop')
        # Command was 0.8 m; observed displacement is 1.6 m including unexpected coast.
        x.step(8,observation(8,world=(-3.4,0,0),local=(1.6,0,0)))
        x.step(8.1,observation(8.1,world=(-3.4,0,0),local=(1.6,0,0)))
        self.assertEqual(x.request['pose'],[-3.4,0,0])
        self.assertGreater(x.responses.data['forward']['coast'],.4)
        ev=[e for e in x.status()['events'] if e['event']=='step_done'];self.assertEqual(len(ev),1)   # 2026-10-20 per-step record
        e=ev[0];self.assertEqual(e['command'],'forward');self.assertEqual(e['unit'],'m');self.assertAlmostEqual(e['coast'],.8,places=6)
        self.assertAlmostEqual(e['stop_progress'],.8,places=6);self.assertIsNotNone(e['stop_reserve']);self.assertTrue(e['vio_ok']);self.assertIn('learned',e)
    def test_stale_and_generation_changes_stop_without_integrating_commands(self):
        for fault in ['stale','generation','epoch','imu','unusable','nonfinite']:
            with self.subTest(fault=fault):
                x=Controller(self.c,-2.5,0);x.begin_drive(1,observation(1),.2,False)
                o=observation(1.1,stationary=False)
                if fault=='stale':o['arrival']=-2.   # 3.1 s old > motion_age_s 3.0 (2026-10-20)
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
        x.step(2,o);self.assertIsNone(x.action)  # Briefly wait after a recent tag.
        o['capture_id']=o['arrival']=4.;x.step(4,o);self.assertAlmostEqual(x.action['target'],math.radians(self.c['tuning']['search_step_deg']))
        o=observation(4.2,local=(0,0,.1),usable=False,valid=False,pose=None,stationary=False)
        x.step(4.2,o);self.assertEqual(x.state,'SETTLE');self.assertEqual(x.command,'stop')
    def test_search_exhausted_no_seventh_turn(self):
        x=self.started();x.sweep=2*math.pi
        x.step(2,observation(2,valid=False,usable=False,pose=None,tag=dict(reason='missing',detected=False)))
        self.assertEqual(x.state,'FAULT');self.assertIsNone(x.action)
    def test_planner_timeout_and_old_reply_do_not_move(self):
        x=self.started();x.step(2,observation(2));self.assertEqual(x.state,'PLANNING')
        x.accept_plan(dict(id=0,plan=dict(actions=[dict(turn=0,distance=.8,fine=False)])),2.1,observation(2.1))
        self.assertEqual(x.command,'stop')
        # 2026-10-20: a late reply re-requests (planner_retries times) before it is a fault
        for n in range(3):
            x.await_plan['at']=0.;x.step(20.+2*n,observation(20.+2*n));self.assertEqual(x.state,'PLAN');x.step(20.1+2*n,observation(20.1+2*n));self.assertEqual(x.state,'PLANNING')
        x.await_plan['at']=0.;x.step(40.,observation(40.));self.assertEqual(x.state,'FAULT')
    def test_plan_invalidated_by_actual_pose_change(self):
        x=self.started();x.step(2,observation(2))
        x.accept_plan(dict(id=x.plan_id,plan=dict(actions=[dict(turn=0,distance=.8,fine=False)])),2.1,observation(2.1,world=(-4,0,0)))
        self.assertEqual(x.state,'PLAN');self.assertEqual(x.command,'stop')
    def test_recovery_and_settle_timeout_differ(self):
        x=self.started();x.begin_drive(2,observation(2),.2,False);x.stop(3)
        x.step(24,observation(24,local_valid=False));self.assertEqual(x.state,'RECOVER')
        x.step(62,observation(62,local_valid=False));self.assertEqual(x.state,'RECOVER')
        x.step(64,observation(64,local_valid=False));self.assertEqual(x.state,'FAULT')
    def test_blind_starts_from_projected_transition_not_random_tag_loss(self):
        x=self.started();x.step(2,observation(2,world=(-4,0,0),tag=dict(reason='missing',detected=False)))
        self.assertFalse(x.blind);self.assertEqual(x.state,'PLANNING')
        x.state='PLAN';x.step(3,observation(3,world=(-2.5,0,0)))
        self.assertTrue(x.blind);self.assertEqual(x.command,'forward')
    def test_target_stop_central_and_left(self):
        for mode,body_x in [('central',-2.),('left',-.5)]:
            x=Controller(config(mode),-2.5,0.);x.state='PLAN';x.mission_start=0.
            x.step(1,observation(1,world=(body_x,0,0)))
            self.assertEqual(x.state,'COMPLETE');self.assertEqual(x.command,'stop')
    def test_tag_cut_transition_accounts_for_observed_yaw(self):
        c=config('left');x=Controller(c,cut_position(c),0.);x.state='PLAN';x.mission_start=0.
        x.step(1,observation(1,world=(-2.335,.074,-.031),mode='left'))
        self.assertTrue(x.blind);self.assertEqual(x.command,'forward')
    def test_total_timer_excludes_preparation(self):
        x=self.started();x.mission_start=1000
        x.step(1500,observation(1500));self.assertNotEqual(x.reason,'mission_timeout')
        x.step(1601,observation(1601));self.assertEqual(x.state,'FAULT')

if __name__=='__main__':unittest.main()
