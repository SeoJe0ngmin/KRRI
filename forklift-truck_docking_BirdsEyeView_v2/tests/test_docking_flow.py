import math,unittest
import numpy as np
from docking.controller import Controller
from docking.geometry import moved,cut_position
from docking.fine import final_check_distance
from support import config,observation,depth_from_world,rails_from_world

class Sim:
    def __init__(self,ctrl,cfg,pose,depth=True,width=1.5,t=0.):
        self.x=ctrl;self.cfg=cfg;self.pose=np.array(pose,float);self.local=np.zeros(3);self.t=t;self.depth=depth;self.width=width
        self.rezero=0;self.extra={};self.freeze=False;self.rezero_works=True;self.depth_bias=0.;self.depth_fixed=None
        # rails (2026-10-27): reported when the site has rail_inner_width_m; centre/axis in the dock frame, front_x = rail front ends
        self.rail_width=cfg['site'].get('rail_inner_width_m');self.rail_centre=0.;self.rail_axis=0.;self.rail_front=float(cfg['site'].get('rail_front_offset_m') or 0.);self.rails_on=True
    def move(self,turn,dist):
        if self.freeze:return
        self.pose=moved(self.pose,turn,dist,self.cfg['pivot']);self.local=moved(self.local,turn,dist,self.cfg['pivot'])
    def obs(self,stationary):
        o=observation(self.t,world=tuple(self.pose),local=tuple(self.local),stationary=stationary,**self.extra)
        o['heading_rezero']=dict(count=self.rezero,requested=self.x.rezero_request,info=None)
        if self.depth:
            o['depth']=depth_from_world(self.pose,stamp=self.t,width=self.width);o['depth']['lateral_body_m']+=self.depth_bias
            if self.depth_fixed is not None:o['depth']['lateral_body_m']=self.depth_fixed
        if self.rail_width and self.rails_on:
            r=rails_from_world(self.pose,self.rail_width,self.rail_centre,self.rail_axis,self.rail_front,self.cfg['tuning']['rail_fit_max_m'],self.cfg['tuning']['rail_min_len_m'])
            if 'depth' not in o:o['depth']=dict(ok=False,reason='no_pair',stamp=self.t)
            if r is not None:o['depth']['rails']=r
        return o
    def run(self,seconds,enter=False,until=('COMPLETE','FAULT')):
        end=self.t+seconds
        while self.t<end-1e-9 and self.x.state not in until:
            self.t=round(self.t+.1,3);cmd=self.x.command;moving=cmd!='stop'
            if cmd=='forward':self.move(0.,.029)
            elif cmd=='backward':self.move(0.,-.029)
            elif cmd.startswith('rotate'):rate=math.radians(12. if 'slow' in cmd else 8.5)*.1;self.move(rate if 'left' in cmd else -rate,0.)
            if self.rezero_works and not moving and self.x.rezero_request>self.rezero:self.rezero=self.x.rezero_request
            self.x.step(self.t,self.obs(stationary=not moving),enter)
        return self.x

def fresh_controller(mode='left',rails=None,**tuning):
    c=config(mode);c['tuning'].update(preparation_s=1.,**tuning);cut=cut_position(c)
    if rails:c['site'].update(rail_inner_width_m=rails[0],rail_front_offset_m=rails[1])   # (inner width, front offset) -> rail tracking on
    x=Controller(c,cut,0.);x.state='PLAN';x.mission_start=0.;x.baseline_done=True
    return c,x

def events(x,name):return [e for e in x.events if e['event']==name]

def final_sim(c,x,lateral=0.,heading=0.,**kw):
    """A controller at the FINAL check point (the camera final_check_distance before the entrance)."""
    x.phase='FINAL';x.depth_seen=True;d=final_check_distance(c);return Sim(x,c,(-d,lateral,heading),**kw)

class RailEntryTests(unittest.TestCase):
    """Rail tracking (2026-10-27 v2): test site vehicle 1.3 wide, rails 1.32 apart (1 cm clearance), posts 1.5 apart."""
    def test_rails_off_keeps_the_one_leg_entry(self):
        c,x=fresh_controller('left');sim=final_sim(c,x);sim.run(120)
        self.assertEqual(x.state,'COMPLETE',x.reason);e=events(x,'entry');self.assertEqual(len(e),1);self.assertNotIn('segmented',e[0])
        self.assertFalse(events(x,'entry_leg'));self.assertIsNone(x.rail_centre_y)
    def test_segmented_entry_checks_the_rails_at_every_stop(self):
        c,x=fresh_controller('left',rails=(1.32,.3));sim=final_sim(c,x);sim.run(300)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]));self.assertEqual(x.final_report['ended_by'],'vio_progress')
        e=events(x,'entry')[0];self.assertTrue(e['segmented']);self.assertEqual(events(x,'final_check_ok')[0]['heading_source'],'rails')
        legs=events(x,'entry_leg');checks=events(x,'entry_rails')
        self.assertGreaterEqual(len(legs),3,legs);self.assertEqual(len(checks),len(legs)-1)
        for l in legs[:-1]:self.assertAlmostEqual(l['leg_m'],c['tuning']['entry_leg_m'],delta=1e-6)
        self.assertTrue(legs[-1]['last']);self.assertGreaterEqual(legs[-1]['leg_m'],c['tuning']['minimum_translation_m'])
        self.assertTrue(any(ch['inside'] for ch in checks));self.assertFalse(events(x,'entry_heading_turn'));self.assertFalse(events(x,'entry_back_out'))
        self.assertLess(abs(x.final_report['remaining_m']),.15);self.assertLess(abs(x.final_report['lateral_vs_rail_centre_m']),.02)
        self.assertEqual(x.final_report['rails']['source'],'frame');self.assertEqual(x.final_report['entry_legs'],len(legs))
    def test_final_uses_the_rail_centreline_not_the_posts(self):
        # the posts (depth pair) say centred, the rails sit 4 cm to the left: FINAL must correct towards the rails
        c,x=fresh_controller('left',rails=(1.32,.3));sim=final_sim(c,x);sim.rail_centre=.04;sim.run(400)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]));self.assertEqual(x.final_report['ended_by'],'vio_progress')
        names=[e['event'] for e in x.events];self.assertIn('final_back_out',names,names);self.assertIn('bundle',names)
        self.assertEqual(events(x,'bundle')[0]['lateral_source'],'rails');self.assertAlmostEqual(events(x,'bundle')[0]['lateral_error_m'],-.04,delta=.01)
        self.assertAlmostEqual(x.final_report['fork_lateral_m'],.04,delta=c['tuning']['rail_lateral_tol_m']+.01)
        self.assertLess(abs(x.final_report['lateral_vs_rail_centre_m']),c['tuning']['rail_lateral_tol_m']+.01)
    def test_rail_direction_becomes_the_heading_reference(self):
        # rails tilted 1.5 deg left of the dock axis, VIO heading 0: FINAL turns onto the rails, enters along them
        c,x=fresh_controller('left',rails=(1.32,.3));sim=final_sim(c,x);sim.rail_axis=math.radians(1.5);sim.run(300)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]))
        turn=events(x,'final_heading_turn');self.assertTrue(turn);self.assertEqual(turn[0]['heading_source'],'rails');self.assertAlmostEqual(turn[0]['heading_err_deg'],-1.5,delta=.2)
        self.assertAlmostEqual(math.degrees(x.rail_axis),1.5,delta=.2);self.assertAlmostEqual(math.degrees(x.heading_target()),1.5,delta=.2)
        self.assertAlmostEqual(x.final_report['heading_deg'],1.5,delta=c['tuning']['rail_heading_tol_deg']+.5)
        self.assertLess(abs(x.final_report['heading_vs_rails_deg']),c['tuning']['rail_heading_tol_deg']+.5)
    def test_heading_is_turned_out_before_the_rail_front(self):
        # rails not in view at the check point (front 1.2 m inside), tilted 1.2 deg through the check point: seen after the first
        # leg, turned before the tip reaches them; the residual below the turn resolution over the run-up is excused, no back-out
        c,x=fresh_controller('left',rails=(1.32,1.2));sim=final_sim(c,x);sim.rail_axis=math.radians(1.2);sim.rail_centre=math.tan(sim.rail_axis)*final_check_distance(c);sim.run(300)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]));self.assertNotEqual(events(x,'final_check_ok')[0]['heading_source'],'rails')
        t=events(x,'entry_heading_turn');self.assertTrue(t,[e['event'] for e in x.events]);self.assertAlmostEqual(t[0]['heading_err_deg'],-1.2,delta=.2)
        before=[ch for ch in events(x,'entry_rails') if not ch['inside']];self.assertTrue(before);self.assertGreater(before[-1]['excused_drift_m'],0.)
        self.assertFalse(events(x,'entry_back_out'),events(x,'entry_back_out'));self.assertLess(abs(x.final_report['heading_vs_rails_deg']),c['tuning']['rail_heading_tol_deg']+.5)
    def test_lateral_miss_before_the_rails_backs_out_and_corrects(self):
        # rails unseen at the check point, 4 cm left of the posts' centre: first sighting during the entry -> back out -> bundle on the rail centreline -> enter
        c,x=fresh_controller('left',rails=(1.32,1.2));sim=final_sim(c,x);sim.rail_centre=.04;sim.run(600)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]));self.assertEqual(x.final_report['ended_by'],'vio_progress')
        b=events(x,'entry_back_out');self.assertEqual(len(b),1,b);self.assertAlmostEqual(b[0]['predicted_front_lateral_m'],-.04,delta=.01)
        self.assertEqual(len(events(x,'entry')),2);self.assertEqual(events(x,'final_check_ok')[-1]['lateral_source'],'rails_vio')
        self.assertIn('bundle',[e['event'] for e in x.events]);self.assertLess(abs(x.final_report['lateral_vs_rail_centre_m']),c['tuning']['rail_lateral_tol_m']+.01)
        self.assertEqual(x.entry_corrections,1)
    def test_entry_retries_exhausted_ends_without_entry(self):
        # the rails keep reporting 5 cm off whatever the pose does (a fixed reading): back-outs stop after entry_retries_max
        c,x=fresh_controller('left',rails=(1.32,1.2),entry_retries_max=2);sim=final_sim(c,x)
        orig=sim.obs
        def obs(stationary):
            o=orig(stationary)
            if 'rails' in o.get('depth',{}):o['depth']['rails']['lateral_error_m']=.05
            return o
        sim.obs=obs;sim.run(900)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]));self.assertEqual(x.reason,'entry_retries_exhausted_recorded_no_entry')
        self.assertEqual(x.final_report['ended_by'],'entry_retries_exhausted_no_entry');self.assertTrue(events(x,'entry_retries_exhausted'))
        self.assertEqual(len(events(x,'entry_back_out')),2);self.assertFalse(x.blind)
    def test_predicted_rail_contact_inside_stops_the_vehicle(self):
        c,x=fresh_controller('left',rails=(1.32,.3));sim=final_sim(c,x)
        while x.state not in ('COMPLETE','FAULT') and not any(ch['inside'] for ch in events(x,'entry_rails')):sim.run(.1)
        self.assertNotIn(x.state,('COMPLETE','FAULT'),x.reason)
        sim.rail_centre=-.05;sim.run(60)   # the rails now read the body 5 cm left of their centre, between the rails: no correction possible
        self.assertEqual(x.state,'FAULT');self.assertEqual(x.reason,'rail_contact_predicted');self.assertEqual(x.command,'stop')
        self.assertTrue(events(x,'rail_contact_predicted'))
    def test_rails_never_seen_enters_on_the_vio(self):
        c,x=fresh_controller('left',rails=(1.32,.3));sim=final_sim(c,x);sim.rails_on=False;sim.run(300)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]));self.assertEqual(x.final_report['ended_by'],'vio_progress')
        self.assertTrue(events(x,'entry_rails_unseen'));self.assertFalse(events(x,'entry_rails'));self.assertEqual(events(x,'final_check_ok')[0]['lateral_source'],'depth')
        self.assertIsNone(x.final_report['rails']);self.assertGreaterEqual(len(events(x,'entry_leg')),3)
    def test_vio_lost_during_a_leg_finishes_the_rest_on_time(self):
        c,x=fresh_controller('left',rails=(1.32,.3));sim=final_sim(c,x)
        while x.state not in ('COMPLETE','FAULT') and not (x.state=='MOVING' and len(events(x,'entry_leg'))==2 and x.action.get('progress',0)>.05):sim.run(.1)
        self.assertEqual(x.phase,'ENTRY');sim.extra=dict(local_valid=False);sim.run(120)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]));self.assertEqual(x.final_report['ended_by'],'timed_fallback')
        self.assertTrue(events(x,'entry_vio_lost_rest_timed'));self.assertEqual(len(events(x,'entry_leg')),2)
    def test_vio_lost_at_an_entry_stop_finishes_the_rest_on_time(self):
        c,x=fresh_controller('left',rails=(1.32,.3),entry_recover_s=2.);sim=final_sim(c,x)
        while x.state not in ('COMPLETE','FAULT') and not (x.state=='PLAN' and x.phase=='ENTRY' and len(events(x,'entry_leg'))==2):sim.run(.1)
        self.assertEqual(x.phase,'ENTRY');sim.extra=dict(local_valid=False);sim.run(120)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]));self.assertEqual(x.final_report['ended_by'],'timed_fallback')
        e=events(x,'entry_vio_lost_at_rest_timed');self.assertTrue(e);self.assertGreaterEqual(e[0]['waited_s'],2.)
        self.assertLess(x.final_report['remaining_m'],.1)   # reached (the sim has no start-up delay, so the time model overshoots by startup x speed ~0.4 m)

class FullFlowTests(unittest.TestCase):
    def test_off_centre_oblique_start_reaches_the_target(self):
        c,x=fresh_controller('left');sim=Sim(x,c,(-7.,-.6,math.radians(17.)));sim.run(600)
        self.assertEqual(x.state,'COMPLETE',(x.reason,x.events[-3:]))
        names=[e['event'] for e in x.events]
        for must in ('sidestep','rough_done','heading_rezero_applied','final_check_ok','entry'):self.assertIn(must,names,names)
        self.assertEqual(x.final_report['ended_by'],'vio_progress')
        self.assertLess(abs(x.final_report['fork_lateral_m']),c['tuning']['final_lateral_tol_m']+.01)
        self.assertLess(abs(x.final_report['heading_deg']),c['tuning']['final_heading_tol_deg']+.5)
        self.assertLess(abs(x.final_report['remaining_m']),.15)
        rough=[e for e in x.events if e['event']=='legs' and e['label']=='rough_drive'];self.assertEqual(len(rough),1)
        self.assertAlmostEqual(events(x,'rough_done')[0]['pose'][0],x.cut-c['tuning']['rough_stop_before_cut_m'],delta=.25)
    def test_centred_start_needs_no_sidestep_and_no_bundle(self):
        c,x=fresh_controller('left');sim=Sim(x,c,(-6.,0.,0.));sim.run(400)
        self.assertEqual(x.state,'COMPLETE',x.reason);names=[e['event'] for e in x.events]
        self.assertNotIn('sidestep',names);self.assertNotIn('bundle',names);self.assertIn('fine_straight',names);self.assertIn('final_check_ok',names)
    def test_bundle_turns_toward_the_centre_and_lands_on_it(self):
        c,x=fresh_controller('left');x.phase='FINE';x.depth_seen=True
        start=(x.cut-1.,.12,0.);sim=Sim(x,c,start);sim.run(60,until=('COMPLETE','FAULT'))
        b=events(x,'bundle');self.assertTrue(b,x.events[:4]);self.assertLess(b[0]['alpha_deg'],0.)
        self.assertEqual(b[0]['source'],'depth');self.assertAlmostEqual(b[0]['lateral_error_m'],.12,delta=.01)
        self.assertEqual(x.state,'COMPLETE',x.reason);self.assertLess(abs(x.final_report['fork_lateral_m']),.03)
    def test_final_check_backs_out_then_corrects(self):
        c,x=fresh_controller('left');x.phase='FINAL';x.depth_seen=True
        d=final_check_distance(c);sim=Sim(x,c,(-d,.08,0.));sim.run(200)
        names=[e['event'] for e in x.events]
        self.assertIn('final_back_out',names,names);self.assertIn('bundle',names);self.assertIn('final_check_ok',names)
        self.assertEqual(x.state,'COMPLETE',x.reason);self.assertLess(abs(x.final_report['fork_lateral_m']),.03)
        back=[e for e in x.events if e['event']=='legs' and e['label']=='final_back'][0]['legs'][0]['value']
        self.assertLess(back,0.);self.assertLessEqual(-back,c['tuning']['backward_max_m']+1e-9)
    def test_final_heading_error_is_turned_out(self):
        c,x=fresh_controller('left');x.phase='FINAL';x.depth_seen=True
        d=final_check_distance(c);sim=Sim(x,c,(-d,0.,math.radians(3.)));sim.run(120)
        turned=[e for e in x.events if e['event']=='legs' and e['label'] in ('straighten_fine','final_heading')]
        self.assertTrue(turned,x.events[:3]);self.assertEqual(x.state,'COMPLETE',x.reason)
        self.assertLess(abs(x.final_report['heading_deg']),c['tuning']['final_heading_tol_deg']+.5)
    def test_no_room_to_correct_ends_without_entry(self):
        c,x=fresh_controller('left');x.phase='FINAL';x.depth_seen=True;c['tuning']['backward_max_m']=0.
        d=final_check_distance(c);sim=Sim(x,c,(-d,.08,0.));sim.run(30)
        self.assertEqual(x.state,'COMPLETE');self.assertEqual(x.reason,'final_no_room_recorded_no_entry');self.assertFalse(x.blind)
        self.assertEqual(x.final_report['ended_by'],'no_room_to_correct_no_entry')
    def test_corrections_exhausted_ends_without_entry(self):
        c,x=fresh_controller('left',fine_corrections_max=1);x.phase='FINAL';x.depth_seen=True
        d=final_check_distance(c);sim=Sim(x,c,(-d,0.,0.));sim.depth_fixed=.08
        sim.run(300)
        self.assertEqual(x.state,'COMPLETE');self.assertEqual(x.reason,'final_corrections_exhausted_recorded_no_entry');self.assertFalse(x.blind)
        self.assertEqual(x.final_report['ended_by'],'corrections_exhausted_no_entry');self.assertEqual(x.corrections,2)
    def test_depth_never_seen_uses_the_tag_then_the_fallback(self):
        c,x=fresh_controller('left',depth_wait_s=.5);x.phase='FINE'
        sim=Sim(x,c,(x.cut-1.,.1,0.),depth=False);sim.run(40)
        b=events(x,'bundle');self.assertTrue(b);self.assertEqual(b[0]['source'],'tag')
        c,x=fresh_controller('left',depth_wait_s=.5);x.phase='FINE'
        sim=Sim(x,c,(x.cut-1.,.1,0.),depth=False);sim.extra=dict(blind_age_s=5.);sim.run(300)
        self.assertIn('depth_unavailable_fallback',[e['event'] for e in x.events]);self.assertEqual(x.state,'COMPLETE',x.reason)
        self.assertEqual([e for e in x.events if e['event']=='entry'][0]['source'],'fallback_tag_pose')
    def test_depth_lost_after_seen_is_carried_by_the_vio(self):
        c,x=fresh_controller('left',depth_wait_s=1.);x.phase='FINE'
        sim=Sim(x,c,(x.cut-1.,.1,0.));sim.run(3)
        self.assertTrue(x.depth_seen);sim.depth=False;sim.run(200)
        srcs={e['source'] for e in x.events if e['event'] in ('bundle','fine_straight','final_check_ok')}
        self.assertIn('vio_propagated',srcs);self.assertEqual(x.state,'COMPLETE',x.reason)
    def test_no_progress_stops_a_rough_leg_and_repeats_are_a_fault(self):
        c,x=fresh_controller('left',no_progress_stops_max=1);sim=Sim(x,c,(-6.,0.,0.))
        sim.run(1);self.assertEqual(x.action['label'],'rough_drive')
        sim.run(1);sim.freeze=True;sim.run(5)
        self.assertTrue(events(x,'no_progress_stop'));sim.run(60)
        self.assertEqual(x.state,'FAULT');self.assertIn(x.reason,('no_progress_repeated','motion_time_limit'))
    def test_rezero_times_out_and_continues(self):
        c,x=fresh_controller('left',heading_rezero_timeout_s=2.);sim=Sim(x,c,(x.cut-1.,0.,0.));sim.rezero_works=False
        sim.run(5)
        self.assertIn('heading_rezero_timeout',[e['event'] for e in x.events]);self.assertIn(x.phase,('FINE','FINAL','ENTRY'))
    def test_path_bias_aims_the_entry_against_the_drift(self):
        c,x=fresh_controller('left');c['site']['forward_path_bias_deg']=-2.
        self.assertAlmostEqual(math.degrees(x.heading_target()),2.)
        x.phase='FINAL';x.depth_seen=True;d=final_check_distance(c);sim=Sim(x,c,(-d,0.,0.));sim.run(60)
        turned=[e for e in x.events if e['event']=='legs' and e['label'] in ('straighten_fine','final_heading')];self.assertTrue(turned)
        self.assertAlmostEqual(events(x,'entry')[0]['heading_deg'],2.,delta=.6)

if __name__=='__main__':unittest.main()
