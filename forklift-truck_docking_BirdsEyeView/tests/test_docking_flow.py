import math,unittest
import numpy as np
from docking.controller import Controller
from docking.geometry import moved,cut_position
from docking.fine import final_check_distance
from support import config,observation,depth_from_world

class Sim:
    def __init__(self,ctrl,cfg,pose,depth=True,width=1.5,t=0.):
        self.x=ctrl;self.cfg=cfg;self.pose=np.array(pose,float);self.local=np.zeros(3);self.t=t;self.depth=depth;self.width=width
        self.rezero=0;self.extra={};self.freeze=False;self.rezero_works=True;self.depth_bias=0.;self.depth_fixed=None
    def move(self,turn,dist):
        if self.freeze:return
        self.pose=moved(self.pose,turn,dist,self.cfg['pivot']);self.local=moved(self.local,turn,dist,self.cfg['pivot'])
    def obs(self,stationary):
        o=observation(self.t,world=tuple(self.pose),local=tuple(self.local),stationary=stationary,**self.extra)
        o['heading_rezero']=dict(count=self.rezero,requested=self.x.rezero_request,info=None)
        if self.depth:
            o['depth']=depth_from_world(self.pose,stamp=self.t,width=self.width);o['depth']['lateral_body_m']+=self.depth_bias
            if self.depth_fixed is not None:o['depth']['lateral_body_m']=self.depth_fixed
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

def fresh_controller(mode='left',**tuning):
    c=config(mode);c['tuning'].update(preparation_s=1.,**tuning);cut=cut_position(c)
    x=Controller(c,cut,0.);x.state='PLAN';x.mission_start=0.;x.baseline_done=True
    return c,x

def events(x,name):return [e for e in x.events if e['event']==name]

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
