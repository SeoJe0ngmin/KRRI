"""Closed-loop synthetic plant. Does not validate physical braking or accuracy."""
import math
import unittest
import numpy as np
from docking.controller import Controller
from docking.planner import plan
from docking.geometry import moved,cut_position,tag_margin,remaining,front
from support import config,observation

def simulate(mode,start):
    cfg=config(mode);cfg['tuning']['preparation_s']=1.
    ctrl=Controller(cfg,cut_position(cfg),0.)
    p=np.array(start,float);rate=0.;speed=0.;quiet=0.;last_tag=0.;timeline=[]
    # 20 Hz plant (real observations arrive at 15-30 fps). At 10 Hz the plant's smallest executable
    # turn was 1.6 deg (one extra 0.1 s step), larger than the 3 cm arrival tolerance at a 3.3 m lever,
    # so transition corrections oscillated. Response per 0.1 s is kept identical to the old plant.
    dt=.05;k_rate=1-(1-.4)**(dt/.1);k_speed=1-(1-.35)**(dt/.1)
    for k in range(int(600/dt)):
        now=k*dt;cmd=ctrl.command
        desired_rate=math.radians(8 if 'fine' in cmd else 12)*(1 if 'left' in cmd else -1) if 'rotate' in cmd else 0.
        desired_speed=-.27 if cmd=='backward' else .14 if cmd=='forward_slow' else .29 if cmd=='forward' else 0.
        # Both driving response and coast persist, never inferred from target.
        rate+=k_rate*(desired_rate-rate);speed+=k_speed*(desired_speed-speed)
        p=moved(p,rate*dt,speed*dt,cfg['pivot'])
        quiet=quiet+dt if abs(speed)<.001 and abs(rate)<.001 else 0.
        visible=tag_margin(p,cfg)>=15
        if visible:last_tag=now
        o=observation(now,mode=mode,world=p,local=p,stationary=quiet>=2,blind_age_s=now-last_tag,
                      tag=dict(reason='accepted' if visible else 'missing',detected=visible))
        if ctrl.request:
            item=ctrl.request;ctrl.request=None
            result=plan(cfg,item['pose'],item['cut'],item['responses'])
            ctrl.accept_plan(dict(id=item['id'],plan=result),now,o)
        before=ctrl.state;ctrl.step(now,o,enter=ctrl.state=='WAIT_ENTER')
        if ctrl.state!=before:timeline.append((round(now,2),ctrl.state,ctrl.reason,p.round(3).tolist()))
        if ctrl.state in ('FAULT','COMPLETE'):break
    return cfg,ctrl,p,timeline

class Missions(unittest.TestCase):
    def test_central_and_left_replan_from_actual_coasting_pose(self):
        for mode in ['central','left']:
            with self.subTest(mode=mode):
                c,x,p,history=simulate(mode,[-6.,.15,math.radians(3)])
                self.assertEqual(x.state,'COMPLETE',str(history[-15:]))
                self.assertLessEqual(abs(remaining(p,c)),.05)
                self.assertLessEqual(abs(front(p,c)[1]),.03)
                self.assertGreater(len(x.responses.data),0)
                self.assertEqual(x.command,'stop')

if __name__=='__main__':unittest.main()
