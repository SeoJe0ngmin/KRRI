import math,unittest
import numpy as np
from docking.depth_goal import measure_bev,bev_points,find_entrance_bev
from docking.bev import mount_rotation
from support import config,observation

INTR=dict(fx=380.,fy=380.,cx=320.,cy=240.);H=.46

def render(cones=(),walls=(),pitch_deg=0.,noise=0.,seed=0):
    R=mount_rotation(pitch_deg);t=np.array([0.,0.,H]);v,u=np.mgrid[0:480,0:640]
    d=np.stack([(u-INTR['cx'])/INTR['fx'],(v-INTR['cy'])/INTR['fy'],np.ones_like(u,float)],-1)@R.T
    with np.errstate(all='ignore'):
        down=d[...,2]<-1e-6;z=np.where(down,-H/np.where(down,d[...,2],1),np.inf)
        for (x,y,r,h) in cones:
            a=d[...,0]**2+d[...,1]**2;b=-2*(d[...,0]*x+d[...,1]*y);c=x*x+y*y-r*r;disc=b*b-4*a*c
            s=np.where(disc>=0,(-b-np.sqrt(np.maximum(disc,0)))/(2*a+1e-12),np.inf);zh=H+s*d[...,2]
            hit=(s>0)&(zh>=0)&(zh<=h);z=np.where(hit&(s<z),s,z)
        for (y0,x0,x1,h) in walls:
            ok=np.abs(d[...,1])>1e-6;s=np.where(ok,y0/np.where(ok,d[...,1],1),np.inf);xh=s*d[...,0];zh=H+s*d[...,2]
            hit=(s>0)&(xh>=x0)&(xh<=x1)&(zh>=0)&(zh<=h);z=np.where(hit&(s<z),s,z)
    z[~np.isfinite(z)]=0
    if noise:
        rng=np.random.default_rng(seed);z=z*(1+rng.normal(0,noise,z.shape));z[~np.isfinite(z)]=0
    T=np.eye(4);T[:3,:3]=R;T[:3,3]=t
    return z.astype(np.float32),T

class BevEntranceTests(unittest.TestCase):
    def test_two_cones(self):
        z,T=render(cones=[(3.3,.7,.15,.6),(3.3,-.7,.15,.6)],noise=.01)
        m=measure_bev(z,INTR,T,1.1,.2,stamp=1.)
        self.assertTrue(m['ok'],m);self.assertEqual(m['method'],'bev');self.assertFalse(m['walls'])
        self.assertAlmostEqual(m['x_body_m'],3.3,delta=.15);self.assertAlmostEqual(m['lateral_body_m'],0.,delta=.03);self.assertAlmostEqual(m['width_m'],1.1,delta=.08)
        self.assertIsNotNone(m['u_goal']);self.assertAlmostEqual(m['u_goal'],320.,delta=6.);self.assertEqual(m['obstacle_cells'],0)
    def test_offset_cones_give_the_lateral(self):
        z,T=render(cones=[(3.3,.9,.15,.6),(3.3,-.5,.15,.6)])
        m=measure_bev(z,INTR,T,1.1,.2,stamp=1.);self.assertTrue(m['ok'],m);self.assertAlmostEqual(m['lateral_body_m'],.2,delta=.03)
    def test_truck_side_panels_give_the_axis_heading(self):
        for ang in (0.,-5.,4.):
            a=math.radians(ang);walls=[]
            cones=[]
            for side in (.7,-.7):
                for k in range(40):
                    xt=3.3+k*.05;x=xt*math.cos(a)-side*math.sin(a);y=xt*math.sin(a)+side*math.cos(a);cones.append((x,y,.03,.8))
            z,T=render(cones=cones)
            m=measure_bev(z,INTR,T,1.4,.2,stamp=1.)
            self.assertTrue(m['ok'],(ang,m.get('reason'),m.get('blobs')));self.assertTrue(m['walls'],m)
            self.assertAlmostEqual(m['axis_heading_deg'],ang,delta=1.0,msg=m);self.assertAlmostEqual(m['width_m'],1.4,delta=.1)
    def test_band_fallback_when_clusters_are_missing(self):
        z,T=render(cones=[(3.3,.7,.15,.6),(3.3,-.7,.15,.6)]);z[:,:]=np.where(z>0,z,0)
        z[0:480,0:640]=z;z_low=z.copy();z_low[:200,:]=0
        m=measure_bev(z_low,INTR,T,1.1,.2,stamp=1.);self.assertTrue(m['ok'])
        m2=measure_bev(z,INTR,T,5.,.1,stamp=1.)
        self.assertFalse(m2['ok']);self.assertEqual(m2['method'],'band_fallback');self.assertIn('no_pair',m2['bev_reason'])
    def test_corridor_count_sees_an_obstacle(self):
        z,T=render(cones=[(3.3,.7,.15,.6),(3.3,-.7,.15,.6),(1.5,0.,.15,.6)])
        m=measure_bev(z,INTR,T,1.1,.2,stamp=1.);self.assertGreater(m['obstacle_cells'],3)

class FinalHeadingFromWallsTests(unittest.TestCase):
    def test_final_turn_uses_the_truck_axis_when_walls_are_seen(self):
        from docking.controller import Controller
        from docking.geometry import cut_position
        from docking.fine import final_check_distance
        from support import depth_from_world
        c=config('left');c['tuning']['preparation_s']=1.;x=Controller(c,cut_position(c),0.);x.state='PLAN';x.mission_start=0.;x.baseline_done=True
        x.phase='FINAL';x.depth_seen=True;d=final_check_distance(c)
        for k in range(10):
            t=1.+k*.1;o=observation(t,world=(-d,0.,0.));dep=depth_from_world((-d,0.,0.),stamp=t);dep.update(walls=True,axis_heading_deg=3.);o['depth']=dep;x.step(t,o)
        ev=[e for e in x.events if e['event']=='final_heading_turn'];self.assertTrue(ev,x.events[-3:]);self.assertEqual(ev[0]['heading_source'],'truck_walls')
        self.assertAlmostEqual(ev[0]['heading_err_deg'],-3.,places=6)
        self.assertEqual(x.command,'rotate_left_fine');self.assertAlmostEqual(x.action['target'],math.radians(3.),places=6)

if __name__=='__main__':unittest.main()
