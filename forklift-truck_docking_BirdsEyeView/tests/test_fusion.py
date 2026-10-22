import math
import unittest
import numpy as np
from docking.config import TUNING
from docking.fusion import Fusion,Baseline,DEFAULT_BASELINE,body_in_dock,tag_bearing_range,pnp_dock_pose
from docking.geometry import matrix,wrap
from docking.uncertainty import inverse
from support import config


class GeometryTests(unittest.TestCase):
    def test_body_in_dock_round_trip(self):
        tag=np.array([0.,.9]);heading=math.radians(12.);body=np.array([-4.,.3])
        v=matrix([*body,heading])[:2,:2].T@(tag-body)
        p=body_in_dock(heading,math.atan2(v[1],v[0]),np.hypot(*v),tag)
        np.testing.assert_allclose(p,body,atol=1e-12)
    def test_bearing_range_and_pnp_pose_from_exact_view(self):
        s=config()['sensor'];cal=s['calibration'];truth=[-3.,.2,.1]
        Tct=inverse(cal['T_body_camera'])@inverse(matrix(truth))@np.array(cal['T_dock_tag'])
        np.testing.assert_allclose(pnp_dock_pose(Tct,cal['T_body_camera'],cal['T_dock_tag']),truth,atol=1e-9)
        bearing,rng=tag_bearing_range(Tct,cal['T_body_camera'])
        p=body_in_dock(truth[2],bearing,rng,[0.,0.]);np.testing.assert_allclose(p,truth[:2],atol=1e-9)


class FusionTests(unittest.TestCase):
    def setUp(self):
        self.s=config()['sensor'];cal=self.s['calibration']
        self.f=Fusion(dict(TUNING),cal['T_body_camera'],cal['T_dock_tag']);self.f.reset(1)
    def view(self,truth,psi_bias=0.):
        cal=self.s['calibration']
        Tct=inverse(cal['T_body_camera'])@inverse(matrix(truth))@np.array(cal['T_dock_tag'])
        bearing,rng=tag_bearing_range(Tct,cal['T_body_camera'])
        return bearing,rng,wrap(truth[2]+psi_bias)
    def test_window_needs_three_frames_then_anchors(self):
        truth=[-4.,.1,.02];local=[1.,2.,.5]
        for i in range(2):r=self.f.tag(i*.1,*self.view(truth),local,True)
        self.assertEqual(r['reason'],'tag_window_filling');self.assertFalse(self.f.ready)
        r=self.f.tag(.2,*self.view(truth),local,True);self.assertEqual(r['reason'],'first_anchor');self.assertTrue(self.f.ready)
        world,P,parts=self.f.estimate(local);np.testing.assert_allclose(world,truth,atol=1e-9)
        self.assertEqual(parts['baseline_source'],DEFAULT_BASELINE['source']);self.assertEqual(P.shape,(3,3))
    def test_heading_offset_follows_pnp_only_slowly_at_rest_and_not_while_moving(self):
        truth=[-4.,0.,0.];local=[0.,0.,0.]
        for i in range(4):self.f.tag(i*.1,*self.view(truth),local,True)
        offset0=self.f.offset
        for i in range(4,8):r=self.f.tag(i*.1,*self.view(truth,psi_bias=math.radians(2.)),local,False)
        self.assertEqual(r['heading_offset_source'],'held_while_moving');self.assertAlmostEqual(self.f.offset,offset0)
        sources=[self.f.tag(.8+i*.1,*self.view(truth,psi_bias=math.radians(2.)),local,True)['heading_offset_source'] for i in range(10)]
        self.assertEqual(sources.count('rest_update'),2)
        self.assertIn('rest_hold_until_next_window',sources)
        self.assertAlmostEqual(math.degrees(self.f.offset-offset0),2.*(1-(1-TUNING['heading_offset_gain'])**2),delta=.01)
    def test_vio_only_propagation_and_sigma_growth(self):
        truth=[-4.,0.,0.]
        for i in range(4):self.f.tag(i*.1,*self.view(truth),[0.,0.,0.],True)
        Q=np.diag([1e-4,1e-4,1e-6])
        for k in range(10):self.f.vio_step([.1,0.],Q,0.)
        world,P,parts=self.f.estimate([1.,0.,0.])
        np.testing.assert_allclose(world,[-3.,0.,0.],atol=1e-9)
        self.assertAlmostEqual(parts['travel_since_anchor_m'],1.)
        self.assertGreater(parts['vio_since_anchor'][0],math.sqrt(10*1e-4)*.99)
    def test_jump_bound_is_physical_not_statistical(self):
        truth=[-4.,0.,0.]
        for i in range(4):self.f.tag(i*.1,*self.view(truth),[0.,0.,0.],True)
        r=self.f.tag(.4,*self.view([-4.,.03,0.]),[0.,0.,0.],True)
        self.assertEqual(r['reason'],'refreshed');self.assertAlmostEqual(r['allowed_jump_m'],.04+.29*.1,places=6)
        reasons=[self.f.tag(.5+i*.1,*self.view([-4.,.5,0.]),[0.,0.,0.],True)['reason'] for i in range(14)]
        self.assertEqual(reasons[:3],['refreshed','refreshed','refreshed'])
        self.assertIn('tag_jump_skipped',reasons);self.assertIn('accepted_after_skip',reasons)
        self.assertLess(reasons.index('accepted_after_skip')*.1,1.3)
        world,_,_=self.f.estimate([0.,0.,0.]);self.assertAlmostEqual(world[1],.5,places=6)
        self.assertEqual(self.f.jump_allowed(8.,0.),.08);self.assertEqual(self.f.jump_allowed(1.,0.),.03)
    def test_skipped_window_means_vio_carries_on(self):
        truth=[-4.,0.,0.]
        for i in range(4):self.f.tag(i*.1,*self.view(truth),[0.,0.,0.],True)
        for i in range(4):self.f.tag(.4+i*.1,*self.view([-3.,0.,0.]),[0.,0.,0.],True)
        world,_,_=self.f.estimate([.2,0.,0.]);np.testing.assert_allclose(world,[-3.8,0.,0.],atol=1e-9)


class BaselineTests(unittest.TestCase):
    def rows(self,b,still=True,drift=.005,seconds=31.,gyro=.1):
        for t in np.arange(0,seconds,.15):
            b.add(float(t),4.,drift*math.sin(2*math.pi*t/20),math.radians(.3)*math.sin(2*math.pi*t/15),4.,still,gyro)
    def test_30s_still_baseline_has_no_pass_fail(self):
        b=Baseline(30.,.5);self.rows(b)
        self.assertIsNotNone(b.value);self.assertTrue(b.value['source'].startswith('still_30s'))
        self.assertGreater(b.value['lateral_m'],.002);self.assertLess(b.value['lateral_m'],.006)
        self.assertGreater(b.value['heading_rad'],math.radians(.1));self.assertAlmostEqual(b.value['distance_m'],4.)
    def test_halves_take_the_larger_drift(self):
        b=Baseline(30.,.5)
        for t in np.arange(0,31.,.15):b.add(float(t),4.,0. if t<15 else .01*math.sin(2*math.pi*t/5),0.,4.,True,.1)
        whole=np.std([0. if t<15 else .01*math.sin(2*math.pi*t/5) for t in np.arange(0,31,.5)])
        self.assertGreater(b.value['lateral_m'],whole)
    def test_never_still_falls_back_to_the_quietest_window_and_says_so(self):
        b=Baseline(30.,.5);self.rows(b,still=False,seconds=45.)
        self.assertIsNone(b.value);v=b.freeze()
        self.assertTrue(v['source'].startswith('quietest_30s'));self.assertGreater(v['blocks'],40)
    def test_no_tag_at_all_uses_the_declared_default(self):
        b=Baseline(30.,.5);v=b.freeze();self.assertEqual(v,DEFAULT_BASELINE)
        self.assertEqual(v['source'],'default_20261002_calibrate_not_measured_here')
        np.testing.assert_allclose(b.scaled(8.4),[.019*4,.019*2,math.radians(1.5)*2])
        np.testing.assert_allclose(b.scaled(1.0),[.01,.01,math.radians(.5)])
    def test_frozen_baseline_ignores_later_rows(self):
        b=Baseline(30.,.5);self.rows(b);v=b.freeze();self.rows(b,drift=.05,seconds=40.)
        self.assertEqual(b.value,v)


class PreparationTests(unittest.TestCase):
    def test_preparation_is_5s_before_enter_and_baseline_is_30s(self):
        from docking.controller import Controller
        from support import observation
        c=config();self.assertEqual(c['tuning']['preparation_s'],5.);self.assertEqual(__import__('docking.config',fromlist=['TUNING']).TUNING['baseline_s'],30.)
        x=Controller(c,-2.5,0.)
        for i in range(55):x.step(i*.1,observation(i*.1,sigma_baseline=dict(lateral_m=.01,source='still_30s'),baseline_ready=True))
        self.assertEqual(x.state,'WAIT_ENTER');self.assertEqual(x.preparation_report['sigma_baseline']['source'],'still_30s')

if __name__=='__main__':unittest.main()
