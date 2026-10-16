import unittest
import math
from types import SimpleNamespace
import numpy as np
from docking.observer import Observer
from docking.geometry import matrix
from support import config

class ObserverTests(unittest.TestCase):
    def setUp(self):
        self.c=config()['sensor'];self.c['tag']['enabled']=False;self.c['odom']['enabled']=False
        self.o=Observer(self.c);self.k=0;self.reference=None
        self.Tbc=np.array(self.c['calibration']['T_body_camera']);self.Tdt=np.array(self.c['calibration']['T_dock_tag'])
        self.delta=[0.,0.,0.];self.world=[-5.,0.,0.];self.ok=True;self.gen=0;self.tag=True;self.yaw=0.
        def odom(*args):
            value=dict(ok=self.ok,reason='tracked' if self.ok else 'too_few_features',generation=self.gen,
                reference_stamp=self.reference,T_previous_current=np.linalg.inv(self.Tbc) @ matrix(self.delta) @ self.Tbc)
            self.reference=args[-1];return value
        def tag(*args):
            return dict(ok=True,reason='accepted',detected=True,corners=None,
                T_camera_tag=np.linalg.inv(self.Tbc) @ np.linalg.inv(matrix(self.world)) @ self.Tdt) if self.tag else dict(ok=False,reason='missing',corners=None)
        self.o.odom=SimpleNamespace(update=odom);self.o.tag=SimpleNamespace(update=tag)
    def feed(self,n=1):
        for _ in range(n):
            self.k+=1;stamp=self.k/10.;self.yaw+=self.delta[2]
            self.result=self.o.process(np.zeros((4,4),np.uint8),None,np.eye(3),np.zeros(5),stamp,
                capture_imu=dict(device_stamp_s=stamp,yaw_rad=self.yaw,gaps=0,rate_rps=self.delta[2]*10))
        return self.result
    def test_real_still_window_and_no_certified_bound(self):
        r=self.feed(30);self.assertTrue(r['stationary']);self.assertTrue(r['usable'])
        np.testing.assert_allclose(r['pose'],[-5,0,0],atol=1e-10)
        self.assertFalse(r['bounded']);self.assertEqual(r['confidence_kind'],'heuristic_not_certified')
    def test_coast_and_rebound_are_measured_even_when_neutral(self):
        self.feed(30);self.tag=False
        self.delta=[.1,0,0];r=self.feed(4);self.assertAlmostEqual(r['pose'][0],-4.6)
        self.delta=[-.025,0,0];r=self.feed(2);self.assertAlmostEqual(r['pose'][0],-4.65)
        self.assertFalse(r['stationary'])
    def test_consensus_compares_moving_observations_in_same_frame(self):
        self.o.neutral=False
        for i in range(25):
            self.delta=[.03,0,0];self.world=[-5+(i+1)*.03,0,0];r=self.feed()
        self.assertTrue(r['usable']);np.testing.assert_allclose(r['pose'],self.world,atol=1e-10)
    def test_single_bad_tag_does_not_teleport_pose(self):
        self.feed(30);self.world=[-2,1,.3];r=self.feed(1)
        np.testing.assert_allclose(r['pose'],[-5,0,0],atol=1e-10)
    def test_sustained_conflict_cannot_reanchor_while_moving(self):
        self.feed(30);self.o.neutral=False;self.world=[-2,1,.3];r=self.feed(35)
        self.assertFalse(r['usable']);np.testing.assert_allclose(r['pose'],[-5,0,0],atol=1e-10)
        self.o.neutral=True;r=self.feed(35)
        self.assertTrue(r['usable']);self.assertGreater(r['motion_epoch'],1)
        np.testing.assert_allclose(r['pose'],self.world,atol=1e-10)
    def test_feature_loss_invalidates_absolute_pose_until_tag_reacquired(self):
        self.feed(30);self.ok=False;self.gen+=1;self.tag=False;r=self.feed()
        self.assertIsNone(r['pose']);self.assertFalse(r['usable'])
        self.ok=True;r=self.feed(30);self.assertIsNone(r['pose'])
        self.tag=True;r=self.feed(20);self.assertTrue(r['usable'])
    def test_blind_age_is_not_reset_by_neutral_or_unqualified_tags(self):
        self.feed(30);self.tag=False;r=self.feed(650)
        self.assertFalse(r['usable']);self.assertGreater(r['blind_age_s'],60.)
    def test_video_clock_reversal_stops_processing(self):
        self.feed()
        with self.assertRaisesRegex(ValueError,'clock reversed'):
            self.o.process(np.zeros((4,4)),None,np.eye(3),np.zeros(5),.05)

if __name__=='__main__':unittest.main()
