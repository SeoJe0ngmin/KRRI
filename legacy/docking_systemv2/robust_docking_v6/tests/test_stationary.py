import unittest
import math
import numpy as np
from legacy.docking_systemv2.robust_docking_v6.docking.stationary import StationaryDetector
from legacy.docking_systemv2.robust_docking_v6.docking.config import TUNING

class StationaryTest(unittest.TestCase):
    def detector(self):return StationaryDetector(TUNING,dict(still_gyro_norm_rps=math.radians(1)))
    def feed(self,d,start,end,fn,**kwargs):
        result=False
        for t in np.arange(start,end,1/60):
            p=fn(float(t));result=d.update(float(t),p,.025,True,True,**kwargs)
        return result
    def vibration(self,t):return [.006*math.sin(2*math.pi*12*t),0,math.radians(.6)*math.sin(2*math.pi*12*t)]
    def test_vibration_exceeding_old_range_is_accepted_with_uncertainty(self):
        d=self.detector();self.assertTrue(self.feed(d,0,7,self.vibration))
        self.assertGreater(d.pose_std[0],.004)
        self.assertGreater(d.pose_std[2],math.radians(.35))
    def test_constant_drift_not_learned_as_vibration(self):
        d=self.detector();self.assertFalse(self.feed(d,0,8,lambda t:[t*.003,0,0]))
        self.assertIsNone(d.baseline)
    def test_large_oscillation_not_learned(self):
        d=self.detector();self.assertFalse(self.feed(d,0,8,lambda t:[.1*math.sin(2*math.pi*12*t),0,0]))
        self.assertIsNone(d.baseline)
    def test_neutral_discards_old_samples_and_rebound_needs_full_window(self):
        d=self.detector();self.assertTrue(self.feed(d,0,7,self.vibration))
        baseline=d.baseline['std'].copy()
        d.update(7,[0,0,0],0,True,False)
        self.assertFalse(self.feed(d,7.02,8,lambda t:[.02*(t-7),0,0]))
        self.assertFalse(self.feed(d,8,9,lambda t:[.02*(9-t),0,0]))
        self.assertFalse(self.feed(d,9,10,lambda t:[0,0,0]))
        self.assertTrue(self.feed(d,10,12,lambda t:[0,0,0]))
        np.testing.assert_array_equal(d.baseline['std'],baseline)
    def test_visual_loss_gap_and_neutral_token_clear_confirmation(self):
        d=self.detector();self.assertTrue(self.feed(d,0,7,lambda t:[0,0,0]))
        self.assertFalse(d.update(7,[0,0,0],0,False,True))
        self.assertTrue(self.feed(d,7.02,10,lambda t:[0,0,0]))
        self.assertFalse(d.update(10,[0,0,0],0,True,True,neutral_token=10))
        self.assertFalse(d.update(12,[0,0,0],0,True,True,neutral_token=10))
    def test_zero_net_rebound_is_not_rest(self):
        d=self.detector();self.feed(d,0,7,lambda t:[0,0,0])
        self.assertFalse(self.feed(d,7,9.1,lambda t:[.015*math.sin(math.pi*(t-7)),0,0]))
    def test_sustained_rotation_and_excess_gyro_rejected(self):
        d=self.detector();self.feed(d,0,7,lambda t:[0,0,0])
        self.assertFalse(self.feed(d,7,10,lambda t:[0,0,.01*(t-7)]))
        for t in np.arange(10,13,1/60):result=d.update(t,[0,0,0],.2,True,True)
        self.assertFalse(result)
    def test_rest_at_measured_14_hz_and_drift_rejection(self):
        for speed, expected in ((0.,True),(.003,False)):
            d=self.detector()
            for t in np.arange(0,8,1/14):
                result=d.update(float(t),[speed*t,0,0],0.,True,True)
            self.assertEqual(result,expected,d.report)
    def test_sparse_visual_samples_still_rejected(self):
        d=self.detector()
        for t in np.arange(0,8,1/4):
            result=d.update(float(t),[0,0,0],0.,True,True)
        self.assertFalse(result)
