import unittest
import math
import numpy as np
from docking.stationary import StationaryDetector
from docking.config import TUNING

class StationaryTest(unittest.TestCase):
    def detector(self):return StationaryDetector(TUNING,gyro_hz=200.)
    def feed(self,d,t0,t1,rate_dps=0.,bias=(0.,0.,0.),speed=0.,neutral=True,since_neutral=10.,vio=True,raw=True,norm=None):
        result=False;step=1/30
        for t in np.arange(t0,t1,step):
            if raw:
                for k in range(7):d.add_gyro(float(t+k/200),np.array(bias)+[0.,0.,math.radians(rate_dps)])
            stamp=float(t+6/200);p=[speed*stamp,0.,0.]
            result=d.update(stamp,p,neutral,since_neutral,vio,norm)
        return result
    def test_rest_is_stationary_from_raw_gyro(self):
        d=self.detector();self.assertTrue(self.feed(d,0,2))
        r=d.report;self.assertEqual(r['reason'],'stationary');self.assertEqual(r['gyro_source'],'raw_200hz')
        self.assertLess(r['gyro_p90_dps'],.01);self.assertEqual(r['translation_rule'],'vio_speed')
    def test_rotation_above_half_degree_per_second_is_not_rest(self):
        d=self.detector();self.assertFalse(self.feed(d,0,2,rate_dps=2.));self.assertEqual(d.report['reason'],'rotating')
        d=self.detector();self.assertTrue(self.feed(d,0,2,rate_dps=.3))
    def test_bias_from_openvins_is_removed(self):
        d=self.detector();bias=(0.,0.,math.radians(1.))
        self.assertFalse(self.feed(d,0,2,bias=bias));self.assertEqual(d.report['bias_source'],'raw_no_bias')
        d.set_bias(bias);self.assertTrue(self.feed(d,2,4,bias=bias));self.assertEqual(d.report['bias_source'],'openvins')
    def test_straight_coast_is_caught_by_vio_speed_not_gyro(self):
        d=self.detector();self.assertFalse(self.feed(d,0,2,speed=.1));self.assertEqual(d.report['reason'],'translating_or_waiting')
        self.assertAlmostEqual(d.report['vio_speed_mps'],.1,places=3)
        d=self.detector();self.assertTrue(self.feed(d,0,2,speed=.01))
    def test_without_vio_the_v4_time_rule_applies(self):
        d=self.detector();self.assertFalse(self.feed(d,0,2,vio=False,since_neutral=1.))
        self.assertEqual(d.report['translation_rule'],'time_since_neutral')
        self.assertTrue(self.feed(d,2,4,vio=False,since_neutral=2.5))
    def test_command_not_neutral_is_never_rest(self):
        d=self.detector();self.assertFalse(self.feed(d,0,2,neutral=False));self.assertEqual(d.report['reason'],'command_not_neutral')
    def test_packet_norm_fallback_when_no_raw_samples(self):
        d=self.detector();self.assertTrue(self.feed(d,0,2,raw=False,norm=0.))
        self.assertEqual(d.report['gyro_source'],'vio_packet_norm')
        self.assertFalse(self.feed(d,2,4,raw=False,norm=math.radians(3.)))
    def test_too_few_samples_is_not_rest(self):
        d=self.detector();self.assertFalse(d.update(1.,[0,0,0],True,10.,True))
        self.assertEqual(d.report['reason'],'no_gyro_samples')

if __name__=='__main__':unittest.main()
