import unittest
import copy
import numpy as np
from legacy.docking_systemv2.robust_docking_v6.docking.mount_pulse import Pulse,estimate
import legacy.docking_systemv2.robust_docking_v6.tests.test_straight_mount as test_straight_mount
from legacy.docking_systemv2.robust_docking_v6.tests.support import config

class PulseTests(unittest.TestCase):
    def rows(self):
        base,_=test_straight_mount.StraightMountTest().rows()
        start=[];end=[];motion=[]
        for i in range(301):
            r=copy.deepcopy(base[0]);r.update(arrival=i*.1,capture_id=i*.1)
            r['tag']['capture_key']=str(i);start.append(r)
        for i in range(1,31):
            r=copy.deepcopy(base[40+int(i*40/30)]);r.update(arrival=30+i*.1,capture_id=30+i*.1)
            r['tag']['capture_key']='m'+str(i);motion.append(r)
        for i in range(1,41):
            r=copy.deepcopy(base[-1]);r.update(arrival=33+i*.1,capture_id=33+i*.1)
            r['tag']['capture_key']='e'+str(i);end.append(r)
        return start,motion,end
    def test_one_pulse_and_settle(self):
        a,m,b=self.rows();p=Pulse();p.tick(0,a[0],True)
        for r in a:p.tick(r['arrival'],r)
        self.assertEqual(p.command,'forward')
        for r in m[:-1]:p.tick(r['arrival'],r)
        p.tick(32.99,None);self.assertEqual(p.command,'forward')
        p.tick(33,None);self.assertEqual(p.command,'stop')
        for r in b:p.tick(r['arrival'],r)
        self.assertTrue(p.done)
        p.tick(40,None,True);self.assertEqual(p.command,'stop')
    def test_estimate_and_outlier_uncertainty(self):
        a,m,b=self.rows();a[100]['tag']['camera_translation_m'][0]+=.2
        r=estimate(a+m+b,a,b,config()['sensor'],{})
        self.assertTrue(r['usable'],r);self.assertAlmostEqual(r['yaw_left_deg'],7)
        m[0]['motion_epoch']=2
        self.assertFalse(estimate(a+m+b,a,b,config()['sensor'],{})['usable'])
    def test_tracking_loss_stops_no_retry(self):
        a,m,b=self.rows();p=Pulse();p.tick(0,a[0],True)
        for r in a:p.tick(r['arrival'],r)
        m[0]['local_valid']=False
        with self.assertRaises(RuntimeError):p.tick(30.1,m[0])
        self.assertEqual(p.command,'stop')
    def test_incomplete_and_stationary_only_rejected(self):
        a,m,b=self.rows()
        self.assertFalse(estimate(a,a,[],config()['sensor'],{})['usable'])
        self.assertFalse(estimate(a+a,a,a,config()['sensor'],{})['usable'])

    def test_can_pulse_deadline_and_watchdog_are_independent(self):
        from legacy.docking_systemv2.robust_docking_v6.docking.canbus import effective
        self.assertEqual(effective(1,3.1,False,3.,3.),(0,False))
        self.assertEqual(effective(1,2.8,False,3.,3.),(0,True))
        self.assertEqual(effective(1,3.1,True,3.,3.),(0,True))
        self.assertEqual(effective(1,3.1,False,2.99,3.),(1,False))
    def test_sensor_error_does_not_repeat_automatic_motion(self):
        from unittest.mock import patch
        from types import SimpleNamespace
        from legacy.docking_systemv2.robust_docking_v6.docking.runtime import run
        args=SimpleNamespace(command='measure',calibration_forward=True,output=None)
        with patch('docking.runtime._run_once',side_effect=RuntimeError('reason=sample_gap')) as once:
            with self.assertRaises(RuntimeError):run(args)
            self.assertEqual(once.call_count,1)
    def test_settle_tracking_loss_recovers_without_forward(self):
        a,m,b=self.rows();p=Pulse();p.phase='SETTLE';p.since=33;p.last=m[-1]
        bad=copy.deepcopy(b[0]);bad['local_valid']=False
        p.tick(33.1,bad)
        self.assertEqual(p.command,'stop');self.assertFalse(p.done)
        p.tick(40,None)
        self.assertIsNone(p.stable_since)
        for r in b:
            r['arrival']+=10;r['capture_id']+=10
            p.tick(r['arrival'],r)
            self.assertEqual(p.command,'stop')
        self.assertTrue(p.done)
    def test_settle_silence_resets_dwell_and_timeout_remains(self):
        a,m,b=self.rows();p=Pulse();p.phase='SETTLE';p.since=33;p.last=m[-1]
        for r in b[:20]:p.tick(r['arrival'],r)
        p.tick(36,None)
        self.assertIsNone(p.stable_since);self.assertEqual(p.end,[])
        with self.assertRaisesRegex(RuntimeError,'60초'):p.tick(93,None)
        self.assertEqual(p.command,'stop')
    def test_recovery_reset_allowed_but_moving_reset_rejected(self):
        a,m,b=self.rows();gap=[]
        for i in range(500):
            r=copy.deepcopy(m[-1]);r.update(arrival=33+i*.1,capture_id=33+i*.1,local_valid=False)
            gap.append(r)
        for r in b:r['arrival']+=50;r['capture_id']+=50
        report=estimate(a+m+gap+b,a,b,config()['sensor'],{},stop_capture=33)
        self.assertTrue(report['usable'],report)
        gap[10]['motion_epoch']=2
        self.assertTrue(estimate(a+m+gap+b,a,b,config()['sensor'],{},stop_capture=33)['usable'])
        m[10]['motion_epoch']=2
        self.assertFalse(estimate(a+m+gap+b,a,b,config()['sensor'],{},stop_capture=33)['usable'])

    def test_rest_reset_does_not_mix_local_headings(self):
        a,m,b=self.rows()
        for r in a[:100]:r['motion_epoch']=0;r['local_pose'][2]=1.
        for r in b:r['motion_epoch']=2;r['local_pose'][2]=-1.
        report=estimate(a+m+b,a,b,config()['sensor'],{},stop_capture=33)
        self.assertTrue(report['usable'],report)
        self.assertTrue(report['rest_only_motion_reset_tolerated'])
        b[0]['sensor_epoch']=2
        self.assertFalse(estimate(a+m+b,a,b,config()['sensor'],{},stop_capture=33)['usable'])
