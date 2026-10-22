import unittest
from unittest.mock import patch,Mock
from types import SimpleNamespace
from docking.stream import ImuSynchronizer
from docking.runtime import run
from docking.controller import Controller
from support import config,observation

class RecoveryPolicyTest(unittest.TestCase):
    def test_gyro_gaps_keep_real_timestamps_and_are_counted_not_fatal(self):
        imu=ImuSynchronizer(short_gap_s=.1)
        for i in range(21):imu.add('accel',i*.01,[0,9.81,0],i*.01)
        for t in (0.,.005,.075):imu.add('gyro',t,[0,0,.2],t)
        samples=imu.drain()
        self.assertEqual([x['stamp'] for x in samples],[0.,.005,.075])
        self.assertEqual(imu.status()['gaps'],1)
        self.assertAlmostEqual(sum((b['stamp']-a['stamp'])*.2 for a,b in zip(samples,samples[1:])),.015)
        imu.add('gyro',.145,[0,0,.2],.145);imu.add('gyro',.5,[0,0,.2],.5)   # second and third gaps: still no fault (v4 rule)
        self.assertIsNone(imu.fault);self.assertEqual(imu.status()['gaps'],3)
        imu.add('gyro',.4,[0,0,.2],.6)   # backwards stamp: skipped and counted, not fatal
        self.assertIsNone(imu.fault);self.assertEqual(imu.status()['reversals'],1)
    def test_short_accel_gap_bracketed_interpolation(self):
        imu=ImuSynchronizer(short_gap_s=.1)
        imu.add('accel',0,[0,0,0],0);imu.add('accel',.07,[.07,0,0],.07)
        imu.add('gyro',.035,[0,0,0],.035)
        self.assertAlmostEqual(imu.drain()[0]['accel'][0],.035)
    def test_measure_restarts_and_resets_callback_but_drive_never_retries(self):
        cb=Mock();cb.reset=Mock()
        args=SimpleNamespace(command='measure',output=None)
        with patch('docking.runtime._run_once',side_effect=[RuntimeError('unbracketed_imu'),'done']) as once,patch('docking.runtime.time.sleep'):
            self.assertEqual(run(args,stop_when=cb),'done');self.assertEqual(once.call_count,2);cb.reset.assert_called_once()
        args.command='drive'
        with patch('docking.runtime._run_once',side_effect=RuntimeError('unbracketed_imu')) as once:
            with self.assertRaises(RuntimeError):run(args)
            self.assertEqual(once.call_count,1)
    def test_preparation_pause_retains_only_confirmed_time(self):
        cfg=config();cfg['tuning']['preparation_s']=3.
        c=Controller(cfg,0,0)
        for i in range(21):c.step(i*.1,observation(i*.1))
        before=c.prep_seconds
        for i in range(21,26):c.step(i*.1,observation(i*.1,stationary=False,stationary_report=dict(reason='settling_window')))
        self.assertAlmostEqual(c.prep_seconds,before)
        for i in range(26,41):c.step(i*.1,observation(i*.1))
        self.assertEqual(c.state,'WAIT_ENTER')
    def test_preparation_movement_resets_and_settle_timeout_recovers(self):
        cfg=config();c=Controller(cfg,0,0)
        for i in range(21):c.step(i*.1,observation(i*.1))
        c.step(2.1,observation(2.1,stationary=False,local=(.1,0,0),stationary_report=dict(reason='settling_window')))
        self.assertEqual(c.prep_seconds,0)
        c.state='SETTLE';c.stop_at=0
        c.step(21,observation(21,stationary=False))
        self.assertEqual(c.state,'RECOVER');self.assertEqual(c.command,'stop')
        c.step(61,observation(61,stationary=False));self.assertEqual(c.state,'FAULT')
