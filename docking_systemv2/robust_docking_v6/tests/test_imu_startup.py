import unittest
from docking.stream import ImuSynchronizer


def live_window(imu,device=278.468867,host=10.001871,burst=False):
    for i in range(121):
        stamp=device+i*.005
        arrival=host+i*(.00001 if burst else .005)
        if i%2==0:imu.add('accel',stamp,[0,9.81,0],arrival)
        imu.add('gyro',stamp,[0,0,.01],arrival)
    return device+.6,host+120*(.00001 if burst else .005)


class ImuStartupTests(unittest.TestCase):
    def test_reported_26_second_startup_jump_never_reaches_estimator(self):
        imu=ImuSynchronizer(startup_window_s=.5)
        imu.add('accel',252.437301,[0,9.81,0],10)
        stamp,now=live_window(imu)
        self.assertIsNone(imu.fault)
        self.assertEqual(imu.startup_resets,1)
        self.assertIn('delta_s=26.031566000',imu.last_startup_reset)
        self.assertEqual(imu.drain(),[])
        self.assertTrue(imu.begin(stamp,now))
        samples=imu.drain()
        self.assertGreater(len(samples),100)
        self.assertEqual(samples[0]['sequence'],1)
        self.assertTrue(all(p['stamp']>=278.468867 for p in samples))
        self.assertTrue(all(p['accel']==[0.,9.81,0.] for p in samples))
        imu.add('accel',stamp+26,[0,9.81,0],now+.002)
        with self.assertRaisesRegex(RuntimeError,'stream=accel reason=sample_gap'):imu.drain()
        self.assertEqual(imu.startup_resets,1)

    def test_old_burst_is_not_live_warmup(self):
        imu=ImuSynchronizer(startup_window_s=.5)
        stamp,now=live_window(imu,burst=True)
        self.assertFalse(imu.begin(stamp,now))
        self.assertEqual(imu.drain(),[])

    def test_video_alignment_and_fresh_both_streams_required(self):
        imu=ImuSynchronizer(startup_window_s=.5)
        stamp,now=live_window(imu)
        self.assertFalse(imu.begin(stamp-26,now))
        self.assertFalse(imu.begin(stamp,now+.2))
        self.assertFalse(imu.begin(stamp,now-.01))
        self.assertTrue(imu.begin(stamp,now))

    def test_both_queues_restart_before_commit(self):
        for delta in (26,-26,0):
            imu=ImuSynchronizer(startup_window_s=.5)
            imu.add('gyro',100,[0,0,0],10);imu.add('accel',100,[0,9.81,0],10)
            imu.add('gyro',100+delta,[0,0,0],10.002)
            self.assertEqual(len(imu.accel),0)
            self.assertEqual(len(imu.gyro),1)
            self.assertEqual(imu.sequence,0)
            self.assertFalse(imu.begin(100+delta,10.002))

    def test_video_wait_keeps_imu_memory_bounded(self):
        imu=ImuSynchronizer(capacity=512,startup_window_s=.5)
        for i in range(6001):
            t=i*.005
            if i%2==0:imu.add('accel',t,[0,9.81,0],t+10)
            imu.add('gyro',t,[0,0,0],t+10)
        self.assertIsNone(imu.fault)
        self.assertLessEqual(len(imu.gyro),201)
        self.assertLessEqual(len(imu.accel),101)
        self.assertGreater(imu.startup_discarded,0)
        self.assertEqual(imu.drain(),[])

    def test_startup_warmup_does_not_certify_stationarity(self):
        imu=ImuSynchronizer(startup_window_s=.5)
        stamp,now=live_window(imu)
        self.assertTrue(imu.begin(stamp,now))
        samples=imu.drain()
        self.assertTrue(all(p['gyro'][2]==.01 for p in samples))
        self.assertNotIn('stationary',imu.status())
