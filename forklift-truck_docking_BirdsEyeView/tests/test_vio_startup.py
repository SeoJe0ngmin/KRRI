import unittest
from unittest.mock import Mock,patch
from types import SimpleNamespace
from docking.stream import OutputWatchdog,VideoDeadline,ImuSynchronizer
from docking.ros_adapter import Bridge
from docking.runtime import advance_capture


class VioStartupTests(unittest.TestCase):
    def test_partial_frames_and_duplicates_cannot_extend_deadline(self):
        d=VideoDeadline(10,startup_timeout=30)
        for t in range(11,40):
            d.incomplete+=1;d.check(t)
        with self.assertRaisesRegex(RuntimeError,'startup.*incomplete_framesets=29'):d.check(40.1)
        d=VideoDeadline(10);d.accepted(12);d.check(13.9)
        with self.assertRaisesRegex(RuntimeError,'streaming'):d.check(14.1)

    def test_imu_gap_is_recorded_with_device_and_host_time_but_not_fatal(self):
        for stamp,reason,field in ((.99,'clock_reversed','reversals'),(1,'duplicate_timestamp','reversals'),(1.06,'sample_gap','gaps')):
            imu=ImuSynchronizer();imu.add('accel',1,[0,9.8,0],10)
            imu.add('accel',stamp,[0,9.8,0],10.1)
            self.assertIsNone(imu.fault);status=imu.status()
            self.assertIn('stream=accel reason='+reason,status['last_gap'])
            self.assertIn('host_arrival_gap_s=0.100000',status['last_gap'])
            self.assertEqual(status[field],1);imu.drain()
            self.assertEqual(status['counts'],{'accel':1 if field=='reversals' else 2})

    def test_ros_decode_exception_reaches_supervisor(self):
        b=Bridge.__new__(Bridge);b.receive_error=None
        b.receive(SimpleNamespace())
        self.assertIn('state decoding',b.receive_error)
        with self.assertRaisesRegex(RuntimeError,'state decoding'):b.check()

    def test_partial_ros_startup_can_close_twice(self):
        b=Bridge.__new__(Bridge);b.closing=False
        b.process=b.executor=b.thread=b.log=None
        b.node=Mock();b.node.destroy_node.side_effect=RuntimeError('shutdown failed')
        b.context=object();b.rclpy=Mock()
        self.assertIn('node: shutdown failed',b.close())
        b.rclpy.shutdown.assert_called_once_with(context=b.context)
        b.close();self.assertEqual(b.node.destroy_node.call_count,1)

    def test_dead_launcher_still_cleans_native_process_group(self):
        b=Bridge.__new__(Bridge);b.closing=False
        b.executor=b.thread=b.log=b.node=b.context=None
        b.process=Mock(pid=123);b.process.poll.return_value=1
        with patch('docking.ros_adapter.os.killpg') as kill:
            b.close();self.assertEqual(kill.call_count,2)

    def test_executor_failure_is_reported(self):
        b=Bridge.__new__(Bridge);b.closing=False;b.receive_error=None
        b.executor=Mock();b.executor.spin.side_effect=RuntimeError('DDS failed')
        b.spin();self.assertIn('DDS failed',b.receive_error)

    def test_preview_time_does_not_reject_first_delayed_estimator_state(self):
        last=advance_capture(None,dict(capture_id=20,preview_only=True))
        self.assertIsNone(last)
        last=advance_capture(last,dict(capture_id=19))
        self.assertEqual(last,19)
        with self.assertRaisesRegex(RuntimeError,'nonmonotonic'):
            advance_capture(last,dict(capture_id=18))

    def test_slow_camera_start_is_not_a_native_output_timeout(self):
        w=OutputWatchdog()
        self.assertFalse(w.expired(25))
        w.input(25)
        self.assertFalse(w.expired(34.9))
        self.assertTrue(w.expired(35.1))

    def test_more_input_cannot_hide_a_stuck_estimator(self):
        w=OutputWatchdog();w.input(0)
        for i in range(11):w.input(i)
        self.assertTrue(w.expired(10.1))
        w.output(11)
        self.assertFalse(w.expired(20.9))
        self.assertTrue(w.expired(21.1))

    def test_discovery_requires_all_three_message_paths(self):
        for missing in range(4):
            b=SimpleNamespace(imu_pub=SimpleNamespace(get_subscription_count=lambda:int(missing!=0)),
                image_pub=SimpleNamespace(get_subscription_count=lambda:int(missing!=1)),
                sub=SimpleNamespace(get_publisher_count=lambda:int(missing!=2)))
            self.assertEqual(Bridge.peers_ready(b),missing==3)
