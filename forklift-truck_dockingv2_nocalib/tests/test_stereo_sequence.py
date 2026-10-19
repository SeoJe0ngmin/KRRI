import unittest
from docking.stream import StereoSequence,CaptureClock

class StereoSequenceTest(unittest.TestCase):
    def test_repeated_ir_composite_is_not_new_vio_observation(self):
        sequence=StereoSequence();clock=CaptureClock();accepted=[]
        for row in ((10,10,1.,1.),(10,10,1.,1.),(11,11,1.033,1.033)):
            if sequence.accept(*row):
                clock.observe('left',row[2],row[2]+5);accepted.append(row[0])
        self.assertEqual(accepted,[10,11]);self.assertEqual(sequence.duplicates,1)
    def test_real_clock_regression_and_nonadvancing_timestamps_are_faults(self):
        for bad in ((11,11,.9,.9),(11,11,1.,1.),(11,11,1.03,.9)):
            sequence=StereoSequence();sequence.accept(10,10,1.,1.)
            with self.assertRaisesRegex(ValueError,'stereo_sequence_invalid'):
                sequence.accept(*bad)
    def test_capture_clock_reports_actual_values(self):
        clock=CaptureClock();clock.observe('left',2.,5.)
        with self.assertRaisesRegex(ValueError,'left: 2.000000000->1.000000000'):
            clock.observe('left',1.,6.)
    def test_reported_jetson_counter_reset_with_forward_time_is_accepted(self):
        sequence=StereoSequence();clock=CaptureClock()
        rows=((5,5,239.498699,239.498699),(0,0,239.669966,239.669966),
              (0,0,239.7033,239.7033),(1,1,239.7366,239.7366))
        for row in rows:
            self.assertTrue(sequence.accept(*row));clock.observe('left',row[2],row[2]+10)
        self.assertGreater(sequence.counter_anomalies,0)
        self.assertFalse(sequence.accept(*rows[-1]))
