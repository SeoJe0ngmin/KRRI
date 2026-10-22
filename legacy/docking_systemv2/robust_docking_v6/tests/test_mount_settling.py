import copy
import unittest
import cv2
import numpy as np
from legacy.docking_systemv2.robust_docking_v6.tests.support import config
from legacy.docking_systemv2.robust_docking_v6.docking.mount_calibration import estimate,R_OPTICAL_TO_BODY,R_REFERENCE_TAG

class MountSettlingTest(unittest.TestCase):
    def rows(self):
        rv=cv2.Rodrigues(R_OPTICAL_TO_BODY.T@R_REFERENCE_TAG)[0].ravel().tolist()
        rows=[]
        for i in range(350):
            settling=150<=i<160
            rows.append(dict(arrival=i/5,stationary=not settling,sensor_ready=True,local_valid=True,
                sensor_epoch=1,motion_epoch=1,stationary_report=dict(reason='settling_window' if settling else 'stationary_with_vibration'),
                vio_gyro_norm=.002,imu=dict(age_s=.01,calibration=dict(state='READY')),
                tag=dict(ok=True,reason='accepted',capture_key=str(i),camera_rotation_vector_rad=rv,camera_translation_m=[0,0,3.])))
        return rows
    def check(self,rows):return estimate(rows,config()['sensor'],{},60)
    def test_short_observed_recheck_excluded_and_extra_time_collected(self):
        r=self.check(self.rows());self.assertTrue(r['usable'],r)
        self.assertGreater(r['elapsed_window_s'],62)
        self.assertGreaterEqual(r['confirmed_stationary_s'],60)
    def test_missing_tracking_movement_or_epoch_change_rejected(self):
        for field,value in [('local_valid',False),('motion_epoch',2)]:
            rows=self.rows();rows[155][field]=value
            self.assertFalse(self.check(rows)['usable'])
        rows=self.rows();rows[155]['tag']['camera_translation_m']=[.03,0,3]
        self.assertFalse(self.check(rows)['usable'])
        rows=self.rows();rows[155]['tag']['ok']=False
        self.assertFalse(self.check(rows)['usable'])
    def test_settling_not_counted_toward_required_duration(self):
        r=self.check(self.rows()[:306]);self.assertIn('measurement_too_short',r['reasons'])
    def test_real_rebound_reason_not_skipped(self):
        rows=self.rows();rows[155]['stationary_report']['reason']='residual_motion_or_rebound'
        self.assertFalse(self.check(rows)['usable'])
