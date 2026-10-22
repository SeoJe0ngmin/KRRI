"""Native detector regression, using an independently generated upright tag."""
import unittest
import cv2
import numpy as np
from legacy.docking_systemv2.robust_docking_v6.docking.frames import R_OPTICAL_TO_BODY, R_REFERENCE_TAG
from legacy.docking_systemv2.robust_docking_v6.docking.tag import Detector

class TagFrameTest(unittest.TestCase):
    def test_frontal_native_detector_heading_and_mount(self):
        try:
            import pupil_apriltags
        except ImportError:
            self.skipTest('native pupil_apriltags unavailable')
        if not hasattr(cv2, 'aruco'):
            self.skipTest('marker generator unavailable')
        image=np.full((480,640),255,np.uint8)
        image[120:360,200:440]=cv2.aruco.generateImageMarker(
            cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11),1,240)
        detector=Detector(dict(threads=1,quad_decimate=1.,full_scan_period_s=1.,
            id=1,min_margin=0,max_hamming=0,roi_padding=.2,edge_px=5,
            size_m=.3,max_reproj_px=2.,max_ambiguity_ratio=1.01))
        obs=detector.update(image,np.array([[600.,0,320],[0,600,240],[0,0,1]]),None,1.)
        self.assertTrue(obs['ok'],obs)
        Tct=obs['T_camera_tag']
        Rbc=R_REFERENCE_TAG@Tct[:3,:3].T
        Rdb=Rbc@R_OPTICAL_TO_BODY.T
        self.assertLess(abs(np.degrees(np.arctan2(Rdb[1,0],Rdb[0,0]))),5.)
        np.testing.assert_allclose(Rbc,R_OPTICAL_TO_BODY,atol=.1)
        camera_position=-R_REFERENCE_TAG@Tct[:3,:3].T@Tct[:3,3]
        self.assertLess(camera_position[0],-.5)
