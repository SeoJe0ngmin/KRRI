"""Optional numerical vision tests. No camera or CAN device is opened."""
import importlib.util
import unittest
from unittest.mock import patch

AVAILABLE = importlib.util.find_spec('numpy') is not None and importlib.util.find_spec('cv2') is not None


@unittest.skipUnless(AVAILABLE, 'NumPy/OpenCV not installed in this interpreter')
class VisionTests(unittest.TestCase):
    def test_se3_inverse_and_body_camera_lever_arm(self):
        import numpy as np
        from docking.vision import inverse
        T = np.eye(4)
        T[:3, :3] = [[0, -1, 0], [1, 0, 0], [0, 0, 1]]
        T[:3, 3] = [.4, -.1, .6]
        np.testing.assert_allclose(T.dot(inverse(T)), np.eye(4), atol=1e-12)

    def test_tag_roi_restores_full_image_coordinates(self):
        import numpy as np
        from docking.vision import Tag
        from test_core import config
        class Detection:
            tag_id, decision_margin, hamming = 1, 100, 0
            # Tiny synthetic tag near edge: early clipped branch avoids PnP.
            corners = np.array([[1., 1.], [8., 1.], [8., 8.], [1., 8.]])
        class Detector:
            def detect(self, gray):
                return [Detection()]
        tag = Tag.__new__(Tag)
        tag.cfg = config()['sensor']['tag']
        tag.cfg['edge_px'] = 200
        tag.detector, tag.roi, tag.last_full = Detector(), (100, 50, 200, 150), 0.
        out = tag.update(np.zeros((480, 640), np.uint8), np.eye(3), np.zeros(5), .1)
        self.assertEqual(out['corners'][0], [101., 51.])

    def test_odometry_missing_depth_is_lost_not_zero_motion(self):
        import numpy as np
        from docking.vision import SparseOdom
        from test_core import config
        od = SparseOdom(config()['sensor']['odom'])
        gray = np.zeros((120, 160), np.uint8)
        depth = np.zeros(gray.shape, np.float32)
        K = np.array([[100., 0, 80], [0, 100, 60], [0, 0, 1]])
        od.update(gray, depth, K, np.zeros(5), 0.)
        result = od.update(gray, depth, K, np.zeros(5), .1)
        self.assertFalse(result['ok'])
        self.assertEqual(result['generation'], 1)

    def fake_tag(self, corners):
        from docking.vision import Tag
        from test_core import config
        class Detection:
            tag_id, decision_margin, hamming = 1, 100, 0
        detection = Detection()
        detection.corners = corners
        class Detector:
            def detect(self, gray):
                return [detection]
        tag = Tag.__new__(Tag)
        tag.cfg = config()['sensor']['tag']
        tag.detector, tag.roi, tag.last_full = Detector(), None, -1.
        return tag

    def test_small_tag_is_rejected_before_pose_solver(self):
        import numpy as np
        corners = np.array([[100, 100], [110, 100], [110, 110], [100, 110]], float)
        tag = self.fake_tag(corners)
        with patch('cv2.solvePnPGeneric') as solve:
            out = tag.update(np.zeros((480, 640), np.uint8), np.eye(3), np.zeros(5), 1.)
        self.assertEqual(out['reason'], 'tag_too_small')
        solve.assert_not_called()

    def test_two_equally_perfect_pnp_solutions_remain_ambiguous(self):
        import numpy as np
        import cv2
        obj = np.array([[-.15, .15, 0], [.15, .15, 0], [.15, -.15, 0], [-.15, -.15, 0]])
        K = np.array([[500., 0, 320], [0, 500, 240], [0, 0, 1]])
        rv, tv, dist = np.zeros((3, 1)), np.array([[0.], [0.], [2.]]), np.zeros(5)
        tag = self.fake_tag(cv2.projectPoints(obj, rv, tv, K, dist)[0].reshape(-1, 2))
        with patch('cv2.solvePnPGeneric', return_value=(2, [rv, rv], [tv, tv])):
            out = tag.update(np.zeros((480, 640), np.uint8), K, dist, 1.)
        self.assertFalse(out['ok'])
        self.assertEqual(out['ambiguity_ratio'], 1.)
        # Startup search stops on a decoded ID even before pose acceptance.
        self.assertTrue(out['detected'])
        self.assertEqual(out['id'], 1)

    def test_missing_optical_flow_invalidates_chain_instead_of_crashing(self):
        import numpy as np
        from docking.vision import SparseOdom
        from test_core import config
        gray, K = np.zeros((120, 160), np.uint8), np.eye(3)
        for missing_backward in (False, True):
            od = SparseOdom(config()['sensor']['odom'])
            od.prev, od.last_stamp = gray, 0.
            od.points, od.xyz = np.ones((20, 1, 2), np.float32), np.ones((20, 3))
            replies = [(None, None, None)]
            if missing_backward:
                replies.insert(0, (od.points.copy(), np.ones((20, 1), np.uint8), None))
            with patch('cv2.calcOpticalFlowPyrLK', side_effect=replies), patch.object(od, 'seed'):
                out = od.update(gray, None, K, np.zeros(5), .1)
            self.assertFalse(out['ok'])
            self.assertEqual(out['reason'], 'optical_flow_unavailable')
            self.assertEqual(out['generation'], 1)


if __name__ == '__main__':
    unittest.main()


@unittest.skipUnless(AVAILABLE, 'NumPy/OpenCV not installed in this interpreter')
class ReanchorTests(unittest.TestCase):
    def test_lost_tag_preserves_chain_but_lost_chain_requires_new_absolute_anchor(self):
        import numpy as np
        from docking.vision import Observer
        from test_core import config
        c = config()['sensor']
        c['tag']['enabled'] = False
        c['calibration'].update(T_body_camera=np.eye(4).tolist(), T_dock_tag=np.eye(4).tolist(), verified=True)
        c['bounds'].update(tag_pose_error=[.01, .01, .01], odom_error_per_m=[.01]*3,
                           odom_error_per_frame=[.001]*3, odom_error_per_rad=[.01]*3,
                           gyro_rate_error_rps=.001)
        observer = Observer(c)
        class Odom:
            ok, generation = True, 0
            def update(self, *args):
                return dict(ok=self.ok, generation=self.generation, dt=.2,
                    T_local_camera=np.eye(4), T_previous_current=np.eye(4), translation_m=0., rotation_rad=0.)
        class Tag:
            ok = True
            def update(self, *args):
                return dict(ok=self.ok, reason='synthetic', T_camera_tag=np.eye(4))
        observer.odom, observer.tag = Odom(), Tag()
        def frame(t):
            return observer.process(np.zeros((10, 10), np.uint8), np.ones((10, 10)), np.eye(3), np.zeros(5), t, 0.)
        first = frame(1.)
        self.assertTrue(first['bounded'])
        self.assertEqual(first['anchor_stamp'], 1.)
        observer.tag.ok = False
        tracked = frame(1.2)
        self.assertTrue(tracked['valid'])
        self.assertEqual(tracked['anchor_stamp'], 1.)
        self.assertGreater(tracked['error'][2], first['error'][2])
        observer.odom.ok, observer.odom.generation = False, 1
        self.assertFalse(frame(1.4)['valid'])
        observer.odom.ok = True
        reseeded = frame(1.6)
        self.assertFalse(reseeded['valid'])
        self.assertIsNone(reseeded['anchor_stamp'])
        observer.tag.ok = True
        reanchored = frame(1.8)
        self.assertTrue(reanchored['bounded'])
        self.assertEqual(reanchored['anchor_stamp'], 1.8)
        self.assertEqual(reanchored['tracking_generation'], 1)

    def test_local_motion_exists_before_tag_and_is_not_reset_by_reanchoring(self):
        import numpy as np
        from docking.vision import Observer, inverse
        from test_core import config
        c = config()['sensor']
        c['tag']['enabled'] = False
        bc = np.array([[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, .57], [0, 0, 0, 1.]])
        c['calibration'].update(T_body_camera=bc.tolist(), T_dock_tag=np.eye(4).tolist(), verified=True)
        c['bounds'].update(verified=True, tag_pose_error=[.01]*3, odom_error_per_m=[.01]*3,
                           odom_error_per_frame=[.001]*3, odom_error_per_rad=[.01]*3,
                           gyro_rate_error_rps=.001, relative_translation_error_m=.001,
                           max_acceleration_mps2=.01)
        observer = Observer(c)
        body_delta = np.eye(4)
        body_delta[:2, 3] = [.1, .2]
        class Odom:
            generation = 0
            last_stamp = None
            def update(self, *args):
                reference, self.last_stamp = self.last_stamp, args[-1]
                return dict(ok=True, reference_stamp=reference, generation=self.generation, dt=.2,
                    T_local_camera=inverse(bc).dot(body_delta).dot(bc), T_previous_current=np.eye(4),
                    translation_m=0., rotation_rad=0.)
        class Tag:
            ok = False
            def update(self, *args):
                return dict(ok=self.ok, id=1, reason='synthetic', T_camera_tag=np.eye(4))
        observer.odom, observer.tag = Odom(), Tag()
        def frame(t):
            return observer.process(np.zeros((10, 10), np.uint8), np.ones((10, 10)), np.eye(3), np.zeros(5), t, 0.,
                                    capture_imu=dict(device_stamp_s=t, yaw_rad=0., gaps=0))
        seed = frame(.8)
        self.assertFalse(seed['motion_bounded'])  # A single exposure cannot prove motion.
        first = frame(1.)
        self.assertFalse(first['valid'])
        self.assertTrue(first['local_motion']['bounded'])
        self.assertTrue(first['motion_bounded'])
        self.assertLess(first['speed_upper'], .02)
        np.testing.assert_allclose(first['local_motion']['xy'], [.1, .2])
        observer.tag.ok = True
        second = frame(1.2)
        self.assertTrue(second['valid'])
        self.assertGreater(second['local_motion']['error_xy'][0], first['local_motion']['error_xy'][0])
        np.testing.assert_allclose(second['local_motion']['xy'], first['local_motion']['xy'])
        observer.tag.ok = False
        observer.anchor = None
        c['bounds']['verified'] = False
        self.assertFalse(frame(1.4)['motion_bounded'])
