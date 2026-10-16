import copy
import unittest
from docking.core import Box
from docking.visibility import tag_in_view, observable, validate_visibility
from test_core import config


def view_config():
    c = config()
    cal = c['sensor']['calibration']
    cal['T_body_camera'] = [[0, 0, 1, 0], [-1, 0, 0, 0], [0, -1, 0, .57], [0, 0, 0, 1]]
    cal['T_dock_tag'] = [[0, 0, -1, 0], [-1, 0, 0, .8], [0, 1, 0, 1.58], [0, 0, 0, 1]]
    cal['camera_model'] = dict(width=640, height=480, fx=400, fy=400, cx=320, cy=240,
                               coeffs=[0]*5, distortion='none', stream='infrared')
    c['visibility'] = {'enabled': True, 'projection_margin_px': 1., 'odometry_regions': [], 'odometry_evidence': None}
    return c


class VisibilityTests(unittest.TestCase):
    def test_side_tag_visible_farther_away_but_clips_close(self):
        c = view_config()
        validate_visibility(c)
        self.assertTrue(tag_in_view(Box.around([-3, 0, 0], [.01, .01, .001]), c))
        self.assertFalse(tag_in_view(Box.around([-.5, 0, 0], [.01, .01, .001]), c))
        self.assertFalse(tag_in_view(Box.around([-20, 0, 0], [.01, .01, .001]), c))

    def test_ninety_degree_turn_loses_tag(self):
        self.assertFalse(tag_in_view(Box.around([-3, 0, 1.5708], [.01, .01, .001]), view_config()))

    def test_printed_back_is_not_observable_even_if_projection_fits(self):
        c = view_config()
        # Rotate the tag by pi around its local vertical axis.
        for row in c['sensor']['calibration']['T_dock_tag'][:3]:
            row[0] *= -1
            row[2] *= -1
        validate_visibility(c)
        self.assertFalse(tag_in_view(Box.around([-3, 0, 0], [.01, .01, .001]), c))

    def test_tracking_region_needs_evidence_and_does_not_mean_tag_visible(self):
        c = view_config()
        b = Box.around([-.5, 0, 0], [.01, .01, .001])
        self.assertFalse(observable(b, c))
        c['visibility']['odometry_regions'] = [[-1, 1, -1, 1]]
        with self.assertRaises(ValueError):
            validate_visibility(c)
        c['visibility']['odometry_evidence'] = 'synthetic test; not a site qualification'
        validate_visibility(c)
        self.assertTrue(observable(b, c))
        self.assertFalse(tag_in_view(b, c))

    def test_model_for_another_stream_rejected(self):
        c = view_config()
        c['sensor']['calibration']['camera_model']['stream'] = 'color'
        with self.assertRaises(ValueError):
            validate_visibility(c)

    def test_odometry_pose_region_covers_heading_and_whole_uncertainty(self):
        c = view_config()
        c['visibility']['odometry_pose_regions'] = [[-1, 1, -1, 1, 1., 2.]]
        with self.assertRaises(ValueError): validate_visibility(c)
        c['visibility']['odometry_evidence'] = 'synthetic heading qualification'
        validate_visibility(c)
        self.assertTrue(observable(Box.around([-.5, 0, 1.5], [.01, .01, .1]), c))
        self.assertFalse(observable(Box.around([-.5, 0, 0.], [.01, .01, .1]), c))
        self.assertFalse(observable(Box.around([-.5, 0, 1.9], [.01, .01, .2]), c))
