"""Frames for native pupil AprilTag corners passed to IPPE_SQUARE.

An upright frontal tag yields R_camera_tag = diag(-1, -1, 1).
Dock/body axes: forward, left, up; optical axes: right, down, forward.
"""
import numpy as np

R_OPTICAL_TO_BODY = np.array([[0., 0., 1.], [-1., 0., 0.], [0., -1., 0.]])
R_REFERENCE_TAG = R_OPTICAL_TO_BODY @ np.diag([-1., -1., 1.])
