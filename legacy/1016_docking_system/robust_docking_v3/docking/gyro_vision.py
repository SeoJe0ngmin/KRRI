"""Short-interval gyro rotation used only to initialize visual matching.

Not a full VIO: translation still requires image/depth correspondences. SDK
extrinsics map IMU vectors to the active optical stream; body yaw axes are
not substituted for that transform. No acceleration double integration.
"""
import math
import numpy as np
import cv2


def sdk_rotation(extrinsics):
    # rs2_extrinsics.rotation is column-major, including Python bindings.
    R = np.asarray(extrinsics.rotation, dtype=float).reshape(3, 3, order='F')
    if (not np.all(np.isfinite(R)) or
            not np.allclose(R.T.dot(R), np.eye(3), atol=1e-4) or
            abs(np.linalg.det(R)-1.) > 1e-4):
        raise ValueError('invalid factory gyro-to-camera rotation')
    return R


class GyroPrediction:
    def __init__(self, imu, gyro_profile, video_profile):
        self.imu, self.clock_valid = imu, False
        self.rotation = None
        self.settings = {'available': False, 'source': 'device_factory_extrinsics',
                         'use': 'optical_flow_initial_guess_only', 'full_vio': False}
        try:
            self.rotation = sdk_rotation(gyro_profile.get_extrinsics_to(video_profile))
            self.settings.update(available=True, R_camera_gyro=self.rotation.tolist())
        except (RuntimeError, ValueError, TypeError) as e:
            self.settings['reason'] = 'extrinsics_unavailable: '+str(e)

    def __call__(self, a, b):
        if self.rotation is None:
            return None, {'reason': 'extrinsics_unavailable'}
        if not self.clock_valid:
            return None, {'reason': 'clock_domains_not_matched'}
        samples, reason = self.imu.rotation_samples(a, b)
        if samples is None:
            return None, {'reason': reason}
        # R maps current IMU axes into reference IMU axes. Compose each
        # body-rate increment in chronological order, not as one Euler sum.
        R = np.eye(3)
        for (t0, w0), (t1, w1) in zip(samples, samples[1:]):
            rv = (np.asarray(w0)+np.asarray(w1))*(.5*(t1-t0))
            R = R.dot(cv2.Rodrigues(rv)[0])
        R = self.rotation.dot(R).dot(self.rotation.T)
        angle = math.acos(float(np.clip((np.trace(R)-1.)/2., -1., 1.)))
        return R, {'reason': 'exposure_aligned_gyro', 'samples': len(samples),
                   'interval_s': b-a, 'rotation_rad': angle}


def predict_pixels(points, K, dist, R_reference_current):
    """Predict static-scene image rotation; translation is left to LK/PnP."""
    rays = cv2.undistortPoints(points.astype(np.float64), K, dist).reshape(-1, 2)
    rays = np.column_stack((rays, np.ones(len(rays))))
    # A positive camera rotation moves scene rays in the opposite direction.
    current = rays.dot(R_reference_current)
    valid = np.all(np.isfinite(current), axis=1) & (current[:, 2] > 1e-8)
    guess = points.copy()
    if np.any(valid):
        projected = cv2.projectPoints(current[valid], np.zeros(3), np.zeros(3), K, dist)[0]
        finite = np.all(np.isfinite(projected.reshape(-1, 2)), axis=1)
        ids = np.flatnonzero(valid)
        valid[ids[~finite]] = False
        guess[ids[finite]] = projected[finite].astype(np.float32)
    return guess, valid
