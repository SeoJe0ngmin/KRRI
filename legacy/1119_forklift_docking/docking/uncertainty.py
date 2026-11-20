"""Geometry and uncertainty conventions used by the adapters.

SE3 errors: [dx,dy,dz,rx,ry,rz], translation and small LEFT rotation in
the parent frame (metres/radians). SE2 errors: [x,y,yaw]. Unknown cross
correlation is bounded explicitly, not silently assumed to be zero.
"""

import math
import numpy as np
import cv2
from .geometry import wrap, pose, matrix

EYE3 = np.eye(3)
BOTTOM_ROW = np.array([0.0, 0.0, 0.0, 1.0])


def close(a, b, atol, rtol=1e-5):
    """numpy.allclose's rule (|a - b| <= atol + rtol * |b|, NaN never close) without its per-call overhead: the validation
    helpers below run ~20 times per VIO packet and allclose was 45 % of the observer's time on the 2026-10-30 replay."""
    a = np.asarray(a, dtype=float)
    b = np.asarray(b, dtype=float)
    return bool(np.all(np.abs(a - b) <= atol + rtol * np.abs(b)))


def inverse(T):
    T = transform(T)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def transform(T):
    T = np.asarray(T, dtype=float)
    if T.shape != (4, 4) or not np.isfinite(T).all():
        raise ValueError('invalid rigid transform')
    R = T[:3, :3]
    if not close(T[3], BOTTOM_ROW, 1e-7) or not close(R.T @ R, EYE3, 1e-5) or abs(np.linalg.det(R) - 1) > 1e-5:
        raise ValueError('invalid rigid transform')
    return T


def covariance(P, n, positive=False):
    P = np.asarray(P, dtype=float)
    if P.shape != (n, n) or not np.isfinite(P).all() or not close(P, P.T, 1e-8):
        raise ValueError('invalid covariance shape/finite/symmetry')
    P = (P + P.T) / 2
    eig = np.linalg.eigvalsh(P)
    if eig[0] < -1e-9 or (positive and eig[0] <= 0):
        raise ValueError('covariance must be positive (semi)definite')
    return P


def perturb(T, delta):
    out = np.array(T, float).copy()
    out[:3, 3] += delta[:3]
    out[:3, :3] = cv2.Rodrigues(np.asarray(delta[3:]))[0] @ out[:3, :3]
    return out


def jacobian(function, x, angular=(), epsilon=1e-5):
    x = np.asarray(x, float)
    J = np.empty((len(function(x)), len(x)))
    for i in range(len(x)):
        d = np.zeros(len(x))
        d[i] = epsilon
        diff = np.asarray(function(x + d)) - np.asarray(function(x - d))
        for a in angular:
            diff[a] = wrap(diff[a])
        J[:, i] = diff / (2 * epsilon)
    return J


def skew(v):
    return np.array([[0.0, -v[2], v[1]], [v[2], 0.0, -v[0]], [-v[1], v[0], 0.0]])


def yaw_gradient(R):
    """d atan2(R10, R00) / d(left rotation vector) at R: the yaw of (I + [phi]x) R to first order."""
    den = R[0, 0] * R[0, 0] + R[1, 0] * R[1, 0]
    if den < 1e-12:
        return None
    return np.array([-R[0, 0] * R[2, 0] / den, -R[1, 0] * R[2, 0] / den, 1.0])


def planar_jacobian(T):
    """Closed form of jacobian(lambda d: pose(perturb(T, d)), zeros(6), (2,)): perturb adds the translation part to t and
    left-multiplies R by Rodrigues(phi), so x, y move with dx, dy only and the yaw with phi through yaw_gradient."""
    g = yaw_gradient(T[:3, :3])
    if g is None:
        return jacobian(lambda d: pose(perturb(T, d)), np.zeros(6), (2,))
    J = np.zeros((3, 6))
    J[0, 0] = 1.0
    J[1, 1] = 1.0
    J[2, 3:] = g
    return J


def conjugated_planar_jacobian(Tbi, D):
    """Closed form of jacobian(lambda e: pose(Tbi @ perturb(D, e) @ inverse(Tbi)), zeros(6), (2,)) - the body-frame planar
    increment of an IMU-frame relative transform D: with A = R_D Rb^T and v = A tb, t = -Rb Rodrigues(phi) v + Rb (t_D + rho) + tb
    and R = Rb Rodrigues(phi) A, i.e. a left rotation of the result by Rb phi."""
    Rb = Tbi[:3, :3]
    tb = Tbi[:3, 3]
    M = Tbi @ D @ inverse(Tbi)
    g = yaw_gradient(M[:3, :3])
    if g is None:
        Tib = inverse(Tbi)
        return jacobian(lambda e: pose(Tbi @ perturb(D, e) @ Tib), np.zeros(6), (2,))
    v = D[:3, :3] @ Rb.T @ tb
    J = np.zeros((3, 6))
    J[:2, :3] = Rb[:2, :]
    J[:2, 3:] = (Rb @ skew(v))[:2, :]
    J[2, 3:] = g @ Rb
    return J


def se3_planar(T, P):
    T = transform(T)
    P = covariance(P, 6)
    J = planar_jacobian(T)
    return pose(T), covariance(J @ P @ J.T, 3)


def compose(a, b):
    return pose(matrix(a) @ matrix(b))


def relative(a, b):
    return pose(inverse(matrix(a)) @ matrix(b))


def fork_uncertainty(p, P, length, length_std_m=0.0):
    h = p[2]
    J = np.array([[1.0, 0.0, -length * math.sin(h)], [0.0, 1.0, length * math.cos(h)]])
    v = np.array([math.cos(h), math.sin(h)])
    C = J @ covariance(P, 3) @ J.T + length_std_m**2 * np.outer(v, v)
    return np.asarray(p[:2]) + length * v, covariance(C, 2)


def target_lateral_uncertainty(p, P, target_x, bias=0.0):
    h = p[2] + bias  # measured straight-drive drift (geometry.path_bias), 2026-10-21
    if math.cos(h) <= 0.2:
        return None, None
    value = p[1] + (target_x - p[0]) * math.tan(h)
    J = np.array([-math.tan(h), 1.0, (target_x - p[0]) / math.cos(h) ** 2])
    return float(value), float(max(0.0, J @ covariance(P, 3) @ J))
