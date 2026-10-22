"""Geometry and uncertainty conventions used by the v6 adapters.

SE3 errors: [dx,dy,dz,rx,ry,rz], translation and small LEFT rotation in
the parent frame (metres/radians). SE2 errors: [x,y,yaw]. Unknown cross
correlation is bounded explicitly, not silently assumed to be zero.
"""
import math
import numpy as np
import cv2
from .geometry import wrap, pose, matrix


def inverse(T):
    T = transform(T)
    out = np.eye(4)
    out[:3, :3] = T[:3, :3].T
    out[:3, 3] = -out[:3, :3] @ T[:3, 3]
    return out


def transform(T):
    T = np.asarray(T, dtype=float)
    if (T.shape != (4, 4) or not np.isfinite(T).all()
            or not np.allclose(T[3], [0, 0, 0, 1], atol=1e-7)
            or not np.allclose(T[:3, :3].T @ T[:3, :3], np.eye(3), atol=1e-5)
            or abs(np.linalg.det(T[:3, :3])-1) > 1e-5):
        raise ValueError('invalid rigid transform')
    return T


def covariance(P, n, positive=False):
    P = np.asarray(P, dtype=float)
    if P.shape != (n, n) or not np.isfinite(P).all() or not np.allclose(P, P.T, atol=1e-8):
        raise ValueError('invalid covariance shape/finite/symmetry')
    P = (P + P.T)/2
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
        d = np.zeros(len(x)); d[i] = epsilon
        diff = np.asarray(function(x+d))-np.asarray(function(x-d))
        for a in angular:
            diff[a] = wrap(diff[a])
        J[:, i] = diff/(2*epsilon)
    return J


def se3_planar(T, P):
    T = transform(T); P = covariance(P, 6)
    J = jacobian(lambda d: pose(perturb(T, d)), np.zeros(6), (2,))
    return pose(T), covariance(J @ P @ J.T, 3)


def compose(a, b):
    return pose(matrix(a) @ matrix(b))


def relative(a, b):
    return pose(inverse(matrix(a)) @ matrix(b))


def fork_uncertainty(p, P, length, length_std_m=0.):
    h = p[2]
    J = np.array([[1., 0., -length*math.sin(h)], [0., 1., length*math.cos(h)]])
    v = np.array([math.cos(h), math.sin(h)])
    C = J @ covariance(P, 3) @ J.T + length_std_m**2*np.outer(v, v)
    return np.asarray(p[:2])+length*v, covariance(C, 2)


def target_lateral_uncertainty(p, P, target_x, bias=0.):
    h = p[2]+bias                     # measured straight-drive drift (geometry.path_bias), 2026-10-21
    if math.cos(h) <= .2:
        return None, None
    value = p[1]+(target_x-p[0])*math.tan(h)
    J = np.array([-math.tan(h), 1., (target_x-p[0])/math.cos(h)**2])
    return float(value), float(max(0., J @ covariance(P, 3) @ J))
