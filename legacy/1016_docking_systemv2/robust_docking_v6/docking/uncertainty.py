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


def compose_cov(a, Pa, b, Pb, unknown_correlation=True):
    A = jacobian(lambda q: compose(q, b), a, (2,))
    B = jacobian(lambda q: compose(a, q), b, (2,))
    P = A @ covariance(Pa, 3) @ A.T + B @ covariance(Pb, 3) @ B.T
    # Cov(X+Y) <= 2(Cov(X)+Cov(Y)) for arbitrary cross-correlation.
    return compose(a, b), covariance((2 if unknown_correlation else 1)*P, 3)


def relative(a, b):
    return pose(inverse(matrix(a)) @ matrix(b))


def anchor_observation(dock_body, Pd, odom_body, Po):
    fn = lambda x: pose(matrix(x[:3]) @ inverse(matrix(x[3:])))
    x = np.r_[dock_body, odom_body]
    J = jacobian(fn, x, (2,))
    P = 2*(J[:, :3] @ covariance(Pd, 3) @ J[:, :3].T
           + J[:, 3:] @ covariance(Po, 3) @ J[:, 3:].T)
    return fn(x), covariance(P, 3)


def innovation(a, Pa, b, Pb):
    d = np.asarray(b)-a; d[2] = wrap(d[2])
    # Upper covariance for a difference of correlated estimates. This statistic
    # is a conservative gate, not an exact chi-square NIS consistency test.
    S = 2*(covariance(Pa, 3)+covariance(Pb, 3)) + np.eye(3)*1e-12
    return d, float(d @ np.linalg.solve(S, d))


def intersection(a, Pa, b, Pb):
    """Covariance intersection: repeated correlated images cannot average away P."""
    Pa = covariance(Pa, 3, True); Pb = covariance(Pb, 3, True)
    Ia, Ib = np.linalg.inv(Pa), np.linalg.inv(Pb)
    best = None
    delta = np.asarray(b)-a; delta[2] = wrap(delta[2])
    for w in np.linspace(.05, .95, 19):
        P = np.linalg.inv(w*Ia+(1-w)*Ib)
        score = np.linalg.slogdet(P)[1]
        if best is None or score < best[0]:
            mean = np.asarray(a) + P @ ((1-w)*Ib @ delta)
            mean[2] = wrap(mean[2]); best = (score, mean, P)
    return best[1], covariance(best[2], 3)


def fork_uncertainty(p, P, length, length_std_m=0.):
    h = p[2]
    J = np.array([[1., 0., -length*math.sin(h)], [0., 1., length*math.cos(h)]])
    v = np.array([math.cos(h), math.sin(h)])
    C = J @ covariance(P, 3) @ J.T + length_std_m**2*np.outer(v, v)
    return np.asarray(p[:2])+length*v, covariance(C, 2)


def target_lateral_uncertainty(p, P, target_x):
    if math.cos(p[2]) <= .2:
        return None, None
    value = p[1]+(target_x-p[0])*math.tan(p[2])
    J = np.array([-math.tan(p[2]), 1., (target_x-p[0])/math.cos(p[2])**2])
    return float(value), float(max(0., J @ covariance(P, 3) @ J))


def drift_bound(P,standard_deviation_sum):
    """Unknown temporal correlation: accumulate std, not independent variances.

    For 3-D errors Q <= 3 diag(Q); Cauchy bounds cross-time covariance.
    A tag covariance is not made artificially small by resetting the odom gauge.
    """
    v=np.maximum(np.asarray(standard_deviation_sum,float),0.)
    if np.max(v)<1e-15:return covariance(P,3)
    return covariance(2*(P+3*np.diag(v*v)),3)
