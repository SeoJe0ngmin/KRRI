"""서서 판단하는 지게차용 작은 칼만 필터 (EKF) — 상태 셋, 측정 셋.

좌표 (v5 와 같은 부호 — 2026-10-01 기록 11,916 장으로 확인: lateral = −ρ·sin(ψ − β), 잔차 RMS 1.1 cm)
    L  카메라 좌우 [m]   — v5 Fix.lateral_m 와 같은 축(태그 좌표 x)
    F  카메라 앞거리 [m] — 태그면까지 법선 방향, v5 Fix.forward_m
    ψ  카메라 방향 [rad] — 태그 법선 기준. v5 heading_deg(차체)에 카메라 어긋난 각(CAM_YAW_OFFSET_DEG)을 더한 것

측정 (한 번 서 있을 때 하나씩)
    β       화면위치 [rad] = ψ − atan2(−L, F)          — 픽셀에서 바로 나와 정확하다
    d       태그까지 3D 거리 [m] = √(L² + F² + dz²)     — dz = 줄자 높이차
    ψ_pnp   PnP 방향 [rad] = ψ                          — 자세마다 치우친다. R 을 크게 줘서 "기준" 만 잡게 한다

움직임 (자이로가 회전을 잰다. 직진 거리는 명령 시간으로 추정)
    rotate  회전중심(A, b — 차체 축) 둘레로 Δ 만큼. v5 plan._Geo.center/camera 와 같은 식
    drive   차체 방향으로 s 만큼: L += s·sin h, F −= s·cos h  (h = ψ − 카메라 어긋난 각)

주의 — 태그 한 장으로는 "태그 둘레로 전부 같이 돈 자세" 를 β·d 가 구분하지 못한다(관측 불가).
기준 방향은 ψ_pnp 로만 잡히고, 그 오차가 치우침이면 칼만 필터도 못 없앤다. 이 파일은 계산만 한다 — 정확도 주장이 아니다.
"""
import math

import numpy as np


def wrap(a):
    """각을 (−π, π] 로."""
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def measure(x, dz):
    """상태 → (β, d, ψ_pnp) 예측."""
    L, F, psi = x
    rho = math.hypot(L, F)
    return np.array([wrap(psi - math.atan2(-L, F)), math.sqrt(rho * rho + dz * dz), psi])


def measure_jacobian(x, dz):
    L, F, _ = x
    rho2 = L * L + F * F
    d = math.sqrt(rho2 + dz * dz)
    return np.array([[F / rho2, -L / rho2, 1.0],
                     [L / d, F / d, 0.0],
                     [0.0, 0.0, 1.0]])


def from_measurement(beta, d, psi, dz):
    """(β, d, ψ) → 상태. 첫 판단에서 필터를 연다."""
    rho = math.sqrt(max(1e-9, d * d - dz * dz))
    phi = psi - beta                                    # = atan2(−L, F)
    return np.array([-rho * math.sin(phi), rho * math.cos(phi), psi])


def from_measurement_jacobian(beta, d, psi, dz):
    rho = math.sqrt(max(1e-9, d * d - dz * dz))
    phi = psi - beta
    drho = d / rho
    # L = −ρ sin φ, F = ρ cos φ, φ = ψ − β
    return np.array([[rho * math.cos(phi), -drho * math.sin(phi), -rho * math.cos(phi)],
                     [rho * math.sin(phi), drho * math.cos(phi), -rho * math.sin(phi)],
                     [0.0, 0.0, 1.0]])


def rotate(x, delta, A, b, yaw_off):
    """회전중심 둘레로 delta [rad]. A·b 는 v5 config 부호(A 음수 = 중심이 카메라 앞, b + = 왼쪽), 차체 방향 기준."""
    L, F, psi = x
    h = psi - yaw_off
    Lc = L - A * math.sin(h) + b * math.cos(h)
    Fc = F + A * math.cos(h) + b * math.sin(h)
    h2 = h + delta
    return np.array([Lc + A * math.sin(h2) - b * math.cos(h2), Fc - A * math.cos(h2) - b * math.sin(h2), psi + delta])


def drive(x, s, delta, yaw_off):
    """차체 방향으로 s [m] (후진은 음수), 그동안 자이로가 잰 회전 delta 는 가운데 방향으로 친다."""
    L, F, psi = x
    h = psi - yaw_off + delta / 2.0
    return np.array([L + s * math.sin(h), F - s * math.cos(h), psi + delta])


def numeric_jacobian(f, x, eps=1e-6):
    x = np.asarray(x, float)
    y0 = f(x)
    J = np.zeros((len(y0), len(x)))
    for i in range(len(x)):
        dx = np.zeros(len(x))
        dx[i] = eps
        J[:, i] = (f(x + dx) - f(x - dx)) / (2 * eps)
    return J


class Filter:
    """x·P 를 들고 다닌다. predict_* 로 움직이고 update 로 측정을 넣는다."""

    def __init__(self, x, P):
        self.x, self.P = np.asarray(x, float), np.asarray(P, float)

    def predict(self, f, Q):
        J = numeric_jacobian(f, self.x)
        self.x = f(self.x)
        self.x[2] = wrap(self.x[2])
        self.P = J @ self.P @ J.T + Q

    def update(self, z, R, dz, use=(True, True, True)):
        """z = (β, d, ψ_pnp), R = 각 측정의 분산. use 로 빼고 넣는다(헷갈린 PnP 방향 등).
        돌려주는 것: 혁신(측정 − 예측), 혁신 분산 대각, NIS(혁신²/분산) — 넣기 전 예측 기준."""
        idx = [i for i, u in enumerate(use) if u]
        zhat = measure(self.x, dz)
        H = measure_jacobian(self.x, dz)
        nu = np.array(z, float) - zhat
        nu[0], nu[2] = wrap(nu[0]), wrap(nu[2])
        S = H @ self.P @ H.T + np.diag(R)
        out = {"nu": nu, "S": np.diag(S).copy(), "nis": nu ** 2 / np.diag(S)}
        if idx:
            Hs, Ss, nus = H[idx], S[np.ix_(idx, idx)], nu[idx]
            K = self.P @ Hs.T @ np.linalg.inv(Ss)
            self.x = self.x + K @ nus
            self.x[2] = wrap(self.x[2])
            I = np.eye(len(self.x))
            self.P = (I - K @ Hs) @ self.P @ (I - K @ Hs).T + K @ np.diag(np.asarray(R)[idx]) @ K.T   # Joseph 꼴
        return out


if __name__ == "__main__":
    # 자체 시험 — 식이 서로 맞는지만 본다(논리 확인). 정확도 주장이 아니다
    dz = 1.01
    x = np.array([-0.4, 4.2, math.radians(7.0)])
    z = measure(x, dz)
    assert np.allclose(from_measurement(*z, dz), x), (from_measurement(*z, dz), x)              # 측정 ↔ 상태 왕복
    assert np.allclose(measure_jacobian(x, dz), numeric_jacobian(lambda y: measure(y, dz), x), atol=1e-6)
    assert np.allclose(from_measurement_jacobian(*z, dz),
                       numeric_jacobian(lambda m: from_measurement(*m, dz), z), atol=1e-6)
    # 회전: v5 plan._Geo 와 같은 식 — 중심은 그대로, 한 바퀴 돌면 제자리
    A, b, yaw = -0.465, -0.112, math.radians(1.14)

    def center(y):
        h = y[2] - yaw
        return (y[0] - A * math.sin(h) + b * math.cos(h), y[1] + A * math.cos(h) + b * math.sin(h))
    y = rotate(x, math.radians(30.0), A, b, yaw)
    assert np.allclose(center(y), center(x)) and abs(y[2] - x[2] - math.radians(30.0)) < 1e-12
    assert np.allclose(rotate(x, 2 * math.pi, A, b, yaw)[:2], x[:2])
    # 직진: 정면(h = 0)으로 1 m 가면 앞거리만 1 m 준다
    x0 = np.array([0.0, 5.0, yaw])
    assert np.allclose(drive(x0, 1.0, 0.0, yaw), [0.0, 4.0, yaw])
    # 갱신: 측정을 그대로 넣으면 상태가 안 움직이고 P 는 줄어든다
    f = Filter(x, np.diag([0.04, 0.04, math.radians(2) ** 2]))
    P0 = f.P.copy()
    r = f.update(measure(x, dz), [math.radians(0.01) ** 2, 0.003 ** 2, math.radians(1.5) ** 2], dz)
    assert np.allclose(f.x, x) and np.all(np.diag(f.P) < np.diag(P0)) and np.allclose(r["nu"], 0)
    print("ekf 자체 시험 통과 (식끼리 맞는지만 — 정확도 아님)")
