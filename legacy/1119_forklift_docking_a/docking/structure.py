"""적재함의 앞뒤 방향 모서리(바닥과 옆벽이 만나는 선 등)가 화면에서 모이는 점(소실점)으로 차체가 트럭 축에서 틀어진 각을 잰다.

흑백 축소 → Canny → HoughLinesP 선분 → 수평 · 수직에 가까운 선분(격자 살 · 문틀)은 버림 → 선분마다 가장자리 점으로 직선을 다시
맞춤 → 두 직선의 교점 가운데 가장 많은 선분이 지나는 점 → 그 점을 지나는 선분들로 최소제곱. 트럭의 앞뒤 모서리는 수평이므로
장착 보정을 알면 수평선 가까이의 교점만 후보로 삼는다. 실제 현장 영상으로는 아직 확인하지 못한 측정이다.
"""

import math
import time
import numpy as np

DEFAULTS = dict(
    scale=0.5,  # 검출에 쓰는 화면 축소 비율
    clahe_clip=2.0,  # 어두운 적재함 안의 약한 모서리를 살리는 국소 대비 보정(0 = 끔)
    canny_percentile=90.0,  # 기울기 크기의 이 백분위를 가장자리 문턱값으로 쓴다(장면 밝기 · 대비에 맞춤)
    canny_min=20.0,
    canny_max=80.0,
    noise_factor=3.5,  # 문턱값은 기울기 크기 가운데 값(평평한 곳의 잡음 · 무늬)의 이 배수보다 낮추지 않는다
    min_polarity=0.8,  # 직선을 따라 밝기 기울기의 방향이 이만큼 한쪽으로 고르지 않으면 버린다(잡음 · 무늬가 우연히 늘어선 것)
    hough_threshold=30,
    min_length_px=30.0,  # 축소한 화면에서의 최소 선분 길이
    max_gap_px=4.0,
    skip_deg=10.0,  # 수평 · 수직에서 이 각 안의 선분은 버린다
    max_lines=60,  # 긴 것부터 이 수만 다시 맞춘다
    pair_lines=40,  # 교점 후보를 만드는 직선 수
    yaw_max_deg=20.0,  # 소실점을 찾는 창: 좌우 각
    prior_window_deg=4.0,  # 지금 믿는 방향을 알면 그 둘레 이 각 안에서만 찾는다(무늬가 우연히 만든 교점을 줄인다)
    elevation_max_deg=3.0,  # 장착 보정을 알 때: 수평에서 벗어난 각
    pitch_max_deg=15.0,  # 장착 보정을 모를 때: 카메라 광축 기준 위아래 각
    tol_deg=1.0,  # 직선이 그 점을 지난다고 보는 각도 허용값
    floor_px=1.5,
    min_lines=2,
    min_side=1,  # 소실점 왼쪽 · 오른쪽에 각각 있어야 하는 직선 수
    min_side_px=60.0,  # 그리고 왼쪽 · 오른쪽 각각의 직선 길이 합(원래 화소)
    max_spread_deg=0.6,  # 맞춘 좌우 각의 표준편차가 이보다 크면 쓰지 않는다
)
FORWARD = np.array([[0.0, 0.0, 1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
KEYS = ('max_spread_deg', 'min_lines', 'elevation_max_deg')  # 운영 설정(structure_ 로 시작하는 값)으로 바꿀 수 있는 것


def settings_from(tuning):
    """운영 설정에서 검출기에 넘길 값만 뽑는다."""
    return {k: tuning['structure_' + k] for k in KEYS if tuning.get('structure_' + k) is not None}


def model_matrix(model):
    return np.array([[model['fx'], 0.0, model['cx']], [0.0, model['fy'], model['cy']], [0.0, 0.0, 1.0]])


def normalized(points, model):
    """화소 좌표 → 렌즈 왜곡을 푼 정규 좌표(x/z, y/z)."""
    pts = np.asarray(points, float).reshape(-1, 2)
    coeffs = np.asarray(model.get('coeffs') or [], float)
    if coeffs.size and np.any(np.abs(coeffs) > 1e-12):
        import cv2

        return cv2.undistortPoints(pts.reshape(-1, 1, 2), model_matrix(model), coeffs).reshape(-1, 2)
    return np.column_stack([(pts[:, 0] - model['cx']) / model['fx'], (pts[:, 1] - model['cy']) / model['fy']])


def refit(edges, seg, dx=None, dy=None):
    """선분 둘레의 가장자리 점으로 직선을 다시 맞춘다: (끝점 둘, 점 수, 가운데의 수직 오차, 각 오차, 기울기 고름) 또는 None."""
    a = np.array(seg[:2], float)
    d = np.array(seg[2:], float) - a
    length = float(np.hypot(*d))
    if length < 1.0:
        return None
    u = d / length
    h, w = edges.shape
    for band in (2.5, 1.5):
        b = a + u * length
        pad = int(math.ceil(band)) + 1
        x0, x1 = int(max(0, math.floor(min(a[0], b[0])) - pad)), int(min(w - 1, math.ceil(max(a[0], b[0])) + pad))
        y0, y1 = int(max(0, math.floor(min(a[1], b[1])) - pad)), int(min(h - 1, math.ceil(max(a[1], b[1])) + pad))
        yy, xx = np.nonzero(edges[y0 : y1 + 1, x0 : x1 + 1])
        if len(xx) < 8:
            return None
        pts = np.column_stack([xx + x0, yy + y0]).astype(float)
        rel = pts - a
        along = rel @ u
        across = rel @ np.array([-u[1], u[0]])
        pts = pts[(np.abs(across) <= band) & (along >= -1.0) & (along <= length + 1.0)]
        if len(pts) < 8:
            return None
        centre = pts.mean(0)
        q = pts - centre
        _, vec = np.linalg.eigh(q.T @ q)
        v = vec[:, 1]  # 큰 고유값 쪽이 직선 방향
        if v @ u < 0:
            v = -v
        t = q @ v
        a, u, length = centre + v * t.min(), v, float(t.max() - t.min())
        if length < 1.0:
            return None
    across = np.array([-u[1], u[0]])
    r = q @ across
    sd = max(float(np.sqrt(np.mean(r**2))), 0.3)  # 가장자리 점은 화소 격자에 놓인다
    polarity = 1.0
    if dx is not None:
        xi, yi = pts[:, 0].astype(int), pts[:, 1].astype(int)
        gx, gy = dx[yi, xi].astype(float), dy[yi, xi].astype(float)
        polarity = abs(float(np.mean((gx * across[0] + gy * across[1]) / np.maximum(np.hypot(gx, gy), 1e-6))))
    return (
        a,
        a + u * length,
        len(pts),
        sd / math.sqrt(len(pts)),
        sd / max(float(np.sqrt(np.sum(t**2))), 1e-6),
        polarity,
    )


def measure(image, model, settings=None, T_body_camera=None, prior_deg=None):
    """컬러(또는 흑백) 한 장면에서 트럭 축 방향의 소실점을 찾는다. ok 가 아니면 reason 에 까닭을 적는다.
    T_body_camera 를 주면 heading_deg 는 차체 방향(트럭 축 기준, 왼쪽 +)이고, 없으면 카메라 광축 기준이다.
    prior_deg 는 지금 믿는 차체 방향으로, 주면 그 둘레에서만 찾는다."""
    import cv2

    t0 = time.perf_counter()
    s = dict(DEFAULTS)
    s.update(settings or {})

    def out(ok, reason, **more):
        return dict(ok=bool(ok), reason=reason, ms=round((time.perf_counter() - t0) * 1000.0, 2), **more)

    gray = image if image.ndim == 2 else cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    k = float(s['scale'])
    small = gray if k == 1.0 else cv2.resize(gray, None, fx=k, fy=k, interpolation=cv2.INTER_AREA)
    if float(s['clahe_clip']) > 0:
        small = cv2.createCLAHE(clipLimit=float(s['clahe_clip']), tileGridSize=(8, 8)).apply(small)
    blur = cv2.GaussianBlur(small, (3, 3), 0)
    dx = cv2.Sobel(blur, cv2.CV_16S, 1, 0, ksize=3)
    dy = cv2.Sobel(blur, cv2.CV_16S, 0, 1, ksize=3)
    mag = np.abs(dx[::4, ::4]).astype(np.int32) + np.abs(dy[::4, ::4])
    high = float(
        np.clip(np.percentile(mag, float(s['canny_percentile'])), float(s['canny_min']), float(s['canny_max']))
    )
    high = max(high, float(s['noise_factor']) * float(np.median(mag)))
    edges = cv2.Canny(dx, dy, 0.4 * high, high)
    raw = cv2.HoughLinesP(
        edges,
        1.0,
        math.pi / 180.0,
        int(s['hough_threshold']),
        minLineLength=float(s['min_length_px']),
        maxLineGap=float(s['max_gap_px']),
    )
    if raw is None:
        return out(False, 'no_segments', segments=0)
    seg = raw.reshape(-1, 4).astype(float)
    d = seg[:, 2:] - seg[:, :2]
    tilt = np.degrees(np.arctan2(np.abs(d[:, 1]), np.abs(d[:, 0])))  # 0 = 수평, 90 = 수직
    keep = (tilt >= float(s['skip_deg'])) & (tilt <= 90.0 - float(s['skip_deg']))
    seg, d = seg[keep], d[keep]
    order = np.argsort(-np.hypot(d[:, 0], d[:, 1]))[: int(s['max_lines'])]
    fits = [f for f in (refit(edges, seg[i], dx, dy) for i in order) if f is not None]
    fits = [f for f in fits if f[5] >= float(s['min_polarity'])]
    if len(fits) < int(s['min_lines']):
        return out(False, 'too_few_lines', segments=len(fits))
    # 축소한 화소 → 원래 화소 → 정규 좌표
    f = 0.5 * (float(model['fx']) + float(model['fy']))
    a = normalized((np.array([v[0] for v in fits]) + 0.5) / k - 0.5, model)
    b = normalized((np.array([v[1] for v in fits]) + 0.5) / k - 0.5, model)
    sd_c = np.array([v[3] for v in fits]) / k / f
    sd_t = np.array([v[4] for v in fits])
    d = b - a
    length = np.hypot(d[:, 0], d[:, 1])
    u = d / length[:, None]
    nrm = np.column_stack([-u[:, 1], u[:, 0]])
    c = -np.sum(nrm * a, 1)
    mid = 0.5 * (a + b)
    # 같은 모서리를 여러 번 잡은 것은 긴 것 하나만 남긴다
    order = np.argsort(-length)
    kept = []
    for i in order:
        same = False
        for j in kept:
            if abs(float(u[i] @ nrm[j])) < math.sin(math.radians(1.0)) and abs(float(nrm[j] @ mid[i] + c[j])) < 2.0 / f:
                same = True
                break
        if not same:
            kept.append(int(i))
    kept = np.array(kept, int)
    nrm, c, mid, length, sd_c, sd_t = nrm[kept], c[kept], mid[kept], length[kept], sd_c[kept], sd_t[kept]
    n = len(kept)
    if n < int(s['min_lines']):
        return out(False, 'too_few_lines', segments=n)
    tol = math.tan(math.radians(float(s['tol_deg'])))
    floor = float(s['floor_px']) / f
    yaw_mid = 0.0 if prior_deg is None else math.radians(float(prior_deg))
    yaw_max = math.radians(float(s['yaw_max_deg' if prior_deg is None else 'prior_window_deg']))
    up_max = math.radians(float(s['elevation_max_deg' if T_body_camera is not None else 'pitch_max_deg']))
    # 장착 보정이 없으면 광축을 앞, 화면 오른쪽을 차체 오른쪽으로 본다
    rot = FORWARD if T_body_camera is None else np.asarray(T_body_camera, float)[:3, :3]

    def angles(points):
        """정규 좌표의 점 → (좌우 각, 수평에서 벗어난 각), 차체 기준."""
        d = np.column_stack([points, np.ones(len(points))]) @ rot.T
        return np.arctan2(-d[:, 1], d[:, 0]), np.arctan2(d[:, 2], np.hypot(d[:, 0], d[:, 1]))

    side_px = float(s['min_side_px']) / f

    def support(points):
        """후보 점마다: 그 점을 지나는 직선, 왼쪽 · 오른쪽의 직선 수, 양쪽 모두 충분한지."""
        far = np.hypot(points[:, None, 0] - mid[None, :, 0], points[:, None, 1] - mid[None, :, 1])
        hit = np.abs(points @ nrm.T + c[None, :]) <= tol * far + floor
        on_left = hit & (mid[None, :, 0] < points[:, None, 0])
        on_right = hit & (mid[None, :, 0] > points[:, None, 0])
        left, right = on_left.sum(1), on_right.sum(1)
        enough = (left >= int(s['min_side'])) & (right >= int(s['min_side']))
        enough &= ((on_left * length[None, :]).sum(1) >= side_px) & ((on_right * length[None, :]).sum(1) >= side_px)
        return hit, left, right, far, enough

    def inside(points):
        yaw, up = angles(points)
        return (np.abs(yaw - yaw_mid) <= yaw_max) & (np.abs(up) <= up_max)

    # 두 직선의 교점 후보
    m = min(n, int(s['pair_lines']))
    i, j = np.triu_indices(m, 1)
    cross = np.cross(np.column_stack([nrm[i], c[i]]), np.column_stack([nrm[j], c[j]]))
    good = np.abs(cross[:, 2]) > 1e-9
    cand = cross[good, :2] / cross[good, 2:3]
    cand = cand[inside(cand)]
    if not len(cand):
        return out(False, 'no_candidate', segments=n)
    hit, left, right, _, enough = support(cand)
    score = (hit * length[None, :]).sum(1) * enough
    if float(score.max()) <= 0.0:
        return out(False, 'one_side_only', segments=n)
    v = cand[int(np.argmax(score))].copy()
    cov = None
    for _ in range(3):
        hit, left, right, far, _ = support(v[None, :])
        hit, far = hit[0], far[0]
        if hit.sum() < 2:
            return out(False, 'lost_in_refit', segments=n)
        w = hit / (sd_c**2 + (far * sd_t) ** 2 + (0.3 / f) ** 2)
        m2 = (nrm * w[:, None]).T @ nrm
        if np.linalg.cond(m2) > 1e10:
            return out(False, 'degenerate', segments=n)
        v = np.linalg.solve(m2, (nrm * w[:, None]).T @ (-c))
        res = nrm @ v + c
        dof = int(hit.sum()) - 2
        cov = np.linalg.inv(m2) * (max(1.0, float(np.sum(w * res**2)) / dof) if dof > 0 else 1.0)
    hit, left, right, _, enough = support(v[None, :])
    lines, left, right = int(hit[0].sum()), int(left[0]), int(right[0])
    spread = math.degrees(math.sqrt(max(float(cov[0, 0]), 0.0)) / (1.0 + float(v[0]) ** 2))
    direction = np.array([v[0], v[1], 1.0])
    yaw, up = angles(v[None, :])
    more = dict(
        heading_deg=float(math.degrees(yaw[0])),
        elevation_deg=float(math.degrees(up[0])),
        segments=n,
        lines=lines,
        left=left,
        right=right,
        length_px=float((hit[0] * length).sum() * f),
        spread_deg=spread,
        d_cam=(direction / np.linalg.norm(direction)).tolist(),
        vp=[float(model['fx'] * v[0] + model['cx']), float(model['fy'] * v[1] + model['cy'])],
    )
    if not bool(inside(v[None, :])[0]):
        return out(False, 'outside_window', **more)
    if lines < int(s['min_lines']) or not bool(enough[0]):
        return out(False, 'one_side_only', **more)
    if spread > float(s['max_spread_deg']):
        return out(False, 'spread_too_large', **more)
    return out(True, 'ok', **more)
