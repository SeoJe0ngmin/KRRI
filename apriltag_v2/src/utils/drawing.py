import cv2
import numpy as np

def draw_cube(overlay, camera_params, tag_size, pose, z_sign=1):
    opoints = np.array([-1, -1, 0, 1, -1, 0, 1, 1, 0, -1, 1, 0, -1, -1, -2 * z_sign, 1, -1, -2 * z_sign, 1, 1, -2 * z_sign, -1, 1, -2 * z_sign]).reshape(-1, 1, 3) * 0.5 * tag_size
    edges = np.array([0, 1, 1, 2, 2, 3, 3, 0, 0, 4, 1, 5, 2, 6, 3, 7, 4, 5, 5, 6, 6, 7, 7, 4]).reshape(-1, 2)
    fx, fy, cx, cy = camera_params
    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]
    dcoeffs = np.zeros(5)
    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, dcoeffs)
    ipoints = np.round(ipoints).astype(int)
    ipoints = [tuple(pt) for pt in ipoints.reshape(-1, 2)]
    for i, j in edges:
        cv2.line(overlay, ipoints[i], ipoints[j], (0, 255, 0), 1, 16)
    return overlay

def draw_axes(overlay, camera_params, tag_size, pose, length=None, thickness=3):
    L = length if length is not None else tag_size * 0.5
    opoints = np.float32([[0, 0, 0], [L, 0, 0], [0, L, 0], [0, 0, -L]]).reshape(-1, 3)
    fx, fy, cx, cy = camera_params
    K = np.array([fx, 0, cx, 0, fy, cy, 0, 0, 1]).reshape(3, 3)
    rvec, _ = cv2.Rodrigues(pose[:3, :3])
    tvec = pose[:3, 3]
    ipoints, _ = cv2.projectPoints(opoints, rvec, tvec, K, np.zeros(5))
    o, x, y, z = [tuple(p) for p in np.round(ipoints).astype(int).reshape(-1, 2)]
    cv2.line(overlay, o, x, (0, 0, 255), thickness, 16)
    cv2.line(overlay, o, y, (0, 255, 0), thickness, 16)
    cv2.line(overlay, o, z, (255, 0, 0), thickness, 16)
    return overlay

def draw_corners(overlay, detection, color=(0, 0, 255), thickness=2):
    pts = np.round(detection.corners).astype(int)
    cv2.polylines(overlay, [pts.reshape(-1, 1, 2)], True, color, thickness, 16)
    cx, cy = np.round(detection.center).astype(int)
    cv2.putText(overlay, f'id:{detection.tag_id}', (cx - 20, cy - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.8, color, 2, 16)
    return overlay
