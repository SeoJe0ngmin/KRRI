"""3. 자세 구하기 — 검출된 태그에서 도킹 값까지.

lateral / forward / heading 이 제어에 쓰는 값.
"""
from dataclasses import dataclass, field
from pathlib import Path

import cv2
import numpy as np

from ...config import (DEPTH_CHECK_MAX_Z, DEPTH_TOL_COEF, DEPTH_TOL_FLOOR_M,
                      MAX_REPROJ_RMS_PX, MIN_DECISION_MARGIN, MIN_TAG_PX,
                      RELIABLE_TILT_DEG)
from ...utils.util import r2rpy, invert_T, t2pr
from .image import (ASSUMED_HFOV_DEG, depth_at, from_video, intrinsics_from_hfov,
                    intrinsics_from_ref,
                    open_bag, open_realsense, to_gray)
from .detection_tag import (DEFAULT_QUAD_BLUR, STABLE_TAG_PX, detect, make_detector,
                     tag_pixel_size)


# 태그 네 모서리의 3D 좌표. detection.corners 와 같은 순서.
def _object_points(tag_size):
    s = tag_size / 2.0
    return np.array([[-s, -s, 0.], [s, -s, 0.], [s, s, 0.], [-s, s, 0.]], dtype=np.float64)


def pose_by_pnp(detection, intrinsics, tag_size):
    """OpenCV solvePnP 로 자세를 구함. detection_pose 의 대안."""
    obj = _object_points(tag_size)
    img = np.asarray(detection.corners, dtype=np.float64).reshape(-1, 1, 2)
    dist = np.array(intrinsics.distortion, dtype=np.float64) if intrinsics.distortion else np.zeros(5)

    ok, rvec, tvec = cv2.solvePnP(obj, img, intrinsics.K, dist,
                                  flags=cv2.SOLVEPNP_ITERATIVE)
    if not ok:
        return np.full((4, 4), np.nan), float("inf")

    R, _ = cv2.Rodrigues(rvec)
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.ravel()

    proj, _ = cv2.projectPoints(obj, rvec, tvec, intrinsics.K, dist)
    err = float(np.sqrt(((proj.reshape(-1, 2) - img.reshape(-1, 2)) ** 2).sum(axis=1)).mean())
    return T, err


def estimate_pose(detector, detection, intrinsics, tag_size, method="auto"):
    """태그 하나의 자세를 구함. (T, e0, e1) 3-튜플.

    detector.detection_pose() 와 같은 반환 형식. 실측 예:

        T  = [[-0.999  0.009  0.032  0.151]   좌상 3x3 = 회전, 우측 3x1 = 위치 [m]
              [ 0.005 -0.920  0.391  0.322]   카메라 기준 태그
              [ 0.033  0.391  0.920  1.387]
              [ 0.     0.     0.     1.   ]]
        e0 = e1 = 1.38e-06                    재투영 잔차

    AT2 는 다듬기 전/후 오차 둘을 줬지만 AT3 는 하나뿐이라 같은 값을 두 번 넣음.
    """
    if method == "pnp":
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return T, err, err

    R = getattr(detection, "pose_R", None)
    tvec = getattr(detection, "pose_t", None)
    if R is None or tvec is None:
        if method == "tag":
            raise ValueError("AT3 자세가 없다 — detect(intrinsics=..., tag_size=...) 로 부를 것")
        T, err = pose_by_pnp(detection, intrinsics, tag_size)
        return T, err, err

    T = np.eye(4)
    T[:3, :3] = np.asarray(R, dtype=float)
    T[:3, 3] = np.asarray(tvec, dtype=float).ravel()
    # AT3 의 pose_err 는 재투영 잔차. AT2 의 e0/e1(다듬기 전/후)과 달리 하나뿐이라
    err = float(getattr(detection, "pose_err", 0.0) or 0.0)

    if method == "auto" and not np.isfinite(T).all():
        T, e = pose_by_pnp(detection, intrinsics, tag_size)
        return T, e, e
    return T, err, err


def pose_to_xyzrpy(T, unit='deg'):
    """4x4 -> 읽을 수 있는 형태. 사람이 보기 위한 것이지 제어용이 아님.

        {'x': 0.151, 'y': 0.322, 'z': 1.387,        카메라 기준 태그 위치 [m]
         'roll': 23.02, 'pitch': -1.87, 'yaw': 179.73,   [도]
         'distance': 1.432}                          원점 거리 [m]

    roll/pitch/yaw 는 분해 순서에 따라 값이 달라지고 짐벌락에서 튐.
    제어에는 docking_state() 를 쓸 것. yaw 는 지게차 방향이 아니라
    카메라 장착 기울기를 잼(카메라 좌표계는 y 가 아래라 축 이름이 다름).
    """
    p, R = t2pr(T)
    rpy = r2rpy(R, unit=unit)
    return {"x": p[0], "y": p[1], "z": p[2],
            "roll": rpy[0], "pitch": rpy[1], "yaw": rpy[2],
            "distance": float(np.linalg.norm(p))}


def docking_state(T_camera_tag):
    """카메라 기준 태그 자세를 **도킹 제어가 쓸 형태**로 바꿈. 실측 예:

        {'lateral':  0.104,      태그 축에서 좌우로 얼마나 벗어났나 [m]
         'vertical': -0.248,     카메라높이 - 태그높이 [m]
         'forward':  1.407,      태그면까지 남은 거리 [m]
         'distance': 1.432,      직선 거리 [m]
         'approach_deg':  4.22,  태그에서 봤을 때 축에서 몇 도 비켜 있나 (위치)
         'heading_deg':   2.04,  지게차가 태그 축과 몇 도 틀어져 있나 (자세)
         'tilt_deg':     23.10,  태그가 화면에서 찌그러진 정도
         'reliable_angle': True} tilt >= 10도 라 각도를 믿어도 됨

    제어에 쓰는 것은 lateral / forward / heading_deg 셋.
    x,y,z 는 지게차가 고개만 돌려도 부호까지 바뀌지만 이 셋은 안 변함.
    approach 와 heading 은 독립임 — 축 위에 서서 고개를 돌리면
    approach=0 인데 heading!=0 임. 진입하려면 둘 다 0 이어야 함.
    """
    T_tag_cam = invert_T(np.asarray(T_camera_tag))
    p = T_tag_cam[:3, 3]
    R = T_tag_cam[:3, :3]

    lateral = float(p[0])
    vertical = float(p[1])
    forward = float(-p[2])                      # 태그 앞쪽이 양수가 되게 뒤집음
    distance = float(np.linalg.norm(p))

    # 내가 태그 정면축에서 몇 도 벗어난 위치에 있나
    approach = float(np.degrees(np.arctan2(abs(lateral), abs(forward))))

    # 카메라 광축(+z)이 태그 좌표계에서 어디를 향하나.
    fwd = R @ np.array([0.0, 0.0, 1.0])
    heading = float(np.degrees(np.arctan2(fwd[0], fwd[2])))

    # 각도를 믿어도 되는지는 **태그가 화면에서 얼마나 찌그러져 보이나(tilt)** 로 정함.
    tilt = tag_tilt_deg(T_camera_tag)
    return {"lateral": lateral, "vertical": vertical, "forward": forward,
            "distance": distance,
            "approach_deg": approach, "heading_deg": heading,
            "tilt_deg": tilt,
            "reliable_angle": tilt >= RELIABLE_TILT_DEG}


def tag_tilt_deg(T_camera_tag):
    """태그면이 카메라를 정면으로 마주보는 정도 [도]. float 하나.

    0 이면 정면(각도를 못 믿음), 클수록 비스듬(각도가 정확). 실측 예 23.10.
    3m/25cm 태그에서 tilt 2도면 좌우 변 길이차가 0.3px 라 노이즈에 묻힘.
    그래서 RELIABLE_TILT_DEG = 10.
    """
    n = np.asarray(T_camera_tag)[:3, :3] @ np.array([0.0, 0.0, 1.0])
    return float(np.degrees(np.arccos(min(1.0, abs(n[2])))))


# ===========================================================================


# ---------------------------------------------------------------------------

# 재투영 RMS 상한 [px].

# decision_margin 하한.

# 각도를 믿을 수 있는 최소 기울기 [도]. tag_tilt_deg 의 docstring 과 같은 값이고

# depth 교차검증 허용 오차 [m].


def _sample_depth_m(detection, depth, patch, depth_scale):
    """태그 중심 주변 patch x patch 의 depth 중앙값 [m]. 못 재면 None."""
    if depth is None:
        return None
    u, v = np.asarray(detection.corners, dtype=np.float64).mean(axis=0)
    return depth_at(depth, u, v, patch=patch, scale=depth_scale)


def depth_cross_check(detection, depth, T_camera_tag, patch=5, depth_scale=None):
    """태그 자세의 z 와 depth 센서의 z 를 대조함. 원리가 다른 두 측정이라
    어긋나면 tag_size 나 내부파라미터가 틀렸다는 신호.

        {'z_pose':   1.388,   자세로 잰 거리 [m]
         'z_depth':  1.332,   depth 센서로 잰 거리 [m]
         'diff_m':  -0.056,   차이 [m]
         'diff_pct': -4.06,   차이 [%]
         'tol_m':    0.096,   허용치. max(0.02, 0.05*z^2) 로 거리에 따라 커짐
         'in_range': True,    z 가 판정 가능 거리(1.5m) 안인가
         'agree':    True}    |diff_m| <= tol_m

    z 가 DEPTH_CHECK_MAX_Z(1.5m) 를 넘으면 허용치가 너무 헐거워져 판정을 건너뜀.
    """
    z_pose = float(np.asarray(T_camera_tag)[2, 3])
    tol = max(DEPTH_TOL_FLOOR_M, DEPTH_TOL_COEF * z_pose * z_pose)
    out = {"z_pose": z_pose, "z_depth": None, "diff_m": None, "diff_pct": None,
           "tol_m": float(tol), "in_range": bool(0.0 < z_pose <= DEPTH_CHECK_MAX_Z),
           "agree": False}

    z_depth = _sample_depth_m(detection, depth, patch, depth_scale)
    if z_depth is None or not np.isfinite(z_pose) or z_pose <= 0:
        return out

    diff = z_depth - z_pose
    out["z_depth"] = float(z_depth)
    out["diff_m"] = float(diff)
    out["diff_pct"] = float(100.0 * diff / z_pose)
    out["agree"] = bool(abs(diff) <= tol)
    return out


def pose_quality(detector, detection, intrinsics, tag_size, T_camera_tag, method="auto"):
    """이 프레임의 자세를 믿어도 되는지 한 번에 판정함. 실측 예:

        {'reproj_rms_px': 1.4e-06,   구한 자세로 모서리를 되찍어 본 오차 [px]
         'reproj_err':    1.4e-06,   같은 값 (단위는 reproj_units)
         'reproj_units': 'px',
         'tag_px':      131.71,      태그 한 변의 화면상 길이 [px]
         'tilt_deg':     23.10,      찌그러진 정도
         'reliable_angle': True,     각도를 믿어도 되나
         'decision_margin': 43.44,   검출기 판정 여유
         'hamming': 0,               고쳐낸 비트 수
         'ok': True,                 아래 reasons 가 비었나
         'reasons': []}              걸린 항목들. ok=False 면 여기에 이유가 옴

    depth 를 넘기면 'depth' 키가 더 붙음(depth_cross_check 결과).
    거르는 기준: tag_px >= 20, reproj <= 2.0px, margin >= 20, hamming == 0.
    """
    T = np.asarray(T_camera_tag, dtype=np.float64)

    if method == "pnp":
        _, err = pose_by_pnp(detection, intrinsics, tag_size)
        e1, squared = err, False
    else:
        pe = getattr(detection, "pose_err", None)
        if pe is None:
            _, err = pose_by_pnp(detection, intrinsics, tag_size)
            e1, squared = err, False
        else:
            e1, squared = float(pe), False

    e1 = float(e1)
    rms = float(np.sqrt(e1)) if (squared and e1 >= 0) else e1

    tag_px = tag_pixel_size(detection)
    tilt = tag_tilt_deg(T)
    margin = float(getattr(detection, "decision_margin", 0.0))
    hamming = int(getattr(detection, "hamming", 0))

    reasons = []
    if not np.isfinite(T).all():
        reasons.append("pose_nan")
    if tag_px < STABLE_TAG_PX:
        reasons.append(f"tag_px<{STABLE_TAG_PX:g}")
    if not np.isfinite(rms) or rms > MAX_REPROJ_RMS_PX:
        reasons.append(f"reproj>{MAX_REPROJ_RMS_PX:g}px")
    if margin < MIN_DECISION_MARGIN:
        reasons.append(f"margin<{MIN_DECISION_MARGIN:g}")
    if hamming != 0:
        reasons.append("hamming!=0")

    return {"reproj_err": e1,
            "reproj_units": "px^2" if squared else "px",
            "reproj_rms_px": rms,
            "tag_px": tag_px,
            "tilt_deg": tilt,
            "reliable_angle": bool(tilt >= RELIABLE_TILT_DEG),
            "decision_margin": margin,
            "hamming": hamming,
            "ok": not reasons,
            "reasons": reasons}

@dataclass
class Result:
    """한 프레임에서 나온 것 전부. TagPipeline 이 프레임마다 하나씩 만듦."""
    index: int
    timestamp: float
    image: object
    detections: list = field(default_factory=list)
    poses: dict = field(default_factory=dict)
    docking: dict = field(default_factory=dict)
    quality: dict = field(default_factory=dict)
    intrinsics: object = None
    errors: dict = field(default_factory=dict)

    @property
    def tag_ids(self):
        return [int(d.tag_id) for d in self.detections]

    def __len__(self):
        return len(self.detections)

    def primary(self, tag_id=None):
        """이 프레임에서 **믿고 쓸 태그 하나**를 골라 한 묶음으로 돌려줌."""
        usable = [d for d in self.detections if int(d.tag_id) in self.poses]
        if not usable:
            return None
        if tag_id is not None:
            usable = [d for d in usable if int(d.tag_id) == int(tag_id)]
            if not usable:
                return None
            det = usable[0]
        else:
            det = max(usable, key=tag_pixel_size)
        tid = int(det.tag_id)
        return {"tag_id": tid, "detection": det, "T": self.poses[tid],
                "docking": self.docking.get(tid), "quality": self.quality.get(tid)}

_PIPE_KEYS = ("detector", "families", "quad_blur", "method", "min_margin",
              "max_hamming", "gray_channel",
              "hfov", "quality", "depth_check", "label", "origin")

class TagPipeline:
    """소스 한 개 + 검출기 한 개를 들고, 프레임마다 Result 를 뱉음."""

    def __init__(self, frames=None, intrinsics=None, tag_size=None, detector=None,
                 families="tag36h11", quad_blur=DEFAULT_QUAD_BLUR, method="auto",
                 min_margin=0.0, max_hamming=0, gray_channel=None,
                 hfov=None, quality=True, depth_check=True,
                 label="", origin="", close=None):
        """Args:
        tag_size: **필수. 기본값을 두지 않았음.**
            이 값이 틀리면 거리 전체가 그 비율만큼 조용히 틀어짐(화면은
        """
        if tag_size is None:
            raise ValueError(
                "tag_size 를 반드시 줘야 한다 [m]. 거리가 이 값에 정비례하므로 "
                "기본값을 두면 틀린 거리가 조용히 나온다. 검은 테두리 바깥까지 잰 한 변.")
        self.frames = frames
        self.intr = intrinsics
        self.tag_size = float(tag_size)
        self.detector = detector if detector is not None else make_detector(
            families=families, quad_blur=quad_blur)
        self.method = method
        self.min_margin = float(min_margin)
        self.max_hamming = int(max_hamming)
        self.gray_channel = gray_channel
        self.hfov = None if hfov is None else float(hfov)
        self.quality = bool(quality)
        self.depth_check = bool(depth_check)
        self.label = label
        self.origin = origin or ("given" if intrinsics is not None else "")
        self.intrinsics_assumed = False
        self._close = close
        self._n = 0

    @staticmethod
    def _split_kw(kw):
        """kw 를 (파이프라인용, 소스용) 으로 가름. 이름이 겹치는 인자는 없음."""
        pipe = {k: kw.pop(k) for k in list(kw) if k in _PIPE_KEYS}
        return pipe, kw

    @classmethod
    def from_realsense(cls, tag_size, stream="color", **kw):
        """실물 D435i 를 열어 파이프라인을 만듦. 남는 인자는 open_realsense 로 감."""
        pipe_kw, open_kw = cls._split_kw(kw)
        frames, intr = open_realsense(stream=stream, **open_kw)
        pipe_kw.setdefault("label", "realsense/%s" % stream)
        pipe_kw.setdefault("origin", "factory (RealSense %s)" % stream)
        return cls(frames, intrinsics=intr, tag_size=tag_size,
                   close=frames.close, **pipe_kw)

    @classmethod
    def from_bag(cls, path, tag_size, **kw):
        """녹화한 .bag 을 재생함. 남는 인자는 open_bag 으로 감."""
        pipe_kw, open_kw = cls._split_kw(kw)
        frames, intr = open_bag(str(path), **open_kw)
        pipe_kw.setdefault("label", "bag (%s)" % Path(path).name)
        pipe_kw.setdefault("origin", "bag stream profile")
        return cls(frames, intrinsics=intr, tag_size=tag_size,
                   close=frames.close, **pipe_kw)

    @classmethod
    def from_video(cls, path, tag_size, intrinsics=None, hfov=None,
                   loop=False, **kw):
        """영상 파일. 카메라 값이 없으므로 intrinsics 를 주거나 화각을 가정함."""
        pipe_kw, _rest = cls._split_kw(kw)
        if _rest:
            raise TypeError("모르는 인자: %s" % ", ".join(sorted(_rest)))
        frames, _none = from_video(path, loop=loop)
        pipe_kw.setdefault("label", "video (%s)" % Path(path).name)
        pipe_kw.setdefault("origin", "given" if intrinsics is not None else "")
        return cls(frames, intrinsics=intrinsics, tag_size=tag_size, hfov=hfov,
                   close=frames.close, **pipe_kw)

    def intrinsics_for(self, shape):
        """카메라 값을 확정함. 없으면 화면 크기 + 화각 가정으로 지어냄."""
        if self.intr is None:
            self.intr = intrinsics_from_hfov(shape, self.hfov)
            self.intrinsics_assumed = True
            self.origin = ("ASSUMED from D435i ref" if self.hfov is None
                           else "ASSUMED from hfov=%.0fdeg" % self.hfov)
        return self.intr

    def process(self, img, index=None, timestamp=None):
        """이미지 한 장 -> Result. 어떤 실패도 밖으로 새지 않음."""
        src = img                 
        gray = to_gray(img, channel=self.gray_channel)
        intr = self.intrinsics_for(np.asarray(img).shape)

        if index is None:
            index = self._n
        self._n = index + 1

        res = Result(index=int(index),
                     timestamp=float(timestamp) if timestamp is not None else 0.0,
                     image=img, intrinsics=intr)

        try:
            res.detections = detect(self.detector, gray,
                                    intrinsics=intr, tag_size=self.tag_size,
                                    min_margin=self.min_margin,
                                    max_hamming=self.max_hamming)
        except Exception as exc:

            res.errors["detect"] = "%s: %s" % (type(exc).__name__, exc)
            return res

        depth = getattr(src, "depth", None)
        depth_scale = getattr(src, "depth_scale", 0.0) or None

        for d in res.detections:
            tid = int(d.tag_id)
            try:
                T, _e0, _e1 = estimate_pose(self.detector, d, intr, self.tag_size,
                                            method=self.method)
            except Exception as exc:
                res.errors[tid] = "estimate_pose %s: %s" % (type(exc).__name__, exc)
                continue
            T = np.asarray(T, dtype=np.float64)
            if T.shape != (4, 4) or not np.isfinite(T).all():
                res.errors[tid] = "non-finite pose (NaN)"
                continue

            res.poses[tid] = T
            res.docking[tid] = docking_state(T)
            if self.quality:
                q = pose_quality(self.detector, d, intr, self.tag_size, T,
                                 method=self.method)
                if self.depth_check and depth is not None:
                    q["depth"] = depth_cross_check(d, src, T, depth_scale=depth_scale)
                res.quality[tid] = q
        return res

    def __iter__(self):
        """소스를 끝까지 돌며 Result 를 뱉음. 3-튜플 계약을 그대로 소비함."""
        if self.frames is None:
            raise RuntimeError("소스 없이 만든 파이프라인이다 — process(img) 로 한 장씩 넣어라.")
        for i, ts, img in self.frames:
            yield self.process(img, index=i, timestamp=ts)

    @property
    def stats(self):
        """소스의 프레임 드롭 회계(FrameStats). 없는 소스면 None."""
        return getattr(self.frames, "stats", None)

    @property
    def ae_roi(self):
        """자동노출 ROI 객체(open_realsense(ae_roi=True) 일 때만). 없으면 None."""
        return getattr(self.frames, "ae_roi", None)

    def close(self):
        """소스를 닫고 검출기를 풂. 몇 번 불러도 안전함.

        검출기 참조만 놓음. 실제 해제는 make_detector 가 막아 둠 —
        pupil_apriltags 의 apriltag_detector_destroy 가 세그폴트를 냄.
        """
        c, self._close = self._close, None
        if c is not None:
            try:
                c()
            except Exception:
                pass
        self.detector = None

    def __enter__(self):
        return self

    def __exit__(self, *exc):
        self.close()
        return False


# ===========================================================================


def measure(results, tag_id=None, n=None, max_frames=None, require_ok=True):
    """정지 상태에서 여러 프레임을 모아 **대표값 하나**를 냄.

    한 프레임 값은 못 씀 — 태그·카메라를 고정해 둬도 실측으로 이만큼 흔들림:
        lateral +-9.0mm / heading +-0.45도 (1.4m, 1920x1080)
    코너 검출이 0.35px 떨리고, 그게 heading 을 흔들고, 다시 거리에 곱해져
    lateral 을 흔듦. n 프레임을 모으면 sqrt(n) 로 줌(30이면 1/5.5).

    중앙값을 씀. 평균과 달리 가끔 튀는 프레임에 안 끌려감.
    spread 는 **대표값의 표준오차**(표준편차/sqrt(n))다 — "이 값을 얼마나
    믿을 수 있나"이지 장면이 얼마나 흔들리나가 아님.

    반환:
        {'lateral': 0.104, 'forward': 1.407, 'heading_deg': 2.04,   대표값
         'tilt_deg': 23.1, 'z_optical': 1.388, 'distance': 1.432,
         'n': 30,                                                    쓴 프레임 수
         'spread': {'lateral': 0.0014, 'forward': 0.0003, ...},      표준오차
         'reliable_angle': True,        tilt >= RELIABLE_TILT_DEG
         'stable': True,                spread 가 임계 안
         'reasons': []}                 stable=False 면 왜인지
        태그를 못 봤으면 None.

    stable 이 False 면 **명령을 내지 말고 다시 재라.** 누가 지나갔거나
    조명이 깜빡였거나 아직 안 멈춘 것.
    """
    from ...config import (MEASURE_FRAMES, MEASURE_MAX_FRAMES,
                           STABLE_HEADING_DEG, STABLE_LATERAL_M)
    n = int(n or MEASURE_FRAMES)
    max_frames = int(max_frames or MEASURE_MAX_FRAMES)

    keys = ("lateral", "vertical", "forward", "distance",
            "approach_deg", "heading_deg", "tilt_deg")
    got = {k: [] for k in keys}
    got["z_optical"] = []
    seen = 0

    for res in results:
        seen += 1
        tid = tag_id
        if tid is None:
            tid = next(iter(res.docking), None)   
        if tid is None or tid not in res.docking:
            if seen >= max_frames:
                break
            continue
        if require_ok and not res.quality.get(tid, {}).get("ok", True):
            if seen >= max_frames:
                break
            continue
        d = res.docking[tid]
        for k in keys:
            got[k].append(float(d[k]))
        got["z_optical"].append(float(res.poses[tid][2, 3]))
        if len(got["lateral"]) >= n or seen >= max_frames:
            break

    m = len(got["lateral"])
    if m == 0:
        return None

    def med(v):
        s = sorted(v)
        return s[m // 2] if m % 2 else 0.5 * (s[m // 2 - 1] + s[m // 2])

    def stderr(v):
        if m < 2:
            return float("inf")
        mu = sum(v) / m
        var = sum((x - mu) ** 2 for x in v) / (m - 1)
        return (var ** 0.5) / (m ** 0.5)

    out = {k: med(v) for k, v in got.items()}
    out["n"] = m
    out["spread"] = {k: stderr(v) for k, v in got.items()}
    out["reliable_angle"] = bool(out["tilt_deg"] >= RELIABLE_TILT_DEG)

    reasons = []
    if m < n:
        reasons.append("프레임 부족 %d/%d" % (m, n))
    if out["spread"]["lateral"] > STABLE_LATERAL_M:
        reasons.append("lateral 흔들림 %.1fmm" % (out["spread"]["lateral"] * 1000))
    if out["spread"]["heading_deg"] > STABLE_HEADING_DEG:
        reasons.append("heading 흔들림 %.2f도" % out["spread"]["heading_deg"])
    out["stable"] = not reasons
    out["reasons"] = reasons
    return out
