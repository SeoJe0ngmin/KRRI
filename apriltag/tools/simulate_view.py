"""시뮬레이터의 **보여주기 절반**. 숫자는 tools/simulate.py 가 만든다."""
import argparse
import sys
from pathlib import Path

import cv2
import numpy as np

import matplotlib
matplotlib.use("Agg")                 # 창은 cv2 가 띄운다. 여기는 파일/배열만 만든다.
import matplotlib.pyplot as plt       # noqa: E402
from matplotlib.patches import Arc, Polygon, Rectangle   # noqa: E402

ROOT = Path(__file__).resolve().parent.parent
sys.path.insert(0, str(ROOT))
sys.path.insert(0, str(Path(__file__).resolve().parent))   # simulate.py 를 옆에서 찾기 위해

from src.config import TAG_SIZE_M as DEFAULT_TAG_SIZE  # noqa: E402
from src.config import CAM_HEIGHT_M, TAG_HEIGHT_M      # noqa: E402
from src.models import (CameraIntrinsics,          # noqa: E402
                                 make_detector, detect, to_gray,
                                 estimate_pose, docking_state, pose_to_xyzrpy,
                                 tag_tilt_deg, tag_pixel_size, pose_quality,
                                 MIN_TAG_PX, STABLE_TAG_PX, RELIABLE_TILT_DEG,
                                 DEFAULT_QUAD_BLUR)
from src.models.detection.detection_pose import _object_points  # 정답 생성용 (private)
from src.utils.drawing import draw_axes, draw_cube, draw_corners    # noqa: E402
from src.utils.util import pr2t, invert_T                           # noqa: E402

OUTDIR = ROOT / "work_dirs" / "simulate"

# D435i 컬러 공장값 — 이 개체에서 실측한 값이다. 왜곡계수는 0 이라(실측 확인)
RESOLUTIONS = {
    "1920x1080": (1359.2, 1359.0, 956.9, 571.3),
    "1280x720":  (906.1,  906.0,  637.9, 380.9),
    "640x480":   (604.1,  604.0,  318.6, 253.9),
}
DEFAULT_RES = "1280x720"

#: 배치 한 벌. 전부 **바닥 기준**으로 잰다 — 실제 방을 재는 방식 그대로다.
DEFAULT_PLACE = {"tag_height": TAG_HEIGHT_M, "cam_height": CAM_HEIGHT_M,
                 "distance": 3.00, "lateral": -1.03, "heading": 20.0}

# 색. 태그계열=붉은색, 카메라/지게차계열=파란색, 치수선=회색.
C_TAG = "#c1352b"
C_TAG_F = "#f2c9c5"
C_CAM = "#1f5fbf"
C_CAM_F = "#cfe0f7"
C_DIM = "#5a5a5a"
C_AXIS = "#8a8a8a"
C_FLOOR = "#3a3a3a"
C_BAD = "#d94a3d"
C_OK = "#2e8b57"

_UNSET = object()
_SIM = _UNSET
_WARNED = set()


# ===========================================================================

def _sim():
    """tools/simulate.py 를 늦게 불러온다."""
    global _SIM
    if _SIM is not _UNSET:
        return _SIM
    me = sys.modules.get(__name__)
    for name in ("simulate", "tools.simulate", "__main__"):
        m = sys.modules.get(name)
        # 이 파일 자신을 붙잡으면 안 된다. simulate_view.py 를 __main__ 으로 돌리면
        if m is None or m is me or getattr(m, "__file__", "") == __file__:
            continue
        if hasattr(m, "placement_to_T") and hasattr(m, "render"):
            _SIM = m
            return _SIM
    for name in ("simulate", "tools.simulate"):
        try:
            _SIM = __import__(name, fromlist=["placement_to_T"])
            return _SIM
        except Exception:
            continue
    _SIM = None
    return _SIM


def _from_sim(name):
    """simulate.py 의 함수를 꺼낸다. 없으면 None 을 주고 **한 번 경고를 찍는다**."""
    fn = getattr(_sim(), name, None)
    if fn is None and name not in _WARNED:
        _WARNED.add(name)
        print("[simulate_view] tools/simulate.py 의 %s() 를 못 찾았다 — "
              "내장 참조구현을 쓴다." % name)
    return fn


def placement_to_T(tag_height, cam_height, distance, lateral, heading):
    """배치 5개 값 -> T_camera_tag. simulate.py 것이 있으면 그것을 쓴다."""
    fn = _from_sim("placement_to_T")
    if fn is not None:
        return np.asarray(fn(tag_height, cam_height, distance, lateral, heading),
                          dtype=float)
    return _ref_placement_to_T(tag_height, cam_height, distance, lateral, heading)


def render(T, intr, tag_size, tag_id, width, height):
    """자세 -> 합성 BGR 한 장. simulate.py 것이 있으면 그것을 쓴다."""
    fn = _from_sim("render")
    if fn is not None:
        img = np.asarray(fn(T, intr, tag_size, tag_id, width, height))
        return img if img.ndim == 3 else cv2.cvtColor(img, cv2.COLOR_GRAY2BGR)
    return _ref_render(T, intr, tag_size, tag_id, width, height)


def _ref_placement_to_T(tag_height, cam_height, distance, lateral, heading):
    """참조구현. **부호가 전부 여기서 결정된다.**"""
    h = np.deg2rad(float(heading))
    c, s = np.cos(h), np.sin(h)
    R_tag_cam = np.array([[-c, 0.0, s],       # 카메라 +x (오른쪽) 를 태그축으로
                          [0.0, -1.0, 0.0],   # 카메라 +y (아래)   -> 수평 카메라
                          [s, 0.0, c]])       # 카메라 +z (광축)   -> heading
    p_tag_cam = np.array([float(lateral),
                          float(cam_height) - float(tag_height),
                          -float(distance)])
    return invert_T(pr2t(p_tag_cam, R_tag_cam))


def _ref_render(T, intr, tag_size, tag_id, width, height, px=600, pad=150):
    """참조구현. 핀홀 투영으로 tag36h11 텍스처를 한 장 붙인다."""
    dic = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_APRILTAG_36h11)
    core = cv2.aruco.generateImageMarker(dic, int(tag_id), int(px))
    full = cv2.copyMakeBorder(core, pad, pad, pad, pad, cv2.BORDER_CONSTANT, value=255)
    src = np.float32([[pad, pad], [pad + px, pad], [pad + px, pad + px], [pad, pad + px]])

    s = float(tag_size) / 2.0
    obj = np.float32([[s, s, 0], [-s, s, 0], [-s, -s, 0], [s, -s, 0]])   # TL,TR,BR,BL
    T = np.asarray(T, dtype=float)
    pts = (T[:3, :3] @ obj.T).T + T[:3, 3]
    if (pts[:, 2] <= 0).any():
        raise ValueError("태그가 카메라 뒤로 갔다 — 배치를 다시 볼 것")
    uv = (intr.K @ pts.T).T
    uv = (uv[:, :2] / uv[:, 2:]).astype(np.float32)

    M = cv2.getPerspectiveTransform(src, uv)
    gray = cv2.warpPerspective(full, M, (int(width), int(height)), borderValue=255)
    return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)


# ===========================================================================

def intrinsics_for(res=DEFAULT_RES):
    """해상도 이름 -> CameraIntrinsics. 왜곡은 비어 있다(이 개체는 0 이다)."""
    if res not in RESOLUTIONS:
        raise SystemExit("모르는 해상도: %s (가능: %s)" % (res, ", ".join(RESOLUTIONS)))
    fx, fy, cx, cy = RESOLUTIONS[res]
    w, h = (int(v) for v in res.split("x"))
    return CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h)


def as_place(place=None, **kw):
    """배치 dict 를 만든다. 안 준 칸은 기본 배치에서 채운다."""
    out = dict(DEFAULT_PLACE)
    if place:
        out.update({k: v for k, v in place.items() if k in DEFAULT_PLACE})
    out.update({k: v for k, v in kw.items() if k in DEFAULT_PLACE and v is not None})
    return {k: float(v) for k, v in out.items()}


def truth_of(place, tag_size=DEFAULT_TAG_SIZE):
    """배치에서 정답을 뽑는다. **직접 계산하지 않고 파이프라인 함수에 물어본다.**"""
    T = placement_to_T(**place)
    st = docking_state(T)
    v = pose_to_xyzrpy(T)
    return {"T": T, "docking": st, "xyzrpy": v,
            "tilt": tag_tilt_deg(T),
            "z_optical": float(T[2, 3]),
            "tag_size": float(tag_size)}


def visibility(T, intr, tag_size):
    """네 모서리가 정말 화면 안에 있고 카메라 앞에 있는지 본다."""
    T = np.asarray(T, dtype=float)
    obj = _object_points(float(tag_size))
    cam = (T[:3, :3] @ obj.T).T + T[:3, 3]
    behind = bool((cam[:, 2] <= 0).any())
    uv = np.full((4, 2), np.nan)
    if not behind:
        p = (intr.K @ cam.T).T
        uv = p[:, :2] / p[:, 2:]
    w = intr.width or 0
    h = intr.height or 0
    outside = bool(behind or (w and h and (
        (uv[:, 0] < 0).any() or (uv[:, 0] >= w).any() or
        (uv[:, 1] < 0).any() or (uv[:, 1] >= h).any())))
    edges = np.linalg.norm(uv - np.roll(uv, -1, axis=0), axis=1)
    return {"ok": not outside, "behind": behind, "outside": outside,
            "uv": uv, "tag_px": float(edges.mean()) if not behind else float("nan")}


def degrade(img, blur_px=0.0, noise=0.0, seed=0):
    """합성 영상을 일부러 나쁘게 만든다. 모션블러와 센서잡음."""
    out = np.asarray(img)
    k = int(round(float(blur_px)))
    if k >= 2:
        kern = np.zeros((1, k), np.float32)
        kern[0, :] = 1.0 / k
        out = cv2.filter2D(out, -1, kern, borderType=cv2.BORDER_REPLICATE)
    if float(noise) > 0:
        rng = np.random.default_rng(int(seed))
        f = out.astype(np.float32) + rng.normal(0.0, float(noise), out.shape)
        out = np.clip(f, 0, 255).astype(np.uint8)
    return out


# 잡음을 넣을 때 검출기에 반드시 넣어야 하는 quad_blur 하한.
NOISE_QUAD_BLUR_LEVELS = 3.0
NOISE_QUAD_BLUR = 2.0
_QB_WARNED = [False]


def quad_blur_for(noise, quad_blur=None):
    """잡음 수준에 맞는 quad_blur 를 고른다. 사용자가 준 값이 위험하면 올리고 말한다."""
    noise = float(noise)
    if quad_blur is None:
        return DEFAULT_QUAD_BLUR if noise <= NOISE_QUAD_BLUR_LEVELS else NOISE_QUAD_BLUR
    quad_blur = float(quad_blur)
    if noise > NOISE_QUAD_BLUR_LEVELS and quad_blur < NOISE_QUAD_BLUR:
        if not _QB_WARNED[0]:
            _QB_WARNED[0] = True
            print("[simulate_view] 잡음 %.1f 에 quad_blur=%.1f 은 AT2 검출기가 "
                  "세그폴트로 죽는 조합이다. %.1f 로 올려서 돈다."
                  % (noise, quad_blur, NOISE_QUAD_BLUR))
        return NOISE_QUAD_BLUR
    return quad_blur


def measure(place, intr, tag_size=DEFAULT_TAG_SIZE, tag_id=0, method="auto",
            blur_px=0.0, noise=0.0, seed=0, detector=None, quad_blur=None):
    """한 배치를 렌더 -> 검출 -> 추정까지 돌려 정답/추정/오차를 낸다."""
    place = as_place(place)
    truth = truth_of(place, tag_size)
    vis = visibility(truth["T"], intr, tag_size)

    qb = quad_blur_for(noise, quad_blur)
    out = {"place": place, "truth": truth, "vis": vis, "tag_size": float(tag_size),
           "tag_id": int(tag_id), "method": method, "intr": intr,
           "blur_px": float(blur_px), "noise": float(noise), "quad_blur": qb,
           "image": None, "detection": None, "T_est": None,
           "est": None, "err": None, "quality": {}, "ok": False, "reason": ""}

    if not vis["ok"]:
        out["reason"] = "behind camera" if vis["behind"] else "tag outside frame"
        return out

    # simulate.render() 는 여기 vis 가 통과시킨 배치도 거절할 수 있다. 이 파일의
    try:
        img = render(truth["T"], intr, tag_size, tag_id, intr.width, intr.height)
    except Exception as exc:                      # simulate.OutOfView 를 이름으로 못 잡는다
        if type(exc).__name__ != "OutOfView":     # (참조구현으로 떨어지면 ValueError 다)
            if not isinstance(exc, ValueError):
                raise
        # reason 은 **그림에 찍힌다**. 이 파일의 규약대로 ASCII 로만 적는다 —
        print("[simulate_view] not renderable: %s" % exc)
        out["reason"] = "not renderable (tag or its quiet zone is clipped " \
                        "by the frame edge; see console)"
        return out
    img = degrade(img, blur_px, noise, seed)
    out["image"] = img

    det_obj = detector if detector is not None else make_detector(quad_blur=qb)
    dets = detect(det_obj, to_gray(img))
    if not dets:
        out["reason"] = "no detection"
        return out
    want = [d for d in dets if int(d.tag_id) == int(tag_id)]
    d = (want or dets)[0] if len(dets) == 1 else max(want or dets, key=tag_pixel_size)
    out["detection"] = d

    T, _e0, _e1 = estimate_pose(det_obj, d, intr, tag_size, method=method)
    if not np.isfinite(np.asarray(T)).all():
        out["reason"] = "pose NaN (%s)" % method
        return out
    out["T_est"] = np.asarray(T, dtype=float)
    out["quality"] = pose_quality(det_obj, d, intr, tag_size, T, method=method)

    st, v = docking_state(T), pose_to_xyzrpy(T)
    out["est"] = {"lateral": st["lateral"], "forward": st["forward"],
                  "vertical": st["vertical"], "distance": st["distance"],
                  "heading": st["heading_deg"], "approach": st["approach_deg"],
                  "tilt": tag_tilt_deg(T), "z_optical": float(v["z"]),
                  "rel_approach": bool(st["reliable_angle"]),
                  "rel_tilt": bool(out["quality"].get("reliable_angle", False)),
                  "tag_px": float(out["quality"].get("tag_px", float("nan")))}
    t = truth_true_values(truth)
    out["err"] = {k: out["est"][k] - t[k] for k in ERR_KEYS}
    out["ok"] = True
    return out


#: 정답과 추정을 나란히 비교할 값. (키, 표시이름, 단위, 오차단위, 오차환산)
ROWS = [
    ("lateral",   "lateral",   "m",   "mm",  1000.0),
    ("forward",   "forward",   "m",   "mm",  1000.0),
    ("vertical",  "vertical",  "m",   "mm",  1000.0),
    ("distance",  "distance",  "m",   "mm",  1000.0),
    ("z_optical", "z(optical)", "m",  "mm",  1000.0),
    ("heading",   "heading",   "deg", "deg", 1.0),
    ("approach",  "approach",  "deg", "deg", 1.0),
    ("tilt",      "tag tilt",  "deg", "deg", 1.0),
]
ERR_KEYS = [r[0] for r in ROWS]
LEN_KEYS = [r[0] for r in ROWS if r[3] == "mm"]
ANG_KEYS = [r[0] for r in ROWS if r[3] == "deg"]


def truth_true_values(truth):
    """정답 dict 를 ROWS 키에 맞춘 평평한 dict 로 편다."""
    st = truth["docking"]
    return {"lateral": st["lateral"], "forward": st["forward"],
            "vertical": st["vertical"], "distance": st["distance"],
            "heading": st["heading_deg"], "approach": st["approach_deg"],
            "tilt": truth["tilt"], "z_optical": truth["z_optical"]}


# ===========================================================================

def _forklift_plan(ax, cx, cy, heading_deg, length=1.55, width=0.90, nose=0.18):
    """평면도에 지게차 몸통을 그린다. 카메라는 앞코에 달려 있다고 본다."""
    h = np.deg2rad(heading_deg)
    u = np.array([np.sin(h), -np.cos(h)])      # 광축(진행방향)을 평면에 내린 것
    lft = np.array([np.cos(h), np.sin(h)])     # 운전자의 왼쪽
    c = np.array([cx, cy])
    body = [c + nose * u + width / 2 * lft,
            c + nose * u - width / 2 * lft,
            c - (length - nose) * u - width / 2 * lft,
            c - (length - nose) * u + width / 2 * lft]
    ax.add_patch(Polygon(body, closed=True, facecolor=C_CAM_F, edgecolor=C_CAM,
                         lw=1.4, zorder=3))
    # 포크 두 개 — 어느 쪽이 앞인지 한눈에 보이게
    for b in (0.22, -0.22):
        p0 = c + b * lft
        p1 = p0 + 0.55 * u
        ax.plot([p0[0], p1[0]], [p0[1], p1[1]], color=C_CAM, lw=2.0, zorder=3)
    ax.plot([cx], [cy], marker="o", ms=7, color=C_CAM, zorder=5)
    return u, lft


def layout_figure(place=None, tag_size=DEFAULT_TAG_SIZE, intr=None, title=None,
                  truth=None, **kw):
    """배치를 **눈으로** 확인하는 그림. 평면도 + 입면도 + 정답 숫자."""
    place = as_place(place, **kw)
    truth = truth or truth_of(place, tag_size)
    st = truth["docking"]
    S = float(tag_size)
    L = place["lateral"]
    D = place["distance"]
    Hd = place["heading"]
    th = place["tag_height"]
    ch = place["cam_height"]

    fig = plt.figure(figsize=(13.5, 8.6), dpi=110)
    gs = fig.add_gridspec(2, 2, height_ratios=[3.0, 1.15], hspace=0.28, wspace=0.18,
                          left=0.06, right=0.98, top=0.90, bottom=0.05)
    axp = fig.add_subplot(gs[0, 0])
    axs = fig.add_subplot(gs[0, 1])
    axt = fig.add_subplot(gs[1, :])

    # ---------------------------------------------------------------- 평면도
    span = max(abs(L) + 1.2, S * 3, 1.4)
    axp.axhline(0.0, color=C_AXIS, lw=1.0, zorder=1)
    axp.text(span * 0.96, 0.06, "tag plane / dock face", color=C_AXIS,
             ha="right", va="bottom", fontsize=8)
    # 태그 자체 (평면에서는 폭만 보인다)
    axp.plot([-S / 2, S / 2], [0, 0], color=C_TAG, lw=6, solid_capstyle="butt", zorder=4)
    # 태그 정면축 (앞으로 뻗는 축) 과 태그 +x
    axp.plot([0, 0], [0, D * 1.18], color=C_TAG, lw=1.2, ls="--", zorder=2)
    axp.annotate("", xy=(0, 0.55), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=C_TAG, lw=1.6))
    axp.text(0.03, 0.58, "tag +z is BEHIND the tag;\nthis arrow is 'forward' (-z)",
             color=C_TAG, fontsize=8, va="bottom")
    axp.annotate("", xy=(S * 1.6, 0), xytext=(0, 0),
                 arrowprops=dict(arrowstyle="-|>", color=C_TAG, lw=1.4))
    axp.text(S * 1.7, -0.05, "tag +x  (driver's LEFT)", color=C_TAG, fontsize=8,
             va="top")

    # 카메라/지게차
    u, _lft = _forklift_plan(axp, L, D, Hd)
    # 수평화각 부채꼴
    if intr is not None and intr.fx:
        cxp = intr.cx if intr.cx else (intr.width or 0) / 2.0
        half_l = np.arctan2(cxp, intr.fx)
        half_r = np.arctan2((intr.width or 0) - cxp, intr.fx)
        rays = []
        for sgn, half in ((+1.0, half_l), (-1.0, half_r)):
            a = np.deg2rad(Hd) + sgn * half
            rays.append(np.array([L, D]) + (D + 1.0) * np.array([np.sin(a), -np.cos(a)]))
        axp.add_patch(Polygon([(L, D), rays[0], rays[1]], closed=True,
                              facecolor=C_CAM, alpha=0.07, edgecolor=C_CAM,
                              lw=0.8, ls=":", zorder=1))
        axp.text(L, D + 0.16, "HFOV %.0f deg" % np.degrees(half_l + half_r),
                 color=C_CAM, fontsize=8, ha="center")
    # heading 화살표
    axp.annotate("", xy=(L + 0.95 * u[0], D + 0.95 * u[1]), xytext=(L, D),
                 arrowprops=dict(arrowstyle="-|>", color=C_CAM, lw=2.2))
    # 시선 (카메라 -> 태그 중심)
    axp.plot([L, 0], [D, 0], color=C_DIM, lw=0.9, ls=":", zorder=2)

    # 치수선 — lateral / forward
    axp.annotate("", xy=(0, D), xytext=(L, D),
                 arrowprops=dict(arrowstyle="<->", color=C_DIM, lw=1.1))
    axp.text(L / 2.0, D + 0.10, "lateral %+.3f m" % st["lateral"], color=C_DIM,
             ha="center", fontsize=9)
    axp.plot([L, L], [0, D], color=C_DIM, lw=0.7, ls="--", zorder=1)
    axp.annotate("", xy=(L, D), xytext=(L, 0),
                 arrowprops=dict(arrowstyle="<->", color=C_DIM, lw=1.1))
    axp.text(L - 0.06, D / 2.0, "forward %.3f m" % st["forward"], color=C_DIM,
             rotation=90, ha="right", va="center", fontsize=9)

    # 각도 호 — approach(위치) 는 태그 원점에, heading(자세) 은 카메라에
    a_cam = np.degrees(np.arctan2(D, L))
    r = min(0.55, D * 0.35)
    axp.add_patch(Arc((0, 0), 2 * r, 2 * r, theta1=min(90.0, a_cam),
                      theta2=max(90.0, a_cam), color=C_DIM, lw=1.3))
    mid = np.deg2rad((90.0 + a_cam) / 2.0)
    axp.text(1.32 * r * np.cos(mid), 1.32 * r * np.sin(mid),
             "approach %.1f deg" % st["approach_deg"], color=C_DIM, fontsize=9,
             ha="center", va="center")
    a_u = np.degrees(np.arctan2(u[1], u[0]))
    r2 = 0.6
    axp.add_patch(Arc((L, D), 2 * r2, 2 * r2, theta1=min(270.0, a_u % 360.0),
                      theta2=max(270.0, a_u % 360.0), color=C_CAM, lw=1.3))
    mid2 = np.deg2rad((270.0 + (a_u % 360.0)) / 2.0)
    axp.text(L + 1.30 * r2 * np.cos(mid2), D + 1.30 * r2 * np.sin(mid2),
             "heading %+.1f deg" % st["heading_deg"], color=C_CAM, fontsize=9,
             ha="center", va="center")

    axp.set_title("PLAN  -  looking DOWN from above", fontsize=11, weight="bold")
    axp.set_xlabel("tag +x  [m]   (+ = driver's left)")
    axp.set_ylabel("forward from tag plane  [m]")
    axp.set_xlim(-span, span)
    axp.set_ylim(-0.75, D * 1.35 + 0.5)
    axp.set_aspect("equal", adjustable="box")
    axp.grid(alpha=0.18, lw=0.6)

    # ---------------------------------------------------------------- 입면도
    top = max(th, ch) + 0.9
    axs.axhline(0.0, color=C_FLOOR, lw=2.2, zorder=3)
    axs.fill_between([-0.55, D + 1.1], -0.28, 0.0, color=C_FLOOR, alpha=0.12, zorder=1)
    axs.text(D + 1.05, -0.20, "FLOOR", color=C_FLOOR, ha="right", fontsize=9)
    # 벽 + 태그 (수직 설치, 이 시뮬레이터에는 tag-roll 이 없다)
    axs.plot([0, 0], [0, top], color=C_AXIS, lw=2.0, zorder=2)
    axs.plot([0, 0], [th - S / 2, th + S / 2], color=C_TAG, lw=8,
             solid_capstyle="butt", zorder=4)
    axs.text(0.06, th + S / 2 + 0.06, "tag  %.0f cm, VERTICAL" % (S * 100),
             color=C_TAG, fontsize=9)
    axs.annotate("", xy=(0.5, th), xytext=(0, th),
                 arrowprops=dict(arrowstyle="-|>", color=C_TAG, lw=1.5))
    # 지게차 + 카메라 (수평 설치)
    axs.add_patch(Rectangle((D - 0.55, 0.0), 0.85, ch * 0.62, facecolor=C_CAM_F,
                            edgecolor=C_CAM, lw=1.3, zorder=3))
    axs.plot([D, D], [ch * 0.62, ch], color=C_CAM, lw=2.4, zorder=3)
    axs.plot([D], [ch], marker="o", ms=8, color=C_CAM, zorder=5)
    axs.annotate("", xy=(D - 0.7, ch), xytext=(D, ch),
                 arrowprops=dict(arrowstyle="-|>", color=C_CAM, lw=1.8))
    axs.text(D - 0.72, ch + 0.06, "optical axis, LEVEL", color=C_CAM, fontsize=9,
             ha="right")

    for x, hgt, lab, col in ((-0.30, th, "tag height %.3f m" % th, C_TAG),
                             (D + 0.62, ch, "cam height %.3f m" % ch, C_CAM)):
        axs.annotate("", xy=(x, hgt), xytext=(x, 0),
                     arrowprops=dict(arrowstyle="<->", color=col, lw=1.0))
        axs.text(x - 0.04 if x < 0 else x + 0.04, hgt / 2.0, lab, color=col,
                 rotation=90, fontsize=9, va="center",
                 ha="right" if x < 0 else "left")
    xv = D * 0.45
    axs.plot([0, D], [th, th], color=C_DIM, lw=0.7, ls="--", zorder=1)
    axs.plot([0, D], [ch, ch], color=C_DIM, lw=0.7, ls="--", zorder=1)
    axs.annotate("", xy=(xv, ch), xytext=(xv, th),
                 arrowprops=dict(arrowstyle="<->", color=C_DIM, lw=1.2))
    axs.text(xv + 0.06, (th + ch) / 2.0,
             "vertical %+.3f m\n(= cam - tag)" % st["vertical"], color=C_DIM,
             fontsize=9, va="center")
    axs.annotate("", xy=(D, -0.42), xytext=(0, -0.42),
                 arrowprops=dict(arrowstyle="<->", color=C_DIM, lw=1.1))
    axs.text(D / 2.0, -0.52, "forward %.3f m  (perpendicular to tag plane)" % st["forward"],
             color=C_DIM, ha="center", va="top", fontsize=9)

    axs.set_title("SIDE ELEVATION  -  lateral is out of the page", fontsize=11,
                  weight="bold")
    axs.set_xlabel("forward from tag plane  [m]")
    axs.set_ylabel("height above floor  [m]")
    axs.set_xlim(-0.62, D + 1.15)
    axs.set_ylim(-0.95, top + 0.15)
    axs.set_aspect("equal", adjustable="box")
    axs.grid(alpha=0.18, lw=0.6)

    # ---------------------------------------------------------------- 숫자판
    axt.axis("off")
    vis = visibility(truth["T"], intr, tag_size) if intr is not None else None
    v = truth["xyzrpy"]
    left = [
        "PLACEMENT (as measured in the room, floor as reference)",
        "  tag-size %.3f m   tag-height %.3f m   cam-height %.3f m" % (S, th, ch),
        "  distance %.3f m   lateral %+.3f m   heading %+.1f deg" % (D, L, Hd),
        "",
        "TRUTH from docking_state(T_camera_tag)  -  not hand-computed",
        "  lateral  %+8.3f m      forward  %+8.3f m" % (st["lateral"], st["forward"]),
        "  vertical %+8.3f m      distance %8.3f m  (3-D)" % (st["vertical"], st["distance"]),
        "  heading  %+8.1f deg    approach %8.1f deg (unsigned)"
        % (st["heading_deg"], st["approach_deg"]),
        "  tag tilt %8.1f deg     reliable_angle: approach>=10 %s | tilt>=10 %s"
        % (truth["tilt"], "YES" if st["reliable_angle"] else "no",
           "YES" if truth["tilt"] >= RELIABLE_TILT_DEG else "no"),
    ]
    right = [
        "THREE DISTANCES - never compare them to each other",
        "  docking.forward   %8.3f m   perpendicular to the tag plane" % st["forward"],
        "  T[2,3] optical Z  %8.3f m   what a depth reading would show" % truth["z_optical"],
        "  docking.distance  %8.3f m   |camera position| in tag frame" % st["distance"],
        "",
        "T_camera_tag  x %+.3f  y %+.3f  z %+.3f  [m]" % (v["x"], v["y"], v["z"]),
        "  rpy %+.1f / %+.1f / %+.1f deg" % (v["roll"], v["pitch"], v["yaw"]),
        "  pitch = -heading and yaw is pinned at 180 - an artifact of the ZYX",
        "  decomposition. Read heading from docking_state, never from rpy.",
    ]
    if vis is not None:
        right.append("")
        right.append("VISIBILITY  tag %.0f px  %s"
                     % (vis["tag_px"],
                        "in frame" if vis["ok"] else
                        ("BEHIND CAMERA" if vis["behind"] else "OUTSIDE FRAME")))
    axt.text(0.005, 0.98, "\n".join(left), family="monospace", fontsize=9,
             va="top", ha="left", transform=axt.transAxes)
    axt.text(0.52, 0.98, "\n".join(right), family="monospace", fontsize=9,
             va="top", ha="left", transform=axt.transAxes)

    fig.suptitle(title or ("SIMULATED PLACEMENT   tag %.0fcm @ %.2fm   "
                           "cam @ %.2fm   d=%.2fm  lat=%+.2fm  hdg=%+.1fdeg"
                           % (S * 100, th, ch, D, L, Hd)),
                 fontsize=13, weight="bold")
    return fig


# ===========================================================================

def overlay(img, det, T, intr, tag_size, mode="both", T_truth=None):
    """live_pose.py 와 **같은 그림**을 그린다. drawing.py 함수만 쓴다."""
    vis = np.asarray(img).copy()
    if vis.ndim == 2:
        vis = cv2.cvtColor(vis, cv2.COLOR_GRAY2BGR)
    if T_truth is not None:
        Tt = np.asarray(T_truth, float)
        rvec, _ = cv2.Rodrigues(Tt[:3, :3])
        uv, _ = cv2.projectPoints(_object_points(float(tag_size)), rvec, Tt[:3, 3],
                                  intr.K, np.zeros(5))
        cv2.polylines(vis, [np.round(uv).astype(int).reshape(-1, 1, 2)], True,
                      (0, 255, 0), 1, 16)
    if det is not None:
        try:
            draw_corners(vis, det)
            if T is not None:
                if mode in ("cube", "both"):
                    draw_cube(vis, intr.params, tag_size, T)
                if mode in ("axes", "both"):
                    draw_axes(vis, intr.params, tag_size, T)
        except Exception:
            pass          # 그리기 실패로 도구가 죽지는 않게. 숫자는 이미 다 있다.
    return vis


def detection_figure(place=None, intr=None, tag_size=DEFAULT_TAG_SIZE, tag_id=0,
                     method="auto", blur_px=0.0, noise=0.0, seed=0, mode="both",
                     result=None, title=None, quad_blur=None, **kw):
    """합성 영상 한 장 + 검출 오버레이 + 정답/추정/오차 표."""
    intr = intr or intrinsics_for()
    res = result or measure(as_place(place, **kw), intr, tag_size=tag_size,
                            tag_id=tag_id, method=method, blur_px=blur_px,
                            noise=noise, seed=seed, quad_blur=quad_blur)
    truth = res["truth"]
    tv = truth_true_values(truth)

    fig = plt.figure(figsize=(15.0, 6.4), dpi=110)
    gs = fig.add_gridspec(1, 2, width_ratios=[1.55, 1.0], wspace=0.06,
                          left=0.02, right=0.99, top=0.88, bottom=0.04)
    axi = fig.add_subplot(gs[0, 0])
    axt = fig.add_subplot(gs[0, 1])
    axi.axis("off")
    axt.axis("off")

    if res["image"] is not None:
        vis = overlay(res["image"], res["detection"], res["T_est"], intr, tag_size,
                      mode=mode, T_truth=truth["T"])
        axi.imshow(cv2.cvtColor(vis, cv2.COLOR_BGR2RGB))
        axi.set_title("synthetic render %dx%d  +  detection overlay  "
                      "(green = truth corners, red = detected)"
                      % (vis.shape[1], vis.shape[0]), fontsize=10)
    else:
        axi.text(0.5, 0.5, "NOT RENDERED\n%s" % res["reason"], ha="center",
                 va="center", fontsize=16, color=C_BAD, transform=axi.transAxes)

    L = []
    L.append("PLACEMENT  d=%.3f m  lat=%+.3f m  hdg=%+.1f deg" %
             (res["place"]["distance"], res["place"]["lateral"], res["place"]["heading"]))
    L.append("           tag %.3f m @ %.2f m   cam @ %.2f m   id=%d" %
             (tag_size, res["place"]["tag_height"], res["place"]["cam_height"], tag_id))
    L.append("RENDER     %s   blur %.0f px   noise %.1f levels" %
             (("simulate.render" if _from_sim("render") is not None
               else "simulate_view._ref_render"), res["blur_px"], res["noise"]))
    L.append("DETECTOR   quad_blur=%.1f%s" %
             (res.get("quad_blur", DEFAULT_QUAD_BLUR),
              "   (raised for noise)"
              if res.get("quad_blur", 0) > DEFAULT_QUAD_BLUR else ""))
    L.append("ESTIMATOR  method=%s" % method)
    L.append("")
    if not res["ok"]:
        L.append("*** FAILED: %s ***" % res["reason"])
    else:
        q = res["quality"]
        L.append("%-11s %11s %11s %11s" % ("", "truth", "estimated", "error"))
        L.append("-" * 47)
        for key, name, unit, eunit, k in ROWS:
            L.append("%-11s %11.3f %11.3f %8.2f %s"
                     % ("%s[%s]" % (name, unit), tv[key], res["est"][key],
                        res["err"][key] * k, eunit))
        L.append("-" * 47)
        L.append("tag_px      %8.1f   (MIN %.0f / STABLE %.0f)"
                 % (q.get("tag_px", float("nan")), MIN_TAG_PX, STABLE_TAG_PX))
        L.append("reproj rms  %8.2f px   margin %6.1f   hamming %d"
                 % (q.get("reproj_rms_px", float("nan")),
                    q.get("decision_margin", float("nan")), q.get("hamming", -1)))
        L.append("quality     %8s   %s" % ("ok" if q.get("ok") else "SUSPECT",
                                           ",".join(q.get("reasons", ())) or "-"))
        L.append("")
        L.append("reliable_angle  docking(approach>=10) : %s"
                 % ("YES" if res["est"]["rel_approach"] else "no"))
        L.append("                quality (tilt>=10)    : %s"
                 % ("YES" if res["est"]["rel_tilt"] else "no"))
        L.append("   two different flags, same name; they do disagree")
        worst_mm = max(abs(res["err"][k]) for k in LEN_KEYS) * 1000.0
        worst_dg = max(abs(res["err"][k]) for k in ANG_KEYS)
        L.append("")
        L.append("WORST  %.2f mm   %.3f deg" % (worst_mm, worst_dg))
        if res["blur_px"] == 0 and res["noise"] == 0:
            L.append("this is the zero-blur zero-noise NOISE FLOOR of the")
            L.append("renderer+detector pair; blur/noise runs are only")
            L.append("meaningful where they exceed it.")
    axt.text(0.0, 1.0, "\n".join(L), family="monospace", fontsize=9.5, va="top",
             ha="left", transform=axt.transAxes)

    fig.suptitle(title or "SIMULATED DETECTION  -  truth vs recovered",
                 fontsize=13, weight="bold")
    return fig, res


# ===========================================================================

#: 스윕할 수 있는 축과 x축 이름. 앞의 셋은 배치를 바꾸고(정답이 매 점 달라진다),
SWEEP_AXES = {
    "heading":  "heading [deg]",
    "lateral":  "lateral [m]",
    "distance": "distance [m]",
    "blur":     "motion blur [px]",
    "noise":    "sensor noise [gray levels]",
}


def sweep_rows(axis="heading", values=None, place=None, intr=None,
               tag_size=DEFAULT_TAG_SIZE, tag_id=0, method="auto",
               blur_px=0.0, noise=0.0, seed=0, quad_blur=None, **kw):
    """스윕을 돌려 sweep_figure 가 먹는 행 목록을 만든다."""
    if axis not in SWEEP_AXES:
        raise SystemExit("모르는 스윕 축: %s (가능: %s)" % (axis, ", ".join(SWEEP_AXES)))
    intr = intr or intrinsics_for()
    base = as_place(place, **kw)
    if values is None:
        values = np.linspace(-40.0, 40.0, 41) if axis == "heading" else np.linspace(0, 1, 11)
    # 잡음 스윕에서는 행마다 필요한 quad_blur 가 달라진다. 검출기를 매번 새로
    dets = {}

    rows = []
    for x in np.asarray(values, dtype=float):
        p = dict(base)
        b, n = float(blur_px), float(noise)
        if axis in ("heading", "lateral", "distance"):
            p[axis] = float(x)
        elif axis == "blur":
            b = float(x)
        else:
            n = float(x)
        qb = quad_blur_for(n, quad_blur)
        if qb not in dets:
            dets[qb] = make_detector(quad_blur=qb)
        r = measure(p, intr, tag_size=tag_size, tag_id=tag_id, method=method,
                    blur_px=b, noise=n, seed=seed, detector=dets[qb], quad_blur=qb)
        row = {"x": float(x), "ok": bool(r["ok"]), "reason": r["reason"],
               "err": r["err"] or {}, "tag_px": float(r["vis"]["tag_px"]),
               "rel_tilt": bool(r["truth"]["tilt"] >= RELIABLE_TILT_DEG),
               "rel_approach": bool(r["truth"]["docking"]["reliable_angle"])}
        rows.append(row)
    return rows


def _spans(xs, flags):
    """flags 가 True 인 연속 구간을 (x0, x1) 목록으로. 음영칠에 쓴다."""
    out = []
    start = None
    xs = np.asarray(xs, dtype=float)
    if xs.size < 2:
        return [(float(xs[0]), float(xs[0]))] if (xs.size and any(flags)) else out
    for i, f in enumerate(list(flags) + [False]):
        if f and start is None:
            start = i
        elif not f and start is not None:
            a = xs[start] - (xs[1] - xs[0]) / 2.0 if start > 0 else xs[0]
            b = xs[i - 1] + (xs[1] - xs[0]) / 2.0 if i - 1 < len(xs) - 1 else xs[-1]
            out.append((a, b))
            start = None
    return out


def sweep_figure(rows, axis="heading", title=None, keys=None, logy=False):
    """스윕 오차 곡선. x = 스윕한 축, y = 오차, 선 하나가 측정값 하나."""
    xs = np.array([r["x"] for r in rows], dtype=float)
    okf = np.array([bool(r.get("ok", True)) for r in rows])
    xl = SWEEP_AXES.get(axis, axis)

    fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12.0, 8.6), dpi=110, sharex=True,
                                   gridspec_kw={"hspace": 0.10, "left": 0.09,
                                                # 오른쪽에 tag_px twin 축이 붙고 그
                                                "right": 0.75, "top": 0.90,
                                                "bottom": 0.10})

    # 음영: 각도를 믿을 수 없는 구간
    bad_tilt = [not r.get("rel_tilt", True) for r in rows]
    bad_appr = [not r.get("rel_approach", True) for r in rows]
    first = True
    for a, b in _spans(xs, bad_tilt):
        for ax in (ax1, ax2):
            ax.axvspan(a, b, color=C_BAD, alpha=0.10, lw=0,
                       label="tag tilt < %g deg\n(angle not trustworthy)"
                             % RELIABLE_TILT_DEG if first and ax is ax1 else None)
        first = False
    first = True
    for a, b in _spans(xs, bad_appr):
        for ax in (ax1, ax2):
            ax.axvspan(a, b, facecolor="none", edgecolor=C_BAD, hatch="///",
                       alpha=0.35, lw=0.0,
                       label="approach < 10 deg\n(docking_state flag)"
                             if first and ax is ax1 else None)
        first = False
    # 검출/자세 실패는 세로선으로. 곡선이 끊긴 이유를 그림 안에 남긴다.
    first = True
    for x in xs[~okf]:
        for ax in (ax1, ax2):
            ax.axvline(x, color=C_BAD, lw=1.0, ls=":",
                       label="no result" if first and ax is ax1 else None)
        first = False

    def plot(ax, group, scale, unit):
        for key in group:
            if keys is not None and key not in keys:
                continue
            y = np.array([r.get("err", {}).get(key, np.nan) for r in rows],
                         dtype=float) * scale
            if not np.isfinite(y).any():
                continue
            ax.plot(xs, np.abs(y) if logy else y, marker="o", ms=2.6, lw=1.3,
                    label="%s [%s]" % (key, unit))
        ax.axhline(0.0, color=C_AXIS, lw=0.9)
        if logy:
            ax.set_yscale("log")
        ax.grid(alpha=0.20, lw=0.6)

    plot(ax1, LEN_KEYS, 1000.0, "mm")
    plot(ax2, ANG_KEYS, 1.0, "deg")
    ax1.set_ylabel("length error  [mm]" + ("  |abs|" if logy else ""))
    ax2.set_ylabel("angle error  [deg]" + ("  |abs|" if logy else ""))
    ax2.set_xlabel(xl)

    # tag_px 를 오른쪽 축에 겹쳐 둔다. 오차가 커질 때 "멀어서" 인지
    px = np.array([r.get("tag_px", np.nan) for r in rows], dtype=float)
    if np.isfinite(px).any():
        axp = ax1.twinx()
        axp.plot(xs, px, color=C_DIM, lw=1.0, ls="--", label="tag_px")
        axp.axhline(STABLE_TAG_PX, color=C_DIM, lw=0.8, ls=":")
        axp.set_ylabel("tag size [px]  (dashed)", color=C_DIM, fontsize=9)
        axp.tick_params(axis="y", labelcolor=C_DIM, labelsize=8)

    h1, l1 = ax1.get_legend_handles_labels()
    h2, l2 = ax2.get_legend_handles_labels()
    seen = {}
    for h, l in zip(h1 + h2, l1 + l2):
        seen.setdefault(l, h)
    fig.legend(seen.values(), seen.keys(), loc="center left",
               bbox_to_anchor=(0.815, 0.5), fontsize=8.5, frameon=False)

    n_bad = int((~okf).sum())
    fig.suptitle(title or ("ERROR vs %s   (%d points%s)"
                           % (xl, len(rows),
                              "" if not n_bad else ", %d with no result" % n_bad)),
                 fontsize=13, weight="bold")
    return fig


# ===========================================================================

def fig_to_bgr(fig):
    """Figure -> BGR 배열. Agg 백엔드라 화면이 없어도 된다."""
    fig.canvas.draw()
    rgba = np.asarray(fig.canvas.buffer_rgba())
    return cv2.cvtColor(rgba, cv2.COLOR_RGBA2BGR)


def save_figure(fig, path, check=True):
    """PNG 로 쓰고 **쓴 것을 다시 읽어 확인한다**."""
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(str(path), dpi=fig.dpi, facecolor="white")
    if check:
        back = cv2.imread(str(path))
        if back is None:
            raise RuntimeError("저장한 PNG 를 다시 못 읽었다: %s" % path)
        sd = float(back.std())
        if sd < 1.0:
            raise RuntimeError("빈 그림이 저장됐다 (std=%.3f): %s" % (sd, path))
        print("saved: %s  (%dx%d, %.0f KB, std=%.1f)"
              % (path, back.shape[1], back.shape[0],
                 path.stat().st_size / 1024.0, sd))
    return path


def show_interactive(named):
    """matplotlib 자체 창으로 띄운다. **마우스로 3D 를 돌려볼 수 있다.**"""
    import matplotlib.pyplot as _plt
    print("  마우스 드래그로 회전, 휠로 확대. 창을 닫으면 끝난다.")
    for name, fig in named.items():
        try:
            fig.canvas.manager.set_window_title(name)
        except Exception:
            pass
    _plt.show()


def show_figures(named, wait=True):
    """cv2 창으로 띄운다. q/ESC 로 닫는다."""
    for name, fig in named.items():
        img = fig_to_bgr(fig)
        h, w = img.shape[:2]
        cv2.namedWindow(name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(name, min(w, 1500), int(h * min(w, 1500) / w))
        cv2.imshow(name, img)
    if not wait:
        return
    print("창을 닫으려면 q 또는 ESC")
    while True:
        k = cv2.waitKey(30) & 0xFF
        if k in (ord("q"), 27):
            break
        if all(cv2.getWindowProperty(n, cv2.WND_PROP_VISIBLE) < 1 for n in named):
            break
    cv2.destroyAllWindows()


# ----------------------------------------------------------------- 3D 배치 뷰
def layout3d_figure(place=None, tag_size=DEFAULT_TAG_SIZE, intr=None,
                    title=None, truth=None, elev=18.0, azim=-58.0):
    """배치를 3차원으로 한눈에. 평면도/입면도의 보조다."""
    import numpy as _np
    from mpl_toolkits.mplot3d.art3d import Poly3DCollection

    place = as_place(place)
    th = float(place["tag_height"])
    T = placement_to_T(**place)
    T_tag_cam = _np.asarray(invert_T(T))
    p = T_tag_cam[:3, 3]            # 태그 좌표계에서 본 카메라 위치
    R = T_tag_cam[:3, :3]           # 열 = 카메라 축을 태그 좌표계로

    def w(v):                       # 태그 좌표 -> 그리기 좌표
        v = _np.asarray(v, dtype=float)
        return _np.array([v[0], -v[2], v[1] + th])

    cam = w(p)
    fig = plt.figure(figsize=(9.0, 7.0))
    ax = fig.add_subplot(111, projection="3d")

    # 바닥 격자
    span = max(2.0, abs(p[0]) + 1.0, -p[2] + 1.0)
    g = _np.arange(-_np.ceil(span), _np.ceil(span) + 0.001, 0.5)
    for v in g:
        ax.plot([v, v], [0, span + 0.5], [0, 0], color="0.88", lw=0.6, zorder=0)
        ax.plot([-span, span], [v, v], [0, 0], color="0.88", lw=0.6, zorder=0)

    # 태그면 — 실제 크기·높이. 법선이 지게차 쪽(+Y)을 향한다
    s = tag_size / 2.0
    quad = [w([-s, -s, 0]), w([s, -s, 0]), w([s, s, 0]), w([-s, s, 0])]
    ax.add_collection3d(Poly3DCollection([quad], facecolor="0.15",
                                         edgecolor="k", lw=1.2, alpha=0.95))
    n = w([0, 0, -1]) - w([0, 0, 0])            # 태그 법선(앞쪽)
    ax.quiver(*w([0, 0, 0]), *(n * 0.45), color="tab:orange", lw=2.0,
              arrow_length_ratio=0.25)
    ax.plot([0, 0], [0, -p[2]], [th, th], color="tab:orange", ls="--", lw=1.0)

    # 정면축이 바닥에 닿는 선
    ax.plot([0, 0], [0, -p[2]], [0, 0], color="tab:orange", ls=":", lw=1.0)

    # 카메라 시야뿔 — 실제 화각으로
    if intr is not None:
        hw, hh = intr.width / 2.0 / intr.fx, intr.height / 2.0 / intr.fy
        L = max(0.8, -p[2] * 1.05)
        rays = [R @ _np.array([sx * hw, sy * hh, 1.0]) * L
                for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
        pts = [w(p + r) for r in rays]
        for q in pts:
            ax.plot([cam[0], q[0]], [cam[1], q[1]], [cam[2], q[2]],
                    color="tab:blue", lw=0.8, alpha=0.55)
        ax.add_collection3d(Poly3DCollection([pts], facecolor="tab:blue",
                                             edgecolor="tab:blue", lw=0.6, alpha=0.10))

    # 카메라(지게차) 위치와 진행 방향
    ax.scatter(*cam, s=55, color="tab:blue", depthshade=False, zorder=5)
    fwd = w(p + R[:, 2] * 0.6) - cam
    ax.quiver(*cam, *fwd, color="tab:blue", lw=2.2, arrow_length_ratio=0.25)
    ax.plot([cam[0], cam[0]], [cam[1], cam[1]], [0, cam[2]],
            color="tab:blue", ls=":", lw=0.9)      # 바닥까지 수선

    ax.set_xlabel("X  lateral [m]"); ax.set_ylabel("Y  in front of tag [m]")
    ax.set_zlabel("Z  height [m]")
    ax.set_xlim(-span, span); ax.set_ylim(0, span + 0.5)
    ax.set_zlim(0, max(2.0, th + 0.6, cam[2] + 0.6))
    try:
        ax.set_box_aspect((2 * span, span + 0.5, max(2.0, th + 0.6)))
    except Exception:
        pass
    ax.view_init(elev=elev, azim=azim)
    t = truth if truth is not None else truth_of(place, tag_size)
    v = truth_true_values(t)
    ax.set_title(title or
                 ("3D layout   tag %.2fm / cam %.2fm / dist %.2fm / "
                  "lateral %+.2fm / heading %+.1f deg"
                  % (th, place["cam_height"], place["distance"],
                     place["lateral"], place["heading"])), fontsize=10)
    fig.tight_layout()
    return fig

def main():
    ap = argparse.ArgumentParser(
        description="AprilTag docking simulator - views (no camera, no display needed)")
    ap.add_argument("--tag-size", type=float, default=DEFAULT_TAG_SIZE,
                    help="태그 한 변 [m], 검은 테두리 바깥까지")
    ap.add_argument("--tag-height", type=float, default=DEFAULT_PLACE["tag_height"],
                    help="바닥에서 태그 중심까지 [m]")
    ap.add_argument("--cam-height", type=float, default=DEFAULT_PLACE["cam_height"],
                    help="바닥에서 카메라 렌즈까지 [m]")
    ap.add_argument("--distance", type=float, default=DEFAULT_PLACE["distance"],
                    help="태그 정면축을 따라 태그까지 [m]")
    ap.add_argument("--lateral", type=float, default=DEFAULT_PLACE["lateral"],
                    help="정면축에서 좌우 [m] (+ 는 태그 +x = 운전자의 왼쪽)")
    ap.add_argument("--heading", type=float, default=DEFAULT_PLACE["heading"],
                    help="지게차가 축과 이루는 각 [도] (+ 는 운전자가 왼쪽으로 튼 것)")
    ap.add_argument("--res", default=DEFAULT_RES, choices=list(RESOLUTIONS),
                    help="내부 파라미터를 고르는 해상도 (D435i 컬러 공장값)")
    ap.add_argument("--tag-id", type=int, default=0)
    ap.add_argument("--method", default="auto", choices=["auto", "tag", "pnp"])
    ap.add_argument("--blur", type=float, default=0.0, metavar="PX",
                    help="가로 모션블러 길이 [px]")
    ap.add_argument("--noise", type=float, default=0.0, metavar="LEVELS",
                    help="가우시안 잡음 표준편차 [그레이레벨]")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--quad-blur", type=float, default=None, metavar="SIGMA",
                    help="검출기 quad_blur. 안 주면 잡음에 맞춰 고른다 "
                         "(잡음 >%.0f 이면 %.1f — 0 이면 검출기가 죽는다)"
                    % (NOISE_QUAD_BLUR_LEVELS, NOISE_QUAD_BLUR))
    ap.add_argument("--draw", default="both", choices=["cube", "axes", "both"])
    ap.add_argument("--view", default="all",
                    choices=["all", "layout", "layout3d", "detection", "sweep"])
    ap.add_argument("--sweep", default="heading", choices=list(SWEEP_AXES))
    ap.add_argument("--from", dest="lo", type=float, default=None)
    ap.add_argument("--to", dest="hi", type=float, default=None)
    ap.add_argument("--steps", type=int, default=41)
    ap.add_argument("--logy", action="store_true", help="오차를 절대값 로그축으로")
    ap.add_argument("--show", action="store_true", help="cv2 창으로 띄운다(정지)")
    ap.add_argument("--interactive", "-i", action="store_true",
                    help="matplotlib 창으로 띄운다 — 3D 를 마우스로 돌릴 수 있다")
    ap.add_argument("--save", nargs="?", const=str(OUTDIR), default=None, metavar="DIR",
                    help="PNG 를 쓸 폴더 (기본 work_dirs/simulate)")
    ap.add_argument("--prefix", default="", help="파일 이름 앞에 붙일 말")
    args = ap.parse_args()

    # 대화형 창은 **그림을 만들기 전에** 백엔드를 바꿔야 한다. 모듈 상단이
    if args.interactive:
        args.show = True
        try:
            matplotlib.use("TkAgg", force=True)
        except Exception as exc:
            print("  대화형 백엔드를 못 열었다(%s). cv2 정지화면으로 띄운다." % exc)
            args.interactive = False

    # 아무것도 안 주면 저장한다. 헤드리스가 기본 사용처다.
    save_dir = Path(args.save) if args.save else (None if args.show else OUTDIR)

    intr = intrinsics_for(args.res)
    place = as_place(tag_height=args.tag_height, cam_height=args.cam_height,
                     distance=args.distance, lateral=args.lateral, heading=args.heading)
    truth = truth_of(place, args.tag_size)
    vis = visibility(truth["T"], intr, args.tag_size)

    print("placement  : tag %.3f m @ %.2f m | cam @ %.2f m | d %.2f m | lat %+.3f m | hdg %+.1f deg"
          % (args.tag_size, place["tag_height"], place["cam_height"],
             place["distance"], place["lateral"], place["heading"]))
    print("intrinsics : %s  fx=%.1f fy=%.1f cx=%.1f cy=%.1f"
          % (args.res, intr.fx, intr.fy, intr.cx, intr.cy))
    st = truth["docking"]
    print("truth      : lateral %+.3f  forward %+.3f  vertical %+.3f  "
          "heading %+.1f  approach %.1f  tilt %.1f"
          % (st["lateral"], st["forward"], st["vertical"], st["heading_deg"],
             st["approach_deg"], truth["tilt"]))
    print("visibility : %s  tag %.0f px"
          % ("in frame" if vis["ok"] else
             ("BEHIND CAMERA" if vis["behind"] else "OUTSIDE FRAME"), vis["tag_px"]))
    if not vis["ok"]:
        print("  ! 이 배치는 태그가 화면 밖이다. 배치도는 그려 주지만 검출은 실패한다.")

    figs = {}
    want = (("layout", "layout3d", "detection", "sweep")
            if args.view == "all" else (args.view,))

    if "layout" in want:
        figs["layout"] = layout_figure(place, tag_size=args.tag_size, intr=intr,
                                       truth=truth)
    if "layout3d" in want:
        # 평면도/입면도는 값을 정확히 읽는 용도, 이건 전체를 감으로 보는 용도다.
        figs["layout3d"] = layout3d_figure(place, tag_size=args.tag_size,
                                           intr=intr, truth=truth)
    if "detection" in want:
        fig, res = detection_figure(place, intr=intr, tag_size=args.tag_size,
                                    tag_id=args.tag_id, method=args.method,
                                    blur_px=args.blur, noise=args.noise,
                                    seed=args.seed, mode=args.draw,
                                    quad_blur=args.quad_blur)
        figs["detection"] = fig
        if res["ok"]:
            print("detection  : worst %.2f mm / %.3f deg  tag_px %.1f  %s"
                  % (max(abs(res["err"][k]) for k in LEN_KEYS) * 1000.0,
                     max(abs(res["err"][k]) for k in ANG_KEYS),
                     res["est"]["tag_px"],
                     "quality ok" if res["quality"].get("ok") else
                     ",".join(res["quality"].get("reasons", ()))))
        else:
            print("detection  : FAILED (%s)" % res["reason"])
    if "sweep" in want:
        # --from 과 --to 는 각각 따로 채운다. 한쪽만 줬을 때 다른 쪽까지
        d_lo, d_hi = {"heading": (-40.0, 40.0), "lateral": (-2.0, 2.0),
                      "distance": (0.8, 6.0), "blur": (0.0, 24.0),
                      "noise": (0.0, 12.0)}[args.sweep]
        lo = d_lo if args.lo is None else args.lo
        hi = d_hi if args.hi is None else args.hi
        vals = np.linspace(lo, hi, max(2, int(args.steps)))
        print("sweep      : %s from %g to %g in %d steps ..." % (args.sweep, lo, hi, len(vals)))
        rows = sweep_rows(args.sweep, vals, place=place, intr=intr,
                          tag_size=args.tag_size, tag_id=args.tag_id,
                          method=args.method, blur_px=args.blur, noise=args.noise,
                          seed=args.seed, quad_blur=args.quad_blur)
        figs["sweep_%s" % args.sweep] = sweep_figure(rows, axis=args.sweep,
                                                     logy=args.logy)
        nbad = sum(0 if r["ok"] else 1 for r in rows)
        print("             %d/%d points produced a pose" % (len(rows) - nbad, len(rows)))
        if nbad > 0.2 * len(rows):
            # 실패가 많으면 거의 항상 '태그가 화면 밖' 이다. 그림에는 세로 점선으로
            ok_x = [r["x"] for r in rows if r["ok"]]
            why = [r["reason"] for r in rows if not r["ok"] and r["reason"]]
            top = max(set(why), key=why.count) if why else "see figure"
            print("             ! %d points had no result (%s). usable span here is "
                  "%s .. %s — try --from/--to inside it."
                  % (nbad, top,
                     ("%g" % ok_x[0]) if ok_x else "-",
                     ("%g" % ok_x[-1]) if ok_x else "-"))

    if save_dir is not None:
        for name, fig in figs.items():
            save_figure(fig, Path(save_dir) / ("%s%s.png" % (args.prefix, name)))
    if args.show:
        show_interactive(figs) if args.interactive else show_figures(figs)
    for fig in figs.values():
        plt.close(fig)


if __name__ == "__main__":
    main()
