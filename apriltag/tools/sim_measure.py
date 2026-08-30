"""배치 하나를 렌더 -> 검출 -> 추정까지 돌려 오차를 냄.

sim_engine.py 가 그림과 정답을 만들고, 여기서 진짜 검출기에 넣어 봄.
tools/sim.py 가 이 measure() 를 부름.
"""
import os
import sys

import cv2
import numpy as np

sys.path.insert(0, os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

from src.config import TAG_SIZE_M as DEFAULT_TAG_SIZE                # noqa: E402
from src.config import CAM_HEIGHT_M, TAG_HEIGHT_M                    # noqa: E402
from src.models import (CameraIntrinsics, DEFAULT_QUAD_BLUR, detect,  # noqa: E402
                        docking_state, estimate_pose, make_detector,
                        pose_quality, pose_to_xyzrpy, tag_pixel_size,
                        tag_tilt_deg, to_gray)
from sim_engine import (DEFAULT_RES, PRESETS, placement_to_T,        # noqa: E402
                        render, visibility)

#: 배치 한 벌. 전부 바닥 기준으로 잼.
DEFAULT_PLACE = {"tag_height": TAG_HEIGHT_M, "cam_height": CAM_HEIGHT_M,
                 "distance": 3.00, "lateral": -1.03, "heading": 20.0}

#: 해상도 이름 -> 카메라 값 (sim_engine.PRESETS 와 같은 것)
RESOLUTIONS = {k: (v.fx, v.fy, v.cx, v.cy) for k, v in PRESETS.items()}


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


NOISE_QUAD_BLUR = 2.0


# 잡음을 넣을 때 검출기에 반드시 넣어야 하는 quad_blur 하한.
NOISE_QUAD_BLUR_LEVELS = 3.0


_QB_WARNED = [False]


def as_place(place=None, **kw):
    """배치 dict 를 만듦. 안 준 칸은 기본 배치에서 채움."""
    out = dict(DEFAULT_PLACE)
    if place:
        out.update({k: v for k, v in place.items() if k in DEFAULT_PLACE})
    out.update({k: v for k, v in kw.items() if k in DEFAULT_PLACE and v is not None})
    return {k: float(v) for k, v in out.items()}


def intrinsics_for(res=DEFAULT_RES):
    """해상도 이름 -> CameraIntrinsics. 왜곡은 비어 있음(이 개체는 0 임)."""
    if res not in RESOLUTIONS:
        raise SystemExit("모르는 해상도: %s (가능: %s)" % (res, ", ".join(RESOLUTIONS)))
    fx, fy, cx, cy = RESOLUTIONS[res]
    w, h = (int(v) for v in res.split("x"))
    return CameraIntrinsics(fx=fx, fy=fy, cx=cx, cy=cy, width=w, height=h)


def quad_blur_for(noise, quad_blur=None):
    """잡음 수준에 맞는 quad_blur 를 고름. 사용자가 준 값이 위험하면 올리고 말함."""
    noise = float(noise)
    if quad_blur is None:
        return DEFAULT_QUAD_BLUR if noise <= NOISE_QUAD_BLUR_LEVELS else NOISE_QUAD_BLUR
    quad_blur = float(quad_blur)
    if noise > NOISE_QUAD_BLUR_LEVELS and quad_blur < NOISE_QUAD_BLUR:
        if not _QB_WARNED[0]:
            _QB_WARNED[0] = True
            print("[sim] 잡음 %.1f 에 quad_blur=%.1f 은 AT2 검출기가 "
                  "세그폴트로 죽는 조합이다. %.1f 로 올려서 돈다."
                  % (noise, quad_blur, NOISE_QUAD_BLUR))
        return NOISE_QUAD_BLUR
    return quad_blur


def degrade(img, blur_px=0.0, noise=0.0, seed=0):
    """합성 영상을 일부러 나쁘게 만듦. 모션블러와 센서잡음."""
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


def truth_of(place, tag_size=DEFAULT_TAG_SIZE):
    """배치에서 정답을 뽑음. **직접 계산하지 않고 파이프라인 함수에 물어봄.**"""
    # 엔진은 heading_deg 라는 이름을 씀. 배치 dict 는 heading 임.
    T = placement_to_T(place["tag_height"], place["cam_height"], place["distance"],
                       place["lateral"], place["heading"])
    st = docking_state(T)
    v = pose_to_xyzrpy(T)
    return {"T": T, "docking": st, "xyzrpy": v,
            "tilt": tag_tilt_deg(T),
            "z_optical": float(T[2, 3]),
            "tag_size": float(tag_size)}


def truth_true_values(truth):
    """정답 dict 를 ROWS 키에 맞춘 평평한 dict 로 폄."""
    st = truth["docking"]
    return {"lateral": st["lateral"], "forward": st["forward"],
            "vertical": st["vertical"], "distance": st["distance"],
            "heading": st["heading_deg"], "approach": st["approach_deg"],
            "tilt": truth["tilt"], "z_optical": truth["z_optical"]}


def measure(place, intr, tag_size=DEFAULT_TAG_SIZE, tag_id=0, method="auto",
            blur_px=0.0, noise=0.0, seed=0, detector=None, quad_blur=None):
    """한 배치를 렌더 -> 검출 -> 추정까지 돌려 정답/추정/오차를 냄."""
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
        out["reason"] = vis["reason"]      # 엔진이 이유를 문장으로 줌
        return out

    # sim_engine.render() 는 vis 가 통과시킨 배치도 거절할 수 있음(여백까지 보므로).
    try:
        img = render(truth["T"], intr, tag_size, tag_id, intr.width, intr.height)
    except Exception as exc:                      # OutOfView 를 이름으로 못 잡음
        if type(exc).__name__ != "OutOfView":     # (참조구현으로 떨어지면 ValueError 다)
            if not isinstance(exc, ValueError):
                raise
        # reason 은 **그림에 찍힌다**. 이 파일의 규약대로 ASCII 로만 적음 —
        print("[sim] not renderable: %s" % exc)
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
