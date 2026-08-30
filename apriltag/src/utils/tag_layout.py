"""태그 배치표 — 여러 태그를 하나의 좌표계로 묶는다."""
from dataclasses import dataclass

import numpy as np

from .util import invert_T, pr2t, r2rpy, rpy2r


def facing_approach(roll_deg=0.0):
    """들어오는 지게차를 마주보는 태그의 자세(회전행렬)."""
    # dock 축 기준으로 태그축을 배치: 태그x -> dock -y, 태그y -> dock -z, 태그z -> dock -x
    R = np.array([[0.0,  0.0, -1.0],
                  [-1.0, 0.0,  0.0],
                  [0.0, -1.0,  0.0]])
    if roll_deg:
        R = R @ rpy2r(np.array([0.0, 0.0, np.deg2rad(roll_deg)]))
    return R


@dataclass
class TagPlacement:
    """태그 한 장의 설치 정보."""
    tag_id: int
    position: np.ndarray           # dock 기준 태그 중심 [m]
    R: np.ndarray                  # dock 기준 태그 자세 (3x3)
    size: float                    # 태그 한 변 [m] — 검은 테두리 포함
    name: str = ""

    @property
    def T_dock_tag(self):
        """dock 기준 태그의 4x4 자세."""
        return pr2t(np.asarray(self.position, dtype=float), self.R)


def average_rotations(Rs):
    """회전행렬 여러 개의 평균."""
    M = np.mean(np.asarray(Rs, dtype=float), axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:          # 반사(거울상)가 나오면 뒤집는다
        U[:, -1] *= -1
        R = U @ Vt
    return R


class TagLayout:
    """태그 배치표."""

    def __init__(self, placements):
        self.placements = {p.tag_id: p for p in placements}

    def __contains__(self, tag_id):
        return tag_id in self.placements

    def __getitem__(self, tag_id):
        return self.placements[tag_id]

    def size_of(self, tag_id, default=None):
        """그 태그의 실제 크기 [m]. 배치표에 없으면 default."""
        p = self.placements.get(tag_id)
        return p.size if p else default

    def camera_pose_in_dock(self, tag_id, T_cam_tag):
        """태그 하나를 보고 얻은 자세 -> dock 기준 카메라 자세."""
        return self.placements[tag_id].T_dock_tag @ invert_T(np.asarray(T_cam_tag))

    def fuse(self, observations):
        """여러 태그에서 나온 결과를 하나로 합친다."""
        Ts = [self.camera_pose_in_dock(tid, T)
              for tid, T in observations if tid in self.placements]
        if not Ts:
            return None, 0
        p = np.mean([T[:3, 3] for T in Ts], axis=0)
        R = average_rotations([T[:3, :3] for T in Ts])
        return pr2t(p, R), len(Ts)

    def describe(self):
        """배치표를 사람이 읽을 수 있게."""
        lines = [f"{'id':>4}  {'이름':<12} {'위치 (x,y,z)[m]':<26} {'자세 rpy[도]':<24} {'크기[m]':>7}"]
        for tid in sorted(self.placements):
            p = self.placements[tid]
            xyz = ", ".join(f"{v:+.3f}" for v in p.position)
            rpy = ", ".join(f"{v:+7.1f}" for v in r2rpy(p.R, "deg"))
            lines.append(f"{tid:>4}  {p.name:<12} ({xyz})   ({rpy})  {p.size:>7.3f}")
        return "\n".join(lines)


# ── 우리 도킹 설정 ────────────────────────────────────────────────
DOCK_LAYOUT = TagLayout([
    # size 는 인쇄한 태그를 자로 잰 값(검은 테두리 바깥까지)으로 반드시 고칠 것.
    TagPlacement(tag_id=1, name="상단(외부)",
                 position=np.array([-0.20, 0.0, 1.60]),
                 R=facing_approach(), size=0.20),
    TagPlacement(tag_id=2, name="내부(안쪽)",
                 position=np.array([1.20, 0.0, 0.80]),
                 R=facing_approach(), size=0.20),
])
