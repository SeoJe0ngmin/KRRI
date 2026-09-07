from dataclasses import dataclass
import numpy as np
from .util import invert_T, pr2t, r2rpy, rpy2r

def facing_approach(roll_deg=0.0):
    R = np.array([[0.0, 0.0, -1.0], [-1.0, 0.0, 0.0], [0.0, -1.0, 0.0]])
    if roll_deg:
        R = R @ rpy2r(np.array([0.0, 0.0, np.deg2rad(roll_deg)]))
    return R

@dataclass
class TagPlacement:
    tag_id: int
    position: np.ndarray
    R: np.ndarray
    size: float
    name: str = ''

    @property
    def T_dock_tag(self):
        return pr2t(np.asarray(self.position, dtype=float), self.R)

def average_rotations(Rs):
    M = np.mean(np.asarray(Rs, dtype=float), axis=0)
    U, _, Vt = np.linalg.svd(M)
    R = U @ Vt
    if np.linalg.det(R) < 0:
        U[:, -1] *= -1
        R = U @ Vt
    return R

class TagLayout:

    def __init__(self, placements):
        self.placements = {p.tag_id: p for p in placements}

    def __contains__(self, tag_id):
        return tag_id in self.placements

    def __getitem__(self, tag_id):
        return self.placements[tag_id]

    def size_of(self, tag_id, default=None):
        p = self.placements.get(tag_id)
        return p.size if p else default

    def camera_pose_in_dock(self, tag_id, T_cam_tag):
        return self.placements[tag_id].T_dock_tag @ invert_T(np.asarray(T_cam_tag))

    def fuse(self, observations):
        Ts = [self.camera_pose_in_dock(tid, T) for tid, T in observations if tid in self.placements]
        if not Ts:
            return (None, 0)
        p = np.mean([T[:3, 3] for T in Ts], axis=0)
        R = average_rotations([T[:3, :3] for T in Ts])
        return (pr2t(p, R), len(Ts))

    def describe(self):
        lines = [f"{'id':>4}  {'이름':<12} {'위치 (x,y,z)[m]':<26} {'자세 rpy[도]':<24} {'크기[m]':>7}"]
        for tid in sorted(self.placements):
            p = self.placements[tid]
            xyz = ', '.join((f'{v:+.3f}' for v in p.position))
            rpy = ', '.join((f'{v:+7.1f}' for v in r2rpy(p.R, 'deg')))
            lines.append(f'{tid:>4}  {p.name:<12} ({xyz})   ({rpy})  {p.size:>7.3f}')
        return '\n'.join(lines)
DOCK_LAYOUT = TagLayout([TagPlacement(tag_id=1, name='상단(외부)', position=np.array([-0.2, 0.0, 1.6]), R=facing_approach(), size=0.2), TagPlacement(tag_id=2, name='내부(안쪽)', position=np.array([1.2, 0.0, 0.8]), R=facing_approach(), size=0.2)])
