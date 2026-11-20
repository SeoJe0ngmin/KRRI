"""AprilTag detection in its own process (2026-11-01): the colour frame goes through the shared ring (docking.ring), the result
is the same dict TagObserver.process returns (plus capture_key), consumed by the camera worker exactly like the thread's result.
The detector's C code already releases the interpreter lock; what moves out of the camera process is the Python around it (colour
conversion call, corner handling, PnP, the eight-corner covariance of the inner-tag bundle). tag.process false keeps the thread.
"""

import numpy as np
from . import ring


class TagMeasurer:
    def __init__(self, settings):
        from .tag import TagObserver

        self.observer = TagObserver(dict(tag=settings['tag'], uncertainty=settings['uncertainty']))
        self.model = settings['model']
        self.structure = settings.get('structure')  # 구조물 방향 측정 설정(없으면 재지 않는다)

    def frame(self, raw, stamp, epoch, capture_key, structure=None):
        tag = self.observer.process(raw, self.model, stamp, epoch, None)
        tag['capture_key'] = capture_key
        if structure is not None and self.structure:
            tag['structure'] = structure_result(raw, self.model, self.structure, structure.get('prior_deg'))
        return tag


def structure_result(raw, model, cfg, prior_deg=None):
    """같은 컬러 장면에서 구조물 방향을 잰다. 실패해도 태그 결과는 그대로 나가야 하므로 예외는 까닭으로만 적는다."""
    try:
        from .structure import measure

        return measure(raw, model, cfg.get('settings'), cfg.get('T_body_camera'), prior_deg)
    except Exception as exc:
        return dict(ok=False, reason='error: ' + str(exc)[:120])


def link(ctx, camera_cfg, slots=3):
    return ring.link(
        ctx, (int(camera_cfg['color_height']), int(camera_cfg['color_width']), 3), np.uint8, slots, results_depth=32
    )


def run_process(shm_name, slots, frame_shape, frames_q, results_q, stop, settings_q=None):
    ring.run(TagMeasurer, shm_name, slots, frame_shape, np.uint8, frames_q, results_q, stop, settings_q)
