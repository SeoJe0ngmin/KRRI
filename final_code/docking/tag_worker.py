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

    def frame(self, raw, stamp, epoch, capture_key):
        tag = self.observer.process(raw, self.model, stamp, epoch, None)
        tag['capture_key'] = capture_key
        return tag


def link(ctx, camera_cfg, slots=3):
    return ring.link(
        ctx, (int(camera_cfg['color_height']), int(camera_cfg['color_width']), 3), np.uint8, slots, results_depth=32
    )


def run_process(shm_name, slots, frame_shape, frames_q, results_q, stop, settings_q=None):
    ring.run(TagMeasurer, shm_name, slots, frame_shape, np.uint8, frames_q, results_q, stop, settings_q)
