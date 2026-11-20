#!/usr/bin/env python3
"""Micro-benchmarks of the per-frame Python work (no hardware): the depth bird's-eye measurement, the observer per VIO packet, the
stationary judgment, the colour preview encoding, the JSON records and the preview deep copy. Prints ms per call; the Orin Nano is
roughly 3-5x slower than a desktop core.  python scripts/bench.py [--repeat 30] [--run work/<day>/drive/<run>]"""

import argparse
import copy
import json
import math
import sys
import timeit
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parents[1] / 'tests'))


def ms(fn, n):
    fn()
    t = timeit.timeit(fn, number=n)
    return 1e3 * t / n


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--repeat', type=int, default=30)
    ap.add_argument('--run')
    a = ap.parse_args()
    rows = []
    # ---- depth: a synthetic frame with the doors, the X frames, the lattice (like the field: ~5000-7000 band points)
    from test_depth_goal import render, doors_scene, INTR
    from docking.depth_goal import measure_bev, bev_points
    from docking.bev import occupancy, depth_to_points

    dist = 2.0
    cones = doors_scene(dist)
    fronts = [(dist + 2.5, -0.8, 0.8, 0.365, 0.405)] + [
        (dist + 2.5, y - 0.02, y + 0.02, 0.42, 0.75) for y in (0.785, 0.405, 0.0, -0.405, -0.785)
    ]
    z, T = render(cones=cones, fronts=fronts, noise=0.003)
    entrance = dict(expected_x_m=dist, width_m=1.65, width_tol_m=0.2, heading_rad=0.0)
    lattice = dict(
        expected_x_m=dist + 2.5, expected_y_m=0.0, pair_width_m=0.77, pair_tol_m=0.1, plates=[(0.6, 0.13), (-0.6, 0.13)]
    )
    params = dict(max_m=4.0, lattice_max_m=6.5)
    m = measure_bev(z, INTR, T, entrance, lattice, params=params, stamp=1.0)
    rows.append(
        (
            'measure_bev (full frame, %d band pts)' % m['points'],
            ms(lambda: measure_bev(z, INTR, T, entrance, lattice, params=params, stamp=1.0), a.repeat),
        )
    )
    rows.append(('  depth_to_points step 2', ms(lambda: depth_to_points(z, INTR, 2, 6.5), a.repeat)))
    body, _, _ = bev_points(z, INTR, T, 2, 6.5, 0.33, 0.75, 0.2)
    rows.append(
        ('  occupancy (%d pts)' % len(body), ms(lambda: occupancy(body, 8.0, 3.0, 0.05, -math.inf, math.inf), a.repeat))
    )
    grid = occupancy(body, 8.0, 3.0, 0.05, -math.inf, math.inf)
    from docking.bev import bev_cells

    acc = grid.astype(np.float32)
    rows.append(('  dashboard cells list (%d cells)' % len(bev_cells(acc)), ms(lambda: bev_cells(acc), a.repeat)))
    # ---- observer per packet (synthetic VIO + tag windows)
    from support import config, VioSource, synthetic_tag
    from docking.observer import Observer

    c = config()
    s = c['sensor']
    s['calibration']['T_body_imu'] = np.eye(4).tolist()
    o = Observer(s)
    src = VioSource()
    truth = (-4.0, 0.1, 0.02)
    t = [0.0]

    def step():
        t[0] += 0.04
        r = o.add_vio(src.packet(t[0], truth, gyro_norm=0.001))
        o.add_tag(synthetic_tag(s, truth, t[0]))
        return r

    rows.append(('observer.add_vio + add_tag (per packet)', ms(step, a.repeat * 5)))
    o2 = Observer(s)
    src2 = VioSource()
    t2 = [0.0]

    def step2():
        t2[0] += 0.04
        return o2.add_vio(src2.packet(t2[0], truth, gyro_norm=0.001))

    rows.append(('observer.add_vio alone (per packet)', ms(step2, a.repeat * 5)))
    # ---- stationary judgment with 400 raw gyro samples in the window
    from docking.stationary import StationaryDetector

    sd = StationaryDetector(c['tuning'])
    stamp = 10.0
    for i in range(400):
        sd.add_gyro(stamp - 2.0 + i / 200.0, [1e-3, 0.0, 0.0])
    rows.append(('stationary.rate (400 samples)', ms(lambda: sd.rate(stamp), a.repeat * 5)))
    rows.append(('stationary.update', ms(lambda: sd.update(stamp, np.zeros(3), True, 1.0, True, 1e-3), a.repeat * 5)))
    # ---- preview encoding
    from docking.webview import encode_preview

    img = np.random.default_rng(0).integers(0, 255, (720, 1280, 3), np.uint8)
    obs = dict(
        tag=dict(reason='accepted', stream='color', corners=[[100, 100], [200, 100], [200, 200], [100, 200]], ok=True),
        odom=dict(reason='openvins_corrected', tracked_points=[]),
        depth=m,
        inner_tags=None,
    )
    cal = dict(
        T_body_camera=s['calibration']['T_body_camera'],
        camera_model=dict(fx=924.0, fy=924.0, cx=640.0, cy=360.0, coeffs=[0.0] * 5, width=1280, height=720),
        T_body_ir=s['calibration']['T_body_camera'],
        depth_intr=INTR,
        site=c['site'],
    )
    rows.append(
        (
            'encode_preview colour 1280x720 -> 640',
            ms(lambda: encode_preview(img, obs, dict(width=640, jpeg_quality=65), 'color', cal), 10),
        )
    )
    # ---- records
    rec = None
    if a.run:
        with open(Path(a.run) / 'observations.jsonl', 'rb') as fh:
            for line in fh:
                r = json.loads(line)
                if 'kind' not in r and r.get('pose') is not None:
                    rec = r
                    break
        ctl = None
        with open(Path(a.run) / 'control.jsonl', 'rb') as fh:
            for line in fh:
                ctl = line
        ctl = json.loads(ctl)
        from docking.serialization import numpy_json

        rows.append(
            (
                'json.dumps observation row (%d B)' % len(json.dumps(rec)),
                ms(lambda: json.dumps(rec, ensure_ascii=False, allow_nan=False, default=numpy_json), a.repeat * 5),
            )
        )
        rows.append(
            (
                'json.dumps control row, %d events (%d B)'
                % (len(ctl['status'].get('events') or []), len(json.dumps(ctl))),
                ms(lambda: json.dumps(ctl, ensure_ascii=False, allow_nan=False, default=numpy_json), a.repeat * 5),
            )
        )
        rows.append(('deepcopy observation (preview)', ms(lambda: copy.deepcopy(rec), a.repeat * 5)))
    print('%-48s %10s' % ('item', 'ms/call'))
    for name, v in rows:
        print('%-48s %10.3f' % (name, v))


if __name__ == '__main__':
    main()
