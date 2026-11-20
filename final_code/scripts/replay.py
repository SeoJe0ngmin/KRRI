#!/usr/bin/env python3
"""Offline replay of a recorded run (work/<day>/<drive|shadow>/<run>). No hardware.

  python scripts/replay.py <run folder> [--signature out.json] [--compare ref.json] [--profile 25] [--quiet]

Two replays from the same folder:
  observer    the recorded OpenVINS packets ('vio' records) and tag frames ('tag' records) go through Observer.add_vio / add_tag in
              the order the camera worker used, with the re-zero and re-anchor requests the controller made at the time (from
              control.jsonl). The poses are compared with the recorded observations. Without the raw 200 Hz gyro (not logged) the
              stationary judgment falls back to the per-packet gyro norm, so small differences from the field are expected.
  controller  the recorded observations go through Controller.step at the recorded supervisor ticks (control.jsonl: now +
              observation_capture), with the operator's Enter where the recorded run left WAIT_ENTER. The resulting
              state/phase/command sequence and the events are compared with the recording.
--signature writes both replays' outputs (poses, decisions, events, final report); --compare checks a new run of the code against
such a file: the way to prove that a refactoring changed nothing. --profile prints the hottest Python functions of each replay.
The recorded config_used.json drives everything (site, tuning, calibration); keys added since the recording take the code defaults.
"""

import argparse
import collections
import cProfile
import io
import json
import pstats
import sys
import time
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from docking.config import TUNING, LEGACY_SITE_KEYS, apply_inner
from docking.controller import Controller
from docking.geometry import cut_position
from docking.observer import Observer


def load_cfg(folder):
    cfg = json.loads((folder / 'config_used.json').read_text())
    site = cfg['site']
    for old, new in LEGACY_SITE_KEYS.items():
        if old in site and new not in site:
            site[new] = site.pop(old)
    tuning = dict(TUNING)
    tuning.update(cfg['tuning'])
    cfg['tuning'] = tuning
    sensor = cfg['sensor']
    sensor['tuning'] = tuning
    sensor['site'] = site
    for old, new in LEGACY_SITE_KEYS.items():
        s = sensor.get('site') or {}
        if old in s and new not in s:
            s[new] = s.pop(old)
    sensor['calibration'].setdefault('inner', None)
    sensor['tag'].setdefault('inner', None)
    # 실험판 D: a run recorded by the base / C version has no inner_tag_from_entrance_m; its tag plane was
    # truck_depth_m - inner_tag_plane_offset_m (the tags on the lattice), so old logs replay with the same geometry.
    if site.get('inner_tag_from_entrance_m') is None and site.get('truck_depth_m') is not None:
        site['inner_tag_from_entrance_m'] = float(site['truck_depth_m']) - float(site.get('inner_tag_plane_offset_m') or 0.0)
    if site.get('inner_tag_ids') and sensor['calibration'].get('inner') is None:
        apply_inner(sensor, site)
    for k in INNER_DEFAULTS:
        site.setdefault(k, None)
    return cfg


INNER_DEFAULTS = (
    'inner_tag_ids',
    'inner_tag_size_m',
    'inner_tag_height_m',
    'inner_tag_spacing_m',
    'inner_tag_lateral_m',
    'inner_tag_from_entrance_m',
    'inner_tag_plate_width_m',
    'lattice_bottom_bar_top_m',
    'lattice_top_bar_bottom_m',
    'lattice_bar_thickness_m',
)


def read_jsonl(path):
    with open(path, 'rb') as fh:
        for line in fh:
            try:
                yield json.loads(line)
            except ValueError:
                continue


def anchor_from_status(req):
    if not req:
        return None
    lp = list(req.get('local_pose') or [0.0, 0.0, 0.0])
    return dict(
        id=int(req.get('id') or 0),
        absolute=bool(req.get('absolute')),
        x=req.get('x', req.get('dx', 0.0)) or 0.0,
        y=req.get('y', req.get('dy', 0.0)) or 0.0,
        h=req.get('h', 0.0) or 0.0,
        dx=req.get('x', req.get('dx', 0.0)) or 0.0,
        dy=req.get('y', req.get('dy', 0.0)) or 0.0,
        local_pose=lp,
    )


def replay_observer(folder, cfg, quiet=False):
    sensor = cfg['sensor']
    observer = Observer(sensor)
    records = list(read_jsonl(folder / 'observations.jsonl'))
    control = [(r['now'], r['status']) for r in read_jsonl(folder / 'control.jsonl') if r.get('status')]
    recorded = {r['capture_id']: r for r in records if 'kind' not in r and 'capture_id' in r}
    pending = collections.deque()
    outputs = []
    ci = 0
    rezero = 0
    anchor_id = 0
    diffs = dict(pose=[], usable=0, stationary=0, n=0, compared=0)
    t0 = time.perf_counter()
    for r in records:
        kind = r.get('kind')
        if kind == 'tag':
            pending.append(r['value'])
            continue
        if kind != 'vio':
            continue
        packet = r['value']
        # the hints the camera worker saw: the newest controller status before this packet arrived
        while ci < len(control) and control[ci][0] <= packet['arrival']:
            st = control[ci][1]
            ci += 1
            if st.get('rezero_request', 0) > rezero:
                rezero = st['rezero_request']
                observer.request_rezero(rezero)
            req = st.get('anchor_request')
            if req and int(req.get('id') or 0) > anchor_id:
                anchor_id = int(req['id'])
                observer.request_anchor(anchor_from_status(req))
        neutral = packet.get('since_neutral_s') is not None
        obs = observer.add_vio(packet, neutral)
        while pending and observer.last_state and pending[0]['stamp'] <= observer.last_state.stamp:
            observer.add_tag(pending.popleft())
        if obs is None:
            continue
        pose = obs.get('pose')
        outputs.append(
            dict(
                capture_id=obs['capture_id'],
                pose=None if pose is None else [round(float(v), 6) for v in pose],
                usable=bool(obs.get('usable')),
                stationary=bool(obs.get('stationary')),
                landmark=obs.get('landmark'),
                tag=(obs.get('tag_fusion') or {}).get('reason'),
            )
        )
        rec = recorded.get(obs['capture_id'])
        diffs['n'] += 1
        if rec is not None:
            diffs['compared'] += 1
            if rec.get('pose') is not None and pose is not None:
                diffs['pose'].append(float(np.max(np.abs(np.array(rec['pose']) - np.array(pose)))))
            diffs['usable'] += int(bool(rec.get('usable')) != bool(obs.get('usable')))
            diffs['stationary'] += int(bool(rec.get('stationary')) != bool(obs.get('stationary')))
    dt = time.perf_counter() - t0
    if not quiet:
        p = diffs['pose']
        print(
            'observer: %d packets in %.2f s (%.2f ms/packet) | compared %d | pose max-abs diff vs recording: med %.4f p90 %.4f max %.4f | usable differs %d, stationary differs %d'
            % (
                diffs['n'],
                dt,
                1e3 * dt / max(1, diffs['n']),
                diffs['compared'],
                np.median(p) if p else float('nan'),
                np.percentile(p, 90) if p else float('nan'),
                max(p) if p else float('nan'),
                diffs['usable'],
                diffs['stationary'],
            )
        )
    return outputs


def replay_controller(folder, cfg, quiet=False):
    observations = {
        r['capture_id']: r for r in read_jsonl(folder / 'observations.jsonl') if 'kind' not in r and 'capture_id' in r
    }
    rows = [r for r in read_jsonl(folder / 'control.jsonl') if r.get('status')]
    if not rows:
        return dict(error='no control rows')
    cut = cut_position(cfg)
    x = Controller(cfg, cut, rows[0]['now'])
    entered = False
    was_waiting = False
    trace = []
    last = None
    agree = 0
    t0 = time.perf_counter()
    o = None
    for i, r in enumerate(rows):
        o = observations.get(r['observation_capture']) or o
        st = r['status']
        enter = False
        if was_waiting and st['state'] != 'WAIT_ENTER' and not entered:
            enter = True
            entered = True
        was_waiting = st['state'] == 'WAIT_ENTER' or (was_waiting and not entered)
        s = x.step(r['now'], o, enter)
        key = (s['state'], s.get('phase'), s['command'])
        if key != last:
            trace.append(dict(now=r['now'], state=key[0], phase=key[1], command=key[2], reason=s.get('reason')))
            last = key
        agree += int(key == (st['state'], st.get('phase'), st['command']))
        # the supervisor ticked every ~10 ms with the same observation between two recorded rows (rows are written per new observation)
        nxt = rows[i + 1]['now'] if i + 1 < len(rows) else None
        t = r['now'] + 0.01
        while nxt is not None and t < nxt - 1e-6 and x.state not in ('FAULT', 'COMPLETE'):
            s = x.step(t, o, False)
            key = (s['state'], s.get('phase'), s['command'])
            if key != last:
                trace.append(dict(now=round(t, 3), state=key[0], phase=key[1], command=key[2], reason=s.get('reason')))
                last = key
            t += 0.01
    dt = time.perf_counter() - t0
    events = [{k: (round(v, 6) if isinstance(v, float) else v) for k, v in e.items()} for e in x.events]
    out = dict(trace=trace, events=events, final_report=x.final_report, state=x.state, reason=x.reason)
    if not quiet:
        recorded_events = next(
            (r['status']['events'] for r in reversed(rows) if r['status'].get('events') is not None), []
        )  # rows carry the list only when it grew (2026-11-01)
        rec_events = collections.Counter(e.get('event') for e in recorded_events)
        mine = collections.Counter(e.get('event') for e in x.events)
        print(
            'controller: %d ticks in %.2f s (%.2f ms/tick) | state/phase/command equal to the recording on %d/%d ticks | end %s/%s (recorded %s/%s)'
            % (
                len(rows),
                dt,
                1e3 * dt / max(1, len(rows)),
                agree,
                len(rows),
                x.state,
                x.reason,
                rows[-1]['status']['state'],
                rows[-1]['status']['reason'],
            )
        )
        print(
            '  events replay vs recorded:',
            {
                k: (mine.get(k, 0), rec_events.get(k, 0))
                for k in sorted(set(mine) | set(rec_events))
                if k not in ('legs', 'step_done')
            },
        )
    return out


def profile(fn, top):
    pr = cProfile.Profile()
    pr.enable()
    out = fn()
    pr.disable()
    s = io.StringIO()
    pstats.Stats(pr, stream=s).sort_stats('tottime').print_stats(top)
    print(s.getvalue())
    return out


def main():
    ap = argparse.ArgumentParser(description='offline replay of a recorded run')
    ap.add_argument('folder')
    ap.add_argument('--signature')
    ap.add_argument('--compare')
    ap.add_argument('--profile', type=int, default=0)
    ap.add_argument('--quiet', action='store_true')
    a = ap.parse_args()
    folder = Path(a.folder)
    cfg = load_cfg(folder)
    if a.profile:
        obs = profile(lambda: replay_observer(folder, cfg, a.quiet), a.profile)
        ctl = profile(lambda: replay_controller(folder, cfg, a.quiet), a.profile)
    else:
        obs = replay_observer(folder, cfg, a.quiet)
        ctl = replay_controller(folder, cfg, a.quiet)
    sig = dict(run=folder.name, observer=obs, controller=ctl)
    if a.signature:
        Path(a.signature).write_text(json.dumps(sig, ensure_ascii=False))
        print('signature:', a.signature)
    if a.compare:
        ref = json.loads(Path(a.compare).read_text())
        same = json.loads(json.dumps(sig)) == ref
        if same:
            print('COMPARE: identical to', a.compare)
        else:

            def first_diff(x, y, path='$'):
                if type(x) != type(y):
                    return path
                if isinstance(x, dict):
                    for k in sorted(set(x) | set(y)):
                        if k not in x or k not in y:
                            return path + '.' + str(k)
                        d = first_diff(x[k], y[k], path + '.' + str(k))
                        if d:
                            return d
                elif isinstance(x, list):
                    if len(x) != len(y):
                        return path + '[len %d vs %d]' % (len(x), len(y))
                    for i, (p, q) in enumerate(zip(x, y)):
                        d = first_diff(p, q, path + '[%d]' % i)
                        if d:
                            return d
                elif x != y:
                    return path + ' (%r vs %r)' % (x, y)
                return None

            print('COMPARE: DIFFERENT, first difference at', first_diff(json.loads(json.dumps(sig)), ref))
            return 1
    return 0


if __name__ == '__main__':
    sys.exit(main())
