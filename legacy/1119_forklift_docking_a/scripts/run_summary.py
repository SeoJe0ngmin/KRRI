"""한 판 폴더의 기록에서 점검할 줄만 뽑아 보여 준다(주행 뒤 현장에서 바로 보는 용도). 장치 · 추가 패키지 없이 돈다.

    python scripts/run_summary.py work/<날짜>/drive/<판 폴더>

끝 자세(태그로 잰 값), 기준 태그가 바뀔 때의 위치 차(줄자 값 점검), 묶음이 처음 맞춘 방향과 그 전 방향의 차, VIO 자 대 태그 자,
겨냥 회전이 실제로 돈 각, 진입 중 감시, 버린 depth 를 한 줄씩 적는다.
"""

import argparse
import json
import math
from pathlib import Path


def rows(path):
    if not path.exists():
        return
    with path.open() as f:
        for line in f:
            try:
                yield json.loads(line)
            except ValueError:
                continue


def fmt(v, unit=''):
    return '-' if v is None else ('%+.3f%s' % (v, unit) if isinstance(v, float) else str(v))


def main():
    ap = argparse.ArgumentParser(description='한 판 요약')
    ap.add_argument('folder')
    a = ap.parse_args()
    folder = Path(a.folder)
    events, status = {}, {}
    for r in rows(folder / 'control.jsonl'):
        status = r.get('status') or status
        for e in (r.get('status') or {}).get('events') or []:
            events[(e.get('t'), e.get('event'), e.get('label'))] = e
    ev = [events[k] for k in sorted(events, key=lambda k: (k[0] is None, k[0]))]
    by = lambda name: [e for e in ev if e.get('event') == name]
    print('state=%s reason=%s events=%d' % (status.get('state'), status.get('reason'), len(ev)))
    fr = status.get('final_report') or {}
    tm = fr.get('tag_measured') or {}
    print(
        'final ended_by=%s remaining_m=%s fork_lateral_m=%s heading_deg=%s'
        % (fr.get('ended_by'), fmt(fr.get('remaining_m')), fmt(fr.get('fork_lateral_m')), fmt(fr.get('heading_deg')))
    )
    print(
        'tag_measured lateral_m=%s heading_err_deg=%s remaining_m=%s frames=%s'
        % (fmt(tm.get('fork_lateral_m')), fmt(tm.get('heading_err_deg')), fmt(tm.get('remaining_m')), tm.get('frames'))
    )
    # 기준 태그가 바뀐 순간의 위치 차: 줄자 값(입구 태그 · 안쪽 태그 · 탑재부 깊이)이 맞으면 작다
    seen = set()
    for o in rows(folder / 'observations.jsonl'):
        for key in ('inner_fusion', 'tag_fusion'):
            f = o.get(key) or {}
            if f.get('reason') == 'landmark_switch' and f.get('residual_m'):
                k = (f.get('switched_from'), f.get('landmark'), round(f['residual_m'][0], 4))
                if k not in seen:
                    seen.add(k)
                    dx, dy = f['residual_m'][:2]
                    print(
                        'landmark_switch %s->%s dx_m=%+.3f dy_m=%+.3f range_m=%s'
                        % (k[0], k[1], dx, dy, fmt(f.get('range_m')))
                    )
    for e in by('bundle_heading_set'):
        print('bundle_heading_set step_deg=%+.2f from=%s' % (e['step_deg'], e.get('from_landmark')))
    for e in by('structure_heading'):
        # 구조물로 잰 방향: 융합 방향과의 차이(묶음이 기준일 때의 값이 두 측정이 얼마나 맞는지를 보여 준다)
        print(
            'structure_heading landmark=%s frames=%d ok=%d minus_fused_deg=%s applied=%d reason=%s'
            % (
                e.get('landmark'),
                e['frames'],
                e['ok_frames'],
                fmt(e.get('minus_fused_deg')),
                e['applied'],
                e.get('reason'),
            )
        )
    for e in by('vio_scale_sample'):
        print(
            'vio_scale_sample ratio=%.3f used=%s scale=%.3f label=%s'
            % (e['ratio'], e['used'], e['scale'], e.get('label'))
        )
    for e in by('final_check_ok') + by('final_small_lateral_aim'):
        print(
            '%s lateral_m=%s heading_err_deg=%s predicted_arrival_m=%s source=%s'
            % (
                e['event'],
                fmt(e.get('lateral_error_m')),
                fmt(e.get('heading_err_deg')),
                fmt(e.get('predicted_arrival_m')),
                e.get('lateral_source'),
            )
        )
    for e in by('best_effort_entry'):
        print(
            'best_effort_entry reason=%s lateral_m=%s aim_deg=%s'
            % (e.get('reason'), fmt(e.get('lateral_error_m')), fmt(e.get('aim_deg')))
        )
    for e in by('step_done'):
        if (
            e.get('label') in ('best_effort_aim', 'final_heading')
            and e.get('target') is not None
            and e.get('end_progress') is not None
        ):
            print(
                'turn label=%s target_deg=%.2f end_deg=%.2f'
                % (e['label'], math.degrees(e['target']), math.degrees(e['end_progress']))
            )
    for e in by('entry_lateral_watch_stop') + by('entry_lateral_watch_recorded'):
        print(
            '%s predicted_arrival_m=%s fork_tip_x_m=%s'
            % (e['event'], fmt(e.get('predicted_arrival_m')), fmt(e.get('fork_tip_x_m')))
        )
    for e in by('entry_depth_ruler_ignored'):
        print(
            'entry_depth_ruler_ignored lattice_remaining_m=%s tag_remaining_m=%s'
            % (fmt(e.get('lattice_remaining_m')), fmt(e.get('tag_remaining_m')))
        )
    names = (
        'bundle',
        'final_back_out',
        'lattice_pair_rejected',
        'depth_low_points',
        'lateral_sources_disagree',
        'distance_sources_disagree',
        'bundle_drive_depth_watch_ignored',
        'depth_unavailable_fallback',
        'turn_suppressed_inside',
    )
    print('counts ' + ' '.join('%s=%d' % (n, len(by(n))) for n in names))


if __name__ == '__main__':
    main()
