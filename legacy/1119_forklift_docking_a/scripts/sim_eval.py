#!/usr/bin/env python3
"""도킹 제어기 폐루프 시뮬레이터 — 현장 잡음을 넣고 출발 자리를 바꿔 여러 번 돌려, 같은 조건끼리 판을 짝 비교한다.

  python scripts/sim_eval.py run --runs 200 --scenario afternoon --out work/sim/<이름>.jsonl [--set 키=값 ...]
  python scripts/sim_eval.py compare work/sim/A.jsonl work/sim/B.jsonl

run      실제 현장 설정(configs/common.json · mount.json)으로 Controller 를 돌린다. 운전 코드는 그대로 쓰고, 차·센서·추정만 흉내 낸다.
         --set 은 cfg['tuning'] 덮어쓰기(판 비교용). 같은 --seed 와 번호면 출발 자리 · 줄자 오차 · 쏠림이 같다(짝 비교).
compare  두 파일의 같은 번호끼리 짝 부트스트랩으로 차이와 95 % 구간을 낸다.
시뮬레이터 숫자는 현장 성능이 아니라 판끼리의 순위 비교용이다. 잡음 값과 출처는 NOISE 에 있다.
"""

import argparse
import json
import math
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from docking.config import load  # noqa: E402
from docking.controller import Controller  # noqa: E402
from docking.geometry import wrap, front, arrival_y, cut_position  # noqa: E402

DT = 0.1  # 감독 틱(초) 기본값, 테스트 시뮬레이터와 같음 — 실제 값은 NOISE['tick_s']
NOISE = dict(
    # 출발 자리: 테스트의 전체 흐름 예와 비슷한 범위(가정)
    start_x=(-8.0, -6.0),
    start_y=(-0.6, 0.6),
    start_h_deg=(-10.0, 10.0),
    # 차: 속도 · 회전은 테스트 시뮬레이터 값, 개체 차이 ±5 %(가정)
    forward_mps=0.29,
    fine_mps=0.15,
    rotate_dps=8.5,
    rotate_slow_dps=12.0,
    rotate_fine_dps=4.0,
    speed_spread=0.05,
    # 직진 쏠림: 2026-10-20 포크 앞 장착 3회 −1.7 / −4.2 / −3.0°
    drift_deg=(-3.0, 1.0),
    # 정지 명령 뒤 더 밀리는 틱 수 0~2(가정)
    coast_ticks=(0, 2),
    # VIO: 거리 배율 오차 1 %, 방향은 회전량의 0.3 % + 1 m 당 0.05°(가정 — 10-30 VIO 는 정상)
    vio_scale=0.01,
    vio_rot_frac=0.003,
    vio_h_per_m_deg=0.05,
    # 입구 태그: 시작 방향 오차 1°, 태그컷 전 영점 뒤 0.5°(가정), 방위 0.05°, 거리 0.5 %(가정)
    entrance_h0_deg=1.0,
    entrance_rezero_deg=0.5,
    bearing_deg=0.05,
    range_frac=0.005,
    # 묶음 방향 한 장면 잡음(°): 1.6 m 0.2~0.3, 3.1 m 0.5~1, 4.9 m 약 3 — 2026-11-01 합성 렌더 검증의 가운데 값
    bundle_h_table=((1.6, 0.25), (3.1, 0.75), (4.9, 3.0)),
    # 창 크기: 정지 창 중앙값은 한 장면의 1/3 수준(10 프레임), 이동 중은 3 장
    window_n_still=9,
    window_n_moving=3,
    # 줄자 오차 1 cm, 태그 평판 기울기 0.3°(가정)
    tape_m=0.01,
    plate_yaw_deg=0.3,
    # 컬러 · depth 시야 반각(°): 진입 끝 화면 좌우 ±34°, depth_hfov_deg 80
    colour_half_deg=34.0,
    depth_half_deg=40.0,
    # depth 잡음(가정) · 문 4 m, 격자 6.5 m 까지
    depth_x_m=0.02,
    depth_y_m=0.01,
    doors_max_m=4.0,
    lattice_max_m=6.5,
    # 격자 칸 피치 0.405 m — 10-30 오후 4판에서 옆 칸을 잡음
    lattice_pitch_m=0.405,
    # 감독 틱(초)
    tick_s=DT,
    # VIO 거리 배율의 치우침(가정 0; 2026-10-30 정오 판은 −0.10 ~ −0.21)
    vio_scale_bias=0.0,
    # 정지 중 태그 창: False = 틱마다 새 창 하나(예전), True = 틱마다 한 장면을 재고 최근 0.5 s 의 가운데 값(이웃 틱이 겹침)
    rest_rolling=False,
    # 가속 · 정지 모형: None = 켜자마자 제 속도, 정지 명령 뒤 coast_ticks 틱 더 감(예전). PRESETS['field'] 가 현장 실측 모형
    plant=None,
    # 구조물(적재함 모서리가 모이는 점)로 잰 방향 — 모두 가정: 한 장면의 흔들림(°, None = 측정 없음)과 판마다 고정된 치우침(°)
    structure_h_deg=None,
    structure_bias_deg=0.0,
)
# 10-30 현장 step_done 실측(work/analysis/20261118/turns): 속도, 정지 명령 뒤 밀린 양 ÷ 속도 = 밀림 시간(사분위 범위).
# 가속 구간은 약한 회전의 작은 목표(1~1.5°)에서 밀림이 절반인 것에 맞춘 값, 직진 가속 1 s 는 가정
PRESETS = dict(
    field=dict(
        tick_s=0.05,
        forward_mps=0.231,
        fine_mps=0.142,
        rotate_fine_dps=3.6,
        plant=dict(
            ramp_turn_s=2.0,
            ramp_drive_s=1.0,
            coast_drive_s=(0.45, 0.85),
            coast_turn_s=(0.35, 0.55),
            coast_turn_fine_s=(0.0, 0.7),
        ),
    ),
)
# 컬러 카메라 모델: 10-30 기록 config_used.json(장치 공장값, 1280×720) — load() 는 실행 때 장치에서 채우므로 비어 있다
COLOUR_MODEL = dict(
    width=1280,
    height=720,
    fx=906.1278076171875,
    fy=905.9816284179688,
    cx=637.9266967773438,
    cy=380.8492126464844,
    coeffs=[0.0] * 5,
    distortion='inverse_brown_conrady',
    source='sim',
)
SCENARIOS = dict(
    # 오후: depth 가 살아 있고, 정지 창마다 격자 쌍이 옆 칸일 확률(가정 — 10-30 오후 4판은 대부분 틀림)
    afternoon=dict(depth=True, alias_p=0.3, line=False),
    # 정오: depth 전멸(10-30 정오 9판)
    noon=dict(depth=False, alias_p=0.0, line=False),
    # 깨끗한 depth: 비교 기준용
    clean=dict(depth=True, alias_p=0.0, line=True),
    # 안쪽 태그가 안 보이는 오후(가림 · 떨어짐): 입구 태그가 잘린 뒤 depth 가 기준 — 10-30 오후 판과 같은 상황
    notag=dict(depth=True, alias_p=0.3, line=False, inner=False),
    # 정오에 안쪽 태그도 안 보임: 입구 태그가 잘린 뒤 VIO 만으로 진입 — 10-30 정오 판과 같은 상황
    noon_notag=dict(depth=False, alias_p=0.0, line=False, inner=False),
)


def rot(h, v):
    c, s = math.cos(h), math.sin(h)
    return np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])


def compose(p, inc):
    q = np.array(p, float).copy()
    q[:2] += rot(p[2], inc[:2])
    q[2] = wrap(p[2] + inc[2])
    return q


def increment(a, b):
    """a 에서 b 로 가는 몸체 좌표 증분."""
    d = rot(-a[2], np.array(b[:2]) - np.array(a[:2]))
    return np.array([d[0], d[1], wrap(b[2] - a[2])])


def bundle_sigma_deg(r):
    tab = NOISE['bundle_h_table']
    if r <= tab[0][0]:
        return max(0.1, tab[0][1] * r / tab[0][0])
    xs, ys = [a for a, _ in tab], [b for _, b in tab]
    return float(np.interp(r, xs, ys)) if r <= xs[-1] else ys[-1] * r / xs[-1]


class World:
    """참값(차) · VIO · 융합 추정 · 센서를 한 틱씩 흉내 낸다. 제어기에는 Observer 가 내는 모양의 관측 dict 를 준다."""

    def __init__(self, cfg, ctrl, scen, run_rng, tick_rng):
        n = NOISE
        self.cfg, self.x, self.scen, self.r = cfg, ctrl, scen, tick_rng
        site = cfg['site']
        cal = cfg['sensor']['calibration']
        inner = cal.get('inner') or {}
        u = run_rng.uniform
        # 짝 비교: 출발 · 줄자 · 쏠림 · 속도는 run_rng 에서 먼저 뽑는다
        self.T = np.array([u(*n['start_x']), u(*n['start_y']), math.radians(u(*n['start_h_deg']))])
        self.drift = math.radians(run_rng.normal(*n['drift_deg']))
        self.speed = 1.0 + run_rng.normal(0, n['speed_spread'])
        self.turn_k = 1.0 + run_rng.normal(0, n['speed_spread'])
        self.vio_s = n['vio_scale_bias'] + run_rng.normal(0, n['vio_scale'])
        tape = lambda: float(run_rng.normal(0, n['tape_m']))  # noqa: E731
        self.e_tag_cfg = np.array(
            [float(site.get('entrance_tag_plane_offset_m') or 0.0), float(site['entrance_tag_lateral_m'])]
        )
        self.e_tag_true = self.e_tag_cfg + np.array([tape(), tape()])
        self.D_cfg = float(site.get('truck_depth_m') or 0.0)
        self.D_true = self.D_cfg + tape()
        self.inner_cfg = None
        if inner:
            px, yc, half = (
                float(inner['plane_x_m']),
                float(inner.get('lateral_m') or 0.0),
                float(inner['spacing_m']) / 2,
            )
            off = px - self.D_cfg
            self.inner_cfg = dict(x=px, y=yc, half=half)
            self.inner_true = dict(
                x=self.D_true + off + tape(),
                y=yc + tape(),
                half=half + tape() / 2,
                yaw=math.radians(run_rng.normal(0, n['plate_yaw_deg'])),
            )
        self.lat_off_cfg = float(site.get('lattice_centre_offset_m') or 0.0)
        self.lat_off_true = self.lat_off_cfg + tape()
        self.h0_err = math.radians(run_rng.normal(0, n['entrance_h0_deg']))
        self.coast_rng = np.random.default_rng(run_rng.integers(1 << 31))
        # 추정 상태
        self.L = np.zeros(3)  # VIO 국소 자세
        self.S = self.T.copy()
        self.S[2] = wrap(self.T[2] + self.h0_err)
        self.S[:2] = (
            self.tag_position(self.e_tag_cfg, self.e_tag_true, self.S[2], n_eff=9)
            if self.entrance_visible()
            else self.S[:2]
        )
        self.t = 0.0
        self.tick = 0
        self.last_fix = 0.0
        self.landmark = 'entrance'
        self.rezero = 0
        self.anchor_done = 0
        self.prev_cmd = 'stop'
        self.coast_left = 0
        self.coast_cmd = None
        self.ramp = 0.0  # 가속 구간에서 제 속도의 몇 배인지(현장 정지 모형)
        self.roll = []  # 정지 중 최근 0.5 s 의 한 장면 측정(rest_rolling)
        self.ramp_cmd = None
        self.alias = 0.0
        self.was_moving = False
        self.inner_seen_ever = False
        self.structure_rng = None  # 구조물 방향 측정의 잡음(다른 난수 흐름을 건드리지 않게 따로 둔다)
        self.structure_rows = []
        self.structure_obs = None
        self.structure_total = 0.0

    # ---- 보이는지
    def bearing(self, p):
        v = rot(-self.T[2], np.array(p) - self.T[:2])
        return v, math.degrees(math.atan2(v[1], v[0]))

    def entrance_visible(self):
        if self.T[0] >= self.x.cut:
            return False
        v, b = self.bearing(self.e_tag_true)
        return v[0] > 0.3 and abs(b) <= NOISE['colour_half_deg']

    def inner_visible(self):
        if not self.inner_cfg or not self.scen.get('inner', True):
            return []
        it = self.inner_true
        seen = []
        for k, sg in ((2, 1.0), (3, -1.0)):
            p = np.array([it['x'], it['y']]) + rot(it['yaw'], np.array([0.0, sg * it['half']]))
            v, b = self.bearing(p)
            if 0.5 <= v[0] <= 7.0 and abs(b) <= NOISE['colour_half_deg'] and abs(math.cos(self.T[2])) > 0.2:
                seen.append((k, p, v))
        return seen

    # ---- 태그 관측 → 위치
    def measured_vector(self, p_true, n_eff):
        """카메라→태그 몸체 벡터: 방위는 정확, 거리는 비율 잡음."""
        v = rot(-self.T[2], np.array(p_true) - self.T[:2])
        r, b = float(np.hypot(*v)), math.atan2(v[1], v[0])
        k = 1.0 / math.sqrt(n_eff)
        r *= 1.0 + self.r.normal(0, NOISE['range_frac'] * k)
        b += math.radians(self.r.normal(0, NOISE['bearing_deg'] * k))
        return r * np.array([math.cos(b), math.sin(b)])

    def tag_position(self, p_cfg, p_true, h_used, n_eff):
        """방위·거리 앵커: 설정된 태그 자리에서 (쓰는 방향으로 돌린) 관측 벡터를 뺀다 — 방향 오차가 좌우 오차로 번진다."""
        return np.array(p_cfg) - rot(h_used, self.measured_vector(p_true, n_eff))

    def bundle(self, n_eff):
        """두 장 묶음: 차로 축 기준 방향 + 태그 가운데 몸체 벡터(한 평면 표적 — 방향 잡음이 거리에 따라 커짐)."""
        it, ic = self.inner_true, self.inner_cfg
        centre = np.array([it['x'], it['y']])
        v = self.measured_vector(centre, n_eff)
        r = float(np.hypot(*v))
        h = wrap(self.T[2] - it['yaw'] + math.radians(self.r.normal(0, bundle_sigma_deg(r) / math.sqrt(n_eff))))
        S_xy = np.array([ic['x'], ic['y']]) - rot(h, v)
        return h, S_xy, v

    # ---- 한 틱
    def step_plant(self, cmd):
        n = NOISE
        DT = n['tick_s']
        pl = n['plant']
        moving_cmd = cmd if cmd != 'stop' else None
        coasting = False
        if cmd == 'stop' and self.prev_cmd != 'stop':
            if pl:
                kind = (
                    'turn_fine' if self.prev_cmd.endswith('fine2') else 'turn' if 'rotate' in self.prev_cmd else 'drive'
                )
                self.coast_left = int(round(self.coast_rng.uniform(*pl['coast_%s_s' % kind]) / DT))
            else:
                self.coast_left = int(self.coast_rng.integers(n['coast_ticks'][0], n['coast_ticks'][1] + 1))
            self.coast_cmd = self.prev_cmd
        if moving_cmd is None and self.coast_left > 0:
            moving_cmd = self.coast_cmd
            self.coast_left -= 1
            coasting = True
        self.prev_cmd = cmd
        if moving_cmd is None:
            self.ramp, self.ramp_cmd = 0.0, None
            return False, np.zeros(3)
        k = 1.0
        if pl:
            # 켠 뒤 제 속도까지 고르게 빨라지고, 정지 명령 뒤에는 그때 속도로 밀림 시간만큼 더 간다
            if moving_cmd != self.ramp_cmd:
                self.ramp, self.ramp_cmd = 0.0, moving_cmd
            if not coasting:
                self.ramp = min(1.0, self.ramp + DT / pl['ramp_turn_s' if 'rotate' in moving_cmd else 'ramp_drive_s'])
            k = self.ramp
        dist, turn = 0.0, 0.0
        c = moving_cmd
        if c == 'forward':
            dist = n['forward_mps'] * DT * self.speed
        elif c == 'backward':
            dist = -n['forward_mps'] * DT * self.speed
        elif c in ('forward_fine', 'backward_fine'):
            dist = (1 if c == 'forward_fine' else -1) * n['fine_mps'] * DT * self.speed
        elif c.endswith('fine2'):
            turn = (1 if 'left' in c else -1) * math.radians(n['rotate_fine_dps']) * DT * self.turn_k
        elif c.startswith('rotate'):
            turn = (
                (1 if 'left' in c else -1)
                * math.radians(n['rotate_slow_dps'] if 'slow' in c else n['rotate_dps'])
                * DT
                * self.turn_k
            )
        dist, turn = dist * k, turn * k
        old = self.T.copy()
        piv = np.array(self.cfg['pivot'], float)
        if turn:
            centre = self.T[:2] + rot(self.T[2], piv)
            self.T[2] = wrap(self.T[2] + turn)
            self.T[:2] = centre - rot(self.T[2], piv)
        if dist:
            self.T[:2] += dist * np.array([math.cos(self.T[2] + self.drift), math.sin(self.T[2] + self.drift)])
        return True, increment(old, self.T)

    def step_estimate(self, moving, inc_true):
        n = NOISE
        if moving:
            ds, dh = float(np.hypot(*inc_true[:2])), abs(inc_true[2])
            herr = math.radians(self.r.normal(0, n['vio_rot_frac'] * math.degrees(dh) + n['vio_h_per_m_deg'] * ds))
            inc = np.array([inc_true[0] * (1 + self.vio_s), inc_true[1] * (1 + self.vio_s), wrap(inc_true[2] + herr)])
            self.L = compose(self.L, inc)
            self.S = compose(self.S, inc)
        # 앵커 요청(depth 재앵커): Observer 처럼 바로 옮긴다
        req = self.x.anchor_request
        if req and req['id'] > self.anchor_done and not req.get('absolute'):
            self.S[0] += float(req.get('dx') or 0.0)
            self.S[1] += float(req.get('dy') or 0.0)
            self.anchor_done = req['id']
        # 영점 요청(태그컷 전): 정지 중이면 적용
        if not moving and self.x.rezero_request > self.rezero:
            self.rezero = self.x.rezero_request
            self.S[2] = wrap(self.T[2] + math.radians(self.r.normal(0, n['entrance_rezero_deg'])))
            self.structure_total = 0.0
        # 태그 창: 0.5 s 마다(이동 중), 정지 중엔 매 틱
        roll = bool(n['rest_rolling']) and not moving
        if moving:
            self.roll = []
        if not moving or self.tick % max(1, round(0.5 / n['tick_s'])) == 0:
            n_eff = 1 if roll else (n['window_n_still'] if not moving else n['window_n_moving'])
            inner = self.inner_visible()
            rb = float(self.cfg['tuning'].get('inner_bundle_max_range_m', 3.5))
            kind = None
            if len(inner) == 2:
                self.inner_seen_ever = True
                h, S_xy, v = self.bundle(n_eff)
                if v[0] <= rb:
                    kind = 'bundle'
                    self.S[:2] = S_xy
                    if not moving:
                        self.S[2] = h  # 정지 중 묶음 방향으로 오프셋을 그대로 맞춤
                        self.structure_total = 0.0
            if kind is None and self.entrance_visible():
                kind = 'entrance'
                self.S[:2] = self.tag_position(self.e_tag_cfg, self.e_tag_true, self.S[2], n_eff)
            if kind is None and inner:
                k, p_true, _ = inner[0]
                ic = self.inner_cfg
                p_cfg = np.array([ic['x'], ic['y'] + (ic['half'] if k == 2 else -ic['half'])])
                kind = 'inner'
                self.S[:2] = self.tag_position(p_cfg, p_true, self.S[2], n_eff)
            if kind and roll:
                # 한 장면씩 재서 최근 0.5 s 의 가운데 값을 쓴다 — 이웃 틱의 값이 겹쳐, 오래 서서 모아야 흔들림이 준다
                if self.roll and self.roll[-1][0] != kind:
                    self.roll = []
                self.roll.append((kind, self.S[:2].copy(), float(self.S[2])))
                self.roll = self.roll[-max(1, round(0.5 / n['tick_s'])) :]
                self.S[:2] = np.median([r[1] for r in self.roll], axis=0)
                if kind == 'bundle':
                    self.S[2] = wrap(float(np.median(np.unwrap([r[2] for r in self.roll]))))
            if kind:
                self.landmark = kind
                self.last_fix = self.t
        self.step_structure(moving)

    def step_structure(self, moving):
        """구조물로 잰 방향(가정한 측정): 서 있는 동안 structure_period_s 마다 한 장면, 융합과 같은 규칙으로 방향을 조금 옮긴다."""
        n, t = NOISE, self.cfg['tuning']
        if n['structure_h_deg'] is None or self.structure_rng is None or not t.get('structure_heading'):
            return
        if moving or self.tick % max(1, round(float(t['structure_period_s']) / n['tick_s'])):
            return
        measured = wrap(
            self.T[2]
            + math.radians(n['structure_bias_deg'])
            + math.radians(self.structure_rng.normal(0, n['structure_h_deg']))
        )
        obs = dict(
            stamp=self.t,
            ok=True,
            heading_deg=math.degrees(measured),
            minus_fused_deg=math.degrees(wrap(measured - self.S[2])),
            applied=False,
            landmark=self.landmark,
        )
        self.structure_rows.append((self.t, wrap(measured - self.L[2]), float(self.L[0]), float(self.L[1])))
        self.structure_rows = [r for r in self.structure_rows if self.t - r[0] <= t['structure_window_s']]
        rows = np.unwrap([r[1] for r in self.structure_rows])
        centre = float(np.median(rows))
        step = wrap(centre - wrap(self.S[2] - self.L[2]))
        if not t.get('structure_heading_apply'):
            obs['reason'] = 'record_only'
        elif self.landmark == 'bundle':
            obs['reason'] = 'bundle_is_reference'
        elif self.S[0] < -t['structure_apply_range_m']:
            obs['reason'] = 'too_far'
        elif len(rows) < t['structure_min_frames']:
            obs['reason'] = 'collecting'
        elif max(math.hypot(r[2] - self.L[0], r[3] - self.L[1]) for r in self.structure_rows) < (
            t['structure_min_baseline_m']
        ):
            obs['reason'] = 'one_viewpoint'
        elif math.degrees(1.4826 * float(np.median(np.abs(rows - centre)))) > t['structure_consistency_deg']:
            obs['reason'] = 'frames_disagree'
        elif abs(math.degrees(step)) > t['structure_heading_max_step_deg']:
            obs['reason'] = 'step_too_large'
        elif abs(math.degrees(self.structure_total + t['structure_heading_gain'] * step)) > (
            t['structure_heading_max_total_deg']
        ):
            obs['reason'] = 'total_limit'
        else:
            self.S[2] = wrap(self.S[2] + t['structure_heading_gain'] * step)
            self.structure_total += t['structure_heading_gain'] * step
            obs.update(applied=True, reason='applied')
        self.structure_obs = obs

    def depth(self):
        n, sc = NOISE, self.scen
        out = dict(
            ok=False,
            stamp=self.t,
            method='window',
            doors=dict(ok=False, reason='none', n=0),
            lattice_pair=None,
            lattice_line=None,
        )
        if not sc['depth']:
            out['doors'] = dict(ok=False, reason='no_points_in_band', n=0)
            out['lattice_pair'] = dict(ok=False, reason='no_points_in_range', n=0, members=0)
            out['lattice_line'] = dict(ok=False, reason='no_points_in_range', n=0)
            return out
        W = float(self.cfg['site']['truck_inner_width_m'])
        hd = n['depth_half_deg']
        vl, bl = self.bearing((0.0, W / 2))
        vr, br = self.bearing((0.0, -W / 2))
        vc, _ = self.bearing((0.0, 0.0))
        if 0.3 <= vc[0] <= n['doors_max_m'] and abs(bl) <= hd and abs(br) <= hd:
            x = vc[0] + self.r.normal(0, n['depth_x_m'])
            y = vc[1] + self.r.normal(0, n['depth_y_m'])
            doors = dict(
                ok=True,
                reason='doors',
                x_body_m=float(x),
                x0_m=float(x),
                lateral_body_m=float(y),
                width_m=W,
                pair_heading_deg=-math.degrees(self.T[2]),
                left_body=[float(x), float(y + W / 2)],
                right_body=[float(x), float(y - W / 2)],
                n_left=40,
                n_right=40,
                n=80,
            )
            out.update(ok=True, reason='doors', x_body_m=float(x), lateral_body_m=float(y), width_m=W, doors=doors)
        Wp = float(self.cfg['site'].get('lattice_pair_width_m') or 0.0)
        if self.D_cfg and Wp:
            pc = np.array([self.D_true, -self.lat_off_true])
            v, _ = self.bearing(pc)
            _, b1 = self.bearing(pc + np.array([0.0, Wp / 2]))
            _, b2 = self.bearing(pc + np.array([0.0, -Wp / 2]))
            if 0.3 <= v[0] <= n['lattice_max_m'] and abs(b1) <= hd and abs(b2) <= hd:
                y = v[1] + self.alias + self.r.normal(0, n['depth_y_m'])
                x = v[0] + self.r.normal(0, n['depth_x_m'])
                out['lattice_pair'] = dict(
                    ok=True,
                    reason='lattice_pair',
                    x_body_m=float(x),
                    lateral_body_m=float(y),
                    width_m=Wp,
                    left_body=[float(x), float(y + Wp / 2)],
                    right_body=[float(x), float(y - Wp / 2)],
                    n_left=30,
                    n_right=30,
                    members=5,
                    n=150,
                )
            else:
                out['lattice_pair'] = dict(ok=False, reason='no_points_in_range', n=0, members=0)
            if sc['line'] and 0.3 <= v[0] <= n['lattice_max_m']:
                out['lattice_line'] = dict(
                    ok=True,
                    reason='line',
                    x_body_m=float(
                        (self.D_true - self.T[0]) / max(0.2, math.cos(self.T[2])) + self.r.normal(0, n['depth_x_m'])
                    ),
                    heading_err_deg=float(math.degrees(self.T[2])),
                    slope=math.tan(self.T[2]),
                    span_m=1.5,
                    bins=28,
                    n=300,
                    residual_m=0.01,
                )
            else:
                out['lattice_line'] = dict(ok=False, reason='no_points_in_range', n=0)
        return out

    def observation(self, moving):
        cfg = self.cfg
        S = self.S
        o = dict(
            capture_id=self.t,
            arrival=self.t,
            clock_fault=False,
            sensor_ready=True,
            imu_bad=False,
            imu=dict(calibration=dict(state='READY')),
            local_valid=True,
            usable=True,
            valid=True,
            pose=[float(v) for v in S],
            local_pose=[float(v) for v in self.L],
            stationary=not moving,
            motion_epoch=0,
            tracking_generation=0,
            blind_age_s=float(self.t - self.last_fix),
            landmark=self.landmark,
            emitter_on=True,
            anchor_applied=self.anchor_done,
            transport_lag_s=0.0,
            sigma=None,
        )
        ev = self.entrance_visible()
        o['tag'] = dict(ok=ev, detected=ev, reason='accepted' if ev else 'missing_or_stale')
        o['fork_uncertainty'] = dict(
            position=front(S, cfg).tolist(),
            std_m=[0.0001, 0.0001],
            predicted_arrival_y_m=arrival_y(S, cfg),
            predicted_arrival_std_m=0.0001,
        )
        o['heading_rezero'] = dict(count=self.rezero, requested=self.x.rezero_request, info=None)
        inner = self.inner_visible()
        if inner:
            it = dict(stamp=self.t, seen=[k for k, _, _ in inner], bundle=None, single=None)
            if len(inner) == 2:
                h, _, v = self.bundle(1)
                it['bundle'] = dict(
                    ok=True,
                    stamp=self.t,
                    x_body_m=float(v[0]),
                    lateral_body_m=float(v[1]),
                    heading_err_deg=float(math.degrees(h)),
                )
            o['inner_tags'] = it
        o['depth'] = self.depth()
        if self.structure_obs is not None:
            o['structure'] = dict(self.structure_obs)
        return o

    def new_alias(self):
        p = self.scen['alias_p']
        self.alias = float(self.r.choice([-1.0, 1.0]) * NOISE['lattice_pitch_m']) if p and self.r.random() < p else 0.0

    def run(self, seconds):
        x = self.x
        while self.t < seconds and x.state not in ('COMPLETE', 'FAULT'):
            self.t = round(self.t + NOISE['tick_s'], 3)
            self.tick += 1
            moving, inc = self.step_plant(x.command)
            if moving != self.was_moving:
                self.new_alias()  # 정지 창 · 이동 구간마다 격자 옆 칸 여부를 새로
                self.was_moving = moving
            self.step_estimate(moving, inc)
            x.step(self.t, self.observation(moving), False)


def one_run(args):
    i, seed, scenario, overrides, seconds, noise = args
    NOISE.update({k: tuple(v) if isinstance(v, list) else v for k, v in noise.items()})
    cfg = load()
    cfg['tuning'].update(preparation_s=1.0, baseline_s=0.0)
    cfg['sensor']['calibration']['camera_model'] = cfg['sensor']['calibration'].get('camera_model') or dict(
        COLOUR_MODEL
    )
    for k, v in overrides.items():
        cfg['tuning'][k] = v
    ctrl = Controller(cfg, cut_position(cfg), 0.0)
    ctrl.state = 'PLAN'
    ctrl.mission_start = 0.0
    ctrl.baseline_done = True
    run_rng = np.random.default_rng([seed, i, 1])
    tick_rng = np.random.default_rng([seed, i, 2])
    w = World(cfg, ctrl, SCENARIOS[scenario], run_rng, tick_rng)
    if NOISE['structure_h_deg'] is not None:
        w.structure_rng = np.random.default_rng([seed, i, 3])
    start = w.T.copy()
    w.run(seconds)
    tip = front(w.T, cfg)
    names = [e['event'] for e in ctrl.events]
    fr = ctrl.final_report or {}
    tm = fr.get('tag_measured') or {}
    count = lambda n: names.count(n)  # noqa: E731
    return dict(
        i=i,
        scenario=scenario,
        state=ctrl.state,
        reason=ctrl.reason,
        t_s=w.t,
        start=[float(start[0]), float(start[1]), math.degrees(start[2])],
        drift_deg=math.degrees(w.drift),
        # 참값(현장의 줄자·태그가 아니라 실제 자리)
        fork_lateral_m=float(tip[1]),
        heading_deg=float(math.degrees(w.T[2])),
        travel_heading_deg=float(math.degrees(w.T[2] + w.drift)),  # 직진할 때 실제로 가는 방향
        remaining_m=float(cfg['target_front_x'] - tip[0]),
        # 제어기가 믿은 값 · 태그로 잰 값
        believed_fork_lateral_m=fr.get('fork_lateral_m'),
        tag_fork_lateral_m=tm.get('fork_lateral_m'),
        tag_heading_deg=tm.get('heading_err_deg'),
        tag_remaining_m=tm.get('remaining_m'),
        sidesteps=count('sidestep'),
        bundles=count('bundle'),
        back_outs=count('final_back_out'),
        depth_drive_stops=count('bundle_drive_stopped_on_depth'),
        disagree=count('lateral_sources_disagree'),
        anchors=count('anchor_requested'),
        best_effort=count('best_effort_entry'),
        fallback=count('depth_unavailable_fallback'),
        inner_reference=count('inner_tags_reference'),
        exhausted=count('final_corrections_exhausted'),
        no_room=count('final_no_room_to_correct'),
        final_checks=count('final_check_start'),
        watch_stops=count('entry_lateral_watch_stop'),
        watch_recorded=count('entry_lateral_watch_recorded'),
        alias_rejected=count('lattice_pair_rejected'),
        structure_applied=sum(int(e.get('applied') or 0) for e in ctrl.events if e['event'] == 'structure_heading'),
        # 진입 시작 때 제어기가 본 좌우(진입 중 생긴 오차와 가르려고)
        entry_seen_lateral_m=next((e.get('lateral_error_m') for e in ctrl.events if e['event'] == 'entry'), None),
        entry_distance_m=next((e.get('distance_m') for e in ctrl.events if e['event'] == 'entry'), None),
        fault_detail=(ctrl.events[-1] if ctrl.state == 'FAULT' and ctrl.events else None),
        best_effort_reason=next(
            (e.get('reason') for e in reversed(ctrl.events) if e['event'] == 'best_effort_entry'), None
        ),
        best_effort_lateral_m=next(
            (e.get('lateral_error_m') for e in reversed(ctrl.events) if e['event'] == 'best_effort_entry'), None
        ),
        lattice_ruler=fr.get('lattice_ruler'),
        entry_lattice_remaining_m=fr.get('lattice_remaining_m'),
        entry_tag_remaining_m=fr.get('tag_remaining_m'),
    )


def ok(r, lat=0.03, rem=0.05):
    """좌우 3 cm · 남은 거리 5 cm 안에서 끝남(방향은 따로 본다 — 쏠림 보정 때문에 차체 방향과 가는 방향이 다르다)."""
    return r['state'] == 'COMPLETE' and abs(r['fork_lateral_m']) <= lat and abs(r['remaining_m']) <= rem


def summary(rows):
    a = lambda k: np.array([abs(r[k]) for r in rows], float)  # noqa: E731
    lat, head, rem = a('fork_lateral_m'), a('heading_deg'), a('remaining_m')
    trav = a('travel_heading_deg')
    return dict(
        n=len(rows),
        complete=sum(r['state'] == 'COMPLETE' for r in rows),
        fault=sum(r['state'] == 'FAULT' for r in rows),
        ok_rate=float(np.mean([ok(r) for r in rows])),
        lat_median_m=float(np.median(lat)),
        lat_p90_m=float(np.percentile(lat, 90)),
        lat_within_3cm=float(np.mean(lat <= 0.03)),
        head_median_deg=float(np.median(head)),
        head_p90_deg=float(np.percentile(head, 90)),
        travel_head_median_deg=float(np.median(trav)),
        travel_head_p90_deg=float(np.percentile(trav, 90)),
        rem_median_m=float(np.median(rem)),
        rem_p90_m=float(np.percentile(rem, 90)),
        bundles_mean=float(np.mean([r['bundles'] for r in rows])),
        back_outs_mean=float(np.mean([r['back_outs'] for r in rows])),
        depth_drive_stops_mean=float(np.mean([r['depth_drive_stops'] for r in rows])),
        time_median_s=float(np.median([r['t_s'] for r in rows])),
    )


def parse_set(items):
    out = {}
    for s in items or []:
        k, v = s.split('=', 1)
        out[k] = json.loads(v)
    return out


def cmd_run(a):
    overrides = parse_set(a.set)
    noise = dict(PRESETS[a.preset]) if a.preset else {}
    noise.update(parse_set(a.noise))
    jobs = [(i, a.seed, a.scenario, overrides, a.seconds, noise) for i in range(a.runs)]
    t0 = time.perf_counter()
    with ProcessPoolExecutor(max_workers=a.workers) as ex:
        rows = list(ex.map(one_run, jobs, chunksize=4))
    out = Path(a.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    with out.open('w') as f:
        f.write(
            json.dumps(dict(meta=dict(seed=a.seed, scenario=a.scenario, set=overrides, runs=a.runs, noise_set=noise)))
            + '\n'
        )
        for r in rows:
            f.write(json.dumps(r) + '\n')
    s = summary(rows)
    print(
        ' '.join(f'{k}={v:.4g}' if isinstance(v, float) else f'{k}={v}' for k, v in s.items()),
        f'wall_s={time.perf_counter() - t0:.1f}',
    )


def read(path):
    lines = Path(path).read_text().splitlines()
    return json.loads(lines[0])['meta'], {r['i']: r for r in map(json.loads, lines[1:])}


def cmd_compare(a):
    ma, A = read(a.a)
    mb, B = read(a.b)
    if (ma['seed'], ma['scenario'], ma.get('noise_set')) != (mb['seed'], mb['scenario'], mb.get('noise_set')):
        raise SystemExit('seed · scenario · noise 가 다른 두 파일은 짝 비교하지 않습니다')
    ids = sorted(set(A) & set(B))
    rng = np.random.default_rng(0)
    metrics = dict(
        ok_rate=lambda r: float(ok(r)),
        lat_abs_m=lambda r: abs(r['fork_lateral_m']),
        head_abs_deg=lambda r: abs(r['heading_deg']),
        travel_head_abs_deg=lambda r: abs(r['travel_heading_deg']),
        rem_abs_m=lambda r: abs(r['remaining_m']),
        bundles=lambda r: float(r['bundles']),
        time_s=lambda r: float(r['t_s']),
    )
    print(f'n={len(ids)} a={a.a} b={a.b}')
    for name, f in metrics.items():
        va = np.array([f(A[i]) for i in ids])
        vb = np.array([f(B[i]) for i in ids])
        d = vb - va
        boot = [d[rng.integers(0, len(d), len(d))].mean() for _ in range(2000)]
        lo, hi = np.percentile(boot, [2.5, 97.5])
        print(f'metric={name} a={va.mean():.4g} b={vb.mean():.4g} diff={d.mean():.4g} ci95=[{lo:.4g},{hi:.4g}]')


def main():
    ap = argparse.ArgumentParser(description='도킹 제어기 폐루프 시뮬레이터')
    sub = ap.add_subparsers(dest='cmd', required=True)
    r = sub.add_parser('run')
    r.add_argument('--runs', type=int, default=200)
    r.add_argument('--seed', type=int, default=1)
    r.add_argument('--scenario', choices=sorted(SCENARIOS), default='afternoon')
    r.add_argument('--set', nargs='*', help='cfg tuning 덮어쓰기 키=JSON값')
    r.add_argument('--noise', nargs='*', help='NOISE 덮어쓰기 키=JSON값(예: drift_deg=[0,0])')
    r.add_argument('--preset', choices=sorted(PRESETS), help='NOISE 묶음(field = 10-30 현장 실측 속도 · 정지 모형)')
    r.add_argument('--seconds', type=float, default=900.0)
    r.add_argument('--workers', type=int, default=10)
    r.add_argument('--out', required=True)
    c = sub.add_parser('compare')
    c.add_argument('a')
    c.add_argument('b')
    a = ap.parse_args()
    cmd_run(a) if a.cmd == 'run' else cmd_compare(a)


if __name__ == '__main__':
    main()
