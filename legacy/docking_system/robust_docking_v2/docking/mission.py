"""Normal approach first; conditional lateral realignment outside the entry.

Both stages use the same stop-feedback primitives and all-outcome certificates.
No timed lateral dead reckoning or mandatory ninety-degree turn is introduced.
"""
import copy
from .core import Box, Grid, Policy, body_aabb, fingerprint, synthesize, validate, finite

ALGORITHM = 'conditional-sidestep-mission-v2'


def enabled(cfg):
    return cfg.get('prealignment', {}).get('enabled', False)


def validate_alignment(cfg):
    if not enabled(cfg):
        return
    p = cfg['prealignment']
    for k in ('entry_x_m', 'center_y_m', 'corridor_half_width_m', 'alignment_y_m', 'alignment_yaw_rad',
              'max_duration_s', 'max_actions'):
        if not finite([p.get(k)]):
            raise ValueError('missing prealignment.'+k)
    for k in ('corridor_half_width_m', 'alignment_y_m', 'alignment_yaw_rad', 'max_duration_s', 'max_actions'):
        if p[k] <= 0:
            raise ValueError('invalid prealignment.'+k)
    if not isinstance(p['max_actions'], int) or p['alignment_yaw_rad'] >= 1.5707963267948966:
        raise ValueError('invalid prealignment action budget/yaw')
    xr = p.get('staging_x_range_m')
    if not isinstance(xr, list) or len(xr) != 2 or not finite(xr) or xr[0] >= xr[1]:
        raise ValueError('missing prealignment.staging_x_range_m')
    goal = alignment_goal(cfg)
    aabb = body_aabb(goal, cfg['body'])
    margin = cfg['scene']['clearance_m']
    if aabb[1] > p['entry_x_m']-margin:
        raise ValueError('alignment goal must keep the complete body outside entry')
    if max(abs(aabb[2]-p['center_y_m']), abs(aabb[3]-p['center_y_m']))+margin > p['corridor_half_width_m']:
        raise ValueError('alignment goal does not fit corridor width including yaw')
    if trigger(cfg) <= 0:
        raise ValueError('no lateral entry clearance')


def trigger(cfg):
    p = cfg['prealignment']
    value = p.get('trigger_lateral_m')
    if value is None:
        value = p['corridor_half_width_m']-cfg['body']['half_width_m']-cfg['scene']['clearance_m']
    if not finite([value]) or value <= 0:
        raise ValueError('invalid lateral trigger')
    return value


def alignment_goal(cfg):
    p = cfg['prealignment']
    return Box([p['staging_x_range_m'][0], p['center_y_m']-p['alignment_y_m'], -p['alignment_yaw_rad']],
               [p['staging_x_range_m'][1], p['center_y_m']+p['alignment_y_m'], p['alignment_yaw_rad']])


def phase_config(cfg, phase, normal_table=None):
    out = copy.deepcopy(cfg)
    out['_phase'] = phase
    if phase == 'SIDESTEP':
        goal, grid = alignment_goal(cfg), Grid(cfg)
        out['goal'] = goal.to_json()
        # Finishing alignment must hand over to an already proven approach.
        out['_terminal_keys'] = sorted(k for k in normal_table['nodes']
                                       if goal.contains(grid.belief(tuple(map(int, k.split(','))))))
    return out


def build(cfg):
    if not enabled(cfg):
        return synthesize(cfg)
    validate(cfg)
    normal = synthesize(phase_config(cfg, 'APPROACH'))
    side = synthesize(phase_config(cfg, 'SIDESTEP', normal))
    return {'schema': 1, 'algorithm': ALGORITHM, 'fingerprint': fingerprint(cfg),
            'approach': normal, 'sidestep': side, 'total_nodes': normal['total_nodes'],
            'winning_nodes': normal['winning_nodes'], 'sidestep_nodes': side['winning_nodes'],
            'layers': normal['layers']+side['layers']}


class MissionPolicy:
    def __init__(self, cfg, table):
        if table.get('algorithm') != ALGORITHM or table.get('fingerprint') != fingerprint(cfg):
            raise ValueError('mission/config mismatch: synthesize again')
        self.cfg, self.grid = cfg, Grid(cfg)
        self.normal = Policy(phase_config(cfg, 'APPROACH'), table['approach'])
        self.side = Policy(phase_config(cfg, 'SIDESTEP', table['approach']), table['sidestep'])
        self.phase = None

    @property
    def active_cfg(self):
        return self.side.cfg if self.phase == 'SIDESTEP' else self.normal.cfg

    def lookup(self, pose, belief):
        if self.phase is None:
            choice = self.normal.lookup(pose, belief)
            if choice is not None:
                self.phase = 'APPROACH'
            else:
                p = self.cfg['prealignment']
                lo, hi = belief.lo[1]-p['center_y_m'], belief.hi[1]-p['center_y_m']
                clearly_far = lo > trigger(self.cfg) or hi < -trigger(self.cfg)
                outside = body_aabb(belief, self.cfg['body'])[1] <= p['entry_x_m']-self.cfg['scene']['clearance_m']
                if not clearly_far or not outside:
                    return None
                choice = self.side.lookup(pose, belief)
                if choice is None or choice['rank'] > p['max_actions']:
                    return None
                self.phase = 'SIDESTEP'
        elif self.phase == 'SIDESTEP':
            choice = self.side.lookup(pose, belief)
        else:
            choice = self.normal.lookup(pose, belief)
        if choice is not None and self.phase == 'SIDESTEP' and choice['rank'] == 0:
            choice = self.normal.lookup(pose, belief)
            if choice is None:
                return None
            self.phase = 'APPROACH'
        if choice is not None:
            choice['phase'] = self.phase
        return choice


def load_policy(cfg, table):
    return MissionPolicy(cfg, table) if enabled(cfg) else Policy(cfg, table)
