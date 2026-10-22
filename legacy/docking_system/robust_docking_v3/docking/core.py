"""Interval geometry and a finite observation-feedback reachability game.

Units: metres, radians, seconds. Dock frame: x into bay, y left, yaw CCW.
No Gaussian covariance is interpreted as a worst-case error bound.
"""
import hashlib
import itertools
import json
import math

SCHEMA = 1
ALGORITHM = 'observation-game-recovery-v4'


def finite(values):
    return all(not isinstance(v, bool) and isinstance(v, (int, float)) and math.isfinite(v) for v in values)


class Box:
    def __init__(self, lo, hi):
        self.lo, self.hi = tuple(lo), tuple(hi)
        if len(lo) != 3 or len(hi) != 3 or not finite(tuple(lo) + tuple(hi)):
            raise ValueError("pose box needs six finite numbers")
        if any(a > b for a, b in zip(lo, hi)):
            raise ValueError("inverted interval")

    @classmethod
    def around(cls, center, radius):
        if not finite(radius) or min(radius) < 0:
            raise ValueError("invalid radius")
        return cls([c-r for c, r in zip(center, radius)], [c+r for c, r in zip(center, radius)])

    @property
    def center(self):
        return tuple((a+b)/2 for a, b in zip(self.lo, self.hi))

    @property
    def radius(self):
        return tuple((b-a)/2 for a, b in zip(self.lo, self.hi))

    def inflate(self, r):
        return Box([a-e for a, e in zip(self.lo, r)], [b+e for b, e in zip(self.hi, r)])

    def contains(self, other):
        return all(a <= c+1e-10 and d <= b+1e-10 for a, b, c, d in zip(self.lo, self.hi, other.lo, other.hi))

    def intersection(self, other):
        lo = [max(a, b) for a, b in zip(self.lo, other.lo)]
        hi = [min(a, b) for a, b in zip(self.hi, other.hi)]
        return None if any(a > b for a, b in zip(lo, hi)) else Box(lo, hi)

    def to_json(self):
        return {"lo": self.lo, "hi": self.hi}


def trig_bounds(lo, hi, cosine=False):
    if hi-lo >= 2*math.pi:
        return (-1., 1.)
    fn = math.cos if cosine else math.sin
    phase = 0 if cosine else math.pi/2
    pts = [lo, hi]
    pts += [phase+k*math.pi for k in range(int(math.ceil((lo-phase)/math.pi)),
                                         int(math.floor((hi-phase)/math.pi))+1)]
    vals = [fn(p) for p in pts]
    return min(vals), max(vals)


def product(a, b):
    p = [x*y for x in a for y in b]
    return min(p), max(p)


def interval_sum(a, b):
    return a[0]+b[0], a[1]+b[1]


def project_xy(box, axis):
    return interval_sum(product((box.lo[0], box.hi[0]), (axis[0], axis[0])),
                        product((box.lo[1], box.hi[1]), (axis[1], axis[1])))


def interpolate_interval(start, end, progress):
    values = [(1-p)*a+p*b for p, a, b in itertools.product(progress, start, end)]
    return min(values), max(values)


def body_aabb(box, body):
    """All rectangle corners, for every pose in box (not just centre pose)."""
    c = trig_bounds(box.lo[2], box.hi[2], True)
    s = trig_bounds(box.lo[2], box.hi[2])
    xs, ys = [], []
    for x, y in itertools.product((-body['rear_m'], body['front_m']),
                                   (-body['half_width_m'], body['half_width_m'])):
        xc, ysine = product((x, x), c), product((y, y), s)
        xs += [box.lo[0]+xc[0]-ysine[1], box.hi[0]+xc[1]-ysine[0]]
        xsine, yc = product((x, x), s), product((y, y), c)
        ys += [box.lo[1]+xsine[0]+yc[0], box.hi[1]+xsine[1]+yc[1]]
    return [min(xs), max(xs), min(ys), max(ys)]


def safe(box, cfg):
    a = body_aabb(box, cfg['body'])
    # Each conservative body AABB must fit in one known free rectangle.
    # This may reject safe paths across a union seam; it never fills that seam.
    m = cfg['scene']['clearance_m']
    p = cfg.get('prealignment', {})
    if p.get('enabled'):
        if cfg.get('_phase') == 'SIDESTEP' and a[1] > p['entry_x_m']-m:
            return False
        if a[1] > p['entry_x_m']-m:
            # No large yaw or cross-track correction after any part enters.
            if (box.lo[1] < p['center_y_m']-p['alignment_y_m'] or
                    box.hi[1] > p['center_y_m']+p['alignment_y_m'] or
                    box.lo[2] < -p['alignment_yaw_rad'] or box.hi[2] > p['alignment_yaw_rad']):
                return False
    from .visibility import observable
    return (any(r[0]+m <= a[0] and a[1] <= r[1]-m and
                r[2]+m <= a[2] and a[3] <= r[3]-m for r in cfg['scene']['free_rectangles'])
            and observable(box, cfg))


def action_box(start, action, progress=(1., 1.), sweep=False):
    d = action['target']
    err = action['sweep_error'] if sweep else action['terminal_error']
    if action['kind'] == 'rotate':
        # Absolute heading target chosen from the observation node centre.
        # Feedback contracts the heading set, rather than adding an uncertain
        # open-loop angle to an already uncertain heading.
        target_h = start.center[2]+d
        h = interpolate_interval((start.lo[2], start.hi[2]),
                                 (target_h-err[2], target_h+err[2]), progress)
        if sweep:
            h = (h[0]-err[2], h[1]+err[2])
        return Box((start.lo[0]-err[0], start.lo[1]-err[1], h[0]),
                   (start.hi[0]+err[0], start.hi[1]+err[1], h[1]))
    else:
        h = start.center[2]
        n, q = (math.cos(h), math.sin(h)), (-math.sin(h), math.cos(h))
        n0, q0 = project_xy(start, n), project_xy(start, q)
        nc = start.center[0]*n[0]+start.center[1]*n[1]+d
        ng = (nc-err[0], nc+err[0])
        dn = (ng[0]-n0[1], ng[1]-n0[0])
        dh = (start.lo[2]-h-err[2], start.hi[2]-h+err[2])
        if dh[0] <= -math.pi/2 or dh[1] >= math.pi/2:
            raise ValueError('translation heading interval crosses 90 degrees')
        # The vehicle does not steer while translating. Cross-track motion
        # includes every heading in the initial set plus yaw/slip bounds.
        dq = product(product(dn, progress), (math.tan(dh[0]), math.tan(dh[1])))
        qt = (q0[0]+dq[0]-err[1], q0[1]+dq[1]+err[1])
        nt = interpolate_interval(n0, ng, progress)
        if sweep:
            nt = (nt[0]-err[0], nt[1]+err[0])
        x = interval_sum(product(nt, (n[0], n[0])), product(qt, (q[0], q[0])))
        y = interval_sum(product(nt, (n[1], n[1])), product(qt, (q[1], q[1])))
        return Box((x[0], y[0], start.lo[2]-err[2]), (x[1], y[1], start.hi[2]+err[2]))


def transition(start, action, cfg):
    if action.get('sidestep_only') and cfg.get('_phase') != 'SIDESTEP':
        return None
    if action['kind'] == 'rotate':
        initial_spread = start.radius[2]
    else:
        h = start.center[2]
        limits = project_xy(start, (math.cos(h), math.sin(h)))
        initial_spread = (limits[1]-limits[0])/2
    if abs(action['target'])-initial_spread < action['min_target']:
        return None
    # Every possible starting position must precede the braking threshold.
    # Otherwise a stationary vehicle near a bin edge can immediately STOP.
    if abs(action['target'])-initial_spread <= action['brake_reserve']:
        return None
    n = cfg['synthesis']['sweep_segments']
    # sweep_error bounds nonmonotone motion, coast, command delay, slip and
    # residual motion at entry. It must enclose every prefix, including STOP.
    for k in range(n):
        segment = action_box(start, action, (k/n, (k+1)/n), True)
        if not safe(segment, cfg):
            return None
    return action_box(start, action)


def fingerprint(cfg):
    return hashlib.sha256(json.dumps(cfg, sort_keys=True, separators=(',', ':'),
                                     allow_nan=False).encode('utf8')).hexdigest()


def validate(cfg, live=False):
    """Unknown hardware bounds are errors, not zeros or historical defaults."""
    if cfg.get('schema') != SCHEMA:
        raise ValueError('unsupported config schema')
    required = ('body', 'scene', 'grid', 'goal', 'observation_error', 'actions', 'limits')
    for key in required:
        if cfg.get(key) is None:
            raise ValueError('missing measured/configured field: '+key)
    if not finite(cfg['observation_error']) or len(cfg['observation_error']) != 3 or min(cfg['observation_error']) < 0:
        raise ValueError('invalid observation_error')
    for key in ('front_m', 'rear_m', 'half_width_m'):
        if not finite([cfg['body'][key]]) or cfg['body'][key] <= 0:
            raise ValueError('invalid body '+key)
    Box(cfg['goal']['lo'], cfg['goal']['hi'])
    if cfg['scene']['clearance_m'] < 0 or not finite([cfg['scene']['clearance_m']]):
        raise ValueError('invalid clearance')
    if not cfg['scene']['free_rectangles']:
        raise ValueError('no known free space')
    for r in cfg['scene']['free_rectangles']:
        if len(r) != 4 or not finite(r) or r[0] >= r[1] or r[2] >= r[3]:
            raise ValueError('invalid free rectangle')
    if not cfg['actions']:
        raise ValueError('no motion contracts')
    names = set()
    for a in cfg['actions']:
        if a['name'] in names or a['kind'] not in ('translate', 'rotate'):
            raise ValueError('invalid/duplicate action')
        names.add(a['name'])
        if not finite([a['target']]) or a['target'] == 0:
            raise ValueError('zero/invalid action target')
        expected = ('forward_slow' if a['target'] > 0 else 'backward') if a['kind'] == 'translate' else ('rotate_left_slow' if a['target'] > 0 else 'rotate_right_slow')
        if a['command'] != expected:
            raise ValueError('command/target direction mismatch')
        for key in ('terminal_error', 'sweep_error'):
            if len(a[key]) != 3 or not finite(a[key]) or min(a[key]) < 0:
                raise ValueError('invalid action bound')
        if any(s < e for s, e in zip(a['sweep_error'], a['terminal_error'])):
            raise ValueError('sweep must include terminal uncertainty')
        for key in ('brake_reserve', 'min_target', 'max_duration_s'):
            if not finite([a[key]]) or a[key] <= 0:
                raise ValueError('invalid action '+key)
        for key in ('max_start_delay_s', 'max_stop_response_s'):
            if (live or key in a) and (not finite([a.get(key)]) or a[key] <= 0):
                raise ValueError('missing/invalid measured action '+key)
        if abs(a['target']) < a['min_target'] or a['brake_reserve'] >= abs(a['target']):
            raise ValueError('unexecutable action size')
    for key in ('max_age_s', 'max_speed_mps', 'max_yaw_rate_rps', 'stop_speed_mps',
                'stop_yaw_rate_rps', 'stop_dwell_s', 'settle_timeout_s', 'lease_s'):
        if not finite([cfg['limits'][key]]) or cfg['limits'][key] <= 0:
            raise ValueError('invalid limit '+key)
    for key in ('run_timeout_s', 'max_steps'):
        if key in cfg['limits'] and (not finite([cfg['limits'][key]]) or cfg['limits'][key] <= 0):
            raise ValueError('invalid limit '+key)
    if not isinstance(cfg['limits'].get('max_steps', 20), int):
        raise ValueError('max_steps must be an integer')
    g = cfg['grid']
    if len(g['lo']) != 3 or not finite(g['lo']+g['step']) or min(g['step']) <= 0:
        raise ValueError('invalid grid')
    if len(g['count']) != 3 or any(not isinstance(n, int) or n < 1 for n in g['count']):
        raise ValueError('invalid grid counts')
    if g['count'][0]*g['count'][1]*g['count'][2] > cfg['synthesis']['max_nodes']:
        raise ValueError('grid exceeds configured node budget')
    if not isinstance(cfg['synthesis']['sweep_segments'], int) or cfg['synthesis']['sweep_segments'] < 1:
        raise ValueError('invalid sweep segment count')
    from .mission import validate_alignment
    from .visibility import validate_visibility
    validate_alignment(cfg)
    validate_visibility(cfg)
    from .recovery import validate_recovery
    validate_recovery(cfg, live)
    from .startup import validate_startup
    validate_startup(cfg, live)
    from .startup_measurement import validate_startup_measurement
    validate_startup_measurement(cfg)
    caps = cfg.get('operational', {})
    for k, v in caps.items():
        if not finite([v]) or v <= 0:
            raise ValueError('invalid operational limit: '+k)
    for a in cfg['actions']:
        distance_key = 'max_forward_m' if a['target'] > 0 else 'max_backward_m'
        if a['kind'] == 'translate' and distance_key in caps and abs(a['target']) > caps[distance_key]:
            raise ValueError('action exceeds '+distance_key)
        duration_key = 'max_rotate_s' if a['kind'] == 'rotate' else 'max_translate_s'
        if duration_key in caps and a['max_duration_s'] > caps[duration_key]:
            raise ValueError('action exceeds '+duration_key)
        if a['kind'] == 'rotate' and a['min_target'] < caps.get('min_rotate_rad', 0):
            raise ValueError('rotation smaller than measured floor')
    if live:
        if cfg.get('purpose') != 'hardware' or not cfg.get('validation', {}).get('approved'):
            raise ValueError('hardware profile is unvalidated; use measure/shadow first')
        for k in ('geometry', 'extrinsics', 'vision_error', 'motion_contracts', 'timing', 'stop_detection'):
            if not cfg['validation'].get(k):
                raise ValueError('missing validation evidence: '+k)


class Grid:
    def __init__(self, cfg):
        self.cfg = cfg
        self.lo, self.step, self.count = (cfg['grid'][k] for k in ('lo', 'step', 'count'))
        self.error = cfg['observation_error']

    def keys(self):
        return itertools.product(*(range(n) for n in self.count))

    def belief(self, key):
        lo = [a+i*s for a, i, s in zip(self.lo, key, self.step)]
        return Box(lo, [a+s for a, s in zip(lo, self.step)]).inflate(self.error)

    def locate(self, z):
        k = tuple(int(math.floor((v-a)/s)) for v, a, s in zip(z, self.lo, self.step))
        return k if all(0 <= i < n for i, n in zip(k, self.count)) else None

    def possible_observations(self, endpoint):
        """All observation bins, including both sides of exact boundaries.

        z = true_pose + bounded measurement error. Each next bin itself
        denotes bin +/- error. Thus the game is over observation beliefs,
        not a nominal-state MDP or optimistic nearest-cell lookup.
        """
        z = endpoint.inflate(self.error)
        ranges = []
        for low, high, origin, step, n in zip(z.lo, z.hi, self.lo, self.step, self.count):
            if low < origin or high >= origin+n*step:
                return None
            first = max(0, int(math.floor((low-origin)/step-1e-10)))
            last = min(n-1, int(math.floor((high-origin)/step+1e-10)))
            ranges.append(range(first, last+1))
        return tuple(itertools.product(*ranges))


def synthesize(cfg, progress=None):
    validate(cfg)
    grid = Grid(cfg)
    goal = Box(cfg['goal']['lo'], cfg['goal']['hi'])
    ranks, policy, edges = {}, {}, {}
    keys = list(grid.keys())
    for index, key in enumerate(keys):
        b = grid.belief(key)
        if not safe(b, cfg):
            continue
        if goal.contains(b) and ('_terminal_keys' not in cfg or ','.join(map(str, key)) in cfg['_terminal_keys']):
            ranks[key] = 0
            continue
        options = []
        for ai, a in enumerate(cfg['actions']):
            end = transition(b, a, cfg)
            successors = grid.possible_observations(end) if end else None
            if successors:
                options.append((ai, successors))
        edges[key] = options
        if progress and index % 500 == 0:
            progress(index, len(keys))
    # Synchronous layers: every possible observation successor must have a
    # strictly smaller existing rank. Self-loops cannot certify progress.
    layer = 1
    while True:
        additions = {}
        for key, options in edges.items():
            if key in ranks:
                continue
            valid = [(max(ranks[s] for s in succ), ai) for ai, succ in options
                     if all(s in ranks for s in succ)]
            if valid:
                additions[key] = min(valid)[1]
        if not additions:
            break
        for key, ai in additions.items():
            ranks[key], policy[key] = layer, ai
        layer += 1
    return {'schema': SCHEMA, 'algorithm': ALGORITHM, 'fingerprint': fingerprint(cfg), 'total_nodes': len(keys),
            'winning_nodes': len(ranks), 'layers': layer,
            'nodes': {','.join(map(str, k)): [r, policy.get(k)] for k, r in ranks.items()}}


class Policy:
    def __init__(self, cfg, table):
        validate(cfg)
        if table.get('schema') != SCHEMA or table.get('algorithm') != ALGORITHM or table.get('fingerprint') != fingerprint(cfg):
            raise ValueError('policy/config mismatch: synthesize again')
        self.cfg, self.grid, self.table = cfg, Grid(cfg), table
        self.verify()

    def verify(self):
        """Independently recheck a saved game's certificate before lookup."""
        ranks = {}
        for encoded, node in self.table['nodes'].items():
            key = tuple(map(int, encoded.split(',')))
            if len(key) != 3 or any(not 0 <= k < n for k, n in zip(key, self.grid.count)):
                raise ValueError('policy node outside domain')
            rank, ai = node
            if not isinstance(rank, int) or rank < 0:
                raise ValueError('invalid policy rank')
            ranks[key] = rank
        goal = Box(self.cfg['goal']['lo'], self.cfg['goal']['hi'])
        for encoded, (rank, ai) in self.table['nodes'].items():
            key = tuple(map(int, encoded.split(',')))
            b = self.grid.belief(key)
            if not safe(b, self.cfg):
                raise ValueError('unsafe policy belief')
            if rank == 0:
                if ai is not None or not goal.contains(b) or ('_terminal_keys' in self.cfg and encoded not in self.cfg['_terminal_keys']):
                    raise ValueError('invalid terminal policy node')
            else:
                if not isinstance(ai, int) or not 0 <= ai < len(self.cfg['actions']):
                    raise ValueError('invalid policy action')
                end = transition(b, self.cfg['actions'][ai], self.cfg)
                succ = self.grid.possible_observations(end) if end else None
                if not succ or any(s not in ranks or ranks[s] >= rank for s in succ):
                    raise ValueError('policy does not prove all-successor progress')

    def lookup(self, observation, belief):
        key = self.grid.locate(observation)
        if key is None or not self.grid.belief(key).contains(belief):
            return None
        node = self.table['nodes'].get(','.join(map(str, key)))
        if node is None:
            return None
        rank, ai = node
        return {'key': key, 'rank': rank, 'action': None if ai is None else self.cfg['actions'][ai]}
