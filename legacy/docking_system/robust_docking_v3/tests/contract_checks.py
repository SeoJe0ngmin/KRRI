"""Test-only bounded samples; not a vehicle model or field entry point."""
import copy
import random
from docking.core import Box, Grid, action_box
from docking.mission import load_policy


def simulate(cfg, table, episodes, seed):
    """Sample bounded stop-to-stop contracts; not a forklift physics model."""
    rng = random.Random(seed)
    grid = Grid(cfg)
    prototype = load_policy(cfg, table)
    tables = [table['approach'], table['sidestep']] if 'approach' in table else [table]
    starts = [(tuple(map(int, k.split(','))), v[0]) for t in tables for k, v in t['nodes'].items() if v[0] >= 2]
    starts = [(k, r) for k, r in starts if copy.copy(prototype).lookup(grid.belief(k).center, grid.belief(k)) is not None]
    if not starts:
        raise ValueError('no nontrivial winning region: adjust validated domain/contracts')
    records, max_steps = [], 0
    for episode in range(episodes):
        policy = copy.copy(prototype)
        key, rank = rng.choice(starts)
        belief = grid.belief(key)
        pose = list(belief.center)
        previous_rank = rank+1
        previous_phase = None
        path = []
        for step in range(table['layers']+1):
            noise = [rng.uniform(-e, e) for e in cfg['observation_error']]
            z = [v+e for v, e in zip(pose, noise)]
            measured = Box.around(z, cfg['observation_error'])
            choice = policy.lookup(z, measured)
            phase = choice.get('phase', 'APPROACH') if choice else None
            if choice is None or (phase == previous_phase and choice['rank'] >= previous_rank):
                raise AssertionError('sample left robust winning layers')
            if previous_phase is not None and phase != previous_phase and (previous_phase, phase) != ('SIDESTEP', 'APPROACH'):
                raise AssertionError('invalid mission handover')
            previous_phase = phase
            path.append({'true_pose': pose, 'observed': z, 'rank': choice['rank'],
                         'action': choice['action']['name'] if choice['action'] else None, 'phase': phase})
            if choice['rank'] == 0:
                if not Box(cfg['goal']['lo'], cfg['goal']['hi']).contains(Box.around(pose, [0, 0, 0])):
                    raise AssertionError('false success')
                break
            previous_rank = choice['rank']
            end = action_box(grid.belief(choice['key']), choice['action'])
            pose = [rng.uniform(a, b) for a, b in zip(end.lo, end.hi)]
        else:
            raise AssertionError('no finite progress')
        max_steps = max(max_steps, len(path)-1)
        if episode < 5:
            records.append(path)
    return {'kind': 'synthetic_bounded_contract_check_not_hardware_accuracy', 'seed': seed,
            'episodes': episodes, 'all_reached_goal': True, 'max_actions': max_steps, 'example_paths': records}

