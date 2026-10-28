"""Bounded fine-alignment lookahead with insertion goals and lateral fallback.

The best complete route FOUND is selected; beam pruning is not a proof of
global optimality or impossibility. Only its first action is executed before
replanning from a fresh stopped observation.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Callable, Optional

from . import config as cfg
from .controllers import RotationController, heading_rotation_limit_deg
from .motion import drive_seconds
from .pose import VisualPose
from .planner import (
    Waypoint, _pose_after_action, _vision_meta_after_turn,
    action_keeps_front_visible, insertion_alignment_turn, plan_waypoint,
)


@dataclass(frozen=True)
class RoutePlan:
    waypoints: tuple[Waypoint, ...]
    insertion_found: bool
    final_turn_deg: Optional[float]
    estimated_seconds: float
    final_lateral_m: float
    expanded_nodes: int
    reason: str


@dataclass(frozen=True)
class _Node:
    pose: VisualPose
    meta: Optional[dict]
    path: tuple[Waypoint, ...]
    used_m: float
    seconds: float
    direction_changes: int = 0
    last_turn_sign: int = 0


def rotation_seconds(turn: float, fine: bool = False, *,
                     slope_multiplier: float = 1.0, time_scale: float = 1.0) -> float:
    """Use the same fitted controller as execution, without sending commands."""
    if not math.isfinite(turn) or abs(turn) >= 180.:
        return math.inf
    if abs(turn) < 1e-9:
        return 0.0
    controller = RotationController()
    controller.start(turn, 'ROT_RIGHT' if turn > 0 else 'ROT_LEFT', 0., 0.,
                     domain='heading', slope_multiplier=slope_multiplier,
                     adaptive_time_scale=time_scale, insertion_fine=fine)
    plan = controller.plan
    if (plan is None or not plan.feasible or plan.hold_sec <= 0.0
            or abs(abs(plan.target_deg)-abs(turn)) > 1e-7):
        return math.inf
    return plan.hold_sec


def action_seconds(waypoint: Waypoint, turn_time: Callable) -> float:
    seconds = drive_seconds(waypoint.forward_m, apply_max_limit=False)
    if seconds > min(cfg.FWD_COMMAND_MAX_SEC, cfg.FWD_MAX_COMMAND_SEC):
        return math.inf
    seconds += cfg.STOP_MIN_SETTLE_SEC + cfg.MULTISTEP_OBSERVATION_SEC
    if abs(waypoint.turn_deg) > 1e-9:
        seconds += turn_time(waypoint.turn_deg, False) + cfg.STOP_MIN_SETTLE_SEC + cfg.MULTISTEP_OBSERVATION_SEC
    return seconds


def _successors(node: _Node, remaining_m: float, visibility_backend=None, rotation_limit_deg=None, turn_time=None):
    candidates: list[Waypoint] = []
    plan_waypoint(node.pose, 0., 0., min(remaining_m, cfg.MULTISTEP_MAX_FORWARD_M), node.meta,
                  all_candidates=candidates, visibility_backend=visibility_backend, rotation_limit_deg=rotation_limit_deg,
                  rotation_feasible=(None if turn_time is None else lambda t: math.isfinite(turn_time(t, False))))
    for candidate in candidates:
        # Include short actions: the longest endpoint may remove room needed
        # by a later turn. The minimum command is also an explicit candidate.
        distances = {candidate.forward_m * f for f in cfg.MULTISTEP_FORWARD_FRACTIONS}
        distances.add(cfg.FWD_RELIABLE_MIN_DISTANCE_M)
        for distance in sorted(distances, reverse=True):
            if not cfg.FWD_RELIABLE_MIN_DISTANCE_M <= distance <= candidate.forward_m:
                continue
            waypoint = replace(candidate, forward_m=distance)
            pose = _pose_after_action(node.pose, waypoint.turn_deg, distance)
            meta = _vision_meta_after_turn(node.meta, waypoint.turn_deg, distance)
            yield waypoint, pose, meta


def _terminal(node: _Node, turn_time: Callable, alignment_entered: bool, rotation_limit_deg=None):
    # A root is the actual stopped pose: near-range final alignment uses the
    # execution margin. Future endpoints after planned motion must earn the
    # stricter planning clearance, even when their predicted Z is near.
    margin = (cfg.PLANNING_INSERT_WALL_MARGIN_M if node.path
              else cfg.INSERT_WALL_MARGIN_M)
    turn = insertion_alignment_turn(node.pose, node.meta,
                                    alignment_entered=alignment_entered,
                                    wall_margin_m=margin, rotation_limit_deg=rotation_limit_deg)
    if turn is None:
        return None
    # The existing final-turn helper permits recovery from outside its ROI.
    # A planned future final turn must satisfy full visibility as well.
    if turn and not action_keeps_front_visible(node.pose, turn, 0., node.meta):
        return None
    pose = _pose_after_action(node.pose, turn, 0.)
    distance = pose.pallet_z_m - cfg.INSERT_CAMERA_Z_REMAINDER_M
    if distance <= 0.0:
        return None
    hold = drive_seconds(distance, apply_max_limit=False)
    if (hold >= min(cfg.FWD_COMMAND_MAX_SEC, cfg.FWD_MAX_COMMAND_SEC)
            or cfg.task_time_expired(hold + cfg.STOP_MIN_SETTLE_SEC, cfg.INSERT_MAX_TOTAL_SEC)):
        return None
    seconds = node.seconds + hold + cfg.STOP_MIN_SETTLE_SEC
    if turn:
        seconds += turn_time(turn, True) + cfg.STOP_MIN_SETTLE_SEC + cfg.MULTISTEP_OBSERVATION_SEC
    return (seconds, turn, pose) if math.isfinite(seconds) else None


def _bucket(node: _Node):
    # Keep geometrically different headings/approach depths in the beam.
    return (round(node.pose.rot_x_pallet_m / .025),
            round(node.pose.rot_z_pallet_m / .10),
            round(node.pose.yaw_deg / 2.0))


def plan_fine_route(pose: VisualPose, vision_meta: Optional[dict],
                    remaining_forward_m: float, *, remaining_cycles: int,
                    remaining_seconds: float = math.inf,
                    turn_time: Callable = rotation_seconds,
                    alignment_entered: bool = False,
                    remaining_direction_changes: Optional[int] = None,
                    last_rotation_command: Optional[str] = None,
                    visibility_backend=None, rotation_limit_deg=None) -> RoutePlan:
    if rotation_limit_deg is None:
        rotation_limit_deg = heading_rotation_limit_deg()
    raw_turn_time = turn_time
    time_cache = {}
    def turn_time(turn, fine):
        key = (turn, fine)
        if key not in time_cache:
            time_cache[key] = raw_turn_time(turn, fine)
        return time_cache[key]
    last_sign = {'ROT_RIGHT': 1, 'ROT_LEFT': -1}.get(last_rotation_command, 0)
    root = _Node(pose, vision_meta, (), 0., 0., 0, last_sign)
    if remaining_direction_changes is None:
        remaining_direction_changes = cfg.ROT_MAX_DIRECTION_CHANGES
    frontier = [root]
    best = None
    fallback = None
    expanded = 0
    max_depth = min(cfg.MULTISTEP_MAX_DEPTH, max(0, remaining_cycles))
    visited = {}
    def direct_first(node):
        if not node.path or abs(pose.rot_x_pallet_m) > cfg.MULTISTEP_DIRECT_LATERAL_M:
            return False
        first = node.path[0]
        after = _pose_after_action(pose, first.turn_deg, first.forward_m)
        return (abs(first.forward_m - cfg.MULTISTEP_MAX_FORWARD_M) < 1e-6
                and abs(after.rot_x_pallet_m) < 1e-6)
    for depth in range(max_depth + 1):
        children = []
        for node in frontier:
            terminal = _terminal(node, turn_time, alignment_entered and not node.path, rotation_limit_deg)
            if terminal is not None and terminal[0] <= remaining_seconds:
                # Already insertable roots need no extra movement. Otherwise
                # prefer the requested full-length exact-centreline first leg
                # among complete routes, then minimise estimated total time.
                key = (bool(node.path) and not direct_first(node),
                       terminal[0], len(node.path), abs(terminal[1]))
                if best is None or key < best[0]:
                    best = (key, node, terminal)
            if depth >= max_depth or expanded >= cfg.MULTISTEP_MAX_EXPANSIONS:
                continue
            if (best is not None and (bool(node.path) and not direct_first(node)) >= best[0][0]
                    and node.seconds >= best[0][1]):
                continue
            expanded += 1
            for waypoint, predicted, meta in _successors(
                    node, remaining_forward_m - node.used_m, visibility_backend, rotation_limit_deg, turn_time):
                sign = (1 if waypoint.turn_deg > 0 else -1) if waypoint.turn_deg else node.last_turn_sign
                changes = node.direction_changes + int(bool(node.last_turn_sign and sign != node.last_turn_sign))
                if (remaining_direction_changes is not None
                        and changes > remaining_direction_changes):
                    continue
                seconds = node.seconds + action_seconds(waypoint, turn_time)
                if not math.isfinite(seconds) or seconds > remaining_seconds:
                    continue
                child = _Node(predicted, meta, node.path + (waypoint,),
                              node.used_m + waypoint.forward_m, seconds, changes, sign)
                # Fallback is maximum lateral correction over the searched
                # route, with elapsed time breaking ties. No unsafe motion.
                if abs(predicted.rot_x_pallet_m) < abs(pose.rot_x_pallet_m) - 1e-6:
                    key = (round(abs(predicted.rot_x_pallet_m), 6), not direct_first(child), seconds)
                    if fallback is None or key < fallback[0]:
                        fallback = (key, child)
                bucket = (direct_first(child), child.last_turn_sign, _bucket(child))
                previous = visited.get(bucket)
                # Depth/distance budgets matter as well as elapsed time.
                resources = (seconds, child.used_m, len(child.path), changes)
                if previous is not None and all(a <= b for a, b in zip(previous, resources)):
                    continue
                visited[bucket] = resources
                children.append(child)
        if not children:
            break
        # Heuristic only controls beam retention; COMPLETE routes are ranked
        # by actual fitted command + settle estimates, never this heuristic.
        def priority(node):
            remaining_z = max(0., node.pose.pallet_z_m - cfg.INSERT_ALIGNMENT_MAX_CAMERA_Z_M)
            estimate = node.seconds + remaining_z / cfg.FWD_SLOPE_M_S + 20. * abs(node.pose.rot_x_pallet_m)
            return (not direct_first(node), estimate)
        unique = {}
        for child in sorted(children, key=priority):
            unique.setdefault((direct_first(child), child.last_turn_sign, _bucket(child)), child)
        frontier = list(unique.values())[:cfg.MULTISTEP_BEAM_WIDTH]
    if best is not None:
        _, node, terminal = best
        return RoutePlan(node.path, True, terminal[1], terminal[0],
                         terminal[2].rot_x_pallet_m, expanded,
                         ('centreline-first insertion route; shortest estimated time within priority'
                          if direct_first(node) else
                          'fastest insertion route found within search limits'))
    if fallback is not None:
        node = fallback[1]
        return RoutePlan(node.path, False, None, node.seconds,
                         node.pose.rot_x_pallet_m, expanded,
                         'insertion route not found; maximum lateral correction found')
    return RoutePlan((), False, None, 0., pose.rot_x_pallet_m, expanded,
                     'no feasible insertion or lateral-correction route found')
