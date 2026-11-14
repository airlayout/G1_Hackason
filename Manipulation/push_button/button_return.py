"""Precompute short collision-checked lamp-return paths before motion starts."""
import math
import time

import numpy as np

from robust_scene import contact_metrics


PERIOD = .02
NODE_SPACING = 25
# Keep the planner's existing 20% speed and acceleration headroom.
LIMIT_FRACTION = .8
CLEARANCE_M = .003


def _duration(a, b, limits):
    delta = float(np.max(np.abs(b-a)))
    return max(PERIOD, 1.875*delta/(LIMIT_FRACTION*limits.joint_speed_rad_s),
               math.sqrt(5.774*delta/(LIMIT_FRACTION*limits.joint_acceleration_rad_s2)))


def _segment(kin, scene, base, start, end, limits, deadline):
    """Check a straight joint-space shortcut, including clearance along it."""
    pose = base.copy()
    pose[kin.qadr] = start
    kin.update(pose)
    target = kin.model.geom('robust_target_geom').id
    kin.validate_path(end, allow_tip_contact=True, allowed_tip_geoms={target})
    delta = float(np.max(np.abs(end-start)))
    depth_min = float('inf')
    for t in np.linspace(0., 1., max(13, math.ceil(delta/.015)+1)):
        if time.perf_counter() >= deadline:
            raise TimeoutError('lamp return optimization budget')
        pose[kin.qadr] = start+(end-start)*t
        kin.update(pose)
        if contact_metrics(kin, scene, contact=True) < CLEARANCE_M:
            raise ValueError('lamp return obstacle clearance')
        depth = float((kin.tip_position()-scene.face)@scene.normal+scene.tip_radius)
        depth_min = min(depth_min, depth)
        if depth > -.005 and depth > depth_min+.0005:
            raise ValueError('lamp return moves toward panel near contact')
    steps = math.ceil(_duration(start, end, limits)/PERIOD)
    t = np.arange(steps+1, dtype=float)/steps
    blend = 10*t**3-15*t**4+6*t**5
    qs = start[None,:]+blend[:,None]*(end-start)[None,:]
    tcps = []
    for q in qs:
        if time.perf_counter() >= deadline:
            raise TimeoutError('lamp return optimization budget')
        pose[kin.qadr] = q
        kin.update(pose)
        tcps.append(kin.tip_position().copy())
    return qs, np.asarray(tcps)


def optimize_lamp_returns(kin, scene, qs, tcps, press_end, limits, deadline):
    """Minimize estimated travel time over collision-checked shortcut nodes.

    The reversed reference remains an edge, so each sampled node has a route
    even where a shortcut is obstructed. Real commands are checked again.
    """
    nodes = list(range(0, press_end+1, NODE_SPACING))
    if nodes[-1] != press_end:
        nodes.append(press_end)
    base = kin.data.qpos.copy()
    costs = [0.]
    edges = [None]
    try:
        for n in range(1, len(nodes)):
            start, previous = nodes[n], nodes[n-1]
            best = costs[n-1]+(start-previous)*PERIOD
            best_edge = dict(to=n-1, q=qs[start:previous-1 if previous else None:-1].tolist(),
                             tcp=tcps[start:previous-1 if previous else None:-1].tolist(), kind='reference')
            for j in range(n):
                if time.perf_counter() >= deadline:
                    raise TimeoutError('lamp return optimization budget')
                end = nodes[j]
                estimate = costs[j]+math.ceil(_duration(qs[start], qs[end], limits)/PERIOD)*PERIOD
                if estimate >= best:
                    continue
                try:
                    segment_q, segment_tcp = _segment(kin, scene, base, qs[start], qs[end], limits, deadline)
                except ValueError:
                    continue
                best = estimate
                best_edge = dict(to=j,q=segment_q.tolist(),tcp=segment_tcp.tolist(),kind='shortcut')
            costs.append(best)
            edges.append(best_edge)
    finally:
        kin.update(base)
    return dict(version=1,nodes=nodes,edges=edges,estimated_s=costs,
                limit_fraction=LIMIT_FRACTION,clearance_m=CLEARANCE_M)


def build_lamp_return(candidate, index):
    """Connect the current reference to the nearest earlier optimized node."""
    plan = candidate.motion_metrics.get('lamp_return_plan')
    if not plan or plan.get('version') != 1:
        return candidate.q[:index+1][::-1].copy(), candidate.tcp[:index+1][::-1].copy(), False
    nodes, edges = plan['nodes'], plan['edges']
    node = int(np.searchsorted(nodes, index, side='right'))-1
    anchor = nodes[node]
    joint_parts = [candidate.q[anchor:index+1][::-1]]
    tcp_parts = [candidate.tcp[anchor:index+1][::-1]]
    used_shortcut = False
    while node:
        edge = edges[node]
        joint_parts.append(np.asarray(edge['q'], dtype=float)[1:])
        tcp_parts.append(np.asarray(edge['tcp'], dtype=float)[1:])
        used_shortcut |= edge['kind'] == 'shortcut'
        node = edge['to']
    return np.concatenate(joint_parts), np.concatenate(tcp_parts), used_shortcut
