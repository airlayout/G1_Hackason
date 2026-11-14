"""Static layout diagnostics: projection and existing IK/path checks, no stepping."""
from dataclasses import asdict

import numpy as np

from bottle import BottleObservation, BottlePushRoutine, BottleTarget, validate_bottle_target


def validate_layout_xy(x, y, settings):
    target = BottleTarget((x, y), .034, (x-.034, y, settings.table_height+.08), 1., 1., (0, 0, 0, 0))
    validate_bottle_target(target, settings)
    return target


def project_bottle_bounds(xy, settings, optical_transform, intrinsics):
    """Conservative sampled silhouette of build_scene's rigid bottle, before/after push."""
    # Heights are relative to tabletop. Shoulder latitude rings follow its ellipsoid.
    rings = [(.005, .0345), (.205, .0345), (.213, .016), (.259, .016)]
    for angle in np.linspace(0, np.pi, 33):
        rings.append((.200+.030*np.cos(angle), .034*np.sin(angle)))
    samples = []
    for x in (xy[0], xy[0]+settings.push_distance):
        for height, radius in rings:
            for angle in np.linspace(0, 2*np.pi, 96, endpoint=False):
                samples.append((x+radius*np.cos(angle), xy[1]+radius*np.sin(angle),
                                settings.table_height+height))
    transform = np.asarray(optical_transform)
    optical = (np.asarray(samples)-transform[:3, 3])@transform[:3, :3]
    if np.any(optical[:, 2] <= .1):
        return {'in_front': False, 'margin_px': None, 'bbox_uv': None}
    uv = optical[:, :2]/optical[:, 2, None]*[intrinsics.fx, intrinsics.fy]+[intrinsics.cx, intrinsics.cy]
    lo, hi = uv.min(axis=0), uv.max(axis=0)
    margin = min(*lo, intrinsics.width-1-hi[0], intrinsics.height-1-hi[1])
    return {'in_front': True, 'margin_px': float(margin), 'bbox_uv': [*lo.tolist(), *hi.tolist()]}


def project_table_edges(settings, optical_transform, intrinsics):
    """Projected spans only; a visible span is not evidence of valid RGB-D depth."""
    x0, x1, y0, y1 = settings.table_bounds
    edges = {}
    transform = np.asarray(optical_transform)
    for name, axis, bound, start, end in (('x_min', 0, x0, y0, y1), ('x_max', 0, x1, y0, y1),
                                         ('y_min', 1, y0, x0, x1), ('y_max', 1, y1, x0, x1)):
        points = np.full((257, 3), settings.table_height)
        points[:, axis] = bound
        points[:, 1-axis] = np.linspace(start, end, len(points))
        optical = (points-transform[:3, 3])@transform[:3, :3]
        front = optical[:, 2] > .1
        uv = np.full((len(points), 2), -np.inf)
        uv[front] = optical[front, :2]/optical[front, 2, None] \
            * [intrinsics.fx, intrinsics.fy]+[intrinsics.cx, intrinsics.cy]
        visible = front & (uv[:, 0] >= 8) & (uv[:, 0] <= intrinsics.width-9) \
            & (uv[:, 1] >= 8) & (uv[:, 1] <= intrinsics.height-9)
        indices = np.flatnonzero(visible)
        runs = np.split(indices, np.flatnonzero(np.diff(indices) > 1)+1)
        span = max((len(run)-1 for run in runs), default=0)*(end-start)/256
        edges[name] = {'visible_span_m': max(0., float(span)), 'visible': bool(span >= .08)}
    return edges


def inspect_layout(kin, rest, xy, settings, optical_transform, intrinsics, *, preview_step_m=.005):
    """Ground-truth placement is diagnostic only; it never enters live control."""
    row = {'bottle_xy_m': list(xy), 'settings': asdict(settings), 'static_only': True,
           'visibility': project_bottle_bounds(xy, settings, optical_transform, intrinsics),
           'table_edges': project_table_edges(settings, optical_transform, intrinsics),
           'path_passed': False, 'ready': False}
    original = kin.data.qpos.copy()
    routine = BottlePushRoutine(kin, None, None, lambda: None, lambda: 0., settings, .015, .001)
    routine.preview_step_m = preview_step_m
    try:
        target = validate_layout_xy(*xy, settings)
        kin.update(rest)
        routine.preview(BottleObservation(target, 0., 1))
        row['path_passed'] = True
    except ValueError as exc:
        row['reason'] = str(exc)
    finally:
        kin.update(original)
    row['ik_checks'] = routine.report['ik_checks']
    row['ik_searches'] = routine.report['ik_searches']
    view = row['visibility']
    table_visible = all(any(row['table_edges'][f'{axis}_{side}']['visible'] for side in ('min', 'max'))
                        for axis in ('x', 'y'))
    row['ready'] = bool(row['path_passed'] and view['in_front'] and view['margin_px'] >= 8. and table_visible)
    if not row['ready'] and 'reason' not in row:
        row['reason'] = 'ボトルの画角余裕が8px未満、または台の直交する二辺が見えません'
    return row
