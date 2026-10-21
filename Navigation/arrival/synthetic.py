"""Analytic fixtures only: no locomotion simulation or robot performance claims."""
from __future__ import annotations

import math
from pathlib import Path

import numpy as np

from .geometry import camera_transform, pose_matrix
from .session import Writer, initialize, mark, write_json


def generate(root):
    root = Path(root)
    manifest = initialize(root, "synthetic")
    manifest["clock"].update(synchronized=True, max_offset_s=.001)
    manifest["target"]["nav_goal"] = {"frame_id": "map", "child_frame_id": "base_link", "xyz": [0., 0., 0.], "xyzw": [0., 0., 0., 1.]}
    optical = np.eye(4)
    optical[:3, :3] = [[0, 0, 1], [-1, 0, 0], [0, -1, 0]]
    optical[:3, 3] = [.08, .02, .1]
    joint_origin = np.eye(4)
    joint_origin[2, 3] = .3
    base_pelvis = np.eye(4)
    base_pelvis[2, 3] = .8
    calibration = {"id": "SYNTHETIC_NOT_REAL", "data_kind": "synthetic", "camera": "head_camera",
                   "optical_frame": "head_optical", "measured": True,
                   "joints": [{"T_parent_joint": joint_origin.tolist(), "axis": [0, 0, 1], "q_index": 12}],
                   "T_last_optical": optical.tolist(), "T_pelvis_imu": np.eye(4).tolist(),
                   "T_base_pelvis": base_pelvis.tolist(), "navigation_base_frame": "base_link",
                   "translation_uncertainty_m": .001, "rotation_uncertainty_rad": .001}
    write_json(root/"session.json", manifest)
    write_json(root/"calibration.json", calibration)
    root.joinpath("frames").mkdir(exist_ok=True)
    writers = {name: Writer(root, name) for name in ("ros", "body", "frames")}
    annotations, expected = [], []
    rng = np.random.default_rng(21)
    K = np.array([[70., 0, 48.], [0, 70., 36.], [0, 0, 1.]])
    height, width = 72, 96
    v, u = np.indices((height, width))
    rays = np.linalg.solve(K, np.array([u.ravel(), v.ravel(), np.ones(width*height)]))
    entries = [{"id": "reference", "route": "reference", "attempt": 0}, *manifest["trials"]]
    try:
        for index, trial in enumerate(entries):
            trial_id = trial["id"]
            start = 1000.+8*index
            xyz = np.zeros(3) if index == 0 else np.r_[rng.uniform(-.08, .08, 2), 0.]
            yaw = 0. if index == 0 else float(rng.uniform(-.16, .16))
            xyzw = [0., 0., math.sin(yaw/2), math.cos(yaw/2)]
            expected.append({"id": trial_id, "delta_normal_m": -float(xyz[0]), "delta_lateral_m": -float(xyz[1]),
                             "delta_heading_rad": yaw, "data_kind": "synthetic"})
            if index == 30:  # Deliberately absent attempt, retained in the report.
                continue
            goal_id = f"synthetic-goal-{index}"
            mark(root, trial_id, "start", goal_id=goal_id, stamp=start)
            for relative_t in np.arange(0., 6., .02):
                t = start+float(relative_t)
                q = np.zeros(29)
                q[12] = .08*math.sin(.7*relative_t)
                if index != 25:  # Missing body stream for one full attempt.
                    writers["body"].append({"kind": "body", "t": t, "q": q.tolist(),
                        "dq": ([.12]*12+[0.]*17 if index and relative_t < 1.4 else [0.]*29),
                        "imu_wxyz": [xyzw[3], 0., 0., xyzw[2]],
                        "clock_id": "unix", "stamp_kind": "sample", "time_uncertainty_s": .001})
            for relative_t in np.arange(0., 6., .1):
                t = start+float(relative_t)
                nav_xyz = xyz + ([.01, -.005, 0.] if index else np.zeros(3))
                for kind, frame in (("nav_pose", "map"), ("odom", "odom")):
                    writers["ros"].append({"kind": kind, "t": t, "frame_id": frame,
                        "child_frame_id": "base_link", "xyz": nav_xyz.tolist(), "xyzw": xyzw})
                writers["ros"].append({"kind": "velocity_command", "t": t, "source": "/cmd_vel_smoothed",
                    "linear": [.25 if relative_t < .8 else 0., 0., 0.], "angular": [0., 0., 0.]})
            if index:
                writers["ros"].append({"kind": "nav_status", "t": start+.1, "goal_time": start+.05,
                    "goal_id": goal_id, "status": "executing"})
                writers["ros"].append({"kind": "nav_status", "t": start+1., "goal_time": start+.05,
                    "goal_id": goal_id, "status": "aborted" if index == 26 else "succeeded"})
            T_map_pelvis = pose_matrix(xyz, xyzw) @ base_pelvis
            for frame_index, relative_t in enumerate((2.2, 3., 4.3)):
                t = start+relative_t
                q = np.zeros(29)
                q[12] = .08*math.sin(.7*relative_t)
                T = T_map_pelvis @ camera_transform(calibration, q)
                world_rays = T[:3, :3] @ rays
                depth = ((.8-T[0, 3])/world_rays[0]).reshape(height, width)
                depth += rng.normal(0., .00025, depth.shape)
                if index == 28:
                    depth[:] = 0.
                button_camera = np.linalg.inv(T) @ [.8, 0., 1.15, 1.]
                uvw = K @ button_camera[:3]
                button_uv = (uvw[:2]/uvw[2]).tolist()
                rgb = np.full((height, width, 3), 150, np.uint8)
                rgb[(u-button_uv[0])**2+(v-button_uv[1])**2 < 16] = [60, 90, 220]
                frame_id = f"{trial_id}_{frame_index}"
                relative_path = f"frames/{frame_id}.npz"
                np.savez_compressed(root/relative_path, rgb=rgb, depth_m=depth.astype(np.float32), K=K)
                writers["frames"].append({"id": frame_id, "camera": "head_camera", "optical_frame": "head_optical",
                    "t": t, "received_at": t+.03, "clock_id": "unverified" if index == 29 else "unix",
                    "stamp_kind": "exposure", "depth_aligned": True, "path": relative_path})
                annotations.append({"frame_id": frame_id, "panel_polygon": [[5, 5], [90, 5], [90, 66], [5, 66]],
                    "button_uv": button_uv, "button_exclusion_px": 5, "method": "analytic_synthetic_fixture"})
            if index == 27:
                mark(root, trial_id, "manual_intervention", reason="synthetic intervention", stamp=start+4.5)
            mark(root, trial_id, "end", stamp=start+6.)
    finally:
        for writer in writers.values():
            writer.close()
    write_json(root/"annotations.json", {"frames": annotations})
    write_json(root/"expected.json", {"data_kind": "synthetic", "trials": expected,
                                     "note": "Analytic fixture truth; never read by the analyzer."})
    return manifest
