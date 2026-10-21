"""全環境の取得周期・因果参照、URDFカメラ姿勢とLiDAR幾何を検査する。"""

import argparse
import json
from pathlib import Path

import h5py
import numpy as np

from g1_e2e.data.dataset import DatasetReader
from g1_e2e.data.geometry import MOUNT_FROM_OPTICAL, mount_matrix
from scipy.spatial.transform import Rotation


def check(directory):
    import warp as wp
    from sim.batch.kernels import ray_distances

    summary = json.loads((directory / "summary.json").read_text())
    mesh_data = np.load(directory / "scene-mesh.npz")
    mesh = wp.Mesh(
        wp.array(mesh_data["vertices"], dtype=wp.vec3f, device="cpu"),
        wp.array(mesh_data["faces"].reshape(-1), dtype=wp.int32, device="cpu"),
    )
    reports = []
    for path in sorted(directory.glob("env_*/dataset.h5")):
        with DatasetReader(path) as reader:
            f = reader.file
            n = len(reader)
            if not summary.get("complete") or not f.attrs["complete"] or n != summary["steps_per_env"]:
                raise ValueError(f"{path}: 収集が完了していません")
            expected = {
                "camera": round(n * 0.02 * 30),
                "lidar": n // 5,
                **{f"imu_{name}": n * 12 + 1 for name in ("pelvis", "torso", "lidar")},
            }
            for name, count in expected.items():
                times = f[f"sensors/{name}/time_s"][:]
                if len(times) != count:
                    raise ValueError(f"{path}: {name}のframe欠損 {len(times)}/{count}")
                period = 1 / (30 if name == "camera" else 10 if name == "lidar" else 600)
                target = (np.arange(count) + (name == "lidar")) * period
                np.testing.assert_allclose(times, target, atol=1e-9, rtol=0)
            for index in (0, n // 2, n - 1):
                reader.sample(index)
            camera = f["sensors/camera"]
            if camera["rgb"].shape[1:] != (480, 640, 3):
                raise ValueError(f"{path}: RGBの画像解像度が変更されています")
            if set(camera) != {"episode_id", "time_s", "rgb", "world_from_optical", "base_from_optical"}:
                raise ValueError(f"{path}: RGB以外のカメラ入力が保存されています")
            mount = reader.metadata["sensor_config"]["camera"]["mount"]
            parent_from_optical = mount_matrix(mount["position_m"], mount["rpy_deg"]) @ MOUNT_FROM_OPTICAL
            parent_from_imu = f["calibration/imu/torso/parent_from_sensor"][:]
            for index in (0, len(camera["time_s"]) // 2, len(camera["time_s"]) - 1):
                if np.max(camera["rgb"][index]) == 0:
                    raise ValueError(f"{path}: RGB frame {index}が全黒です")
                tick = round(float(camera["time_s"][index]) * 600)
                world_from_parent = f["sensors/imu_torso/world_from_sensor"][tick] @ np.linalg.inv(
                    parent_from_imu
                )
                np.testing.assert_allclose(
                    camera["world_from_optical"][index],
                    world_from_parent @ parent_from_optical,
                    atol=0.003,
                    rtol=0,
                )
            errors = []
            lidar = f["sensors/lidar"]
            for index in (0, len(lidar["time_s"]) // 2, len(lidar["time_s"]) - 1):
                selected = np.flatnonzero(lidar["valid"][index])[::157]
                if len(selected) == 0:
                    raise ValueError(f"{path}: LiDARの有効反射点がありません")
                origins = lidar["measurement_positions_world_m"][index][selected]
                directions = (
                    Rotation.from_quat(lidar["measurement_orientation_xyzw"][index][selected])
                    .apply(lidar["ray_directions_sensor"][index][selected])
                    .astype(np.float32)
                )
                ranges = wp.empty(len(selected), dtype=wp.float32, device="cpu")
                wp.launch(
                    ray_distances,
                    dim=len(selected),
                    inputs=[
                        mesh.id,
                        wp.array(origins, dtype=wp.vec3f, device="cpu"),
                        wp.array(directions, dtype=wp.vec3f, device="cpu"),
                        ranges,
                    ],
                    device="cpu",
                )
                expected_ranges = ranges.numpy()
                if np.any(expected_ranges <= 0):
                    raise ValueError(f"{path}: 有効LiDAR rayが静的シーンと交差しません")
                actual_ranges = lidar["ranges_m"][index][selected]
                errors.extend(np.abs(actual_ranges - expected_ranges).tolist())
                np.testing.assert_allclose(
                    lidar["points_world_m"][index][selected],
                    origins + directions * actual_ranges[:, None],
                    atol=0.002,
                    rtol=1e-5,
                )
            median, p95 = float(np.median(errors)), float(np.quantile(errors, 0.95))
            if median > 0.001 or p95 > 0.005:
                raise ValueError(f"{path}: LiDARと保存poseが不一致 median={median:.4f}m p95={p95:.4f}m")
            reports.append(
                {
                    "environment": path.parent.name,
                    "rows": n,
                    "frames": expected,
                    "lidar_pose_median_error_m": median,
                    "lidar_pose_p95_error_m": p95,
                    "patrol_training_accepted": bool(f.attrs["patrol_training_accepted"]),
                }
            )
    if len(reports) != summary["num_envs"]:
        raise ValueError("保存された環境数が不足しています")
    return {"accepted": True, "environments": reports}


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("directory", type=Path)
    args = parser.parse_args()
    try:
        report = check(args.directory)
    except Exception as error:
        for path in args.directory.glob("env_*/dataset.h5"):
            with h5py.File(path, "r+") as file:
                file.attrs["input_integrity_accepted"] = False
        report = {"accepted": False, "error": str(error)}
        (args.directory / "parallel-audit.json").write_text(json.dumps(report, indent=2) + "\n")
        raise
    for path in args.directory.glob("env_*/dataset.h5"):
        with h5py.File(path, "r+") as file:
            file.attrs["input_integrity_accepted"] = True
    (args.directory / "parallel-audit.json").write_text(json.dumps(report, indent=2) + "\n")
    print(json.dumps(report, indent=2))


if __name__ == "__main__":
    main()
