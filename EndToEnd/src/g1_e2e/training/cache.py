"""生HDF5を保持したまま、学習用の観測トークンと疎な補助教師を事前生成する。"""

from dataclasses import asdict
import hashlib
import json
import time

import h5py
import numpy as np
import torch

from g1_e2e.data.dataset import DatasetReader
from g1_e2e.data.targets import SupervisionReader
from g1_e2e.models import ModelInputAdapter, SensorTokens
from .field import RollingField


class CachedReader(DatasetReader):
    def __init__(self, path):
        super().__init__(path)
        self._frames = {}

    def sensor_frame(self, name, index):
        previous, value = self._frames.get(name, (None, None))
        if previous != index:
            value = super().sensor_frame(name, index)
            self._frames[name] = (index, value)
        return value


def prepare(source, destination, config, *, aux_stride=10, field_queries=96, ray_stride=20):
    destination.parent.mkdir(parents=True, exist_ok=True)
    started = time.perf_counter()
    with CachedReader(source) as reader:
        if not reader.file.attrs["complete"] or not reader.file.attrs.get("patrol_training_accepted", False):
            raise ValueError(f"未完了または巡回品質が不採用です: {source}")
        if not reader.file.attrs.get("input_integrity_accepted", False):
            raise ValueError(f"入力監査が未通過です: {source}")
        n = len(reader)
        if np.unique(reader.file["state/episode_id"][:]).size != 1:
            raise ValueError("学習cacheは1ファイル1 episodeです")
        adapter = ModelInputAdapter(config, control_dt_s=reader.metadata["control_dt_s"])
        supervision = SupervisionReader(reader)
        field = RollingField(
            window_s=config.context_window * config.recurrence_period * config.control_dt_s,
            ray_stride=ray_stride,
        )
        trajectory_times = np.array([0.1, 0.5, 1.0, 2.0], np.float32)
        temporary = destination.with_suffix(".partial.h5")
        with h5py.File(temporary, "x") as out:
            metadata = {
                "source": str(source.resolve()),
                "source_bytes": source.stat().st_size,
                "source_metadata_sha256": hashlib.sha256(
                    reader.file.attrs["metadata_json"].encode()
                ).hexdigest(),
                "model_config": asdict(config),
                "aux_stride": aux_stride,
                "field_queries": field_queries,
                "field_ray_stride": ray_stride,
                "field_window_s": field.window_s,
                "field_world_bounds_m": field.bounds.tolist(),
                "field_resolution_m": field.resolution,
                "trajectory_times_s": trajectory_times.tolist(),
                "observation_feature_dtype": "float16 cache, converted to float32 for model input",
                "teacher": reader.metadata["patrol_teacher"],
            }
            out.attrs.update(schema_version=1, complete=False, rows=n, metadata_json=json.dumps(metadata))

            def create(name, shape, dtype):
                return out.create_dataset(
                    name,
                    shape=(n, *shape),
                    dtype=dtype,
                    chunks=(1 if np.prod(shape) * np.dtype(dtype).itemsize > 4096 else min(64, n), *shape),
                    compression="lzf",
                )

            first = adapter(reader.sample(0))
            streams = {}
            for name, tokens in first.items():
                streams[name] = {}
                for key in ("features", "valid", "positions", "age_s"):
                    value = getattr(tokens, key)
                    if value is None:
                        continue
                    dtype = np.bool_ if key == "valid" else np.float16 if key == "features" else np.float32
                    streams[name][key] = create(f"inputs/{name}/{key}", value.shape[1:], dtype)
            velocity, velocity_valid = (
                create("targets/velocity", (3,), np.float32),
                create("targets/velocity_valid", (), np.bool_),
            )
            auxiliary = create("targets/auxiliary", (), np.bool_)
            aux_count = (n - 1) // aux_stride + 1
            aux = {}
            for key, shape, dtype in (
                ("coordinates", (field_queries, 3), np.float32),
                ("occupied", (field_queries,), np.float32),
                ("observed", (field_queries,), np.float32),
                ("poses", (4, 7), np.float32),
                ("pose_valid", (4,), np.bool_),
            ):
                aux[key] = out.create_dataset(
                    f"aux/{key}", shape=(aux_count, *shape), dtype=dtype, compression="lzf"
                )
            last_scan = -1
            for index in range(n):
                sample = reader.sample(index)
                inputs = adapter(sample)
                for name, values in streams.items():
                    for key, dataset in values.items():
                        dataset[index] = getattr(inputs[name], key)[0].numpy()
                velocity[index], velocity_valid[index] = supervision.velocity(index)
                scan_index = int(sample["state"]["lidar_index"])
                if scan_index > last_scan:
                    for scan in range(last_scan + 1, scan_index + 1):
                        field.add(reader.sensor_frame("lidar", scan))
                    last_scan = scan_index
                auxiliary[index] = index % aux_stride == 0
                if index % aux_stride == 0:
                    target = field.sample(
                        sample["state"]["base_pose_world"],
                        float(sample["state"]["time_s"]),
                        count=field_queries,
                        seed=index,
                    )
                    trajectory = supervision.trajectory(index, trajectory_times)
                    row = index // aux_stride
                    for key in ("coordinates", "occupied", "observed"):
                        aux[key][row] = getattr(target, key)
                    aux["poses"][row], aux["pose_valid"][row] = trajectory.poses, trajectory.valid
                if (index + 1) % 500 == 0:
                    print(f"[CACHE] {source.parent.name} {index + 1}/{n}", flush=True)
            out.attrs["complete"] = True
        temporary.replace(destination)
    return {
        "source": str(source.resolve()),
        "cache": str(destination.resolve()),
        "rows": n,
        "wall_seconds": time.perf_counter() - started,
    }


class TrainingEpisode:
    def __init__(self, path, device):
        self.file, self.device = h5py.File(path, "r"), device
        if not self.file.attrs.get("complete", False):
            raise ValueError("未完了の学習cacheです")
        self.metadata = json.loads(self.file.attrs["metadata_json"])
        # 1 episodeずつRAMへ読む。学習中の細かいHDF5 I/OとCPU待ちを避ける。
        self.inputs = {
            name: {key: value[:] for key, value in group.items()}
            for name, group in self.file["inputs"].items()
        }
        self.targets = {key: value[:] for key, value in self.file["targets"].items()}
        self.aux = {key: value[:] for key, value in self.file["aux"].items()}

    def __len__(self):
        return int(self.file.attrs["rows"])

    def tokens(self, index):
        return {
            name: SensorTokens(
                **{
                    key: torch.as_tensor(value[index : index + 1], device=self.device).to(
                        torch.bool if key == "valid" else torch.float32
                    )
                    for key, value in stream.items()
                }
            )
            for name, stream in self.inputs.items()
        }

    def target(self, index):
        result = {key: value[index] for key, value in self.targets.items()}
        if result["auxiliary"]:
            result.update(
                {key: value[index // self.metadata["aux_stride"]] for key, value in self.aux.items()}
            )
        return result

    def close(self):
        self.file.close()


class TrainingBatch:
    """同じ長さの独立episodeを束ねる。時間軸とepisode境界は混ぜない。"""

    def __init__(self, paths, device):
        self.episodes = []
        try:
            for path in paths:
                self.episodes.append(TrainingEpisode(path, device))
            self.metadata = self.episodes[0].metadata
            if len({len(episode) for episode in self.episodes}) != 1:
                raise ValueError("同じbatchのepisode長を揃えてください")
        except BaseException:
            self.close()
            raise
        self.device, self.batch_size = device, len(paths)

    def __len__(self):
        return len(self.episodes[0])

    def tokens(self, index):
        result = {}
        for name, stream in self.episodes[0].inputs.items():
            values = {}
            for key in stream:
                array = np.stack([episode.inputs[name][key][index] for episode in self.episodes])
                values[key] = torch.as_tensor(array, device=self.device).to(
                    torch.bool if key == "valid" else torch.float32
                )
            result[name] = SensorTokens(**values)
        return result

    def target(self, index):
        targets = [episode.target(index) for episode in self.episodes]
        if len({bool(target["auxiliary"]) for target in targets}) != 1:
            raise ValueError("同じbatchの補助教師の取得周期が一致しません")
        return {
            key: bool(targets[0][key])
            if key == "auxiliary"
            else np.stack([target[key] for target in targets])
            for key in targets[0]
        }

    def close(self):
        for episode in self.episodes:
            episode.close()
