"""学習用HDF5と同時に撮影した外部映像の、独立した保存先。"""

from dataclasses import dataclass

import h5py
import numpy as np

from .dataset import DatasetReader


@dataclass(frozen=True)
class SynchronizedFrame:
    time_s: float
    observer_rgb: np.ndarray
    camera: dict
    lidar: dict | None
    base_pose_world: np.ndarray


def synchronized_frames(collection):
    """外部映像の取得時刻で結合する。制御行の時刻や身体状態を書き換えない。"""
    with (
        DatasetReader(collection / "dataset.h5") as reader,
        h5py.File(collection / "observer.h5", "r") as observer,
    ):
        if not reader.file.attrs["complete"] or not observer.attrs["complete"]:
            raise ValueError("記録が完了していません")
        for i in range(int(observer.attrs["committed_frames"])):
            episode, time_s = int(observer["episode_id"][i]), float(observer["time_s"][i])
            if "camera_index" in observer and observer["camera_index"][i] >= 0:
                camera = reader.sensor_frame("camera", int(observer["camera_index"][i]))
            else:
                # schema 1/2の外部映像は、同時刻の制御行を参照していた。
                camera = reader.sample(int(observer["state_row"][i]))["camera"]
            if camera is None or camera["episode_id"] != episode or abs(camera["time_s"] - time_s) > 1e-8:
                raise ValueError("外部映像とカメラ時刻が一致しません")
            yield SynchronizedFrame(
                time_s,
                observer["rgb"][i],
                camera,
                reader.latest_sensor_frame("lidar", episode, time_s),
                camera["world_from_optical"] @ np.linalg.inv(camera["base_from_optical"]),
            )


class ObserverWriter:
    def __init__(self, path, resolution):
        self.file = h5py.File(path, "x")
        self.file.attrs["complete"] = False
        self.file.attrs["committed_frames"] = 0
        self.file.create_dataset(
            "rgb",
            shape=(0, *resolution, 3),
            maxshape=(None, *resolution, 3),
            chunks=(1, *resolution, 3),
            dtype="u1",
            compression="lzf",
        )
        for name, dtype in [
            ("time_s", "f8"),
            ("episode_id", "i8"),
            ("state_row", "i8"),
            ("camera_index", "i8"),
        ]:
            self.file.create_dataset(name, (0,), maxshape=(None,), dtype=dtype)

    def append(self, observation, state_row, rgb, *, camera_index=-1):
        index = int(self.file.attrs["committed_frames"])
        for name, value in {
            "rgb": np.asarray(rgb, dtype=np.uint8),
            "time_s": observation.simulation_time_s,
            "episode_id": observation.episode_id,
            "state_row": state_row,
            "camera_index": camera_index,
        }.items():
            self.file[name].resize(index + 1, axis=0)
            self.file[name][index] = value
        self.file.attrs.modify("committed_frames", index + 1)
        self.file.flush()

    def close(self, *, complete=False):
        self.file.attrs.modify("complete", complete)
        self.file.close()
