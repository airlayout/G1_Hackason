"""Livoxが公開するMID-360角度表。独自のランダム走査は生成しない。"""

from functools import lru_cache
import hashlib
import json
from pathlib import Path

import numpy as np


class LivoxPattern:
    def __init__(self, table, point_rate_hz=200_000, *, repeat=False):
        table = np.asarray(table, dtype=np.float64)
        if table.ndim != 2 or table.shape[1] != 3 or not len(table) or not np.isfinite(table).all():
            raise ValueError("Livox CSVには有限のindex・azimuth・zenithが必要です")
        if not np.array_equal(table[:, 0], np.arange(1, len(table) + 1)):
            raise ValueError("公開MID-360 CSVの第1列は秒ではなく連続した点番号です")
        if point_rate_hz != 200_000:
            raise ValueError("MID-360の公称first-return点頻度は200000 points/sです")
        self.table, self.rate, self.repeat = table, point_rate_hz, repeat
        azimuth, zenith = np.deg2rad(table[:, 1]), np.deg2rad(table[:, 2])
        # Gazebo公式pluginのpitch=zenith-90°と同じ、x前/y左/z上の方向。
        self.directions = np.column_stack(
            [
                np.sin(zenith) * np.cos(azimuth),
                np.sin(zenith) * np.sin(azimuth),
                np.cos(zenith),
            ]
        ).astype(np.float32)
        self.table.flags.writeable = self.directions.flags.writeable = False
        self.duration_s = len(table) / self.rate

    def require_duration(self, duration_s):
        if not self.repeat and duration_s > self.duration_s + 1e-9:
            raise ValueError(
                f"メーカー公開走査表は{self.duration_s:g}秒分です。"
                "それ以上の実行には長い走査表が必要です。repeat_pattern=trueは明示的な周期再生になります"
            )

    def sample(self, indices):
        indices = np.asarray(indices, dtype=np.int64)
        if np.any(indices < 0):
            raise ValueError("点番号は非負です")
        if not self.repeat and np.any(indices >= len(self.table)):
            raise ValueError("メーカー公開走査表の末尾に達しました")
        return self.directions[indices % len(self.table)]

    @property
    def metadata(self):
        elevation = 90 - self.table[:, 2]
        return {
            "model": "Livox public MID-360 simulation angle table",
            "point_rate_hz": self.rate,
            "table_rows": len(self.table),
            "table_duration_s": self.duration_s,
            "repeat_pattern": self.repeat,
            "csv_first_column": "point index, NOT seconds despite Time/s header",
            "timestamp_source": "point index / published 200000 points/s; not device timestamp capture",
            "csv_elevation_bounds_deg": [float(elevation.min()), float(elevation.max())],
            "nominal_hardware_elevation_bounds_deg": [-7, 52],
            "fov_note": "vendor simulation table extends ~0.22 degrees beyond nominal hardware FOV; not clipped",
            "ring": "not present in vendor CSV; ring_valid=false",
        }


@lru_cache(maxsize=4)
def load_livox_pattern(root: Path, relative_path: str, repeat: bool = False):
    path = root / relative_path
    manifest = json.loads((root / "assets/manifest.json").read_text())
    item = next(
        (item for item in manifest["files"] if item["path"] == str(path.relative_to(root / "assets/cache"))),
        None,
    )
    if item is None or not path.is_file():
        raise FileNotFoundError("先にsetup.sh --with-demoを実行し、固定版MID-360 CSVを取得してください")
    if hashlib.sha256(path.read_bytes()).hexdigest() != item["sha256"]:
        raise ValueError("MID-360走査表のSHA-256がmanifestと異なります")
    result = LivoxPattern(np.loadtxt(path, delimiter=",", skiprows=1), repeat=repeat)
    result.source = {"url": item["url"], "sha256": item["sha256"]}
    return result
