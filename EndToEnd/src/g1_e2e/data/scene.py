"""USD室内シーンの出所と、巡回教師の幾何・移動設定。Isaacには依存しない。"""

from dataclasses import dataclass
from pathlib import Path
from urllib.parse import urljoin

import numpy as np
import yaml


@dataclass(frozen=True)
class OfficeConfig:
    asset_root_url: str
    asset_path: str
    asset_sha256: str
    cache_directory: str
    source_documentation: str
    floor_height_m: float = 0.0
    navigation_bounds_xy_m: tuple[tuple[float, float], tuple[float, float]] = ((-24.0, 6.0), (-30.0, 60.0))
    grid_resolution_m: float = 0.1
    obstacle_height_range_m: tuple[float, float] = (0.12, 1.7)
    robot_radius_m: float = 0.4
    tracking_margin_m: float = 0.1
    cruise_speed_m_s: float = 0.3
    lookahead_m: float = 0.8
    goal_tolerance_m: float = 0.55

    def __post_init__(self):
        if not self.asset_root_url.startswith("https://") or not self.asset_root_url.endswith("/"):
            raise ValueError("asset_root_urlは末尾/を持つHTTPS URLです")
        if Path(self.asset_path).is_absolute() or ".." in Path(self.asset_path).parts:
            raise ValueError("asset_pathはアセットroot内の相対パスです")
        if len(self.asset_sha256) != 64 or any(c not in "0123456789abcdef" for c in self.asset_sha256):
            raise ValueError("OfficeのSHA-256を固定してください")
        positive = [
            self.grid_resolution_m,
            self.robot_radius_m,
            self.cruise_speed_m_s,
            self.lookahead_m,
            self.goal_tolerance_m,
        ]
        low, high = self.obstacle_height_range_m
        if not np.isfinite(self.tracking_margin_m) or self.tracking_margin_m < 0:
            raise ValueError("tracking_margin_mは有限の非負値です")
        bounds = np.asarray(self.navigation_bounds_xy_m)
        if bounds.shape != (2, 2) or not np.isfinite(bounds).all() or np.any(bounds[:, 0] >= bounds[:, 1]):
            raise ValueError("navigation_bounds_xy_mはXY各軸の有限なmin < maxです")
        if (
            not np.isfinite([self.floor_height_m, low, high, *positive]).all()
            or min(positive) <= 0
            or not 0 < low < high
        ):
            raise ValueError("シーンの寸法・速度・障害物高さが不正です")

    @classmethod
    def load(cls, path: Path):
        data = yaml.safe_load(path.read_text())
        if "obstacle_height_range_m" in data:
            data["obstacle_height_range_m"] = tuple(data["obstacle_height_range_m"])
        return cls(**data)

    @property
    def url(self):
        return urljoin(self.asset_root_url, self.asset_path)

    def directory(self, root: Path):
        return root / self.cache_directory

    def usd_path(self, root: Path):
        return self.directory(root) / self.asset_path
