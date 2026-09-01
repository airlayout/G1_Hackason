"""sim / mock / 実機で共通に使うNavigationの本体。

**標準ライブラリのみに依存する。** `scripts/ci/run_all_tests.sh` は
`pip install` を持たず、`Mapping/` も同じ理由で標準ライブラリだけで書かれている。
numpyが要る処理（点群の大量変換など）は `Navigation/sim/` 側に置くこと。

| モジュール | 役割 |
|---|---|
| `geometry` | 平面姿勢と四元数<->yaw変換。表現の違いをここに閉じ込める |
| `protocol` | `slam_operate` のJSON組み立てとパース。通信手段は知らない |
| `occupancy` | 地図PCD -> 2D占有格子。直線が歩けるかの判定に使う |
| `route` | ウェイポイント列 -> 10m未満の直線区間列 |
"""

from __future__ import annotations

from .geometry import Pose2D, normalize_angle, quaternion_to_yaw, yaw_to_quaternion
from .occupancy import OccupancyGrid, build_grid, load_grid, load_pcd
from .route import MAX_SEGMENT_M, RouteError, Segment, plan_route

__all__ = [
    "MAX_SEGMENT_M",
    "OccupancyGrid",
    "Pose2D",
    "RouteError",
    "Segment",
    "build_grid",
    "load_grid",
    "load_pcd",
    "normalize_angle",
    "plan_route",
    "quaternion_to_yaw",
    "yaw_to_quaternion",
]
