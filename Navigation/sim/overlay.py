"""ビューアの 3D 画面に「どこからどこへ行くのか」を描く。

これが無いと、機体が歩いているのは見えても**目標が見えない**ので、
どこへ向かっているのか・いま何番目のウェイポイントなのかが分からない
（実際に見て分からなかったので足した）。

描くもの:

| 見た目 | 意味 |
|---|---|
| 灰緑の低いポール | 到達済みのウェイポイント |
| 水色の高いポール | これから行くウェイポイント |
| **橙の太いポール + 足元の輪** | **いま 1102 で指示している目標**（区間ごとに動く） |
| 黄色の細い線 | 歩いてきた軌跡 |

MuJoCo の `viewer.user_scn` に毎コマ geom を積み直す方式。物理には影響しない。
"""

from __future__ import annotations

import numpy as np

# ウェイポイントのポールの高さ[m]。機体（約1.3m）より高くして、
# 真上から見下ろす既定のカメラでも機体に隠れないようにする。
PENDING_POLE_HEIGHT_M = 1.8
REACHED_POLE_HEIGHT_M = 0.5

# いま指示している目標のポール。ウェイポイントより太く高くして一目で分かるようにする。
TARGET_POLE_HEIGHT_M = 2.4
TARGET_POLE_RADIUS_M = 0.06
TARGET_RING_RADIUS_M = 0.35

WAYPOINT_POLE_RADIUS_M = 0.035

# 軌跡の点を打つ間隔[m]。細かすぎると geom を食い潰す。
TRAIL_STEP_M = 0.15
TRAIL_WIDTH_M = 0.02

# 積める geom の上限に近づいたら軌跡から捨てる。
# `user_scn` の maxgeom は viewer 側が決めるので、実際の値を見て切る。
TRAIL_MAX_POINTS = 400

_REACHED_RGBA = (0.35, 0.65, 0.45, 0.9)
_PENDING_RGBA = (0.55, 0.80, 0.95, 0.75)
_TARGET_RGBA = (1.00, 0.55, 0.10, 0.95)
_TRAIL_RGBA = (0.95, 0.90, 0.35, 0.8)


class Trail:
    """歩いた跡。一定距離ごとに点を足すだけ。"""

    def __init__(self, step_m: float = TRAIL_STEP_M) -> None:
        self._step = step_m
        self.points: list[tuple[float, float]] = []

    def add(self, x: float, y: float) -> None:
        if self.points:
            last_x, last_y = self.points[-1]
            if (x - last_x) ** 2 + (y - last_y) ** 2 < self._step**2:
                return
        self.points.append((x, y))
        if len(self.points) > TRAIL_MAX_POINTS:
            del self.points[0]


def draw(scene, *, waypoints, reached: int, target, trail: Trail) -> None:
    """`scene` に目印を**足す**。毎コマ呼ぶ。

    **消すのは呼び側の仕事。** `viewer.user_scn` は毎コマ `ngeom = 0` に
    戻してから呼ぶ。逆にオフスクリーン描画（`Renderer`）では、
    `update_scene()` が積んだ部屋と機体の geom の**後ろに足す**必要があるので
    消してはいけない。ここで勝手に消すと、他人のシーンを壊す
    （実際に一度やって、部屋と機体が真っ黒になった）。

    `reached` は到達済みのウェイポイント数。`waypoints[0]` は出発点なので、
    `waypoints[1 + reached]` が次に目指すウェイポイントになる。
    """

    _draw_trail(scene, trail)
    for order, pose in enumerate(waypoints):
        done = order <= reached
        _pole(
            scene,
            pose.x,
            pose.y,
            REACHED_POLE_HEIGHT_M if done else PENDING_POLE_HEIGHT_M,
            WAYPOINT_POLE_RADIUS_M,
            _REACHED_RGBA if done else _PENDING_RGBA,
            label=f"WP{order}",
        )
    if target is not None:
        _pole(
            scene, target.x, target.y, TARGET_POLE_HEIGHT_M,
            TARGET_POLE_RADIUS_M, _TARGET_RGBA, label="1102",
        )
        _ring(scene, target.x, target.y)


def _draw_trail(scene, trail: Trail) -> None:
    for (x0, y0), (x1, y1) in zip(trail.points, trail.points[1:]):
        _line(scene, (x0, y0, 0.03), (x1, y1, 0.03), TRAIL_WIDTH_M, _TRAIL_RGBA)


def _pole(scene, x, y, height, radius, rgba, *, label="") -> None:
    _line(scene, (x, y, 0.02), (x, y, height), radius, rgba, label=label)


def _ring(scene, x, y) -> None:
    """目標の足元に置く輪。真上から見ても位置が分かるようにする。"""

    import mujoco

    if not _room_for(scene):
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        int(mujoco.mjtGeom.mjGEOM_CYLINDER),
        np.array([TARGET_RING_RADIUS_M, TARGET_RING_RADIUS_M, 0.008]),
        np.array([float(x), float(y), 0.008]),
        np.eye(3).ravel(),
        np.array(_TARGET_RGBA, np.float32),
    )
    geom.label = ""
    scene.ngeom += 1


def _line(scene, start, end, width, rgba, *, label="") -> None:
    import mujoco

    if not _room_for(scene):
        return
    geom = scene.geoms[scene.ngeom]
    mujoco.mjv_initGeom(
        geom,
        int(mujoco.mjtGeom.mjGEOM_CAPSULE),
        np.zeros(3),
        np.zeros(3),
        np.eye(3).ravel(),
        np.array(rgba, np.float32),
    )
    mujoco.mjv_connector(
        geom,
        int(mujoco.mjtGeom.mjGEOM_CAPSULE),
        float(width),
        np.array(start, dtype=float),
        np.array(end, dtype=float),
    )
    geom.label = label
    scene.ngeom += 1


def _room_for(scene) -> bool:
    """geom の枠が残っているか。超えると MuJoCo が黙って壊れる。"""

    return scene.ngeom < scene.maxgeom
