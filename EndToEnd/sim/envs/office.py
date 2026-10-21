"""公式Office USDの取り込みと、同じ形状からの巡回用格子生成。"""

import hashlib
import json

import numpy as np

from g1_e2e.navigation.grid import NavigationGrid


def spawn_office(root, config):
    import isaaclab.sim as sim_utils

    path = config.usd_path(root)
    lock = config.directory(root) / "office-assets.lock.json"
    if not path.is_file() or not lock.is_file():
        raise FileNotFoundError("先にbash EndToEnd/prepare-office.shを実行してください")
    if hashlib.sha256(path.read_bytes()).hexdigest() != config.asset_sha256:
        raise ValueError("Office USDのSHA-256が設定と異なります")
    # 依存USDはローカル参照。アセット自身の物理collisionもそのまま取り込む。
    asset = sim_utils.UsdFileCfg(usd_path=str(path))
    asset.func("/World/Office", asset)
    return {
        "source": config.url,
        "root_prim": "/World/Office",
        "asset_lock_sha256": hashlib.sha256(lock.read_bytes()).hexdigest(),
        "asset_files": len(json.loads(lock.read_text())["files"]),
        "floor_height_m": config.floor_height_m,
    }


def navigation_grid(vertices, faces, config):
    from PIL import Image, ImageDraw
    from scipy.ndimage import distance_transform_edt, label

    triangles = vertices[faces]
    low, high = triangles[..., 2].min(axis=1), triangles[..., 2].max(axis=1)
    floor = np.maximum(np.abs(low - config.floor_height_m), np.abs(high - config.floor_height_m)) < 0.04
    if not floor.any():
        raise ValueError("設定した床高さに水平面がありません。Officeの単位・床高さを確認してください")
    resolution = config.grid_resolution_m
    bounds = np.asarray(config.navigation_bounds_xy_m)
    origin = np.floor(bounds[:, 0] / resolution) * resolution - resolution
    shape = np.ceil((bounds[:, 1] - origin) / resolution).astype(int) + 2
    if shape.max() > 2500:
        raise ValueError("室内格子が過大です。USDの単位・resolutionを確認してください")
    support, obstacles = Image.new("1", tuple(shape)), Image.new("1", tuple(shape))
    draw_support, draw_obstacles = ImageDraw.Draw(support), ImageDraw.Draw(obstacles)
    slab_low, slab_high = config.obstacle_height_range_m
    blocking = (high > config.floor_height_m + slab_low) & (low < config.floor_height_m + slab_high)
    for mask, drawing in [(floor, draw_support), (blocking, draw_obstacles)]:
        for triangle in triangles[mask, :, :2]:
            pixels = (triangle - origin) / resolution
            drawing.polygon([tuple(point) for point in pixels], fill=1)
    traversable = np.asarray(support, bool) & ~np.asarray(obstacles, bool)
    # 境界の外も未知・通行不可として距離変換する。
    clearance = distance_transform_edt(np.pad(traversable, 1))[1:-1, 1:-1] * resolution
    walkable = traversable & (clearance >= config.robot_radius_m + config.tracking_margin_m)
    components, count = label(walkable)
    if not count:
        raise ValueError("G1の半径を確保できるOfficeの通行領域がありません")
    sizes = np.bincount(components.reshape(-1))
    sizes[0] = 0
    walkable &= components == sizes.argmax()
    return NavigationGrid(origin, resolution, walkable, clearance.astype(np.float32))
