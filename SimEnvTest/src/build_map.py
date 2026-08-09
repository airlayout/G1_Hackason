"""スキャンと真値 odom から直接 2D 占有格子地図を作る。

slam_toolbox を使わずに地図を作る。Isaac Sim は真値の姿勢を持っているため、
SLAM が行う姿勢推定は本来不要で、スキャンを姿勢どおりに重ねるだけでよい。

slam_toolbox を使わない理由:
    Warehouse は同じ形の棚が並び 2D スキャンでは特徴が乏しいため、
    相関スキャンマッチャが誤マッチして地図が壊れた（実測で 6 回試行）。
    探索範囲を絞っても map -> odom に 1.3 m / 33 度の補正が乗り、
    地図の輪郭が歪んだ。真値 odom があるならこの推定自体が不要。

出力は Nav2 がそのまま読める .pgm / .yaml 形式。

実行方法:
    source env.sh && python3 src/build_map.py --output maps/warehouse
"""

from __future__ import annotations

import argparse
import math
import sys

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy
from sensor_msgs.msg import LaserScan

# 地図の解像度 [m/cell]。Nav2 の標準的な値。
RESOLUTION: float = 0.05
# 占有格子の値（ROS の慣習）
UNKNOWN: int = -1
FREE: int = 0
OCCUPIED: int = 100
# .pgm に書き出すときの画素値
PGM_UNKNOWN: int = 205
PGM_FREE: int = 254
PGM_OCCUPIED: int = 0
# 障害物と判定する対数オッズの閾値
LOG_ODDS_OCCUPIED: float = 1.0
LOG_ODDS_FREE: float = -1.0
# 1 回の観測で加算する対数オッズ。
# hit を大きく、miss を小さくする。同じ場所を何度も通ると
# レイの通過（空き）の回数が当たり（障害物）の回数を大きく上回るため、
# miss が大きいと壁が消える（実測: 障害物が 833 セル = 0.0% しか残らなかった）。
HIT_GAIN: float = 2.0
MISS_GAIN: float = 0.05
# 対数オッズの上下限。これが無いと何度も観測した空きが際限なく
# 負に振れ、あとから壁を見つけても打ち消せない。
LOG_ODDS_MAX: float = 6.0
LOG_ODDS_MIN: float = -2.0
# 地図の余白 [m]
MARGIN: float = 2.0
# この秒数スキャンが来なければ Isaac Sim が終了したとみなして収集を打ち切る。
# 待ち続けてもデータは増えず、そこまでの分で地図を作るほうがよい。
IDLE_TIMEOUT_SEC: float = 30.0
# 地図作成に使うレイの最大長 [m]。
#
# 短くすると壁を貫通する長いレイを除ける一方、壁を定義する見通しまで
# 失われる。実測（9497 スキャン）での比較:
#     30 m: 探索 3030 m2、壁は出るが扇状のノイズあり  <- 最良
#     25 m: 探索 1925 m2、ノイズは減るが壁も痩せる
#     12 m: 探索  631 m2、地図の形が失われる
# 現状は制限しないのが最良。ノイズの根本原因は別途調査が必要。
MAP_MAX_RAY: float = 30.0
# 障害物セルとして残すために必要な、8 近傍にある障害物の数。
#
# 1 セル幅の直線の壁は近傍が 2 個（左右の隣）しかない。3 にすると
# 実在の壁まで消えるため 2 とする。孤立した単発のノイズは近傍 0〜1 なので
# これで除去できる。
MIN_OBSTACLE_NEIGHBORS: int = 2


class MapBuilder(Node):
    """/scan と /odom を集めて占有格子地図を作るノード。"""

    def __init__(self, duration_sec: float) -> None:
        super().__init__("map_builder")
        sensor_qos = QoSProfile(
            depth=50,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            durability=QoSDurabilityPolicy.VOLATILE,
        )
        self._latest_odom: tuple[float, float, float] | None = None
        # (点群, センサ位置) の並び。あとで格子に焼く。
        self.observations: list[tuple[np.ndarray, tuple[float, float]]] = []
        self._duration = duration_sec
        self.create_subscription(Odometry, "/odom", self._on_odom, 10)
        self.create_subscription(LaserScan, "/scan", self._on_scan, sensor_qos)

    def _on_odom(self, msg: Odometry) -> None:
        """直近の姿勢を保持する。"""
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        )
        self._latest_odom = (p.x, p.y, yaw)

    def _on_scan(self, msg: LaserScan) -> None:
        """スキャンを世界座標の点群に変換して保存する。"""
        if self._latest_odom is None:
            return
        x, y, yaw = self._latest_odom
        ranges = np.array(msg.ranges)
        # 有効な測距のみ使う。0.0（近すぎ）と inf（当たらず）は障害物ではない。
        # 遠すぎるレイは使わない（壁を貫通して扇状のノイズを作るため）
        valid = (
            np.isfinite(ranges)
            & (ranges > msg.range_min)
            & (ranges < min(msg.range_max, MAP_MAX_RAY))
        )
        idx = np.where(valid)[0]
        if idx.size == 0:
            return
        angles = msg.angle_min + idx * msg.angle_increment + yaw
        d = ranges[idx]
        points = np.stack([x + d * np.cos(angles), y + d * np.sin(angles)], axis=1)
        self.observations.append((points, (x, y)))


def bresenham_free_cells(
    x0: int, y0: int, x1: int, y1: int
) -> list[tuple[int, int]]:
    """センサから当たり点までの間の格子（＝空き）を列挙する。

    Bresenham の直線アルゴリズム。終点（障害物のセル）は含めない。
    """
    cells: list[tuple[int, int]] = []
    dx = abs(x1 - x0)
    dy = abs(y1 - y0)
    sx = 1 if x0 < x1 else -1
    sy = 1 if y0 < y1 else -1
    err = dx - dy
    cx, cy = x0, y0
    # 無限ループ防止（地図の対角より長くはならない）
    for _ in range(dx + dy + 2):
        if cx == x1 and cy == y1:
            break
        cells.append((cx, cy))
        e2 = 2 * err
        if e2 > -dy:
            err -= dy
            cx += sx
        if e2 < dx:
            err += dx
            cy += sy
    return cells


def build_grid(
    observations: list[tuple[np.ndarray, tuple[float, float]]]
) -> tuple[np.ndarray, float, float]:
    """観測から占有格子を作る。

    Returns:
        (格子, 原点X, 原点Y)。格子は [行, 列] で行が Y 方向。
    """
    all_points = np.concatenate([p for p, _ in observations], axis=0)
    origins = np.array([o for _, o in observations])

    lo_x = min(all_points[:, 0].min(), origins[:, 0].min()) - MARGIN
    lo_y = min(all_points[:, 1].min(), origins[:, 1].min()) - MARGIN
    hi_x = max(all_points[:, 0].max(), origins[:, 0].max()) + MARGIN
    hi_y = max(all_points[:, 1].max(), origins[:, 1].max()) + MARGIN

    width = int((hi_x - lo_x) / RESOLUTION) + 1
    height = int((hi_y - lo_y) / RESOLUTION) + 1
    print(f"[Map] 格子の大きさ: {width} x {height} セル "
          f"({width * RESOLUTION:.1f} x {height * RESOLUTION:.1f} m)")

    # 対数オッズで蓄積する。複数回の観測で確信度が上がる。
    log_odds = np.zeros((height, width), dtype=np.float32)

    def to_cell(wx: float, wy: float) -> tuple[int, int]:
        return (
            int((wx - lo_x) / RESOLUTION),
            int((wy - lo_y) / RESOLUTION),
        )

    # レイごとに Python でループすると 9500 スキャン × 270 本で
    # 30 分以上かかる。numpy で一括処理する。
    #
    # 各レイを等間隔にサンプリングして通過セルを求める（Bresenham の
    # 代わり）。解像度の半分の刻みで取れば、セルの取りこぼしは起きない。
    for index, (points, (ox, oy)) in enumerate(observations):
        if points.size == 0:
            continue

        # 長すぎるレイは落とす。キャッシュから読んだ場合は収集時の
        # フィルタが効いていないため、ここでも同じ条件を課す。
        ray_len = np.hypot(points[:, 0] - ox, points[:, 1] - oy)
        points = points[ray_len <= MAP_MAX_RAY]
        if points.size == 0:
            continue

        # 当たり点のセル座標
        pcx = ((points[:, 0] - lo_x) / RESOLUTION).astype(np.int32)
        pcy = ((points[:, 1] - lo_y) / RESOLUTION).astype(np.int32)
        inside = (
            (pcx >= 0) & (pcx < width) & (pcy >= 0) & (pcy < height)
        )
        pcx, pcy = pcx[inside], pcy[inside]
        hit_points = points[inside]
        if pcx.size == 0:
            continue

        # レイの通過セルを求める。最長のレイに合わせた数のサンプルを取り、
        # 各レイは自分の長さの範囲だけを使う（t は 0..1 の比率）。
        distances = np.hypot(hit_points[:, 0] - ox, hit_points[:, 1] - oy)
        max_samples = int(distances.max() / (RESOLUTION * 0.5)) + 2
        t = np.linspace(0.0, 1.0, max_samples)[None, :]  # (1, S)
        # 終点（障害物のセル）は含めない
        sx = ox + (hit_points[:, 0:1] - ox) * t  # (N, S)
        sy = oy + (hit_points[:, 1:2] - oy) * t

        fcx = ((sx - lo_x) / RESOLUTION).astype(np.int32).ravel()
        fcy = ((sy - lo_y) / RESOLUTION).astype(np.int32).ravel()
        valid = (
            (fcx >= 0) & (fcx < width) & (fcy >= 0) & (fcy < height)
        )
        fcx, fcy = fcx[valid], fcy[valid]

        # 同じセルを何度も引かないよう、このスキャン内では 1 回だけ数える
        flat_free = np.unique(fcy.astype(np.int64) * width + fcx)
        flat_hit = np.unique(pcy.astype(np.int64) * width + pcx)
        # 障害物になるセルは空きから除く（終点を含めないため）
        flat_free = flat_free[~np.isin(flat_free, flat_hit)]

        flat = log_odds.ravel()
        np.subtract.at(flat, flat_free, MISS_GAIN)
        np.add.at(flat, flat_hit, HIT_GAIN)
        # 上下限を課す。これが無いと何度も観測した空きが際限なく負に振れる。
        np.clip(log_odds, LOG_ODDS_MIN, LOG_ODDS_MAX, out=log_odds)

        if (index + 1) % 2000 == 0:
            print(f"[Map] {index + 1}/{len(observations)} スキャンを処理しました")

    grid = np.full((height, width), UNKNOWN, dtype=np.int8)
    grid[log_odds <= LOG_ODDS_FREE] = FREE
    grid[log_odds >= LOG_ODDS_OCCUPIED] = OCCUPIED

    # 孤立した障害物セルを消す。
    #
    # スキャンのばらつきで単発の当たりが記録されると、開けた空間に
    # 点々と「障害物」が散らばる。Nav2 はこれを本物の障害物として扱うため、
    # どこにいても数センチ先が塞がっていることになり経路を作れなくなる
    # （実測: 開けた場所でも障害物までの距離が 0.05〜0.36 m しかなかった）。
    # 実在の壁は連続しているので、近傍に仲間が少ないセルだけを消せばよい。
    occupied_mask = grid == OCCUPIED
    # 8 近傍の障害物数を数える（自分は含めない）
    padded = np.pad(occupied_mask, 1, mode="constant", constant_values=False)
    neighbor_count = np.zeros(occupied_mask.shape, dtype=np.int16)
    for dy in (0, 1, 2):
        for dx in (0, 1, 2):
            if dy == 1 and dx == 1:
                continue
            neighbor_count += padded[
                dy:dy + occupied_mask.shape[0], dx:dx + occupied_mask.shape[1]
            ]
    isolated = occupied_mask & (neighbor_count < MIN_OBSTACLE_NEIGHBORS)
    removed = int(isolated.sum())
    # 消した跡は「空き」にする（周囲が空きなら通行できるはずのため）
    grid[isolated] = FREE
    print(
        f"[Map] 孤立した障害物セルを {removed} 個除去しました"
        f"（残り {int((grid == OCCUPIED).sum())} 個）"
    )

    return grid, lo_x, lo_y


def save_map(grid: np.ndarray, origin_x: float, origin_y: float, stem: str) -> None:
    """Nav2 が読める .pgm / .yaml として書き出す。"""
    height, width = grid.shape
    image = np.full((height, width), PGM_UNKNOWN, dtype=np.uint8)
    image[grid == FREE] = PGM_FREE
    image[grid == OCCUPIED] = PGM_OCCUPIED
    # .pgm は左上が原点なので Y を反転する
    image = np.flipud(image)

    pgm_path = f"{stem}.pgm"
    with open(pgm_path, "wb") as fh:
        fh.write(f"P5\n{width} {height}\n255\n".encode())
        fh.write(image.tobytes())

    yaml_path = f"{stem}.yaml"
    basename = pgm_path.rsplit("/", 1)[-1]
    with open(yaml_path, "w") as fh:
        fh.write(
            f"image: {basename}\n"
            f"mode: trinary\n"
            f"resolution: {RESOLUTION}\n"
            f"origin: [{origin_x:.4f}, {origin_y:.4f}, 0.0]\n"
            f"negate: 0\n"
            f"occupied_thresh: 0.65\n"
            f"free_thresh: 0.196\n"
        )

    occupied = int((grid == OCCUPIED).sum())
    free = int((grid == FREE).sum())
    unknown = int((grid == UNKNOWN).sum())
    total = grid.size
    print(f"[Map] 障害物: {occupied} セル ({occupied / total:.1%})")
    print(f"[Map] 空き:   {free} セル ({free / total:.1%})")
    print(f"[Map] 未知:   {unknown} セル ({unknown / total:.1%})")
    print(f"[Map] 探索済み面積: {(occupied + free) * RESOLUTION ** 2:.1f} m2")
    print(f"[OK] 地図を保存しました: {pgm_path} / {yaml_path}")


def main() -> None:
    """スキャンを集めて地図を作る。"""
    parser = argparse.ArgumentParser(description="真値 odom から 2D 地図を作る")
    parser.add_argument(
        "--output",
        default="/home/spacedata/isaac_dev/G1/SimEnvTest/maps/warehouse",
        help="出力先（拡張子なし）",
    )
    parser.add_argument(
        "--duration", type=float, default=120.0, help="収集する秒数"
    )
    parser.add_argument(
        "--cache",
        default="",
        help=(
            "スキャンの保存先 .npz。指定すると収集結果を保存し、"
            "次回から Isaac Sim 無しで再構築できる（格子パラメータの調整用）"
        ),
    )
    parser.add_argument(
        "--from-cache",
        default="",
        help="保存済みの .npz から地図を作る（Isaac Sim は不要）",
    )
    args = parser.parse_args()

    # キャッシュから作る場合は ROS を使わない
    if args.from_cache:
        print(f"[Map] キャッシュから読み込みます: {args.from_cache}")
        data = np.load(args.from_cache, allow_pickle=True)
        observations = [
            (points, tuple(origin))
            for points, origin in zip(data["points"], data["origins"])
        ]
        print(f"[Map] {len(observations)} スキャンを読み込みました")
        grid, ox, oy = build_grid(observations)
        save_map(grid, ox, oy, args.output)
        return

    rclpy.init()
    node = MapBuilder(args.duration)

    print(f"[Map] 最大 {args.duration:.0f} 秒間スキャンを集めます...")
    import time

    start = time.time()
    last_report = start
    last_count = 0
    idle_since = start
    while time.time() - start < args.duration:
        rclpy.spin_once(node, timeout_sec=0.1)
        now = time.time()
        count = len(node.observations)

        # スキャンが途絶えたら（Isaac Sim が終了したら）収集を打ち切る。
        # 待ち続けても増えないため、そこまでの分で地図を作る。
        if count > last_count:
            last_count = count
            idle_since = now
        elif count > 0 and now - idle_since > IDLE_TIMEOUT_SEC:
            print(
                f"[Map] {IDLE_TIMEOUT_SEC:.0f} 秒間スキャンが来ないため収集を終了します"
                "（Isaac Sim が終了したと判断）"
            )
            break

        if now - last_report >= 20.0:
            print(f"[Map] {now - start:.0f} 秒経過、{count} スキャン取得")
            last_report = now

    print(f"[Map] 合計 {len(node.observations)} スキャンを取得しました")
    if len(node.observations) < 5:
        print("[NG] スキャンが足りません。Isaac Sim が動いているか確認してください")
        sys.exit(1)

    if args.cache:
        # あとで Isaac Sim 無しに再構築できるよう保存する。
        # 格子のパラメータ調整のたびにシムを回すのは時間の無駄なため。
        np.savez_compressed(
            args.cache,
            points=np.array([p for p, _ in node.observations], dtype=object),
            origins=np.array([o for _, o in node.observations]),
        )
        print(f"[Map] スキャンを保存しました: {args.cache}")

    print("[Map] 占有格子を構築しています...")
    grid, ox, oy = build_grid(node.observations)
    save_map(grid, ox, oy, args.output)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
