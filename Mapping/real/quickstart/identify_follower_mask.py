#!/usr/bin/env python3
"""追従者ありとなしの静止記録を突き合わせ、センサ座標系でのマスク値を実測で決める。

`2026-09-04-g1-mapping-drift-and-follower.md` 第8.2節「段2: センサ座標系で
追従者と自己遮蔽をマスク」の入力を作る。

**なぜ差分でやるのか。** 追従者ありの記録だけを見ても、後方 1.3m の点が
追従者なのか壁なのか棚なのかを区別できない。同じ場所で追従者だけを入れ替えた
2つの記録を取れば、**差分がそのまま追従者**になる。推定ではなく実測になる。

**なぜセンサ座標系でやるのか。** 追従者はセンサから見てほぼ定位置なので、
姿勢推定を一切必要とせずに落とせる。「正しい姿勢を得るには追従者を消す必要が
あり、追従者を消すには姿勢が要る」という循環に陥らない。

    ./quickstart/identify_follower_mask.py \\
        runs/<追従者なしのsession> runs/<追従者ありのsession>
"""
from __future__ import annotations

import argparse
import sqlite3
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))

import numpy as np  # noqa: E402

from g1_mapping.rebuild import parse_pointcloud2  # noqa: E402

RAW_POINTS_TOPIC = "/utlidar/cloud_livox_mid360"
VOXEL = 0.05          # m。追従者の胴体（幅 0.4m 程度）が数十ボクセルに載る粗さ
MIN_RATE = 0.30       # そのボクセルが「常にある」とみなす出現率（スキャン比）
MAX_RATE_ABSENT = 0.05  # 「無い」とみなす出現率。センサのちらつきを吸収する


def load_scans(session: Path, max_scans: int) -> tuple[np.ndarray, np.ndarray, int]:
    """センサ座標系の点を積む。戻りは (点 Nx3, 各点のスキャン番号, スキャン数)。"""
    bags = sorted(session.glob("raw/rosbag2/*.db3"))
    if not bags:
        raise SystemExit(f"db3 が見つかりません: {session}")
    with sqlite3.connect(f"file:{bags[0]}?mode=ro", uri=True) as connection:
        row = connection.execute(
            "SELECT id FROM topics WHERE name=?", (RAW_POINTS_TOPIC,)).fetchone()
        if row is None:
            raise SystemExit(f"{RAW_POINTS_TOPIC} が {session.name} にありません")
        blobs = connection.execute(
            "SELECT data FROM messages WHERE topic_id=? ORDER BY timestamp",
            (row[0],)).fetchall()

    step = max(1, len(blobs) // max_scans)
    selected = blobs[::step]
    chunks, index = [], []
    for scan_id, (blob,) in enumerate(selected):
        payload = bytes(blob)
        layout = parse_pointcloud2(payload)
        raw = np.frombuffer(payload, dtype=np.uint8,
                            count=layout.data_length, offset=layout.data_start)
        block = raw.reshape(-1, layout.point_step)
        xyz = np.stack([
            block[:, o:o + 4].copy().view(np.float32).ravel()
            for o in (layout.x_offset, layout.y_offset, layout.z_offset)
        ], axis=1).astype(np.float64)
        keep = np.isfinite(xyz).all(axis=1) & (np.linalg.norm(xyz, axis=1) > 1e-3)
        xyz = xyz[keep]
        chunks.append(xyz)
        index.append(np.full(len(xyz), scan_id))
    return np.concatenate(chunks), np.concatenate(index), len(selected)


def occupancy(points: np.ndarray, scan_index: np.ndarray, scans: int) -> dict:
    """ボクセルごとの「何割のスキャンで見えたか」を返す。

    点の総数ではなく**出現率**にするのは、たまたま1スキャンだけ大量に当たった
    面と、常にそこに在る物体を区別するため。追従者は常に在る。
    """
    keys = np.floor(points / VOXEL).astype(np.int32)
    packed = np.unique(np.column_stack([keys, scan_index]), axis=0)
    voxels, counts = np.unique(packed[:, :3], axis=0, return_counts=True)
    return {"voxels": voxels, "rate": counts / scans}


def cluster(voxels: np.ndarray) -> list[np.ndarray]:
    """隣接ボクセルを繋いで塊に分ける。大きい順に返す。

    差分には追従者以外も混ざる。2026-09-04 の実測では、追従者の他に
    後方左 2.8m にもう1つの塊があった（2つの記録の間に動いた別の物体）。
    塊に分けずに外接範囲を取ると、無関係な物体まで囲うマスクになる。
    """
    from scipy import ndimage

    origin = voxels.min(axis=0)
    shape = (voxels.max(axis=0) - origin + 1).astype(int)
    grid = np.zeros(shape, dtype=bool)
    local = (voxels - origin).astype(int)
    grid[local[:, 0], local[:, 1], local[:, 2]] = True
    # 26近傍で繋ぐ。5cm ボクセルなので、人体は必ず1つに繋がる
    labels, count = ndimage.label(grid, structure=np.ones((3, 3, 3), dtype=int))
    tags = labels[local[:, 0], local[:, 1], local[:, 2]]
    groups = [voxels[tags == tag] for tag in range(1, count + 1)]
    return sorted(groups, key=len, reverse=True)


def describe(points: np.ndarray, label: str, scans: int) -> None:
    rng = np.linalg.norm(points, axis=1)
    azimuth = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
    elevation = np.degrees(np.arcsin(np.clip(points[:, 2] / rng, -1, 1)))
    print(f"  {label}")
    print(f"    点数        : {len(points):,}  （1スキャンあたり {len(points) / scans:.0f} 点）")
    print(f"    距離        : 最小 {rng.min():.2f} / p5 {np.percentile(rng, 5):.2f}"
          f" / 中央 {np.median(rng):.2f} / p95 {np.percentile(rng, 95):.2f}"
          f" / 最大 {rng.max():.2f} m")
    # 方位は ±180° で折り返すので、円平均で代表値を出す
    radians = np.radians(azimuth)
    mean_az = np.degrees(np.arctan2(np.sin(radians).mean(), np.cos(radians).mean()))
    spread = np.degrees(np.sqrt(-2 * np.log(np.hypot(
        np.sin(radians).mean(), np.cos(radians).mean()))))
    print(f"    方位        : 円平均 {mean_az:+.1f}° / 円標準偏差 {spread:.1f}°"
          "   (0°=前方 / +90°=左 / ±180°=後方)")
    print(f"    仰角        : p5 {np.percentile(elevation, 5):+.1f}°"
          f" / 中央 {np.median(elevation):+.1f}° / p95 {np.percentile(elevation, 95):+.1f}°")
    print(f"    センサ基準 z: p5 {np.percentile(points[:, 2], 5):+.2f}"
          f" / 中央 {np.median(points[:, 2]):+.2f}"
          f" / p95 {np.percentile(points[:, 2], 95):+.2f} m")


def _japanese_font() -> None:
    """図に日本語を出せるフォントを選ぶ。無ければ黙って既定のままにする。"""
    import matplotlib
    from matplotlib import font_manager

    available = {f.name for f in font_manager.fontManager.ttflist}
    for candidate in ("Hiragino Sans", "Hiragino Kaku Gothic ProN",
                      "Noto Sans CJK JP", "IPAexGothic", "Arial Unicode MS"):
        if candidate in available:
            matplotlib.rcParams["font.family"] = candidate
            return


def draw(base_points: np.ndarray, test_points: np.ndarray, follower: np.ndarray,
         others: "list[np.ndarray]", out: Path) -> None:
    """差分が本当に人の形をしているかを目で確かめられる図を出す。

    数値だけでは「後方 1m の何か」が人なのか台車なのか分からない。
    上面図と背面図を並べれば、胴体の幅と高さの形で判断できる。
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    _japanese_font()
    rng = np.random.default_rng(0)

    def thin(points: np.ndarray, limit: int = 120000) -> np.ndarray:
        if len(points) <= limit:
            return points
        return points[rng.choice(len(points), limit, replace=False)]

    figure, axes = plt.subplots(1, 3, figsize=(18, 6))
    for axis, points, title in (
            (axes[0], base_points, "追従者なし（上面図）"),
            (axes[1], test_points, "追従者あり（上面図・赤=差分）")):
        sample = thin(points)
        axis.scatter(sample[:, 0], sample[:, 1], s=0.3, c="#9aa5b1", lw=0)
    for chunk in others:
        sample = thin(chunk, 40000)
        axes[1].scatter(sample[:, 0], sample[:, 1], s=0.6, c="#f5a623", lw=0,
                        label="_")
    sample = thin(follower)
    axes[1].scatter(sample[:, 0], sample[:, 1], s=0.6, c="#e5484d", lw=0)
    axes[0].set_title("追従者なし（上面図）")
    axes[1].set_title("追従者あり（赤=追従者 / 橙=別物体）")
    for axis in axes[:2]:
        axis.set_xlim(-5, 5)
        axis.set_ylim(-5, 5)
        axis.set_aspect("equal")
        axis.grid(alpha=0.25)
        axis.axhline(0, c="k", lw=0.5)
        axis.axvline(0, c="k", lw=0.5)
        axis.plot(0, 0, marker="*", ms=16, c="#0090ff")
        axis.set_xlabel("x 前方 [m]")
        axis.set_ylabel("y 左 [m]")
        for radius in (1, 2, 3):
            axis.add_patch(plt.Circle((0, 0), radius, fill=False, ls=":",
                                      ec="#555", lw=0.7))

    sample = thin(follower)
    axes[2].scatter(-sample[:, 1], sample[:, 2], s=0.6, c="#e5484d", lw=0)
    axes[2].set_title("追従者の背面図（後ろから見る）")
    axes[2].set_xlabel("横 [m]（+が右）")
    axes[2].set_ylabel("センサ基準 z [m]")
    axes[2].set_aspect("equal")
    axes[2].grid(alpha=0.25)
    axes[2].set_xlim(-2, 2)
    axes[2].set_ylim(-0.6, 1.6)

    figure.tight_layout()
    out.parent.mkdir(parents=True, exist_ok=True)
    figure.savefig(out, dpi=110)
    print(f"\n図を書きました: {out}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("without_follower", type=Path, help="追従者なしの runs/<id>")
    parser.add_argument("with_follower", type=Path, help="追従者ありの runs/<id>")
    parser.add_argument("--max-scans", type=int, default=250)
    parser.add_argument("--figure", type=Path, default=None,
                        help="図の出力先。既定は <追従者ありのrun>/report/follower_mask.png")
    args = parser.parse_args()

    print("=" * 72)
    print("追従者マスクの同定（センサ座標系 livox_frame・差分法）")
    print("=" * 72)
    base_points, base_index, base_scans = load_scans(args.without_follower, args.max_scans)
    test_points, test_index, test_scans = load_scans(args.with_follower, args.max_scans)
    print(f"追従者なし: {args.without_follower.name}  {base_scans} スキャン / {len(base_points):,} 点")
    print(f"追従者あり: {args.with_follower.name}  {test_scans} スキャン / {len(test_points):,} 点")
    print(f"ボクセル  : {VOXEL * 100:.0f} cm")

    base = occupancy(base_points, base_index, base_scans)
    test = occupancy(test_points, test_index, test_scans)

    # 「追従者ありでは常にあるが、なしではほぼ無い」ボクセルを拾う
    base_key = {tuple(v): r for v, r in zip(map(tuple, base["voxels"]), base["rate"])}
    stable = test["rate"] >= MIN_RATE
    candidates = test["voxels"][stable]
    candidate_rate = test["rate"][stable]
    is_new = np.array([base_key.get(tuple(v), 0.0) <= MAX_RATE_ABSENT for v in candidates])
    new_voxels = candidates[is_new]
    print()
    print(f"追従者ありで安定して見えるボクセル : {stable.sum():,}"
          f"（出現率 {MIN_RATE:.0%} 以上）")
    print(f"うち追従者なしには無いもの         : {len(new_voxels):,}"
          f"（なし側の出現率 {MAX_RATE_ABSENT:.0%} 以下）")
    if len(new_voxels) == 0:
        print("\n⚠️ 差分が出ませんでした。2つの記録の条件が同じか確認してください")
        return 1

    # --- 塊に分けて、追従者とそれ以外を切り離す --------------------------
    groups = cluster(new_voxels)
    test_keys = np.floor(test_points / VOXEL).astype(np.int32)

    def points_in(voxels: np.ndarray) -> np.ndarray:
        wanted = set(map(tuple, voxels))
        hit = np.fromiter((tuple(k) in wanted for k in test_keys), bool, len(test_keys))
        return test_points[hit]

    print()
    print("-" * 72)
    print(f"差分を塊に分ける（隣接ボクセルを 26 近傍で連結）… {len(groups)} 個")
    print("-" * 72)
    print(f"{'#':>3} {'ボクセル':>9} {'点数':>10} {'距離中央':>9} {'方位':>8} {'高さ幅':>14}")
    ranked = []
    for index, voxels in enumerate(groups[:8]):
        chunk = points_in(voxels)
        if len(chunk) == 0:
            continue
        distance = np.linalg.norm(chunk, axis=1)
        radians = np.arctan2(chunk[:, 1], chunk[:, 0])
        mean_az = np.degrees(np.arctan2(np.sin(radians).mean(), np.cos(radians).mean()))
        print(f"{index:>3} {len(voxels):>9,} {len(chunk):>10,} {np.median(distance):>8.2f}m"
              f" {mean_az:>+7.1f}° {chunk[:, 2].min():>+6.2f}〜{chunk[:, 2].max():>+6.2f}m")
        ranked.append((voxels, chunk))

    # 追従者は「後方にあり、最も点数が多い塊」。方位で先に絞ってから大きさで選ぶ。
    def is_behind(chunk: np.ndarray) -> bool:
        radians = np.arctan2(chunk[:, 1], chunk[:, 0])
        mean_az = np.degrees(np.arctan2(np.sin(radians).mean(), np.cos(radians).mean()))
        return abs(abs(mean_az) - 180.0) <= 90.0

    behind = [(v, c) for v, c in ranked if is_behind(c)]
    if not behind:
        print("\n⚠️ 後方に塊がありません。追従者が写っていない可能性があります")
        return 1
    follower_voxels, follower = max(behind, key=lambda pair: len(pair[1]))
    others = [c for v, c in ranked if v is not follower_voxels]
    print()
    print(f"→ 追従者とみなす塊: {len(follower):,} 点"
          f"（差分全体の {len(follower) / sum(len(c) for _, c in ranked) * 100:.0f}%）")
    if others:
        stray = sum(len(c) for c in others)
        print(f"   残り {len(others)} 個 / {stray:,} 点は追従者以外として除外した")
        print("   （2つの記録の間に動いた別の物体。マスクに含めてはいけない）")

    print()
    print("-" * 72)
    print("追従者の塊")
    print("-" * 72)
    describe(follower, "追従者", test_scans)

    # --- マスク条件を決める ---------------------------------------------
    rng = np.linalg.norm(follower, axis=1)
    azimuth = np.degrees(np.arctan2(follower[:, 1], follower[:, 0]))
    # 後方が ±180° で折り返すので、後方中心（180°）からの角度差に直す
    back_offset = np.abs((azimuth - 180.0 + 180.0) % 360.0 - 180.0)
    lo_r, hi_r = np.percentile(rng, [1, 99])
    hi_az = np.percentile(back_offset, 99)
    lo_z, hi_z = np.percentile(follower[:, 2], [1, 99])

    print()
    print("-" * 72)
    print("マスク条件（追従者の 98% を含む範囲。センサ座標系）")
    print("-" * 72)
    print(f"  距離        : {lo_r:.2f} 〜 {hi_r:.2f} m")
    print(f"  方位        : 真後ろ ±{hi_az:.0f}°")
    print(f"  センサ基準 z: {lo_z:+.2f} 〜 {hi_z:+.2f} m")

    # --- 巻き添えを測る -------------------------------------------------
    # 同じ条件を「追従者なし」の点に当てると、構造物を何点落とすかが分かる。
    def apply(points: np.ndarray) -> np.ndarray:
        r = np.linalg.norm(points, axis=1)
        az = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
        back = np.abs((az - 180.0 + 180.0) % 360.0 - 180.0)
        return ((r >= lo_r) & (r <= hi_r) & (back <= hi_az)
                & (points[:, 2] >= lo_z) & (points[:, 2] <= hi_z))

    hit_base = apply(base_points)
    hit_test = apply(test_points)
    print()
    print("この条件を当てたときの除去量")
    print(f"  追従者あり : {hit_test.sum():,} / {len(test_points):,} 点"
          f"（{hit_test.mean() * 100:.2f}%）")
    print(f"  追従者なし : {hit_base.sum():,} / {len(base_points):,} 点"
          f"（{hit_base.mean() * 100:.2f}%）← **巻き添え**（構造物の損失）")
    print(f"  追従者の捕捉: {apply(follower).mean() * 100:.1f}%")

    # --- 歩行時に向けて広げる -------------------------------------------
    # 上の値は**静止した人**の位置。歩行中は人が揺れるので、そのままでは狭い。
    # 2026-09-03 の歩行記録の実測ばらつき（距離 σ=0.27m / 方位の円標準偏差 37°）を
    # 足して、静止で決めた中心の周りに余裕を持たせる。
    walk_range_sigma, walk_azimuth_sigma = 0.27, 37.0
    centre_r = float(np.median(rng))
    wide_r0 = max(0.3, centre_r - 2 * walk_range_sigma)
    wide_r1 = centre_r + 2 * walk_range_sigma
    wide_az = min(90.0, hi_az + 1.5 * walk_azimuth_sigma)
    wide_z0, wide_z1 = lo_z - 0.10, hi_z + 0.10
    print()
    print("-" * 72)
    print("歩行時に使う値（静止で決めた中心に、2026-09-03 の歩行時ばらつきを足す）")
    print("-" * 72)
    print(f"  距離        : {wide_r0:.2f} 〜 {wide_r1:.2f} m"
          f"   （中心 {centre_r:.2f}m ± 2σ、σ=歩行時 {walk_range_sigma}m）")
    print(f"  方位        : 真後ろ ±{wide_az:.0f}°"
          f"   （静止 ±{hi_az:.0f}° + 1.5×円標準偏差 {walk_azimuth_sigma}°）")
    print(f"  センサ基準 z: {wide_z0:+.2f} 〜 {wide_z1:+.2f} m")

    def apply_wide(points: np.ndarray) -> np.ndarray:
        r = np.linalg.norm(points, axis=1)
        az = np.degrees(np.arctan2(points[:, 1], points[:, 0]))
        back = np.abs((az - 180.0 + 180.0) % 360.0 - 180.0)
        return ((r >= wide_r0) & (r <= wide_r1) & (back <= wide_az)
                & (points[:, 2] >= wide_z0) & (points[:, 2] <= wide_z1))

    print(f"  追従者の捕捉: {apply_wide(follower).mean() * 100:.1f}%")
    print(f"  巻き添え    : {apply_wide(base_points).mean() * 100:.2f}%"
          "   ← この記録の場所での値")
    print("  ⚠️ 巻き添えは**場所で決まる**。ここは吊り下げ場所で近傍 2m に構造物が"
          "密集しており、")
    print("     実際の建図対象（25.7×32.1m の部屋）ではこれよりずっと小さくなる。")

    # --- blind パラメータとの比較 ---------------------------------------
    print()
    print("-" * 72)
    print("FAST-LIO2 の `blind`（センサから一定距離内を全方位で捨てる）との比較")
    print("-" * 72)
    print(f"{'blind':>7} {'追従者の除去':>13} {'構造物の巻き添え':>17}")
    for blind in (0.5, 1.0, 1.5, 2.0, 2.2, 2.5, 3.0):
        base_rng = np.linalg.norm(base_points, axis=1)
        follow_rng = np.linalg.norm(follower, axis=1)
        print(f"{blind:>6.1f}m {(follow_rng < blind).mean() * 100:12.1f}%"
              f" {(base_rng < blind).mean() * 100:16.1f}%")
    print()
    print("※ 巻き添えは「追従者なしの記録で失われる点の割合」。"
          "この環境は近傍に構造物が多く、")
    print("   blind を上げると追従者より先に周囲の幾何を失う。"
          "後方セクタ限定のマスクの方が損失が小さい。")

    figure_path = args.figure or (args.with_follower / "report" / "follower_mask.png")
    draw(base_points, test_points, follower, others, figure_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
