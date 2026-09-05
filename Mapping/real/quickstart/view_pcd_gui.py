"""map_raw.pcd を Mac のネイティブウィンドウで回して見る。

RViz2 の代わり。Navigation/.venv の open3d をそのまま使うので追加導入は要らない。
高さで色を付け、原点に座標軸を置く（赤=X 緑=Y 青=Z）。

  左ドラッグ: 回転   右ドラッグ/二本指: 平行移動   スクロール: ズーム
  R: 視点リセット    Q または ESC: 閉じる

視点を明示しないと、部屋のような扁平な点群では真横から見た状態で開くことがあり
「板が一枚あるだけ」に見える。既定を上面図にしてあるのはこのため。

外れ値のトリムも既定で入れてある。窓やドア越しに遠くを拾った点が bounding box を
数倍に引き伸ばすため、Open3D の自動フィットだと肝心の部屋が小さく端へ寄る。
2026-09-03 の UiS_room_v1 は全体 60×45×12m に対し、点の 94% が 26×32×2.9m に入っていた。

天井も既定で落とす。屋内の地図を上や斜めから見ると、不透明な天井板が全部を
覆い隠して「オレンジ一色の板」しか見えない。床と天井は Z のヒストグラムの
ピークとして明確に出るので、自動検出して天井側を切る。

  python view_pcd_gui.py map_raw.pcd                    # 斜め見下ろし・天井カット
  python view_pcd_gui.py map_raw.pcd --view top         # 間取り図として見る
  python view_pcd_gui.py map_raw.pcd --ceiling keep     # 天井も残す
  python view_pcd_gui.py map_raw.pcd --trim 0           # 外れ値も捨てない
"""
import argparse
import numpy as np
import open3d as o3d

# 視点名 -> (front: 注視点からカメラへ向かうベクトル, up: 画面の上方向)
VIEWS: dict[str, tuple[tuple[float, float, float], tuple[float, float, float]]] = {
    "top": ((0.0, 0.0, 1.0), (0.0, 1.0, 0.0)),      # 真上から。間取りが見える
    "iso": ((0.6, -0.8, 0.55), (0.0, 0.0, 1.0)),    # 斜め見下ろし
    "side": ((1.0, 0.0, 0.0), (0.0, 0.0, 1.0)),     # 真横から。床の水平性を見る
}


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="PCD を高さ色付きで表示する")
    parser.add_argument("path", help="表示する .pcd")
    parser.add_argument("--view", choices=sorted(VIEWS), default="iso",
                        help="初期視点（既定: iso）")
    parser.add_argument("--trim", type=float, default=1.0,
                        help="各軸で上下から捨てるパーセンタイル。0で全点（既定: 1.0）")
    parser.add_argument("--ceiling", choices=("cut", "keep"), default="cut",
                        help="天井を落とすか（既定: cut。屋内を上/斜めから見るのに必要）")
    parser.add_argument("--zoom", type=float, default=0.62,
                        help="小さいほど寄る（既定: 0.62）")
    return parser.parse_args()


def trim_outliers(points: np.ndarray, percentile: float) -> np.ndarray:
    """各軸の p..100-p の箱に入る点だけ残す。表示のためだけの間引き。"""
    if percentile <= 0.0:
        return points
    keep = np.ones(len(points), dtype=bool)
    for axis in range(3):
        low, high = np.percentile(points[:, axis], [percentile, 100.0 - percentile])
        keep &= (points[:, axis] >= low) & (points[:, axis] <= high)
    return points[keep]


def find_floor_ceiling(z: np.ndarray) -> tuple[float, float]:
    """Z ヒストグラムの下半分・上半分それぞれの最頻ビンを床・天井とみなす。

    ヒストグラムの範囲は min/max ではなく p1〜p99 で取る。窓の外を拾った点などで
    Z の全範囲が実際の部屋の数倍に広がると、上下を分ける中点が床より下へ落ちて
    「床下のノイズを床、本当の床を天井」と誤検出する。2026-09-04 に実際に踏んだ
    （全範囲 -8.66〜+3.47 に対し、床 -1.30 を天井と誤判定した）。
    """
    low, high = np.percentile(z, [1.0, 99.0])
    core = z[(z >= low) & (z <= high)]
    if len(core) < 100:
        core = z
    hist, edges = np.histogram(core, bins=80)
    centers = (edges[:-1] + edges[1:]) / 2.0
    middle = (centers[0] + centers[-1]) / 2.0
    lower, upper = centers < middle, centers >= middle
    floor = float(centers[lower][np.argmax(hist[lower])])
    ceiling = float(centers[upper][np.argmax(hist[upper])])
    return floor, ceiling


def cut_ceiling(points: np.ndarray, margin: float = 0.35) -> tuple[np.ndarray, float, float]:
    """検出した天井より margin だけ下で切る。戻り値は (残った点, 床Z, 天井Z)。"""
    floor, ceiling = find_floor_ceiling(points[:, 2])
    return points[points[:, 2] < ceiling - margin], floor, ceiling


def height_colors(z: np.ndarray) -> np.ndarray:
    """高さを 2〜98 パーセンタイルで正規化して色にする。外れ値に色域を食わせない。"""
    low, high = np.percentile(z, [2, 98])
    t = np.clip((z - low) / max(high - low, 1e-9), 0.0, 1.0)
    return np.column_stack([t, 0.45 + 0.35 * np.sin(np.pi * t), 1.0 - t])


def main() -> None:
    args = parse_args()
    raw = np.asarray(o3d.io.read_point_cloud(args.path).points)
    if raw.size == 0:
        raise SystemExit(f"点が読めません: {args.path}")

    points = trim_outliers(raw, args.trim)
    if len(points) == 0:
        raise SystemExit("トリムで全点消えました。--trim 0 を試してください")
    trimmed = len(raw) - len(points)

    if args.ceiling == "cut":
        points, floor_z, ceiling_z = cut_ceiling(points)
        print(f"[view] 床 Z={floor_z:+.2f}m / 天井 Z={ceiling_z:+.2f}m"
              f"（高さ {ceiling_z - floor_z:.2f}m）を検出し、天井を落としました")
        if len(points) == 0:
            raise SystemExit("天井カットで全点消えました。--ceiling keep を試してください")

    extent = points.max(axis=0) - points.min(axis=0)
    print(f"[view] {len(raw)} 点 -> 表示 {len(points)} 点"
          f"（trim={args.trim}% で {trimmed} 点を除外）")
    print(f"[view] 表示範囲 x={extent[0]:.2f}m y={extent[1]:.2f}m z={extent[2]:.2f}m")
    print(f"[view] 視点={args.view}  R で戻せます。Q または ESC で閉じます")

    pcd = o3d.geometry.PointCloud()
    pcd.points = o3d.utility.Vector3dVector(points)
    pcd.colors = o3d.utility.Vector3dVector(height_colors(points[:, 2]))
    axis = o3d.geometry.TriangleMesh.create_coordinate_frame(
        size=float(max(extent.max() * 0.08, 0.3)))

    visualizer = o3d.visualization.Visualizer()
    visualizer.create_window(window_name=f"G1 LiDAR — {args.path.split('/')[-1]}",
                             width=1400, height=900)
    visualizer.add_geometry(pcd)
    visualizer.add_geometry(axis)

    front, up = VIEWS[args.view]
    control = visualizer.get_view_control()
    control.set_lookat(points.mean(axis=0))
    control.set_front(list(front))
    control.set_up(list(up))
    control.set_zoom(args.zoom)

    render = visualizer.get_render_option()
    render.background_color = np.asarray([0.06, 0.07, 0.09])
    render.point_size = 1.5

    visualizer.run()
    visualizer.destroy_window()


if __name__ == "__main__":
    main()
