"""天井を落とした PCD を書き出す。

屋内の地図は、上や斜めから見ると不透明な天井板が全部を覆い隠す。表示のたびに
切るのではなく、切った状態の PCD を成果物として残しておくと、ビューアでも
ナビの経路計画でもそのまま使える。

床と天井の検出は `view_pcd_gui.py` の実装をそのまま使う（Zヒストグラムの
下半分・上半分それぞれの最頻ビン）。2026-09-03 の UiS_room_v1 では
床 Z=-1.30 / 天井 Z=+1.49（高さ 2.79 m）を検出した。

    python cut_ceiling_pcd.py in.pcd out.pcd
    python cut_ceiling_pcd.py in.pcd out.pcd --margin 0.5   # 天井の 0.5m 下で切る
    python cut_ceiling_pcd.py in.pcd out.pcd --floor-margin 0.05   # 床も少し落とす
"""
from __future__ import annotations

import argparse
import importlib.util
import sys
from pathlib import Path

import numpy as np
import open3d as o3d

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "python"))
from g1_mapping.rebuild import write_pcd  # noqa: E402

# view_pcd_gui.py の床・天井検出をそのまま使う（実装を二重に持たない）
_spec = importlib.util.spec_from_file_location("view_pcd_gui", HERE / "view_pcd_gui.py")
_viewer = importlib.util.module_from_spec(_spec)
_spec.loader.exec_module(_viewer)


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(description="天井を落とした PCD を書き出す")
    parser.add_argument("source", type=Path, help="入力 .pcd")
    parser.add_argument("output", type=Path, help="出力 .pcd")
    parser.add_argument("--margin", type=float, default=0.35,
                        help="検出した天井から何m下で切るか（既定 0.35）")
    parser.add_argument("--floor-margin", type=float, default=None,
                        help="床から何m下までを残すか。指定すると床下のノイズも落とす")
    return parser.parse_args()


def main() -> None:
    args = parse_args()
    points = np.asarray(o3d.io.read_point_cloud(str(args.source)).points)
    if points.size == 0:
        raise SystemExit(f"点が読めません: {args.source}")

    floor_z, ceiling_z = _viewer.find_floor_ceiling(points[:, 2])
    keep = points[:, 2] < ceiling_z - args.margin
    if args.floor_margin is not None:
        keep &= points[:, 2] > floor_z - args.floor_margin
    kept = points[keep]
    if len(kept) == 0:
        raise SystemExit("全点が消えました。--margin を小さくしてください")

    print(f"[cut] 床 Z={floor_z:+.2f} m / 天井 Z={ceiling_z:+.2f} m"
          f"（高さ {ceiling_z - floor_z:.2f} m）を検出")
    print(f"[cut] 天井の {args.margin:.2f} m 下（Z={ceiling_z - args.margin:+.2f} m）で切りました")
    print(f"[cut] {len(points)} 点 -> {len(kept)} 点"
          f"（{len(points) - len(kept)} 点を除去 / {100 * (1 - len(kept) / len(points)):.1f}%）")

    write_pcd(args.output, [tuple(map(float, p)) for p in kept])
    low, high = kept.min(axis=0), kept.max(axis=0)
    print(f"[OK] {args.output}")
    print(f"[INFO] 範囲 x={high[0]-low[0]:.2f}m y={high[1]-low[1]:.2f}m z={high[2]-low[2]:.2f}m")


if __name__ == "__main__":
    main()
