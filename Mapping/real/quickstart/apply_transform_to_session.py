#!/usr/bin/env python3
"""記録 1 本ぶんを、別の地図の座標系へまるごと移す。

## 何のためか

新しい記録から作った地図を**過去の地図の座標系の上で**使いたい。座標系を変えないので、
14 点のウェイポイント・AMCL の初期姿勢・Isaac の spawn・AprilTag の登録が
そのまま生き続ける（`docs/plan/2026-09-18-swap-map-to-0911-and-remap-apriltag.md` の案 A）。

## ⚠️ 点群だけ動かしても足りない

`run_octomap.py` は **各 PCD の VIEWPOINT をレイの始点に使う**
（`insertPointCloud(points, origin, ...)`）。点だけ動かすと、
可視性除去が**別の場所からレイを飛ばす**ことになり、掃除が静かに壊れる。

だからここでまとめて掛ける。

| 対象 | 何を掛けるか |
|---|---|
| 姿勢つき PCD の点 | `p' = R p + t` |
| 姿勢つき PCD の VIEWPOINT | 並進 `R t_v + t`、回転 `q_R ⊗ q_v` |
| 軌跡（`poses.txt` / `traj.txt`） | 同じ |
| 単体の点群（`map_raw.pcd` など） | 点だけ（VIEWPOINT を持たない） |

## 掛かったことの確かめ方

**剛体変換は「始点から各点までの距離」を変えない。** これを実際に測って報告する
（`--check` 枚ぶん）。点だけ動かして VIEWPOINT を忘れると、ここが即座に壊れる。
2026-09-17 の「NG が出たらまず計器を疑う」の裏返しで、**通ったことにも証拠を出す**。

## 使い方

    ../../Navigation/.venv/bin/python quickstart/apply_transform_to_session.py \
        runs/20260911T201836_wireless_dualcam_map01 \
        runs/20260911T201836_wireless_dualcam_map01/align/T_old_from_new_band030-180.json \
        --source benchmark_s1 --dest benchmark_s1_aligned \
        --clouds map/map_raw.pcd
"""
from __future__ import annotations

import argparse
import json
import shutil
import sys
import time
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "python"))
from g1_mapping.pcd_io import read_pcd, write_pcd_array  # noqa: E402

# 不変量の確かめに使う枚数。全部やっても構わないが、先頭・中間・末尾で十分
DEFAULT_CHECK = 5
# 剛体性が壊れたとみなす距離の差[m]。float32 で書き戻すので丸めぶんの余裕を取る
RIGID_TOLERANCE_M = 1e-3


def load_transform(path: Path) -> np.ndarray:
    matrix = np.asarray(json.loads(path.read_text())["transform"], dtype=np.float64)
    if matrix.shape != (4, 4):
        raise SystemExit("4x4 ではない: {}".format(matrix.shape))
    rotation = matrix[:3, :3]
    # 剛体であることを先に確かめる。縮尺が入った行列を掛けると地図の寸法が変わる
    scale = float(np.mean(np.linalg.norm(rotation, axis=0)))
    if abs(scale - 1.0) > 1e-4:
        raise SystemExit("剛体変換ではない（縮尺 {:.6f}）。align_to_map.py の出力を渡すこと".format(scale))
    return matrix


def transform_points(points: np.ndarray, matrix: np.ndarray) -> np.ndarray:
    with np.errstate(all="ignore"):     # numpy 2 + Apple silicon の偽の警告
        return points @ matrix[:3, :3].T + matrix[:3, 3]


def transform_viewpoint(viewpoint, matrix: np.ndarray):
    """VIEWPOINT (tx,ty,tz,qw,qx,qy,qz) を移す。**並びは PCL 式で qw が先。**"""
    translation = transform_points(np.asarray(viewpoint[:3], dtype=np.float64)[None, :], matrix)[0]
    qw, qx, qy, qz = viewpoint[3:]
    rotated = Rotation.from_matrix(matrix[:3, :3]) * Rotation.from_quat([qx, qy, qz, qw])
    nx, ny, nz, nw = rotated.as_quat()
    return (float(translation[0]), float(translation[1]), float(translation[2]),
            float(nw), float(nx), float(ny), float(nz))


def rigid_check(before, after, matrix: np.ndarray) -> float:
    """始点から各点までの距離が保たれているか。**返すのは最大の差[m]。**"""
    radius_before = np.linalg.norm(before.points - before.origin, axis=1)
    radius_after = np.linalg.norm(after["points"] - np.asarray(after["viewpoint"][:3]), axis=1)
    return float(np.abs(radius_before - radius_after).max())


def move_trajectory(source: Path, destination: Path, matrix: np.ndarray) -> int:
    """`時刻 tx ty tz qx qy qz qw ...` の行を移す。⚠️ こちらの並びは qw が最後。"""
    lines_out = []
    moved = 0
    for line in source.read_text().splitlines():
        if not line.strip() or line.lstrip().startswith("#"):
            lines_out.append(line)
            continue
        parts = line.split()
        if len(parts) < 8:
            lines_out.append(line)
            continue
        translation = transform_points(
            np.array([[float(parts[1]), float(parts[2]), float(parts[3])]]), matrix)[0]
        rotated = Rotation.from_matrix(matrix[:3, :3]) * Rotation.from_quat(
            [float(parts[4]), float(parts[5]), float(parts[6]), float(parts[7])])
        qx, qy, qz, qw = rotated.as_quat()
        parts[1:8] = ["{:.6f}".format(v) for v in
                      (translation[0], translation[1], translation[2], qx, qy, qz, qw)]
        lines_out.append(" ".join(parts))
        moved += 1
    destination.parent.mkdir(parents=True, exist_ok=True)
    destination.write_text("\n".join(lines_out) + "\n")
    return moved


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session_dir", type=Path)
    p.add_argument("transform", type=Path, help="align_to_map.py --save の JSON")
    p.add_argument("--source", default="benchmark_s1", help="姿勢つき PCD の置き場（記録側）")
    p.add_argument("--dest", default=None,
                   help="出力先（既定は <source>_aligned）。⚠️ 元は残す")
    p.add_argument("--clouds", nargs="*", default=[],
                   help="ついでに移す単体の点群（session_dir からの相対）")
    p.add_argument("--check", type=int, default=DEFAULT_CHECK,
                   help="剛体性を確かめる枚数")
    args = p.parse_args()

    matrix = load_transform(args.transform)
    source = args.session_dir / args.source
    dest = args.session_dir / (args.dest or args.source + "_aligned")
    files = sorted((source / "pcd").glob("*.pcd"))
    if not files:
        raise SystemExit("姿勢つき PCD がありません: {}".format(source / "pcd"))

    print("変換: {} -> {}".format(source.name, dest.name))
    print("  並進[m] {}".format(np.round(matrix[:3, 3], 4)))
    print("  回転[deg] {}".format(
        np.round(Rotation.from_matrix(matrix[:3, :3]).as_euler("xyz", degrees=True), 3)))
    print("  {:,} 枚".format(len(files)))

    (dest / "pcd").mkdir(parents=True, exist_ok=True)
    # 確かめに使う枚数を先頭・中間・末尾から散らして選ぶ。先頭だけ見ると
    # 「最初の 1 枚は原点付近なので誤りが目立たない」型を見逃す
    check_indices = set(np.linspace(0, len(files) - 1, max(args.check, 1)).astype(int).tolist())
    worst_rigid = 0.0
    began = time.time()
    total_points = 0
    for index, path in enumerate(files):
        data = read_pcd(path)
        moved_points = transform_points(data.points, matrix)
        moved_viewpoint = transform_viewpoint(data.viewpoint, matrix)
        write_pcd_array(dest / "pcd" / path.name, moved_points, moved_viewpoint)
        total_points += len(moved_points)
        if index in check_indices:
            error = rigid_check(data, {"points": moved_points, "viewpoint": moved_viewpoint}, matrix)
            worst_rigid = max(worst_rigid, error)
            print("  [確認] {} 始点 {} -> {} / 半径の差 最大 {:.2e} m".format(
                path.name,
                np.round(data.origin, 3), np.round(np.asarray(moved_viewpoint[:3]), 3), error))
        if (index + 1) % 1000 == 0:
            print("  {:,}/{:,} 枚（{:.0f} 秒）".format(index + 1, len(files), time.time() - began))

    print("  {:,} 枚 / {:,} 点 を {:.0f} 秒で移した".format(len(files), total_points, time.time() - began))

    # 軌跡。clean_map.py は <session>/mola_floor0/traj.txt を読む
    for name in ("poses.txt", "traj.txt"):
        if (source / name).exists():
            moved = move_trajectory(source / name, dest / name, matrix)
            print("  {}: {:,} 行を移した".format(name, moved))

    for relative in args.clouds:
        origin_path = args.session_dir / relative
        if not origin_path.exists():
            print("  [飛ばす] 無い: {}".format(origin_path))
            continue
        data = read_pcd(origin_path)
        out = origin_path.with_name(origin_path.stem + "_aligned" + origin_path.suffix)
        write_pcd_array(out, transform_points(data.points, matrix),
                        transform_viewpoint(data.viewpoint, matrix))
        print("  {} -> {}（{:,} 点）".format(origin_path.name, out.name, len(data.points)))

    # 身元を置いていく。**どの地図の座標系に居るか**をファイル側に持たせる
    # （tag_registry.json が「どの地図の map か」を記録していなかった問題の同型）
    (dest / "frame.json").write_text(json.dumps({
        "aligned_to": json.loads(args.transform.read_text())["reference"],
        "transform_json": str(args.transform),
        "transform": matrix.tolist(),
        "source": str(source),
        "files": len(files),
        "rigid_check_worst_m": worst_rigid,
    }, indent=2, ensure_ascii=False) + "\n")

    print()
    if worst_rigid > RIGID_TOLERANCE_M:
        print("[NG] 剛体性が壊れている（半径の差 最大 {:.2e} m > {:.0e}）。"
              "VIEWPOINT の並び（qw が先）を疑うこと".format(worst_rigid, RIGID_TOLERANCE_M))
        return 1
    print("[OK] 剛体性を確認した（半径の差 最大 {:.2e} m）。身元: {}".format(
        worst_rigid, dest / "frame.json"))
    print("次: mola_floor0/traj.txt を置いて clean_map.py --source {}".format(dest.name))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
