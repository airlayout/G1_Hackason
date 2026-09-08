#!/usr/bin/env python3
"""できあがった点群から「短時間しか・近くからしか見えなかった」ボクセルを消す。

## 何のための道具か

`filter_scans_near.py`（近距離除去）と `run_octomap.py`（可視性除去）を通しても、
追従者は **3.2%** 残る（`map_octomap_clean.pcd` の障害物帯 0.23〜1.80 m の点のうち、
「5 m 超から一度も見えていない かつ 観測が 5 秒未満」に当たるもの）。
近距離除去は**点**を落とすので「近くと遠くの両方から見えたボクセル」は残り、
OctoMap は「後から光線が通り抜けた」証拠が無い場所を消せないためである。

ここでは**ボクセル単位**で持続性を見て落とす。姿勢つき全スキャンがあれば
「そのボクセルをいつからいつまで・どこまで遠くから見たか」が取れる
（内蔵 SLAM の点群は増分配信なので取れない。`filter_follower.py` 冒頭の却下理由）。

## ⚠️ Nav2 の 2D 地図はほとんど変わらない

実測（`map_octomap_clean.pcd`、しきい値 最遠 6 m / 時間幅 5 秒）:

| 量 | 前 | 後 |
|---|---|---|
| 点 | 784,237 | 744,247 |
| 占有セル（帯 0.23〜1.80） | 19,464 | 18,211 |
| 軌跡クリアランス 中央値 | 0.781 m | **0.781 m** |
| 0.30 m 未満の軌跡セル | 1.9% | **1.7%** |
| **観測 60 秒以上のセルの残存** | — | **100.0%** |

**構造は 1 セルも失わない**が、経路計画への効果はほぼ無い。残っていた追従者は
軌跡の上ではなく脇にいたので、2D へ潰す時点で既に効かなくなっている。

→ **効くのは 3D の用途**である: Isaac Sim / MuJoCo のシーン化（部屋に幽霊の人が
立っている状態を消す）、RViz の表示、3DGS。**Nav2 のためだけなら要らない。**

## 使い方

    ../../G1_Hackason/.venv/bin/python quickstart/filter_persistence.py \\
        runs/<id> map_octomap_clean.pcd --source benchmark_s5 \\
        --max-range 6.0 --span 5.0 --output map_octomap_clean_p.pcd

⚠️ 入力の点は **`--source` のスキャンから作られたもの**でなければならない。
内蔵 SLAM 由来の点群（`map_raw.pcd` 系）を渡すと、点の位置が違うので
ボクセルが対応せず、判定できた点が数割しかない状態で「掃除できた」ように見える。
対応率を印字するので、**100% から大きく外れたら入力を疑う**こと。
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "python"))
from g1_mapping.pcd_io import read_pcd, write_pcd_array  # noqa: E402

DEFAULT_MAX_RANGE = 6.0   # これより遠くから一度も見えていないボクセルを疑う
DEFAULT_SPAN = 5.0        # 観測がこの秒数に収まるボクセルを疑う
DEFAULT_VOXEL = 0.10      # 判定の粒度。占有格子の解像度に合わせる
STRUCTURE_SPAN = 60.0     # これ以上の時間幅を持つボクセルは「構造」とみなす歯止め
MATCH_WARN = 0.98         # 対応率がこれを下回ったら入力を疑う

# ⚠️ **帯の両端に理由がある。広げてはいけない。**（2026-09-08 に実測して決めた）
# 床と天井は grazing 角でしか見えないので、**近くからしか見えない**という
# 追従者と同じ署名を持つ。持続性で切ると床や天井が抜ける。
#
#   真の床基準の高さ      点数      署名     解釈
#   −1.00〜 0.15      365,615    7.3%   床そのもの      ← 触ってはいけない
#    0.15〜 0.23        5,249    1.0%   床のうねりの上端
#    0.23〜 1.75      189,655  0.0〜0.2% 脚・机・人の胴（既に掃除済み）
#    1.75〜 2.10       35,699    3.2%   ここが残っていた実体
#    2.10〜 2.60       43,844    1.0%   天井の縁
#    2.60〜             132,587    0.0%   天井そのもの
BAND = (0.23, 2.20)       # 下限は床のうねり（最大 +0.126 m）の上、上限は天井の下

# ⚠️ 床は**最頻ビン**で推定する。5% 分位は真の床より 0.06〜0.07 m 低く出るので、
# 帯の下限がその分だけ床に食い込む（pcd_to_occupancy.py が実際にそれで床を撃っていた）。
FLOOR_HIST_BINS = 100


def estimate_floor(points: np.ndarray) -> float:
    """z の下側の最頻ビンを床とみなす（`run_octomap.estimate_floor` と同じ）。

    分位ではなく最頻ビンを使う理由は上の BAND の注記を読むこと。
    """
    z = points[:, 2]
    low, high = np.percentile(z, [1.0, 50.0])
    lower = z[(z >= low) & (z <= high)]
    if len(lower) == 0:
        return float(z.min())
    counts, edges = np.histogram(lower, bins=FLOOR_HIST_BINS)
    return float(edges[int(counts.argmax())] + (edges[1] - edges[0]) / 2)


def voxel_key(points: np.ndarray, voxel: float) -> np.ndarray:
    """(N,3) を 1 本の int64 キーにする。負の座標も扱えるよう下駄を履かせる。"""
    v = np.floor(points / voxel).astype(np.int64)
    v -= v.min(axis=0)
    span = v.max(axis=0) + 1
    return (v[:, 0] * span[1] + v[:, 1]) * span[2] + v[:, 2]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session_dir", type=Path)
    p.add_argument("pcd", help="map/ 配下の入力名")
    p.add_argument("--source", default="benchmark_s5", help="観測履歴を取る姿勢つき PCD 置き場")
    p.add_argument("--output", default=None, help="map/ 配下の出力名（省略で <入力>_p.pcd）")
    p.add_argument("--max-range", type=float, default=DEFAULT_MAX_RANGE)
    p.add_argument("--span", type=float, default=DEFAULT_SPAN)
    p.add_argument("--voxel", type=float, default=DEFAULT_VOXEL)
    p.add_argument("--band", type=float, nargs=2, default=BAND,
                   help="この床上の帯だけを対象にする（帯の外は触らない）")
    p.add_argument("--dry-run", action="store_true")
    args = p.parse_args()

    target_path = args.session_dir / "map" / args.pcd
    if not target_path.exists():
        raise SystemExit("入力がありません: {}".format(target_path))
    target = read_pcd(target_path).points

    # 履歴と対象で同じキーを使うため、両方を連結してから一度に振る
    src = args.session_dir / args.source / "pcd"
    files = sorted(src.glob("*.pcd"))
    if not files:
        raise SystemExit("PCD がありません: {}".format(src))
    poses = np.loadtxt(args.session_dir / args.source / "poses.txt")
    hz = len(files) / (poses[-1, 0] - poses[0, 0])
    print("{} 枚 / {:.2f} 枚每秒 / ボクセル {} m".format(len(files), hz, args.voxel))

    chunks, ranges, indices = [], [], []
    for i, path in enumerate(files):
        data = read_pcd(path)
        pts = data.points
        if not len(pts):
            continue
        chunks.append(pts)
        ranges.append(np.hypot(pts[:, 0] - data.origin[0],
                               pts[:, 1] - data.origin[1]).astype(np.float32))
        indices.append(np.full(len(pts), i, dtype=np.int32))
    scan_pts = np.concatenate(chunks)
    both = voxel_key(np.vstack([scan_pts, target]), args.voxel)
    scan_key, target_key = both[:len(scan_pts)], both[len(scan_pts):]

    rng = np.concatenate(ranges)
    idx = np.concatenate(indices)
    order = np.argsort(scan_key, kind="stable")
    uniq, start = np.unique(scan_key[order], return_index=True)
    max_range = np.maximum.reduceat(rng[order], start)
    span = (np.maximum.reduceat(idx[order], start)
            - np.minimum.reduceat(idx[order], start) + 1) / hz

    at = np.searchsorted(uniq, target_key)
    at = np.clip(at, 0, len(uniq) - 1)
    matched = uniq[at] == target_key
    print("履歴を持つボクセル {:,} / 入力の点 {:,} のうち対応した {:,} ({:.1f}%)".format(
        len(uniq), len(target), int(matched.sum()), 100.0 * matched.mean()))
    if matched.mean() < MATCH_WARN:
        print("  ⚠️ 対応率が低い。入力がこのスキャン群から作られていない疑い"
              "（内蔵 SLAM 由来の点群を渡していないか）")

    floor_z = estimate_floor(target)
    height = target[:, 2] - floor_z
    in_band = (height >= args.band[0]) & (height <= args.band[1])
    drop = (matched & in_band
            & (max_range[at] < args.max_range) & (span[at] < args.span))
    structure = matched & (span[at] >= STRUCTURE_SPAN)

    print("床 z={:+.3f} m（最頻ビン）/ 帯 {:.2f}〜{:.2f} m の点 {:,}".format(
        floor_z, args.band[0], args.band[1], int(in_band.sum())))
    print("条件: 最遠 < {} m かつ 観測の時間幅 < {} 秒".format(args.max_range, args.span))
    print("  落とす点 {:,}（帯の {:.1f}% / 全体の {:.1f}%）".format(
        int(drop.sum()), 100.0 * drop.sum() / max(1, in_band.sum()),
        100.0 * drop.mean()))

    keep = ~drop
    kept_struct = structure & keep
    print("  構造ボクセル（観測 {:.0f} 秒以上）の残存: {:,} / {:,} = {:.1f}%".format(
        STRUCTURE_SPAN, int(kept_struct.sum()), int(structure.sum()),
        100.0 * kept_struct.sum() / max(1, structure.sum())))
    if structure.any() and kept_struct.sum() < structure.sum():
        print("  ⚠️ 構造を削っている。しきい値を緩めること")

    if args.dry_run:
        print("[dry-run] 書き出していない")
        return 0
    name = args.output or (Path(args.pcd).stem + "_p.pcd")
    out = args.session_dir / "map" / name
    write_pcd_array(out, target[keep])
    print("[OK] {}（{:,} 点）".format(out, int(keep.sum())))
    print("     ⚠️ Nav2 の 2D 地図への効果はほぼ無い（冒頭の表）。"
          "3D の用途（Isaac Sim / MuJoCo / RViz）のための掃除である")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
