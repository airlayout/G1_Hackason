"""ボクセルごとの観測履歴を **1 回だけ**計算し、対象の点ごとに展開して .npz に保存する。

filter_persistence.py と**同じキーの作り方**（連結してから voxel_key）を使う。
これがあれば、しきい値の掃引は 1 秒で回せる（毎回スキャン 1135 枚を読み直さない）。
"""
import sys
from pathlib import Path
REAL = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REAL / "quickstart"))
sys.path.insert(0, str(REAL / "python"))
import numpy as np
from g1_mapping.pcd_io import read_pcd
from filter_persistence import voxel_key, estimate_floor

S = REAL / "runs" / "20260906T135940_UiS_room_v3"
SOURCE, VOXEL = "benchmark_s5", 0.10
OUT = Path(sys.argv[1])

target = read_pcd(S / "map" / "ref_full_aligned.pcd").points     # benchmark 系
files = sorted((S / SOURCE / "pcd").glob("*.pcd"))
poses = np.loadtxt(S / SOURCE / "poses.txt")
hz = len(files) / (poses[-1, 0] - poses[0, 0])
print("{} 枚 / {:.2f} 枚每秒".format(len(files), hz))

chunks, ranges, indices = [], [], []
for i, path in enumerate(files):
    d = read_pcd(path)
    if not len(d.points):
        continue
    chunks.append(d.points)
    ranges.append(np.hypot(d.points[:, 0] - d.origin[0],
                           d.points[:, 1] - d.origin[1]).astype(np.float32))
    indices.append(np.full(len(d.points), i, dtype=np.int32))
    if i % 200 == 0:
        print("  {}/{}".format(i, len(files)), flush=True)
scan_pts = np.concatenate(chunks)
both = voxel_key(np.vstack([scan_pts, target]), VOXEL)
scan_key, target_key = both[:len(scan_pts)], both[len(scan_pts):]
rng, idx = np.concatenate(ranges), np.concatenate(indices)
order = np.argsort(scan_key, kind="stable")
uniq, start = np.unique(scan_key[order], return_index=True)
vox_max_range = np.maximum.reduceat(rng[order], start)
vox_span = (np.maximum.reduceat(idx[order], start)
            - np.minimum.reduceat(idx[order], start) + 1) / hz
vox_nscan = np.diff(np.append(start, len(order)))        # そのボクセルを見たスキャン数
vox_first = np.minimum.reduceat(idx[order], start) / hz  # 最初に見た時刻[s]
vox_last = np.maximum.reduceat(idx[order], start) / hz   # 最後に見た時刻[s]

at = np.clip(np.searchsorted(uniq, target_key), 0, len(uniq) - 1)
matched = uniq[at] == target_key
floor_z = estimate_floor(target)
print("スキャンの点 {:,} / 履歴を持つボクセル {:,} / 対応率 {:.1f}% / 床 z={:+.3f}".format(len(scan_pts), len(uniq), 100 * matched.mean(), floor_z))

np.savez_compressed(
    OUT, matched=matched,
    max_range=vox_max_range[at].astype(np.float32),
    span=vox_span[at].astype(np.float32),
    nscan=vox_nscan[at].astype(np.int32),
    first=vox_first[at].astype(np.float32),
    last=vox_last[at].astype(np.float32),
    height=(target[:, 2] - floor_z).astype(np.float32),
    floor_z=floor_z)
print("[OK]", OUT)
