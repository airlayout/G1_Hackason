# 11-06（カラー＋深度）: 集めた記録（shard_*.npz）の深度が使えるかを確かめる。枚数・深度の測れた割合・距離の分布、
# 段階ごとの画面の中央の距離と扉の本当の距離（pos_x）の比べ、カラーと深度を並べた絵（PNG）を書く。
# 使い方: python dep_check_1106.py <frames の親> <絵の出力先.png>
import glob
import os
import sys

import numpy as np

root, png = sys.argv[1], sys.argv[2]
files = sorted(glob.glob(os.path.join(root, "**", "shard_*.npz"), recursive=True))
assert files, f"記録が無い: {root}"
z = np.load(files[0])
assert "depth" in z.files, f"深度が記録に無い: {z.files}"
rgb, depth, truth, stage = z["rgb"], z["depth"].astype(np.float32), z["truth"], z["stage"]
n = len(depth)
valid = depth > 0
print(f"[dep] 束 {len(files)}・1 束目 {n} 枚・カラー {rgb.shape} {rgb.dtype}・深度 {depth.shape} {z['depth'].dtype}")
print(f"[dep] 測れた割合 {valid.mean():.3f}・距離（測れた所）中央 {np.median(depth[valid]):.2f} m・"
      f"5〜95% {np.percentile(depth[valid], 5):.2f}〜{np.percentile(depth[valid], 95):.2f} m・3 m に詰めた割合 {(depth >= 2.999).mean():.3f}")
h, w = depth.shape[1:]
center = depth[:, h // 3 : 2 * h // 3, w // 3 : 2 * w // 3]
for s in range(6):
    m = stage == s
    if m.sum() == 0:
        continue
    c = np.array([np.median(x[x > 0]) if (x > 0).any() else np.nan for x in center[m]])
    print(f"[dep] 段階 {s}: {m.sum()} 枚・画面の中央の距離 中央 {np.nanmedian(c):.2f} m・扉の枠の前後 pos_x 中央 {np.median(truth[m, 1]):.2f} m")
# 相関: 段階 0（近づく間）で画面の中央の距離と pos_x が一緒に動くか
m = stage == 0
if m.sum() > 20:
    c = np.array([np.median(x[x > 0]) if (x > 0).any() else np.nan for x in center[m]])
    ok = np.isfinite(c)
    print(f"[dep] 段階 0 の相関（画面の中央の距離 vs pos_x）: {np.corrcoef(c[ok], truth[m][ok, 1])[0, 1]:.2f}")
try:
    from PIL import Image

    idx = np.linspace(0, n - 1, 6).astype(int)
    tiles = []
    for i in idx:
        d = depth[i]
        dv = (np.clip(d / 3.0, 0, 1) * 255).astype(np.uint8)
        dv = np.repeat(np.repeat(dv, 2, axis=0), 2, axis=1)
        dv = np.stack([dv, dv, dv], -1)
        dv[np.repeat(np.repeat(d == 0, 2, axis=0), 2, axis=1)] = [255, 0, 0]  # 測れない所は赤
        tiles.append(np.concatenate([rgb[i][..., :3], dv], axis=1))
    Image.fromarray(np.concatenate(tiles, axis=0)).save(png)
    print(f"[dep] 絵 {png}（左カラー・右深度。白いほど遠い・赤は測れない）")
except Exception as e:  # noqa: BLE001
    print(f"[dep] 絵は書けなかった: {e}")
