#!/usr/bin/env python3
"""追従者ゴーストを消す 3 つの手。**入口はここ 1 つ**にして同じスコアカードで比べる。

| 手 | 証拠 | 効く所 | 効かない所 |
|---|---|---|---|
| `tube` | **機体が後から通り抜けた**（軌跡） | 機体の通り道に居た追従者 | 通り道から外れた人 |
| `carve` | **後の光線が貫いた**（視線） | 見通せた所すべて | 一度も見通していない所 |
| `blob` | **人の形で、上に支えが無い** | 孤立した人サイズの塊 | 壁際・机に寄りかかった人 |

⚠️ どれも「守るもの」は同じ（`score.py` のスコアカード）。**机と壁を消したら不合格。**
"""
from __future__ import annotations

import numpy as np
from scipy import ndimage
from scipy.spatial import cKDTree

FZ = 0.013
RES = 0.10
TALL = 1.90          # 人の高さの上限。ここまでが「消してよい帯」

# ⚠️ 「支え」は**帯**で見る。2026-09-12 は「1.9 m より上に点があるセル ＝ 構造」だったが、
# 2026-09-13 に実測: **天井が 2.7-3.0 m に 8,127 セルぶん写っている**ので、
# その規則だと**占有セルの 53% が構造**になる（帯にすると 10%）。
# 柱・棚・壁はこの帯を貫くが、天井は入らない
SUPPORT = (1.90, 2.30)


def _support_cells(p: np.ndarray, h: np.ndarray) -> np.ndarray:
    m = (h > SUPPORT[0]) & (h < SUPPORT[1])
    return np.unique(_cellkey(p[m, :2]))


# セルの鍵。負の座標でも衝突しないようにゲタを履かせる。
# 従来の `cx*100000 + cy` はこの部屋（|cy| < 300）では衝突しないが、
# cy が 100000 に届く地図では別のセルと同値になる。**地図を広げたら黙って壊れる**式なので
# 使わない（2026-09-13。当初これを不具合と読んだが、実測では衝突ゼロだった）
_KOFF = 1 << 20
_KMUL = 1 << 22


def _cellkey(xy: np.ndarray) -> np.ndarray:
    c = np.floor(xy / RES).astype(np.int64)
    return (c[:, 0] + _KOFF) * _KMUL + (c[:, 1] + _KOFF)


def _cellkey_ij(cx, cy):
    """セル添字から同じ鍵を作る。"""
    return (np.asarray(cx, np.int64) + _KOFF) * _KMUL + (np.asarray(cy, np.int64) + _KOFF)


def _in_boxes(p: np.ndarray, boxes) -> np.ndarray:
    if boxes is None:
        return np.ones(len(p), bool)
    m = np.zeros(len(p), bool)
    for x0, x1, y0, y1 in boxes:
        m |= (p[:, 0] >= x0) & (p[:, 0] <= x1) & (p[:, 1] >= y0) & (p[:, 1] <= y1)
    return m


def tube(p, d, R=0.50, keep_tall=True, band=(0.15, 1.90), boxes=None) -> np.ndarray:
    """案 1: 軌跡から R 以内・帯の中の点を消す。落とす点の bool を返す。

    ⚠️ `keep_tall` は「同じセルに 1.9 m より上の点があれば柱として残す」。
    開けた床では人の上に何も無いので外した方がよく落ちるが、柱も道連れになる。
    """
    h = p[:, 2] - FZ
    drop = (h >= band[0]) & (h <= band[1]) & (d <= R) & _in_boxes(p, boxes)
    if keep_tall:
        drop &= ~np.isin(_cellkey(p[:, :2]), _support_cells(p, h))
    return drop


def blob(p, live_hmax=None, max_area=1.2, hi=1.20, band=(0.15, 1.90),
         min_cells=2, live_keep=0.30, low_area=1.5, boxes=None) -> np.ndarray:
    """案 3: `hi` より上を 2D の塊に分け、**人らしい塊**の柱をまるごと消す。

    人らしさ（塊ごと） = ①1.9 m より上に点が無い（支えが無い）②足跡が `max_area` 以下

    ⚠️ そのうえで**セルごとに 2 つの保護**を掛ける。2026-09-13 に無しで試したら
    **机を 11.1% 巻き込んで不合格**だった（机の上に立った人を消すと机まで消える）:

    - **ライブがそのセルに `live_keep` 以上の高さで点を持つなら残す。**
      6 日後にも在る ＝ 家具。ライブが見ていないセルは判定に使わない
    - **低い帯（0.2-0.8 m）で `low_area` より大きい塊に属するセルは残す。**
      机は広くつながった低い構造、立っている人は孤立した柱

    2026-09-13 に領域 B で確かめた: 塊 11 は面積 0.50 m2 / 最高 1.67 m / 1.9 m 超 0 点 /
    軌跡まで 0.01 m / ライブは床 0.03 m しか無い。対して塊 2 は 1.9 m 超が 1,016 点 ＝ 構造。
    """
    h = p[:, 2] - FZ
    key = _cellkey(p[:, :2])
    tall_cells = set(_support_cells(p, h).tolist())

    # 低い帯の大きな塊 ＝ 家具。ここに属するセルは触らない
    low_keep = set()
    lm = (h >= 0.20) & (h <= 0.80)
    if lm.any():
        li = np.floor(p[lm, :2] / RES).astype(np.int64)
        lx0, ly0 = li[:, 0].min(), li[:, 1].min()
        lg = np.zeros((li[:, 1].max() - ly0 + 1, li[:, 0].max() - lx0 + 1), bool)
        lg[li[:, 1] - ly0, li[:, 0] - lx0] = True
        ll, ln = ndimage.label(lg, structure=np.ones((3, 3)))
        sz = np.bincount(ll.ravel())
        big = np.nonzero(sz * RES * RES > low_area)[0]
        big = big[big > 0]
        if len(big):
            yy, xx = np.nonzero(np.isin(ll, big))
            low_keep = set(_cellkey_ij(xx + lx0, yy + ly0).tolist())

    cand = (h > hi) & (h <= TALL) & _in_boxes(p, boxes)
    if not cand.any():
        return np.zeros(len(p), bool)
    ci = np.floor(p[cand, :2] / RES).astype(np.int64)
    x0, y0 = ci[:, 0].min(), ci[:, 1].min()
    grid = np.zeros((ci[:, 1].max() - y0 + 1, ci[:, 0].max() - x0 + 1), bool)
    grid[ci[:, 1] - y0, ci[:, 0] - x0] = True
    lab, n = ndimage.label(grid, structure=np.ones((3, 3)))

    drop_cells = []
    for cid in range(1, n + 1):
        yy, xx = np.nonzero(lab == cid)
        if len(yy) < min_cells or len(yy) * RES * RES > max_area:
            continue
        keys = _cellkey_ij(xx + x0, yy + y0)
        if any(int(k) in tall_cells for k in keys):
            continue
        for k in keys:
            k = int(k)
            if k in low_keep:
                continue
            if live_hmax is not None and live_hmax.get(k, -9.0) >= live_keep:
                continue
            drop_cells.append(k)
    if not drop_cells:
        return np.zeros(len(p), bool)
    return (np.isin(key, np.array(drop_cells, np.int64))
            & (h >= band[0]) & (h <= band[1]))


def carve(p, scans, poses, max_range=4.0, hits_needed=2, band=(0.15, 1.90),
          keep_tall=True, boxes=None) -> np.ndarray:
    """案 2: **後の光線が貫いた**ボクセルを消す（自由空間の彫り込み）。

    `scans`: [(t, (N,3) livox 系の点)], `poses`: 各スキャンの map 系 4x4。
    地図の点を 0.1 m のボクセルに落とし、各光線が通り抜けたボクセルに票を入れる。
    `hits_needed` 票以上を「貫かれた」とする。

    ⚠️ `max_range` は必ず絞る（無制限だと天井が 57% 消える。既知の最適は 3〜4 m）。
    """
    h = p[:, 2] - FZ
    key = _cellkey(p[:, :2])
    vz = np.floor(p[:, 2] / RES).astype(np.int64)
    vox = key * 4096 + (vz + 2048)
    order = np.argsort(vox)
    uvox = vox[order]
    uniq, first = np.unique(uvox, return_index=True)
    votes = np.zeros(len(uniq), np.int32)

    for T, pts in zip(poses, scans):
        rng = np.linalg.norm(pts, axis=1)
        pts = pts[(rng > 0.3) & (rng < max_range)]
        if len(pts) == 0:
            continue
        w = (T[:3, :3] @ pts.T).T + T[:3, 3]
        o = T[:3, 3]
        v = w - o
        L = np.linalg.norm(v, axis=1)
        steps = np.arange(0.5, 1.0, RES / max_range)          # 光線上の中間点だけ
        for s in steps:
            q = o + v * s
            k = (_cellkey(q[:, :2]) * 4096
                 + (np.floor(q[:, 2] / RES).astype(np.int64) + 2048))
            idx = np.searchsorted(uniq, k)
            ok = (idx < len(uniq)) & (uniq[np.minimum(idx, len(uniq) - 1)] == k)
            np.add.at(votes, idx[ok], 1)
    carved = uniq[votes >= hits_needed]
    out = np.isin(vox, carved) & (h >= band[0]) & (h <= band[1]) & _in_boxes(p, boxes)
    if keep_tall:
        out &= ~np.isin(key, _support_cells(p, h))
    return out
