#!/usr/bin/env python3
"""案 2: **後の光線が貫いた**ボクセルを消す（自由空間の彫り込み）。

「機体が通り抜けた」（案 1）を「**機体が見通した**」に広げる。軌跡が通っていない
開けた床（領域 B）にも効くのがねらい。

判定は方位ビンで行う: あるスキャンについて、候補ボクセルと同じ方位で
**測った点が 1 つ残らずボクセルより遠い**なら、その光線はボクセルを貫いている。

⚠️ **外れだけ数えてはいけない。**2026-09-13 に実測: 「最遠」でも「最近」でも
**机の帯が 3.5〜7.6 pt 落ちて全案不合格**だった。原因は、そのスキャンでたまたま机に
当たらず奥の床に当たった回も「貫いた」と数えるため。**当たりも数えて比で判じる**
（OctoMap の log-odds と同じ考え）。`ratio` は「外れが当たりの何倍なら消すか」。

⚠️ 座標系。スキャンは **benchmark(aligned) 系**、地図は **floor0 系**で
**z が約 1.30 m ずれる**（2026-09-12 の反復 2 で実測）。ここで合わせてから使う。
⚠️ `max_range` は必ず絞る（無制限だと天井が 57% 消える。既知の最適は 3〜4 m）。
"""
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
REAL = HERE.parents[1]
sys.path.insert(0, str(REAL / "python"))

from g1_mapping.pcd_io import read_pcd            # noqa: E402
from score import Scorecard, SESSION, FZ, RES, WORK     # noqa: E402
from filters import _cellkey, _support_cells, tube, TALL   # noqa: E402
from score import REGIONS                          # noqa: E402

NAZ, NEL = 1440, 720          # 方位ビン 0.25 度
MARGIN = 0.20                 # これより遠くで測っていたら「貫いた」


def load_scans(step: int = 1):
    files = sorted((SESSION / "benchmark_s5" / "pcd").glob("*.pcd"))[::step]
    tr = np.loadtxt(SESSION / "mola_floor0" / "traj.txt", usecols=(0, 1, 2, 3))
    ps = np.loadtxt(SESSION / "benchmark_s5" / "poses.txt")
    # 時刻で突き合わせて平行移動を測る（推測しない）
    j = np.searchsorted(tr[:, 0], ps[:, 0]).clip(0, len(tr) - 1)
    off = np.median(tr[j, 1:4] - ps[:, 1:4], axis=0)
    print("benchmark -> floor0 の平行移動（実測）: {}".format(np.round(off, 4).tolist()))
    for f in files:
        d = read_pcd(f)
        if len(d.points):
            yield np.asarray(d.origin) + off, d.points + off


def carve_votes(cand_xyz, max_range=4.0, step=1):
    """(外れの票, 当たりの票) を返す。"""
    votes = np.zeros(len(cand_xyz), np.int32)
    hits = np.zeros(len(cand_xyz), np.int32)
    for n, (o, pts) in enumerate(load_scans(step)):
        v = cand_xyz - o
        r = np.linalg.norm(v, axis=1)
        inrange = r < max_range
        if not inrange.any():
            continue
        idx = np.nonzero(inrange)[0]
        vn, rn = v[idx], r[idx]
        sv = pts - o
        sr = np.linalg.norm(sv, axis=1)
        keep = (sr > 0.3) & (sr < max_range + 3.0)
        sv, sr = sv[keep], sr[keep]
        if len(sr) == 0:
            continue

        def binof(vec, rad):
            az = ((np.arctan2(vec[:, 1], vec[:, 0]) + np.pi) / (2 * np.pi) * NAZ).astype(np.int32)
            el = ((np.arcsin(np.clip(vec[:, 2] / rad, -1, 1)) + np.pi / 2) / np.pi * NEL).astype(np.int32)
            return np.clip(az, 0, NAZ - 1) * NEL + np.clip(el, 0, NEL - 1)

        near = np.full(NAZ * NEL, np.inf, np.float32)
        np.minimum.at(near, binof(sv, sr), sr.astype(np.float32))
        b = binof(vn, rn)
        ok = np.isfinite(near[b])
        votes[idx[ok & (near[b] > rn + MARGIN)]] += 1
        hits[idx[ok & (np.abs(near[b] - rn) <= MARGIN)]] += 1
        if n % 300 == 0:
            print("  スキャン {} ... 外れ {:,} / 当たり {:,}".format(
                n, int((votes > 0).sum()), int((hits > 0).sum())), flush=True)
    return votes, hits


def main() -> int:
    sc = Scorecard()
    out = WORK / "cand" / "carve"
    h = sc.p[:, 2] - FZ
    key = _cellkey(sc.p[:, :2])
    support = _support_cells(sc.p, h)

    band = (h >= 0.15) & (h <= TALL)
    vz = np.floor(sc.p[:, 2] / RES).astype(np.int64)
    vox = key * 4096 + (vz + 2048)
    uniq, inv = np.unique(vox[band], return_inverse=True)
    # ボクセルの代表点（中心のかわりに実点の平均）
    cx = np.zeros((len(uniq), 3))
    np.add.at(cx, inv, sc.p[band])
    cnt = np.bincount(inv, minlength=len(uniq))
    cx /= cnt[:, None]
    print("候補ボクセル: {:,}（帯 0.15-1.90 m）".format(len(uniq)))

    print(sc.header())
    print(Scorecard.row(sc.evaluate("00_before", np.ones(len(sc.p), bool), out)))

    for mr in (3.0, 4.0):
        cache = WORK / "carve_votes_R{:.0f}.npz".format(mr)
        if cache.exists():
            z = np.load(cache); miss, hit = z["miss"], z["hit"]
        else:
            miss, hit = carve_votes(cx, max_range=mr, step=2)
            np.savez_compressed(cache, miss=miss, hit=hit, vox=uniq)
        mmap = np.zeros(len(sc.p), np.int32); mmap[band] = miss[inv]
        hmap = np.zeros(len(sc.p), np.int32); hmap[band] = hit[inv]
        for need, ratio in ((2, 3.0), (2, 10.0), (4, 10.0), (4, 30.0)):
            dr = band & (mmap >= need) & (mmap > ratio * hmap) & ~np.isin(key, support)
            print(Scorecard.row(sc.evaluate(
                "2_彫りR{:.0f}_票{}_比{:.0f}".format(mr, need, ratio), ~dr, out)))
        dr = (tube(sc.p, sc.d, R=0.50, keep_tall=True, boxes=[tuple(REGIONS["A_廊下"])])
              | (band & (mmap >= 4) & (mmap > 10.0 * hmap) & ~np.isin(key, support)))
        print(Scorecard.row(sc.evaluate("2z_A管+彫りR{:.0f}".format(mr), ~dr, out, save_txt=True)))
    print("\n" + Scorecard.rule())
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
