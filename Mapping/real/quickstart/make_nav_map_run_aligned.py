#!/usr/bin/env python3
"""別の地図の座標系へ移した記録から **走る地図** `nav_map_run` を作る。**合否で止まる。**（2026-09-25）

## なぜ `ghost/make_nav_map_run.py` を使わないのか

あちらは 0906 の MOLA 地図（`map/ref/map_full_raw.txt`）を入力に、机を**足す**道具である。
0906 の `nav_map_clean` は OctoMap（maxRange 4 m）が机を消しすぎていた（机の帯 41.5 %）。
0911 は maxRange 2 m で消し方が控えめなので**机は既に在る**（09-12 の記録で机の帯 83.9 %、
旧 `nav_map_run` は 74.3 %）。代わりに**消し残り**が問題になる。

## 何を落とすか

実測（`runs/20260911T201836_wireless_dualcam_map01`、09-12 の記録が見えたセルで裏取り）:

| セル | 数 | 09-12 で物体 | 09-12 で床だけ |
|---|---|---|---|
| 新旧どちらにも在る | 13,791 | 93.3 % | 6.7 % |
| 新にだけ在る・帯に 2 点以上 | 6,413 | 79.9 % | 20.1 % |
| **新にだけ在る・帯に 1 点** | **3,499** | **33.0 %** | **67.0 %** |

最後の行が机の島の間に伸びる放射状の筋で、**部屋の中央の通路を塞いでいた**
（8 m 以上の組で迂回ゼロが旧 29 通り → 9 通り）。0911 の軌跡は外周を回っただけで、
maxRange 2 m では中央を光線で削れていない。

⇒ **帯に 1 点しか無く、どの別の証拠でも裏が取れないセルを落とす。** 別の証拠とは:
- `--corroborate` の地図（既定: 旧座標系の `nav_map_run`）で占有
- ライブの記録 **09-12**（`score.REF_BAG`）がそのセルで床上 0.3 m 以上の点を見ている

⚠️ **合否は 09-10（`score.JUDGE_BAG`）だけで出す。** 守りと合否に同じ記録を使うと、
守ったセルをそのまま採点するので机の帯が 1 点も動かず**素通りする**（2026-09-25 に
一度そう書いて気づいた。`ghost/score.py` の「合否はフィルタが見ない記録で」と同じ理由）。
09-12 の列は報告のみ。

⚠️ **全部の 1 点セルは落とさない。** 裏取りせずに `--min-points 2` 相当にすると壁の帯が
−1.4 pt・机の帯が −6.8 pt 落ちる（09-09 に stride 5 で踏んだ「実在の物が消える」の再演）。

## 使い方

    ../../.venv/bin/python quickstart/make_nav_map_run_aligned.py \\
        runs/20260911T201836_wireless_dualcam_map01            # --dry-run で書かずに採点
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np
from PIL import Image

HERE = Path(__file__).resolve().parent
REAL = HERE.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(HERE / "ghost"))
sys.path.insert(0, str(REAL / "python"))

from check_map_clearance import load_map, sample_clearance     # noqa: E402
from g1_mapping.pcd_io import read_pcd                         # noqa: E402
from measure_overlay import count_hits, read_map               # noqa: E402
import score                                                   # noqa: E402

OLD_SESSION = REAL / "runs" / "20260906T135940_UiS_room_v3"
WAYPOINTS = OLD_SESSION / "measure_20260908" / "waypoints.json"
DEFAULT_CORROBORATE = OLD_SESSION / "map" / "nav_map_run.yaml"
ROBOT_RADIUS = 0.30
PLAN_RADIUS = 0.25            # check_long_routes.py と同じ
LIVE_OBJECT_M = 0.30          # ライブがこれ以上の高さの点を見たセルは実在
BAND_TOP_M = 1.80             # pcd_to_occupancy の帯の上限（clean_map.py と同じ）
# 合否の線。**入力（nav_map_clean）より悪くならないこと**
WALL_DROP_MAX = 0.4           # score.py の GUARD と同じ
DESK_DROP_MAX = 1.5


def band_counts(session: Path, grid, band_lower: float, floor_z: float) -> np.ndarray:
    """nav_map_clean を作った点群の、セルごとの帯の点数（load_map と同じ y 上向きの配列）。"""
    points = read_pcd(session / "map" / "map_octomap_sim.pcd").points
    height = points[:, 2] - floor_z
    band = points[(height >= band_lower) & (height <= BAND_TOP_M)]
    h, w = grid.shape
    col = np.floor((band[:, 0] - grid.origin[0]) / grid.resolution).astype(int)
    row = np.floor((band[:, 1] - grid.origin[1]) / grid.resolution).astype(int)
    ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    counts = np.bincount(row[ok] * w + col[ok], minlength=h * w).reshape(h, w)
    covered = (counts[grid.occupied] > 0).mean()
    if covered < 0.99:
        raise SystemExit("占有セルの {:.1%} しか点を持たない。点群と地図が対応していない"
                         "（床の高さか配列の向きを疑う）".format(covered))
    return counts


def live_floor_only_mask(grid) -> np.ndarray:
    """09-12 が点を持ち、しかも床上 LIVE_OBJECT_M 以上の点が 1 つも無いセル（y 上向き）。

    09-10 の記録は家具が動く前（`room-changed-between-recordings`）なので、
    **09-12 で床しか無いと分かっている所**の 09-10 の当たりを失うのは正しい。合否から外す。
    """
    xyz = score.live_points(score.REF_BAG, score.REF_CACHE)
    h, w = grid.shape
    col = np.floor((xyz[:, 0] - grid.origin[0]) / grid.resolution).astype(int)
    row = np.floor((xyz[:, 1] - grid.origin[1]) / grid.resolution).astype(int)
    ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    seen = np.zeros((h, w), bool)
    seen[row[ok], col[ok]] = True
    return seen & ~live_object_mask(grid)


def live_object_mask(grid) -> np.ndarray:
    """守りのライブ（09-12）が床上 LIVE_OBJECT_M 以上の点を見たセル。合否の 09-10 は使わない。"""
    # ⚠️ `score.cell_hmax` の格子（世界原点に固定）は使わない。地図の原点は端数を持つので
    # セルが半分ずれ、机の縁が守りから漏れる（2026-09-25 実測: 失った当たりの 57 % がこれ）
    xyz = score.live_points(score.REF_BAG, score.REF_CACHE)
    height = xyz[:, 2] - score.FZ
    obj = xyz[(height >= LIVE_OBJECT_M) & (height <= BAND_TOP_M)]
    h, w = grid.shape
    col = np.floor((obj[:, 0] - grid.origin[0]) / grid.resolution).astype(int)
    row = np.floor((obj[:, 1] - grid.origin[1]) / grid.resolution).astype(int)
    ok = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    seen = np.zeros((h, w), bool)
    seen[row[ok], col[ok]] = True
    return seen


def measure(yaml_path: Path, traj: np.ndarray, wps: list, bags: dict,
            moved: np.ndarray) -> dict:
    grid = load_map(yaml_path)
    by_path, _, _, _ = sample_clearance(grid, traj)
    occ, res, ox, oy = read_map(yaml_path)

    def pct(bag, mask):
        hit, tot = count_hits(bag.xyz[mask], occ, res, ox, oy)
        return 100.0 * hit / max(tot, 1)

    from scipy import ndimage
    blocked = ndimage.binary_dilation(grid.occupied, ndimage.generate_binary_structure(2, 1),
                                      iterations=int(np.ceil(PLAN_RADIUS / grid.resolution)))
    h, w = grid.shape
    free = []
    for i, p in enumerate(wps):
        c = int((p["x"] - grid.origin[0]) / grid.resolution)
        r = int((p["y"] - grid.origin[1]) / grid.resolution)
        if 0 <= c < w and 0 <= r < h and not blocked[r, c]:
            free.append(i)
    out = dict(cells=int(grid.occupied.sum()), med=float(np.median(by_path)),
               tight=100.0 * float((by_path < ROBOT_RADIUS).mean()), free=free)
    for name, bag in bags.items():
        out["wall" + name] = pct(bag, bag.wall)
        out["desk" + name] = pct(bag, bag.desk)
    # 09-10 の机の帯から「09-12 で床だけ」のセルに落ちる点を除いたもの（合否はこちら）
    j = bags["0910"]
    col = np.floor((j.xyz[:, 0] - grid.origin[0]) / grid.resolution).astype(int)
    row = np.floor((j.xyz[:, 1] - grid.origin[1]) / grid.resolution).astype(int)
    inside = (col >= 0) & (col < w) & (row >= 0) & (row < h)
    gone = np.zeros(len(j.xyz), bool)
    gone[inside] = moved[row[inside], col[inside]]
    out["desk0910_kept"] = pct(j, j.desk & ~gone)
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", type=Path)
    ap.add_argument("--corroborate", type=Path, default=DEFAULT_CORROBORATE,
                    help="裏取りに使う別の記録の地図（同じ原点・同じ大きさ）")
    ap.add_argument("--band-lower", type=float, required=True,
                    help="clean_map.py が決めた障害物帯の下限[m]（ログの「障害物帯の下限 =」）")
    ap.add_argument("--floor-z", type=float, required=True,
                    help="clean_map.py が測った床の高さ（ログの「床 z=」）")
    ap.add_argument("--dry-run", action="store_true")
    a = ap.parse_args()

    session = a.session_dir.resolve()
    src = session / "map" / "nav_map_clean"
    dst = session / "map" / "nav_map_run"
    grid = load_map(Path(str(src) + ".yaml"))
    ref = load_map(a.corroborate)
    if ref.shape != grid.shape or not np.allclose(ref.origin, grid.origin):
        raise SystemExit("裏取りの地図の原点・大きさが違う: {} {} / {} {}".format(
            ref.shape, ref.origin, grid.shape, grid.origin))

    counts = band_counts(session, grid, a.band_lower, a.floor_z)
    live = live_object_mask(grid)
    single = grid.occupied & (counts < 2)
    drop = single & ~ref.occupied & ~live
    print("占有 {:,} / 帯に 1 点 {:,} / うち別の地図で裏取り {:,}・09-12 で裏取り {:,} → 落とす {:,}"
          .format(int(grid.occupied.sum()), int(single.sum()), int((single & ref.occupied).sum()),
                  int((single & live).sum()), int(drop.sum())))

    img = np.asarray(Image.open(str(src) + ".pgm")).copy()
    h = img.shape[0]
    rows, cols = np.nonzero(drop)
    img[h - 1 - rows, cols] = 254          # load_map は y 上向き、PGM は上下逆
    cand = session / "map" / "_cand_nav_map_run"
    Image.fromarray(img).save(str(cand) + ".pgm")
    yaml_text = Path(str(src) + ".yaml").read_text().replace("nav_map_clean.pgm", cand.name + ".pgm")
    Path(str(cand) + ".yaml").write_text(yaml_text)

    traj = np.loadtxt(session / "mola_floor0" / "traj.txt", usecols=(1, 2))
    wps = json.loads(WAYPOINTS.read_text())
    bags = {"0910": score._Bag(score.live_points(score.JUDGE_BAG, score.JUDGE_CACHE)),
            "0912": score._Bag(score.live_points(score.REF_BAG, score.REF_CACHE))}
    moved = live_floor_only_mask(grid)
    base = measure(Path(str(src) + ".yaml"), traj, wps, bags, moved)
    new = measure(Path(str(cand) + ".yaml"), traj, wps, bags, moved)
    print("\n{:<16}{:>8}{:>10}{:>8}{:>9}{:>9}{:>9}{:>9}{:>8}".format(
        "地図", "占有", "クリア中央", "<0.30m", "壁09-10", "机09-10", "壁09-12", "机09-12", "wp"))
    for name, m in (("nav_map_clean", base), ("→ nav_map_run", new)):
        print("{:<16}{:>8,}{:>9.3f}m{:>7.1f}%{:>8.1f}%{:>8.1f}%{:>8.1f}%{:>8.1f}%{:>5}/{}".format(
            name, m["cells"], m["med"], m["tight"], m["wall0910"], m["desk0910"],
            m["wall0912"], m["desk0912"], len(m["free"]), len(wps)))
    blocked = [i for i in range(len(wps)) if i not in new["free"]]
    print("  塞がる wp: {}".format(blocked or "なし"))

    checks = [("クリアランス中央 > {:.2f} m".format(ROBOT_RADIUS), new["med"] > ROBOT_RADIUS),
              ("通れる wp が入力以上", set(base["free"]) <= set(new["free"]))]
    # 合否は守りに使っていない 09-10 だけ（09-12 は報告のみ）
    checks.append(("壁の帯(09-10) が入力 -{} pt 以内".format(WALL_DROP_MAX),
                   new["wall0910"] >= base["wall0910"] - WALL_DROP_MAX))
    print("  机の帯(09-10) から 09-12 で床だけのセルを除くと: {:.1f}% → {:.1f}%"
          "（除かない値 {:.1f}% → {:.1f}%）".format(base["desk0910_kept"], new["desk0910_kept"],
                                               base["desk0910"], new["desk0910"]))
    checks.append(("机の帯(09-10・動かされた物を除く) が入力 -{} pt 以内".format(DESK_DROP_MAX),
                   new["desk0910_kept"] >= base["desk0910_kept"] - DESK_DROP_MAX))
    print()
    for label, ok in checks:
        print("  {} {}".format("✅" if ok else "❌", label))
    if not all(ok for _, ok in checks):
        print("\n[NG] 合否に落ちた。**本番は書き換えない**。候補は {}".format(cand), file=sys.stderr)
        return 1
    if a.dry_run:
        print("\n[dry-run] 書かない。候補は {}".format(cand))
        return 0
    Image.fromarray(img).save(str(dst) + ".pgm")
    Path(str(dst) + ".yaml").write_text(
        Path(str(src) + ".yaml").read_text().replace("nav_map_clean.pgm", "nav_map_run.pgm"))
    print("\n[OK] {}.pgm と .yaml を書いた（落とした {:,} セル）".format(dst, int(drop.sum())))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
