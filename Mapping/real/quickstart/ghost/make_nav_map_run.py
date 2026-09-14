#!/usr/bin/env python3
"""**走る地図** `nav_map_run` を作る。**合否で止まる。**（2026-09-14）

## なぜ作り直すか

Nav2 / RViz2 の静的レイヤは 2026-09-08 の `nav_map_clean` だった。あれは OctoMap の
動的点除去で作ったもので、**机を消しすぎている** —— 別の日（09-10）の記録で測ると
**机の帯の重畳が 41.5 %**（掃除なしの地図なら 77.6 %）。**Nav2 は机を知らないまま走っていた。**

## なぜ「ゴースト除去した地図」に置き換えないのか

09-13 のゴースト除去（`run_all.py` の z4）＋ 胴の半径の管で作った地図は、机は戻るが
**追従者が消えきらない**。実測すると:

| 地図 | クリア中央 | 壁の帯 | 机の帯 | 通れる wp | 迂回ゼロ |
|---|---|---|---|---|---|
| 現行 `nav_map_clean` | 0.781 m | 94.2 % | **41.5 %** | 14/14 | 31 通り |
| z4 ＋ 全域管 R=0.30 | 0.500 m | 92.9 % | 76.3 % | **13/14** | **7 通り** |
| **合成（これ）** | **0.728 m** | **97.3 %** | **78.3 %** | **14/14** | **31 通り** |

置き換え案は wp8 を塞ぎ、迂回ゼロが 31 → 7 に落ちる。wp8 のところには
**高さ中央 1.11 m・軌跡から 0.48 m・どちらのライブからも見えない** 303 点があり、
人のゴーストらしいが**消しきる根拠が無い**（機体は 0.57 m 脇を通っただけ）。

## だから足し算にする

`nav_map_clean` に、**ゴースト除去した地図にだけ在り、かつ別の記録が実在を裏付けた
セルだけ**を足す。940 セル。消さないので**クリアランスも経路も壊れない**。

    Navigation/.venv/bin/python quickstart/ghost/make_nav_map_run.py
    ...                                                    --dry-run   # 書かずに採点だけ
"""
from __future__ import annotations

import argparse
import json
import math
import sys
from pathlib import Path

import numpy as np
from PIL import Image
from scipy import ndimage

HERE = Path(__file__).resolve().parent
REAL = HERE.parents[1]
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(REAL / "quickstart"))

from check_map_clearance import load_map, sample_clearance   # noqa: E402
from measure_overlay import read_map, count_hits             # noqa: E402
from score import Scorecard, WORK, SESSION, RES, cell_hmax   # noqa: E402

CLEAN = SESSION / "map" / "nav_map_clean"
GHOST = WORK / "cand" / "navmap" / "z4+全域管R300mm"          # run_navmap.py が作る
DST = SESSION / "map" / "nav_map_run"
ROBOT_RADIUS = 0.30
PLAN_RADIUS = 0.25            # check_long_routes.py と同じ
KO, KM = 1 << 20, 1 << 22


def cell_index(g) -> np.ndarray:
    yy, xx = np.nonzero(g.occupied)
    return np.c_[np.floor((g.origin[0] + (xx + .5) * g.resolution) / RES).astype(int),
                 np.floor((g.origin[1] + (yy + .5) * g.resolution) / RES).astype(int)]


def measure(yaml_path: Path, sc: Scorecard, tr, wps) -> dict:
    g = load_map(yaml_path)
    by_path, _, _, _ = sample_clearance(g, tr)
    occ, res, ox, oy = read_map(yaml_path)
    J = sc.judge

    def pct(m):
        hit, tot = count_hits(J.xyz[m], occ, res, ox, oy)
        return 100.0 * hit / max(tot, 1)

    blocked = ndimage.binary_dilation(
        g.occupied, ndimage.generate_binary_structure(2, 1),
        iterations=int(math.ceil(PLAN_RADIUS / g.resolution)))
    h, w = g.shape
    free = 0
    for p in wps:
        c = int((p["x"] - g.origin[0]) / g.resolution)
        r = int((p["y"] - g.origin[1]) / g.resolution)
        if 0 <= c < w and 0 <= r < h and not blocked[r, c]:
            free += 1
    return dict(cells=int(g.occupied.sum()), med=float(np.median(by_path)),
                tight=100.0 * float((by_path < ROBOT_RADIUS).mean()),
                wall=pct(J.wall), desk=pct(J.desk), free=free, total=len(wps))


def row(name: str, m: dict) -> None:
    print("{:<24}{:>8,}{:>9.3f}m{:>8.1f}%{:>8.1f}%{:>8.1f}%{:>8}/{}".format(
        name, m["cells"], m["med"], m["tight"], m["wall"], m["desk"], m["free"], m["total"]))


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true", help="書かずに採点だけ")
    a = ap.parse_args()

    if not Path(str(GHOST) + ".yaml").exists():
        raise SystemExit("先に run_navmap.py を回して {} を作ること".format(GHOST.name))

    sc = Scorecard()
    tr = np.loadtxt(SESSION / "mola_floor0" / "traj.txt", usecols=(1, 2))
    wps = json.loads((SESSION / "measure_20260908" / "waypoints.json").read_text())
    rh, gh = cell_hmax(sc.ref.xyz), sc.judge_hmax

    A = load_map(Path(str(CLEAN) + ".yaml"))
    B = load_map(Path(str(GHOST) + ".yaml"))
    have = set(map(tuple, cell_index(A).tolist()))
    only_b = np.array([c for c in cell_index(B).tolist() if tuple(c) not in have])
    k = (only_b[:, 0] + KO) * KM + (only_b[:, 1] + KO)
    # ⚠️ **見えないセルは足さない。**足し算なのでクリアランスを壊しうる。
    # 「どちらかのライブが 0.3 m 以上の高さで点を持つ」＝実在が裏取りできたものだけ
    real = np.array([gh.get(int(v), -9.0) >= 0.30 or rh.get(int(v), -9.0) >= 0.30 for v in k])
    add = only_b[real]
    print("ゴースト除去した地図にだけ在るセル {:,} / 実在が裏取りできた {:,} を足す".format(
        len(only_b), len(add)))

    img = np.asarray(Image.open(str(CLEAN) + ".pgm")).copy()
    meta = dict(l.split(":", 1) for l in Path(str(CLEAN) + ".yaml").read_text().splitlines()
                if ":" in l)
    ox, oy = eval(meta["origin"])[:2]
    h, w = img.shape
    cx = np.floor((add[:, 0] * RES + RES / 2 - ox) / RES).astype(int)
    cy = np.floor((add[:, 1] * RES + RES / 2 - oy) / RES).astype(int)
    m = (cx >= 0) & (cx < w) & (cy >= 0) & (cy < h)
    img[h - 1 - cy[m], cx[m]] = 0

    tmp = WORK / "cand" / "nav_map_run_candidate"
    tmp.parent.mkdir(parents=True, exist_ok=True)
    Image.fromarray(img).save(str(tmp) + ".pgm")
    Path(str(tmp) + ".yaml").write_text(
        Path(str(CLEAN) + ".yaml").read_text().replace("nav_map_clean.pgm", tmp.name + ".pgm"))

    print("\n{:<24}{:>8}{:>10}{:>8}{:>8}{:>8}{:>10}".format(
        "地図", "占有", "クリア中央", "<0.30m", "壁の帯", "机の帯", "通れる wp"))
    base = measure(Path(str(CLEAN) + ".yaml"), sc, tr, wps)
    row("現行 nav_map_clean", base)
    row("z4+全域管R300mm", measure(Path(str(GHOST) + ".yaml"), sc, tr, wps))
    cand = measure(Path(str(tmp) + ".yaml"), sc, tr, wps)
    row("→ 合成（これ）", cand)

    # ── 合否。**現行より悪くならないこと**を線にする ──────────────────
    checks = [("クリアランス中央 > {:.2f} m".format(ROBOT_RADIUS), cand["med"] > ROBOT_RADIUS),
              ("クリアランスが現行から -0.1 m 以内", cand["med"] >= base["med"] - 0.10),
              ("通れる wp が現行と同数", cand["free"] >= base["free"]),
              ("壁の帯が現行以上", cand["wall"] >= base["wall"]),
              ("机の帯が現行以上", cand["desk"] >= base["desk"])]
    print()
    for label, ok in checks:
        print("  {} {}".format("✅" if ok else "❌", label))
    if not all(ok for _, ok in checks):
        print("\n[NG] 合否に落ちた。**本番は書き換えない**", file=sys.stderr)
        return 1
    if a.dry_run:
        print("\n[dry-run] 書かない。候補は {}".format(tmp))
        return 0
    Image.fromarray(img).save(str(DST) + ".pgm")
    Path(str(DST) + ".yaml").write_text(
        Path(str(CLEAN) + ".yaml").read_text().replace("nav_map_clean.pgm", "nav_map_run.pgm"))
    print("\n[OK] {}.pgm と {}.yaml を書いた".format(DST, DST))
    print("     nav_stack.sh の既定はこれ。戻すなら G1_NAV_MAP=.../map/nav_map_clean.yaml")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
