#!/usr/bin/env python3
"""**走る地図**（Nav2 / RViz2 の静的レイヤ）の候補を作って採点する（2026-09-14）。

## 基準地図とは合否が違う

| | 重畳の基準 `nav_map_ref` | 走る地図 `nav_map_clean` |
|---|---|---|
| 要求 | 地図が知っている物を**漏らさず持つ** | 自分の歩いた道を**塞がない** |
| 主な合否 | 壁の帯の重畳 | **軌跡クリアランス > 0.30 m** |
| 帯 | 0.15–1.80 | **0.23–1.80**（床のうねり + 10 cm） |

## なぜ作り直すか

現行 `nav_map_clean`（2026-09-08）は OctoMap の動的点除去で作ったもので、
**机を 98 % 消している**（机の高さ帯の点 11,809 → 271）。クリアランス 0.781 m は
**本物の家具を消して買った数字**である。一方 09-13 のゴースト除去（`run_all.py` の z4）は
机と壁を残すので、そのままだとクリアランスが 0.283 m で**合格しない**。

2026-09-14 に塞いでいるセルを実測したところ、**279 個のうち 263 個（94 %）が
軌跡から 0.225 m 以内**だった。機体の胴（肩幅 0.45 m）が物理的にそこを占めていたので、
**静止物はあり得ない**＝残ったゴースト。だから走る地図には**胴の半径で全域に管をかける**。

⚠️ 半径は `R=0.50` にしないこと。それは机（09-10 の記録で裏の取れた点）を 23 % 消す。
胴の半径 0.225 m の近傍だけが「物理的にあり得ない」と言い切れる範囲である。
"""
from __future__ import annotations

import sys
from pathlib import Path

import argparse

import numpy as np
from scipy.spatial import cKDTree

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
REAL = HERE.parents[1]
sys.path.insert(0, str(REAL / "quickstart"))

from pcd_to_occupancy import build_grid, write_map          # noqa: E402
# ⚠️ クリアランスは**既存の道具の読み込みをそのまま使う**。2026-09-14 に自前で書いたら
# `build_grid` の配列が PGM と上下逆（write_map が反転する）ことに気づかず、
# 「前（掃除なし）」が 0.402 m（正しくは 0.100 m）で合格に見えた。**占有の判定は 1 箇所に**
from check_map_clearance import load_map, sample_clearance   # noqa: E402
from measure_overlay import read_map, count_hits             # noqa: E402
from score import Scorecard, WORK, SESSION, FZ, RES, REGIONS, cell_hmax   # noqa: E402
from filters import tube, blob, _cellkey, _support_cells, TALL             # noqa: E402

RUN_BAND = (0.23, 1.80)      # 走る地図の帯（pcd_to_occupancy の既定と同じ）
ROBOT_RADIUS = 0.30          # 内接円。合否の線
BODY_RADIUS = 0.225          # 肩幅 0.45 m の半分。ここより内は静止物があり得ない
TRAJ = SESSION / "mola_floor0" / "traj.txt"
CLEAN = SESSION / "map" / "nav_map_clean"


def clearance(yaml_path: Path, tr: np.ndarray) -> tuple[float, float]:
    """(軌跡セルの中央クリアランス[m], 0.30 m 未満の割合[%])。"""
    by_path, _, _, _ = sample_clearance(load_map(yaml_path), tr)
    return float(np.median(by_path)), 100.0 * float((by_path < ROBOT_RADIUS).mean())


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--install", type=float, metavar="R",
                    help="この半径の候補を本番 map/nav_map_run に入れる（**合否を通ったときだけ**）")
    a = ap.parse_args()

    sc = Scorecard()                       # 09-10 の裏取りを借りる
    # ⚠️ 机の守りは**両日で安定している机**に限る。2026-09-14 に 3 度目の汚染を実測:
    # `Scorecard` の「机」932 点のうち 228 点（24.5 %）は机ではなかった —— 111 点は
    # **軌跡から 0.225 m 以内**（機体の胴が通った＝静止物はあり得ない。高さ中央 1.08 m）、
    # 91 点は 09-10 に在って 09-12 に無い**動かされた家具**。残る 704 点だけが
    # 地図 0.71 m / 09-12 も 0.73 m で一致する本物である
    rh = cell_hmax(sc.ref.xyz)
    kk0 = _cellkey(sc.p[:, :2])
    stable = np.array([rh.get(int(v), -9.0) >= 0.30 for v in kk0])
    desk = sc.g["机"] & stable & (sc.d > BODY_RADIUS)
    print("机の守り: {:,} 点 -> 両日で安定な {:,} 点に絞った".format(
        int(sc.g["机"].sum()), int(desk.sum())))
    tr = np.loadtxt(TRAJ, usecols=(1, 2))
    out = WORK / "cand" / "navmap"
    out.mkdir(parents=True, exist_ok=True)
    # z4 は run_all.py と同じ組み合わせを**その場で作り直す**（txt の突き合わせはしない）
    h = sc.p[:, 2] - FZ
    kk = _cellkey(sc.p[:, :2])
    sup = _support_cells(sc.p, h)
    band = (h >= 0.15) & (h <= TALL)
    z = np.load(WORK / "carve_votes_R4.npz")
    vox = kk * 4096 + (np.floor(sc.p[:, 2] / RES).astype(np.int64) + 2048)
    ii = np.searchsorted(z["vox"], vox).clip(0, len(z["vox"]) - 1)
    hit_v = z["vox"][ii] == vox
    miss = np.where(hit_v, z["miss"][ii], 0)
    hit = np.where(hit_v, z["hit"][ii], 0)
    drop_z4 = (tube(sc.p, sc.d, R=0.50, keep_tall=True, boxes=[tuple(REGIONS["A_廊下"])])
               | tube(sc.p, sc.d, R=0.70, keep_tall=False, boxes=[tuple(REGIONS["B_開けた床"])])
               | blob(sc.p, live_hmax=cell_hmax(sc.ref.xyz), max_area=1.2, hi=1.20, min_cells=2)
               | (band & (miss >= 4) & (miss > 10.0 * hit) & ~np.isin(kk, sup)))
    keep_z4 = ~drop_z4
    print("z4 で残る点 {:,} / 元 {:,}".format(int(keep_z4.sum()), len(sc.p)))

    # ⚠️ 机の削減だけでは**過剰な除去を捕まえられない**（2026-09-14 に反証で確認。
    # 胴の半径の 4 倍の R=1.00 でも机の削減 2.7 % で通ってしまう。本物の机は軌跡から
    # 1 m 超に居るため）。基準地図と同じ**09-10 の重畳**を合否に足す
    base_wall = base_desk = None
    print("\n{:<24}{:>9}{:>10}{:>9}{:>9}{:>9}{:>7}{:>7}".format(
        "候補", "占有セル", "クリア中央", "<0.30m", "壁の帯", "机の帯", "机減", "判定"))

    def overlay(yaml_path):
        occ, res, ox, oy = read_map(yaml_path)
        J = sc.judge
        f = lambda m: 100.0 * count_hits(J.xyz[m], occ, res, ox, oy)[0] / max(
            count_hits(J.xyz[m], occ, res, ox, oy)[1], 1)
        return f(J.wall), f(J.desk)

    def row(name, keep):
        nonlocal base_wall, base_desk
        img, lo = build_grid(sc.p[keep], RES, RUN_BAND, 0.30, 1, verbose=False)
        write_map(img, lo, RES, out / name)
        y = (out / name).with_suffix(".yaml")
        med, pct = clearance(y, tr)
        wall, dband = overlay(y)
        dpct = 100.0 * ((~keep) & desk).sum() / max(desk.sum(), 1)
        if base_wall is None:
            base_wall, base_desk = wall, dband
            v = "基準"
        else:
            v = "OK" if (med > ROBOT_RADIUS and dpct <= 10.0
                         and wall >= base_wall - 0.4
                         and dband >= base_desk - 1.5) else "NG"
        print("{:<24}{:>9,}{:>9.3f}m{:>8.1f}%{:>8.1f}%{:>8.1f}%{:>6.1f}%{:>7}".format(
            name, int((img == 0).sum()), med, pct, wall, dband, dpct, v))
        return v == "OK"

    row("00_前（掃除なし）", np.ones(len(sc.p), bool))
    row("z4_基準地図の掃除だけ", keep_z4)
    # ⚠️ 出力名にドットを入れない。`Path.with_suffix` が `R0.225` を `R0.pgm` にする
    # （既知の罠。2026-09-14 にまた踏んだ）。cm 単位の整数で名前を作る
    for R in (0.225, 0.25, 0.30, 0.35):
        drop = ~keep_z4 | tube(sc.p, sc.d, R=R, keep_tall=True, boxes=None)
        row("z4+全域管R{:03d}mm".format(int(round(R * 1000))), ~drop)
    # 反証: 胴の半径をはるかに超える管は**本物の構造を食って落ちる**べき
    drop = ~keep_z4 | tube(sc.p, sc.d, R=1.00, keep_tall=True, boxes=None)
    row("[反証] z4+全域管R1000mm", ~drop)

    # 現行の走る地図
    cy = Path(str(CLEAN) + ".yaml")
    g = load_map(cy)
    med, pct = clearance(cy, tr)
    wall, dband = overlay(cy)
    print("{:<24}{:>9,}{:>9.3f}m{:>8.1f}%{:>8.1f}%{:>8.1f}%{:>6}{:>7}".format(
        "現行 nav_map_clean", int(g.occupied.sum()), med, pct, wall, dband, "—", "—"))
    print("\n合否: クリアランス中央 > {:.2f} m / 壁の帯 >= 基準 -0.4pt / 机の帯 >= 基準 -1.5pt"
          " / 机の削減 <= 10 %（重畳は 09-10 の記録）".format(ROBOT_RADIUS))

    if a.install is None:
        return 0
    R = a.install
    print("\n" + "=" * 70)
    keep = ~(~keep_z4 | tube(sc.p, sc.d, R=R, keep_tall=True, boxes=None))
    name = "install_R{:03d}mm".format(int(round(R * 1000)))
    tmp = out / name
    if not row(name, keep):
        print("\n[NG] 合否に落ちた。**本番は書き換えない**", file=sys.stderr)
        return 1
    dst = SESSION / "map" / "nav_map_run"
    for ext in (".pgm", ".yaml"):
        src = tmp.with_suffix(ext)
        text = src.read_bytes()
        if ext == ".yaml":
            text = text.replace(tmp.name.encode(), b"nav_map_run")
        dst.with_suffix(ext).write_bytes(text)
    np.savetxt(SESSION / "map" / "nav_map_run_points.txt", sc.p[keep],
               header="x y z", comments="", fmt="%.6f")
    print("\n[OK] {} と {} を書いた".format(dst.with_suffix(".pgm"), dst.with_suffix(".yaml")))
    print("     点群も残した: map/nav_map_run_points.txt")
    print("     ⚠️ 09-13 以前の走行の数字は nav_map_clean で測っている。**並べて比べない**")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
