#!/usr/bin/env python3
"""記録 1 本から Nav2 用の 2D 地図と Isaac Sim 用の点群を作る。**合否で止まる。**

## この道具の立場

**「黙って地図が出る」ためではなく「合否で止まる」ために 1 本にまとめた。**
手順は記録によらないが、**しきい値の一部は部屋の広さと床のうねりに直接依存する**。
「前の記録で良かった値」をそのまま当てるのが一番危ない — どちらも**地図はできてしまう**ので、
失敗しても黙って通る（詳細は `docs/plan/g1-stack-architecture.html` の図 3b と「運用」節）。

| 値 | 何から決まるか | 記録ごとに |
|---|---|---|
| 近距離除去の半径 | **部屋の広さ。**「構造は遠くからも見える」が成り立つ必要がある | **変わる** |
| OctoMap の maxRange | 掃引で決める。過去に 2 / 3 / 4 m と記録ごとに動いた | **毎回掃引**（`--sweep`） |
| 障害物帯の下限 | **床のうねりの最大値 ＋ 10 cm** | **毎回測る**（この道具が測る） |
| 持続性の 最遠 / 時間幅 | 追従者と構造の分離の度合い | **毎回確認**（構造残存が合否） |

## 段と合否

    ① 追従者とノイズを落とす（共通）
       filter_scans_near.py → run_octomap.py
       → filter_persistence.py（✔ 構造ボクセル残存 100 %）
    ② 2D — Nav2 の静的レイヤ
       pcd_to_occupancy.py（--min-points）→ ✔ check_map_clearance.py（軌跡クリアランス）
    ③ 3D — Isaac Sim / MuJoCo
       pcd_to_mjcf.py（✔ 偽障害物）

⚠️ **①の出力（map_octomap_sim.pcd）を 2D と 3D の両方が読む。**
2026-09-08 まで `filter_persistence.py` は③の中にあり、**2D の nav_map だけ
掃除が浅い**状態だった。入力を揃えてこれを閉じる。
⚠️ **点数の閾値までは揃えない。** `pcd_to_mjcf.py` は 3 点を要求するが、
2D で同じことをすると副作用が勝つ（2026-09-09 実測。MIN_POINTS_2D の注記を読むこと）。

**✔ が落ちたらそこで止まり、何を掃引すべきか印字する。**

## 使い方

    ../../G1_Hackason/.venv/bin/python quickstart/clean_map.py runs/<id>
    ... clean_map.py runs/<id> --sweep 2 3 4 5     # maxRange を掃引して止まる（人が選ぶ）
    ... clean_map.py runs/<id> --max-range 3.0     # 掃引で決めた値を当てる
    ... clean_map.py runs/<id> --skip-sim          # 2D だけ

⚠️ **`octomap` は `G1_Hackason/.venv` にしか入っていない。**
`Navigation/.venv` で走らせると ③ の手前で落ちる。この道具は起動時に確かめる。
"""
from __future__ import annotations

import argparse
import re
import subprocess
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "python"))     # HERE は quickstart/。python/ はその隣
from g1_mapping.pcd_io import read_pcd  # noqa: E402

DEFAULT_SOURCE = "benchmark_s5"
DEFAULT_RADIUS = 4.0          # 近距離除去。部屋の広さに依存する
DEFAULT_MAX_RANGE = 4.0       # OctoMap。掃引で決めるべき値
DEFAULT_FLOOR_MARGIN = 0.10   # 床のうねりの上に積む余裕[m]
BAND_UPPER_2D = 1.80          # Nav2 は天井が要らない
# 帯の中の点数がこれ未満のセルは占有にしない。
# ⚠️ **1（従来の挙動）のままにしてある。**2026-09-09 に上げて測ったが副作用が勝った:
# 2 以上にすると「sim に在るのに nav_map に無い」食い違いが 119 → 196 → 307 件に増え、
# 3 では機体が実際に当たった 0.75 m の物体まで消え、経路が壁を 3 セル抜けた。
# しかも最狭部は良くならない（infl 0.55 で 0.57 → 0.45 / 0.50 m）。
# 経路を広い側へ寄せたいなら inflation_radius を上げる（tune_inflation.py）。
MIN_POINTS_2D = 1
BAND_UPPER_3D = 2.20          # 3D は人の頭（〜2.1m）まで消したい
FLOOR_CELL = 0.5              # 床のうねりを測る格子の一辺[m]
FLOOR_SLAB = 0.30             # 床とみなす帯（推定床からの片側）[m]
# ⚠️ **点の少ないセルを外さないと外れ値に引きずられる。**（2026-09-08 に実測して決めた）
# 床は grazing 角でしか見えないので、遠いセルには点が数個しか入らず、
# そこに写っているのは床ではなく物の底である。同じ記録で:
#     絞り込みなし          p99 = +0.251 m → 下限 0.35（check_calibration の 0.23 と食い違う）
#     1 セル 20 点以上      p99 = +0.157 m
#     1 セル 50 点以上      p99 = +0.130 m ← check_calibration の実測 +0.126 とほぼ一致
FLOOR_MIN_POINTS = 50
FLOOR_PERCENTILE = 99.0       # うねりの上端。最大値は 1 セルの外れ値で決まるので使わない
PHANTOM_AREA_MAX = 3.0        # 偽障害物の面積[m²]。これを超えたら追従者が残っている
NAV2_YAML = HERE.parents[2] / "Navigation/nav2/g1_nav2.yaml"


def run(argv: "list[str]", label: str) -> int:
    """外部ツールを走らせ、出力をそのまま流す。終了コードを返す。"""
    print("\n\033[1m── {} ──\033[0m".format(label), flush=True)
    print("   $ {}".format(" ".join(str(a) for a in argv)), flush=True)
    return subprocess.call([str(a) for a in argv])


def die(step: str, hint: str) -> "None":
    print("\n\033[1m[停止] {}\033[0m".format(step))
    print("  {}".format(hint))
    raise SystemExit(1)


def estimate_floor(points: np.ndarray) -> float:
    """z の下側の最頻ビンを床とみなす（run_octomap / filter_persistence と同じ）。"""
    z = points[:, 2]
    low, high = np.percentile(z, [1.0, 50.0])
    lower = z[(z >= low) & (z <= high)]
    if len(lower) == 0:
        return float(z.min())
    counts, edges = np.histogram(lower, bins=100)
    return float(edges[int(counts.argmax())] + (edges[1] - edges[0]) / 2)


def measure_floor_band(points: np.ndarray, margin: float) -> "tuple[float, float, float]":
    """床のうねりを測り、障害物帯の下限を出す。

    `check_calibration.py` と同じ考え方（**床の最大値 ＋ 余裕**）を、bag ではなく
    出来あがった点群だけで出す。セルごとの床の高さの中央値を取り、その分布の上端を使う
    （1 点の外れ値で決めない）。
    """
    floor = estimate_floor(points)
    height = points[:, 2] - floor
    slab = points[np.abs(height) <= FLOOR_SLAB]
    if len(slab) < 1000:
        raise SystemExit("床とみなせる点が少なすぎる（{} 点）。入力を確認する".format(len(slab)))
    cell = np.floor(slab[:, :2] / FLOOR_CELL).astype(np.int64)
    key = (cell[:, 0] - cell[:, 0].min()) * 100000 + (cell[:, 1] - cell[:, 1].min())
    order = np.argsort(key, kind="stable")
    _, start = np.unique(key[order], return_index=True)
    h = slab[order][:, 2] - floor
    segments = np.split(h, start[1:])
    medians = np.array([np.median(seg) for seg in segments if len(seg) >= FLOOR_MIN_POINTS])
    if len(medians) < 50:
        raise SystemExit(
            "床を測れるセルが {} 個しかない（1 セル {} 点以上が必要）。"
            "間引きを弱めるか FLOOR_MIN_POINTS を下げる".format(len(medians), FLOOR_MIN_POINTS))
    top = float(np.percentile(medians, FLOOR_PERCENTILE))
    return floor, top, round(top + margin, 2)


def nav2_min_obstacle_height() -> "float | None":
    """g1_nav2.yaml の min_obstacle_height を読む（食い違いを声に出すため）。"""
    if not NAV2_YAML.exists():
        return None
    found = re.findall(r"min_obstacle_height:\s*([\d.]+)", NAV2_YAML.read_text())
    return float(found[0]) if found else None


def sweep_max_range(py: str, session: Path, source: str, values: "list[float]") -> None:
    """maxRange を掃引して指標を並べ、**選ばせて止まる**。"""
    print("\n\033[1m[掃引] OctoMap の maxRange を {} で比較する\033[0m".format(values))
    print("  ⚠️ 最適値は記録ごとに動く（過去に 2 / 3 / 4 m と変わった）。伸ばすほど"
          "動的点は消えるが**床と壁も削れる**")
    for value in values:
        name = "map_octomap_sweep{}.pcd".format(str(value).replace(".", ""))
        code = run([py, HERE / "run_octomap.py", session,
                    "--benchmark-dir", session / (source + "_clean"),
                    "--map", session / "map" / "map_scan_clean.pcd",
                    "--max-range", value, "--stride", 1, "--output", name],
                   "maxRange {} m".format(value))
        if code != 0:
            die("掃引", "run_octomap.py が落ちた（maxRange {}）".format(value))
    run([py, HERE / "eval_removal.py", session]
        + ["map_octomap_sweep{}.pcd".format(str(v).replace(".", "")) for v in values]
        + ["--reference", "map_scan_clean.pcd"], "掃引の評価（eval_removal.py）")
    print("\n\033[1m[停止] 掃引はここまで。値を選んで --max-range で当て直す\033[0m")
    print("  見るところ: **歩いた体積は消えているほどよい / 遠方 4 m 以上は残っているほどよい**")
    raise SystemExit(0)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("session_dir", type=Path)
    p.add_argument("--source", default=DEFAULT_SOURCE, help="姿勢つき PCD の置き場")
    p.add_argument("--radius", type=float, default=DEFAULT_RADIUS,
                   help="近距離除去の半径[m]。**部屋の広さに依存する**")
    p.add_argument("--max-range", type=float, default=DEFAULT_MAX_RANGE,
                   help="OctoMap の maxRange[m]。**--sweep で決めてから当てる**")
    p.add_argument("--sweep", type=float, nargs="+", metavar="M",
                   help="maxRange を掃引して指標を並べ、止まる")
    p.add_argument("--band-lower", type=float, default=None,
                   help="障害物帯の下限[m]。省略で床のうねりから算出する")
    p.add_argument("--floor-margin", type=float, default=DEFAULT_FLOOR_MARGIN,
                   help="床のうねりの上に積む余裕[m]")
    p.add_argument("--skip-sim", action="store_true", help="③（3D）を飛ばす")
    args = p.parse_args()

    try:
        import octomap  # noqa: F401
    except ImportError:
        die("前提", "octomap が無い。**G1_Hackason/.venv の python で走らせること**\n"
                    "  例: ../../G1_Hackason/.venv/bin/python quickstart/clean_map.py ...")

    session = args.session_dir.resolve()
    py = sys.executable
    src = session / args.source / "pcd"
    if not src.is_dir() or not any(src.glob("*.pcd")):
        die("前提", "姿勢つきスキャンが無い: {}\n"
                    "  先に: quickstart/export_benchmark_data.py {} --stride 5 "
                    "--out-name {}".format(src, session, args.source))
    traj = session / "mola_floor0" / "traj.txt"
    if not traj.exists():
        die("前提", "軌跡が無い: {}\n  先に MOLA-LO で地図と軌跡を作る"
                    "（run_mola_lo.sh）".format(traj))

    # ── ① 追従者を落とす ───────────────────────────────────
    if run([py, HERE / "filter_scans_near.py", session,
            "--source", args.source, "--out", args.source + "_clean",
            "--radius", args.radius, "--min-range", 0.0,
            "--height", 0.15, 2.0, "--name", "map_scan_clean.pcd"],
           "① 近距離除去（半径 {} m）".format(args.radius)) != 0:
        die("①-1 近距離除去", "filter_scans_near.py が落ちた")

    if args.sweep:
        sweep_max_range(py, session, args.source, args.sweep)

    if run([py, HERE / "run_octomap.py", session,
            "--benchmark-dir", session / (args.source + "_clean"),
            "--map", session / "map" / "map_scan_clean.pcd",
            "--max-range", args.max_range, "--stride", 1,
            "--output", "map_octomap_clean.pcd"],
           "① 可視性除去（OctoMap maxRange {} m）".format(args.max_range)) != 0:
        die("①-2 可視性除去", "run_octomap.py が落ちた")

    run([py, HERE / "eval_removal.py", session, "map_octomap_clean.pcd",
         "--reference", "map_scan_clean.pcd"], "① の評価（eval_removal.py）")

    # ── 床を測って帯の下限を出す ────────────────────────────
    cloud = read_pcd(session / "map" / "map_octomap_clean.pcd").points
    floor, top, derived = measure_floor_band(cloud, args.floor_margin)
    lower = args.band_lower if args.band_lower is not None else derived
    print("\n\033[1m── 床を測る ──\033[0m")
    print("  床 z={:+.3f} m（最頻ビン）/ うねりの上端 +{:.3f} m"
          "（0.5 m セルの中央値・1 セル {} 点以上・p{:.0f}）".format(
              floor, top, FLOOR_MIN_POINTS, FLOOR_PERCENTILE))
    print("  → 障害物帯の下限 = {:.2f} + {:.2f} = \033[1m{:.2f} m\033[0m".format(
        top, args.floor_margin, derived))
    if args.band_lower is not None and abs(args.band_lower - derived) > 0.02:
        print("  ⚠️ --band-lower {:.2f} を指定した（算出値 {:.2f} と違う）".format(lower, derived))
    configured = nav2_min_obstacle_height()
    if configured is not None and abs(configured - lower) > 0.02:
        print("  ⚠️ \033[1mg1_nav2.yaml の min_obstacle_height が {:.2f} で食い違う。\033[0m"
              "静的地図の帯だけ低いと**床を撃つ**".format(configured))
        print("     どちらかに揃えること（2026-09-08 にこれで軌跡の 60.4% が塞がれていた）")

    # ── ①-3 持続性で残りを落とす（2D も 3D もこの出力を読む）─────
    # ⚠️ 2026-09-08 まではここが③の中にあり、2D の nav_map だけ掃除が浅かった。
    if run([py, HERE / "filter_persistence.py", session, "map_octomap_clean.pcd",
            "--source", args.source, "--band", lower, BAND_UPPER_3D,
            "--output", "map_octomap_sim.pcd"],
           "①-3 持続性で残りを落とす（✔ 構造残存）") != 0:
        die("①-3 持続性フィルタ",
            "**構造ボクセルを削っている。**--max-range を下げるか --span を短くする\n"
            "  （既定は 最遠 6 m / 時間幅 5 秒。filter_persistence.py の帯の表を読むこと）")

    # ── ② 2D ─────────────────────────────────────────────
    # ⚠️ 入力は①-3 の出力。**sim シーンと同じ点群を読む**（食い違いを作らない）
    # ⚠️ --min-points は pcd_to_mjcf.py の MIN_POINTS と同じ値にする。
    #    1（＝帯の中に 1 点で占有）だと床のうねりが帯の下端をかすめた所が障害物になり、
    #    2026-09-08 の長距離では**それが経路を幅 0.7 m の隙間へ押し込んでいた**。
    if run([py, HERE / "pcd_to_occupancy.py",
            session / "map" / "map_octomap_sim.pcd", session / "map" / "nav_map_clean",
            "--band", lower, BAND_UPPER_2D, "--min-points", MIN_POINTS_2D],
           "② 占有格子（帯 {:.2f}〜{:.2f} m / 帯の中 {} 点以上）".format(
               lower, BAND_UPPER_2D, MIN_POINTS_2D)) != 0:
        die("②-1 占有格子", "pcd_to_occupancy.py が落ちた")

    # 旧 nav_map は 2026-09-11 に map/old/ へ退避した。**黙って比較を落とさない**
    baseline = session / "map" / "old" / "nav_map"
    if not baseline.with_suffix(".yaml").exists():
        baseline = session / "map" / "nav_map"
    if baseline.with_suffix(".yaml").exists():
        print("  比較の基準: {}".format(baseline.with_suffix(".yaml")))
    else:
        print("  ⚠️ 比較の基準（旧 nav_map）が無いので、差分は出さない")
    gate = [py, HERE / "check_map_clearance.py", session / "map" / "nav_map_clean", traj]
    if baseline.with_suffix(".yaml").exists():
        gate += ["--baseline", baseline]
    if run(gate, "✔ ② の合否（check_map_clearance.py）") != 0:
        die("② 軌跡クリアランス",
            "**地図が機体の歩いた道を塞いでいる**（中央値 ≤ robot_radius）。\n"
            "  掃引するもの: --max-range（--sweep で比べる）と --radius（部屋が広いなら上げる）\n"
            "  帯の下限が床に食い込んでいないかも見る（上の「床を測る」の出力）\n"
            "  render_nav_map.py で**壁が残っているかを目で見る**こと")

    if run([py, HERE / "render_nav_map.py", session / "map" / "nav_map_clean", traj]
           + (["--baseline", baseline] if baseline.with_suffix(".yaml").exists() else [])
           + ["--out", "/tmp/nav_map_clean.png"], "② の図（render_nav_map.py）") != 0:
        print("  ⚠️ 図の生成に失敗した（matplotlib が無い？）。合否には影響しない")

    if args.skip_sim:
        print("\n\033[1m[完了] ② まで。③（3D）は --skip-sim で飛ばした\033[0m")
        return 0

    # ── ③ 3D ─────────────────────────────────────────────
    if run([py, HERE / "pcd_to_mjcf.py", session, "map_octomap_sim.pcd"],
           "③ シーン化（pcd_to_mjcf.py）") != 0:
        die("③-2 シーン化", "pcd_to_mjcf.py が落ちた")

    import json
    scenes = json.loads((session / "sim" / "scenes.json").read_text())
    latest = next((v for v in scenes["variants"] if v["variant"] == "octomap_sim"), None)
    print("\n\033[1m── ✔ ③ の合否（偽障害物）──\033[0m")
    if latest is None:
        die("③ 偽障害物", "scenes.json に octomap_sim が無い")
    area = float(latest["phantom_area_m2"])
    print("  偽障害物 {:,} セル / {:.2f} m²（基準 ≤ {:.1f} m²）  {}".format(
        latest["phantom_cells"], area, PHANTOM_AREA_MAX,
        "✔" if area <= PHANTOM_AREA_MAX else "✕"))
    print("  遠方構造 {:,} セル / 床の実測 {:.1f}%".format(
        latest["structure_cells"], latest["floor_measured_pct"]))
    if area > PHANTOM_AREA_MAX:
        die("③ 偽障害物",
            "**機体が通った場所に障害物が残っている**（追従者の焼き込み）。\n"
            "  掃引するもの: --radius を上げる / filter_persistence の --max-range を上げる\n"
            "  ただし構造残存が 100 % を割ったら行き過ぎ")

    print("\n\033[1m[完了]\033[0m")
    print("  2D  {}".format(session / "map" / "nav_map_clean.yaml"))
    print("  3D  {}".format(session / "map" / "map_octomap_sim.pcd"))
    print("      {}".format(session / "sim" / "octomap_sim.npz"))
    print("  図  /tmp/nav_map_clean.png")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
