#!/usr/bin/env python3
"""MOLA の地図から**重畳の基準地図**（nav_map_ref）を作る。**合否で止まる。**

## なぜ要るのか

重畳（`measure_overlay.py`）の基準は 2026-09-11 まで **2026-09-06 に作った旧 `nav_map`**
（`map/old/`）だった。あれは OctoMap パイプライン由来で、**MOLA が測位に使う地図とは
別系統**である。地図を作り直すと基準だけ 09-06 に取り残され、しかも
**落ちずに数字だけずれる**（測定器が壊れたことに気づけない）。

ここでは同じ `mola_floor0/` の SLAM 結果から基準を作る。地図を作り直したら
基準も一緒に作り直せばよく、旧 `nav_map` を持ち回る必要が無くなる（計画書 §5 段 13）。

## ⚠️ `map.mm` ではなく `map_full.mm` を読む

MOLA が測位で見るのは `map.mm` だが、**その層 `mola::HashedVoxelPointCloud` は
点として取り出せない**。2026-09-12 に既製の道具を両方試して確かめた:

| 試したもの | 結果 |
|---|---|
| `mm2txt --layer localmap` | `... cannot be converted into a point cloud for exporting in TXT format` |
| `mm-filter` ＋ `FilterMerge`（`input_pointcloud_layer`） | `FilterMerge::filter()` で例外（`CPointsMap` を継承していない） |

`map_full.mm` は**同じ `mola_floor0/` の同じ SLAM 結果**を `sm2mm` で全域に起こしたもの
（`run_mola_lo.sh`）。**座標系が同じであることは推測ではなく測ってある**:
MOLA が `map.mm` を見て出した live の姿勢で点群を起こすと、
この地図に対する最良のずらしが **(0.00, 0.00) ちょうど**になる（旧 `nav_map` は (−0.05, −0.05)）。

## 実測（2026-09-12。静止の対照 `still_20260912T085449_r1`）

| 基準 | 占有セル | 重畳 | 最良のずらし | +0.1 m | +0.2 m |
|---|---|---|---|---|---|
| 旧 `nav_map` | 24,370 | 70.1 % | dx −0.05 / dy −0.05 | −2.9 pt | −4.8 pt |
| **`nav_map_ref`** | **20,466** | **73.8 %** | **dx 0.00 / dy 0.00** | **−10.2 pt** | **−21.5 pt** |

→ **占有セルは 16 % 少ないのにスコアは高く、ずれへの感度は 3.5 倍。**
測定器としては厳密に上（緩くなって点が取れているのではない）。

記録 6 本での比較（旧 → 新）:

| 記録 | 旧 | 新 |
|---|---|---|
| 静止 `stage_20260910T084053` | 78.2 % | 77.5 % |
| 静止 `still_20260912T085449_r2` / `_r3` | 70.1 / 70.2 % | 73.8 / 74.0 % |
| 歩行 `stage_20260910T182639_r1/r2/r3` | 31.1 / 27.6 / 27.8 % | 38.9 / 22.4 / 34.6 % |

静止は同等以上、歩行は低いまま ＝ **すべりを見逃すようにはなっていない。**

## ⚠️ これは**測定器であって走る地図ではない**

掃除を一切していないので、追従者も机も焼き込まれたままである。
`check_map_clearance.py` で測った軌跡クリアランス（2026-09-12）:

| 地図 | 中央値 | 0.30 m 未満 | 用途 |
|---|---|---|---|
| `nav_map_clean` | 0.781 m | 1.9 % | **Nav2 が走る**（nav_stack.sh の既定） |
| 旧 `nav_map` | 0.224 m | 60.4 % | （退避済み） |
| **`nav_map_ref`** | **0.100 m** | **78.9 %** | **重畳の基準だけ**。走らせると自分の道を塞ぐ |

基準地図は「地図が知っている物」を漏らさず持っている必要があるので、
**掃除してはいけない**（掃除した clean は静止でも 43.9% が天井になる）。
逆に走る地図は掃除しないと自分の軌跡を塞ぐ。**役割が逆なので 1 つにまとめられない。**
`nav_stack.sh` は `G1_NAV_MAP` に `nav_map_ref` を指すと落ちる。

## つまみ（掃引して決めた。動かすなら測り直すこと）

- `MIN_POINTS = 1` — 2 / 3 / 5 / 10 では 69.9 / 62.9 / 57.2 / 45.5 % と単調に下がる
- `BAND = (0.15, 1.80)` — 下限を 0.10〜0.30 で振ると 74.4 → 72.1 %、
  **ずれへの感度は 20 pt でほぼ不変**。旧 `nav_map` と同じ値に揃えて継続性を取った

## 使い方

    # コンテナ（rviz）が上がっていること。mm2txt はそこにしか無い
    Navigation/.venv/bin/python quickstart/make_ref_map.py \\
        --verify runs/still_20260912T085449_r1/bag

    ... --session <ID>              # 既定は 20260906T135940_UiS_room_v3
    ... --txt <既に出した txt>       # コンテナを使わない
    ... --no-verify                 # 裏取りを飛ばす（**勧めない**）

合否に落ちたら `<出力>.reject.pgm` に残して終了する。**良い地図を置き換えない。**
"""
from __future__ import annotations

import argparse
import subprocess
import sys
from pathlib import Path

import numpy as np
import pandas as pd

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from measure_overlay import read_map, world_points, count_hits   # noqa: E402
from pcd_to_occupancy import build_grid, write_map               # noqa: E402

REAL = HERE.parent                       # .../Mapping/real
CONTAINER = "rviz"
WORK = "/work/G1_Hackason/Mapping/real"
SESSION = "20260906T135940_UiS_room_v3"
PLUGIN = "libmola_metric_maps.so"        # `mola::*` の層はこれを積まないと読めない

RESOLUTION = 0.10
BAND = (0.15, 1.80)
MIN_POINTS = 1
DILATE_FREE = 0.30

# ── 合否 ──────────────────────────────────────────────────────────────
# ① ずらし検査。(0,0) が四方の ±SHIFT_CHECK_M より**強い**こと。
#    自己較正なので部屋にも地図にも依存しない。座標系の取り違えはここで必ず落ちる
SHIFT_CHECK_M = 0.20
# ② 絶対値の下限。静止の対照 4 本の最小は 73.8 %（2 日ぶん）。
#    0.2 m ずれた地図は 53.6 % に落ちるので、その間に線を引く
VERIFY_MIN_PCT = 60.0


def export_points(mm: str, layer: str, out_dir: Path, force: bool) -> Path:
    """コンテナの `mm2txt` で .mm の 1 層を x y z のテキストに落とす。"""
    txt = out_dir / "{}_{}.txt".format(Path(mm).stem, layer)
    if txt.exists() and not force:
        print("既にある txt を使う（作り直すなら --force）: {}".format(txt))
        return txt
    remote_dir = "{}/{}".format(WORK, txt.parent.relative_to(REAL))
    cmd = ("source /opt/ros/humble/setup.bash && mkdir -p {d} && cd {d} && "
           "mm2txt --load-plugins {p} --layer {l} --export-fields x,y,z {mm}").format(
        d=remote_dir, p=PLUGIN, l=layer, mm=mm)
    print("コンテナで mm2txt: {} の層 '{}'".format(mm, layer))
    r = subprocess.run(["docker", "exec", "-u", "ubuntu", CONTAINER, "bash", "-c", cmd],
                       capture_output=True, text=True)
    if r.returncode != 0 or not txt.exists():
        raise SystemExit("mm2txt が落ちた（コンテナ '{}' は上がっているか）:\n{}\n{}".format(
            CONTAINER, r.stdout[-2000:], r.stderr[-2000:]))
    return txt


def verify(bag: Path, yaml_path: Path) -> bool:
    """基準地図として使えるかを 2 つの合否で判定する。"""
    occ, res, ox, oy = read_map(yaml_path)
    print("\n── 裏取り: {} ─────────────────".format(bag.parent.name))
    xy, used, z_lo, z_hi = world_points(bag)
    print("  帯 {:+.2f} .. {:+.2f} m / スキャン {} 枚 / 点 {:,}".format(z_lo, z_hi, used, len(xy)))

    def pct(dx: float, dy: float) -> float:
        hit, tot = count_hits(xy, occ, res, ox, oy, dx, dy)
        return 100.0 * hit / tot if tot else 0.0

    base = pct(0.0, 0.0)
    d = SHIFT_CHECK_M
    shifted = {"+x": pct(d, 0.0), "-x": pct(-d, 0.0), "+y": pct(0.0, d), "-y": pct(0.0, -d)}

    print("\n  ① ずらし検査（±{:.2f} m）".format(d))
    print("     ずらさない  {:5.1f} %".format(base))
    for k, v in shifted.items():
        print("     {:>2}          {:5.1f} %  ({:+.1f} pt)".format(k, v, v - base))
    worst = max(shifted.values())
    ok_shift = base > worst
    print("     -> {}（最良は{}）".format(
        "OK" if ok_shift else "NG", "ずらさないとき" if ok_shift else "ずらした先"))

    ok_abs = base >= VERIFY_MIN_PCT
    print("\n  ② 絶対値  {:5.1f} % >= {:.1f} % -> {}".format(
        base, VERIFY_MIN_PCT, "OK" if ok_abs else "NG"))

    if not ok_shift:
        print("\n  ずらした先の方が高い ＝ **地図と姿勢の座標系がずれている。**", file=sys.stderr)
        print("  mola_floor0 の中身（map_full.mm と map.mm が同じ run か）を疑う", file=sys.stderr)
    if not ok_abs:
        print("\n  低すぎる。静止の記録で測っているか／帯と足切りが効きすぎていないかを見る",
              file=sys.stderr)
    return ok_shift and ok_abs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--session", default=SESSION)
    ap.add_argument("--layer", default="raw", help="map_full.mm の点群の層名")
    ap.add_argument("--out", type=Path, default=None,
                    help="拡張子なしの出力名（既定 runs/<S>/map/nav_map_ref）")
    ap.add_argument("--txt", type=Path, default=None, help="既に出した x y z のテキスト")
    ap.add_argument("--verify", type=Path, default=None,
                    help="**静止の対照**の bag。ここで合否を出す")
    ap.add_argument("--no-verify", action="store_true", help="裏取りを飛ばす（勧めない）")
    ap.add_argument("--force", action="store_true", help="txt を作り直す")
    ap.add_argument("--min-points", type=int, default=MIN_POINTS)
    ap.add_argument("--band", type=float, nargs=2, default=BAND)
    a = ap.parse_args()

    if not a.verify and not a.no_verify:
        raise SystemExit("--verify <静止の bag> か --no-verify のどちらかを明示すること。"
                         "\n**裏取りしていない基準地図は、落ちずに数字だけずらす。**")

    session_dir = REAL / "runs" / a.session
    out = a.out or (session_dir / "map" / "nav_map_ref")
    txt = a.txt or export_points(
        "{}/runs/{}/mola_floor0/map_full.mm".format(WORK, a.session),
        a.layer, session_dir / "map" / "ref", a.force)

    print("\n読み込み: {}".format(txt))
    df = pd.read_csv(txt, sep=r"\s+")
    points = df[["x", "y", "z"]].to_numpy(np.float64)
    image, lo = build_grid(points, RESOLUTION, tuple(a.band), DILATE_FREE, a.min_points)

    # ⚠️ **先に本番の名前で書かない。**落ちた地図で良い基準を上書きすると、
    # 以後の数字が全部ずれたまま気づけない
    tmp = out.with_name(out.name + ".reject")
    pgm_path, yaml_path = write_map(image, lo, RESOLUTION, tmp)

    if not a.no_verify:
        if not verify(a.verify, yaml_path):
            print("\n[NG] 合否に落ちた。{} と {} に残した".format(pgm_path, yaml_path),
                  file=sys.stderr)
            return 1
    else:
        print("\n⚠️ --no-verify。この地図はまだ何も裏取りしていない")

    final_pgm, final_yaml = out.with_suffix(".pgm"), out.with_suffix(".yaml")
    pgm_path.replace(final_pgm)
    yaml_path.write_text(yaml_path.read_text().replace(pgm_path.name, final_pgm.name))
    yaml_path.replace(final_yaml)
    print("\n[OK] {} と {}".format(final_pgm, final_yaml))
    print("     run_stage.sh の既定の基準地図はこれ（G1_OVERLAY_REF_MAP で変えられる）")
    print("     ⚠️ **走る地図ではない。**掃除していないので軌跡クリアランス中央値は 0.10 m。")
    print("        Nav2 の静的レイヤは nav_map_clean のまま（nav_stack.sh の G1_NAV_MAP）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
