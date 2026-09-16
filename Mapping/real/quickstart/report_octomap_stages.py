#!/usr/bin/env python3
"""`verify_octomap_seed.sh` が落とした `/projected_map` の 3 段を並べて合否を出す。

3 段はそれぞれ別のことを確かめている:

| 段 | いつ落としたか | 何が分かるか |
|---|---|---|
| `before` | 種を読んだ直後（点群は無し） | 種が `/projected_map` に出ているか |
| `during` | 点群が流れている間 | 育ちながら出し直されているか |
| `after` | 点群が止まった後に購読 | **latched かどうか**（後から来た購読者が地図を得られるか）|

⚠️ `latch:=false` では `before` と `after` が「来なかった」になるのが**正しい**
（VOLATILE ＝履歴なし・購読者が居ないと 2D 投影を計算すらしない）。
Nav2 の静的レイヤは点群が流れ始めてから最初の 1 通で地図を得るので実害は無いが、
**`map_subscribe_transient_local: true` のままだと QoS が噛み合わない**ので
段 5 で `false` に変えること。

    G1_Hackason/.venv/bin/python quickstart/report_octomap_stages.py \\
        runs/<id>/map/stage2 --latch true
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_map_clearance import load_map  # noqa: E402

STAGES = (("種を読んだ直後", "before"),
          ("点群が流れている間", "during"),
          ("止まった後に購読", "after"))
KEEP_RATIO = 0.70        # 種の占有セルがこれ以上残っていること


def read_stage(base: Path, name: str) -> "dict | None":
    path = base / name / "counts.json"
    if not path.exists():
        return None
    info = json.loads(path.read_text()).get("projected")
    return info if info and info.get("received") else None


def occupied_cells(yaml_path: Path) -> "set[tuple[int, int]]":
    """占有セルを世界座標の整数キー（0.1 m 格子）の集合にする。

    格子の原点が違う地図どうしを比べられるようにする。`octomap` の格子は
    世界原点から解像度の整数倍に固定されているので、同じ木から出た投影なら一致する。
    """
    grid = load_map(yaml_path)
    rows, cols = np.nonzero(grid.occupied)
    x = grid.origin[0] + (cols + 0.5) * grid.resolution
    y = grid.origin[1] + (rows + 0.5) * grid.resolution
    keys = np.floor(np.c_[x, y] / grid.resolution).astype(np.int64)
    return set(map(tuple, keys.tolist()))


def retention(seed_yaml: Path, grown_yaml: Path) -> "dict[str, int]":
    """種の占有セルが、育った後もどれだけ残っているかを数える。

    ⚠️ **合計のセル数では分からない。**新しく現れたセルと消えたセルが打ち消し合う。
    段 6 の「幽霊が消える」も「構造が消える」も、ここを見ないと区別できない。
    """
    seed, grown = occupied_cells(seed_yaml), occupied_cells(grown_yaml)
    return {"seed": len(seed), "grown": len(grown), "kept": len(seed & grown),
            "lost": len(seed - grown), "added": len(grown - seed)}


def occupied(info: dict) -> int:
    return info["lethal_99_100"] + info["occupied_65_98"]


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("stage_dir", type=Path, help="before/ during/ after/ を含むディレクトリ")
    p.add_argument("--latch", default="true", help="octomap_server に渡した latch の値")
    p.add_argument("--seed-map", type=Path, default=None,
                   help="種を 2D に落とした .yaml（octomap_project_2d.py の出力）。"
                        "渡すとセル単位の残存率を出す。latch=false で before が"
                        "取れないときはこれが要る")
    a = p.parse_args()

    found = {name: read_stage(a.stage_dir, name) for _, name in STAGES}

    print("{:<26}{:>12}{:>12}{:>12}{:>8}".format("", "占有", "空き", "未知", "受信"))
    for label, name in STAGES:
        info = found[name]
        if info is None:
            print("{:<26}{:>12}".format(label, "来なかった"))
            continue
        print("{:<26}{:>12,}{:>12,}{:>12,}{:>8}".format(
            label, occupied(info), info["free"], info["unknown"], info["messages"]))

    grown = found["during"] or found["after"]
    if grown is None:
        print("\n  NG   /projected_map が 1 通も取れなかった。"
              "latch と --volatile の組み合わせを疑う")
        return 1

    seed = found["before"]
    if seed is None:
        print(f"\n  ※ latch={a.latch} では「種を読んだ直後」が取れないのが正しい"
              "（VOLATILE ＝履歴なし。購読者が居ないと 2D 投影を計算しない）。")
        checks = [("点群が流れている間に /projected_map が来た", found["during"] is not None)]
    else:
        checks = [
            ("格子の大きさが変わっていない", seed["size"] == grown["size"]),
            ("原点が変わっていない", seed["origin"] == grown["origin"]),
            (f"占有セルが種の {KEEP_RATIO:.0%} 以上残っている（種が消えていない）",
             occupied(grown) >= occupied(seed) * KEEP_RATIO),
            ("空きセルが増えた（走った所が空きになった）", grown["free"] > seed["free"]),
            ("未知セルが減った（地図が育った）", grown["unknown"] < seed["unknown"]),
        ]
        if found["after"] is not None:
            checks.append(("止まった後に購読しても地図が来る（latched）", True))

    # ── セル単位の残存率。合計のセル数では現れたセルと消えたセルが打ち消し合う ──
    seed_map = a.seed_map or (a.stage_dir / "before" / "projected.yaml"
                              if (a.stage_dir / "before" / "projected.yaml").exists() else None)
    grown_stage = "during" if found["during"] is not None else "after"
    grown_map = a.stage_dir / grown_stage / "projected.yaml"
    if seed_map is not None and grown_map.exists():
        counts = retention(seed_map, grown_map)
        kept_pct = 100.0 * counts["kept"] / max(counts["seed"], 1)
        print("\nセル単位の残存（種 {} -> {}）".format(seed_map.name, grown_stage))
        print("  種の占有 {seed:,} / 残った {kept:,}（{pct:.1f} %）/ "
              "消えた {lost:,} / 新しく現れた {added:,}".format(pct=kept_pct, **counts))
        checks.append(("種の占有セルが {:.0%} 以上その場に残っている".format(KEEP_RATIO),
                       kept_pct >= KEEP_RATIO * 100))

    print()
    for text, ok in checks:
        print("  {} {}".format("OK  " if ok else "NG  ", text))
    return 0 if all(ok for _, ok in checks) else 1


if __name__ == "__main__":
    raise SystemExit(main())
