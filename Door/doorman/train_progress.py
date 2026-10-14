"""DoorMan の学習ログ（train.log の枠つきの表）から、反復ごとの数字を CSV に抜き出す。

標準ライブラリだけで動く（OMEN の system python3 でも、Mac でも）。

使い方:
    python3 train_progress.py <train.log> [--csv out.csv] [--every 50]

出力（標準出力）: --every 反復おきの要約の行と、最後の行・区間の進みの最大値。
NaN が出たら「NAN」と出す（報酬が NaN になると学習は黙って壊れる）。
"""

import argparse
import csv
import math
import re
import sys
from pathlib import Path

ANSI = re.compile(r"\x1b\[[0-9;]*m")
ITER = re.compile(r"Learning iteration (\d+)")
KV = re.compile(r"^\s*([A-Za-z_/][A-Za-z0-9_ /]*?):\s+(-?[0-9.]+(?:e[-+]?\d+)?|nan|-?inf)\b")
STEPS = re.compile(r"Computation:\s+(\d+) steps/s")

# 要約に出す列（無い列は空のまま）
SUMMARY_KEYS = [
    "Mean rewards",
    "Mean length",
    "Env/average_stage_reached",
    "Env/average_goal_reached",
    "Env/average_last_stage_goal_reached",
    "Mean episode rew_stage",
    "Mean episode rew_walk_to_door",
    "Mean episode rew_push_door_hinge",
]


def parse(path: Path) -> list[dict]:
    rows: list[dict] = []
    row: dict | None = None
    with path.open(encoding="utf-8", errors="replace") as f:
        for raw in f:
            line = ANSI.sub("", raw).strip().strip("│").strip()
            m = ITER.search(line)
            if m:
                if row is not None:
                    rows.append(row)
                row = {"iteration": int(m.group(1))}
                continue
            if row is None:
                continue
            m = STEPS.search(line)
            if m:
                row["steps_per_s"] = float(m.group(1))
                continue
            m = KV.match(line)
            if m:
                key, value = m.group(1).strip(), m.group(2)
                try:
                    row[key] = float(value)
                except ValueError:
                    row[key] = float("nan")
    if row is not None:
        rows.append(row)
    return rows


def has_nan(row: dict) -> bool:
    return any(isinstance(v, float) and math.isnan(v) for v in row.values())


def fmt(row: dict) -> str:
    parts = [f"it={row['iteration']}", f"sps={row.get('steps_per_s', float('nan')):.0f}"]
    for key in SUMMARY_KEYS:
        if key in row:
            short = key.replace("Env/average_", "").replace("Mean episode rew_", "r_").replace("Mean ", "")
            parts.append(f"{short}={row[key]:.4g}")
    if has_nan(row):
        parts.append("NAN")
    return " ".join(parts)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path)
    ap.add_argument("--csv", type=Path, default=None)
    ap.add_argument("--every", type=int, default=50)
    args = ap.parse_args()

    rows = parse(args.log)
    if not rows:
        print(f"[train_progress] 反復の表がまだ無い: {args.log}")
        sys.exit(1)

    if args.csv is not None:
        keys: list[str] = []
        for r in rows:
            keys.extend(k for k in r if k not in keys)
        with args.csv.open("w", newline="", encoding="utf-8") as f:
            w = csv.DictWriter(f, fieldnames=keys)
            w.writeheader()
            w.writerows(rows)

    for r in rows:
        if r["iteration"] % args.every == 0:
            print(fmt(r))
    last = rows[-1]
    print("LAST " + fmt(last))
    best = max(rows, key=lambda r: r.get("Env/average_stage_reached", -1.0))
    print(
        f"MAX stage_reached={best.get('Env/average_stage_reached', float('nan')):.4f} "
        f"at it={best['iteration']}  nan_rows={sum(has_nan(r) for r in rows)}"
    )


if __name__ == "__main__":
    main()
