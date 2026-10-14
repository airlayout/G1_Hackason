"""PD のクローザーの扉と実物の形の扉（0037、door_en1154_g<k>_shaped）で、同じ方策の評価を等級ごとに並べる。

sweep_tasks.sh の results.tsv を 1 つ以上渡す（同じタスクが複数あれば後のものを使う）。各タスクの記録の summary.json の
per_env（各台の最初の 1 回の最大の扉の角度・届いた区間）から数える。

見るもの（2026-10-13 の見込み）: 実物の形はラッチを外した直後が PD の 1.4〜1.7 倍重く、60〜70° では 0.7〜1.2 倍。
→ 「10° まで」が減り、「30° を越えた回のうち 60°・70° まで行く割合」が増えるなら見込みどおり。

使い方（OMEN の ~/work_inoue/doorman で）:
    python compare_closer_shape.py sweep/<日時>/results.tsv [sweep/<日時>/results.tsv ...]
"""

import csv
import json
import sys
from pathlib import Path

ANGLES_DEG = (10, 30, 60, 70)
THROUGH_STAGE = 5  # 区間 4→5 で通り抜けの区間へ（扉 door_through_angle_deg 以上・面を越える・レバーを離す）
STOP_BINS = ((0, 10), (10, 30), (30, 60), (60, 70), (70, 999))


def read_tasks(tsv_paths: list[str]) -> dict[str, Path]:
    tasks: dict[str, Path] = {}
    for path in tsv_paths:
        with open(path, encoding="utf-8") as f:
            for row in csv.DictReader(f, delimiter="\t"):
                if row.get("record") and row["record"] != "-":
                    tasks[row["task"]] = Path(row["record"])
    return tasks


def task_order(name: str) -> tuple:
    # door_en1154_g3 と door_en1154_g3_shaped を隣に並べる
    base = name.removesuffix("_shaped")
    return (base, name.endswith("_shaped"))


def summarize(record: Path) -> dict:
    summary = json.loads((record / "summary.json").read_text(encoding="utf-8"))
    per_env = summary["per_env"]
    n = len(per_env)
    angles = [e["max_hinge_deg"] for e in per_env]
    over = {a: sum(x >= a for x in angles) for a in ANGLES_DEG}
    stops = [sum(lo <= x < hi for x in angles) for lo, hi in STOP_BINS]
    return {
        "n": n,
        "over": over,
        "stage5": sum(e["max_stage"] >= THROUGH_STAGE for e in per_env),
        "through": summary.get("success_through", 0),
        "stops": stops,
        # 30° を越えた回のうち 60° まで行った割合（後半の軽さが効いたか）
        "60_given_30": over[60] / over[30] if over[30] else float("nan"),
    }


def main() -> int:
    if len(sys.argv) < 2:
        sys.exit(__doc__)
    tasks = read_tasks(sys.argv[1:])
    if not tasks:
        print("[compare] results.tsv に記録のあるタスクが無い")
        return 2
    header = "タスク                         回   " + "  ".join(f"{a:>3d}°以上" for a in ANGLES_DEG)
    print(header + "  区間5へ  通り抜け  30°→60°")
    rows = {}
    for name in sorted(tasks, key=task_order):
        try:
            rows[name] = s = summarize(tasks[name])
        except (FileNotFoundError, KeyError, json.JSONDecodeError) as e:
            print(f"{name:30s} 読めない: {e}")
            continue
        cells = "  ".join(f"{s['over'][a]:>4d}/{s['n']:<3d}" for a in ANGLES_DEG)
        print(f"{name:30s} {s['n']:4d}  {cells}  {s['stage5']:>4d}    {s['through']:>4d}     {s['60_given_30']:5.0%}")
    print()
    print("止まった角度（各台の最大の扉の角度）: " + "  ".join(f"{lo}〜{hi if hi < 999 else ''}°" for lo, hi in STOP_BINS))
    for name, s in rows.items():
        print(f"{name:30s} " + "  ".join(f"{c:>6d}" for c in s["stops"]))
    return 0


if __name__ == "__main__":
    sys.exit(main())
