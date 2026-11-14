"""腕の指令を送った間隔を、記録（runs/<日時>/commands.csv の t 列）から調べる。PC2 の中で動かしたときに 50 Hz が乱れないかを見る。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/send_timing.py _local/button_press/runs/<日時>
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/send_timing.py <フォルダ1> <フォルダ2>   # 並べて比べる（ラボ PC と PC2）

出力: 送った数、間隔の中央値・99% 点・最大 [ms]、遅れた回数（周期の 1.5 倍を超えた間隔）。
dry-run でも commands.csv は残る（送るはずだった時刻）ので、送らずに PC2 の周期を測れる。
"""

from __future__ import annotations

import argparse
import csv
import sys
from pathlib import Path

import numpy as np


def send_intervals_ms(run_dir: Path) -> np.ndarray:
    """commands.csv の t（送った時刻 [s]）の差 [ms]。"""
    with open(run_dir / "commands.csv", encoding="utf-8") as f:
        t = np.array([float(row["t"]) for row in csv.DictReader(f)])
    return np.diff(t) * 1000.0


def summary(dt_ms: np.ndarray, period_ms: float) -> str:
    if dt_ms.size == 0:
        return "送った数 1 以下（間隔が無い）"
    late = int(np.sum(dt_ms > 1.5 * period_ms))
    return (f"送った数 {dt_ms.size + 1}、間隔 中央値 {np.median(dt_ms):.1f} ms・99% 点 {np.percentile(dt_ms, 99):.1f} ms・"
            f"最大 {dt_ms.max():.1f} ms、遅れた回数（{1.5 * period_ms:.0f} ms 超）{late}（{late / dt_ms.size * 100:.2f}%）")


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("runs", nargs="+", type=Path, help="記録のフォルダ（runs/<日時>）")
    p.add_argument("--hz", type=float, default=50.0, help="送る周期 [Hz]（configs/arm.yaml の control_hz）")
    args = p.parse_args()
    for d in args.runs:
        if not (d / "commands.csv").is_file():
            print(f"[timing] ❌ {d}: commands.csv が無い")
            return 1
        print(f"[timing] {d.name}: {summary(send_intervals_ms(d), 1000.0 / args.hz)}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
