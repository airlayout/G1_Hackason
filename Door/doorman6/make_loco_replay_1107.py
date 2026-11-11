"""11-07（計画 docs/plan/2026-11-06-g1-door-depth-rl.md の 7.6・案 A）: シミュレータの脚（HOMIE＋純正に似せる層）に流す
歩きの指令の列を作る（パッチ 0095 の env.config.loco_cmd_replay.path に渡す npz）。

列は 2 種類:
  sweep_<軸>_<大きさ>: 止まる 1 秒 → 負の向き 3 秒 → 止まる 2 秒 → 正の向き 3 秒 → 止まる 2 秒。前後は扉が前にあるので後ろから先に動く。
      大きさは前後・横 0.05・0.1・0.15・0.2・0.3・0.4・0.5 m/s、旋回 0.1・0.2・0.3・0.4・0.5 rad/s（実機の走らせ役は ±0.5 で切る）
  real_<記録の名前>: 実機の door_run_stock の DOOR・押す試しで送った指令（Door/real/ana_stock_gain_1106.segments）を
      0.02 秒ごとに直したもの（零次ホールド）。比べるための実機の odom も同じ格子で入れる（real_odom・NaN で埋める）
どの列も最後に 1 秒止まり、一番長い列に 0 で揃える。

使い方（OMEN・doorman6 の Python）:
  python make_loco_replay_1107.py --out ~/work_inoue/doorman/loco_replay_1107.npz ~/work_inoue/real_data/20261106/door_real/logs/door_stock_*.npz
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path
from typing import List, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parent.parent / "real"))
import ana_stock_gain_1106 as asg  # noqa: E402

DT = 0.02
MAG = {0: (0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5), 1: (0.05, 0.1, 0.15, 0.2, 0.3, 0.4, 0.5), 2: (0.1, 0.2, 0.3, 0.4, 0.5)}
AXIS = ("vx", "vy", "wz")


def sweep(ax: int, mag: float) -> np.ndarray:
    parts = [(1.0, 0.0), (3.0, -mag), (2.0, 0.0), (3.0, mag), (2.0, 0.0)]
    rows = []
    for sec, val in parts:
        n = int(round(sec / DT))
        r = np.zeros((n, 3), dtype=np.float32)
        r[:, ax] = val
        rows.append(r)
    return np.concatenate(rows)


def on_grid(t: np.ndarray, x: np.ndarray) -> np.ndarray:
    """不揃いな周期の列を 0.02 秒の格子へ（各格子の時刻より前の最後の値）"""
    grid = np.arange(0.0, t[-1] + 1e-9, DT)
    idx = np.clip(np.searchsorted(t, grid, side="right") - 1, 0, len(t) - 1)
    return x[idx]


def build(paths: List[str]) -> Tuple[np.ndarray, List[str], np.ndarray]:
    seqs, names, odoms = [], [], []
    for ax in range(3):
        for m in MAG[ax]:
            seqs.append(sweep(ax, m))
            names.append(f"sweep_{AXIS[ax]}_{m:.2f}")
            odoms.append(None)
    for seg in asg.segments(paths):
        seqs.append(on_grid(seg["t"], seg["cmd"]).astype(np.float32))
        names.append(f"real_{seg['name']}")
        odoms.append(on_grid(seg["t"], seg["odom"]))
    tail = int(round(1.0 / DT))
    T = max(len(s) for s in seqs) + tail
    cmd = np.zeros((len(seqs), T, 3), dtype=np.float32)
    real_odom = np.full((len(seqs), T, 3), np.nan, dtype=np.float32)
    for i, (s, o) in enumerate(zip(seqs, odoms)):
        cmd[i, : len(s)] = s
        if o is not None:
            real_odom[i, : len(o)] = o
    return cmd, names, real_odom


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("records", nargs="+", help="実機の door_stock_*.npz")
    a = ap.parse_args(argv)
    cmd, names, real_odom = build(a.records)
    n_real = sum(n.startswith("real_") for n in names)
    np.savez(a.out, cmd=cmd, names=np.array(names), real_odom=real_odom, dt=np.array(DT))
    print(f"[replay] {a.out}: 列 {len(names)} 本（掃引 {len(names) - n_real}・実機 {n_real}）・長さ {cmd.shape[1] * DT:.1f} 秒", flush=True)
    for n in names:
        if n.startswith("real_"):
            print(f"   {n}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
