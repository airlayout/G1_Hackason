"""L5 の準備（2026-10-23）: 巻き尺で測った pelvis の位置と、ライダーの推定器の記録から、取付（pelvis から見たライダーの水平の位置）を出す。

なぜ: 取付をその場で回って解く方法（lidar_offset_from_turns.py）は、回るたびに足が流れるので 2〜3 cm しか決まらない（10-21 の 04 で
1 区間ずつ抜くと 3〜5 cm 動いた）。生徒は最初の値の 2 cm の片寄りで 90.6 → 81.2% に落ちる（L3 の start2）。
機体が HOMIE で立っている間（STAND）に、巻き尺で pelvis（両足の間の中心）を測れば、ライダーとの差がそのまま取付になる。
頭の休む角度は 1 日の中ではほぼ動かない（10-21: 止まるたびに ±0.6°）ので、その日の最初に 1 回測ればよい。

手順（PC2・機体は STAND で立ったまま）:
  1. 巻き尺: 扉の板の廊下側の面から両足の間の中心まで（手前は負に直す前の距離 d）、開口の中心から左右（扉を向いて左が正）
  2. ./lidar_stop.sh で推定器を止める（記録 logs/lidar_node_<日時>.npz が書かれる）
  3. python3 lidar_offset_from_tape.py logs/lidar_node_<日時>.npz --tape-dist <d> --tape-left <左> --thickness <板の厚み>
  → 出た CENTER を run_door_lidar.sh に渡して、起こし直す

扉の座標の x は板の厚みの真ん中が 0（廊下側の面は x = −厚み/2）なので、pelvis の x = −厚み/2 − d。
"""
from __future__ import annotations

import argparse
import math
from pathlib import Path

import numpy as np


def center_from_tape(lidar_xy: np.ndarray, lidar_yaw: float, pelvis_xy: np.ndarray) -> np.ndarray:
    """扉の座標のライダーと pelvis の位置 → pelvis の向き（≈ライダーの向き。STAND では腰の yaw はほぼ 0）で見たライダー（前・左）"""
    d = lidar_xy - pelvis_xy
    c, s = math.cos(lidar_yaw), math.sin(lidar_yaw)
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("node_log", type=Path, help="door_lidar_node.py の記録（logs/lidar_node_*.npz）")
    ap.add_argument("--tape-dist", type=float, required=True, help="[m] 扉の板の廊下側の面から pelvis（両足の間の中心）まで")
    ap.add_argument("--tape-left", type=float, required=True, help="[m] 開口の中心から pelvis まで、扉を向いて左が正")
    ap.add_argument("--thickness", type=float, required=True, help="[m] 板の厚み（推定器に渡したのと同じ値）")
    ap.add_argument("--last", type=int, default=30, help="記録の最後の何回の受け入れた推定を使うか（止まっている間）")
    a = ap.parse_args()
    z = np.load(a.node_log)
    ok = z["accepted"] > 0
    if ok.sum() < 5:
        raise SystemExit("[tape] 受け入れた推定が足りない（推定器の記録を確かめる）")
    idx = np.nonzero(ok)[0][-a.last:]
    xy = np.array([np.median(z["x"][idx]), np.median(z["y"][idx])])
    yaw = math.atan2(np.median(np.sin(z["yaw"][idx])), np.median(np.cos(z["yaw"][idx])))
    spread = np.array([np.ptp(z["x"][idx]), np.ptp(z["y"][idx])]) * 100
    pelvis = np.array([-a.thickness / 2.0 - a.tape_dist, a.tape_left])
    o = center_from_tape(xy, yaw, pelvis)
    print(f"[tape] ライダー（扉の座標）: x {xy[0]:+.3f} y {xy[1]:+.3f} 向き {math.degrees(yaw):+.1f}°（最後の {len(idx)} 回・幅 {spread[0]:.1f}・{spread[1]:.1f} cm）")
    print(f"[tape] 巻き尺の pelvis: x {pelvis[0]:+.3f} y {pelvis[1]:+.3f}")
    print(f"[tape] pelvis から見たライダー: 前 {o[0]:+.3f}・左 {o[1]:+.3f} m（10-21 の旋回では 前 +0.047〜0.048・左 −0.016〜−0.021）")
    if math.hypot(o[0] - 0.047, o[1] + 0.016) > 0.10:
        print("[tape] ⚠️ 10-21 の旋回の値から 10 cm 以上離れている。巻き尺の向き（手前が正の距離・左が正）と、"
              "推定器が扉を正しく測ったか（幅・ライダーの x）を先に確かめる")
    print(f"[tape] → CENTER=\"{o[0]:.3f} {o[1]:.3f}\"")


if __name__ == "__main__":
    main()
