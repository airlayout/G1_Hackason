#!/usr/bin/env python3
"""記録の任意の時刻から再生するための **MOLA の初期姿勢**を軌跡から作る。

    python3 initial_pose_at.py <traj.txt> --offset 230

## なぜ要るか

`nav_stack.sh` の既定の初期姿勢は**記録の先頭（t=0）用**である。
ところが 20260906T135940_UiS_room_v3 は **t+418 秒以降ずっと開始点に戻って静止**しており、
その開始点は静的地図で ±2.5 m の 42% が占有という**散らかった場所**なので、
そこで経路計画を測ると「機体が囲まれていて経路が引けない」しか出ない。
**開けた場所（例 t+230s で (0.86, +15.93)）から再生したい。**

`ros2 bag play --start-offset` で頭を飛ばすと、初期姿勢が合わなくなって
MOLA が収束しない（roll が 178° 違う地点から ICP を始めることになる）。
そこで**その時刻の姿勢を軌跡から読んで渡す**。

## 座標系（間違えやすい）

軌跡 `traj.txt` は **`livox_frame` の map 系での姿勢**である
（`run_mola_lo.sh` が `--base-link-frame-id livox_frame` で走らせているため）。
一方 `nav_stack.sh` は `ignore_lidar_pose_from_tf:=false` で走らせるので、
MOLA に渡すべきは **`base_link` の姿勢**:

    base_link_in_map = livox_in_map ∘ (base_link -> livox_frame)^-1

`base_link -> livox_frame` は `odom_to_tf.py` が流す静的変換（既定は実測値）。
**この 2 つが噛み合っていないと ICP は正しい場所から始まらない。**

依存は標準ライブラリだけ（numpy も scipy も要らない）。
"""
from __future__ import annotations

import argparse
import math
import sys

# odom_to_tf.py の既定と同じ実測値（2026-09-05）
DEFAULT_LIVOX_RPY_DEG = (178.35, -8.41, -0.72)
DEFAULT_LIVOX_XYZ = (-0.004, 0.016, -0.037)

Mat = list[list[float]]


def rpy_to_mat(roll: float, pitch: float, yaw: float) -> Mat:
    """RPY[rad] -> 回転行列。ZYX（R = Rz(yaw) Ry(pitch) Rx(roll)）。

    odom_to_tf.py の quaternion_from_rpy と MRPT の CPose3D と同じ規約。
    """
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    return [
        [cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr],
        [sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr],
        [-sp, cp * sr, cp * cr],
    ]


def quat_to_mat(x: float, y: float, z: float, w: float) -> Mat:
    n = math.sqrt(x * x + y * y + z * z + w * w)
    if n == 0.0:
        raise ValueError("クォータニオンのノルムが 0")
    x, y, z, w = x / n, y / n, z / n, w / n
    return [
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
    ]


def mat_to_yaw_pitch_roll(R: Mat) -> tuple[float, float, float]:
    """ZYX 分解。戻りは (yaw, pitch, roll)[rad]。pitch は ±90° に収める。"""
    sp = -R[2][0]
    sp = max(-1.0, min(1.0, sp))
    pitch = math.asin(sp)
    if abs(sp) > 1.0 - 1e-9:       # ジンバルロック
        return (0.0, pitch, math.atan2(R[0][1], R[1][1]))
    return (math.atan2(R[1][0], R[0][0]), pitch, math.atan2(R[2][1], R[2][2]))


def compose(Ra: Mat, ta: tuple[float, float, float],
            Rb: Mat, tb: tuple[float, float, float]):
    """(Ra,ta) ∘ (Rb,tb)"""
    R = [[sum(Ra[i][k] * Rb[k][j] for k in range(3)) for j in range(3)] for i in range(3)]
    t = tuple(sum(Ra[i][k] * tb[k] for k in range(3)) + ta[i] for i in range(3))
    return R, t


def invert(R: Mat, t: tuple[float, float, float]):
    Ri = [[R[j][i] for j in range(3)] for i in range(3)]
    ti = tuple(-sum(Ri[i][k] * t[k] for k in range(3)) for i in range(3))
    return Ri, ti


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("traj", help="TUM 形式の軌跡（timestamp x y z qx qy qz qw）")
    p.add_argument("--offset", type=float, required=True,
                   help="記録先頭からの秒数。ros2 bag play --start-offset と同じ値を渡す")
    p.add_argument("--livox-rpy-deg", nargs=3, type=float, default=list(DEFAULT_LIVOX_RPY_DEG),
                   metavar=("R", "P", "Y"), help="base_link -> livox_frame の回転[度]")
    p.add_argument("--livox-xyz", nargs=3, type=float, default=list(DEFAULT_LIVOX_XYZ),
                   metavar=("X", "Y", "Z"), help="同並進[m]")
    p.add_argument("--quiet", action="store_true", help="初期姿勢の 1 行だけ出す（スクリプト用）")
    args = p.parse_args(argv)

    rows = []
    with open(args.traj, encoding="utf-8") as fh:
        for line in fh:
            line = line.strip()
            if not line or line.startswith("#"):
                continue
            parts = line.split()
            if len(parts) < 8:
                continue
            rows.append([float(v) for v in parts[:8]])
    if not rows:
        print(f"軌跡が読めない: {args.traj}", file=sys.stderr)
        return 1

    t0 = rows[0][0]
    target = t0 + args.offset
    # 最も近い姿勢を選ぶ（内挿しない。10 Hz なので最大 50 ms のずれ = 数 cm）
    best = min(rows, key=lambda r: abs(r[0] - target))
    dt = best[0] - target
    if abs(dt) > 1.0:
        print(f"⚠️ offset {args.offset}s に近い姿勢が無い（最寄り {dt:+.2f}s ずれ）。"
              f"軌跡の範囲は 0 〜 {rows[-1][0] - t0:.1f}s", file=sys.stderr)

    _, lx, ly, lz, qx, qy, qz, qw = best
    R_lm, t_lm = quat_to_mat(qx, qy, qz, qw), (lx, ly, lz)

    r, pi, y = (math.radians(v) for v in args.livox_rpy_deg)
    R_bl, t_bl = rpy_to_mat(r, pi, y), tuple(args.livox_xyz)

    R_b, t_b = compose(R_lm, t_lm, *invert(R_bl, t_bl))
    yaw, pitch, roll = (math.degrees(v) for v in mat_to_yaw_pitch_roll(R_b))

    pose = f"[{t_b[0]:.4f}, {t_b[1]:.4f}, {t_b[2]:.4f}, {yaw:.3f}, {pitch:.3f}, {roll:.3f}]"
    if args.quiet:
        print(pose)
        return 0

    lyaw, lpitch, lroll = (math.degrees(v) for v in mat_to_yaw_pitch_roll(R_lm))
    print(f"軌跡: {args.traj}")
    print(f"  offset {args.offset:.1f}s（最寄りの姿勢は {dt:+.3f}s ずれ）")
    print(f"  livox_frame の map 系での姿勢: x={lx:+.4f} y={ly:+.4f} z={lz:+.4f} "
          f"yaw={lyaw:+.3f} pitch={lpitch:+.3f} roll={lroll:+.3f}")
    print(f"  base_link -> livox_frame: xyz={tuple(args.livox_xyz)} rpy_deg={tuple(args.livox_rpy_deg)}")
    print()
    print("MOLA に渡す initial_pose（[x, y, z, yaw, pitch, roll] / 角度は度）:")
    print(f"  {pose}")
    print()
    print("使い方:")
    print(f"  G1_BAG_OFFSET={args.offset:.0f} G1_USE_MOLA=1 bash nav_stack.sh --rate 1")
    return 0


if __name__ == "__main__":
    sys.exit(main())
