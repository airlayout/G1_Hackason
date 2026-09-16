#!/usr/bin/env python3
"""複数地点を**同時に**地図へ当てはめる。地点間の相対姿勢はタグで固定する。

## なぜ要るか

1 地点ずつ地図と照合すると、部屋が繰り返し構造だと一意に決まらない
（2026-09-16 実測: regQ は差 4.6 pt、regR は 4.1 pt で決められず、しかも 2 つの答えは
互いに 6.6 m 離れていた——同じタグを 1.5〜2.4 m で見ているのだから、あり得ない）。

**同じタグが複数の地点から見えていれば、地点間の相対姿勢はタグだけで決まる**
（`relative_pose_from_tags.py`）。それを拘束にすれば、探す自由度が
3 地点 × 3 = 9 から **3 に減る**。全部の地点のスキャンが同時に合う場所を探すので、
偽の極大がはるかに残りにくい。

## 実装の要点

基準地点の姿勢 (x, y, yaw) を動かすと、他の地点の姿勢は相対姿勢から決まる。
基準の yaw を固定すれば、**他の地点のスキャン点も基準セルからの定数オフセット**で書ける。
だから 3 本のスキャンを 1 本の「合成スキャン」に畳んでしまえば、
1 地点の全探索と同じ仕掛けがそのまま使える。

## 使い方

    Navigation/.venv/bin/python joint_localize.py nav_map_run.yaml \\
        --scan regP scanP.yaml --scan regQ scanQ.yaml --scan regR scanR.yaml \\
        --relative regP regQ 1.748 -0.166 -172.53 \\
        --relative regP regR 1.116 1.252 -95.12 \\
        --write-pose-dir ./solved --out /tmp/joint.png

`--relative A B dx dy dyaw` は「A の base 座標系で見た B の位置と向き」。
`relative_pose_from_tags.py` の出力をそのまま渡す。
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_scan_overlay import load_scan
from global_localize import FREE_VALUE, RIVAL_DISTANCE_M, RIVAL_YAW_DEG, Field


def compose(base: tuple, relative: tuple) -> tuple:
    """基準の姿勢に相対姿勢を合成する。base=(x, y, yaw_deg)。"""

    x, y, yaw_deg = base
    dx, dy, dyaw = relative
    angle = math.radians(yaw_deg)
    return (x + dx * math.cos(angle) - dy * math.sin(angle),
            y + dx * math.sin(angle) + dy * math.cos(angle),
            yaw_deg + dyaw)


def combined_points(scans: dict, relatives: dict, reference: str,
                    yaw_deg: float) -> tuple:
    """基準の yaw を決めたときの、全地点のスキャン点（基準位置からの相対）。"""

    xs, ys, owners = [], [], []
    for index, (label, (angles, ranges)) in enumerate(scans.items()):
        if label == reference:
            offset_x, offset_y, total_yaw = 0.0, 0.0, yaw_deg
        else:
            offset_x, offset_y, total_yaw = compose((0.0, 0.0, yaw_deg), relatives[label])
        rotation = math.radians(total_yaw)
        xs.append(offset_x + ranges * np.cos(angles + rotation))
        ys.append(offset_y + ranges * np.sin(angles + rotation))
        owners.append(np.full(len(ranges), index))
    return np.concatenate(xs), np.concatenate(ys), np.concatenate(owners)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map_yaml")
    parser.add_argument("--scan", action="append", nargs=2, required=True,
                        metavar=("LABEL", "YAML"))
    parser.add_argument("--relative", action="append", nargs=5, default=[],
                        metavar=("FROM", "TO", "DX", "DY", "DYAW"))
    parser.add_argument("--reference", default=None, help="基準にする地点（既定は最初）")
    parser.add_argument("--yaw-step", type=float, default=2.0)
    parser.add_argument("--used-min", type=float, default=80.0)
    parser.add_argument("--chunk", type=int, default=600)
    parser.add_argument("--top", type=int, default=6)
    parser.add_argument("--write-pose-dir", default=None,
                        help="<DIR>/<地点>/pose.txt を書く")
    parser.add_argument("--force-write", action="store_true")
    parser.add_argument("--out", default=None)
    arguments = parser.parse_args()

    field = Field(Path(arguments.map_yaml))
    scans = {}
    for label, path in arguments.scan:
        angles, ranges, frame = load_scan(Path(path))
        scans[label] = (angles, ranges)
        print(f"{label}: {len(ranges)} 点（frame {frame}）")
    reference = arguments.reference or arguments.scan[0][0]

    relatives = {}
    for item in arguments.relative:
        source, target = item[0], item[1]
        if source != reference:
            parser.error(f"--relative の起点は基準 {reference} に揃えること（{source} が来た）")
        relatives[target] = (float(item[2]), float(item[3]), float(item[4]))
    missing = set(scans) - {reference} - set(relatives)
    if missing:
        parser.error(f"相対姿勢が足りない: {sorted(missing)}")
    for label, value in relatives.items():
        print(f"拘束 {reference} → {label}: ({value[0]:+.3f}, {value[1]:+.3f}) "
              f"{value[2]:+.2f} deg")

    free = np.argwhere(field.grid == FREE_VALUE)
    rows, columns = free[:, 0].astype(np.int64), free[:, 1].astype(np.int64)
    best_value = np.full(len(free), -1.0, np.float32)
    best_yaw = np.zeros(len(free), np.float32)
    print(f"候補 {len(free)} セル × yaw {int(360 / arguments.yaw_step)} 通り "
          f"（スキャンは {len(scans)} 本まとめて評価）", flush=True)

    for yaw_deg in np.arange(0.0, 360.0, arguments.yaw_step):
        local_x, local_y, _ = combined_points(scans, relatives, reference, float(yaw_deg))
        delta_column = np.round(local_x / field.resolution).astype(np.int64)
        delta_row = -np.round(local_y / field.resolution).astype(np.int64)
        total = len(delta_column)
        for start in range(0, len(free), arguments.chunk):
            end = min(start + arguments.chunk, len(free))
            candidate_rows = rows[start:end, None] + delta_row[None, :]
            candidate_columns = columns[start:end, None] + delta_column[None, :]
            inside = ((candidate_rows >= 0) & (candidate_rows < field.height)
                      & (candidate_columns >= 0) & (candidate_columns < field.width))
            flat = (np.clip(candidate_rows, 0, field.height - 1) * field.width
                    + np.clip(candidate_columns, 0, field.width - 1))
            known = field.flat_known[flat] & inside
            count = known.sum(1)
            value = np.where(count > 0,
                             (field.flat_likelihood[flat] * known).sum(1)
                             / np.maximum(count, 1), 0.0) * 100.0
            value = np.where(100.0 * count / total >= arguments.used_min,
                             value, -1.0).astype(np.float32)
            better = value > best_value[start:end]
            best_value[start:end] = np.where(better, value, best_value[start:end])
            best_yaw[start:end] = np.where(better, yaw_deg, best_yaw[start:end])

    if not np.any(best_value > 0):
        print(f"⛔ どの姿勢も --used-min {arguments.used_min} を満たさなかった")
        return 1

    order = np.argsort(-best_value)
    world_x, world_y = field.to_world(free[:, 0].astype(float), free[:, 1].astype(float))
    candidates = []
    for index in order[:40]:
        found = None
        for shift_y in np.arange(-0.05, 0.051, 0.025):
            for shift_x in np.arange(-0.05, 0.051, 0.025):
                for yaw in np.arange(best_yaw[index] - 2.0, best_yaw[index] + 2.001, 0.25):
                    base = (world_x[index] + shift_x, world_y[index] + shift_y, float(yaw))
                    scores = []
                    for label, (angles, ranges) in scans.items():
                        pose = base if label == reference else compose(base, relatives[label])
                        scores.append(field.evaluate(pose[0], pose[1], math.radians(pose[2]),
                                                     angles, ranges))
                    mean_value = float(np.mean([s[0] for s in scores]))
                    if found is None or mean_value > found[0]:
                        found = (mean_value, base, scores)
        mean_value, base, scores = found
        candidates.append({"base": (base[0], base[1], (base[2] + 180) % 360 - 180),
                           "mean": mean_value, "scores": scores})
    candidates.sort(key=lambda c: -c["mean"])

    top = candidates[0]
    rivals = [c for c in candidates[1:]
              if math.hypot(c["base"][0] - top["base"][0], c["base"][1] - top["base"][1])
              > RIVAL_DISTANCE_M
              or abs((c["base"][2] - top["base"][2] + 180) % 360 - 180) > RIVAL_YAW_DEG]

    print(f"\n=== 上位の候補（{len(scans)} 本の平均 dt15）===")
    for rank, candidate in enumerate(candidates[:arguments.top], 1):
        base = candidate["base"]
        detail = " / ".join(f"{label} {s[0]:.0f}"
                            for label, s in zip(scans, candidate["scores"]))
        print("%3d  基準 (%+8.3f, %+8.3f, %+7.2f)  平均 %5.1f   [%s]" % (
            rank, base[0], base[1], base[2], candidate["mean"], detail))

    print("\n=== 一意性 ===")
    if not rivals:
        print(f"✅ 最良から {RIVAL_DISTANCE_M:.0f} m / {RIVAL_YAW_DEG:.0f} deg 以上離れた対抗馬は **0 個**。一意")
    else:
        gap = top["mean"] - rivals[0]["mean"]
        print(f"{'⚠️' if gap > 5.0 else '⛔'} 離れた対抗馬 {len(rivals)} 個。差 {gap:.1f} pt")
        print(f"   次点 基準 ({rivals[0]['base'][0]:+.3f}, {rivals[0]['base'][1]:+.3f}, "
              f"{rivals[0]['base'][2]:+.2f}) 平均 {rivals[0]['mean']:.1f}")

    print("\n=== 各地点の姿勢 ===")
    poses = {}
    for label, metrics in zip(scans, top["scores"]):
        pose = top["base"] if label == reference else compose(top["base"], relatives[label])
        pose = (pose[0], pose[1], (pose[2] + 180) % 360 - 180)
        poses[label] = pose
        print("%-6s (%+8.3f, %+8.3f) %+7.2f deg   dt15 %5.1f / 既知 %5.1f %% / "
              "完全一致 %5.1f %% / ±1セル %5.1f %%" % (label, *pose, *metrics))

    if arguments.write_pose_dir:
        unique = not rivals or (top["mean"] - rivals[0]["mean"]) > 5.0
        if not unique and not arguments.force_write:
            print("\n⛔ 一意に決まっていないので pose.txt は書かない（--force-write で上書き可）")
        else:
            for label, pose in poses.items():
                target = Path(arguments.write_pose_dir) / label
                target.mkdir(parents=True, exist_ok=True)
                yaw = math.radians(pose[2])
                lines = ["# 記録時刻 tx ty tz qx qy qz qw",
                         f"# joint_localize.py（{len(scans)} 地点同時・タグで相対姿勢を拘束）"]
                for index in range(30):
                    lines.append("%.6f %.6f %.6f 0.000000 0.000000 0.000000 %.9f %.9f" % (
                        1000.0 + index * 0.1, pose[0], pose[1],
                        math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
                (target / "pose.txt").write_text("\n".join(lines) + "\n")
                print(f"書いた: {target / 'pose.txt'}")

    if arguments.out:
        import subprocess
        here = Path(__file__).resolve().parent
        command = [sys.executable, str(here / "render_pose_compare.py"), arguments.map_yaml]
        for (label, path), pose in zip(arguments.scan, [poses[l] for l, _ in arguments.scan]):
            command += ["--case", path, label, "同時探索",
                        f"{pose[0]:.4f}", f"{pose[1]:.4f}", f"{pose[2]:.3f}"]
        command += ["--out", arguments.out]
        subprocess.run(command, check=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
