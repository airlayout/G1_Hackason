#!/usr/bin/env python3
"""スキャン 1 枚と 2D 地図だけから、ロボットの姿勢を**地図全体から**探す。

## なぜ要るか

AMCL も MOLA も「だいたいここ」という初期値の近くしか直せない。初期値が大きく
外れていると、近くをいくら探しても正解に届かない（2026-09-16 実測: 目測で与えた
初期値が 6 m ずれていて、AMCL はそこから抜け出せなかった）。
ここは初期値を**使わない**。地図の自由セル全部 × 向き全部を総当たりする。

## 効くのは `--used-min`（ここが本質）

**「スキャン点のうち、地図の既知セルに落ちた割合」**に下限を課す。これが無いと、
**未観測領域（灰色）へスキャンを逃がす姿勢が偽の高得点を出す**。
2026-09-16 の実測では、この制約なしだと 1 m 以上離れた対抗馬が 451〜605 個出たが、
85 % を課すと **0 個**になり一意に決まった。

## 指標

`dt15 = 100 * mean(exp(-d^2 / (2*0.15^2)))`、d = 最近占有セルまでの距離 [m]。
未観測セルに落ちた点は占有とも自由とも数えない（分母からも外す）。

## 使い方

    # 機体で 1 枚取る（PC2 側。測位が動いていなくてよい）
    ssh g1w 'bash /tmp/grab_scan.sh' && scp g1w:/tmp/scan.yaml .

    Navigation/.venv/bin/python global_localize.py nav_map_run.yaml scan.yaml \\
        --out /tmp/global.png

⚠️ 1140 万姿勢を掃くので 2〜4 分かかる。⚠️ **出た姿勢は必ず図で確かめること。**
"""

from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np
import yaml
from scipy import ndimage

sys.path.insert(0, str(Path(__file__).resolve().parent))
from check_scan_overlay import first_document, load_scan, read_pgm, score

OCCUPIED_BELOW = 100
FREE_VALUE = 254
SIGMA = 0.15
# 上位候補のうち、最良から「これだけ離れている」ものを対抗馬として数える。
RIVAL_DISTANCE_M = 1.0
RIVAL_YAW_DEG = 15.0


class Field:
    """地図から、探索に要る配列をまとめて作る。"""

    def __init__(self, map_yaml: Path) -> None:
        meta = yaml.safe_load(map_yaml.read_text())
        self.grid = read_pgm(map_yaml.parent / meta["image"])
        self.resolution = float(meta["resolution"])
        self.origin_x, self.origin_y = (float(meta["origin"][0]), float(meta["origin"][1]))
        self.height, self.width = self.grid.shape
        occupied = self.grid < OCCUPIED_BELOW
        self.occupied = occupied
        self.known = occupied | (self.grid > 210)
        distance = ndimage.distance_transform_edt(~occupied) * self.resolution
        self.likelihood = np.exp(-(distance ** 2) / (2.0 * SIGMA ** 2)).astype(np.float32)
        self.flat_likelihood = self.likelihood.ravel()
        self.flat_known = self.known.ravel()

    def to_world(self, row: np.ndarray, column: np.ndarray) -> tuple:
        return (self.origin_x + column * self.resolution,
                self.origin_y + ((self.height - 1) - row) * self.resolution)

    def evaluate(self, x: float, y: float, yaw_rad: float,
                 angles: np.ndarray, ranges: np.ndarray) -> tuple:
        """厳密な採点。(dt15, used[%], hit0[%], hit1[%]) を返す。"""

        map_x = x + ranges * np.cos(angles + yaw_rad)
        map_y = y + ranges * np.sin(angles + yaw_rad)
        columns = np.round((map_x - self.origin_x) / self.resolution).astype(int)
        rows = np.round((self.height - 1) - (map_y - self.origin_y) / self.resolution).astype(int)
        inside = ((columns >= 0) & (columns < self.width)
                  & (rows >= 0) & (rows < self.height))
        columns, rows = np.clip(columns, 0, self.width - 1), np.clip(rows, 0, self.height - 1)
        used = inside & self.known[rows, columns]
        if not np.any(used):
            return 0.0, 0.0, 0.0, 0.0
        return (100.0 * float(self.likelihood[rows[used], columns[used]].mean()),
                100.0 * float(used.mean()),
                score(self.occupied, columns[used], rows[used], 0),
                score(self.occupied, columns[used], rows[used], 1))


def search(field: Field, angles: np.ndarray, ranges: np.ndarray,
           yaw_step: float, used_min: float, chunk: int) -> tuple:
    """全自由セル × 全 yaw を掃き、セルごとの最良を返す。"""

    free = np.argwhere(field.grid == FREE_VALUE)
    rows, columns = free[:, 0].astype(np.int64), free[:, 1].astype(np.int64)
    best_value = np.full(len(free), -1.0, np.float32)
    best_yaw = np.zeros(len(free), np.float32)
    total = len(ranges)
    print(f"候補 {len(free)} セル × yaw {int(360 / yaw_step)} 通り "
          f"= {len(free) * int(360 / yaw_step) / 1e6:.1f} M 姿勢", flush=True)

    for yaw_deg in np.arange(0.0, 360.0, yaw_step):
        yaw = math.radians(yaw_deg)
        delta_column = np.round(ranges * np.cos(angles + yaw) / field.resolution).astype(np.int64)
        delta_row = -np.round(ranges * np.sin(angles + yaw) / field.resolution).astype(np.int64)
        for start in range(0, len(free), chunk):
            end = min(start + chunk, len(free))
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
            value = np.where(100.0 * count / total >= used_min, value, -1.0).astype(np.float32)
            better = value > best_value[start:end]
            best_value[start:end] = np.where(better, value, best_value[start:end])
            best_yaw[start:end] = np.where(better, yaw_deg, best_yaw[start:end])
    return free, best_value, best_yaw


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("map_yaml")
    parser.add_argument("scan_yaml")
    parser.add_argument("--yaw-step", type=float, default=2.0, help="粗探索の刻み [deg]")
    parser.add_argument("--used-min", type=float, default=85.0,
                        help="スキャン点が既知セルに落ちる割合の下限 [%%]。**ここが効く**")
    parser.add_argument("--chunk", type=int, default=1500)
    parser.add_argument("--top", type=int, default=8, help="詰めて報告する候補の数")
    parser.add_argument("--out", default=None, help="確認用の図")
    parser.add_argument("--write-pose-txt", default=None, metavar="DIR",
                        help="求めた姿勢を <DIR>/pose.txt に TUM 形式で書く。"
                             "register_tags.py がそのまま読める（＝**測位を動かさずに登録できる**）")
    parser.add_argument("--force-write", action="store_true",
                        help="一意でなくても pose.txt を書く")
    arguments = parser.parse_args()

    field = Field(Path(arguments.map_yaml))
    angles, ranges, frame = load_scan(Path(arguments.scan_yaml))
    print(f"地図 {field.width}x{field.height} / スキャン {len(ranges)} 点（frame {frame}）")
    if frame not in ("base_link", "base_footprint"):
        print(f"⚠️ スキャンの frame が {frame}。base_link 前提で計算している")

    free, best_value, best_yaw = search(field, angles, ranges, arguments.yaw_step,
                                        arguments.used_min, arguments.chunk)
    if not np.any(best_value > 0):
        print(f"⛔ どの姿勢も `--used-min {arguments.used_min}` を満たさなかった。"
              "下げて試すか、スキャンか地図を疑う")
        return 1

    order = np.argsort(-best_value)
    world_x, world_y = field.to_world(free[:, 0].astype(float), free[:, 1].astype(float))
    candidates = []
    for index in order[:max(arguments.top * 6, 40)]:
        base_x, base_y, base_yaw = world_x[index], world_y[index], float(best_yaw[index])
        found = None
        for shift_y in np.arange(-0.05, 0.051, 0.025):
            for shift_x in np.arange(-0.05, 0.051, 0.025):
                for yaw in np.arange(base_yaw - 2.0, base_yaw + 2.001, 0.25):
                    value = field.evaluate(base_x + shift_x, base_y + shift_y,
                                           math.radians(yaw), angles, ranges)
                    if found is None or value[0] > found[0][0]:
                        found = (value, base_x + shift_x, base_y + shift_y, yaw)
        metrics, x, y, yaw = found
        candidates.append({"x": x, "y": y, "yaw": (yaw + 180.0) % 360.0 - 180.0,
                           "dt15": metrics[0], "used": metrics[1],
                           "hit0": metrics[2], "hit1": metrics[3]})
    candidates.sort(key=lambda c: -c["dt15"])

    top = candidates[0]
    rivals = [c for c in candidates[1:]
              if math.hypot(c["x"] - top["x"], c["y"] - top["y"]) > RIVAL_DISTANCE_M
              or abs((c["yaw"] - top["yaw"] + 180) % 360 - 180) > RIVAL_YAW_DEG]

    print("\n=== 上位の候補 ===")
    print("順位      x        y      yaw      dt15   既知に落ちた  完全一致  ±1セル")
    for rank, candidate in enumerate(candidates[:arguments.top], 1):
        print("%3d  %+8.3f %+8.3f %+8.2f   %5.1f      %5.1f %%    %5.1f %%  %5.1f %%" % (
            rank, candidate["x"], candidate["y"], candidate["yaw"],
            candidate["dt15"], candidate["used"], candidate["hit0"], candidate["hit1"]))

    print(f"\n=== 一意性 ===")
    if not rivals:
        print(f"✅ 最良から {RIVAL_DISTANCE_M:.0f} m / {RIVAL_YAW_DEG:.0f} deg 以上離れた対抗馬は **0 個**。一意")
    else:
        gap = top["dt15"] - rivals[0]["dt15"]
        mark = "⚠️" if gap > 5.0 else "⛔"
        print(f"{mark} 離れた対抗馬が {len(rivals)} 個。最良との差は {gap:.1f} pt")
        print(f"   次点: ({rivals[0]['x']:+.3f}, {rivals[0]['y']:+.3f}, {rivals[0]['yaw']:+.2f}) "
              f"dt15 {rivals[0]['dt15']:.1f}")
        if gap <= 5.0:
            print("   **決められない。図で確かめるか、別の場所でもう 1 枚取る**")

    print("\n=== この姿勢を AMCL に渡すなら ===")
    print(f"  bash /tmp/amcl_kick.sh {top['x']:.3f} {top['y']:.3f} {top['yaw']:.2f}")
    print("  ⚠️ **図で赤が黒に乗っているか確かめてから渡すこと**")

    if arguments.write_pose_txt:
        unique = not rivals or (top["dt15"] - rivals[0]["dt15"]) > 5.0
        if not unique and not arguments.force_write:
            print("\n⛔ 一意に決まっていないので pose.txt は書かない（--force-write で上書き可）")
        else:
            yaw = math.radians(top["yaw"])
            target = Path(arguments.write_pose_txt) / "pose.txt"
            target.parent.mkdir(parents=True, exist_ok=True)
            # TUM 形式。静止している前提なので同じ姿勢を並べる（register_tags の静止判定を通す）
            lines = ["# 記録時刻 tx ty tz qx qy qz qw",
                     f"# global_localize.py が {Path(arguments.scan_yaml).name} から求めた姿勢",
                     f"# dt15 {top['dt15']:.1f} / 完全一致 {top['hit0']:.1f} % / "
                     f"既知に落ちた {top['used']:.1f} %"]
            for index in range(30):
                lines.append("%.6f %.6f %.6f 0.000000 0.000000 0.000000 %.9f %.9f" % (
                    1000.0 + index * 0.1, top["x"], top["y"],
                    math.sin(yaw / 2.0), math.cos(yaw / 2.0)))
            target.write_text("\n".join(lines) + "\n")
            print(f"\n書いた: {target}（register_tags.py がそのまま読める）")

    if arguments.out:
        import subprocess
        here = Path(__file__).resolve().parent
        subprocess.run([sys.executable, str(here / "render_pose_compare.py"),
                        arguments.map_yaml,
                        "--case", arguments.scan_yaml, "探索", "1位",
                        f"{top['x']:.4f}", f"{top['y']:.4f}", f"{top['yaw']:.3f}",
                        "--case", arguments.scan_yaml, "探索", "2位",
                        f"{candidates[1]['x']:.4f}", f"{candidates[1]['y']:.4f}",
                        f"{candidates[1]['yaw']:.3f}",
                        "--out", arguments.out], check=False)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
