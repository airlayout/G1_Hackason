#!/usr/bin/env python3
"""事前地図が「機体が実際に歩いた道」を塞いでいないかを測る。ROS 不要。

## なぜ要るか

2026-09-08 の実測で、Nav2 の経路計画（段 B）が落ちる原因は planner でも
`robot_radius` でもなく **地図そのもの**だと分かった。`nav_map.pgm` の占有セルまでの
距離を機体の軌跡上で測ると中央値 0.200 m しかなく、**軌跡の 73.1% が
robot_radius(0.30 m) の内側**にいた。つまり事前地図が自分の歩いた道を塞いでいる。

その時はインラインで測ったので、地図を作り直すたびに測り直せるよう道具にした。
**測れないものは直せない。**

## 何を測るか

| 量 | 意味 | 合否 |
|---|---|---|
| 占有セルまでの距離 中央値 | 軌跡がどれだけ余裕を持って通れるか | `> robot_radius` |
| 0.30 m 未満の軌跡点の割合 | 内接円に触る点。これが段 B の落ち方そのもの | 小さいほどよい |
| 占有セル数 | 壁や机まで消していないかの歯止め | 激減しないこと |

**占有セルを全部消せば距離は無限になる。**だから距離だけを見てはいけない。
`--baseline` に元の地図を渡すと、占有セルの減り方を並べて表示する。

## ⚠️ 軌跡の点をそのまま数えてはいけない

軌跡は**時間**で等間隔なので、立ち止まっている場所の点が積み上がる。
`20260906T135940_UiS_room_v3` では 5,670 点のうち **931 点（16.4%）が同じ 1 セル**、
上位 10 セルで 44.4% を占める（t+418 秒以降ずっと開始点で静止しているため）。
そのまま平均を取ると「散らかった開始点」の成績が全体の成績に化ける。

→ **セルで重複を除いた「道」の統計を合否に使う。**時間の統計も参考に並べる。
（同じ型の罠: 床の被覆率も軌跡の長さで分母が動く）

## 使い方

    ../../Navigation/.venv/bin/python quickstart/check_map_clearance.py \\
        runs/<id>/map/nav_map runs/<id>/mola_floor0/traj.txt
    # 作り直した地図を元の地図と比べる
    ... quickstart/check_map_clearance.py \\
        runs/<id>/map/nav_map_clean runs/<id>/mola_floor0/traj.txt \\
        --baseline runs/<id>/map/nav_map

地図は拡張子なし・`.yaml`・`.pgm` のどれで指しても良い。
軌跡は TUM 形式（`time tx ty tz qx qy qz qw`）。`#` で始まる行は読み飛ばす。
"""
from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
from scipy import ndimage

DEFAULT_ROBOT_RADIUS = 0.30   # 肩幅 0.45 m 由来の内接円。下げられない物理値
PGM_MAXVAL = 255
# pcd_to_occupancy.py が書く値。map_server の慣習でもある
PGM_OCCUPIED, PGM_FREE, PGM_UNKNOWN = 0, 254, 205
# 占有セルがこの割合より減ったら「壁まで消したのでは」と疑う
OCCUPIED_DROP_WARN = 0.40
HISTOGRAM_EDGES = (0.0, 0.10, 0.20, 0.30, 0.50, 1.00, 2.00, float("inf"))
HISTOGRAM_WIDTH = 28


class MapGrid:
    """`.pgm` + `.yaml` を読んで、世界座標 → セルの変換を持つ格子。"""

    def __init__(self, name: str, image: np.ndarray, resolution: float,
                 origin: tuple[float, float], occupied: np.ndarray,
                 unknown: np.ndarray, unknown_reads_as_free: bool) -> None:
        self.name = name
        self.image = image            # row 0 が最も y の小さい行（ROS 向きに直したもの）
        self.resolution = resolution
        self.origin = origin
        self.occupied = occupied
        self.unknown = unknown
        self.unknown_reads_as_free = unknown_reads_as_free

    @property
    def free(self) -> np.ndarray:
        return ~self.occupied & ~self.unknown

    @property
    def shape(self) -> tuple[int, int]:
        return self.image.shape       # (rows, cols) = (height, width)

    def to_cells(self, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        """世界座標 (N,2) を (col, row, 地図の内側か) に落とす。"""
        rows, cols = self.shape
        col = np.floor((xy[:, 0] - self.origin[0]) / self.resolution).astype(int)
        row = np.floor((xy[:, 1] - self.origin[1]) / self.resolution).astype(int)
        inside = (col >= 0) & (col < cols) & (row >= 0) & (row < rows)
        return col, row, inside


def read_pgm(path: Path) -> np.ndarray:
    """P5 の pgm を読む。コメント行を飛ばし、8bit 前提で (h, w) を返す。"""
    data = path.read_bytes()
    tokens: list[bytes] = []
    pos = 0
    while len(tokens) < 4:
        while pos < len(data) and data[pos:pos + 1].isspace():
            pos += 1
        if data[pos:pos + 1] == b"#":
            while pos < len(data) and data[pos:pos + 1] != b"\n":
                pos += 1
            continue
        start = pos
        while pos < len(data) and not data[pos:pos + 1].isspace():
            pos += 1
        tokens.append(data[start:pos])
    magic, width, height, maxval = tokens
    if magic != b"P5":
        raise SystemExit("P5 の pgm ではありません: {} ({!r})".format(path, magic))
    if int(maxval) != PGM_MAXVAL:
        raise SystemExit("maxval が {} ではありません: {}".format(PGM_MAXVAL, path))
    pos += 1  # ヘッダ末尾の空白 1 文字
    width, height = int(width), int(height)
    pixels = np.frombuffer(data, dtype=np.uint8, count=width * height, offset=pos)
    return pixels.reshape(height, width)


def read_map_yaml(path: Path) -> dict[str, object]:
    """map_server の yaml を読む。必要な 5 つだけ拾う（PyYAML を持ち込まない）。"""
    fields: dict[str, object] = {}
    for line in path.read_text().splitlines():
        line = line.split("#", 1)[0].strip()
        if ":" not in line:
            continue
        key, value = (part.strip() for part in line.split(":", 1))
        if key == "origin":
            fields[key] = [float(v) for v in value.strip("[]").split(",")]
        elif key in ("resolution", "occupied_thresh", "free_thresh"):
            fields[key] = float(value)
        elif key in ("image", "negate"):
            fields[key] = value
    for required in ("image", "resolution", "origin"):
        if required not in fields:
            raise SystemExit("{} に {} がありません".format(path, required))
    return fields


def load_map(spec: Path) -> MapGrid:
    """拡張子なし / `.yaml` / `.pgm` のどれで指されても地図を読む。"""
    yaml_path = spec if spec.suffix == ".yaml" else spec.with_suffix(".yaml")
    if not yaml_path.exists():
        raise SystemExit("地図の yaml が見つかりません: {}".format(yaml_path))
    fields = read_map_yaml(yaml_path)
    pgm_path = yaml_path.parent / str(fields["image"])
    if not pgm_path.exists():
        raise SystemExit("地図の pgm が見つかりません: {}".format(pgm_path))

    # pgm は左上が原点、ROS の格子は左下が原点。書き出しで反転しているので戻す
    image = np.flipud(read_pgm(pgm_path))
    negate = str(fields.get("negate", "0")).strip() in ("1", "true", "True")
    probability = (image / PGM_MAXVAL) if negate else (1.0 - image / PGM_MAXVAL)
    occupied = probability > float(fields.get("occupied_thresh", 0.65))
    # 未知は「書き手が 205 を置いた所」で数える。map_server の閾値では拾えない
    # （205 は occ=0.196 なので、free_thresh が 0.196 より大きいと空きに落ちる）
    unknown = image == PGM_UNKNOWN
    unknown_occ = 1.0 - PGM_UNKNOWN / PGM_MAXVAL
    reads_as_free = unknown_occ < float(fields.get("free_thresh", 0.25))
    origin = fields["origin"]
    return MapGrid(pgm_path.name, image, float(fields["resolution"]),
                   (float(origin[0]), float(origin[1])), occupied,
                   unknown, reads_as_free)


def read_trajectory(path: Path) -> np.ndarray:
    """TUM 形式の軌跡から (N,2) の xy を取る。"""
    rows = []
    for line in path.read_text().splitlines():
        line = line.strip()
        if not line or line.startswith("#"):
            continue
        parts = line.split()
        if len(parts) < 4:
            continue
        rows.append((float(parts[1]), float(parts[2])))
    if not rows:
        raise SystemExit("軌跡の点が 0 です: {}".format(path))
    return np.asarray(rows, dtype=float)


def distance_to_occupied(grid: MapGrid) -> np.ndarray:
    """各セルから最も近い占有セルまでの距離[m]。占有セル自身は 0。"""
    if not grid.occupied.any():
        raise SystemExit("占有セルが 0 です。地図の作り方を疑ってください: {}".format(grid.name))
    return ndimage.distance_transform_edt(~grid.occupied) * grid.resolution


def describe_map(grid: MapGrid) -> None:
    rows, cols = grid.shape
    total = grid.image.size
    print("地図: {}  {} x {} セル / {:.3f} m / origin ({:.4f}, {:.4f})".format(
        grid.name, cols, rows, grid.resolution, grid.origin[0], grid.origin[1]))
    print("  占有 {:,} ({:.2f}%) / 空き {:,} ({:.2f}%) / 未知 {:,} ({:.2f}%)".format(
        int(grid.occupied.sum()), 100.0 * grid.occupied.sum() / total,
        int(grid.free.sum()), 100.0 * grid.free.sum() / total,
        int(grid.unknown.sum()), 100.0 * grid.unknown.sum() / total))
    if grid.unknown.any() and grid.unknown_reads_as_free:
        print("  ⚠️ yaml の free_thresh では未知(205)が**空き**として読まれる"
              "（205 の occ=0.196 < free_thresh）。Nav2 は未観測の場所も通ろうとする")


def print_histogram(distances: np.ndarray) -> None:
    print("\n  距離の分布（道）")
    total = len(distances)
    counts = []
    for lo, hi in zip(HISTOGRAM_EDGES[:-1], HISTOGRAM_EDGES[1:]):
        counts.append(int(((distances >= lo) & (distances < hi)).sum()))
    peak = max(counts) or 1
    for (lo, hi), count in zip(zip(HISTOGRAM_EDGES[:-1], HISTOGRAM_EDGES[1:]), counts):
        label = "{:>4.2f}〜{:<5}".format(lo, "∞" if hi == float("inf") else "{:.2f}".format(hi))
        bar = "█" * int(round(HISTOGRAM_WIDTH * count / peak))
        print("    {} m  {:<28} {:>6,} ({:>5.1f}%)".format(
            label, bar, count, 100.0 * count / total))


def sample_clearance(grid: MapGrid, xy: np.ndarray) -> tuple[np.ndarray, np.ndarray, int, int]:
    """占有セルまでの距離を「道」と「時間」の 2 通りで返す。

    戻り値: (道＝セルで重複除去, 時間＝軌跡の全点, 地図の外の点数, 最長滞在セルの点数)
    """
    field = distance_to_occupied(grid)
    col, row, inside = grid.to_cells(xy)
    col, row = col[inside], row[inside]
    by_time = field[row, col]
    cells, counts = np.unique(np.column_stack([row, col]), axis=0, return_counts=True)
    by_path = field[cells[:, 0], cells[:, 1]]
    return by_path, by_time, int((~inside).sum()), int(counts.max())


def print_stats(distances: np.ndarray, robot_radius: float, label: str,
                verdict: bool) -> None:
    median = float(np.median(distances))
    mark = ("   {}".format("✅" if median > robot_radius else "❌")) if verdict else ""
    print("\n占有セルまでの距離（{}・{:,} 標本）".format(label, len(distances)))
    print("  中央値      {:.3f} m   (合格: > {:.3f} m){}".format(median, robot_radius, mark))
    for name, q in (("25% 分位", 25.0), (" 5% 分位", 5.0)):
        print("  {}    {:.3f} m".format(name, float(np.percentile(distances, q))))
    print("  最小        {:.3f} m".format(float(distances.min())))
    print("  {:.3f} m 未満   {:,} / {:,} = {:.1f}%".format(
        robot_radius, int((distances < robot_radius).sum()), len(distances),
        100.0 * float((distances < robot_radius).mean())))


def report(grid: MapGrid, xy: np.ndarray, robot_radius: float) -> dict[str, float]:
    describe_map(grid)
    by_path, by_time, outside, dwell = sample_clearance(grid, xy)
    print("\n軌跡: {:,} 点 → セルで重複除去して {:,}（地図の外 {:,} 点）".format(
        len(xy), len(by_path), outside))
    if outside:
        print("  ⚠️ 地図の外に出た点がある。座標系が噛み合っていない疑い")
    if dwell > 0.05 * len(xy):
        print("  ⚠️ 最長滞在セルに {:,} 点（{:.1f}%）。時間の統計はここに引きずられる".format(
            dwell, 100.0 * dwell / len(xy)))

    print_stats(by_path, robot_radius, "道＝セルで重複除去", verdict=True)
    print_stats(by_time, robot_radius, "時間＝軌跡の全点（参考）", verdict=False)
    print_histogram(by_path)
    return {"median": float(np.median(by_path)),
            "near": float((by_path < robot_radius).mean()),
            "occupied": float(grid.occupied.sum())}


def compare(current: dict[str, float], baseline: dict[str, float],
            robot_radius: float) -> None:
    print("\n" + "=" * 64)
    print("元の地図との比較")
    print("  中央値              {:.3f} → {:.3f} m".format(
        baseline["median"], current["median"]))
    print("  {:.2f} m 未満の割合   {:.1f}% → {:.1f}%".format(
        robot_radius, 100.0 * baseline["near"], 100.0 * current["near"]))
    drop = 1.0 - current["occupied"] / baseline["occupied"]
    print("  占有セル数          {:,} → {:,}（{:+.1f}%）".format(
        int(baseline["occupied"]), int(current["occupied"]), -100.0 * drop))
    if drop > OCCUPIED_DROP_WARN:
        print("  ⚠️ 占有セルが {:.0f}% 以上減っている。壁や机まで消していないか確かめること"
              .format(100.0 * OCCUPIED_DROP_WARN))


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("map", type=Path, help="地図（拡張子なし / .yaml / .pgm）")
    p.add_argument("traj", type=Path, help="TUM 形式の軌跡")
    p.add_argument("--baseline", type=Path, help="比較する元の地図")
    p.add_argument("--robot-radius", type=float, default=DEFAULT_ROBOT_RADIUS,
                   help="内接円の半径[m]。これ未満のクリアランスが段 B の落ち方")
    args = p.parse_args()

    xy = read_trajectory(args.traj)
    current = report(load_map(args.map), xy, args.robot_radius)
    if args.baseline:
        print("\n" + "-" * 64 + "\n[元の地図]")
        baseline = report(load_map(args.baseline), xy, args.robot_radius)
        compare(current, baseline, args.robot_radius)

    return 0 if current["median"] > args.robot_radius else 1


if __name__ == "__main__":
    raise SystemExit(main())
