"""ROS の地図（`.pgm` + `.yaml`）をブラウザで描ける PNG とメタ情報に変換する。

⚠️ **地図はセッション中ほぼ変わらない**（静的）ので、起動時に1回だけ読んで使い回す。
毎フレーム送るものではない。

座標の対応（ROS の map_server と同じ規約）:
    列 col = (x - origin_x) / resolution
    行 row = (height - 1) - (y - origin_y) / resolution      ← y は上向き、行は下向き
UI 側（JavaScript）も同じ式で世界座標をピクセルに直す。
"""
from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path

import cv2
import numpy as np
import yaml


@dataclass
class MapImage:
    """地図の PNG と、世界座標へ戻すためのメタ情報。"""

    png: bytes
    resolution: float
    origin_x: float
    origin_y: float
    width: int
    height: int

    def as_dict(self) -> dict[str, object]:
        return {
            "resolution": self.resolution,
            "origin": [self.origin_x, self.origin_y],
            "width": self.width,
            "height": self.height,
        }

    def world_to_px(self, x: float, y: float) -> tuple[int, int]:
        """世界座標 -> ピクセル (col, row)。モック側の自己位置生成で使う。"""
        col = int(round((x - self.origin_x) / self.resolution))
        row = int(round((self.height - 1) - (y - self.origin_y) / self.resolution))
        return col, row


def load_map(yaml_path: Path) -> MapImage:
    """`map_server` 形式の yaml を読み、PNG 化した地図を返す。

    `negate` / `occupied_thresh` / `free_thresh` は ROS の規約どおり解釈する。
    描画は「自由=明るい灰 / 占有=黒 / 未知=中間の灰」の3値にする。生の pgm をそのまま
    出すと未知と自由の区別がつきにくく、警備画面としては読みづらいため。
    """
    meta = yaml.safe_load(yaml_path.read_text(encoding="utf-8"))
    image_path = (yaml_path.parent / str(meta["image"])).resolve()
    raw = cv2.imread(str(image_path), cv2.IMREAD_UNCHANGED)
    if raw is None:
        raise RuntimeError(f"地図画像を読めませんでした: {image_path}")
    if raw.ndim == 3:
        raw = cv2.cvtColor(raw, cv2.COLOR_BGR2GRAY)

    resolution = float(meta["resolution"])
    origin = list(meta["origin"])
    negate = int(meta.get("negate", 0))
    occupied_thresh = float(meta.get("occupied_thresh", 0.65))
    free_thresh = float(meta.get("free_thresh", 0.196))

    # ROS の規約: p = (255 - value) / 255。negate なら反転しない
    value = raw.astype(np.float32) / 255.0
    p = value if negate else (1.0 - value)

    height, width = raw.shape
    canvas = np.full((height, width, 3), 0x44, dtype=np.uint8)      # 未知
    canvas[p < free_thresh] = (0xE8, 0xE8, 0xE4)                     # 自由
    canvas[p > occupied_thresh] = (0x1A, 0x1A, 0x1A)                 # 占有

    ok, buf = cv2.imencode(".png", canvas)
    if not ok:
        raise RuntimeError("地図の PNG 変換に失敗しました")

    print(f"[地図] 読み込んだ: {image_path.name} {width}x{height}px "
          f"({width * resolution:.1f} x {height * resolution:.1f} m) "
          f"resolution={resolution} origin=({origin[0]}, {origin[1]})")
    return MapImage(
        png=buf.tobytes(),
        resolution=resolution,
        origin_x=float(origin[0]),
        origin_y=float(origin[1]),
        width=width,
        height=height,
    )
