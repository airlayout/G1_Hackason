"""エレベーター乗り場のシーンを、シミュレーターに依存しない形（箱とボタンの一覧）にする。

configs/elevator_hall.yaml を読み、すべて pelvis 座標 [m]（x 前方、y 左、z 上）の
箱（Box）とボタン（ButtonSpec）に直す。MuJoCo（sim/mujoco/）と Isaac Sim（sim/isaac/）の
両方がこの一覧からシーンを組み立てるので、2 つのシミュレーターで寸法がずれない。

ボタンの向き: 盤は壁（ロボットの前方 +x）に付いていて、ボタンの面はロボット側（−x）を向く。
押す向きは +x（壁に向かう向き）。
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any

import numpy as np

# 押す向き（pelvis 座標）。ボタンはこの向きに沈む
PRESS_AXIS: tuple[float, float, float] = (1.0, 0.0, 0.0)


@dataclass(frozen=True)
class Box:
    """固定の箱。center と half_size は pelvis 座標 [m]。"""

    name: str
    center: np.ndarray
    half_size: np.ndarray
    rgba: tuple[float, ...]


@dataclass(frozen=True)
class ButtonSpec:
    """押し込めるボタン（円柱）。face_center は押していないときのボタンの面の中心（pelvis 座標）。"""

    name: str
    symbol: str  # "up"（▲）または "down"（▼）
    face_center: np.ndarray
    radius: float
    protrusion: float
    travel: float
    press_depth: float
    stiffness: float
    damping: float
    mass: float
    off_rgba: tuple[float, ...]
    lit_rgba: tuple[float, ...]
    symbol_rgba: tuple[float, ...]
    symbol_size: float
    symbol_thickness: float

    @property
    def base_center(self) -> np.ndarray:
        """ボタンの根元（盤の表面）の中心。円柱はここから face_center まで伸びる。"""
        return self.face_center + np.asarray(PRESS_AXIS) * self.protrusion

    def symbol_triangle(self) -> np.ndarray:
        """記号の三角形の 3 頂点 (3, 2)。面の中心を原点とした (y, z)（pelvis 座標の向き）。

        ▲ は頂点が上、▼ は頂点が下。正三角形の重心を面の中心に合わせる。
        """
        s = self.symbol_size
        h = s * np.sqrt(3.0) / 2.0
        tri = np.array([[0.0, 2.0 * h / 3.0], [-s / 2.0, -h / 3.0], [s / 2.0, -h / 3.0]])
        if self.symbol == "down":
            tri[:, 1] *= -1.0
        elif self.symbol != "up":
            raise ValueError(f"記号は up か down: {self.symbol}")
        return tri


@dataclass(frozen=True)
class FloorSpec:
    """床の見た目（絨毯）。見た目だけで、衝突判定や摩擦には関係しない。"""

    rgb: tuple[float, float, float]
    fine_noise: float
    coarse_noise: float
    tile_size: float
    texture_px: int
    seed: int


def carpet_texture(floor: FloorSpec) -> np.ndarray:
    """絨毯のテクスチャ (px, px, 3) uint8。継ぎ目なく並べられる（むらを周期的なノイズで作るため）。

    細かいむら: 画素ごとの白色ノイズを少しぼかしたもの（毛の 1 本ずつ）。
    大きめのむら: 白色ノイズの低い周波数だけを残したもの（踏まれ方のむら）。
    どちらも FFT で作るので、画像の端どうしがつながる。
    """
    n = int(floor.texture_px)
    rng = np.random.default_rng(int(floor.seed))
    f = np.fft.fftfreq(n)
    r = np.sqrt(f[:, None] ** 2 + f[None, :] ** 2)

    def band(cutoff: float) -> np.ndarray:
        x = np.real(np.fft.ifft2(np.fft.fft2(rng.standard_normal((n, n))) * np.exp(-(r / cutoff) ** 2)))
        return x / (x.std() + 1e-12)

    lum = floor.fine_noise * band(0.35) + floor.coarse_noise * band(0.02)
    img = np.clip(np.asarray(floor.rgb)[None, None, :] * (1.0 + lum[..., None]), 0.0, 1.0)
    return (img * 255.0 + 0.5).astype(np.uint8)


@dataclass(frozen=True)
class HallScene:
    """乗り場のシーン全体。floor_z は床の高さ（pelvis 座標なので負の値）。"""

    pelvis_height: float
    floor: FloorSpec | None = None
    boxes: list[Box] = field(default_factory=list)
    buttons: list[ButtonSpec] = field(default_factory=list)

    @property
    def floor_z(self) -> float:
        return -self.pelvis_height

    def button(self, name: str) -> ButtonSpec:
        for b in self.buttons:
            if b.name == name:
                return b
        raise KeyError(f"ボタンが無い: {name}（あるのは {[b.name for b in self.buttons]}）")


def _box_from_ranges(name: str, x: tuple[float, float], y: tuple[float, float], z: tuple[float, float],
                     rgba: Any) -> Box:
    lo = np.array([min(x), min(y), min(z)])
    hi = np.array([max(x), max(y), max(z)])
    return Box(name=name, center=(lo + hi) / 2.0, half_size=(hi - lo) / 2.0, rgba=tuple(float(v) for v in rgba))


def build_hall_scene(cfg: dict[str, Any]) -> HallScene:
    """configs/elevator_hall.yaml の中身から HallScene を作る。"""
    floor = -float(cfg["pelvis_height"])
    boxes: list[Box] = []

    w = cfg["wall"]
    fx = float(w["front_x"])
    boxes.append(_box_from_ranges("hall_wall", (fx, fx + float(w["thickness"])), tuple(w["y_range"]),
                                  (floor, floor + float(w["height"])), w["rgba"]))

    d = cfg["door"]
    cy, half_w = float(d["center_y"]), float(d["width"]) / 2.0
    top = floor + float(d["height"])
    gap = float(d["gap"])
    front = fx - float(d["protrusion"])
    # 左右の扉（y の大きい側が左）。真ん中に隙間を空ける
    boxes.append(_box_from_ranges("hall_door_left", (front, fx), (cy + gap / 2.0, cy + half_w), (floor, top),
                                  d["leaf_rgba"]))
    boxes.append(_box_from_ranges("hall_door_right", (front, fx), (cy - half_w, cy - gap / 2.0), (floor, top),
                                  d["leaf_rgba"]))
    boxes.append(_box_from_ranges("hall_door_gap", (fx - float(d["protrusion"]) / 2.0, fx),
                                  (cy - gap / 2.0, cy + gap / 2.0), (floor, top), d["gap_rgba"]))
    # 三方枠
    fw = float(d["frame_width"])
    ff = fx - float(d["frame_protrusion"])
    boxes.append(_box_from_ranges("hall_door_frame_left", (ff, fx), (cy + half_w, cy + half_w + fw), (floor, top + fw),
                                  d["frame_rgba"]))
    boxes.append(_box_from_ranges("hall_door_frame_right", (ff, fx), (cy - half_w - fw, cy - half_w),
                                  (floor, top + fw), d["frame_rgba"]))
    boxes.append(_box_from_ranges("hall_door_frame_top", (ff, fx), (cy - half_w - fw, cy + half_w + fw),
                                  (top, top + fw), d["frame_rgba"]))

    p = cfg["panel"]
    pcy, pcz = float(p["center_y"]), floor + float(p["center_height"])
    panel_front = fx - float(p["thickness"])
    boxes.append(_box_from_ranges("hall_panel", (panel_front, fx),
                                  (pcy - float(p["width"]) / 2.0, pcy + float(p["width"]) / 2.0),
                                  (pcz - float(p["height"]) / 2.0, pcz + float(p["height"]) / 2.0), p["rgba"]))

    bc = cfg["buttons"]["common"]
    buttons: list[ButtonSpec] = []
    for item in cfg["buttons"]["items"]:
        face = np.array([panel_front - float(bc["protrusion"]), pcy, pcz + float(item["offset_z"])])
        buttons.append(ButtonSpec(
            name=str(item["name"]), symbol=str(item["symbol"]), face_center=face,
            radius=float(bc["radius"]), protrusion=float(bc["protrusion"]),
            travel=float(bc["travel"]), press_depth=float(bc["press_depth"]),
            stiffness=float(bc["stiffness"]), damping=float(bc["damping"]), mass=float(bc["mass"]),
            off_rgba=tuple(float(v) for v in bc["off_rgba"]), lit_rgba=tuple(float(v) for v in bc["lit_rgba"]),
            symbol_rgba=tuple(float(v) for v in bc["symbol_rgba"]),
            symbol_size=float(bc["symbol_size"]), symbol_thickness=float(bc["symbol_thickness"]),
        ))
        if buttons[-1].press_depth > buttons[-1].travel:
            raise ValueError(f"press_depth が travel より深い（押したと判定できない）: {item['name']}")
    floor_spec = None
    if "floor" in cfg:
        fl = cfg["floor"]
        floor_spec = FloorSpec(rgb=tuple(float(v) for v in fl["rgb"]), fine_noise=float(fl["fine_noise"]),
                               coarse_noise=float(fl["coarse_noise"]), tile_size=float(fl["tile_size"]),
                               texture_px=int(fl["texture_px"]), seed=int(fl.get("seed", 0)))
    return HallScene(pelvis_height=float(cfg["pelvis_height"]), floor=floor_spec, boxes=boxes, buttons=buttons)


class CallButtonState:
    """呼びボタンの点灯の状態。一度押したら reset() まで点灯したまま（実物の呼びボタンと同じ）。

    押し込みの深さ（沈んだ量 [m]）を毎ステップ update() に渡す。press_depth 以上で「押した」、
    その半分より浅くなったら「離した」（境目でちらつかないように差を付ける）。
    """

    def __init__(self, press_depth: float):
        self.press_depth = float(press_depth)
        self.lit = False
        self.held = False
        self.press_count = 0

    def update(self, depth: float) -> bool:
        """深さを渡す。押した瞬間（離した状態 → 押した状態）だけ True を返す。"""
        if not self.held and depth >= self.press_depth:
            self.held = True
            self.lit = True
            self.press_count += 1
            return True
        if self.held and depth < self.press_depth / 2.0:
            self.held = False
        return False

    def reset(self) -> None:
        """消灯する（かごが着いたときに相当）。押している途中なら、離すまで再点灯しない。"""
        self.lit = False
