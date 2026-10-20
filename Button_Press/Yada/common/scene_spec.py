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


# 柱の丸い穴を作る横の帯の段の数（多いほど丸に近いが、箱の数が増える）
HOLE_STRIPS = 10


def _column_with_holes(cfg: dict[str, Any], floor: float, fx: float, front: float) -> list[Box]:
    """ボタンの位置に丸い穴（階段状）の開いた柱（いくつかの箱の組み合わせ）と、穴の奥のふた。

    実物の押しボタンは、パネルの穴にはまっていて、パネルの面より奥まで沈む。穴の無い 1 つの箱にすると、
    ボタンが柱の面まで沈んだところで指先が柱に当たって止まり、出っ張りの小さいボタンが押せなかった（2026-10-02）。
    穴の奥（ボタンが一番沈んだ位置より 1 mm 奥）には、柱と同じ色のふたを置く（壁が透けて見えないように）。
    いちばん上の箱の名前を hall_panel にする（ボタンとの衝突を外す相手として使う）。
    """
    c = cfg["column"]
    rgba = c["rgba"]
    y0, y1 = float(c["center_y"]) - float(c["width"]) / 2.0, float(c["center_y"]) + float(c["width"]) / 2.0
    top = floor + float(c["height"])
    bc = cfg["buttons"]["common"]
    half = float(bc["radius"]) + 0.001  # 穴の半分の大きさ（ボタンとのすき間 1 mm）
    back = front + float(bc["travel"]) + 0.001
    holes = sorted(((float(c["center_y"]) + float(it.get("offset_y", 0.0)), floor + float(it["height"]))
                    for it in cfg["buttons"]["items"]), key=lambda h: -h[1])
    out: list[Box] = []
    z_hi = top
    for k, (yc, zc) in enumerate(holes):
        if z_hi > zc + half:  # 穴より上の帯（全幅）
            out.append(_box_from_ranges("hall_panel" if k == 0 else f"hall_column_band{k}", (front, fx), (y0, y1),
                                        (zc + half, z_hi), rgba))
        # 穴のまわりは、細い横の帯を HOLE_STRIPS 段重ねる。帯ごとに内側の端を、その高さでの円の幅にそろえるので、
        # 穴は階段状の丸になる（四角い穴では、ボタンのまわりに四角い枠が見えた。実物の穴は丸い）
        edges = np.linspace(zc - half, zc + half, HOLE_STRIPS + 1)
        for j in range(HOLE_STRIPS):
            za, zb = edges[j] - zc, edges[j + 1] - zc
            zmin = 0.0 if za <= 0.0 <= zb else min(abs(za), abs(zb))
            w = float(np.sqrt(max(0.0, half ** 2 - zmin ** 2)))  # この帯の中での、円の幅の半分の最大
            z = (float(edges[j]), float(edges[j + 1]))
            out.append(_box_from_ranges(f"hall_column_hole{k}_s{j}_right", (front, fx), (y0, yc - w), z, rgba))
            out.append(_box_from_ranges(f"hall_column_hole{k}_s{j}_left", (front, fx), (yc + w, y1), z, rgba))
        out.append(_box_from_ranges(f"hall_column_hole{k}_back", (back, fx), (yc - half, yc + half),
                                    (zc - half, zc + half), rgba))
        z_hi = zc - half
    out.append(_box_from_ranges("hall_column_bottom", (front, fx), (y0, y1), (floor, z_hi), rgba))
    return out


def build_hall_scene(cfg: dict[str, Any]) -> HallScene:
    """configs/elevator_hall.yaml（壁に付いた盤）/ elevator_prod.yaml（本番に似た黒い柱。layout: column）から HallScene を作る。"""
    floor = -float(cfg["pelvis_height"])
    boxes: list[Box] = []

    w = cfg["wall"]
    fx = float(w["front_x"])
    boxes.append(_box_from_ranges("hall_wall", (fx, fx + float(w["thickness"])), tuple(w["y_range"]),
                                  (floor, floor + float(w["height"])), w["rgba"]))

    column = cfg.get("layout") == "column"
    d = cfg["door"]
    half_w = float(d["width"]) / 2.0
    if column and str(d.get("center_y")) == "auto":
        # 本番に似た形: 扉は、柱のすぐ左（y の大きい側）に置く
        c = cfg["column"]
        cy = float(c["center_y"]) + float(c["width"]) / 2.0 + float(d["frame_width"]) + half_w
    else:
        cy = float(d["center_y"])
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

    if column:
        # 本番に似た形: 床から立つ黒い柱にボタンが縦に並ぶ。柱の名前は hall_panel（ボタンとの衝突を外す相手として使う）
        c = cfg["column"]
        pcy = float(c["center_y"])
        panel_front = fx - float(c["protrusion"])
        boxes += _column_with_holes(cfg, floor, fx, panel_front)
        # 柱に貼る印（車いすの印など。見た目だけの薄い板）
        for dc in cfg.get("decals", []):
            hw, hh = float(dc["size"][0]) / 2.0, float(dc["size"][1]) / 2.0
            zc, yc = floor + float(dc["height"]), pcy + float(dc.get("offset_y", 0.0))
            boxes.append(_box_from_ranges(f"hall_decal_{dc['name']}", (panel_front - 0.0005, panel_front),
                                          (yc - hw, yc + hw), (zc - hh, zc + hh), dc["rgba"]))

        def button_z(item: dict[str, Any]) -> float:
            return floor + float(item["height"])
    else:
        p = cfg["panel"]
        pcy, pcz = float(p["center_y"]), floor + float(p["center_height"])
        panel_front = fx - float(p["thickness"])
        boxes.append(_box_from_ranges("hall_panel", (panel_front, fx),
                                      (pcy - float(p["width"]) / 2.0, pcy + float(p["width"]) / 2.0),
                                      (pcz - float(p["height"]) / 2.0, pcz + float(p["height"]) / 2.0), p["rgba"]))

        def button_z(item: dict[str, Any]) -> float:
            return pcz + float(item["offset_z"])

    bc = cfg["buttons"]["common"]
    buttons: list[ButtonSpec] = []
    for item in cfg["buttons"]["items"]:
        face = np.array([panel_front - float(bc["protrusion"]), pcy + float(item.get("offset_y", 0.0)), button_z(item)])
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
