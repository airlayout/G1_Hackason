"""胸の板（2 台を縦に横並びで載せ、胸に帯で締める）。2 つの部品に分かれる。

    - 背板の部品（back）: 胸に当たる背板と、左右の腕木。背板は胸の傾き（上が奥へ tilt 度）に合わせて傾け、
      裏に EVA を貼る。上の左右に肩へ回す帯の穴、下の左右の端に胴へ回す帯の穴
    - 台の部品（carrier）: 袋 2 つを載せる。下の棚・中ほどの梁・外の側板・内の側板（2 台の間の両側）。
      外の側板を左右の腕木で挟み、横向きの軸（x 軸）のまわりに回して上下の向きを選ぶ

座標: x 右・y 前・z 上（着た人の向き）。原点は 2 台の真ん中、y=0 は袋の厚みの真ん中、z=0 は袋の中の
スマホの下の端。台の座標は、上下 0° のとき背板の部品の座標と同じ。

左右の向き（z 軸まわり）: 袋ごとに外向き 0・10・20°。下の棚の穴で選ぶ。
上下の向き（x 軸まわり）: 台ごと −20〜+10°（+ が上向き）を 5° 刻み。軸は台の下の後ろの角にある
（下を向けても台の下の角がほとんど後ろへ動かず、背板に当たらない）。副尺の形の 2 列の穴で留める:
    - 台の外の側板に、軸の真上の穴 2 つ（軸から r1 と r2）。内の面にナットの六角の穴
    - 腕木に、r1 の弧で −20・−10・0・+10°、r2 の弧で −15・−5・+5° の穴
    → 10° の倍数は内の弧（r1）、5° 余りは外の弧（r2）で留める（穴は 10° おきで足り、間の肉が残る）
2 台は同じ台に載るので、上下の向きを替えても 2 台の間の向き（ステレオの較正）は変わらない。

スマホは上下 0° で水平に向くよう縦に立てる（胸の面とは tilt 度の楔になる）。

刷る向き: 背板の部品は背板の裏を台に置く（腕木は台から 68° に立つ）。台の部品は後ろの面を台に置く。
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from build123d import Box, Cylinder, Location, Part, Plane, Polygon, Pos, RegularPolygon, Rot, extrude

from dims import HW, LEFT, RIGHT, PhoneSlot
from pocket import CFG, index_local, inner, pivot_local


@dataclass(frozen=True)
class ChestCfg:
    # 胸の傾き: MakeHuman の標準の体で、胸骨の上から 6〜14 cm 下の前の面が z 80 mm で y 32 mm 進む（約 22°）
    tilt: float = 22.0
    # 2 つの袋の間（11-18 に 16 → 40 mm に空け、上下に回す腕木の分の幅を戻すため 28 mm にした）。
    # 外の端は ±126.5 mm（胸の高さの胴の横の面は ±140〜150 mm・MakeHuman の体）。広げるほど下ろした腕に近づく
    gap: float = 28.0
    inner_clear: float = 2.0  # 袋の内の面から内の側板まで
    angles: tuple[float, ...] = (0.0, 10.0, 20.0)  # 選べる外向きの角度（左右）
    pitches: tuple[float, ...] = (-20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0)  # 選べる上下の角度（+ が上向き）
    shelf_t: float = 6.0
    top_t: float = 6.0
    plate_t: float = 4.0
    wall_t: float = 4.0  # 内の側板・腕木の厚み
    carrier_wall_t: float = 6.0  # 台の外の側板（内の面にナットの六角の穴 4.2 mm を彫る）
    beam_d: float = 20.0  # 中ほどの棚（梁）の前後の幅
    rim: float = 16.0  # 背板の窓のまわりに残す肉
    side_clear: float = 12.0  # 軸から台の外の側板の内の面まで（20° 回したときの袋の前の角の動き 8 mm＋余り）
    front: float = 14.0  # 下の棚の前の端（y）
    # --- 上下の軸と穴 ---
    hinge_back: float = 15.0  # 軸の、台の後ろの面からの距離
    hinge_up: float = 14.0  # 軸の、台の下の面からの高さ
    r1: float = 45.0  # 内の弧（10° の倍数）。10° おきの穴の間 7.8 mm
    r2: float = 58.0  # 外の弧（5° 余り）。10° おきの穴の間 10.1 mm
    hinge_gap: float = 0.3  # 台の外の側板と腕木の間
    # 台の後ろの面と背板の前の面の間（台の下の面の高さ・上下 0°）。−20° で台の下の後ろの角が 4 mm 後ろへ来る
    back_clear: float = 4.0
    bracket_front: float = 8.0  # 腕木の前の端の y
    plate_top: float = 130.0  # 背板の上の端の z
    strap_zone: float = 34.0  # 背板を台より下へ伸ばす長さ（胴に回す帯の穴を置く）
    strap_bar: float = 6.0  # 胴の帯の穴から背板の外の端まで（帯を巻く棒）


CCFG = ChestCfg()
SLOTS = (LEFT, RIGHT)


def _box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _union(parts: list[Part]) -> Part:
    out = parts[0]
    for p in parts[1:]:
        out = out + p
    return out


# --- 左右の向き（袋ごと）---------------------------------------------------------------
def center_x(slot: PhoneSlot, cfg: ChestCfg = CCFG) -> float:
    """袋の中心の x（向き 0° のとき）。"""
    return slot.side * (cfg.gap / 2 + inner(slot).wo)


def inner_wall_x(cfg: ChestCfg = CCFG) -> float:
    """内の側板の、袋の側の面の x（右。左は −）。"""
    return cfg.gap / 2 - cfg.inner_clear


def pivot(slot: PhoneSlot, cfg: ChestCfg = CCFG) -> tuple[float, float]:
    px, py = pivot_local(slot)
    return center_x(slot, cfg) + px, py


def yaw_sign(slot: PhoneSlot) -> float:
    """外向きに回す向き（z 軸まわり）。右は −（+y→+x）、左は +。"""
    return -float(slot.side)


def _rot_about(px: float, py: float, dx: float, dy: float, deg: float) -> tuple[float, float]:
    a = math.radians(deg)
    return px + dx * math.cos(a) - dy * math.sin(a), py + dx * math.sin(a) + dy * math.cos(a)


def index_at(slot: PhoneSlot, angle: float, cfg: ChestCfg = CCFG) -> tuple[float, float]:
    """外向き angle 度のとき、向きを決める穴が来る位置。"""
    px, py = pivot(slot, cfg)
    (ix, iy), (lx, ly) = index_local(slot), pivot_local(slot)
    return _rot_about(px, py, ix - lx, iy - ly, yaw_sign(slot) * angle)


def pocket_center_at(slot: PhoneSlot, angle: float, cfg: ChestCfg = CCFG) -> tuple[float, float]:
    """外向き angle 度のときの袋の中心。"""
    px, py = pivot(slot, cfg)
    return _rot_about(px, py, center_x(slot, cfg) - px, -py, yaw_sign(slot) * angle)


# --- 要所 -------------------------------------------------------------------------------
@dataclass(frozen=True)
class Frame:
    """台と背板の要所。"""

    z0: float  # 台の下の面
    y_cb: float  # 台の後ろの面
    x_side: float  # 台の外の側板の内の面の x
    y_top_front: float  # 中ほどの棚（梁）の前の端
    z_top: float  # 梁の上の面（＝袋の中ほどの耳の下の面）
    hinge: tuple[float, float]  # 上下の軸 (y, z)
    y_plate0: float  # 背板の前の面の、高さ z0 での y
    z_plate_bottom: float  # 背板の下の端
    x_bracket: float  # 腕木の内の面の x
    x_plate: float  # 背板の外の端の x（＝腕木の外の面）


def frame_dims(cfg: ChestCfg = CCFG) -> Frame:
    to_max = max(inner(s).to for s in SLOTS)
    x_side = max(abs(pivot(s, cfg)[0]) for s in SLOTS) + cfg.side_clear
    z0 = -CFG.floor - cfg.shelf_t
    y_top_front = -to_max - 2.0
    y_cb = y_top_front - cfg.beam_d
    x_bracket = x_side + cfg.carrier_wall_t + cfg.hinge_gap
    return Frame(
        z0=z0,
        y_cb=y_cb,
        x_side=x_side,
        y_top_front=y_top_front,
        z_top=CFG.tab_z,
        hinge=(y_cb + cfg.hinge_back, z0 + cfg.hinge_up),
        y_plate0=y_cb - cfg.back_clear,
        z_plate_bottom=z0 - cfg.strap_zone,
        x_bracket=x_bracket,
        x_plate=x_bracket + cfg.wall_t,
    )


def back_y(z: float, f: Frame, cfg: ChestCfg = CCFG) -> float:
    """背板の前の面の y（高さ z で）。"""
    return f.y_plate0 - (z - f.z0) * math.tan(math.radians(cfg.tilt))


def plate_dy(cfg: ChestCfg = CCFG) -> float:
    """背板の厚みの、y の向きの長さ。"""
    return cfg.plate_t / math.cos(math.radians(cfg.tilt))


# --- 上下の向き ---------------------------------------------------------------------------
def carrier_loc(pitch: float, cfg: ChestCfg = CCFG) -> Location:
    """台の座標 → 背板の部品の座標。上下の軸のまわりに pitch 度（+ が上向き）回す。"""
    y, z = frame_dims(cfg).hinge
    return Pos(0, y, z) * Rot(pitch, 0, 0) * Pos(0, -y, -z)


def pitch_hole(pitch: float, cfg: ChestCfg = CCFG) -> tuple[float, float]:
    """上下 pitch 度で使う腕木の穴の (軸からの距離, 真上から後ろへの角度)。

    台の穴は軸の真上（r1 と r2）にあり、台を pitch 度回すと真上から後ろへ pitch 度の所に来る。
    10° の倍数は r1、5° 余りは r2 の穴を使う。
    """
    if pitch not in cfg.pitches:
        raise ValueError(f"上下の角度 {pitch} は選べない（{cfg.pitches}）")
    return (cfg.r1 if round(pitch / 5.0) % 2 == 0 else cfg.r2), pitch


def hole_yz(r: float, alpha: float, cfg: ChestCfg = CCFG) -> tuple[float, float]:
    """軸から r・真上から後ろへ alpha 度の点の (y, z)。"""
    y, z = frame_dims(cfg).hinge
    a = math.radians(alpha)
    return y - r * math.sin(a), z + r * math.cos(a)


def _x_hole(y: float, z: float, x0: float, x1: float) -> Part:
    """x の向きの丸い穴（x0..x1）。"""
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(HW.hole / 2, x1 - x0)


def _x_nut(y: float, z: float, x0: float, x1: float) -> Part:
    """x の向きのナットの六角の穴（x0..x1）。"""
    hexa = extrude(RegularPolygon(HW.nut_af / 2, 6, major_radius=False), amount=x1 - x0)
    return Pos(x0, y, z) * Rot(0, 90, 0) * hexa


# --- 背板の部品 ---------------------------------------------------------------------------
def _profile(pts: list[tuple[float, float]], x0: float, x1: float) -> Part:
    """YZ の輪郭を x0..x1 に押し出す。"""
    sk = Plane.YZ.offset(x0) * Polygon(*pts, align=None)
    return extrude(sk, amount=x1 - x0)


def _plate_cut(x: float, z: float, wx: float, wz: float, f: Frame, cfg: ChestCfg) -> Part:
    """背板を突き抜く長方形（中心の高さ z・幅 wx・背板に沿った長さ wz）。"""
    y = back_y(z, f, cfg) - plate_dy(cfg) / 2
    return Pos(x, y, z) * Rot(cfg.tilt, 0, 0) * Box(wx, 30.0, wz)


def plate(f: Frame, cfg: ChestCfg = CCFG) -> Part:
    """背板（窓・肩へ回す帯の穴・胴へ回す帯の穴）。"""
    dy = plate_dy(cfg)
    zb, zt = f.z_plate_bottom, cfg.plate_top
    pts = [(back_y(zb, f, cfg), zb), (back_y(zt, f, cfg), zt), (back_y(zt, f, cfg) - dy, zt), (back_y(zb, f, cfg) - dy, zb)]
    body = _profile(pts, -f.x_plate, f.x_plate)
    slant = 1.0 / math.cos(math.radians(cfg.tilt))
    za, zw = f.z0, f.z_top - cfg.rim - 2.0  # 窓の上下
    xa, xb = cfg.rim, f.x_bracket - cfg.rim  # 窓の左右（腕木の根元は残す）
    xs = f.x_plate - cfg.strap_bar - HW.strap_t / 2  # 胴の帯の穴の x
    cuts = []
    for s in (-1, 1):
        cuts.append(_plate_cut(s * (xa + xb) / 2, (za + zw) / 2, xb - xa, (zw - za) * slant, f, cfg))
        cuts.append(_plate_cut(s * (f.x_plate - 32.0), zt - 14.0, HW.strap_w, HW.strap_t, f, cfg))
        cuts.append(_plate_cut(s * xs, zb + 4.0 + HW.strap_w / 2, HW.strap_t, HW.strap_w, f, cfg))
    return body - _union(cuts)


def brackets(f: Frame, cfg: ChestCfg = CCFG) -> Part:
    """左右の腕木（背板から前へ。台の外の側板を外から挟む）と、上下の軸・角度の穴。"""
    yh, zh = f.hinge
    zb, zt = zh - 10.0, zh + cfg.r2 + 8.0
    mid = plate_dy(cfg) / 2  # 後ろの端は背板の厚みの真ん中まで（背板の裏へ出さない）
    pts = [(back_y(zb, f, cfg) - mid, zb), (cfg.bracket_front, zb), (cfg.bracket_front, zt), (back_y(zt, f, cfg) - mid, zt)]
    x0, x1 = f.x_bracket, f.x_plate
    body = _profile(pts, x0, x1) + _profile(pts, -x1, -x0)
    holes = [(yh, zh)] + [hole_yz(*pitch_hole(p, cfg), cfg) for p in cfg.pitches]
    cuts = []
    for y, z in holes:
        cuts += [_x_hole(y, z, x0 - 1, x1 + 1), _x_hole(y, z, -x1 - 1, -x0 + 1)]
    return body - _union(cuts)


def back(cfg: ChestCfg = CCFG) -> Part:
    """背板の部品（背板＋左右の腕木）。"""
    f = frame_dims(cfg)
    return plate(f, cfg) + brackets(f, cfg)


# --- 台の部品 -----------------------------------------------------------------------------
def _carrier_body(f: Frame, cfg: ChestCfg) -> Part:
    xw = f.x_side + 0.5
    shelf = _box(-xw, xw, f.y_cb, cfg.front, f.z0, f.z0 + cfg.shelf_t)
    beam = _box(-xw, xw, f.y_cb, f.y_top_front, f.z_top - cfg.top_t, f.z_top)
    side = [(f.y_cb, f.z0), (cfg.front, f.z0), (cfg.front, f.z0 + 24.0), (f.y_top_front, f.z_top), (f.y_cb, f.z_top)]
    mid = [(f.y_cb, f.z0), (f.y_top_front, f.z0), (f.y_top_front, f.z_top), (f.y_cb, f.z_top)]
    xo, xi = f.x_side, inner_wall_x(cfg)
    walls = [
        _profile(side, xo, xo + cfg.carrier_wall_t), _profile(side, -xo - cfg.carrier_wall_t, -xo),
        _profile(mid, xi - cfg.wall_t, xi), _profile(mid, -xi, -xi + cfg.wall_t),
    ]
    return _union([shelf, beam] + walls)


def _nut_trap(x: float, y: float, z_bottom: float) -> Part:
    hexa = extrude(RegularPolygon(HW.nut_af / 2, 6, major_radius=False), amount=HW.nut_depth)
    return Pos(x, y, z_bottom - 0.01) * hexa


def _cable_cut(slot: PhoneSlot, f: Frame, cfg: ChestCfg) -> Part:
    """袋の底の線の切り欠きの真下を、選べる角度ごとに抜く（向きの穴の肉は残す）。"""
    d = inner(slot)
    cuts = []
    for a in cfg.angles:
        x, y = pocket_center_at(slot, a, cfg)
        box = Box(CFG.cable_w + 2.0, d.t + 2.0, cfg.shelf_t + 2)
        cuts.append(Pos(x, y, f.z0 + cfg.shelf_t / 2) * Rot(0, 0, yaw_sign(slot) * a) * box)
    return _union(cuts)


def _yaw_holes(f: Frame, cfg: ChestCfg) -> Part:
    """左右の向きの穴: 軸の穴（下の棚・梁）・向きの穴（下の棚、角度ごと）・ナットの六角の穴・線の切り欠き。"""
    zt0 = f.z_top - cfg.top_t
    cuts = []
    for slot in SLOTS:
        px, py = pivot(slot, cfg)
        for x, y in [(px, py)] + [index_at(slot, a, cfg) for a in cfg.angles]:
            cuts.append(Pos(x, y, f.z0 + cfg.shelf_t / 2) * Cylinder(HW.hole / 2, cfg.shelf_t + 2) + _nut_trap(x, y, f.z0))
        cuts.append(Pos(px, py, f.z_top - cfg.top_t / 2) * Cylinder(HW.hole / 2, cfg.top_t + 2) + _nut_trap(px, py, zt0))
        cuts.append(_cable_cut(slot, f, cfg))
    return _union(cuts)


def _pitch_holes(f: Frame, cfg: ChestCfg) -> Part:
    """上下の穴（外の側板）: 軸と、軸の真上の r1・r2。内の面にナットの六角の穴。"""
    xo, xw = f.x_side, f.x_side + cfg.carrier_wall_t
    cuts = []
    for y, z in (f.hinge, hole_yz(cfg.r1, 0.0, cfg), hole_yz(cfg.r2, 0.0, cfg)):
        cuts += [_x_hole(y, z, xo - 1, xw + 1), _x_hole(y, z, -xw - 1, -xo + 1),
                 _x_nut(y, z, xo - 0.01, xo + HW.nut_depth), _x_nut(y, z, -xo - HW.nut_depth, -xo + 0.01)]
    return _union(cuts)


def carrier(cfg: ChestCfg = CCFG) -> Part:
    """台の部品（台の座標）。"""
    f = frame_dims(cfg)
    return _carrier_body(f, cfg) - _yaw_holes(f, cfg) - _pitch_holes(f, cfg)
