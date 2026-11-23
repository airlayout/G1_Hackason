"""スマホを（カバーごと）縦に入れる袋。上から差し込み、上の端をヘアゴムで押さえる。

形（座標は dims.py の頭の説明）:
    - 左右の溝: 側壁＋前（画面側）と後（背面側）の縁。縁はカバーの面に 3 mm かぶさる
    - 側面のボタンの帯は、側壁の内側を 3.5 mm 逃がして押さない（壁は残すので溝は硬いまま）
    - 背面のカメラの帯は、背面側の縁を外して写り込まない
    - 下: 底（充電の線を通す切り欠き）・背面の受け・背面の中ほどの帯
    - 上の端の外側: ヘアゴムを掛ける鉤（下向きに口が開く）
    - 胸の板に留める耳（下と中ほどの 2 か所）: 画面側の外の角から後ろへ出す。回す軸（上下）は
      外の側壁の面の 8 mm 後ろ。下の耳には向きを決める穴（軸から内へ 28 mm）もある

刷る向き: 背面（+y）を台に置く。縁・壁・耳はどれも立った壁になり、支えが要らない。
"""

from __future__ import annotations

from dataclasses import dataclass

from build123d import Box, Cylinder, Part, Plane, Polygon, Pos, extrude

from dims import HW, PhoneSlot


@dataclass(frozen=True)
class PocketCfg:
    clear_w: float = 1.0  # 幅の隙間（左右の合計）
    clear_t: float = 0.8  # 厚みの隙間（前後の合計）
    side: float = 2.4  # 側壁の残す厚み
    relief: float = 3.5  # ボタンの帯で側壁の内側を逃がす深さ
    lip_t: float = 2.0  # 縁の厚み
    lip_w: float = 3.0  # 縁がカバーの面にかぶさる幅
    floor: float = 4.0
    cup_h: float = 18.0  # 下の背面の受けの高さ
    brace_h: float = 12.0  # 背面の中ほどの帯の高さ
    brace_frac: float = 0.42  # 帯の位置（袋の高さに対する割合）
    rail_gap: float = 6.0  # 溝の上の端を、カバーの上の端からどれだけ下げるか（ゴムがカバーに掛かるように）
    cable_w: float = 19.0  # 底の切り欠き（USB-C の差し込み口が通る幅）
    pivot_back: float = 8.0  # 回す軸を画面側の面からどれだけ後ろに置くか（回しても袋が後ろへ動かない）
    index_r: float = 28.0  # 向きを決める穴の、軸からの距離（内へ）
    tab_r: float = 6.0  # 耳の穴のまわりの肉
    tab_z: float = 100.0  # 中ほどの耳の下の面の高さ（ボタン・鉤と離す）
    tab_t: float = 5.0  # 中ほどの耳の厚み
    margin: float = 4.0  # ボタン・カメラの帯の余り
    case_back: float = 1.2  # カバーの背面の厚み（見積もり。写り込みの確かめにだけ使う）


CFG = PocketCfg()


@dataclass(frozen=True)
class Inner:
    """袋の内側の寸法と、カバーの中の本体の位置。"""

    w: float
    t: float
    h: float
    wo: float  # 外側の半分の幅
    to: float  # 外側の半分の厚み
    phone_top: float  # 本体の上の端の z
    phone_back: float  # 本体の背面の y
    rail_top: float


def inner(slot: PhoneSlot, cfg: PocketCfg = CFG) -> Inner:
    p, c = slot.phone, slot.case
    w = c.width + cfg.clear_w
    t = c.thickness + cfg.clear_t
    return Inner(
        w=w,
        t=t,
        h=c.height,
        wo=w / 2 + cfg.relief + cfg.side,
        to=t / 2 + cfg.lip_t,
        phone_top=(c.height - p.height) / 2 + p.height,
        phone_back=c.thickness / 2 - cfg.case_back,
        rail_top=c.height - cfg.rail_gap,
    )


def _box(x0: float, x1: float, y0: float, y1: float, z0: float, z1: float) -> Part:
    return Pos((x0 + x1) / 2, (y0 + y1) / 2, (z0 + z1) / 2) * Box(x1 - x0, y1 - y0, z1 - z0)


def _rails(d: Inner, cfg: PocketCfg) -> Part:
    """左右の溝（側壁＋前後の縁）。"""
    part = None
    for s in (-1, 1):
        lo, hi = sorted((s * (d.w / 2 - cfg.lip_w), s * d.wo))
        web = _box(*sorted((s * d.w / 2, s * d.wo)), -d.to, d.to, 0, d.rail_top)
        front = _box(lo, hi, -d.to, -d.t / 2, 0, d.rail_top)
        back = _box(lo, hi, d.t / 2, d.to, 0, d.rail_top)
        side = web + front + back
        part = side if part is None else part + side
    return part


def _bottom(d: Inner, cfg: PocketCfg) -> Part:
    """底（線の切り欠き）・下の背面の受け・背面の中ほどの帯。"""
    floor = _box(-d.wo, d.wo, -d.to, d.to, -cfg.floor, 0)
    floor = floor - _box(-cfg.cable_w / 2, cfg.cable_w / 2, -d.t / 2, d.t / 2, -cfg.floor - 1, 1)
    cup = _box(-d.w / 2, d.w / 2, d.t / 2, d.to, 0, cfg.cup_h)
    zb = cfg.brace_frac * d.h
    brace = _box(-d.w / 2, d.w / 2, d.t / 2, d.to, zb, zb + cfg.brace_h)
    return floor + cup + brace


def pivot_local(slot: PhoneSlot, cfg: PocketCfg = CFG) -> tuple[float, float]:
    """回す軸（上下）の位置（袋の座標の x・y）。外の側壁の面の、画面側の面から pivot_back 後ろ。"""
    d = inner(slot, cfg)
    return slot.side * d.wo, -(d.to + cfg.pivot_back)


def index_local(slot: PhoneSlot, cfg: PocketCfg = CFG) -> tuple[float, float]:
    """向きを決める穴の位置（軸から内へ index_r）。"""
    px, py = pivot_local(slot, cfg)
    return px - slot.side * cfg.index_r, py


def _hole(x: float, y: float, z0: float, z1: float) -> Part:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(HW.hole / 2, z1 - z0 + 2)


def _tabs(slot: PhoneSlot, d: Inner, cfg: PocketCfg) -> tuple[Part, Part]:
    """下の耳（底と同じ高さ・軸と向きの穴）と中ほどの耳（軸の穴）。返すのは (肉, 穴)。"""
    s, r = slot.side, cfg.tab_r
    px, py = pivot_local(slot, cfg)
    ix, _ = index_local(slot, cfg)
    lo_x, hi_x = sorted((ix - s * r, px + s * r))
    bottom = _box(lo_x, hi_x, py - r, -d.to + 0.01, -cfg.floor, 0)
    top_x = sorted((s * (d.wo - 6.0), px + s * r))
    top = _box(*top_x, py - r, -d.to + 3.0, cfg.tab_z, cfg.tab_z + cfg.tab_t)
    holes = _hole(px, py, -cfg.floor, 0) + _hole(ix, py, -cfg.floor, 0)
    holes = holes + _hole(px, py, cfg.tab_z, cfg.tab_z + cfg.tab_t)
    return bottom + top, holes


def _hooks(d: Inner) -> Part:
    """上の端の外側の鉤（ゴムは屋根の下に掛ける。屋根の上は斜めに埋めて刷れるようにする）。"""
    zh = d.h - 16.0
    part = None
    for s in (-1, 1):
        # 屋根（zh..zh+3）・外の端の下向きの返し（zh−4..zh）・屋根の上の斜めの埋めを 1 つの輪郭で
        x0, x5, x7 = d.wo - 0.5, d.wo + 5.0, d.wo + 7.0
        pts = [(x0, zh), (x5, zh), (x5, zh - 4), (x7, zh - 4), (x7, zh + 3), (x0, zh + 10)]
        outline = Polygon(*[(s * x, z) for x, z in pts], align=None)
        hook = extrude(Plane.XZ * outline, amount=d.to, both=True)
        part = hook if part is None else part + hook
    return part


def _button_cuts(slot: PhoneSlot, d: Inner, cfg: PocketCfg) -> Part | None:
    """ボタンの帯で側壁の内側を逃がす（画面から見て右＝+x、左＝−x）。"""
    cuts = None
    for b in slot.phone.buttons:
        s = 1 if b.side == "R" else -1
        z0 = d.phone_top - b.top_to - cfg.margin
        z1 = d.phone_top - b.top_from + cfg.margin
        x0, x1 = sorted((s * (d.w / 2 - 0.01), s * (d.w / 2 + cfg.relief)))
        cut = _box(x0, x1, -d.t / 2, d.t / 2, z0, z1)
        cuts = cut if cuts is None else cuts + cut
    return cuts


def camera_span(slot: PhoneSlot, d: Inner, cfg: PocketCfg = CFG) -> tuple[float, float, float]:
    """背面のカメラの帯の x の範囲と、下の端の z（余りを足す）。"""
    p = slot.phone
    bx0, _, bx1, by1 = p.cam_box
    x0, x1 = p.width / 2 - bx1 - cfg.margin, p.width / 2 - bx0 + cfg.margin
    return x0, x1, d.phone_top - by1 - cfg.margin


def _camera_cuts(slot: PhoneSlot, d: Inner, cfg: PocketCfg) -> Part | None:
    """カメラの帯にかかる側の、背面側の縁を外す（側壁は台に届くよう残す）。"""
    cx0, cx1, z0 = camera_span(slot, d, cfg)
    cuts = None
    for s in (-1, 1):
        lx0, lx1 = sorted((s * (d.w / 2 - cfg.lip_w), s * d.w / 2))
        if lx1 < cx0 or lx0 > cx1:
            continue
        cut = _box(lx0 - 0.01, lx1 + 0.01, d.t / 2 - 0.01, d.to + 1, z0, d.h + 10)
        cuts = cut if cuts is None else cuts + cut
    return cuts


def pocket(slot: PhoneSlot, cfg: PocketCfg = CFG) -> Part:
    d = inner(slot, cfg)
    tabs, holes = _tabs(slot, d, cfg)
    part = _rails(d, cfg) + _bottom(d, cfg) + tabs + _hooks(d)
    for cuts in (_button_cuts(slot, d, cfg), _camera_cuts(slot, d, cfg), holes):
        if cuts is not None:
            part = part - cuts
    return part


def fit_slice(slot: PhoneSlot, cfg: PocketCfg = CFG, z0: float = 3.0, h: float = 12.0) -> Part:
    """試し刷り: 袋の下の受けの輪切り（幅・厚みの隙間を確かめる）。z が上の向きで刷る。"""
    d = inner(slot, cfg)
    ring = _rails(d, cfg) + _bottom(d, cfg)
    return ring & _box(-d.wo - 1, d.wo + 1, -d.to - 1, d.to + 1, z0, z0 + h)
