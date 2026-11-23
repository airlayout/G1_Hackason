"""胸の板に 2 台を載せた形。袋ごとの外向きの角度（0・10・20°）と、台ごとの上下の角度（−20〜+10°）を選べる。

左に iPhone 11 Pro（cam1）、右に Pixel 7a（cam2）。座標は chest.py の背板の部品の座標。
"""

from __future__ import annotations

from build123d import Box, Location, Part, Pos, Rot

from chest import back, carrier, carrier_loc, center_x, pivot, yaw_sign
from dims import LEFT, RIGHT, PhoneSlot
from pocket import pocket


def pocket_loc(slot: PhoneSlot, angle: float = 0.0) -> Location:
    """袋の座標 → 台の座標。袋を並べた位置に置き、回す軸のまわりに外向き angle 度回す。"""
    px, py = pivot(slot)
    return Pos(px, py, 0) * Rot(0, 0, yaw_sign(slot) * angle) * Pos(-px, -py, 0) * Pos(center_x(slot), 0, 0)


def phone_loc(slot: PhoneSlot, angle: float = 0.0, pitch: float = 0.0) -> Location:
    """袋の座標 → 背板の部品の座標（上下 pitch 度・外向き angle 度）。"""
    return carrier_loc(pitch) * pocket_loc(slot, angle)


def case_box(slot: PhoneSlot) -> Part:
    """カバー付きのスマホの外形（袋の座標）。"""
    c = slot.case
    return Pos(0, 0, c.height / 2) * Box(c.width, c.thickness, c.height)


def parts(angle_left: float = 0.0, angle_right: float = 0.0, pitch: float = 0.0) -> dict[str, Part]:
    """名前 → 背板の部品の座標に置いた部品。"""
    out: dict[str, Part] = {"back": back(), "carrier": carrier_loc(pitch) * carrier()}
    for name, slot, a in (("left", LEFT, angle_left), ("right", RIGHT, angle_right)):
        loc = phone_loc(slot, a, pitch)
        out[f"pocket_{name}"] = loc * pocket(slot)
        out[f"phone_{name}"] = loc * case_box(slot)
    return out
