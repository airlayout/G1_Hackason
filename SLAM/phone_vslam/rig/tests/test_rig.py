"""図面の確かめ。公式の寸法（Apple の図面・Google の外形・Google 純正カバー）を袋や胸の板と突き合わせる。

- カバー付きのスマホが上から入り、前後左右・下に動けない
- iPhone 11 Pro の側面のボタン（公式の位置）を押さない
- iPhone 11 Pro のレンズの写り込みの円錐（公式）に何も入らない。Pixel 7a はカメラの帯の前に何も無い
- 左右の向き: 回す軸の穴が上下で一直線、向きの穴が 0・10・20° のどれでも合う
- 上下の向き: 軸と、その角度で使う穴が、腕木と台で一直線（−20〜+10° を 5° 刻み）
- 左右と上下のどの組でもぶつからず、2 台の間が空いている
- どの部品も A2L の床に収まり、支え無しで刷れる
"""

from __future__ import annotations

import math
import sys
from pathlib import Path

import pytest
from build123d import Box, Cone, Cylinder, Part, Pos, Rot

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from assembly import case_box, parts, phone_loc, pocket_loc  # noqa: E402
from chest import (  # noqa: E402
    CCFG,
    back,
    back_y,
    carrier,
    carrier_loc,
    frame_dims,
    hole_yz,
    index_at,
    pitch_hole,
    pivot,
    plate_dy,
)
from dims import A2L, HW, IPHONE_11_PRO, LEFT, RIGHT, PhoneSlot  # noqa: E402
from pocket import CFG, camera_span, inner, pocket  # noqa: E402

EPS = 1e-3  # mm³。これより大きい重なりはぶつかり


def overlap(a: Part, b: Part) -> float:
    inter = a & b
    return 0.0 if inter is None else inter.volume


@pytest.fixture(scope="module")
def pockets() -> dict[str, Part]:
    return {s.label: pocket(s) for s in (LEFT, RIGHT)}


SLOTS = [LEFT, RIGHT]


@pytest.mark.parametrize("slot", SLOTS, ids=lambda s: s.label)
def test_case_slides_in_from_top(slot: PhoneSlot, pockets: dict[str, Part]) -> None:
    c = slot.case
    path = Pos(0, 0, (c.height + 300) / 2) * Box(c.width, c.thickness, c.height + 300)
    assert overlap(path, pockets[slot.label]) < EPS


@pytest.mark.parametrize("slot", SLOTS, ids=lambda s: s.label)
@pytest.mark.parametrize(
    "shift",
    [(0, 1.0, 0), (0, -1.0, 0), (0.9, 0, 0), (-0.9, 0, 0), (0, 0, -0.5)],
    ids=["前", "後", "右", "左", "下"],
)
def test_case_is_held(slot: PhoneSlot, shift: tuple[float, float, float], pockets: dict[str, Part]) -> None:
    moved = Pos(*shift) * case_box(slot)
    assert overlap(moved, pockets[slot.label]) > 1.0


def _button_bumps(slot: PhoneSlot, push: float) -> Part:
    """カバーのボタンの出っ張り（側面から 1.0 mm、見積もり）。push は袋の中でその側へ寄せる量。"""
    d = inner(slot)
    c = slot.case
    part = None
    for b in slot.phone.buttons:
        s = 1 if b.side == "R" else -1
        z0, z1 = d.phone_top - b.top_to, d.phone_top - b.top_from
        x_face = s * (c.width / 2 + push)
        bump = Pos(x_face + s * 0.5, 0, (z0 + z1) / 2) * Box(1.0, 5.0, z1 - z0)
        part = bump if part is None else part + bump
    return part


@pytest.mark.parametrize("slot", SLOTS, ids=lambda s: s.label)
def test_buttons_not_pressed(slot: PhoneSlot, pockets: dict[str, Part]) -> None:
    # 一番悪い場合: スマホがボタンの側へ隙間いっぱい寄った
    push = CFG.clear_w / 2
    assert overlap(_button_bumps(slot, push), pockets[slot.label]) < EPS


def _lens_cone(slot: PhoneSlot, lens, height: float = 40.0) -> Part:
    d = inner(slot)
    p = slot.phone
    x = p.width / 2 - lens.bx  # 背面から見た左は +x
    z = d.phone_top - lens.by
    y_apex = d.phone_back - p.thickness + lens.apex_from_front
    r = height * math.tan(math.radians(lens.cone_deg / 2))
    # Cone は底が z=0・頂点が +z。頂点を原点にして +y へ開く向きに回す
    cone = Pos(0, 0, -height) * Cone(r, 0.0, height, align=None)  # 頂点が原点・−z へ開く
    return Pos(x, y_apex, z) * Rot(90, 0, 0) * cone  # X 軸で 90°: −z → +y


def test_lens_cone_orientation() -> None:
    """円錐が背面の向き（+y）に開いていること（確かめの道具の確かめ）。"""
    cone = _lens_cone(LEFT, IPHONE_11_PRO.lenses[0])
    bb = cone.bounding_box()
    d = inner(LEFT)
    y_apex = d.phone_back - IPHONE_11_PRO.thickness + IPHONE_11_PRO.lenses[0].apex_from_front
    assert bb.min.Y == pytest.approx(y_apex, abs=1e-3)
    assert bb.max.Y > y_apex + 30


@pytest.mark.parametrize("lens", IPHONE_11_PRO.lenses, ids=lambda lens: lens.name)
def test_iphone_lens_cones_clear(lens, pockets: dict[str, Part]) -> None:
    assert overlap(_lens_cone(LEFT, lens), pockets[LEFT.label]) < EPS


@pytest.mark.parametrize("slot", SLOTS, ids=lambda s: s.label)
def test_camera_band_front_is_open(slot: PhoneSlot, pockets: dict[str, Part]) -> None:
    """背面のカメラの帯の真ん前（カバーの外形の中）に袋が無い。"""
    d = inner(slot)
    c = slot.case
    x0, x1, z0 = camera_span(slot, d)
    x0, x1 = max(x0, -c.width / 2), min(x1, c.width / 2)
    z1 = d.phone_top
    zone = Pos((x0 + x1) / 2, c.thickness / 2 + 10, (z0 + CFG.margin + z1) / 2)
    region = zone * Box(x1 - x0, 20, z1 - z0 - CFG.margin)
    assert overlap(region, pockets[slot.label]) < EPS


ANGLES = [(0, 0), (10, 10), (20, 20), (0, 20), (20, 0)]
PITCH_ENDS = [-20.0, 10.0]
# 左右の組 × 上下の両端、と、上下の全部（左右 0°）
COMBOS = [(a, p) for a in ANGLES for p in PITCH_ENDS] + [((0, 0), p) for p in CCFG.pitches]


@pytest.fixture(scope="module")
def carrier_part() -> Part:
    return carrier()


@pytest.fixture(scope="module")
def back_part() -> Part:
    return back()


@pytest.mark.parametrize("combo", COMBOS, ids=lambda c: f"左{c[0][0]}右{c[0][1]}上下{c[1]:+.0f}")
def test_no_collision(combo) -> None:
    (al, ar), pitch = combo
    ps = parts(al, ar, pitch)
    names = list(ps)
    hits = []
    for i, a in enumerate(names):
        for b in names[i + 1:]:
            if {a, b} in ({"pocket_left", "phone_left"}, {"pocket_right", "phone_right"}):
                continue  # 袋とスマホは別の試験（保たれる・隙間）
            v = overlap(ps[a], ps[b])
            if v > EPS:
                hits.append((a, b, round(v, 2)))
    assert hits == []


def _zpin(x: float, y: float, z0: float, z1: float, dia: float) -> Part:
    return Pos(x, y, (z0 + z1) / 2) * Cylinder(dia / 2, z1 - z0)


def _xpin(y: float, z: float, x0: float, x1: float, dia: float) -> Part:
    return Pos((x0 + x1) / 2, y, z) * Rot(0, 90, 0) * Cylinder(dia / 2, x1 - x0)


@pytest.mark.parametrize("slot", [LEFT, RIGHT], ids=lambda s: s.label)
@pytest.mark.parametrize("angle", list(CCFG.angles))
def test_yaw_holes_line_up(slot: PhoneSlot, angle: float, carrier_part: Part) -> None:
    """左右: 軸の穴（下と中ほど）・向きの穴（その角度の穴）が、袋と台で一直線（細い軸は素通り・太い軸は両方に当たる）。"""
    f = frame_dims()
    placed = pocket_loc(slot, angle) * pocket(slot)
    px, py = pivot(slot)
    ix, iy = index_at(slot, angle)
    z_lo, z_hi = f.z0 - 1, 0.5  # 下の棚＋袋の下の耳
    z_tlo, z_thi = f.z_top - CCFG.top_t - 1, f.z_top + CFG.tab_t + 1  # 中ほどの棚＋袋の中ほどの耳
    for x, y, z0, z1 in ((px, py, z_lo, z_hi), (ix, iy, z_lo, z_hi), (px, py, z_tlo, z_thi)):
        thin = _zpin(x, y, z0, z1, HW.hole - 0.5)
        thick = _zpin(x, y, z0, z1, HW.hole + 4)
        assert overlap(thin, placed) < EPS and overlap(thin, carrier_part) < EPS, (x, y, z0)
        assert overlap(thick, placed) > 1 and overlap(thick, carrier_part) > 1, (x, y, z0)


@pytest.mark.parametrize("pitch", list(CCFG.pitches))
def test_pitch_holes_line_up(pitch: float, carrier_part: Part, back_part: Part) -> None:
    """上下: 軸と、その角度で使う穴が、腕木と（回した）台の外の側板で一直線。左右とも。"""
    f = frame_dims()
    placed = carrier_loc(pitch) * carrier_part
    r, alpha = pitch_hole(pitch)
    for y, z in (f.hinge, hole_yz(r, alpha)):
        for s in (-1, 1):
            x0, x1 = sorted((s * (f.x_side + 1.0), s * (f.x_plate + 1.0)))
            thin = _xpin(y, z, x0, x1, HW.hole - 0.5)
            thick = _xpin(y, z, x0, x1, HW.hole + 4)
            assert overlap(thin, placed) < EPS and overlap(thin, back_part) < EPS, (pitch, s, y, z)
            assert overlap(thick, placed) > 1 and overlap(thick, back_part) > 1, (pitch, s, y, z)


def test_pitch_holes_wrong_angle_blocked(carrier_part: Part) -> None:
    """確かめの道具の確かめ: 用意していない角度（−12°）では、台の穴は腕木のどの穴とも合わない。"""
    f = frame_dims()
    placed = carrier_loc(-12.0) * carrier_part
    for r in (CCFG.r1, CCFG.r2):
        y, z = hole_yz(r, -12.0)
        thin = _xpin(y, z, f.x_side + 1.0, f.x_plate + 1.0, HW.hole - 0.5)
        assert overlap(thin, placed) < EPS  # 台の穴は通る
        assert overlap(thin, back()) > 1  # 腕木の穴は無い


def test_pitch_steps_cover_range() -> None:
    """−20〜+10° を 5° 刻み。10° の倍数は内の弧、5° 余りは外の弧。腕木の穴どうしは肉が残る。"""
    assert CCFG.pitches == (-20.0, -15.0, -10.0, -5.0, 0.0, 5.0, 10.0)
    for p in CCFG.pitches:
        assert pitch_hole(p)[0] == (CCFG.r1 if p % 10 == 0 else CCFG.r2)
    pts = [hole_yz(*pitch_hole(p)) for p in CCFG.pitches] + [frame_dims().hinge]
    gaps = [math.dist(a, b) for i, a in enumerate(pts) for b in pts[i + 1:]]
    assert min(gaps) >= HW.hole + 2.0, min(gaps)


def test_pockets_sit_on_shelf() -> None:
    """袋の底は下の棚の上の面に載る（宙に浮かない・めり込まない）。"""
    top_of_shelf = frame_dims().z0 + CCFG.shelf_t
    for slot in (LEFT, RIGHT):
        assert pocket(slot).bounding_box().min.Z == pytest.approx(top_of_shelf, abs=1e-6)


@pytest.mark.parametrize("pitch", PITCH_ENDS + [0.0])
@pytest.mark.parametrize("angle", list(CCFG.angles))
@pytest.mark.parametrize("lens", IPHONE_11_PRO.lenses, ids=lambda lens: lens.name)
def test_iphone_lens_cones_clear_of_rig(pitch: float, angle: float, lens) -> None:
    """iPhone のレンズの円錐（公式）に、背板の部品・台・もう 1 台の袋とスマホが入らない。"""
    ps = parts(angle, angle, pitch)
    cone = phone_loc(LEFT, angle, pitch) * _lens_cone(LEFT, lens)
    for name in ("back", "carrier", "pocket_right", "phone_right"):
        assert overlap(cone, ps[name]) < EPS, name


def test_stereo_baseline_about_12cm() -> None:
    """向き 0° の 2 台の主カメラの間（iPhone は公式、Pixel は見積もり）。"""
    from chest import center_x

    xl = center_x(LEFT) + IPHONE_11_PRO.width / 2 - 12.04
    xr = center_x(RIGHT) + 72.9 / 2 - 13.0
    assert 110 <= xr - xl <= 125


@pytest.mark.parametrize("pitch", PITCH_ENDS + [0.0])
def test_gap_between_pockets_is_open(pitch: float) -> None:
    """2 つの袋の間は棚から梁まで空いている（前から見て向こうが見え、風が抜ける）。どの部品も入らない。"""
    f = frame_dims()
    x = CCFG.gap / 2 - 0.5
    z0, z1 = f.z0 + CCFG.shelf_t + 0.5, f.z_top - CCFG.top_t - 0.5
    gap_box = carrier_loc(pitch) * Pos(0, (f.y_top_front + 0.5 + CCFG.front) / 2, (z0 + z1) / 2) * Box(
        2 * x, CCFG.front - f.y_top_front - 0.5, z1 - z0)
    ps = parts(0, 0, pitch)
    hits = {name: round(overlap(gap_box, part), 2) for name, part in ps.items() if overlap(gap_box, part) > EPS}
    assert hits == {}
    assert CCFG.gap >= 25.0


def _fits_bed(part: Part) -> bool:
    bb = part.bounding_box()
    a, b, c = sorted((bb.size.X, bb.size.Y, bb.size.Z))
    return c <= A2L.x and b <= A2L.y and a <= A2L.z


@pytest.mark.parametrize("name,make", [("back", back), ("carrier", carrier),
                                       ("pocket_left", lambda: pocket(LEFT)), ("pocket_right", lambda: pocket(RIGHT))])
def test_parts_valid_and_fit_bed(name: str, make) -> None:
    part = make()
    assert part.is_valid, name
    assert len(part.solids()) == 1, name
    assert _fits_bed(part), name


def test_back_is_flat(back_part: Part) -> None:
    """背板の裏の面より後ろに出る所が無い（刷るときは裏の面が台に載り、胸には平らな面が当たる）。"""
    f = frame_dims()
    th = math.radians(CCFG.tilt)
    worst = min((v.Y - (back_y(v.Z, f) - plate_dy())) * math.cos(th) for v in back_part.vertices())
    assert worst > -1e-4, worst


@pytest.mark.parametrize("name", ["back", "carrier"])
def test_prints_without_supports(name: str) -> None:
    """刷る向きで、下を向いた面（45° より寝た面）は台の上の面か、長さ 52 mm 以下の橋だけ。"""
    from build import print_parts

    part = print_parts()[name][0]
    long_bridges = []
    for face in part.faces():
        c = face.center()
        if face.normal_at(c).Z < -0.71 and c.Z > 1e-3:
            bb = face.bounding_box()
            length = max(bb.size.X, math.hypot(bb.size.Y, bb.size.Z))
            if length > 52.0:
                long_bridges.append((round(length, 1), round(c.X, 1), round(c.Z, 1)))
    assert long_bridges == []


def test_strap_slots_take_25mm_webbing() -> None:
    assert HW.strap_w >= 25.5 and HW.strap_t >= 3.0


def test_angle_steps() -> None:
    assert CCFG.angles == (0.0, 10.0, 20.0) and math.isclose(CCFG.tilt, 22.0)
