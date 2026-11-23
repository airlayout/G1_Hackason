"""部品を刷る向きに置いて書き出す（STL・STEP）。確かめの絵も描く。

    .venv/bin/python build.py            # out/ に書き出す
    .venv/bin/python build.py --no-render

出す物（out/）:
    print/*.stl・*.step   刷る向きに置いた部品（Bambu Studio にそのまま入れる）
    assembly.step         着た形の組み立て（見る用）
    preview/*.png         絵
"""

from __future__ import annotations

import argparse
from pathlib import Path

from build123d import Part, Pos, Rot, export_step, export_stl

from assembly import parts as assembled
from chest import CCFG, back, carrier
from dims import LEFT, RIGHT
from pocket import fit_slice, pocket

OUT = Path(__file__).resolve().parent / "out"


def on_bed(part: Part) -> Part:
    """一番下を z=0 に、床の真ん中へ。"""
    bb = part.bounding_box()
    return Pos(-bb.center().X, -bb.center().Y, -bb.min.Z) * part


def print_parts() -> dict[str, tuple[Part, int]]:
    """名前 → (刷る向きの部品, 刷る数)。"""
    lay_back = Rot(-90, 0, 0)  # 袋: +y（背面）を下へ
    lay_plate = Rot(90 - CCFG.tilt, 0, 0)  # 背板の部品: 背板の裏を下へ（腕木は台から 68° に立つ）
    lay_carrier = Rot(90, 0, 0)  # 台: 後ろの面（−y）を下へ
    return {
        "back": (on_bed(lay_plate * back()), 1),
        "carrier": (on_bed(lay_carrier * carrier()), 1),
        "pocket_iphone11pro_left": (on_bed(lay_back * pocket(LEFT)), 1),
        "pocket_pixel7a_right": (on_bed(lay_back * pocket(RIGHT)), 1),
        "fitslice_iphone11pro": (on_bed(fit_slice(LEFT)), 1),
        "fitslice_pixel7a": (on_bed(fit_slice(RIGHT)), 1),
    }


def export_all(items: dict[str, tuple[Part, int]]) -> None:
    pdir = OUT / "print"
    pdir.mkdir(parents=True, exist_ok=True)
    for name, (part, _) in items.items():
        export_stl(part, str(pdir / f"{name}.stl"), tolerance=0.02, angular_tolerance=0.1)
        export_step(part, str(pdir / f"{name}.step"))
    from build123d import Compound

    asm = assembled(0, 0, 0)
    export_step(Compound([p for n, p in asm.items() if not n.startswith("phone")]), str(OUT / "assembly.step"))


def render_all(items: dict[str, tuple[Part, int]]) -> None:
    from render import draw

    prev = OUT / "preview"
    views = [(8, 90, "前から"), (25, 55, "斜め前から"), (5, 0, "右から"), (88, -90, "上から（上が前）")]
    for a in CCFG.angles:
        draw(assembled(a, a, 0), prev / f"assembly_a{a:.0f}.png", views,
             f"胸の板（外向き {a:.0f}°・上下 0°・左 iPhone 11 Pro／右 Pixel 7a、スマホは紺の箱）")
    for p in (min(CCFG.pitches), max(CCFG.pitches)):
        draw(assembled(0, 0, p), prev / f"assembly_p{p:+.0f}.png", views,
             f"胸の板（外向き 0°・上下 {p:+.0f}°。+ が上向き）")
    for name, (part, n) in items.items():
        draw({name: part}, prev / f"part_{name}.png", [(30, -60, "刷る向き"), (89, -90, "上から")],
             f"{name}（{n} 個）")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--no-render", action="store_true")
    args = ap.parse_args()
    items = print_parts()
    export_all(items)
    for name, (part, n) in items.items():
        bb = part.bounding_box()
        print(f"[build] {name}: {bb.size.X:.0f}×{bb.size.Y:.0f}×{bb.size.Z:.0f} mm・{part.volume / 1000:.1f} cm³・{n} 個")
    if not args.no_render:
        render_all(items)
    print(f"[build] 書き出し: {OUT}")


if __name__ == "__main__":
    main()
