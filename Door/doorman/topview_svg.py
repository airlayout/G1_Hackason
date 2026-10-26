"""上から見た図（扉・カメラの水平の視野）を 8 コマ並べた SVG を作る。色は HTML 側の CSS 変数を使う。"""
import json
import math
import sys

STAGE = {0: "扉へ歩く", 1: "構える", 2: "レバーに手", 3: "レバーを下げる", 4: "押して開ける", 5: "通り抜ける"}
HALF_FOV = 43.5
FOV_LEN = 0.75
SCALE = 100          # 1 m あたりの px
CELL_W, CELL_H = 250, 320
X_MIN, X_MAX = -1.35, 1.35   # 扉の法線方向の範囲 [m]
Y_C = 0.0


def to_px(x, y, ox, oy):
    return ox + CELL_W / 2 - (y - Y_C) * SCALE, oy + 48 + (X_MAX - x) * SCALE


def cell(fr, w, ox, oy):
    out = []
    P = lambda x, y: to_px(x, y, ox, oy)
    # 壁と枠
    for y0, y1 in [(w / 2, w / 2 + 0.9), (-w / 2, -w / 2 - 0.9)]:
        a, b = P(0, y0), P(0, y1)
        out.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" class="tv-wall"/>')
    # 扉の板（蝶番は -w/2）
    th = math.radians(fr["hinge"])
    hx, hy = 0.0, -w / 2
    ex, ey = hx + w * math.sin(th), hy + w * math.cos(th)
    a, b = P(hx, hy), P(ex, ey)
    out.append(f'<line x1="{a[0]:.1f}" y1="{a[1]:.1f}" x2="{b[0]:.1f}" y2="{b[1]:.1f}" class="tv-door"/>')
    out.append(f'<circle cx="{a[0]:.1f}" cy="{a[1]:.1f}" r="4" class="tv-hinge"/>')
    lv = P(*fr["lever"])
    out.append(f'<circle cx="{lv[0]:.1f}" cy="{lv[1]:.1f}" r="5" class="tv-lever"/>')
    # カメラの水平の視野
    cx, cy = fr["cam"]
    hd = math.radians(fr["head"])
    pts = [P(cx, cy)]
    for k in range(13):
        ang = hd + math.radians(-HALF_FOV + k * 2 * HALF_FOV / 12)
        pts.append(P(cx + FOV_LEN * math.cos(ang), cy + FOV_LEN * math.sin(ang)))
    out.append('<polygon points="' + " ".join(f"{p[0]:.1f},{p[1]:.1f}" for p in pts) + '" class="tv-fov"/>')
    c = P(cx, cy)
    f = P(cx + 0.28 * math.cos(hd), cy + 0.28 * math.sin(hd))
    out.append(f'<line x1="{c[0]:.1f}" y1="{c[1]:.1f}" x2="{f[0]:.1f}" y2="{f[1]:.1f}" class="tv-axis"/>')
    out.append(f'<circle cx="{c[0]:.1f}" cy="{c[1]:.1f}" r="6" class="tv-cam"/>')
    out.append(f'<text x="{ox + 12}" y="{oy + 20}" class="tv-t">{fr["t"]:.1f} 秒 · 区間 {fr["stage"]}</text>')
    out.append(f'<text x="{ox + 12}" y="{oy + 38}" class="tv-s">{STAGE[fr["stage"]]} · 扉 {fr["hinge"]:.0f}° · 向き {fr["head"]:+.0f}°</text>')
    return out


def main(src, dst):
    d = json.load(open(src))
    frames, w = d["frames"], d["width"]
    cols = 4
    rows = math.ceil(len(frames) / cols)
    W, H = cols * CELL_W, rows * CELL_H
    body = []
    for i, fr in enumerate(frames):
        ox, oy = (i % cols) * CELL_W, (i // cols) * CELL_H
        body.append(f'<clipPath id="tvc{i}"><rect x="{ox + 3}" y="{oy + 3}" width="{CELL_W - 6}" height="{CELL_H - 6}" rx="6"/></clipPath>')
        body.append(f'<rect x="{ox + 3}" y="{oy + 3}" width="{CELL_W - 6}" height="{CELL_H - 6}" rx="6" class="tv-bg"/>')
        body.append(f'<g clip-path="url(#tvc{i})">' + "".join(cell(fr, w, ox, oy)) + '</g>')
    svg = (f'<svg viewBox="0 0 {W} {H}" role="img" aria-labelledby="tvTitle tvDesc" class="topview">'
           '<title id="tvTitle">上から見た扉と頭のカメラの視野（成功した 1 回、8 つの時刻）</title>'
           '<desc id="tvDesc">扉へ歩く間と構える間はカメラの扇が扉とレバーを向く。押して開ける間（6.7〜9.7 秒）は体が −81〜−129° 向きを変え、'
           '扇は蝶番の側と元の部屋を向き、動く戸先とレバーは扇の外になる。</desc>'
           + "".join(body) + "</svg>")
    open(dst, "w").write(svg)
    print(dst, len(svg))


if __name__ == "__main__":
    main(sys.argv[1], sys.argv[2])
