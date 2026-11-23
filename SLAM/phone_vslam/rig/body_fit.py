"""胸の板が胸に沿うかを、実在の人体の形（MakeHuman の標準の体・CC0）で確かめる。

体の座標を図面と合わせる（x 右・y 前・z 上・mm）。胸の板の上の端を胸骨の上の切り欠きの少し下に置き、
後ろへ押して胸に最初に当たる所を求める。そこで背板の場所ごとの隙間・食い込み・カメラの写り込みを測る。

    .venv/bin/python body_fit.py            # 表と絵（out/preview/body_fit_*.png）

体は身長 1,666 mm・腕を少し開いた形（A ポーズ）・止まった形。下を向いたときのあご、胸の大きさ、
服の厚みは入っていない。
"""

from __future__ import annotations

import urllib.request
from pathlib import Path

import numpy as np
import trimesh

from assembly import parts as assembled
from assembly import phone_loc
from chest import CCFG
from dims import IPHONE_11_PRO, LEFT, PIXEL_7A, RIGHT
from pocket import inner

URL = "https://raw.githubusercontent.com/makehumancommunity/makehuman/master/makehuman/data/3dobjs/base.obj"
CACHE = Path(__file__).resolve().parent / "out" / "body" / "makehuman_base.obj"
NOTCH_Z = 565.0  # 胸骨の上の切り欠きの高さ（体の座標。前の面が一番引っ込む所）
TOP_BELOW_NOTCH = 20.0  # 背板の上の端を切り欠きからどれだけ下げるか


def load_body() -> trimesh.Trimesh:
    if not CACHE.exists():
        CACHE.parent.mkdir(parents=True, exist_ok=True)
        urllib.request.urlretrieve(URL, CACHE)
    verts, faces, group = [], [], None
    for line in CACHE.read_text().splitlines():
        if line.startswith("v "):
            verts.append([float(x) for x in line.split()[1:4]])
        elif line.startswith("g "):
            group = line.split()[1]
        elif line.startswith("f ") and group == "body":
            idx = [int(t.split("/")[0]) - 1 for t in line.split()[1:]]
            faces.extend([idx[0], idx[i], idx[i + 1]] for i in range(1, len(idx) - 1))
    v = np.array(verts)[:, [0, 2, 1]] * 100.0  # dm・y 上 → mm・z 上
    return trimesh.Trimesh(v, np.array(faces), process=True)


def to_mesh(part) -> trimesh.Trimesh:
    verts, faces = part.tessellate(0.3, 0.3)
    return trimesh.Trimesh(np.array([[p.X, p.Y, p.Z] for p in verts]), np.array(faces), process=True)


def back_samples(frame_mesh: trimesh.Trimesh, n: int = 4000) -> np.ndarray:
    """背板の裏（後ろを向いた面）の点。"""
    back = frame_mesh.face_normals[:, 1] < -0.8
    sub = frame_mesh.submesh([np.where(back)[0]], append=True)
    pts, _ = trimesh.sample.sample_surface_even(sub, n)
    return pts


def place(body: trimesh.Trimesh, pts: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """背板を胸に後ろへ押し当てる移動（4x4）と、背板の点ごとの胸との隙間。"""
    z_shift = NOTCH_Z - TOP_BELOW_NOTCH - CCFG.plate_top
    moved = pts + [0, 400.0, z_shift]  # 体の前に出しておく
    dirs = np.tile([0.0, -1.0, 0.0], (len(moved), 1))
    locs, idx, _ = body.ray.intersects_location(moved, dirs, multiple_hits=False)
    dist = np.full(len(moved), np.inf)
    dist[idx] = moved[idx, 1] - locs[:, 1]
    push = float(np.min(dist))
    m = trimesh.transformations.translation_matrix([0, 400.0 - push, z_shift])
    return m, dist - push


def report_gaps(pts: np.ndarray, gaps: np.ndarray) -> None:
    """背板の場所ごとの隙間（上下・左右に 3 つずつに分けた一番小さい値）。"""
    x, z = pts[:, 0], pts[:, 2]
    xs = np.quantile(x, [0, 1 / 3, 2 / 3, 1])
    zs = np.quantile(z, [0, 1 / 3, 2 / 3, 1])
    print("背板と胸の隙間（mm・各区画で一番小さい値）  左 / 真ん中 / 右")
    for j, name in ((2, "上"), (1, "中"), (0, "下")):
        row = []
        for i in range(3):
            sel = (x >= xs[i]) & (x <= xs[i + 1]) & (z >= zs[j]) & (z <= zs[j + 1]) & np.isfinite(gaps)
            row.append(f"{gaps[sel].min():5.1f}" if sel.any() else "  なし")
        print(f"  {name}: " + " / ".join(row))


def _depth(body: trimesh.Trimesh, pts: np.ndarray) -> float:
    """体の中の点の深さ（前へ出る光線が体の面を抜けるまでの長さ）の一番大きい値。"""
    dirs = np.tile([0.0, 1.0, 0.0], (len(pts), 1))
    locs, idx, _ = body.ray.intersects_location(pts, dirs, multiple_hits=False)
    return float(np.max(locs[:, 1] - pts[idx, 1])) if len(idx) else 0.0


def check_inside(body: trimesh.Trimesh, m: np.ndarray, angle: float, pitch: float) -> None:
    """部品の面の点（決まった種で 3,000 点）が体の中に入るか。入った点は深さも出す。

    背板は一番近い点がちょうど胸に触れるよう置くので、触れる線の上の点が「中」と出ることがある（深さ 0.0x mm）。
    """
    ps = assembled(angle, angle, pitch)
    hits = []
    for name in ("back", "carrier", "pocket_left", "pocket_right", "phone_left", "phone_right"):
        mesh = to_mesh(ps[name])
        pts = trimesh.transform_points(trimesh.sample.sample_surface(mesh, 3000, seed=0)[0], m)
        inside = body.contains(pts)
        if inside.any():
            hits.append(f"{name}:{int(inside.sum())} 点・深さ {_depth(body, pts[inside]):.2f} mm")
    print(f"[食い込み] 外向き {angle:.0f}°・上下 {pitch:+.0f}°: {', '.join(hits) if hits else 'なし'}")


# 主カメラの位置（袋の座標の x と、本体の上の端からの距離）。iPhone は Apple の図面、Pixel は見積もり
LENS = {"left": (LEFT, IPHONE_11_PRO.width / 2 - 12.04, 12.04), "right": (RIGHT, PIXEL_7A.width / 2 - 13.0, 23.0)}


def view_check(body: trimesh.Trimesh, m: np.ndarray, a: float, pitch: float) -> None:
    """画角（縦向きの 720x1280。主カメラ 39×65°・超広角 66×102°）に体が写り込むか。"""
    for name in ("left", "right"):
        slot, lx, lby = LENS[name]
        d = inner(slot)
        lens = np.array([lx, slot.case.thickness / 2 + 0.5, d.phone_top - lby])
        mm = m @ _loc_matrix(phone_loc(slot, a, pitch))
        org = trimesh.transform_points(lens[None], mm)[0]
        for fov, hf, vf in (("主カメラ", 39.0, 65.0), ("超広角", 66.0, 102.0)):
            hs, vs = np.meshgrid(np.linspace(-hf / 2, hf / 2, 41), np.linspace(-vf / 2, vf / 2, 61))
            dl = np.column_stack([np.tan(np.radians(hs.ravel())), np.ones(hs.size), np.tan(np.radians(vs.ravel()))])
            dirs = dl @ mm[:3, :3].T
            dirs /= np.linalg.norm(dirs, axis=1, keepdims=True)
            _, idx, _ = body.ray.intersects_location(np.tile(org, (len(dirs), 1)), dirs, multiple_hits=False)
            print(f"[写り込み] {name} 外向き {a:.0f}°・上下 {pitch:+.0f}° {fov}: レンズの高さ {org[2]:.0f}・"
                  f"画角の中で体に当たる光線 {len(idx)}/{len(dirs)}")


def _loc_matrix(loc) -> np.ndarray:
    t = loc.wrapped.Transformation()
    return np.array([[t.Value(r, c) for c in range(1, 5)] for r in range(1, 4)] + [[0, 0, 0, 1]])


def draw_fit(body: trimesh.Trimesh, m: np.ndarray, angle: float, pitch: float, tag: str) -> None:
    from render import draw

    c = body.triangles_center
    keep = (c[:, 2] > 300) & (c[:, 2] < 850) & (np.abs(c[:, 0]) < 330)
    scene = {"body": body.submesh([np.where(keep)[0]], append=True)}
    for name, part in assembled(angle, angle, pitch).items():
        scene[name] = to_mesh(part).apply_transform(m)
    out = Path(__file__).resolve().parent / "out" / "preview" / f"body_fit_{tag}.png"
    draw(scene, out, [(5, 90, "前から"), (5, 0, "右から"), (25, 55, "斜め前から")],
         f"MakeHuman の標準の体（身長 1,666 mm）に胸の板（外向き {angle:.0f}°・上下 {pitch:+.0f}°）")
    print(f"[body_fit] 絵: {out}")


def main() -> None:
    body = load_body()
    fr = to_mesh(assembled(0, 0, 0)["back"])
    pts = back_samples(fr)
    m, gaps = place(body, pts)
    moved = trimesh.transform_points(pts, m)
    print(f"背板の上の端: 体の高さ {moved[:, 2].max():.0f} mm（胸骨の上の切り欠き {NOTCH_Z:.0f}）")
    report_gaps(moved, gaps)
    combos = ((0.0, 0.0, "a0"), (20.0, 0.0, "a20"), (0.0, min(CCFG.pitches), "pm20"), (0.0, max(CCFG.pitches), "pp10"))
    for angle, pitch, _ in combos:
        check_inside(body, m, angle, pitch)
        view_check(body, m, angle, pitch)
    for angle, pitch, tag in combos:
        draw_fit(body, m, angle, pitch, tag)


if __name__ == "__main__":
    main()
