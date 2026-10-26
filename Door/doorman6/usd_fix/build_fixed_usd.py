"""G1 29 関節（rev_1_0）の USD に、当たりの形が抜けている剛体の当たりの形を足す（直し A）。

元の USD は書き換えない。orig/g1_29dof_rev_1_0/ を fixed/<名前>/ へ写し（いちばん上のファイルは <名前>.usd に改名、
中身は同じ）、configuration/ の physics.usd だけを書き換える。base.usd・sensor.usd は 1 バイトも変えない。

- 形は公式の unitree_ros の g1_29dof_rev_1_0.urdf の <collision> どおり。円柱は UsdGeom.Cylinder、
  メッシュは STL の凸包を頂点 64 以下に間引いた UsdGeom.Mesh＋MeshCollisionAPI の approximation=convexHull
  （頂点 64 以下なので PhysX は凸包を作り直さない＝ここで作った形がそのまま使われる）
- URDF に <collision> が無い剛体（waist_yaw・waist_roll）は <visual> のメッシュの凸包（--extra で選ぶ。既定は waist_roll だけ。
  waist_yaw の <visual> の凸包は既定の姿勢で torso_link に約 19 mm 食い込む＝起動の瞬間に弾けるので既定では足さない）
- logo は剛体でなく torso_link に固定された部品（--extra logo_link のときだけ torso_link の当たりの形に足す。
  胴の凸包から出るのは 2.2 mm 以下なので既定では足さない＝torso_link の当たりの形は元のまま）
- 作りは既存の当たりの形に合わせる: /<ロボット>/<剛体>/collisions を instanceable にして </colliders/<剛体>> を参照、
  /colliders/<剛体>/<子>（Xform・purpose=guide・translate/orient/scale）の下に形、CollisionAPI
  （メッシュは MeshCollisionAPI と MaterialBindingAPI＋material:binding→Looks/DefaultMaterial も。既存の /meshes と同じ）

使い方（Mac）:
  uvx --with usd-core --with numpy --with scipy --with python-fcl python build_fixed_usd.py --force
"""
from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
import shutil
import xml.etree.ElementTree as ET
from typing import Dict, List, Sequence

import numpy as np
from pxr import Gf, Sdf, Vt
from scipy.spatial import ConvexHull

import g1_geom as gg

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT = "g1_29dof_rev_1_0"
OUT_NAME = "g1_29dof_rev_1_0_col"
FROM_COLLISION = (  # 公式 URDF に <collision> がある 10 個
    "left_hip_roll_link", "right_hip_roll_link", "left_ankle_pitch_link", "right_ankle_pitch_link",
    "left_shoulder_roll_link", "right_shoulder_roll_link", "left_elbow_link", "right_elbow_link",
    "left_wrist_pitch_link", "right_wrist_pitch_link",
)
WAIST = ("waist_yaw_link", "waist_roll_link")  # 公式 URDF に <collision> が無い（<visual> だけ）
URDF_COMMIT = "7c40519e02d7dd16c06b25fe3fa3b67fdeb7cd74"  # unitreerobotics/unitree_ros（urdf_src/ の出どころ）
DEFAULT_MATERIAL = f"/{ROBOT}/Looks/DefaultMaterial"
LOGO_PARENT = "torso_link"  # logo_link は torso_link に固定（URDF の logo_joint）


def md5(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def rpy_to_quat_wxyz(r: float, p: float, y: float) -> List[float]:
    """URDF の rpy（固定軸 X→Y→Z、R = Rz·Ry·Rx）を四元数 (w, x, y, z) へ"""
    cr, sr = math.cos(r / 2), math.sin(r / 2)
    cp, sp = math.cos(p / 2), math.sin(p / 2)
    cy, sy = math.cos(y / 2), math.sin(y / 2)
    return [cr * cp * cy + sr * sp * sy, sr * cp * cy - cr * sp * sy,
            cr * sp * cy + sr * cp * sy, cr * cp * sy - sr * sp * cy]


def _origin(e: ET.Element) -> tuple:
    o = e.find("origin")
    xyz = [float(v) for v in (o.get("xyz", "0 0 0") if o is not None else "0 0 0").split()]
    rpy = [float(v) for v in (o.get("rpy", "0 0 0") if o is not None else "0 0 0").split()]
    return xyz, rpy


def urdf_shapes(urdf: str, links: Sequence[str]) -> Dict[str, List[dict]]:
    """剛体ごとの当たりの形の定義。<collision> が無ければ <visual> のメッシュを使い source にそう書く"""
    root = ET.parse(urdf).getroot()
    by_name = {l.get("name"): l for l in root.findall("link")}
    joints = {j.find("child").get("link"): j for j in root.findall("joint")}
    out: Dict[str, List[dict]] = {}
    for name in links:
        link = by_name.get(name)
        if link is None:
            raise RuntimeError(f"URDF に剛体が無い: {name}")
        elems, source = link.findall("collision"), "URDF の <collision>"
        if not elems:
            elems = [v for v in link.findall("visual") if v.find("geometry")[0].tag == "mesh"]
            source = "URDF に <collision> が無いので <visual> のメッシュの凸包"
        items = []
        for e in elems:
            xyz, rpy = _origin(e)
            g = e.find("geometry")[0]
            items.append({"tag": g.tag, "attrib": dict(g.attrib), "xyz": xyz, "rpy": rpy, "source": source})
        out[name] = items
    if "logo_link" in links:  # logo は剛体ではないので torso の座標へ移す（URDF の logo_joint の origin）
        j = joints["logo_link"]
        if j.get("type") != "fixed" or j.find("parent").get("link") != LOGO_PARENT:
            raise RuntimeError("logo_link が torso_link に固定されていない")
        jxyz, jrpy = _origin(j)
        if any(abs(v) > 0 for v in jrpy):
            raise RuntimeError("logo_joint に回転がある（未対応）")
        for d in out["logo_link"]:
            if any(abs(v) > 0 for v in d["rpy"]):
                raise RuntimeError("logo の <collision> に回転がある（未対応）")
            d["xyz"] = [a + b for a, b in zip(jxyz, d["xyz"])]
            d["source"] += f"（logo_joint の origin {jxyz} で torso_link の座標へ）"
    return out


def add_xform(parent: Sdf.PrimSpec, name: str, xyz, quat_wxyz, doc: str) -> Sdf.PrimSpec:
    """既存の当たりの形と同じ Xform（translate・orient・scale・purpose=guide）"""
    x = Sdf.PrimSpec(parent, name, Sdf.SpecifierDef, "Xform")
    x.documentation = doc
    Sdf.AttributeSpec(x, "xformOpOrder", Sdf.ValueTypeNames.TokenArray, variability=Sdf.VariabilityUniform).default = \
        Vt.TokenArray(["xformOp:translate", "xformOp:orient", "xformOp:scale"])
    Sdf.AttributeSpec(x, "xformOp:translate", Sdf.ValueTypeNames.Double3).default = Gf.Vec3d(*xyz)
    Sdf.AttributeSpec(x, "xformOp:orient", Sdf.ValueTypeNames.Quatd).default = Gf.Quatd(quat_wxyz[0], *quat_wxyz[1:])
    Sdf.AttributeSpec(x, "xformOp:scale", Sdf.ValueTypeNames.Double3).default = Gf.Vec3d(1, 1, 1)
    Sdf.AttributeSpec(x, "purpose", Sdf.ValueTypeNames.Token, variability=Sdf.VariabilityUniform).default = "guide"
    return x


def add_cylinder(parent: Sdf.PrimSpec, radius: float, length: float) -> None:
    """URDF の円柱（軸は Z）。既存の肩 pitch の円柱と同じ作り"""
    c = Sdf.PrimSpec(parent, "cylinder", Sdf.SpecifierDef, "Cylinder")
    c.SetInfo("apiSchemas", Sdf.TokenListOp.Create(prependedItems=["PhysicsCollisionAPI"]))
    Sdf.AttributeSpec(c, "axis", Sdf.ValueTypeNames.Token, variability=Sdf.VariabilityUniform).default = "Z"
    Sdf.AttributeSpec(c, "height", Sdf.ValueTypeNames.Double).default = length
    Sdf.AttributeSpec(c, "radius", Sdf.ValueTypeNames.Double).default = radius
    Sdf.AttributeSpec(c, "extent", Sdf.ValueTypeNames.Float3Array).default = Vt.Vec3fArray(
        [Gf.Vec3f(-radius, -radius, -length / 2), Gf.Vec3f(radius, radius, length / 2)])


def add_hull_mesh(parent: Sdf.PrimSpec, V: np.ndarray) -> None:
    """凸包の頂点と三角形だけのメッシュ（MeshCollisionAPI の approximation=convexHull）"""
    faces = gg.hull_faces(V)
    m = Sdf.PrimSpec(parent, "mesh", Sdf.SpecifierDef, "Mesh")
    m.SetInfo("apiSchemas", Sdf.TokenListOp.Create(
        prependedItems=["PhysicsCollisionAPI", "PhysicsMeshCollisionAPI", "MaterialBindingAPI"]))
    Sdf.AttributeSpec(m, "points", Sdf.ValueTypeNames.Point3fArray).default = Vt.Vec3fArray([Gf.Vec3f(*p) for p in V])
    Sdf.AttributeSpec(m, "faceVertexCounts", Sdf.ValueTypeNames.IntArray).default = Vt.IntArray([3] * len(faces))
    Sdf.AttributeSpec(m, "faceVertexIndices", Sdf.ValueTypeNames.IntArray).default = \
        Vt.IntArray(faces.reshape(-1).tolist())
    lo, hi = V.min(axis=0), V.max(axis=0)
    Sdf.AttributeSpec(m, "extent", Sdf.ValueTypeNames.Float3Array).default = Vt.Vec3fArray([Gf.Vec3f(*lo), Gf.Vec3f(*hi)])
    Sdf.AttributeSpec(m, "subdivisionScheme", Sdf.ValueTypeNames.Token, variability=Sdf.VariabilityUniform).default = "none"
    Sdf.AttributeSpec(m, "physics:approximation", Sdf.ValueTypeNames.Token,
                      variability=Sdf.VariabilityUniform).default = "convexHull"
    rel = Sdf.RelationshipSpec(m, "material:binding")
    rel.targetPathList.explicitItems = [Sdf.Path(DEFAULT_MATERIAL)]


def usd_mesh_points(base_layer: Sdf.Layer, mesh_name: str) -> np.ndarray:
    spec = base_layer.GetPrimAtPath(f"/meshes/{mesh_name}/mesh")
    return np.zeros((0, 3)) if spec is None else np.array(spec.attributes["points"].default, dtype=float)


def mesh_points(mesh_dir: str, base: Sdf.Layer, d: dict) -> tuple:
    """STL（urdf_src/meshes）があればそれを、無ければ USD の /meshes/<名前>/mesh の点を使う"""
    fname = os.path.basename(d["attrib"]["filename"])
    stem = os.path.splitext(fname)[0]
    scale = [float(s) for s in d["attrib"].get("scale", "1 1 1").split()]
    U = usd_mesh_points(base, stem)
    stl = os.path.join(mesh_dir, fname)
    if os.path.exists(stl):
        P = gg.read_stl(stl) * scale
        gap = None if len(U) == 0 else round(1000 * max(gg.hull_gap(P, U), gg.hull_gap(U, P)), 3)
        return stem, P, f"STL {fname}", gap
    if len(U) == 0:
        raise FileNotFoundError(f"{fname} が urdf_src/meshes にも USD の /meshes にも無い")
    return stem, U * scale, f"USD の /meshes/{stem}/mesh（STL が手元に無い）", None


def build(orig_dir: str, out_dir: str, urdf: str, mesh_dir: str, links: Sequence[str], force: bool) -> List[dict]:
    name = os.path.basename(os.path.normpath(out_dir))
    if os.path.exists(out_dir):
        if not force:
            raise FileExistsError(f"{out_dir} が既にある（--force で作り直す）")
        shutil.rmtree(out_dir)
    shutil.copytree(orig_dir, out_dir)
    os.replace(os.path.join(out_dir, f"{ROBOT}.usd"), os.path.join(out_dir, f"{name}.usd"))
    phys_path = os.path.join(out_dir, "configuration", f"{ROBOT}_physics.usd")
    base = Sdf.Layer.FindOrOpen(os.path.join(orig_dir, "configuration", f"{ROBOT}_base.usd"))
    layer = Sdf.Layer.FindOrOpen(phys_path)
    colliders = layer.GetPrimAtPath("/colliders")
    defs = urdf_shapes(urdf, links)
    summary: List[dict] = []
    for link in links:
        body = LOGO_PARENT if link == "logo_link" else link
        col = layer.GetPrimAtPath(f"/{ROBOT}/{body}/collisions")
        holder = layer.GetPrimAtPath(f"/colliders/{body}") or Sdf.PrimSpec(colliders, body, Sdf.SpecifierOver)
        if link == "logo_link":
            if col is None or not col.instanceable or "logo_link" in holder.nameChildren \
                    or "logo_link" in base.GetPrimAtPath(f"/colliders/{body}").nameChildren:
                raise RuntimeError("torso_link/collisions が想定と違う（instanceable の参照で logo は無いはず）")
        else:
            if col is None or col.hasReferences or col.nameChildren or col.instanceable:
                raise RuntimeError(f"{link}/collisions が想定と違う（空の Xform のはず）")
            if base.GetPrimAtPath(f"/colliders/{link}").nameChildren:
                raise RuntimeError(f"base の /colliders/{link} に既に形がある")
            col.SetInfo("instanceable", True)
            col.referenceList.prependedItems = [Sdf.Reference("", Sdf.Path(f"/colliders/{link}"))]
        for i, d in enumerate(defs[link]):
            quat = rpy_to_quat_wxyz(*d["rpy"])
            row = {"link": link, "body": body, "source": d["source"], "origin_xyz": d["xyz"], "origin_rpy": d["rpy"]}
            doc = f"10-26 直し A で追加。出典: unitree_ros {URDF_COMMIT[:7]} の g1_29dof_rev_1_0.urdf の {link}: {d['source']}"
            if d["tag"] == "cylinder":
                r, length = float(d["attrib"]["radius"]), float(d["attrib"]["length"])
                add_cylinder(add_xform(holder, f"mesh_{i}", d["xyz"], quat, doc), r, length)
                row.update(kind="Cylinder", prim=f"/colliders/{body}/mesh_{i}/cylinder", radius=r, length=length)
            elif d["tag"] == "mesh":
                stem, P, src_pts, gap = mesh_points(mesh_dir, base, d)
                V, dev = gg.reduce_hull(P)
                add_hull_mesh(add_xform(holder, stem, d["xyz"], quat, doc + f"。凸包 {len(V)} 頂点（{src_pts}）"), V)
                row.update(kind="Mesh(convexHull)", prim=f"/colliders/{body}/{stem}/mesh", points_from=src_pts,
                           n_src_hull_vertices=int(len(ConvexHull(P).vertices)), n_hull_vertices=int(len(V)),
                           reduce_max_dev_mm=round(1000 * dev, 2),
                           aabb_min=V.min(axis=0).round(4).tolist(), aabb_max=V.max(axis=0).round(4).tolist(),
                           stl_vs_usd_visual_mesh_hull_gap_mm=gap)
            else:
                raise ValueError(f"{link}: 知らない形 {d['tag']}")
            summary.append(row)
    layer.Save()
    return summary


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--orig", default=os.path.join(HERE, "orig", ROBOT))
    ap.add_argument("--out", default=os.path.join(HERE, "fixed", OUT_NAME))
    ap.add_argument("--urdf", default=os.path.join(HERE, "urdf_src", f"{ROBOT}.urdf"))
    ap.add_argument("--meshes", default=os.path.join(HERE, "urdf_src", "meshes"))
    ap.add_argument("--extra", default="waist_roll_link",
                    help="URDF の <collision> 以外に足すもの（カンマ区切り、waist_roll_link・waist_yaw_link・logo_link。空で無し）")
    ap.add_argument("--manifest", default=None, help="形の一覧の JSON（既定は <out>/../<名前>_shapes.json）")
    ap.add_argument("--force", action="store_true")
    a = ap.parse_args()
    extra = [x for x in a.extra.split(",") if x]
    bad = set(extra) - set(WAIST) - {"logo_link"}
    if bad:
        raise SystemExit(f"--extra に知らない名前: {sorted(bad)}")
    links = list(FROM_COLLISION) + [x for x in (*WAIST, "logo_link") if x in extra]
    summary = build(a.orig, a.out, a.urdf, a.meshes, links, a.force)
    name = os.path.basename(os.path.normpath(a.out))
    manifest = a.manifest or os.path.join(os.path.dirname(os.path.normpath(a.out)), f"{name}_shapes.json")
    files = {}
    for root, _, fs in os.walk(a.out):
        for f in sorted(fs):
            p = os.path.join(root, f)
            files[os.path.relpath(p, a.out)] = {"bytes": os.path.getsize(p), "md5": md5(p)}
    with open(manifest, "w", encoding="utf-8") as f:
        json.dump({"urdf_commit": URDF_COMMIT, "robot": ROBOT, "out_name": name, "links": links,
                   "orig_md5": {os.path.relpath(os.path.join(r, x), a.orig): md5(os.path.join(r, x))
                                for r, _, fs in os.walk(a.orig) for x in sorted(fs)},
                   "files": files, "shapes": summary}, f, ensure_ascii=False, indent=1)
    for r in summary:
        print("[build]", json.dumps({k: r[k] for k in r if k not in ("aabb_min", "aabb_max", "origin_rpy")},
                                    ensure_ascii=False))
    print(f"[build] 書いた: {a.out}（{name}.usd・physics.usd だけ中身を変更）・形の一覧: {manifest}")


if __name__ == "__main__":
    main()
