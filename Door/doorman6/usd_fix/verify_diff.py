"""確かめ ①: 直した USD を読み戻し、元の USD との違いを全部出す（読むだけ・Mac）。

1. ファイル: base.usd・sensor.usd は元と md5 が同じ。いちばん上のファイルは名前だけ違い中身（md5）は同じ
2. physics.usd のレイヤ（Sdf の spec）どうしの差分: 変わった spec・足した spec・消えた spec とその欄を全部出す。
   違ってよいのは「足した剛体の /<ロボット>/<剛体>/collisions の instanceable と references」と「/colliders/<剛体> の下の追加」だけ
3. 合成したステージ（instance proxy まで）を prim ごとに比べる（型・API・instanceable・書かれた属性の値・関係の先）。
   違ってよいのは足した剛体の <剛体>/collisions の下だけ。剛体（質量・重心・慣性・主軸）・関節（駆動・可動域・取付・armature 等）・
   既存の当たりの形は 1 ビットも変わっていないこと
4. 足した形: CollisionAPI があり、メッシュは MeshCollisionAPI の approximation=convexHull・頂点 64 以下・閉じた凸の外向きの面・
   extent が点と合う・material:binding の先が在る

使い方: uvx --with usd-core --with numpy --with scipy --with python-fcl python verify_diff.py [--usd 直した USD]
終了コード: 合格 0・不合格 1
"""
from __future__ import annotations

import argparse
import hashlib
import os
import sys
from typing import Dict, List, Tuple

import numpy as np
from pxr import Sdf, Usd, UsdGeom, UsdPhysics, UsdShade
from scipy.spatial import ConvexHull

HERE = os.path.dirname(os.path.abspath(__file__))
ROBOT = "g1_29dof_rev_1_0"
FIXED = os.path.join(HERE, "fixed", "g1_29dof_rev_1_0_col", "g1_29dof_rev_1_0_col.usd")
ORIG = os.path.join(HERE, "orig", ROBOT, f"{ROBOT}.usd")


def md5(path: str) -> str:
    with open(path, "rb") as f:
        return hashlib.md5(f.read()).hexdigest()


def _v(x):
    return np.asarray(x).tolist() if hasattr(x, "__len__") and not isinstance(x, str) else x


def spec_fields(spec) -> Dict[str, object]:
    out = {k: str(spec.GetInfo(k)) for k in spec.ListInfoKeys()}
    if isinstance(spec, Sdf.PrimSpec):
        for a in spec.attributes:
            out[f".{a.name}"] = (str(a.typeName), str(a.variability), _v(a.default),
                                 {k: str(a.GetInfo(k)) for k in a.ListInfoKeys() if k != "default"})
        for r in spec.relationships:
            out[f"rel {r.name}"] = str(r.targetPathList)
    return out


def layer_specs(layer: Sdf.Layer) -> Dict[str, Dict[str, object]]:
    out = {}

    def walk(spec):
        out[str(spec.path)] = spec_fields(spec)
        for c in spec.nameChildren:
            walk(c)
        for v in spec.variantSets.values() if hasattr(spec, "variantSets") else []:
            for var in v.variants.values():
                walk(var.primSpec)

    head = {"subLayerPaths": list(layer.subLayerPaths),
            "subLayerOffsets": [(o.offset, o.scale) for o in layer.subLayerOffsets]}
    for k in layer.pseudoRoot.ListInfoKeys():
        if k not in ("subLayers", "subLayerOffsets"):
            head[k] = str(layer.pseudoRoot.GetInfo(k))
    out["<layer>"] = head
    for p in layer.rootPrims:
        walk(p)
    return out


def snapshot(path: str) -> Dict[str, Tuple]:
    stage = Usd.Stage.Open(path)
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    out: Dict[str, Tuple] = {}
    for p in stage.Traverse(pred):
        attrs = {a.GetName(): _v(a.Get()) for a in p.GetAuthoredAttributes()}
        rels = {r.GetName(): [str(t) for t in r.GetTargets()] for r in p.GetAuthoredRelationships()}
        out[str(p.GetPath())] = (p.GetTypeName(), tuple(p.GetAppliedSchemas()), p.IsInstance(), attrs, rels)
    return out


def check_files(orig_dir: str, fixed_dir: str, top_name: str) -> List[str]:
    errs = []
    print("[①-1] ファイル（md5）")
    for rel in ("configuration/g1_29dof_rev_1_0_base.usd", "configuration/g1_29dof_rev_1_0_sensor.usd"):
        a, b = md5(os.path.join(orig_dir, rel)), md5(os.path.join(fixed_dir, rel))
        print(f"   {rel}: {'同じ' if a == b else '違う'}（{b}）")
        if a != b:
            errs.append(f"{rel} が元と違う")
    a, b = md5(os.path.join(orig_dir, f"{ROBOT}.usd")), md5(os.path.join(fixed_dir, top_name))
    print(f"   {ROBOT}.usd → {top_name}: 中身 {'同じ' if a == b else '違う'}")
    if a != b:
        errs.append("いちばん上のファイルの中身が元と違う")
    extra = sorted(set(os.listdir(os.path.join(fixed_dir, "configuration")))
                   - set(os.listdir(os.path.join(orig_dir, "configuration"))))
    if extra:
        errs.append(f"configuration に余分なファイル {extra}")
    return errs


def check_layer(orig_phys: str, fixed_phys: str, added: List[str]) -> List[str]:
    errs = []
    A, B = layer_specs(Sdf.Layer.FindOrOpen(orig_phys)), layer_specs(Sdf.Layer.FindOrOpen(fixed_phys))
    print(f"[①-2] physics.usd の spec の差分（元 {len(A)}・直した {len(B)}）")
    ok_col = {f"/{ROBOT}/{l}/collisions" for l in added}
    for p in sorted(set(A) | set(B)):
        if p in A and p in B:
            if A[p] == B[p]:
                continue
            keys = sorted(k for k in set(A[p]) | set(B[p]) if A[p].get(k) != B[p].get(k))
            fine = p in ok_col and set(keys) <= {"instanceable", "references"}
            print(f"   {'変更' if fine else '想定外の変更'}: {p}  欄 {keys}")
            for k in keys:
                print(f"        {k}: {A[p].get(k)} → {B[p].get(k)}")
            if not fine:
                errs.append(f"想定外の変更 {p}")
        elif p in B:
            fine = any(p == f"/colliders/{l}" or p.startswith(f"/colliders/{l}/") for l in added)
            short = {k: (v if len(str(v)) < 90 else str(v)[:90] + "…") for k, v in B[p].items()}
            print(f"   {'追加' if fine else '想定外の追加'}: {p}  {short}")
            if not fine:
                errs.append(f"想定外の追加 {p}")
        else:
            print(f"   想定外の削除: {p}")
            errs.append(f"想定外の削除 {p}")
    return errs


def check_stage(orig: str, fixed: str, added: List[str]) -> List[str]:
    errs = []
    a, b = snapshot(orig), snapshot(fixed)
    only_a, only_b = sorted(set(a) - set(b)), sorted(set(b) - set(a))
    changed = sorted(p for p in set(a) & set(b) if a[p] != b[p])

    def allowed(p: str) -> bool:
        return any(p == f"/{ROBOT}/{l}/collisions" or p.startswith(f"/{ROBOT}/{l}/collisions/") for l in added)

    print(f"[①-3] 合成したステージ: prim 元 {len(a)}・直した {len(b)}。元にだけ {len(only_a)}・直した方にだけ {len(only_b)}・"
          f"中身が違う {len(changed)}")
    for p in changed:
        what = [n for n, x, y in zip(("型", "API", "instance", "属性", "関係"), a[p], b[p]) if x != y]
        print(f"   違う: {p}  {what}  instance {a[p][2]}→{b[p][2]}")
    for p in only_b:
        print(f"   足した: {p}  {b[p][0]}  API={list(b[p][1])}")
    bad = [p for p in only_a + only_b + changed if not allowed(p)] + [p for p in only_a]
    for p in sorted(set(bad)):
        print(f"   想定外: {p}")
        errs.append(f"合成の想定外の違い {p}")
    rb_a = {p: v for p, v in a.items() if "PhysicsRigidBodyAPI" in v[1]}
    rb_b = {p: v for p, v in b.items() if "PhysicsRigidBodyAPI" in v[1]}
    jn_a = {p: v for p, v in a.items() if v[0].startswith("Physics") and "Joint" in v[0]}
    jn_b = {p: v for p, v in b.items() if v[0].startswith("Physics") and "Joint" in v[0]}
    col_a = {p: v for p, v in a.items() if "PhysicsCollisionAPI" in v[1]}
    same_col = all(b.get(p) == v for p, v in col_a.items())
    print(f"   剛体 {len(rb_a)}→{len(rb_b)} 同じ={rb_a == rb_b}（質量・重心・慣性・主軸・自己衝突・反復回数を含む）")
    print(f"   関節 {len(jn_a)}→{len(jn_b)} 同じ={jn_a == jn_b}（駆動の固さ・減衰・力の上限・可動域・取付・最大速度・等価慣性を含む）")
    print(f"   既存の当たりの形 {len(col_a)} 個 同じ={same_col}")
    if not (rb_a == rb_b and jn_a == jn_b and same_col):
        errs.append("剛体・関節・既存の当たりの形のどれかが変わった")
    no_mass = [p for p, v in rb_b.items() if not {"physics:mass", "physics:diagonalInertia", "physics:centerOfMass"}
               <= set(v[3])]
    print(f"   質量・慣性・重心が書かれていない剛体: {no_mass or 'なし'}（形を足しても質量は変わらない）")
    if no_mass:
        errs.append(f"質量の書かれていない剛体 {no_mass}")
    return errs


def check_added(fixed: str, added: List[str]) -> List[str]:
    errs = []
    stage = Usd.Stage.Open(fixed)
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
    print("[①-4] 足した形の読み戻し")
    for l in added:
        root = stage.GetPrimAtPath(f"/{ROBOT}/{l}/collisions")
        cols = [p for p in Usd.PrimRange(root, pred) if p.HasAPI(UsdPhysics.CollisionAPI)]
        if not root.IsInstance() or not cols:
            errs.append(f"{l}: instance でない／当たりの形が無い")
        for p in cols:
            t = p.GetTypeName()
            xf = p.GetParent()
            purpose = UsdGeom.Imageable(xf).GetPurposeAttr().Get()
            msg = f"   {p.GetPath()}  {t}  purpose(親)={purpose}"
            if t == "Mesh":
                m = UsdGeom.Mesh(p)
                P = np.array(m.GetPointsAttr().Get(), dtype=float)
                F = np.array(m.GetFaceVertexIndicesAttr().Get()).reshape(-1, 3)
                approx = UsdPhysics.MeshCollisionAPI(p).GetApproximationAttr().Get()
                h = ConvexHull(P)
                c = P.mean(axis=0)
                n = np.cross(P[F[:, 1]] - P[F[:, 0]], P[F[:, 2]] - P[F[:, 0]])
                outward = bool((np.einsum("ij,ij->i", n, P[F[:, 0]] - c) > 0).all())
                edges: Dict[Tuple[int, int], int] = {}
                for f in F:
                    for i in range(3):
                        e = tuple(sorted((int(f[i]), int(f[(i + 1) % 3]))))
                        edges[e] = edges.get(e, 0) + 1
                closed = all(v == 2 for v in edges.values())
                ext = np.array(m.GetExtentAttr().Get())
                ext_ok = np.allclose(ext, [P.min(axis=0), P.max(axis=0)], atol=1e-6)
                mat = UsdShade.MaterialBindingAPI(p).GetDirectBindingRel().GetTargets()
                mat_ok = bool(mat) and stage.GetPrimAtPath(mat[0]).IsValid()
                ok = (approx == "convexHull" and len(P) <= 64 and len(h.vertices) == len(P) and outward and closed
                      and ext_ok and mat_ok)
                msg += (f"  頂点 {len(P)}（凸包の頂点 {len(h.vertices)}）・面 {len(F)}・approximation={approx}・外向き={outward}"
                        f"・閉じている={closed}・extent={ext_ok}・material={mat[0] if mat else None}（在る={mat_ok}）")
            elif t == "Cylinder":
                g = UsdGeom.Cylinder(p)
                ok = True
                msg += f"  r={g.GetRadiusAttr().Get()} 長さ={g.GetHeightAttr().Get()} 軸={g.GetAxisAttr().Get()}"
            else:
                ok = False
            print(msg + ("" if ok else "  ← 不合格"))
            if not ok:
                errs.append(f"足した形の不備 {p.GetPath()}")
    return errs


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--usd", default=FIXED)
    ap.add_argument("--orig", default=ORIG)
    a = ap.parse_args()
    fixed_dir, orig_dir = os.path.dirname(a.usd), os.path.dirname(a.orig)
    stage_o, stage_f = Usd.Stage.Open(a.orig), Usd.Stage.Open(a.usd)
    pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)

    def bodies_with_col(stage) -> Dict[str, int]:
        cnt = {}
        for p in stage.Traverse(pred):
            if p.HasAPI(UsdPhysics.RigidBodyAPI):
                cnt[p.GetName()] = sum(q.HasAPI(UsdPhysics.CollisionAPI) for q in Usd.PrimRange(p, pred))
        return cnt

    co, cf = bodies_with_col(stage_o), bodies_with_col(stage_f)
    added = sorted(l for l in cf if cf[l] > 0 and co.get(l, 0) == 0)
    print(f"[①] 剛体 {len(co)}。当たりの形が無い剛体: 元 {sorted(l for l, n in co.items() if n == 0)}")
    print(f"     直した方で無い剛体: {sorted(l for l, n in cf.items() if n == 0)}")
    print(f"     形を足した剛体 {len(added)}: {added}")
    print(f"     形の数が変わった剛体: {[(l, co[l], cf[l]) for l in co if co[l] != cf[l]]}")
    errs = check_files(orig_dir, fixed_dir, os.path.basename(a.usd))
    errs += check_layer(os.path.join(orig_dir, "configuration", f"{ROBOT}_physics.usd"),
                        os.path.join(fixed_dir, "configuration", f"{ROBOT}_physics.usd"), added)
    errs += check_stage(a.orig, a.usd, added)
    errs += check_added(a.usd, added)
    print("[①] 判定:", "合格" if not errs else f"不合格 {errs}")
    return 0 if not errs else 1


if __name__ == "__main__":
    sys.exit(main())
