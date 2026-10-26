"""G1 の USD の当たりの形の作り（レイヤ・instance・prototype・属性・材質）を書き出す（読むだけ）。

使い方: uvx --with usd-core --with numpy --with scipy python inspect_usd.py <g1_29dof_rev_1_0.usd> [剛体名 ...]
"""
import sys

from pxr import Sdf, Usd, UsdGeom, UsdPhysics

path = sys.argv[1]
want = sys.argv[2:] or ["left_shoulder_pitch_link", "left_shoulder_yaw_link", "left_shoulder_roll_link", "torso_link"]
stage = Usd.Stage.Open(path)
print("layer stack:", [l.identifier.split("/")[-1] for l in stage.GetLayerStack()])
root = stage.GetDefaultPrim()
print("defaultPrim:", root.GetPath(), root.GetTypeName())


def show_layer(lp):
    lay = Sdf.Layer.FindOrOpen(lp)
    print(f"\n### layer {lp.split('/')[-1]}: subLayers={list(lay.subLayerPaths)} defaultPrim={lay.defaultPrim}")
    n = [0]

    def walk(spec, depth):
        if n[0] > 80 or depth > 4:
            return
        n[0] += 1
        refs = spec.referenceList.GetAddedOrExplicitItems() if spec.hasReferences else []
        pay = spec.payloadList.GetAddedOrExplicitItems() if spec.hasPayloads else []
        print("  " * depth + f"{spec.specifier} {spec.typeName} {spec.path} inst={spec.instanceable} "
              f"api={list(spec.GetInfo('apiSchemas').GetAddedOrExplicitItems()) if spec.HasInfo('apiSchemas') else []} refs={[str(r) for r in refs]} payload={[str(p) for p in pay]}")
        for c in spec.nameChildren:
            walk(c, depth + 1)

    for p in lay.rootPrims:
        walk(p, 0)


for lp in [l.realPath for l in stage.GetLayerStack()][1:]:
    show_layer(lp)
for sub in ["configuration/g1_29dof_rev_1_0_base.usd", "configuration/g1_29dof_rev_1_0_physics.usd", "configuration/g1_29dof_rev_1_0_sensor.usd"]:
    import os
    show_layer(os.path.join(os.path.dirname(path), sub))

print("\n### composed prims for", want)
pred = Usd.TraverseInstanceProxies(Usd.PrimAllPrimsPredicate)
for name in want:
    link = stage.GetPrimAtPath(root.GetPath().AppendChild(name))
    print(f"\n-- {link.GetPath()} type={link.GetTypeName()} apis={link.GetAppliedSchemas()}")
    for p in Usd.PrimRange(link, pred):
        if p == link:
            continue
        stack = [f"{s.layer.identifier.split('/')[-1]}:{s.path}" for s in p.GetPrimStack()][:3]
        print(f"   {p.GetPath()} type={p.GetTypeName()} instance={p.IsInstance()} proxy={p.IsInstanceProxy()} "
              f"apis={p.GetAppliedSchemas()} stack={stack}")
        if p.IsInstance():
            print("      prototype:", p.GetPrototype().GetPath())
        for a in p.GetAuthoredAttributes():
            n = a.GetName()
            if n.startswith(("physics", "physx", "xformOp", "purpose", "visibility")) or n in ("radius", "height", "axis", "size"):
                v = a.Get()
                print(f"      {n} = {v if not hasattr(v, '__len__') or len(str(v)) < 120 else str(v)[:120] + '...'}")
        for r in p.GetRelationships():
            if r.GetTargets():
                print(f"      rel {r.GetName()} -> {[str(t) for t in r.GetTargets()]}")
