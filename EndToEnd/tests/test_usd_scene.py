"""USD実体・instance proxyの形状が、同じワールド座標でraycastへ渡ることを検証する。"""

import numpy as np
import pytest

pytest.importorskip("pxr", reason="USDの照合にはIsaac SimのUSD Python bundleが必要です")
from pxr import Gf, Usd, UsdGeom  # noqa: E402

from sim.sensors.scene import static_mesh  # noqa: E402


def triangle(stage, path):
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr([(0, 0, 0), (1, 0, 0), (0, 1, 0)])
    mesh.CreateFaceVertexCountsAttr([3])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2])
    return mesh


def test_instanced_and_hidden_meshes_are_exported_with_correct_world_units():
    stage = Usd.Stage.CreateInMemory()
    UsdGeom.Xform.Define(stage, "/Template")
    triangle(stage, "/Template/Triangle")
    parent = UsdGeom.Xform.Define(stage, "/World/Office")
    parent.AddTranslateOp().Set(Gf.Vec3d(10, 20, 0))
    instance = UsdGeom.Xform.Define(stage, "/World/Office/Chair")
    instance.GetPrim().GetReferences().AddInternalReference("/Template")
    instance.GetPrim().SetInstanceable(True)
    instance.AddScaleOp().Set(Gf.Vec3f(2, 2, 2))
    hidden = triangle(stage, "/World/Office/Invisible")
    hidden.CreateVisibilityAttr("invisible")
    points, faces, paths = static_mesh(stage, ["/World/Office"])
    assert paths == ["/World/Office/Chair/Triangle"]
    np.testing.assert_allclose(points, [[10, 20, 0], [12, 20, 0], [10, 22, 0]])
    np.testing.assert_array_equal(faces, [[0, 1, 2]])


def test_missing_static_root_is_rejected_instead_of_silently_losing_office_obstacles():
    with pytest.raises(ValueError, match="静的シーン"):
        static_mesh(Usd.Stage.CreateInMemory(), ["/World/Missing"])
