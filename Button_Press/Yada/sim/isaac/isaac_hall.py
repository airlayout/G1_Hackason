"""Isaac Sim のエレベーター乗り場のシーン。common/scene_spec.py の HallScene を USD に組み立てる。

MuJoCo 版（sim/mujoco/mujoco_hall.py）と同じ HallScene から作るので、寸法は同じになる。
Isaac Sim のアプリの起動後に import すること（pxr と isaaclab を使う）。

構成（/World/Hall の下）:
- 壁、扉、ボタン盤: 固定の箱（衝突判定あり、剛体ではない）
- Buttons: ボタンの関節機構（articulation）。土台（base）をワールドに固定し、各ボタンを +x に沈む直動関節でつなぐ。
  ばねは関節のドライブ（目標 0、剛性 = stiffness、減衰 = damping）で作る。沈み量は関節の角度として読める。
  ボタンが盤にめり込んでも押し返されないよう、ボタンと盤の衝突は FilteredPairsAPI で外す。
- 点灯: ボタンの面の材質（UsdPreviewSurface）の色と発光を、押したら書き換える。

座標: HallScene は pelvis 座標なので、/World/Hall を pelvis のワールド位置に置き、その下は HallScene の値をそのまま使う。
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common.config import REPO_ROOT  # noqa: E402
from common.scene_spec import ButtonSpec, CallButtonState, HallScene, carpet_texture  # noqa: E402

HALL_PATH = "/World/Hall"
BUTTONS_PATH = f"{HALL_PATH}/Buttons"
LOOKS_PATH = f"{HALL_PATH}/Looks"
CARPET_PATH = f"{HALL_PATH}/carpet"
# 絨毯のテクスチャの保存先（起動のたびに設定から作り直す）
TEXTURE_DIR = REPO_ROOT / "_local" / "button_press_yada" / "textures"
# 絨毯の板の大きさ [m] と、床からの浮かせ方（床の格子の見た目と重なってちらつかないように）
CARPET_SIZE = 20.0
CARPET_LIFT = 0.0005


def button_link(name: str) -> str:
    return f"button_{name}"


def button_joint(name: str) -> str:
    return f"button_{name}_slide"


def _material(stage: Any, name: str, rgba: tuple[float, ...], emissive: tuple[float, float, float] = (0, 0, 0)) -> Any:
    """UsdPreviewSurface の材質を作る。戻り値は shader（あとで色を書き換えるため）。"""
    from pxr import Gf, Sdf, UsdShade

    mat = UsdShade.Material.Define(stage, f"{LOOKS_PATH}/{name}")
    shader = UsdShade.Shader.Define(stage, f"{LOOKS_PATH}/{name}/Shader")
    shader.CreateIdAttr("UsdPreviewSurface")
    shader.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*rgba[:3]))
    shader.CreateInput("emissiveColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(*emissive))
    shader.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(0.5)
    shader.CreateInput("opacity", Sdf.ValueTypeNames.Float).Set(float(rgba[3]))
    mat.CreateSurfaceOutput().ConnectToSource(shader.ConnectableAPI(), "surface")
    return mat, shader


def _bind(prim: Any, mat: Any) -> None:
    from pxr import UsdShade

    UsdShade.MaterialBindingAPI.Apply(prim).Bind(mat)


def _box(stage: Any, path: str, center: np.ndarray, half: np.ndarray, mat: Any, collide: bool = True) -> Any:
    from pxr import Gf, UsdGeom, UsdPhysics

    cube = UsdGeom.Cube.Define(stage, path)
    cube.CreateSizeAttr(2.0)
    xf = UsdGeom.XformCommonAPI(cube)
    xf.SetTranslate(Gf.Vec3d(*center.tolist()))
    xf.SetScale(Gf.Vec3f(*half.tolist()))
    _bind(cube.GetPrim(), mat)
    if collide:
        UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
    return cube.GetPrim()


def _symbol_mesh(stage: Any, path: str, b: ButtonSpec, mat: Any) -> None:
    """記号（三角形の板）。ボタンの面の手前（−x 側）に symbol_thickness だけ出す。衝突判定なし。"""
    from pxr import Gf, UsdGeom

    tri = b.symbol_triangle()
    t = b.symbol_thickness
    pts = [Gf.Vec3f(x, float(y), float(z)) for x in (-t, 0.0) for y, z in tri]
    # 前の面（x = −t、ロボット側から見て反時計回り）、後ろの面、側面 3 枚
    faces = [[0, 2, 1], [3, 4, 5], [0, 1, 4, 3], [1, 2, 5, 4], [2, 0, 3, 5]]
    mesh = UsdGeom.Mesh.Define(stage, path)
    mesh.CreatePointsAttr(pts)
    mesh.CreateFaceVertexCountsAttr([len(f) for f in faces])
    mesh.CreateFaceVertexIndicesAttr([i for f in faces for i in f])
    mesh.CreateDoubleSidedAttr(True)
    _bind(mesh.GetPrim(), mat)


def _carpet(stage: Any, scene: HallScene) -> None:
    """床に絨毯の板を敷く（見た目だけ。衝突判定は床の GroundPlane のまま）。

    MuJoCo 版と同じ画像（carpet_texture）を PNG にして貼る。板の中心は pelvis の真下。
    """
    from PIL import Image
    from pxr import Gf, Sdf, UsdGeom, UsdShade, Vt

    TEXTURE_DIR.mkdir(parents=True, exist_ok=True)
    png = TEXTURE_DIR / "carpet.png"
    Image.fromarray(carpet_texture(scene.floor)).save(png)

    h = CARPET_SIZE / 2.0
    z = scene.floor_z + CARPET_LIFT
    reps = CARPET_SIZE / scene.floor.tile_size
    mesh = UsdGeom.Mesh.Define(stage, CARPET_PATH)
    mesh.CreatePointsAttr([Gf.Vec3f(-h, -h, z), Gf.Vec3f(h, -h, z), Gf.Vec3f(h, h, z), Gf.Vec3f(-h, h, z)])
    mesh.CreateFaceVertexCountsAttr([4])
    mesh.CreateFaceVertexIndicesAttr([0, 1, 2, 3])
    mesh.CreateNormalsAttr([Gf.Vec3f(0, 0, 1)] * 4)
    mesh.SetNormalsInterpolation(UsdGeom.Tokens.vertex)
    st = UsdGeom.PrimvarsAPI(mesh).CreatePrimvar("st", Sdf.ValueTypeNames.TexCoord2fArray, UsdGeom.Tokens.vertex)
    st.Set(Vt.Vec2fArray([Gf.Vec2f(0, 0), Gf.Vec2f(reps, 0), Gf.Vec2f(reps, reps), Gf.Vec2f(0, reps)]))

    mat = UsdShade.Material.Define(stage, f"{LOOKS_PATH}/mat_carpet")
    surf = UsdShade.Shader.Define(stage, f"{LOOKS_PATH}/mat_carpet/Shader")
    surf.CreateIdAttr("UsdPreviewSurface")
    surf.CreateInput("roughness", Sdf.ValueTypeNames.Float).Set(1.0)
    surf.CreateInput("specularColor", Sdf.ValueTypeNames.Color3f).Set(Gf.Vec3f(0.02, 0.02, 0.02))
    reader = UsdShade.Shader.Define(stage, f"{LOOKS_PATH}/mat_carpet/st")
    reader.CreateIdAttr("UsdPrimvarReader_float2")
    reader.CreateInput("varname", Sdf.ValueTypeNames.Token).Set("st")
    tex = UsdShade.Shader.Define(stage, f"{LOOKS_PATH}/mat_carpet/texture")
    tex.CreateIdAttr("UsdUVTexture")
    tex.CreateInput("file", Sdf.ValueTypeNames.Asset).Set(str(png))
    tex.CreateInput("wrapS", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("wrapT", Sdf.ValueTypeNames.Token).Set("repeat")
    tex.CreateInput("sourceColorSpace", Sdf.ValueTypeNames.Token).Set("sRGB")
    tex.CreateInput("st", Sdf.ValueTypeNames.Float2).ConnectToSource(reader.ConnectableAPI(), "result")
    surf.CreateInput("diffuseColor", Sdf.ValueTypeNames.Color3f).ConnectToSource(tex.ConnectableAPI(), "rgb")
    mat.CreateSurfaceOutput().ConnectToSource(surf.ConnectableAPI(), "surface")
    _bind(mesh.GetPrim(), mat)


def build_hall(stage: Any, scene: HallScene, pelvis_world: np.ndarray) -> dict[str, Any]:
    """乗り場を stage に作る。ボタンの面の材質の shader を {ボタン名: shader} で返す。"""
    from pxr import Gf, Sdf, UsdGeom, UsdPhysics

    hall = UsdGeom.Xform.Define(stage, HALL_PATH)
    UsdGeom.XformCommonAPI(hall).SetTranslate(Gf.Vec3d(*np.asarray(pelvis_world, dtype=float).tolist()))

    if scene.floor is not None:
        _carpet(stage, scene)

    box_paths: dict[str, str] = {}
    for i, box in enumerate(scene.boxes):
        mat, _ = _material(stage, f"mat_{box.name}", box.rgba)
        path = f"{HALL_PATH}/{box.name}"
        _box(stage, path, box.center, box.half_size, mat)
        box_paths[box.name] = path

    # ボタンの関節機構
    root = UsdGeom.Xform.Define(stage, BUTTONS_PATH)
    UsdPhysics.ArticulationRootAPI.Apply(root.GetPrim())
    base = UsdGeom.Xform.Define(stage, f"{BUTTONS_PATH}/base")
    UsdPhysics.RigidBodyAPI.Apply(base.GetPrim())
    UsdPhysics.MassAPI.Apply(base.GetPrim()).CreateMassAttr(1.0)
    fixed = UsdPhysics.FixedJoint.Define(stage, f"{BUTTONS_PATH}/root_joint")
    fixed.CreateBody1Rel().SetTargets([base.GetPath()])

    sym_mat, _ = _material(stage, "mat_button_symbol", scene.buttons[0].symbol_rgba)
    shaders: dict[str, Any] = {}
    for b in scene.buttons:
        link_path = f"{BUTTONS_PATH}/{button_link(b.name)}"
        link = UsdGeom.Xform.Define(stage, link_path)
        UsdGeom.XformCommonAPI(link).SetTranslate(Gf.Vec3d(*b.face_center.tolist()))
        UsdPhysics.RigidBodyAPI.Apply(link.GetPrim())
        UsdPhysics.MassAPI.Apply(link.GetPrim()).CreateMassAttr(b.mass)

        cap = UsdGeom.Cylinder.Define(stage, f"{link_path}/cap")
        cap.CreateAxisAttr("X")
        cap.CreateRadiusAttr(b.radius)
        cap.CreateHeightAttr(b.protrusion)
        UsdGeom.XformCommonAPI(cap).SetTranslate(Gf.Vec3d(b.protrusion / 2.0, 0.0, 0.0))
        UsdPhysics.CollisionAPI.Apply(cap.GetPrim())
        # 押すと盤の中に沈むので、盤との衝突は外す
        UsdPhysics.FilteredPairsAPI.Apply(cap.GetPrim()).CreateFilteredPairsRel().SetTargets(
            [Sdf.Path(box_paths["hall_panel"])])
        mat, shader = _material(stage, f"mat_{button_link(b.name)}_face", b.off_rgba)
        _bind(cap.GetPrim(), mat)
        shaders[b.name] = shader
        _symbol_mesh(stage, f"{link_path}/symbol", b, sym_mat)

        joint = UsdPhysics.PrismaticJoint.Define(stage, f"{BUTTONS_PATH}/{button_joint(b.name)}")
        joint.CreateAxisAttr("X")
        joint.CreateBody0Rel().SetTargets([base.GetPath()])
        joint.CreateBody1Rel().SetTargets([link.GetPath()])
        joint.CreateLocalPos0Attr(Gf.Vec3f(*b.face_center.tolist()))
        joint.CreateLocalPos1Attr(Gf.Vec3f(0.0, 0.0, 0.0))
        joint.CreateLowerLimitAttr(0.0)
        joint.CreateUpperLimitAttr(b.travel)
        drive = UsdPhysics.DriveAPI.Apply(joint.GetPrim(), "linear")
        drive.CreateTypeAttr("force")
        drive.CreateTargetPositionAttr(0.0)
        drive.CreateStiffnessAttr(b.stiffness)
        drive.CreateDampingAttr(b.damping)
    return shaders


def hall_articulation_cfg(scene: HallScene) -> Any:
    """ボタンの関節機構を IsaacLab の Articulation として扱う設定（prim は build_hall で作り済み）。

    IsaacLab はドライブの剛性・減衰をアクチュエータの設定で上書きするので、ここにも同じ値を書く。
    ボタンごとに値が違ってもよいように、関節名ごとの辞書にする。
    """
    from isaaclab.actuators import ImplicitActuatorCfg
    from isaaclab.assets import ArticulationCfg

    return ArticulationCfg(
        prim_path=BUTTONS_PATH,
        spawn=None,
        init_state=ArticulationCfg.InitialStateCfg(joint_pos={button_joint(b.name): 0.0 for b in scene.buttons}),
        actuators={
            "springs": ImplicitActuatorCfg(
                joint_names_expr=[button_joint(b.name) for b in scene.buttons],
                stiffness={button_joint(b.name): b.stiffness for b in scene.buttons},
                damping={button_joint(b.name): b.damping for b in scene.buttons},
            )
        },
    )


class HallIsaac:
    """ボタンの沈み量を読み、点灯の状態と色を更新する。sim.step と articulation.update のあとに update() を呼ぶ。"""

    def __init__(self, articulation: Any, scene: HallScene, shaders: dict[str, Any]):
        self.art, self.scene, self.shaders = articulation, scene, shaders
        self.states = {b.name: CallButtonState(b.press_depth) for b in scene.buttons}
        self._joint_ids: dict[str, int] = {}
        self._shown: dict[str, bool | None] = {b.name: None for b in scene.buttons}

    def _ids(self) -> dict[str, int]:
        # 関節の番号は sim.reset() のあとでないと分からない
        if not self._joint_ids:
            for b in self.scene.buttons:
                ids, _ = self.art.find_joints(button_joint(b.name))
                self._joint_ids[b.name] = int(ids[0])
        return self._joint_ids

    def depths(self) -> dict[str, float]:
        import warp as wp

        q = wp.to_torch(self.art.data.joint_pos)[0]
        return {n: float(q[i]) for n, i in self._ids().items()}

    def update(self) -> list[str]:
        """点灯の状態を更新する。このステップで押されたボタンの名前を返す。"""
        pressed = [n for n, d in self.depths().items() if self.states[n].update(d)]
        self._apply_colors()
        return pressed

    def lit(self, name: str) -> bool:
        return self.states[name].lit

    def reset_lights(self) -> None:
        for s in self.states.values():
            s.reset()
        self._apply_colors()

    def set_targets(self, depths: dict[str, float]) -> None:
        """ばねの目標の位置（沈み量）を変える。0 が普段の状態。外から押す試験に使う。"""
        import torch

        ids = self._ids()
        names = list(depths)
        target = torch.tensor([[depths[n] for n in names]], dtype=torch.float32, device=self.art.device)
        self.art.set_joint_position_target_index(target=target, joint_ids=[ids[n] for n in names])
        self.art.write_data_to_sim()

    def _apply_colors(self) -> None:
        from pxr import Gf

        for b in self.scene.buttons:
            lit = self.states[b.name].lit
            if self._shown[b.name] == lit:
                continue
            sh = self.shaders[b.name]
            rgb = b.lit_rgba[:3] if lit else b.off_rgba[:3]
            sh.GetInput("diffuseColor").Set(Gf.Vec3f(*rgb))
            sh.GetInput("emissiveColor").Set(Gf.Vec3f(*(np.asarray(b.lit_rgba[:3]) * 0.6 if lit else (0, 0, 0))))
            self._shown[b.name] = lit
