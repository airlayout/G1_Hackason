"""MuJoCo のエレベーター乗り場のシーン。common/scene_spec.py の HallScene を、G1 のモデル（MjSpec）に足す。

- 壁、扉、ボタン盤は固定の箱（"hall" という body の中）。
- ボタンは "hall" の子の body で、+x（壁に向かう向き）に沈むスライド関節とばねを持つ。
  ボタンと盤の衝突は exclude で外す（親子の body どうしの自動の除外は、親がワールドに固定された body だと効かない。
  外さないと、ボタンが盤に当たって沈まない）。
- 公式モデルのハンドには衝突判定が無いので、中指の先に小さな球（group 3 = カメラの画像には映らない）を足す。

    from mujoco_hall import build_hall_model, HallMujoco
    model = build_hall_model(scene, robot_cfg, fingertip_cfg)
    hall = HallMujoco(model, data, scene)
    ... mujoco.mj_step(model, data); hall.update()
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

import numpy as np

# Button_Press/Yada/ を import できるようにする（common/）
sys.path.insert(0, str(Path(__file__).resolve().parents[2]))

from common.j1gen_bridge import j1gen  # noqa: E402
from common.scene_spec import ButtonSpec, CallButtonState, HallScene, carpet_texture  # noqa: E402

HALL_BODY = "hall"
CARPET = "hall_carpet"
# 公式 XML の床の geom の名前
FLOOR_GEOM = "floor"
OVERVIEW_CAMERA = "hall_overview"
# ボタンと指先の接触の solref（時定数 [s], 減衰比）。MuJoCo の接触の硬さは「時定数」と「接触する物の重さ」で決まり、
# 硬さ ≈ ボタンの重さ ÷ 時定数²。既定（0.02 s）と 20 g のボタンでは数十 N/m と、ばね（1000 N/m）より柔らかく、
# 指がボタンを押さずにめり込んだ（実測: 指が 4 mm めり込んでも 0.6 mm しか沈まなかった）。
# 時定数は物理の刻みの 2 倍（0.004 s）が下限なので、ボタンの重さを MuJoCo 版だけ BUTTON_MASS_MJ に上げて硬くする
# （0.2 kg / 0.004² ≈ 12,500 N/m）。止まっているときの押す力はばねで決まるので変わらない
CONTACT_SOLREF = [0.004, 1.0]
BUTTON_MASS_MJ = 0.2
# 画像に映さない geom の group（MuJoCo の描画の既定は group 0〜2 だけを映す）
HIDDEN_GROUP = 3


def button_body(name: str) -> str:
    return f"button_{name}"


def button_joint(name: str) -> str:
    return f"button_{name}_slide"


def button_cap(name: str) -> str:
    return f"button_{name}_cap"


def _quat_x_to_z() -> list[float]:
    """円柱の軸（MuJoCo では local z）を x に向ける四元数 (w, x, y, z)。y 軸まわりに +90°。"""
    s = np.sqrt(0.5)
    return [s, 0.0, s, 0.0]


def _symbol_mesh(spec: Any, b: ButtonSpec) -> str:
    """記号（三角形）の板のメッシュを作る。ボタンの面の手前（−x 側）に symbol_thickness だけ出す。"""
    tri = b.symbol_triangle()
    t = b.symbol_thickness
    verts = []
    for x in (-t, 0.0):
        for y, z in tri:
            verts += [x, float(y), float(z)]
    name = f"{button_body(b.name)}_symbol"
    mesh = spec.add_mesh(name=name)
    mesh.uservert = verts
    return name


def _set_scalar_or_poly(obj: Any, attr: str, value: float) -> None:
    """関節の stiffness / damping を設定する。MuJoCo 3.11 以降は 3 要素（1 次・2 次・3 次の係数）、それより前は 1 つの値。"""
    if np.ndim(getattr(obj, attr)) == 0:
        setattr(obj, attr, value)
    else:
        setattr(obj, attr, [value, 0.0, 0.0])


def _look_at_xyaxes(eye: np.ndarray, target: np.ndarray) -> list[float]:
    """MuJoCo のカメラ（−z を見る、+x が右、+y が上）を eye から target に向ける xyaxes。"""
    fwd = target - eye
    fwd /= np.linalg.norm(fwd)
    right = np.cross(fwd, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, fwd)
    return [*right.tolist(), *up.tolist()]


def _set_carpet(spec: Any, scene: HallScene) -> None:
    """床（公式 XML の floor）の材質を絨毯のテクスチャにする。見た目だけで、衝突判定は変えない。"""
    import mujoco

    img = carpet_texture(scene.floor)
    tex = spec.add_texture(name=CARPET, type=mujoco.mjtTexture.mjTEXTURE_2D,
                           width=img.shape[1], height=img.shape[0], nchannel=3)
    tex.data = img.tobytes()
    mat = spec.add_material(name=CARPET)
    textures = list(mat.textures)
    textures[mujoco.mjtTextureRole.mjTEXROLE_RGB] = CARPET
    mat.textures = textures
    # texuniform: texrepeat を「1 m あたりの枚数」として扱う
    mat.texuniform = True
    mat.texrepeat = [1.0 / scene.floor.tile_size] * 2
    mat.reflectance = 0.0
    mat.specular = 0.05
    mat.shininess = 0.0
    spec.geom(FLOOR_GEOM).material = CARPET


def add_hall(spec: Any, scene: HallScene, robot_cfg: dict[str, Any] | None = None,
             fingertip_cfg: dict[str, Any] | None = None) -> None:
    """壁、扉、ボタン盤、ボタン（と指先の衝突判定）を MjSpec に足す。"""
    import mujoco

    if scene.floor is not None:
        _set_carpet(spec, scene)
    pelvis = np.asarray(spec.body("pelvis").pos, dtype=float)
    hall = spec.worldbody.add_body(name=HALL_BODY, pos=pelvis.tolist())

    for box in scene.boxes:
        g = hall.add_geom(name=box.name, type=mujoco.mjtGeom.mjGEOM_BOX,
                          pos=box.center.tolist(), size=box.half_size.tolist())
        g.rgba = list(box.rgba)

    for b in scene.buttons:
        body = hall.add_body(name=button_body(b.name), pos=b.face_center.tolist())
        spec.add_exclude(bodyname1=HALL_BODY, bodyname2=button_body(b.name))
        j = body.add_joint(name=button_joint(b.name), type=mujoco.mjtJoint.mjJNT_SLIDE, axis=[1.0, 0.0, 0.0])
        j.range = [0.0, b.travel]
        j.limited = mujoco.mjtLimited.mjLIMITED_TRUE
        _set_scalar_or_poly(j, "stiffness", b.stiffness)
        _set_scalar_or_poly(j, "damping", b.damping)
        j.springref = 0.0
        cap = body.add_geom(name=button_cap(b.name), type=mujoco.mjtGeom.mjGEOM_CYLINDER,
                            size=[b.radius, b.protrusion / 2.0, 0.0], pos=[b.protrusion / 2.0, 0.0, 0.0],
                            quat=_quat_x_to_z())
        cap.rgba = list(b.off_rgba)
        cap.mass = max(b.mass, BUTTON_MASS_MJ)
        cap.solref = CONTACT_SOLREF
        sym = body.add_geom(name=f"{button_body(b.name)}_symbol", type=mujoco.mjtGeom.mjGEOM_MESH,
                            meshname=_symbol_mesh(spec, b))
        sym.rgba = list(b.symbol_rgba)
        sym.contype = 0
        sym.conaffinity = 0
        sym.density = 0.0

    if fingertip_cfg and fingertip_cfg.get("enabled") and robot_cfg is not None:
        for side, ee in robot_cfg["end_effector"].items():
            g = spec.body(ee["link"]).add_geom(name=f"{side}_fingertip_collision", type=mujoco.mjtGeom.mjGEOM_SPHERE,
                                               size=[float(fingertip_cfg["radius"]), 0.0, 0.0],
                                               pos=list(ee["offset"]))
            g.rgba = [0.1, 0.9, 0.3, 0.5]
            g.group = HIDDEN_GROUP
            g.solref = CONTACT_SOLREF
            g.density = 0.0

    # 全体を見るカメラ: ロボットの右後ろの上から、ボタン盤のあたりを見る
    target = pelvis + np.array([scene.boxes[0].center[0], scene.button(scene.buttons[0].name).face_center[1], 0.1])
    eye = pelvis + np.array([-1.3, -1.1, 0.9])
    cam = spec.worldbody.add_camera(name=OVERVIEW_CAMERA, pos=eye.tolist())
    cam.alt.type = mujoco.mjtOrientation.mjORIENTATION_XYAXES
    cam.alt.xyaxes = _look_at_xyaxes(eye, target)
    cam.fovy = 55.0
    # 画像を作るときの最大の大きさ（既定は 640x480 で、全体のカメラの画像が入らない）
    spec.visual.global_.offwidth = max(spec.visual.global_.offwidth, 1280)
    spec.visual.global_.offheight = max(spec.visual.global_.offheight, 960)


def build_hall_model(scene: HallScene, robot_cfg: dict[str, Any], fingertip_cfg: dict[str, Any] | None = None) -> Any:
    """G1（胴体固定、頭カメラ付き）+ 乗り場の MjModel。"""
    spec = j1gen("robot_model").build_spec(robot_cfg, fixed_base=True)
    add_hall(spec, scene, robot_cfg, fingertip_cfg)
    return spec.compile()


class HallMujoco:
    """ボタンの沈み量を読み、点灯の状態と色を更新する。mj_step のあとに update() を呼ぶ。"""

    def __init__(self, model: Any, data: Any, scene: HallScene):
        import mujoco

        self.model, self.data, self.scene = model, data, scene
        self._qadr: dict[str, int] = {}
        self._cap: dict[str, int] = {}
        self._body: dict[str, int] = {}
        self.states: dict[str, CallButtonState] = {}
        for b in scene.buttons:
            jid = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_JOINT, button_joint(b.name))
            self._qadr[b.name] = int(model.jnt_qposadr[jid])
            self._cap[b.name] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, button_cap(b.name))
            self._body[b.name] = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, button_body(b.name))
            self.states[b.name] = CallButtonState(b.press_depth)
        self._apply_colors()

    def depth(self, name: str) -> float:
        """ボタンの沈み量 [m]（0 = 押していない、travel = 底）。"""
        return float(self.data.qpos[self._qadr[name]])

    def body_id(self, name: str) -> int:
        return self._body[name]

    def update(self) -> list[str]:
        """点灯の状態を更新する。このステップで押されたボタンの名前を返す。"""
        pressed = [n for n, s in self.states.items() if s.update(self.depth(n))]
        self._apply_colors()
        return pressed

    def lit(self, name: str) -> bool:
        return self.states[name].lit

    def reset_lights(self) -> None:
        for s in self.states.values():
            s.reset()
        self._apply_colors()

    def _apply_colors(self) -> None:
        for b in self.scene.buttons:
            self.model.geom_rgba[self._cap[b.name]] = b.lit_rgba if self.states[b.name].lit else b.off_rgba

    def robot_qpos_slice(self) -> np.ndarray:
        """ロボットの関節の qpos の番号（ボタンのスライド関節以外）。"""
        button_adr = set(self._qadr.values())
        return np.array([i for i in range(self.model.nq) if i not in button_adr], dtype=int)
