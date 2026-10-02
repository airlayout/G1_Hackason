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
# 体の揺れ（評価セット realistic）: 腰に足す関節の名前と、それを動かすばねの強さと減衰。
# 並進 2e4 N/m（ボタンを 5 N で押しても 0.25 mm しか下がらない）、回転 3000 N·m/rad。減衰は臨界減衰の近く
BASE_JOINTS = ("base_x", "base_y", "base_z", "base_roll", "base_pitch", "base_yaw")
BASE_KP_LIN, BASE_KD_LIN = 2.0e4, 1.7e3
BASE_KP_ROT, BASE_KD_ROT = 3.0e3, 1.5e2
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


def add_base_sway(spec: Any) -> None:
    """腰（pelvis）に、前後・左右・上下と 3 方向の傾きの関節を足し、強いばね（位置のアクチュエータ）で目標を追わせる。

    目標を揺らすと体が揺れる（実機の立っているときのふらつきの近似）。ばねなので、腕で押した反動で体がわずかに動く。
    上下の関節は体の重さで下がるので、目標に重さ ÷ 強さを足して打ち消す（MujocoRobot.base_targets）。
    """
    import mujoco

    pelvis = spec.body("pelvis")
    for name, kind, axis in zip(BASE_JOINTS, ["slide"] * 3 + ["hinge"] * 3,
                                [[1, 0, 0], [0, 1, 0], [0, 0, 1]] * 2):
        j = pelvis.add_joint(name=name, type=mujoco.mjtJoint.mjJNT_SLIDE if kind == "slide" else mujoco.mjtJoint.mjJNT_HINGE,
                             axis=axis)
        kp, kd = (BASE_KP_LIN, BASE_KD_LIN) if kind == "slide" else (BASE_KP_ROT, BASE_KD_ROT)
        _set_scalar_or_poly(j, "damping", kd)
        a = spec.add_actuator(name=name, target=name, trntype=mujoco.mjtTrn.mjTRN_JOINT)
        a.gaintype = mujoco.mjtGain.mjGAIN_FIXED
        a.biastype = mujoco.mjtBias.mjBIAS_AFFINE
        a.gainprm[0] = kp
        a.biasprm[1] = -kp


def add_hall(spec: Any, scene: HallScene, robot_cfg: dict[str, Any] | None = None,
             fingertip_cfg: dict[str, Any] | None = None, stance_xy: np.ndarray | None = None,
             stance_yaw: float = 0.0) -> None:
    """壁、扉、ボタン盤、ボタン（と指先の衝突判定）を MjSpec に足す。

    stance_xy / stance_yaw: 立ち位置のずれ（評価セット realistic）。乗り場の側を、pelvis の真下を中心に回してずらす
    （ロボットから見ると、乗り場に対して立ち位置と向きがずれている）。
    """
    import mujoco

    if scene.floor is not None:
        _set_carpet(spec, scene)
    pelvis = np.asarray(spec.body("pelvis").pos, dtype=float)
    offset = np.zeros(3) if stance_xy is None else np.array([stance_xy[0], stance_xy[1], 0.0])
    hall = spec.worldbody.add_body(name=HALL_BODY, pos=(pelvis + offset).tolist(),
                                   quat=[np.cos(stance_yaw / 2), 0.0, 0.0, np.sin(stance_yaw / 2)])

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


def build_hall_model(scene: HallScene, robot_cfg: dict[str, Any], fingertip_cfg: dict[str, Any] | None = None,
                     realism: Any = None) -> Any:
    """G1（胴体固定、頭カメラ付き）+ 乗り場の MjModel。

    realism（common/realism.py の Realism）を渡すと、体の揺れの関節、立ち位置のずれ、関節の armature と摩擦を入れる。
    頭カメラの取り付けの誤差は、robot_cfg（perturbed_robot_cfg でずらしたもの）で渡す。
    """
    spec = j1gen("robot_model").build_spec(robot_cfg, fixed_base=True)
    if realism is not None:
        add_base_sway(spec)
        add_hall(spec, scene, robot_cfg, fingertip_cfg, realism.stance_xy, realism.stance_yaw)
    else:
        add_hall(spec, scene, robot_cfg, fingertip_cfg)
    model = spec.compile()
    if realism is not None:
        import mujoco

        names = j1gen("robot_model").JOINT_NAMES
        dofs = [model.joint(n).dofadr[0] for n in names]
        model.dof_armature[dofs] = realism.armature
        model.dof_frictionloss[dofs] = realism.friction_nm
    return model


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
        # 乗り場の固定の箱（壁・扉・盤）の geom と、ロボットの geom（ワールド・乗り場・ボタン以外）。接触力を測るため
        hall_id = mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_BODY, HALL_BODY)
        self._hall_geoms = set(np.flatnonzero(model.geom_bodyid == hall_id).tolist())
        not_robot = [0, hall_id, *self._body.values()]
        self._robot_geoms = set(np.flatnonzero(~np.isin(model.geom_bodyid, not_robot)).tolist())
        self._f6 = np.zeros(6)
        # 指先の衝突判定の球（add_hall で足したもの。無ければ空）
        self._tips = [g for g in (mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, f"{s}_fingertip_collision")
                                  for s in ("left", "right")) if g >= 0]
        self._caps = {b.name: mujoco.mj_name2id(model, mujoco.mjtObj.mjOBJ_GEOM, button_cap(b.name)) for b in scene.buttons}
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

    def max_robot_contact_force(self) -> float:
        """ロボットと乗り場の固定の箱（壁・扉・盤）の接触の、法線方向の力の最大 [N]（ボタンを押す力は含まない）。"""
        import mujoco

        d = self.data
        best = 0.0
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            if (g1 in self._hall_geoms and g2 in self._robot_geoms) or (g2 in self._hall_geoms and g1 in self._robot_geoms):
                mujoco.mj_contactForce(self.model, d, i, self._f6)
                best = max(best, abs(float(self._f6[0])))
        return best

    def diagnostics(self) -> dict[str, dict[str, float | bool]]:
        """弱点のレポート用の、今の状態（エージェントには渡さない）。

        tip_dist: 指先の球の中心から、ボタンの面の中心までの距離の最小 [m]（球が無ければ入れない）
        depth: ボタンの沈み [m]。touched: ロボットのどこかが、そのボタンに触れているか
        """
        d = self.data
        out: dict[str, dict[str, float | bool]] = {"tip_dist": {}, "depth": {}, "touched": {}}
        for b in self.scene.buttons:
            face = d.xpos[self._body[b.name]]
            if self._tips:
                out["tip_dist"][b.name] = float(min(np.linalg.norm(d.geom_xpos[g] - face) for g in self._tips))
            out["depth"][b.name] = self.depth(b.name)
            out["touched"][b.name] = False
        cap_to_name = {g: n for n, g in self._caps.items()}
        for i in range(d.ncon):
            c = d.contact[i]
            g1, g2 = int(c.geom1), int(c.geom2)
            for a, b in ((g1, g2), (g2, g1)):
                if a in cap_to_name and b in self._robot_geoms:
                    out["touched"][cap_to_name[a]] = True
        return out

    def robot_qpos_slice(self) -> np.ndarray:
        """ロボットの関節の qpos の番号（ボタンのスライド関節以外）。"""
        button_adr = set(self._qadr.values())
        return np.array([i for i in range(self.model.nq) if i not in button_adr], dtype=int)
