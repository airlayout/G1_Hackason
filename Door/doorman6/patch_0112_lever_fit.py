"""パッチ 0112（11-14・計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 C）を作業ツリーに当てる。

  python patch_0112_lever_fit.py <作業ツリー（VIRAL6_fix か VIRAL6_vis）>/gr00t/rl

中身（どれも既定は切で、元と 1 つも変わらない）:
  door.py            DoorSpawnerCfg.handle_spring_*（レバーを角度で増えるばねにする）・handle_friction_range（棒の材質の摩擦）。
                     台ごとの値を扉の customData に残す
  door_open_homie.py 台ごとのレバーと掛け金の値（door_lever_params）・扉だけを動かす試し env.config.lever_probe
  state_recorder.py  static.npz に door_lever_<名前> を足す
元のファイルは .bak_0112 に残す。置き換え先が 1 つでなければ何も書かずに止まる。
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

EDITS = {
    "isaac_utils/playground/env_rand/door.py": [
        (  # import
            "from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics\n",
            "from pxr import Gf, PhysxSchema, Sdf, Usd, UsdGeom, UsdPhysics, UsdShade\n",
        ),
        (  # 設定
            "    latch_release_angle_deg_range: Optional[tuple[float, float]] = None\n",
            "    latch_release_angle_deg_range: Optional[tuple[float, float]] = None\n"
            "    # G1_Hackason（0112・11-14）: レバーの戻す回りを角度で増えるばねにする（実機の記録から。Door/real/fit_door_from_records.py）。\n"
            "    # 回り＝予圧 ＋ 硬さ × 角度 [N·m]（env が掛け続ける 0.26 N·m を引いた正味）。両方 None なら元どおり（ドライブの上限で一定の回り。\n"
            "    # 元の減衰 0.5 /deg は 29 N·m·s/rad で、上限で切られて見えていなかった。ばねの時は handle_spring_damping を使う）\n"
            "    handle_spring_stiffness_range: Optional[tuple[float, float]] = None  # 硬さ [N·m/rad]\n"
            "    handle_spring_preload_range: Optional[tuple[float, float]] = None  # 0° での回り [N·m]\n"
            "    handle_spring_damping: float = 0.3  # 減衰 [N·m·s/rad]（実機の押している間と押さえている間の差 約 0.15 N·m ÷ 0.3 rad/s から）\n"
            "    # G1_Hackason（0112）: レバー（棒）の材質の摩擦（静止＝動き）。相手と組む時は max（機体の材質の乱数を切って使う。\n"
            "    # 機体の既定 0.5 より大きい値なら、この値がそのまま手と棒の摩擦になる）。None なら元どおり（材質を付けない）\n"
            "    handle_friction_range: Optional[tuple[float, float]] = None\n",
        ),
        (  # ばね・摩擦
            "    handle_drive.GetDampingAttr().Set(0.5)\n"
            "    handle_drive.GetStiffnessAttr().Set(50.0)\n"
            "    _update_joint_transform(stage, handle_joint_prim_path, panel_prim_path, handle_prim_path)\n",
            "    handle_drive.GetDampingAttr().Set(0.5)\n"
            "    handle_drive.GetStiffnessAttr().Set(50.0)\n"
            "    lever_metadata = _apply_handle_spring(cfg, handle_drive)  # G1_Hackason（0112）: 指定が無ければ何もしない\n"
            "    _update_joint_transform(stage, handle_joint_prim_path, panel_prim_path, handle_prim_path)\n",
        ),
        (
            "    add_mass(stage, handle_shape_outside_prim_path, mass=0.1)\n"
            "    add_collider(stage, handle_shape_outside_prim_path)\n",
            "    add_mass(stage, handle_shape_outside_prim_path, mass=0.1)\n"
            "    add_collider(stage, handle_shape_outside_prim_path)\n"
            "    handle_friction = _bind_handle_friction(  # G1_Hackason（0112）: 指定が無ければ何もしない\n"
            "        stage, cfg, handle_prim_path, (handle_shape_inside_prim_path, handle_shape_outside_prim_path)\n"
            "    )\n",
        ),
        (  # customData
            "    metadata_value.update(closer_metadata)\n",
            "    metadata_value.update(closer_metadata)\n"
            "    metadata_value.update(lever_metadata)  # G1_Hackason（0112）\n"
            "    if handle_friction is not None:\n"
            "        metadata_value[\"handleFriction\"] = float(handle_friction)\n",
        ),
        (  # 関数
            "def _sample_shaped_closer(cfg: \"DoorSpawnerCfg\", hinge_drive) -> dict:\n",
            "# G1_Hackason（0112）: env がレバーに掛け続ける回り（door_open_homie.py の DOOR_HANDLE_TENSION と同じ値）。ばねはこの分を足して作る\n"
            "HANDLE_TENSION_NM = 15.0 * np.pi / 180.0\n"
            "# ばねの時のドライブの力の上限 [N·m]。ばねの回りより十分大きく（止まりの 45° で 2 N·m 前後）\n"
            "HANDLE_SPRING_MAX_FORCE_NM = 20.0\n"
            "\n"
            "\n"
            "def _apply_handle_spring(cfg: \"DoorSpawnerCfg\", handle_drive) -> dict:\n"
            "    \"\"\"G1_Hackason（0112）: レバーのドライブを 回り＝予圧＋硬さ×角度 のばねにする。返す dict は扉の customData に入れる。\n"
            "    USD の回転のドライブの硬さ・減衰は 1° あたり。ドライブの回り＝硬さ×(目標−角度)。目標を負の角に置いて予圧を作り、\n"
            "    env が下げる向きに掛け続ける 0.26 N·m の分を足す。乱数は他の値を引いた後に引く（指定の無い扉は乱数の順も元どおり）\"\"\"\n"
            "    k_range, tau0_range = cfg.handle_spring_stiffness_range, cfg.handle_spring_preload_range\n"
            "    if k_range is None and tau0_range is None:\n"
            "        return {}\n"
            "    if k_range is None or tau0_range is None:\n"
            "        raise ValueError(\"handle_spring_stiffness_range と handle_spring_preload_range は組で指定する\")\n"
            "    k = float(np.random.uniform(*k_range))\n"
            "    tau0 = float(np.random.uniform(*tau0_range))\n"
            "    if not (k > 0.0 and tau0 >= 0.0 and cfg.handle_spring_damping >= 0.0):\n"
            "        raise ValueError(f\"レバーのばねの値がおかしい: 硬さ {k}・予圧 {tau0}・減衰 {cfg.handle_spring_damping}\")\n"
            "    deg = np.pi / 180.0\n"
            "    handle_drive.GetStiffnessAttr().Set(k * deg)\n"
            "    handle_drive.GetDampingAttr().Set(float(cfg.handle_spring_damping) * deg)\n"
            "    handle_drive.GetTargetPositionAttr().Set(-float(np.degrees((tau0 + HANDLE_TENSION_NM) / k)))\n"
            "    handle_drive.GetMaxForceAttr().Set(HANDLE_SPRING_MAX_FORCE_NM)\n"
            "    return {\"handleSpringK\": k, \"handleSpringTau0\": tau0, \"handleSpringDamping\": float(cfg.handle_spring_damping)}\n"
            "\n"
            "\n"
            "def _bind_handle_friction(stage, cfg: \"DoorSpawnerCfg\", handle_prim_path: str, collider_paths) -> Optional[float]:\n"
            "    \"\"\"G1_Hackason（0112）: レバーの棒の当たりに摩擦の材質を付ける（静止＝動き・組み合わせは max）。付けた摩擦を返す\"\"\"\n"
            "    if cfg.handle_friction_range is None:\n"
            "        return None\n"
            "    mu = float(np.random.uniform(*cfg.handle_friction_range))\n"
            "    if not mu > 0.0:\n"
            "        raise ValueError(f\"レバーの摩擦は正: {mu}\")\n"
            "    material = UsdShade.Material.Define(stage, os.path.join(handle_prim_path, \"lever_physics_material\"))\n"
            "    api = UsdPhysics.MaterialAPI.Apply(material.GetPrim())\n"
            "    api.CreateStaticFrictionAttr().Set(mu)\n"
            "    api.CreateDynamicFrictionAttr().Set(mu)\n"
            "    api.CreateRestitutionAttr().Set(0.0)\n"
            "    PhysxSchema.PhysxMaterialAPI.Apply(material.GetPrim()).CreateFrictionCombineModeAttr().Set(\"max\")\n"
            "    for path in collider_paths:\n"
            "        UsdShade.MaterialBindingAPI.Apply(stage.GetPrimAtPath(path)).Bind(\n"
            "            material, UsdShade.Tokens.weakerThanDescendants, \"physics\"\n"
            "        )\n"
            "    return mu\n"
            "\n"
            "\n"
            "def _sample_shaped_closer(cfg: \"DoorSpawnerCfg\", hinge_drive) -> dict:\n",
        ),
    ],
    "envs/door/door_open_homie.py": [
        (
            "DOOR_CLOSER_METADATA_KEYS = {\n",
            "# G1_Hackason（0112・11-14）: レバーと掛け金の台ごとの値 ← 扉の customData のキー（記録と扉だけの試しが使う。無い扉は NaN）\n"
            "DOOR_LEVER_METADATA_KEYS = {\n"
            "    \"spring_k\": \"handleSpringK\",\n"
            "    \"spring_tau0\": \"handleSpringTau0\",\n"
            "    \"spring_damping\": \"handleSpringDamping\",\n"
            "    \"friction\": \"handleFriction\",\n"
            "    \"max_deg\": \"handleMaxAngleDeg\",\n"
            "    \"drive_max_force\": \"handleDriveMaxForce\",\n"
            "    \"latch_release_deg\": \"latchReleaseAngleDeg\",\n"
            "    \"latch_throw\": \"latchThrow\",\n"
            "}\n"
            "DOOR_CLOSER_METADATA_KEYS = {\n",
        ),
        (
            "        for env_id in range(self.num_envs):\n"
            "            door_prim_path = f\"/World/envs/env_{env_id}/door\"\n",
            "        self.door_lever_params = {  # 0112\n"
            "            name: torch.full((self.num_envs,), float(\"nan\"), device=self.device) for name in DOOR_LEVER_METADATA_KEYS\n"
            "        }\n"
            "        for env_id in range(self.num_envs):\n"
            "            door_prim_path = f\"/World/envs/env_{env_id}/door\"\n",
        ),
        (
            "            self.door_gap_width[env_id] = float(door_metadata.get(\"gapWidth\", 0.002))\n",
            "            self.door_gap_width[env_id] = float(door_metadata.get(\"gapWidth\", 0.002))\n"
            "            for name, key in DOOR_LEVER_METADATA_KEYS.items():  # 0112\n"
            "                if key in door_metadata:\n"
            "                    self.door_lever_params[name][env_id] = float(door_metadata[key])\n",
        ),
        (
            "        self.any_door_closer_shaped = bool(self.door_closer_shaped.any())\n",
            "        self.any_door_closer_shaped = bool(self.door_closer_shaped.any())\n"
            "        self._lever_probe = self._lever_probe_config(self.config.get(\"lever_probe\", None))  # 0112\n",
        ),
        (
            "    @override\n"
            "    def _apply_force_in_physics_step(self):\n"
            "        super()._apply_force_in_physics_step()\n"
            "        if self.any_door_closer_shaped:\n"
            "            self._apply_door_closer_torque()\n",
            "    @override\n"
            "    def _apply_force_in_physics_step(self):\n"
            "        super()._apply_force_in_physics_step()\n"
            "        if self.any_door_closer_shaped or getattr(self, \"_lever_probe\", None) is not None:\n"
            "            self._apply_door_closer_torque()\n"
            "\n"
            "    def _lever_probe_config(self, cfg):\n"
            "        \"\"\"G1_Hackason（0112・11-14）: 扉だけを動かす試し（計画の段 C）。台 i のレバーに handle_torque_max × i/(台数−1) の\n"
            "        下げる回りを handle_start_s から、蝶番に開ける向きの hinge_torque を hinge_start_s から足す。レバーが落ち着いた角度で\n"
            "        ばねを、蝶番が開いたかで掛け金が外れる角度を確かめる（機体は扉から離して置く）。既定は切\"\"\"\n"
            "        if not cfg or not cfg.get(\"enabled\", False):\n"
            "            return None\n"
            "        n = max(1, self.num_envs - 1)\n"
            "        p = {\n"
            "            \"handle_torque\": float(cfg.get(\"handle_torque_max\", 2.5))\n"
            "            * torch.arange(self.num_envs, device=self.device, dtype=torch.float32)\n"
            "            / n,\n"
            "            \"handle_start_s\": float(cfg.get(\"handle_start_s\", 1.0)),\n"
            "            \"hinge_torque\": float(cfg.get(\"hinge_torque\", 30.0)),\n"
            "            \"hinge_start_s\": float(cfg.get(\"hinge_start_s\", 4.0)),\n"
            "        }\n"
            "        if p[\"handle_start_s\"] < 0.0 or p[\"hinge_start_s\"] < p[\"handle_start_s\"] or p[\"hinge_torque\"] < 0.0:\n"
            "            raise ValueError(f\"lever_probe の時刻か力がおかしい: {cfg}\")\n"
            "        print(f\"[0112] lever_probe: レバーへ 0〜{float(p['handle_torque'][-1]):.2f} N·m を {p['handle_start_s']} 秒から・\"\n"
            "              f\"蝶番へ {p['hinge_torque']} N·m を {p['hinge_start_s']} 秒から\", flush=True)\n"
            "        return p\n"
            "\n"
            "    def _lever_probe_effort(self):\n"
            "        \"\"\"0112: 試しの回り（レバーは下げる向き・蝶番は開ける向き）を door_dof_effort に足す\"\"\"\n"
            "        p = self._lever_probe\n"
            "        t = self.episode_length_buf.to(torch.float32) * self.dt\n"
            "        zero = torch.zeros_like(p[\"handle_torque\"])\n"
            "        self.door_dof_effort[:, 1] = DOOR_HANDLE_TENSION + torch.where(t >= p[\"handle_start_s\"], p[\"handle_torque\"], zero)\n"
            "        self.door_dof_effort[:, 0] += torch.where(t >= p[\"hinge_start_s\"], zero + p[\"hinge_torque\"], zero)\n",
        ),
        (
            "        self.door_dof_effort[:, 0] = torch.where(self.door_closer_shaped, torque, torch.zeros_like(torque))\n"
            "        self.simulator.apply_torques_at_task_dof(self._all_env_ids, {\"door\": self.door_dof_effort})\n",
            "        self.door_dof_effort[:, 0] = torch.where(self.door_closer_shaped, torque, torch.zeros_like(torque))\n"
            "        if getattr(self, \"_lever_probe\", None) is not None:  # 0112\n"
            "            self._lever_probe_effort()\n"
            "        self.simulator.apply_torques_at_task_dof(self._all_env_ids, {\"door\": self.door_dof_effort})\n",
        ),
    ],
    "utils/state_recorder.py": [
        (
            "            **{\n"
            "                f\"door_closer_{name}\": value[:k].cpu().numpy()\n"
            "                for name, value in getattr(env, \"door_closer_params\", {}).items()\n"
            "            },\n",
            "            **{\n"
            "                f\"door_closer_{name}\": value[:k].cpu().numpy()\n"
            "                for name, value in getattr(env, \"door_closer_params\", {}).items()\n"
            "            },\n"
            "            # G1_Hackason（0112）: レバーと掛け金の台ごとの値（ばね・摩擦・止まり・外れる角度・出。無い扉は NaN）\n"
            "            **{\n"
            "                f\"door_lever_{name}\": value[:k].cpu().numpy()\n"
            "                for name, value in getattr(env, \"door_lever_params\", {}).items()\n"
            "            },\n",
        ),
    ],
}


def main() -> int:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    root = Path(sys.argv[1])
    new = {}
    for rel, edits in EDITS.items():
        path = root / rel
        text = path.read_text(encoding="utf-8")
        if "0112" in text:
            print(f"[0112] 当て済み: {path}")
            continue
        for old, rep in edits:
            n = text.count(old)
            if n != 1:
                print(f"[0112] 置き換え先が {n} 個（1 個のはず）: {path}\n{old[:200]}")
                return 1
            text = text.replace(old, rep)
        new[path] = text
    for path, text in new.items():
        shutil.copy2(path, str(path) + ".bak_0112")
        path.write_text(text, encoding="utf-8")
        print(f"[0112] 書いた: {path}（元は .bak_0112）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
