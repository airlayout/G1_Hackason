"""パッチ 0112 の続き 2（11-14・段 C の扉だけの試し records/start_1114_0824 で分かった直し）を作業ツリーに当てる。

  python patch_0112c_spring_preload.py <作業ツリー>/gr00t/rl

試しで、ばねの扉のレバーの落ち着く角度が「硬さ × 角度 ＝ 0.26（env の張り）＋ レバーの重さ（0.1〜0.26 N·m）＋ 試しの回り」で
説明できた。Isaac Lab がドライブの目標角を毎回 0 に書くので、USD に置いた負の目標角（予圧）は効かない。そこで:
  door.py            ばねの扉はドライブの目標角を 0 にし、レバーの体の重力を切る（実機の記録の回りは重さ込みの正味なので）
  door_open_homie.py ばねの扉のレバーに env が掛ける回りを、張り（下げる 0.26 N·m）から −予圧（上げる向き）に替える。
                     lever_probe もこの値に足す
元のファイルは .bak_0112c に残す。
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

EDITS = {
    "isaac_utils/playground/env_rand/door.py": [
        (
            "    lever_metadata = _apply_handle_spring(cfg, handle_drive)  # G1_Hackason（0112）: 指定が無ければ何もしない\n",
            "    lever_metadata = _apply_handle_spring(  # G1_Hackason（0112）: 指定が無ければ何もしない\n"
            "        cfg, handle_drive, stage.GetPrimAtPath(handle_prim_path)\n"
            "    )\n",
        ),
        (
            "def _apply_handle_spring(cfg: \"DoorSpawnerCfg\", handle_drive) -> dict:\n",
            "def _apply_handle_spring(cfg: \"DoorSpawnerCfg\", handle_drive, handle_prim=None) -> dict:\n",
        ),
        (
            "    handle_drive.GetTargetPositionAttr().Set(-float(np.degrees((tau0 + HANDLE_TENSION_NM) / k)))\n"
            "    handle_drive.GetMaxForceAttr().Set(HANDLE_SPRING_MAX_FORCE_NM)\n"
            "    return {\"handleSpringK\": k, \"handleSpringTau0\": tau0, \"handleSpringDamping\": float(cfg.handle_spring_damping)}\n",
            "    # 0112c: Isaac Lab はドライブの目標角を毎回 0 に書く（USD の目標角は効かない）。予圧は env がレバーに掛ける回り\n"
            "    # （door_open_homie.py の _handle_effort_base ＝ −予圧）で入れる。レバーの重さ（棒と鉤で 0.1〜0.26 N·m）は切る\n"
            "    # （実機の記録の回りは重さ込みの正味）\n"
            "    handle_drive.GetTargetPositionAttr().Set(0.0)\n"
            "    handle_drive.GetMaxForceAttr().Set(HANDLE_SPRING_MAX_FORCE_NM)\n"
            "    if handle_prim is not None:\n"
            "        PhysxSchema.PhysxRigidBodyAPI.Apply(handle_prim).CreateDisableGravityAttr().Set(True)\n"
            "    return {\"handleSpringK\": k, \"handleSpringTau0\": tau0, \"handleSpringDamping\": float(cfg.handle_spring_damping),\n"
            "            \"handleGravityOff\": handle_prim is not None}\n",
        ),
    ],
    "envs/door/door_open_homie.py": [
        (
            "        self._lever_probe = self._lever_probe_config(self.config.get(\"lever_probe\", None))  # 0112\n",
            "        # 0112c: レバーに掛け続ける回り。ばねの扉（handleSpringK あり）は −予圧（上げる向き）、ほかは元の張り\n"
            "        spring = torch.isfinite(self.door_lever_params[\"spring_k\"])\n"
            "        self._handle_effort_base = torch.where(\n"
            "            spring, -torch.nan_to_num(self.door_lever_params[\"spring_tau0\"]), torch.full_like(self.door_lever_params[\"spring_k\"], DOOR_HANDLE_TENSION)\n"
            "        )\n"
            "        self._any_handle_spring = bool(spring.any())\n"
            "        if self._any_handle_spring:\n"
            "            self.door_dof_effort[:, 1] = self._handle_effort_base\n"
            "            print(f\"[0112c] レバーのばね: {int(spring.sum())}/{self.num_envs} 台・予圧 \"\n"
            "                  f\"{float(self.door_lever_params['spring_tau0'][spring].min()):.2f}〜{float(self.door_lever_params['spring_tau0'][spring].max()):.2f} N·m\", flush=True)\n"
            "        self._lever_probe = self._lever_probe_config(self.config.get(\"lever_probe\", None))  # 0112\n",
        ),
        (
            "        if (self.any_door_closer_shaped or getattr(self, \"_lever_probe\", None) is not None\n"
            "                or getattr(self, \"_vlatch\", None) is not None):\n",
            "        if (self.any_door_closer_shaped or getattr(self, \"_lever_probe\", None) is not None\n"
            "                or getattr(self, \"_vlatch\", None) is not None or getattr(self, \"_any_handle_spring\", False)):\n",
        ),
        (
            "        self.door_dof_effort[:, 1] = DOOR_HANDLE_TENSION + torch.where(t >= p[\"handle_start_s\"], p[\"handle_torque\"], zero)\n",
            "        self.door_dof_effort[:, 1] = self._handle_effort_base + torch.where(t >= p[\"handle_start_s\"], p[\"handle_torque\"], zero)\n",
        ),
        (  # 回の戻しでレバーに張りを掛け直す所も、ばねの扉は −予圧に
            "        door_dof_target[:, 1] = DOOR_HANDLE_TENSION  # tension the door handle\n",
            "        door_dof_target[:, 1] = DOOR_HANDLE_TENSION  # tension the door handle\n"
            "        if getattr(self, \"_any_handle_spring\", False):  # 0112c: ばねの扉は −予圧\n"
            "            door_dof_target[:, 1] = self._handle_effort_base\n",
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
        if "0112c" in text:
            print(f"[0112c] 当て済み: {path}")
            continue
        if "0112b" not in text and rel.endswith("door_open_homie.py"):
            print(f"[0112c] 先に 0112・0112b を当てる: {path}")
            return 1
        for old, rep in edits:
            n = text.count(old)
            if n != 1:
                print(f"[0112c] 置き換え先が {n} 個（1 個のはず）: {path}\n{old[:200]}")
                return 1
            text = text.replace(old, rep)
        new[path] = text
    for path, text in new.items():
        shutil.copy2(path, str(path) + ".bak_0112c")
        path.write_text(text, encoding="utf-8")
        print(f"[0112c] 書いた: {path}（元は .bak_0112c）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
