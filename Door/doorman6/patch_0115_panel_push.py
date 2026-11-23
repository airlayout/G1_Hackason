"""パッチ 0115（11-16・計画 docs/plan/2026-11-16-g1-door-push-line-two-hands-sim.md）を作業ツリーに当てる。

  python patch_0115_panel_push.py <作業ツリー>/gr00t/rl <panel_push.py と pelvis_replay.py の置き場>

中身（既定は切で、元と変わらない）:
  envs/door/panel_push.py     新しい口（板の接触から体を回す力 Mz・3c・4a の柔らかい腕）
  envs/door/pelvis_replay.py  cmd_follow.src panel（本当の力と Mz で動く脚）・一歩ごとの揺れ（osc_*）
  door_open_homie.py          env.config.panel_contact.enabled で板の接触センサ（両手首・胴・当たる点つき）を作る
  script_teacher.py           panel・helper_arm（添えた手も動かす）・comply（柔らかい腕）
元のファイルは .bak_0115 に残す。
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

EDITS = {
    "envs/door/door_open_homie.py": [
        (
            "        simulator.scene.sensors[\"door_panel_unwanted_contact_sensor\"] = ContactSensor(\n"
            "            door_panel_unwanted_contact_sensor_config\n"
            "        )\n",
            "        simulator.scene.sensors[\"door_panel_unwanted_contact_sensor\"] = ContactSensor(\n"
            "            door_panel_unwanted_contact_sensor_config\n"
            "        )\n"
            "        # 0115: 扉の板が両手首・胴から受ける力と当たる点（3c の押し方の新案の物差し・panel_push.py。既定は作らない）\n"
            "        _cfg0115 = getattr(self, \"config\", None)\n"
            "        _pc0115 = _cfg0115.get(\"panel_contact\", None) if _cfg0115 is not None else None\n"
            "        if _pc0115 is not None and bool(_pc0115.get(\"enabled\", False)):\n"
            "            from gr00t.rl.envs.door.panel_push import PANEL_BODIES, panel_sensor_name  # noqa: PLC0415\n"
            "            for _b in PANEL_BODIES:  # 体ごとに 1 つ（どの力がどの体かを取り違えないように）\n"
            "                simulator.scene.sensors[panel_sensor_name(_b)] = ContactSensor(ContactSensorCfg(\n"
            "                    prim_path=f\"/World/envs/env_.*/{simulator.task_config.target_obj}/door_panel\",\n"
            "                    filter_prim_paths_expr=[f\"/World/envs/env_.*/Robot/{_b}\"],\n"
            "                    track_contact_points=True,\n"
            "                ))\n",
        ),
    ],
    "envs/door/script_teacher.py": [
        (
            "            \"tau_src\": str(cfg.get(\"tau_src\", \"env\")),\n",
            "            \"tau_src\": str(cfg.get(\"tau_src\", \"env\")),\n"
            "            # 0115: 板の接触から体を回す力・添えた手（反対の腕）も動かす・3c・4a の柔らかい腕\n"
            "            \"panel\": bool(cfg.get(\"panel\", False)), \"helper_arm\": bool(cfg.get(\"helper_arm\", False)),\n"
            "            \"comply\": ({k: cfg.get(\"comply\")[k] for k in cfg.get(\"comply\")}\n"
            "                       if cfg.get(\"comply\", None) is not None and bool(cfg.get(\"comply\").get(\"enabled\", False)) else None),\n",
        ),
        (
            "        self.hand_c_t = torch.full((env.num_envs, 3), float(\"nan\"), device=env.device)\n",
            "        self.hand_c_t = torch.full((env.num_envs, 3), float(\"nan\"), device=env.device)\n"
            "        # 0115: 3c の押し方の新案（panel_push.py）。板とレバーから体が受ける押し返し・回す力 Mz（押されて下がる脚が読む）\n"
            "        self.body_mz_t = torch.full((env.num_envs,), float(\"nan\"), device=env.device)\n"
            "        self.body_push_t = torch.full((env.num_envs,), float(\"nan\"), device=env.device)\n"
            "        self.helper_on = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)\n"
            "        self.lever_left_t = torch.zeros(env.num_envs, dtype=torch.bool, device=env.device)\n"
            "        self.panel, self.comply, self.pp_np, self.panel_row = None, None, None, ()\n"
            "        if cfg.get(\"panel\", False):\n"
            "            from gr00t.rl.envs.door.panel_push import ROW, PanelPush  # noqa: PLC0415\n"
            "            self.panel, self.panel_row = PanelPush(env, self.n, self.dscr.SUBS), ROW\n"
            "        if cfg.get(\"comply\", None):\n"
            "            from gr00t.rl.envs.door.panel_push import ArmComply  # noqa: PLC0415\n"
            "            self.comply = ArmComply(env, cfg[\"comply\"], self.sim_from_real, self.dscr.SUBS)\n",
        ),
        (
            "        lever_left = bool(env.door_open_lr[i] > 0)\n",
            "        lever_left = bool(env.door_open_lr[i] > 0)\n"
            "        self.lever_left_t[i] = lever_left  # 0115\n",
        ),
        (
            "                     if self.replay is not None and self.replay.method == \"door\" else None)\n",
            "                     if self.replay is not None and self.replay.method == \"door\" else None)\n"
            "        self.pp_np = None\n"
            "        if self.panel is not None:  # 0115: 板とレバーから体が受ける力と Mz（前の周期の小段で 3c・4a を貯める）\n"
            "            pp = self.panel.measure(env, self.sub_code)\n"
            "            self.body_mz_t.copy_(pp[\"mz\"])\n"
            "            self.body_push_t.copy_(pp[\"push\"])\n"
            "            self.pp_np = torch.stack([pp[k][:n] for k in self.panel_row], dim=-1).detach().cpu().numpy()\n",
        ),
        (
            "            if q7 is None:\n"
            "                continue  # 腕を下ろしたまま・止まった: 直前の目標を保つ\n",
            "            qo = arm.arm_other if self.cfg.get(\"helper_arm\", False) else None\n"
            "            if qo is not None:  # 0115: 把手と反対の手（台本の helper）も動かす\n"
            "                for k, r in enumerate(arm.helper.idx):\n"
            "                    s = int(self.sim_from_real[r])\n"
            "                    D[i, ARM_ACCUM_START + s - self.num_lower] = (qo[k] - default[i, s]) / self.action_scale\n"
            "                self.hold[i] = D[i, ARM_ACCUM_START:ARM_ACCUM_START + NUM_ARM].copy()\n"
            "                self.helper_on[i] = True\n"
            "            if q7 is None:\n"
            "                continue  # 腕を下ろしたまま・止まった: 直前の目標を保つ\n",
        ),
        (
            "[door[i, 0], door[i, 1]], f_est, f_true, [self.twist_off[i]]]))\n",
            "[door[i, 0], door[i, 1]], f_est, f_true, [self.twist_off[i]]]\n"
            "                                                     + ([self.pp_np[i]] if self.pp_np is not None else [])))  # 0115: 後ろに panel_push.ROW\n",
        ),
        (
            "        self.hand_f_t[:n] = torch.as_tensor(f_est_all, dtype=torch.float32, device=env.device)  # 0114\n",
            "        if self.comply is not None:  # 0115: 3c・4a の腕を柔らかく（把手の側の腕と、添えた手を使う台の反対の腕）\n"
            "            self.comply.step(env, self.sub_code, self.lever_left_t | self.helper_on, (~self.lever_left_t) | self.helper_on)\n"
            "        self.hand_f_t[:n] = torch.as_tensor(f_est_all, dtype=torch.float32, device=env.device)  # 0114\n",
        ),
        (
            "        s[\"hand_force\"] = self._force_summary(i)\n",
            "        s[\"hand_force\"] = self._force_summary(i)\n"
            "        if getattr(self, \"panel\", None) is not None:  # 0115\n"
            "            s[\"panel\"] = self.panel.summary(i)\n"
            "            self.panel.reset_env(i)\n",
        ),
        (
            "                self.hf_palm[i] = None\n"
            "        self.sub_code[env_ids] = -1\n",
            "                self.hf_palm[i] = None\n"
            "                self.helper_on[i] = False  # 0115\n"
            "        if self.comply is not None:  # 0115: 回の戻しで腕の硬さを元へ\n"
            "            self.comply.reset(env_ids)\n"
            "        self.sub_code[env_ids] = -1\n",
        ),
    ],
}
COPIES = ("panel_push.py", "pelvis_replay.py")


def main() -> int:
    if len(sys.argv) != 3:
        print(__doc__)
        return 2
    root, src = Path(sys.argv[1]), Path(sys.argv[2])
    texts = {}
    for rel, edits in EDITS.items():
        path = root / rel
        text = path.read_text(encoding="utf-8")
        if "0115" in text:
            print(f"[0115] もう当たっている: {path}")
            return 1
        for old, new in edits:
            n = text.count(old)
            if n != 1:
                print(f"[0115] {path}: 置き換える所が {n} か所（1 か所のはず）:\n{old}")
                return 1
            text = text.replace(old, new)
        texts[path] = text
    for path, text in texts.items():
        shutil.copy2(path, str(path) + ".bak_0115")
        path.write_text(text, encoding="utf-8")
        print(f"[0115] 書いた: {path}（元は .bak_0115）")
    for name in COPIES:
        dst = root / "envs/door" / name
        if dst.exists():
            shutil.copy2(dst, str(dst) + ".bak_0115")
        shutil.copy2(src / name, dst)
        print(f"[0115] 写した: {dst}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
