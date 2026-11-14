"""パッチ 0112 の続き（11-14・段 C の扉だけの試しで分かった直し）を作業ツリーに当てる。

  python patch_0112b_virtual_latch.py <作業ツリー>/gr00t/rl

扉だけの試し（s0_probe_1114.sh・records/start_1114_0818）で、円錐の掛け金が 2 つ悪さをしていた:
  ① 触れる前から枠に押されて引っ込み、連動してレバーを 13〜26° 回す（レバーのばねを実機の弱さにしたので見えた）
  ② 掛け金の関節の上限（出）とレバーの連動で、レバーが外れる角度より先へ回らない（外れる角度がレバーの止まりになる）
  ③ 開ける向きに押しても円錐の斜面で引っ込むので、掛け金が開ける向きを止めない
中身（どれも既定は切で、元と変わらない）:
  door.py            DoorSpawnerCfg.latch_inert: 円錐の当たりを外し、掛け金の関節の上限を広げる（レバーを止めない）
  door_open_homie.py env.config.virtual_latch: レバーが外れる角度（customData の latchReleaseAngleDeg）より上で、扉が閉じ際
                     zone_deg 以内なら、蝶番を開ける向きにだけ硬いばね（k・d）で止める（実物の掛け金の平らな面と受け）
元のファイルは .bak_0112b に残す。
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

EDITS = {
    "isaac_utils/playground/env_rand/door.py": [
        (
            "    handle_friction_range: Optional[tuple[float, float]] = None\n",
            "    handle_friction_range: Optional[tuple[float, float]] = None\n"
            "    # G1_Hackason（0112b）: 円錐の掛け金を当たらない飾りにする（当たりを外し、関節の上限を広げてレバーを止めない）。\n"
            "    # 掛け金の働きは env.config.virtual_latch が受け持つ（外れる角度は latch_release_angle_deg_range のまま引く）\n"
            "    latch_inert: bool = False\n",
        ),
        (
            "        add_mass(stage, latch_geom_prim_path, mass=0.1)\n"
            "        add_collider(stage, latch_geom_prim_path)\n",
            "        add_mass(stage, latch_geom_prim_path, mass=0.1)\n"
            "        if not cfg.latch_inert:  # G1_Hackason（0112b）: 飾りの掛け金は当たらない\n"
            "            add_collider(stage, latch_geom_prim_path)\n",
        ),
        (
            "        latch_joint.GetLowerLimitAttr().Set(0.0)\n"
            "        latch_joint.GetUpperLimitAttr().Set(latch_throw)\n",
            "        latch_joint.GetLowerLimitAttr().Set(0.0)\n"
            "        # G1_Hackason（0112b）: 飾りの掛け金は、レバーが止まりまで回っても上限に当たらないよう広げる（連動でレバーを止めない）\n"
            "        latch_joint.GetUpperLimitAttr().Set(latch_throw * (3.0 if cfg.latch_inert else 1.0))\n",
        ),
        (
            "        metadata_value[\"latchReleaseAngleDeg\"] = float(latch_release_angle_deg)\n",
            "        metadata_value[\"latchReleaseAngleDeg\"] = float(latch_release_angle_deg)\n"
            "        metadata_value[\"latchInert\"] = bool(cfg.latch_inert)  # G1_Hackason（0112b）\n",
        ),
    ],
    "envs/door/door_open_homie.py": [
        (
            "        self._lever_probe = self._lever_probe_config(self.config.get(\"lever_probe\", None))  # 0112\n",
            "        self._lever_probe = self._lever_probe_config(self.config.get(\"lever_probe\", None))  # 0112\n"
            "        self._vlatch = self._virtual_latch_config(self.config.get(\"virtual_latch\", None))  # 0112b\n",
        ),
        (
            "        if self.any_door_closer_shaped or getattr(self, \"_lever_probe\", None) is not None:\n",
            "        if (self.any_door_closer_shaped or getattr(self, \"_lever_probe\", None) is not None\n"
            "                or getattr(self, \"_vlatch\", None) is not None):\n",
        ),
        (
            "    def _lever_probe_effort(self):\n",
            "    def _virtual_latch_config(self, cfg):\n"
            "        \"\"\"G1_Hackason（0112b・11-14）: 仮想の掛け金。円錐の掛け金（door.py の latch_inert で飾りにする）の代わりに、\n"
            "        レバーの角度が外れる角度より上（＝掛け金が出ている）で、扉が閉じ際 zone_deg 以内なら、蝶番を開ける向きにだけ\n"
            "        硬いばねで止める。扉が zone_deg より開いてからレバーを戻しても止めない（掛け金は枠の外）。閉じてくれば掛かり直す\"\"\"\n"
            "        if not cfg or not cfg.get(\"enabled\", False):\n"
            "            return None\n"
            "        rel = self.door_lever_params[\"latch_release_deg\"]\n"
            "        if not bool(torch.isfinite(rel).all()):\n"
            "            raise ValueError(\"virtual_latch には全部の扉の latchReleaseAngleDeg が要る（build_latch=True の場面）\")\n"
            "        p = {\n"
            "            \"release\": rel * (torch.pi / 180.0),\n"
            "            \"zone\": float(cfg.get(\"zone_deg\", 0.5)) * torch.pi / 180.0,\n"
            "            \"hold\": float(cfg.get(\"hold_deg\", 0.05)) * torch.pi / 180.0,\n"
            "            \"k\": float(cfg.get(\"k\", 30000.0)),\n"
            "            \"d\": float(cfg.get(\"d\", 600.0)),\n"
            "        }\n"
            "        if not (0.0 <= p[\"hold\"] < p[\"zone\"] and p[\"k\"] > 0.0 and p[\"d\"] >= 0.0):\n"
            "            raise ValueError(f\"virtual_latch の値がおかしい: {cfg}\")\n"
            "        print(f\"[0112b] virtual_latch: 閉じ際 {cfg.get('zone_deg', 0.5)}° 以内・外れる角度 \"\n"
            "              f\"{float(rel.min()):.1f}〜{float(rel.max()):.1f}°・k {p['k']}・d {p['d']}\", flush=True)\n"
            "        return p\n"
            "\n"
            "    def _virtual_latch_effort(self, hinge_pos, hinge_vel):\n"
            "        \"\"\"0112b: 掛け金が掛かっている台の蝶番に、開ける向きを止める回り（開く向きが正の回りを負へ）を足す\"\"\"\n"
            "        p = self._vlatch\n"
            "        handle = self.simulator.get_task_dof_pos(\"door\")[:, 1]\n"
            "        engaged = (handle < p[\"release\"]) & (hinge_pos < p[\"zone\"])\n"
            "        pen = torch.clamp(hinge_pos - p[\"hold\"], min=0.0)\n"
            "        tau = p[\"k\"] * pen + p[\"d\"] * torch.clamp(hinge_vel, min=0.0)\n"
            "        self.door_dof_effort[:, 0] -= torch.where(engaged, tau, torch.zeros_like(tau))\n"
            "\n"
            "    def _lever_probe_effort(self):\n",
        ),
        (
            "        if getattr(self, \"_lever_probe\", None) is not None:  # 0112\n"
            "            self._lever_probe_effort()\n",
            "        if getattr(self, \"_lever_probe\", None) is not None:  # 0112\n"
            "            self._lever_probe_effort()\n"
            "        if getattr(self, \"_vlatch\", None) is not None:  # 0112b\n"
            "            self._virtual_latch_effort(hinge_pos, hinge_vel)\n",
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
        if "0112b" in text:
            print(f"[0112b] 当て済み: {path}")
            continue
        if "0112" not in text:
            print(f"[0112b] 先に 0112 を当てる: {path}")
            return 1
        for old, rep in edits:
            n = text.count(old)
            if n != 1:
                print(f"[0112b] 置き換え先が {n} 個（1 個のはず）: {path}\n{old[:200]}")
                return 1
            text = text.replace(old, rep)
        new[path] = text
    for path, text in new.items():
        shutil.copy2(path, str(path) + ".bak_0112b")
        path.write_text(text, encoding="utf-8")
        print(f"[0112b] 書いた: {path}（元は .bak_0112b）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
