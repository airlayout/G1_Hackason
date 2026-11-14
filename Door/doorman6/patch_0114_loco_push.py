"""パッチ 0114（11-14・計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 G・13.1）を作業ツリーに当てる。

  python patch_0114_loco_push.py <作業ツリー>/gr00t/rl <loco_push.py の置き場>

中身（既定は切で、元と変わらない）:
  envs/door/loco_push.py  新しい口（押されて下がる脚・歩き出しの横揺れ。⚠️ 推測で作った形）
  door_open_homie.py      env.config.loco_push を読み、台本の教師の apply の直後・観測を作る直前・回の戻しで呼ぶ。
                          回ごとの結果に足す値を、再生（0113）と押されて下がる脚（0114）の両方から集める
  script_teacher.py       台ごとの手の力（台本の見積もり・接触センサ。扉の座標）をテンソルに持つ（hand_f_t・hand_c_t）
元のファイルは .bak_0114 に残す。0113 の後に当てる。
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

EDITS = {
    "envs/door/door_open_homie.py": [
        (
            "from gr00t.rl.envs.door.pelvis_replay import PelvisReplay, pelvis_replay_config  # 0113\n",
            "from gr00t.rl.envs.door.pelvis_replay import PelvisReplay, pelvis_replay_config  # 0113\n"
            "from gr00t.rl.envs.door.loco_push import LocoPush, loco_push_config  # 0114\n",
        ),
        (
            "        self._pelvis_replay = None\n",
            "        self._pelvis_replay = None\n"
            "        # 0114: 押されて下がる脚と歩き出しの横揺れ（loco_push.py。台本の教師の手の力と応え方の層が要る）\n"
            "        self._loco_push_cfg = loco_push_config(self.config.get(\"loco_push\", None))\n"
            "        assert self._loco_push_cfg is None or self._script_teacher_cfg is not None, \"loco_push には script_teacher が要る\"\n"
            "        self._loco_push = None\n",
        ),
        (
            "                self._script_teacher.extra_summary = self._pelvis_replay.summary\n",
            "                self._script_teacher.extra_summary = self._st_extra_summary  # 0114: 再生と押されて下がる脚の両方\n",
        ),
        (
            "            if self._pelvis_replay is not None:  # 0113: 止める・再生を始める台を決める\n"
            "                self._pelvis_replay.control_step(self)\n",
            "            if self._pelvis_replay is not None:  # 0113: 止める・再生を始める台を決める\n"
            "                self._pelvis_replay.control_step(self)\n"
            "            if getattr(self, \"_loco_push_cfg\", None) is not None:  # 0114: 押されて下がる・歩き出しの横揺れ\n"
            "                if self._loco_push is None:\n"
            "                    self._loco_push = LocoPush(self, self._loco_push_cfg)\n"
            "                    self._script_teacher.extra_summary = self._st_extra_summary\n"
            "                self._loco_push.control_step(self)\n",
        ),
        (
            "        if getattr(self, \"_pelvis_replay\", None) is not None:  # 0113: 再生を止め、方法 door なら扉を戻す（台本の結果を書いた後）\n"
            "            self._pelvis_replay.reset(self, env_ids)\n",
            "        if getattr(self, \"_pelvis_replay\", None) is not None:  # 0113: 再生を止め、方法 door なら扉を戻す（台本の結果を書いた後）\n"
            "            self._pelvis_replay.reset(self, env_ids)\n"
            "        if getattr(self, \"_loco_push\", None) is not None:  # 0114\n"
            "            self._loco_push.reset(env_ids)\n",
        ),
        (
            "    @override\n"
            "    def _post_config_observation_callback(self):\n",
            "    def _st_extra_summary(self, i: int) -> dict:\n"
            "        \"\"\"0114: 台本の教師の回ごとの結果に足す値（再生 0113・押されて下がる脚 0114）\"\"\"\n"
            "        out = {}\n"
            "        for obj in (getattr(self, \"_pelvis_replay\", None), getattr(self, \"_loco_push\", None)):\n"
            "            if obj is not None:\n"
            "                out.update(obj.summary(i))\n"
            "        return out\n"
            "\n"
            "    @override\n"
            "    def _post_config_observation_callback(self):\n"
            "        if getattr(self, \"_loco_push\", None) is not None:  # 0114: 脚に届く指令に歩き出しの横揺れを足す（観測を作る前）\n"
            "            self._loco_push.apply_legs(self)\n",
        ),
    ],
    "envs/door/script_teacher.py": [
        (
            "        self.extra_summary = None  # 0113: 回ごとの結果に足す値（pelvis_replay.summary）\n",
            "        self.extra_summary = None  # 0113: 回ごとの結果に足す値（pelvis_replay.summary）\n"
            "        # 0114: 台ごとの手の力（扉の座標・N）。台本の見積もり（実機と同じ）と接触センサ。押されて下がる脚（loco_push）が読む\n"
            "        self.hand_f_t = torch.full((env.num_envs, 3), float(\"nan\"), device=env.device)\n"
            "        self.hand_c_t = torch.full((env.num_envs, 3), float(\"nan\"), device=env.device)\n",
        ),
        (
            "        dt = float(env.dt)\n"
            "        door_root = (",
            "        dt = float(env.dt)\n"
            "        f_est_all = np.full((n, 3), np.nan)  # 0114\n"
            "        f_con_all = np.full((n, 3), np.nan)\n"
            "        door_root = (",
        ),
        (
            "            else:\n"
            "                f_true, f_est = np.full(3, np.nan), np.full(3, np.nan)\n",
            "            else:\n"
            "                f_true, f_est = np.full(3, np.nan), np.full(3, np.nan)\n"
            "            f_est_all[i], f_con_all[i] = f_est, f_true  # 0114\n",
        ),
        (
            "        if changed.any():\n"
            "            ids = torch.as_tensor(np.flatnonzero(changed), device=env.device)\n",
            "        self.hand_f_t[:n] = torch.as_tensor(f_est_all, dtype=torch.float32, device=env.device)  # 0114\n"
            "        self.hand_c_t[:n] = torch.as_tensor(f_con_all, dtype=torch.float32, device=env.device)\n"
            "        if changed.any():\n"
            "            ids = torch.as_tensor(np.flatnonzero(changed), device=env.device)\n",
        ),
    ],
}


def main() -> int:
    if len(sys.argv) != 3:
        sys.exit(__doc__)
    root, src = Path(sys.argv[1]), Path(sys.argv[2])
    new = {}
    for rel, edits in EDITS.items():
        path = root / rel
        text = path.read_text(encoding="utf-8")
        if "0114" in text:
            print(f"[0114] 当て済み: {path}")
            continue
        if "0113" not in text:
            print(f"[0114] 先に 0113 を当てる: {path}")
            return 1
        for old, rep in edits:
            n = text.count(old)
            if n != 1:
                print(f"[0114] 置き換え先が {n} 個（1 個のはず）: {path}\n{old[:200]}")
                return 1
            text = text.replace(old, rep)
        new[path] = text
    for path, text in new.items():
        shutil.copy2(path, str(path) + ".bak_0114")
        path.write_text(text, encoding="utf-8")
        print(f"[0114] 書いた: {path}（元は .bak_0114）")
    dst = root / "envs/door/loco_push.py"
    shutil.copy2(src, dst)
    print(f"[0114] 写した: {dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
