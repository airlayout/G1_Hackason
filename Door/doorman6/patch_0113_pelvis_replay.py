"""パッチ 0113（11-14・計画 docs/plan/2026-11-14-g1-door-sim-lever-bench-from-real-records.md の段 D）を作業ツリーに当てる。

  python patch_0113_pelvis_replay.py <作業ツリー>/gr00t/rl <pelvis_replay.py の置き場>

中身（既定は切で、元と変わらない）:
  envs/door/pelvis_replay.py  新しい口（実機の pelvis の動きの再生。方法 1 vel・方法 2 door）
  door_open_homie.py          env.config.pelvis_replay を読み、台本の教師の apply の直後・物理 1 回ごと・回の戻しで呼ぶ
  script_teacher.py           方法 door の時は pelvis を扉の根元からの姿勢で読む・回ごとの結果に再生の値を足す
元のファイルは .bak_0113 に残す。
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

EDITS = {
    "envs/door/door_open_homie.py": [
        (
            "from gr00t.rl.envs.door.script_teacher import ScriptTeacher, script_teacher_config\n",
            "from gr00t.rl.envs.door.script_teacher import ScriptTeacher, script_teacher_config\n"
            "from gr00t.rl.envs.door.pelvis_replay import PelvisReplay, pelvis_replay_config  # 0113\n",
        ),
        (
            "        self._script_teacher_cfg = script_teacher_config(self.config.get(\"script_teacher\", None))\n"
            "        self._script_teacher = None\n",
            "        self._script_teacher_cfg = script_teacher_config(self.config.get(\"script_teacher\", None))\n"
            "        self._script_teacher = None\n"
            "        # 0113: 実機の pelvis の動きの再生（pelvis_replay.py。台本の教師が要る。作るのは台本の教師と同じく最初の歩）\n"
            "        self._pelvis_replay_cfg = pelvis_replay_config(self.config.get(\"pelvis_replay\", None))\n"
            "        assert self._pelvis_replay_cfg is None or self._script_teacher_cfg is not None, \\\n"
            "            \"pelvis_replay には script_teacher が要る（再生を始める小段を台本から読む）\"\n"
            "        self._pelvis_replay = None\n",
        ),
        (
            "            self._script_teacher.apply(self)\n",
            "            if getattr(self, \"_pelvis_replay_cfg\", None) is not None and self._pelvis_replay is None:  # 0113\n"
            "                self._pelvis_replay = PelvisReplay(self, self._pelvis_replay_cfg)\n"
            "                self._script_teacher.extra_summary = self._pelvis_replay.summary\n"
            "                self._script_teacher.replay = self._pelvis_replay\n"
            "            self._script_teacher.apply(self)\n"
            "            if self._pelvis_replay is not None:  # 0113: 止める・再生を始める台を決める\n"
            "                self._pelvis_replay.control_step(self)\n",
        ),
        (
            "        if getattr(self, \"_script_teacher\", None) is not None:  # 0108: 台本を回ごとに作り直す\n"
            "            self._script_teacher.reset(env_ids)\n",
            "        if getattr(self, \"_script_teacher\", None) is not None:  # 0108: 台本を回ごとに作り直す\n"
            "            self._script_teacher.reset(env_ids)\n"
            "        if getattr(self, \"_pelvis_replay\", None) is not None:  # 0113: 再生を止め、方法 door なら扉を戻す（台本の結果を書いた後）\n"
            "            self._pelvis_replay.reset(self, env_ids)\n",
        ),
        (
            "    @override\n"
            "    def _apply_force_in_physics_step(self):\n"
            "        super()._apply_force_in_physics_step()\n",
            "    @override\n"
            "    def _apply_force_in_physics_step(self):\n"
            "        super()._apply_force_in_physics_step()\n"
            "        if getattr(self, \"_pelvis_replay\", None) is not None:  # 0113: 物理 1 回ごとに根元の速さ（と扉）を書く\n"
            "            self._pelvis_replay.physics_substep(self)\n",
        ),
    ],
    "envs/door/script_teacher.py": [
        (
            "        self.pairs: list = [[] for _ in range(self.n)]   # 0111: (小段, 見積もり xyz, 接触 xyz, 手のひらの原点で解いた見積もり xyz)\n",
            "        self.pairs: list = [[] for _ in range(self.n)]   # 0111: (小段, 見積もり xyz, 接触 xyz, 手のひらの原点で解いた見積もり xyz)\n"
            "        self.extra_summary = None  # 0113: 回ごとの結果に足す値（pelvis_replay.summary）\n"
            "        self.replay = None  # 0113: 方法 door の時は pelvis を扉の根元からの姿勢で読む\n",
        ),
        (
            "        s[\"last_sub\"] = s[\"subs\"][-1][0] if s[\"subs\"] else \"\"\n",
            "        s[\"last_sub\"] = s[\"subs\"][-1][0] if s[\"subs\"] else \"\"\n"
            "        if self.extra_summary is not None:  # 0113\n"
            "            s.update(self.extra_summary(i))\n",
        ),
        (
            "        dt = float(env.dt)\n"
            "        for i in range(n):\n",
            "        dt = float(env.dt)\n"
            "        door_root = (self.replay.door_root_planar(env, n)  # 0113: 方法 door は扉が動く\n"
            "                     if self.replay is not None and self.replay.method == \"door\" else None)\n"
            "        for i in range(n):\n",
        ),
        (
            "            pose = self.dfr.PlanarPose(root[i, 0] - origin[i, 0], root[i, 1] - origin[i, 1], self.dfr.yaw_from_quat_wxyz(quat))\n",
            "            pose = self.dfr.PlanarPose(root[i, 0] - origin[i, 0], root[i, 1] - origin[i, 1], self.dfr.yaw_from_quat_wxyz(quat))\n"
            "            if door_root is not None:  # 0113: 扉の根元からの姿勢（平面）\n"
            "                dx, dy, dyaw = pose.x - door_root[i, 0], pose.y - door_root[i, 1], door_root[i, 2]\n"
            "                c, s_ = math.cos(dyaw), math.sin(dyaw)\n"
            "                pose = self.dfr.PlanarPose(c * dx + s_ * dy, -s_ * dx + c * dy, pose.yaw - dyaw)\n",
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
        if "0113" in text:
            print(f"[0113] 当て済み: {path}")
            continue
        for old, rep in edits:
            n = text.count(old)
            if n != 1:
                print(f"[0113] 置き換え先が {n} 個（1 個のはず）: {path}\n{old[:200]}")
                return 1
            text = text.replace(old, rep)
        new[path] = text
    for path, text in new.items():
        shutil.copy2(path, str(path) + ".bak_0113")
        path.write_text(text, encoding="utf-8")
        print(f"[0113] 書いた: {path}（元は .bak_0113）")
    dst = root / "envs/door/pelvis_replay.py"
    shutil.copy2(src, dst)
    print(f"[0113] 写した: {dst}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
