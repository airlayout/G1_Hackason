"""パッチ 0113 の続き（11-14・段 E の確かめで分かった直し）を作業ツリーに当てる。

  python patch_0113b_init_arm.py <作業ツリー>/gr00t/rl

段 E の 1 回目（7 cm の組）で、シムの台の 13/24 が 2b で「手がレバーの上に着かない」で止まった（11-13 のシムも 18/32）。
腕は台本の指令どおりに動いていたが、逆運動学が肩 roll 97%・手首 yaw 8% など可動域の端に貼り付いていた。
始めの腕の形が実機と違う: 実機は純正の腕を下ろした形（左 0.28, 0.20, −0.02, 0.99, 0.19, 0.04, −0.02）、シムは全部 0（肘まっすぐ）で
手首は回ごとに ±1.35 rad ばらばら。逆運動学は今の形から少しずつ動かすので、たどり着く形も変わる（2b を通った台でも肩 roll 2.04 対 1.33）。
  script_teacher.py  env.config.script_teacher.init_arm（14 個・SDK の並びの左腕 7・右腕 7）: 回の始めの腕の目標をこの形にする
                     （2a の 1.5 秒で寄せる。既定 None＝元どおり今の角度を保つ）
元のファイルは .bak_0113b に残す。
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

EDITS = {
    "envs/door/script_teacher.py": [
        (
            "            \"tau_src\": str(cfg.get(\"tau_src\", \"env\"))}\n",
            "            \"tau_src\": str(cfg.get(\"tau_src\", \"env\")),\n"
            "            \"init_arm\": None if cfg.get(\"init_arm\", None) is None else [float(v) for v in cfg.get(\"init_arm\")]}\n",
        ),
        (
            "        self.extra_summary = None  # 0113: 回ごとの結果に足す値（pelvis_replay.summary）\n",
            "        self.extra_summary = None  # 0113: 回ごとの結果に足す値（pelvis_replay.summary）\n"
            "        # 0113b: 回の始めの腕の目標（SDK の並びの左腕 7・右腕 7）。実機の純正の腕を下ろした形に寄せる\n"
            "        self.init_arm = cfg.get(\"init_arm\", None)\n"
            "        assert self.init_arm is None or len(self.init_arm) == NUM_ARM, f\"script_teacher.init_arm は 14 個: {self.init_arm}\"\n",
        ),
        (
            "            if self.hold[i] is None:  # 回の始め: 今の角度を保つ目標から（方策の出力は腕に通さない）\n"
            "                self.hold[i] = (q_sim[i, self.num_lower:self.num_lower + NUM_ARM]\n"
            "                                - default[i, self.num_lower:self.num_lower + NUM_ARM]) / self.action_scale\n",
            "            if self.hold[i] is None:  # 回の始め: 今の角度を保つ目標から（方策の出力は腕に通さない）\n"
            "                q_hold = q_sim[i].copy()\n"
            "                if self.init_arm is not None:  # 0113b: 実機の始めの腕の形へ寄せる（2a の間に PD で動く）\n"
            "                    for k in range(NUM_ARM):\n"
            "                        q_hold[int(self.sim_from_real[15 + k])] = self.init_arm[k]\n"
            "                self.hold[i] = (q_hold[self.num_lower:self.num_lower + NUM_ARM]\n"
            "                                - default[i, self.num_lower:self.num_lower + NUM_ARM]) / self.action_scale\n",
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
        if "0113b" in text:
            print(f"[0113b] 当て済み: {path}")
            continue
        for old, rep in edits:
            n = text.count(old)
            if n != 1:
                print(f"[0113b] 置き換え先が {n} 個（1 個のはず）: {path}\n{old[:200]}")
                return 1
            text = text.replace(old, rep)
        new[path] = text
    for path, text in new.items():
        shutil.copy2(path, str(path) + ".bak_0113b")
        path.write_text(text, encoding="utf-8")
        print(f"[0113b] 書いた: {path}（元は .bak_0113b）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
