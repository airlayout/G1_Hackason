"""パッチ 0115b（11-16・計画 docs/plan/2026-11-16-g1-door-push-line-two-hands-sim.md の 5.3）を 0115 の後に当てる。

  python patch_0115b_door_angle.py <作業ツリー>/gr00t/rl

script_teacher.door_angle: True で、台本へ本当の蝶番の角度（|扉の関節|）を渡す（台本の angle_src_3c "ext" の時に 3c・4a で使う。
実機ではライダーの扉の角度に当たる。手のひらの読みは、添えた手で押して把手の側の手が板から離れると当てにならない）。既定は切。
元のファイルは .bak_0115b に残す。
"""
from __future__ import annotations

import shutil
import sys
from pathlib import Path

EDITS = {
    "envs/door/script_teacher.py": [
        (
            "            \"panel\": bool(cfg.get(\"panel\", False)), \"helper_arm\": bool(cfg.get(\"helper_arm\", False)),\n",
            "            \"panel\": bool(cfg.get(\"panel\", False)), \"helper_arm\": bool(cfg.get(\"helper_arm\", False)),\n"
            "            \"door_angle\": bool(cfg.get(\"door_angle\", False)),  # 0115b: 台本へ本当の蝶番の角度を渡す\n",
        ),
        (
            "            out, q7 = arm.step(float(t[i]), q_real[i], quat, pose, None if tau_real is None else tau_real[i])\n",
            "            out, q7 = arm.step(float(t[i]), q_real[i], quat, pose, None if tau_real is None else tau_real[i],\n"
            "                               door_angle=float(abs(door[i, 0])) if self.cfg.get(\"door_angle\", False) else None)  # 0115b\n",
        ),
    ],
}


def main() -> int:
    if len(sys.argv) != 2:
        print(__doc__)
        return 2
    root = Path(sys.argv[1])
    for rel, edits in EDITS.items():
        path = root / rel
        text = path.read_text(encoding="utf-8")
        if "0115b" in text:
            print(f"[0115b] もう当たっている: {path}")
            return 1
        if "0115:" not in text:
            print(f"[0115b] 先に 0115 を当てる: {path}")
            return 1
        for old, new in edits:
            n = text.count(old)
            if n != 1:
                print(f"[0115b] {path}: 置き換える所が {n} か所（1 か所のはず）:\n{old}")
                return 1
            text = text.replace(old, new)
        shutil.copy2(path, str(path) + ".bak_0115b")
        path.write_text(text, encoding="utf-8")
        print(f"[0115b] 書いた: {path}（元は .bak_0115b）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
