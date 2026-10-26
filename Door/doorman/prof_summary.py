"""prof_1023.sh の py-spy の記録（raw＝畳んだ呼び出しの履歴）を、学習の 1 回の中の時間の使い道に分ける。

学習の繰り返し（ppo_trainer.py の train）の中の標本だけを数え、葉に近い方から次の決まりで分類する:
  物理の 1 刻み（simulate_at_each_physics_step）の中は行で分ける: 物理を進める（sim.step）・描画（sim.render）・
    場面の更新（scene.update。カメラの更新と、カメラが頼む描画を含む）・物理への書き込み（write_data_to_sim）
  それ以外: 画像の取り出し・画像の加工・ResNet（集める段）・HOMIE・観測・報酬・学習の段（順伝播・逆伝播・重みの更新・
    データの組み立て）・片付け（empty_cache・gc）・その他
行番号は、渡した作業場所の isaacsim.py から毎回探す（コードを直すと行がずれるため）。

使い方: python prof_summary.py <prof.txt> <作業場所（VIRAL6_speed）> [--top 30]
"""
from __future__ import annotations

import argparse
import re
from collections import Counter
from pathlib import Path

ISAAC = "gr00t/rl/simulator/isaacsim/isaacsim.py"


def physics_lines(repo: Path) -> dict[int, str]:
    """simulate_at_each_physics_step の中の、分類したい呼び出しの行番号"""
    src = (repo / ISAAC).read_text().splitlines()
    start = next(i for i, l in enumerate(src) if "def simulate_at_each_physics_step" in l)
    want = {"self.scene.write_data_to_sim()": "物理への書き込み", "self.sim.step(render=False)": "物理を進める",
            "self.sim.render()": "描画（render の呼び出し）", "self.scene.update(": "場面の更新（カメラを含む）"}
    out = {}
    for i in range(start, start + 40):
        for k, v in want.items():
            if k in src[i]:
                out[i + 1] = v
    return out


RULES = [  # （部分文字列, 分類）。葉に近い方から最初に当たったもの
    ("get_rgb_image", "画像の取り出し"),
    ("_image_augmentation", "画像の加工"),
    ("_grayscale_image", "画像の加工"),
    ("_camera_pitch_debug", "確かめの記録"),
    ("backward", "学習の段：逆伝播"),
    ("_forward_model", "学習の段：順伝播"),
    ("optimizer", "学習の段：重みの更新"),
    ("_get_rollout_data", "学習の段：データの組み立て"),
    ("_get_mb_rollout_data", "学習の段：データの組み立て"),
    ("empty_cache", "片付け"),
    ("gc.collect", "片付け"),
    ("collect (", "片付け"),
    ("homie", "HOMIE"),
    ("rollout_with_obj_pred", "生徒の計算（集める段）"),
    ("teacher", "教師の計算"),
    ("_compute_reward", "報酬"),
    ("compute_reward", "報酬"),
    ("_compute_observations", "観測"),
    ("compute_observations", "観測"),
]


def classify(frames: list[str], phys: dict[int, str]) -> str:
    for f in reversed(frames):  # 葉から
        m = re.search(r"simulate_at_each_physics_step \(.*isaacsim\.py:(\d+)\)", f)
        if m:
            return phys.get(int(m.group(1)), "物理の 1 刻み（その他）")
        for key, cat in RULES:
            if key in f:
                return cat
    if any("_rollout_step" in f for f in frames):
        return "集める段（その他）"
    return "学習の繰り返し（その他）"


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("prof")
    ap.add_argument("repo", type=Path)
    ap.add_argument("--top", type=int, default=30)
    a = ap.parse_args()
    phys = physics_lines(a.repo)
    cats, frames_incl = Counter(), Counter()
    total = in_loop = collect = 0
    for line in Path(a.prof).read_text(errors="replace").splitlines():
        stack, _, n = line.rpartition(" ")
        if not n.isdigit():
            continue
        n = int(n)
        total += n
        frames = stack.split(";")
        # py-spy はファイル名を絶対パスで書くことがあるので、関数名とファイル名を別々に見る
        if not any(f.startswith("train (") and "ppo_trainer.py" in f for f in frames):
            continue
        in_loop += n
        if any("_rollout_step" in f for f in frames):
            collect += n
        cats[classify(frames, phys)] += n
        for f in set(frames):
            frames_incl[f] += n
    if in_loop == 0:
        raise SystemExit("学習の繰り返しの標本が無い（学習が始まる前に止まった？）")
    print(f"標本 {total}・学習の繰り返しの中 {in_loop}（集める段 {100 * collect / in_loop:.0f}%・学習の段ほか {100 * (in_loop - collect) / in_loop:.0f}%）")
    print("| 分類 | 割合 |\n|---|---|")
    for c, n in cats.most_common():
        print(f"| {c} | {100 * n / in_loop:.1f}% |")
    print(f"\n含む割合の上位 {a.top}（関数・場所）:")
    for f, n in frames_incl.most_common(a.top + 5)[5:]:
        print(f"  {100 * n / in_loop:5.1f}%  {f[:150]}")


if __name__ == "__main__":
    main()
