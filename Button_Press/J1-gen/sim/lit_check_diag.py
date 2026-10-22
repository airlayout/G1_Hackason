"""点灯の確認（common/elevator_press.py の _check）を、評価環境で確かめる診断。

評価環境はボタンが点灯した瞬間に試行を終えるので、普段はエージェントの点灯の確認まで進まない。ここでは点灯を
runner に見せず（MuJoCo の点灯の状態は記録だけする）、エージェントが確認して腕を戻し終わるまで動かす。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/lit_check_diag.py --set realistic --detector yolo --seeds 0-19
    $P Button_Press/J1-gen/sim/lit_check_diag.py --set basic --detector classic --seeds 0 1 2

1 試行 1 行（LITDIAG）: 評価セット、検出器、種、押すボタン、エージェントの結論、確認したときの本当の点灯と
明るさの上がり方（押したボタン・もう一方。それぞれ 枠全体の平均, 明るい部分）、エージェントの表示、
壁・柱との接触力の最大（点灯のあとの保持と戻りも含む。評価環境は点灯で終わるので、評価の結果には出ない）。
最後に「本当に点灯していたのに点灯していないとした」「点灯していないのに点灯したとした」の数を出す。
並列で動かすときは、メモリー 15 GB のノート PC で 4 つまで（8 つで残り 1 GB になった）。
"""

from __future__ import annotations

import argparse
import io
import os
import sys
from contextlib import redirect_stdout
from pathlib import Path

ROOT = Path(__file__).resolve().parents[3]
sys.path.insert(0, str(ROOT / "Button_Press" / "Yada"))


def parse_seeds(items: list[str]) -> list[int]:
    out: list[int] = []
    for s in items:
        if "-" in s:
            a, b = s.split("-")
            out += list(range(int(a), int(b) + 1))
        else:
            out.append(int(s))
    return out


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--set", default="realistic", help="評価セット（basic / realistic）")
    ap.add_argument("--detector", default="yolo", choices=["classic", "yolo"])
    ap.add_argument("--seeds", nargs="+", default=["0-19"], help="種（0-19 のような範囲も書ける）")
    a = ap.parse_args()
    os.environ["J1GEN_BUTTON_DETECTOR"] = a.detector

    import numpy as np

    from common.config import load_config
    from contest.interface import load_agent
    from contest.robots.mujoco_robot import MujocoRobot
    from contest.runner import run_episode
    from contest.task import make_trial

    cfg = load_config("contest.yaml")
    agent = load_agent(str(ROOT / "Button_Press" / "J1-gen" / "contest_agent"))
    ctl = agent.ctl
    state: dict = {}

    orig_adv = MujocoRobot.advance

    def advance(self, dt):  # type: ignore[no-untyped-def]
        info = orig_adv(self, dt)
        state["lit"] = dict(info.lit)
        info.lit = {}  # runner に点灯を見せない（試行を終わらせない）
        return info

    def feats(rgb, bbox):  # type: ignore[no-untyped-def]
        return np.array([ctl._brightness(rgb, bbox), ctl._lit_level(rgb, bbox)])

    orig_look, orig_check = type(ctl)._look, type(ctl)._check

    def look(self, rgb, depth, K, q, t):  # type: ignore[no-untyped-def]
        orig_look(self, rgb, depth, K, q, t)
        if self.call is not None and "base_t" not in state:
            other = "up" if self.target == "down" else "down"
            state["base_t"] = feats(rgb, self.call.button(self.target).bbox)
            state["base_o"] = feats(rgb, self.call.button(other).bbox)

    def check(self, rgb, t):  # type: ignore[no-untyped-def]
        other = "up" if self.target == "down" else "down"
        ft = feats(rgb, self._current_bbox(rgb, self.call.button(self.target).bbox)) - state["base_t"]
        fo = feats(rgb, self._current_bbox(rgb, self.call.button(other).bbox)) - state["base_o"]
        before = self.outcome
        ok = orig_check(self, rgb, t)
        said_lit = self.outcome == "点灯した" and before != "点灯した"
        state.setdefault("checks", []).append((self.target in [k for k, v in state.get("lit", {}).items() if v],
                                               said_lit, np.round(ft).tolist(), np.round(fo).tolist()))
        return ok

    MujocoRobot.advance = advance
    type(ctl)._look, type(ctl)._check = look, check

    missed = false_lit = n_checks = 0
    forces: list[float] = []  # 試行ごとの壁・柱との接触力の最大（点灯のあとの保持・戻りも含む）
    for seed in parse_seeds(a.seeds):
        trial = make_trial(seed, cfg, eval_set=a.set)
        state.clear()
        robot = MujocoRobot(trial.scene_cfg, cfg, viewer=False, realism=trial.realism)
        try:
            with redirect_stdout(io.StringIO()):
                r = run_episode(robot, agent, trial, cfg)
        finally:
            robot.close()
        checks = state.get("checks", [])
        for truly, said, _, _ in checks:
            n_checks += 1
            missed += int(truly and not said)
            false_lit += int(said and not truly)
        notes = [s.message for s in ctl.log if "点灯" in s.message]
        shown = [f"{'点' if c[0] else '消'}→{'点' if c[1] else '消'} 押{c[2]} 他{c[3]}" for c in checks]
        forces.append(r.max_contact_force_n or 0.0)
        print(f"LITDIAG\t{a.set}\t{a.detector}\t{seed}\t{trial.target}\t{ctl.outcome}\t{' | '.join(shown)}\t"
              f"{' / '.join(notes)}\t接触 {r.max_contact_force_n} N", flush=True)
    print(f"[lit_check_diag] 確認 {n_checks} 回: 点灯していたのに「点灯していない」{missed} 回、"
          f"点灯していないのに「点灯した」{false_lit} 回。壁・柱との接触力の最大 {max(forces, default=0.0):.1f} N", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
