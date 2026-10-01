"""エージェントを評価して採点する。

    P=~/miniconda3/envs/lerobot/bin/python
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/Yada/contest/example_agent --seeds smoke
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --seeds practice
    $P Button_Press/Yada/contest/evaluate.py --agent ... --seed 3 --view     # 1 試行を画面で見る

結果は表で表示し、JSON を _local/button_press_yada/results/ に保存する。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT, load_config  # noqa: E402
from contest.interface import load_agent  # noqa: E402
from contest.runner import EpisodeResult, run_episode  # noqa: E402
from contest.task import make_trial  # noqa: E402

RESULTS_DIR = REPO_ROOT / "_local" / "button_press_yada" / "results"


def make_robot(sim: str, trial, contest_cfg: dict, view: bool):
    if sim == "mujoco":
        from contest.robots.mujoco_robot import MujocoRobot

        return MujocoRobot(trial.scene_cfg, contest_cfg, viewer=view)
    raise ValueError(f"このシミュレーターはまだ無い: {sim}（今は mujoco だけ）")


def summarize(results: list[EpisodeResult]) -> dict:
    n = len(results)
    ok = [r for r in results if r.outcome == "success"]
    counts: dict[str, int] = {}
    for r in results:
        counts[r.outcome] = counts.get(r.outcome, 0) + 1
    return {
        "trials": n,
        "success_rate": round(len(ok) / n, 3) if n else 0.0,
        "outcomes": counts,
        "mean_time_s_success": round(sum(r.time_s for r in ok) / len(ok), 2) if ok else None,
        "max_contact_force_n": max((r.max_contact_force_n or 0.0) for r in results) if results else None,
    }


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--agent", required=True, help="agent.py か、それがあるフォルダ")
    ap.add_argument("--sim", default="mujoco", choices=["mujoco"], help="シミュレーター")
    ap.add_argument("--seeds", default="practice", help="seeds.yaml の組の名前")
    ap.add_argument("--seeds-file", default=str(Path(__file__).with_name("seeds.yaml")))
    ap.add_argument("--seed", type=int, action="append", help="種を直接指定する（複数可。--seeds より優先）")
    ap.add_argument("--view", action="store_true", help="画面で見る（実時間で進む）")
    ap.add_argument("--out", default="", help="結果の JSON の保存先（既定は _local/button_press_yada/results/）")
    args = ap.parse_args()

    contest_cfg = load_config("contest.yaml")
    seeds = args.seed if args.seed else load_config(args.seeds_file)[args.seeds]
    agent = load_agent(args.agent)
    print(f"[eval] エージェント: {args.agent}、シミュレーター: {args.sim}、試行: {len(seeds)}")

    results: list[EpisodeResult] = []
    try:
        for seed in seeds:
            trial = make_trial(seed, contest_cfg)
            robot = make_robot(args.sim, trial, contest_cfg, args.view)
            try:
                t0 = time.time()
                r = run_episode(robot, agent, trial, contest_cfg, verbose=True)
            finally:
                robot.close()
            results.append(r)
            print(f"[eval] seed {seed:>3}  {r.target:<4}  {r.outcome:<8}  "
                  f"時間 {r.time_s if r.time_s is not None else '-':>6}  接触力 {r.max_contact_force_n:>6} N  "
                  f"丸め {r.clipped_limit_steps}/{r.clipped_rate_steps}  （実時間 {time.time() - t0:.1f} s）")
            if r.error:
                print(r.error)
    finally:
        agent.close()

    summary = summarize(results)
    print(f"[eval] 成功率 {summary['success_rate'] * 100:.0f}%（{summary['outcomes']}）、"
          f"成功時の平均時間 {summary['mean_time_s_success']} s")
    out = Path(args.out) if args.out else RESULTS_DIR / f"{Path(args.agent).resolve().stem}_{args.sim}_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"agent": str(args.agent), "sim": args.sim, "summary": summary,
                               "results": [r.to_dict() for r in results]}, ensure_ascii=False, indent=2))
    print(f"[eval] 保存した: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
