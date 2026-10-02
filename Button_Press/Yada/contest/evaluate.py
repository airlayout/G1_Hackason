"""エージェントを評価して採点する。

    P=~/miniconda3/envs/lerobot/bin/python
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/Yada/contest/example_agent --seeds smoke
    $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名>                  # 試行回数は PC の設定
    $P Button_Press/Yada/contest/evaluate.py --agent ... --trials 100 --set realistic --report
    $P Button_Press/Yada/contest/evaluate.py --agent ... --seed 3 --view     # 1 試行を画面で見る
    bash Button_Press/Yada/contest/evaluate_isaac.sh --agent ... --seeds smoke  # Isaac Sim（evaluate_isaac.py）

試行の決め方（上ほど優先）: --seed（種を直接）> --trials N（種 0〜N−1）> --seeds（seeds.yaml の組）>
この PC の設定の試行回数（初めての PC では、性能を調べて見積もりの時間を示し、試行回数を聞く。contest/machine.py）。
練習用の種（practice）は 0〜19 なので、--trials 20 と同じ。

MuJoCo は、試行を並列のプロセスで回す（並列数は PC の性能から決める。--workers で変えられる。--view のときは 1）。
同じ種なら、並列でも 1 つずつでも結果は同じ。

結果は表で表示し、JSON を _local/button_press_yada/results/ に保存する。
"""

from __future__ import annotations

import argparse
import dataclasses
import json
import sys
import time
from collections.abc import Callable
from pathlib import Path
from typing import Any

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT, load_config  # noqa: E402
from contest.interface import load_agent  # noqa: E402
from contest.runner import EpisodeResult, run_episode  # noqa: E402
from contest.task import make_trial  # noqa: E402

RESULTS_DIR = REPO_ROOT / "_local" / "button_press_yada" / "results"


def make_mujoco_robot(trial, contest_cfg: dict, args: argparse.Namespace):
    from contest.robots.mujoco_robot import MujocoRobot

    return MujocoRobot(trial.scene_cfg, contest_cfg, viewer=getattr(args, "view", False), realism=trial.realism)


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
        "max_contact_force_n": max((r.max_contact_force_n for r in results if r.max_contact_force_n is not None),
                                   default=None),
    }


def add_eval_args(ap: argparse.ArgumentParser) -> None:
    ap.add_argument("--agent", required=True, help="agent.py か、それがあるフォルダ")
    ap.add_argument("--set", default="basic", help="評価セット（configs/contest.yaml の sets: basic / realistic）")
    ap.add_argument("--seeds", default=None, help="seeds.yaml の組の名前（practice / smoke）")
    ap.add_argument("--seeds-file", default=str(Path(__file__).with_name("seeds.yaml")))
    ap.add_argument("--seed", type=int, action="append", help="種を直接指定する（複数可。いちばん優先）")
    ap.add_argument("--trials", type=int, default=0, help="試行回数（種 0〜N−1）。省略するとこの PC の設定")
    ap.add_argument("--out", default="", help="結果の JSON の保存先（既定は _local/button_press_yada/results/）")
    ap.add_argument("--ablation", action="store_true",
                    help="乱しを 1 種類ずつ入れて評価する（乱しなし・各種類だけ・すべて。評価セットは realistic）")
    ap.add_argument("--ablation-only", default="", help="この種類の乱しだけを入れて評価する（例: camera_mount）")
    ap.add_argument("--report", action="store_true", help="評価のあとに弱点のレポート（contest/report.py）を作る")
    ap.add_argument("--reset-machine", action="store_true", help="この PC の設定（並列数・試行回数）を調べ直す")


def resolve_seeds(args: argparse.Namespace, sim: str, multiplier: int) -> tuple[list[int], dict[str, Any]]:
    """試行の種と、この PC の設定を決める（新しい PC なら試行回数を聞く）。"""
    from contest.machine import ensure_profile

    explicit = bool(args.seed or args.trials or args.seeds)
    profile = ensure_profile(sim, ask=not explicit, reset=args.reset_machine, multiplier=multiplier)
    if args.seed:
        seeds = list(args.seed)
    elif args.trials:
        seeds = list(range(int(args.trials)))
    elif args.seeds:
        seeds = list(load_config(args.seeds_file)[args.seeds])
    else:
        seeds = list(range(int(profile["default_trials"])))
    return seeds, profile


# ---- 並列のプロセス（MuJoCo） ----------------------------------------------------

_W: dict[str, Any] = {}


def _worker_init(agent_path: str) -> None:
    """並列のプロセスごとに 1 回: 設定とエージェントを読み込む（エージェントは試行ごとに reset される）。"""
    _W["cfg"] = load_config("contest.yaml")
    _W["agent"] = load_agent(agent_path)


def _worker_run(task: tuple[str | None, int, str]) -> dict[str, Any]:
    import io
    from contextlib import redirect_stdout

    variant, seed, eval_set = task
    r, wall = _run_one(variant, seed, eval_set, _W["cfg"], _W["agent"], make_mujoco_robot, argparse.Namespace(),
                       quiet=True, buf=io.StringIO(), redirect=redirect_stdout)
    d = r.to_dict()
    d["_wall_s"] = wall
    return d


def _run_one(variant: str | None, seed: int, eval_set: str, cfg: dict, agent: Any, make_robot: Callable,
             args: argparse.Namespace, quiet: bool = False, buf: Any = None, redirect: Any = None
             ) -> tuple[EpisodeResult, float]:
    from common.realism import only

    trial = make_trial(seed, cfg, eval_set=eval_set)
    if variant is not None:
        trial = dataclasses.replace(trial, realism=only(trial.realism, variant))
    t0 = time.time()
    robot = make_robot(trial, cfg, args)
    try:
        if quiet and redirect is not None:
            with redirect(buf):  # 並列のときは、エージェントの表示を混ぜない
                r = run_episode(robot, agent, trial, cfg)
        else:
            r = run_episode(robot, agent, trial, cfg, verbose=True)
    finally:
        robot.close()
    if variant is not None:
        r.extra["ablation"] = variant
    return r, time.time() - t0


def _print_result(r: EpisodeResult, wall: float) -> None:
    force = "-" if r.max_contact_force_n is None else r.max_contact_force_n
    tag = f"[{r.extra['ablation']}] " if r.extra.get("ablation") else ""
    print(f"[eval] {tag}seed {r.seed:>3}  {r.target:<4}  {r.outcome:<8} {r.stage:<20} "
          f"時間 {r.time_s if r.time_s is not None else '-':>6}  接触力 {force:>6} N  "
          f"丸め {r.clipped_limit_steps}/{r.clipped_rate_steps}  （実時間 {wall:.1f} s）")
    if r.error:
        print(r.error)


def run(args: argparse.Namespace, sim: str, make_robot: Callable, parallel: bool = False) -> dict:
    """エージェントを評価する。make_robot(trial, contest_cfg, args) が試行ごとのロボットを作る。

    parallel=True（MuJoCo）なら、試行を並列のプロセスで回す（Isaac Sim は 1 つのアプリなので False）。
    """
    from common.realism import ABLATION_GROUPS, GROUP_LABELS

    contest_cfg = load_config("contest.yaml")
    if args.ablation:
        variants: list[str | None] = ["none", *ABLATION_GROUPS, "all"]
    elif args.ablation_only:
        variants = [args.ablation_only]
    else:
        variants = [None]
    eval_set = "realistic" if variants != [None] else args.set
    if eval_set != args.set:
        print(f"[eval] 乱しを入れる評価なので、評価セットを {eval_set} にする")
        args.set = eval_set
    seeds, profile = resolve_seeds(args, sim, len(variants))
    tasks = [(v, s, eval_set) for v in variants for s in seeds]
    workers = 1
    if parallel and not getattr(args, "view", False):
        workers = int(args.workers) if getattr(args, "workers", 0) else int(profile["workers"])
        workers = max(1, min(workers, len(tasks)))
    from contest.machine import estimate_minutes, format_minutes

    est = format_minutes(estimate_minutes({**profile, "workers": workers}, len(tasks), sim))
    print(f"[eval] エージェント: {args.agent}、シミュレーター: {sim}、評価セット: {eval_set}、"
          f"試行: {len(seeds)}" + (f" × 乱し {len(variants)} 通り = {len(tasks)}" if len(variants) > 1 else "")
          + f"、並列数: {workers}、見積もり {est}")

    t_start = time.time()
    results: list[EpisodeResult] = []
    if workers > 1:
        import multiprocessing as mp

        # spawn: 描画（OpenGL）と MuJoCo を、プロセスごとに作り直す（fork だと描画の状態を引き継いで壊れる）
        ctx = mp.get_context("spawn")
        order = {t[:2]: i for i, t in enumerate(tasks)}
        done = 0
        with ctx.Pool(workers, initializer=_worker_init, initargs=(str(args.agent),)) as pool:
            for d in pool.imap_unordered(_worker_run, tasks):
                wall = d.pop("_wall_s")
                r = EpisodeResult(**d)
                results.append(r)
                done += 1
                _print_result(r, wall)
                if done % max(1, len(tasks) // 10) == 0 or done == len(tasks):
                    el = time.time() - t_start
                    print(f"[eval] 進み具合 {done}/{len(tasks)}（経過 {el:.0f} s、残り約 {el / done * (len(tasks) - done):.0f} s）")
        results.sort(key=lambda r: order[(r.extra.get("ablation"), r.seed)])
    else:
        agent = load_agent(args.agent)
        try:
            for v, s, es in tasks:
                if v is not None and s == seeds[0]:
                    print(f"[eval] ---- 乱し: {GROUP_LABELS.get(v, v)}（{v}）----")
                r, wall = _run_one(v, s, es, contest_cfg, agent, make_robot, args)
                results.append(r)
                _print_result(r, wall)
        finally:
            agent.close()

    if len(variants) > 1 or variants[0] is not None:
        for v in variants:
            vr = [x for x in results if x.extra.get("ablation") == v]
            if vr:
                print(f"[eval] {GROUP_LABELS.get(v, v)}: 成功率 {100 * sum(x.outcome == 'success' for x in vr) / len(vr):.0f}%")
    main_results = [r for r in results if r.extra.get("ablation") in (None, "all")]
    summary = summarize(main_results or results)
    summary["wall_time_s"] = round(time.time() - t_start, 1)
    summary["workers"] = workers
    from contest.machine import record_run

    record_run(sim, len(tasks), workers, time.time() - t_start)
    print(f"[eval] 成功率 {summary['success_rate'] * 100:.0f}%（{summary['outcomes']}）、"
          f"成功時の平均時間 {summary['mean_time_s_success']} s（実時間 {summary['wall_time_s']} s、並列数 {workers}）")
    out = Path(args.out) if args.out else RESULTS_DIR / (
        f"{Path(args.agent).resolve().stem}_{sim}_{args.set}{'_ablation' if args.ablation else ''}"
        f"_{time.strftime('%Y%m%d_%H%M%S')}.json")
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"agent": str(args.agent), "sim": sim, "eval_set": args.set,
                               "ablation": bool(args.ablation), "summary": summary,
                               "results": [r.to_dict() for r in results]}, ensure_ascii=False, indent=2))
    print(f"[eval] 保存した: {out}")
    if getattr(args, "report", False):
        from contest.report import REPORT_DIR, build_report, load_results

        res, meta = load_results([out])
        md_path = REPORT_DIR / f"{out.stem}.md"
        md_path.parent.mkdir(parents=True, exist_ok=True)
        md_path.write_text(build_report(res, meta), encoding="utf-8")
        print(f"[eval] 弱点のレポート: {md_path}")
    return summary


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_eval_args(ap)
    ap.add_argument("--view", action="store_true", help="画面で見る（実時間で進む。並列にしない）")
    ap.add_argument("--workers", type=int, default=0, help="並列数（省略するとこの PC の性能から決める）")
    args = ap.parse_args()
    run(args, "mujoco", make_mujoco_robot, parallel=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
