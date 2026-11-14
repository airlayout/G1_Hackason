"""腰を回して ▲▼ を探す機能（press.waist_scan。J1GEN_WAIST_SCAN=1）を、Yada の評価環境（MuJoCo）で確かめる。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/waist_scan_sim.py --xs 0.32 0.36 0.40 --ys -0.28 -0.24 -0.20 0.20 0.24 0.28 --scan both --workers 4
    $P Button_Press/J1-gen/sim/waist_scan_sim.py --xs 0.40 --ys -0.14 0 0.14 --scan on          # 腰 0° でも見える配置（動かないこと）
    $P Button_Press/J1-gen/sim/waist_scan_sim.py --xs 0.40 --ys -0.16 0.16 --scan both --set realistic

歩かない（approach_sim.py と違い、寄る処理は使わない）。ラボの車いす用の ▲▼（--scene lab）を、体から柱の面まで x [m]・
▲▼ の中点の左右 y [m]（左が +）に置き、contest_agent の J1genElevatorAgent に、評価環境の run_episode（contest/runner.py）で
押させる。Yada のファイルは変えず、シーンの設定（辞書）をメモリの中で書き換えて使う（record_trial_video.py と同じ置き方）。
押す組は wheelchair（J1GEN_BUTTON_SET=wheelchair）。環境変数（J1GEN_WAIST_SCAN・J1GEN_ARM）はエージェントを作る前に決める。

注意:
- 評価環境のロボットは骨盤（pelvis）を固定している。腰をひねったときの足側の反動（実機で体が動くこと）は見えない。
- 評価セット realistic は、立ち位置のずれ（stance_xy / stance_yaw）を 0 にして、指定した配置のままにする（ほかの乱し
  = 深度の雑音・カメラのずれ・揺れなどは入る）。
- 評価環境は点灯したあと post_press_s だけで試行を終える。腰が始めの角度へ戻るのを見るため、終わったあとに同じエージェントを
  最大 tail_s 秒だけ動かし続け、腰の実測を見る（点灯の判定・時間・接触力は run_episode の値で、この続きは含まない）。
- 設定を上書きしたいとき（yaml は書き換えずに試す）は --override press.waist_scan.clear_x_m=0.2 のように渡す（メモリの設定だけ）。

記録（試行ごと）: 点灯・時間・outcome・選ばれた腕・回して見た角度・見つけたときの腰 yaw・腰の指令と実測の最大のずれ・
安全フィルタの丸め・最後の腰の実測・前の物までの距離・接触力。保存: <out>/summary.json と results.csv。
"""

from __future__ import annotations

import argparse
import csv
import dataclasses
import json
import multiprocessing as mp
import os
import re
import sys
import time
from concurrent.futures import ProcessPoolExecutor
from pathlib import Path
from typing import Any

import numpy as np

FEATURE_DIR = Path(__file__).resolve().parents[1]
REPO_ROOT = FEATURE_DIR.parents[1]
YADA_DIR = FEATURE_DIR.parent / "Yada"
OUT_DIR = REPO_ROOT / "_local" / "button_press_yada" / "waist_scan_sim"

WAIST_YAW_ALL = 12  # 全 29 関節の中の腰 yaw の番号（上半身 17 関節の中では 0）
TAIL_S = 20.0  # 点灯のあと、腰が戻るのを見るために続ける最大の時間 [s]


@dataclasses.dataclass(frozen=True)
class Job:
    """1 試行の指定。"""

    x: float  # 柱の面まで [m]
    y: float  # ▲▼ の中点の左右 [m]（左が +）
    target: str  # up / down
    scan: bool  # J1GEN_WAIST_SCAN
    arm: str
    eval_set: str
    seed: int
    overrides: tuple[str, ...] = ()
    tail_s: float = TAIL_S


def make_jobs(xs: list[float], ys: list[float], targets: list[str], scan: str, arm: str, eval_set: str, seed0: int = 0,
              overrides: tuple[str, ...] = (), tail_s: float = TAIL_S) -> list[Job]:
    """配置の組み合わせから試行の一覧を作る。scan は on / off / both（both は同じ配置を off → on の順に両方）。"""
    if scan not in ("on", "off", "both"):
        raise ValueError(f"scan は on / off / both: {scan}")
    scans = {"on": [True], "off": [False], "both": [False, True]}[scan]
    jobs: list[Job] = []
    for x in xs:
        for y in ys:
            for tg in targets:
                for s in scans:
                    jobs.append(Job(float(x), float(y), tg, s, arm, eval_set, seed0 + len(jobs) // len(scans), overrides, tail_s))
    return jobs


def parse_override(text: str) -> tuple[list[str], Any]:
    """"press.waist_scan.clear_x_m=0.2" を (["press", "waist_scan", "clear_x_m"], 0.2) にする（値は JSON として読む）。"""
    if "=" not in text:
        raise ValueError(f"--override は キー=値: {text}")
    key, val = text.split("=", 1)
    try:
        v = json.loads(val)
    except json.JSONDecodeError:
        v = val
    return key.split("."), v


def apply_overrides(cfg: dict, overrides: tuple[str, ...]) -> None:
    for text in overrides:
        path, v = parse_override(text)
        d = cfg
        for k in path[:-1]:
            d = d[k]
        d[path[-1]] = v


def place_wheelchair(scene: dict, x: float, y: float) -> None:
    """record_trial_video.py の _place_lab_wheelchair と同じ置き方（pelvis から前 x・左右 y。低い 2 つを up / down にする）。"""
    pel = float(scene["pelvis_height"])
    scene["wall"]["front_x"] = x + float(scene["column"]["protrusion"])
    scene["column"]["center_y"] = y
    heights = {"up": pel + 0.21, "down": pel + 0.14, "wc_up": pel + 0.45, "wc_down": pel + 0.37}
    for b in scene["buttons"]["items"]:
        b["height"] = heights[b["name"]]
    for dc in scene.get("decals", []):
        if dc["name"] == "wheelchair":
            dc["height"] = heights["up"] + 0.08


class RobotProbe:
    """Robot を包み、安全フィルタを通したあとの腰 yaw の指令（command に渡された値）を記録する。ほかは中身にそのまま任せる。"""

    def __init__(self, robot: Any) -> None:
        self._robot = robot
        self.safe_waist: list[float] = []

    def command(self, q_upper: np.ndarray, base_cmd: np.ndarray) -> None:
        self.safe_waist.append(float(q_upper[0]))
        self._robot.command(q_upper, base_cmd)

    def __getattr__(self, name: str) -> Any:
        return getattr(self._robot, name)


class AgentProbe:
    """Agent を包み、毎周期の腰 yaw（エージェントの指令・実測）と段階を記録する。"""

    def __init__(self, agent: Any) -> None:
        self._agent = agent
        self.cmd: list[float] = []
        self.meas: list[float] = []
        self.phase: list[str] = []
        self.done = False

    def reset(self, task: Any) -> None:
        self._agent.reset(task)

    def act(self, obs: Any) -> Any:
        a = self._agent.act(obs)
        self.meas.append(float(obs.q[WAIST_YAW_ALL]))
        self.cmd.append(float(a.q_target[0]))
        self.phase.append(str(getattr(self._agent.ctl, "phase", "")))
        self.done = bool(a.done)
        return a

    def close(self) -> None:
        self._agent.close()

    def __getattr__(self, name: str) -> Any:
        return getattr(self._agent, name)


def classify_failure(outcome: str, ctl_outcome: str, notes: list[str], ep_outcome: str) -> str:
    """点灯しなかった理由を、エージェントの記録の文から分類する（推測を含む。元の文は outcome に残す）。"""
    if ep_outcome == "success":
        return ""
    text = ctl_outcome + " " + " ".join(notes)
    if "柱に近すぎるので腰をひねって探さない" in text:
        return "前の物までの距離の確認で止まった（近い）"
    if "距離を深度から確かめられない" in text:
        return "前の物までの距離を確かめられず止まった（点が足りない）"
    if "腰を回して探した角度" in text and "見つからない" in text:
        return "全角度で見つからない"
    if "時間内に一般用の ▲▼ を見つけられない" in text:
        return "見つからない（腰を回さなかった）"
    if "計画" in text and ("立たない" in text or "届かない" in text or "Unreachable" in text):
        return "見つけたが計画が立たない"
    if "点灯していない" in text:
        return "押したが点灯しない"
    return "分類できない（outcome を見る）"


def run_job(job: Job) -> dict[str, Any]:
    """1 試行を回して記録を返す（別プロセスで呼ぶ。環境変数はここで決める）。"""
    sys.path.insert(0, str(YADA_DIR))
    sys.path.insert(0, str(FEATURE_DIR / "sim"))
    os.environ["J1GEN_BUTTON_SET"] = "wheelchair"
    os.environ["J1GEN_ARM"] = job.arm
    os.environ["J1GEN_WAIST_SCAN"] = "1" if job.scan else "0"
    from common.config import load_config
    from common.realism import sample_realism  # noqa: F401  （make_trial が使う）
    from contest.interface import UPPER_BODY_IDX, load_agent
    from contest.robots.mujoco_robot import MujocoRobot
    from contest.runner import SafetyFilter, run_episode
    from contest.task import make_trial

    contest_cfg = load_config("contest.yaml")
    trial = make_trial(job.seed, contest_cfg, eval_set=job.eval_set)
    trial = dataclasses.replace(trial, target=job.target, instruction=contest_cfg["instructions"][job.target][0])
    scene = trial.scene_cfg
    place_wheelchair(scene, job.x, job.y)
    realism = trial.realism
    if realism is not None:  # 立ち位置のずれを 0 にして、指定した配置のままにする
        realism = dataclasses.replace(realism, stance_xy=np.zeros(2), stance_yaw=0.0)
    row: dict[str, Any] = {"x": job.x, "y": job.y, "target": job.target, "scan": job.scan, "arm_req": job.arm,
                           "set": job.eval_set, "seed": job.seed}
    t0 = time.time()
    robot = MujocoRobot(scene, contest_cfg, realism=realism)
    agent = None
    try:
        agent = load_agent(str(FEATURE_DIR / "contest_agent"))
        if job.overrides:
            apply_overrides(agent.ctl.cfg, job.overrides)
        ap = AgentProbe(agent)
        rp = RobotProbe(robot)
        ep = run_episode(rp, ap, trial, contest_cfg)
        ctl = agent.ctl
        n_ep = len(ap.cmd)
        # 点灯・終了のあと、腰が戻るのを見るために続ける（エージェントが done を返すまで、最大 tail_s 秒）
        dt = 1.0 / float(contest_cfg["control_hz"])
        lo, hi = robot.joint_limits()
        s = contest_cfg["safety"]
        safety = SafetyFilter(lo, hi, float(s["limit_margin_rad"]), float(s["max_joint_speed_rad_s"]) * dt)
        t = n_ep * dt
        obs = robot.observe(t)
        safety.reset(obs.q[list(UPPER_BODY_IDX)])
        waist_end_ep = float(obs.q[WAIST_YAW_ALL])
        tail_n = 0
        while not ap.done and tail_n * dt < job.tail_s and ep.outcome != "error":
            a = agent.act(obs)
            ap.meas.append(float(obs.q[WAIST_YAW_ALL]))
            ap.cmd.append(float(a.q_target[0]))
            ap.phase.append(str(ctl.phase))
            ap.done = bool(a.done)
            q_safe, _, _ = safety(a.q_target)
            robot.command(q_safe, np.zeros(3))
            robot.advance(dt)
            t += dt
            tail_n += 1
            obs = robot.observe(t)
        waist_final = float(obs.q[WAIST_YAW_ALL])
    finally:
        robot.close()
    notes = [f"{s.phase}: {s.message}" for s in ctl.log]
    meas, cmd = np.array(ap.meas), np.array(ap.cmd)
    safe = np.array(rp.safe_waist)
    n_s = min(len(safe), n_ep)  # 点灯までの間の丸め（続きは SafetyFilter が別なので含めない）
    waist0 = float(meas[0]) if len(meas) else 0.0
    found_yaw = None  # 見つけて計画が立ち、reach に入ったときの腰 yaw の指令
    for i, ph in enumerate(ap.phase):
        if ph in ("reach", "settle", "press"):
            found_yaw = float(np.degrees(cmd[i]))
            break
    front = None
    for n in notes:
        m = re.search(r"前の物まで (\d+) cm", n)
        if m:
            front = float(m.group(1))
    # 腰の指令（エージェント）と実測のずれ。実測は 1 周期遅れるので、指令の 1 周期あとの実測と比べない（単純な同時刻の差）
    dev = np.abs(cmd - meas) if len(cmd) else np.zeros(1)
    row.update({
        "lit": ep.outcome == "success", "ep_outcome": ep.outcome, "stage": ep.stage, "time_s": ep.time_s,
        "ctl_outcome": ctl.outcome, "arm": getattr(ctl, "side", None),
        "scan_tried": list(ctl.scan_tried), "found_waist_deg": found_yaw,
        "waist_cmd_max_deg": float(np.degrees(np.max(np.abs(cmd - cmd[0])))) if len(cmd) else 0.0,
        "waist_dev_max_deg": float(np.degrees(dev.max())),
        "waist_round_max_deg": float(np.degrees(np.max(np.abs(cmd[:n_s] - safe[:n_s])))) if n_s else 0.0,
        "clipped_rate_steps": ep.clipped_rate_steps, "clipped_limit_steps": ep.clipped_limit_steps,
        "waist_end_of_episode_deg": float(np.degrees(waist_end_ep - waist0)),
        "waist_final_deg": float(np.degrees(waist_final - waist0)), "agent_done": ap.done,
        "returned": bool(ap.done and abs(np.degrees(waist_final - waist0)) < 1.0), "tail_steps": tail_n,
        "front_cm": front, "contact_force_n": ep.max_contact_force_n,
        "press_force_n": ep.max_press_force_n, "error": ep.error[-400:] if ep.error else "",
        "reason": classify_failure(ep.outcome, ctl.outcome, notes, ep.outcome), "wall_s": round(time.time() - t0, 1),
        "notes": notes,
    })
    return row


def fmt_table(rows: list[dict[str, Any]]) -> str:
    """結果の表（配置の順）。"""
    head = ["x[cm]", "y[cm]", "target", "scan", "lit", "t[s]", "arm", "tried", "found", "dev", "round", "end", "front", "force", "reason/outcome"]
    out = ["  ".join(f"{h:>7}" for h in head)]
    for r in rows:
        why = r["reason"] or ""
        if not r["lit"]:
            why = (why + " | " + (r["ctl_outcome"] or r["ep_outcome"]))[:110]
        cells = [f"{r['x'] * 100:.0f}", f"{r['y'] * 100:+.0f}", r["target"], "on" if r["scan"] else "off",
                 "OK" if r["lit"] else "NG", "-" if r["time_s"] is None else f"{r['time_s']:.1f}",
                 str(r["arm"] or "-")[:5], ",".join(f"{y:+g}" for y in r["scan_tried"]) or "-",
                 "-" if r["found_waist_deg"] is None else f"{r['found_waist_deg']:+.0f}",
                 f"{r['waist_dev_max_deg']:.1f}", f"{r['waist_round_max_deg']:.1f}",
                 f"{r['waist_final_deg']:+.1f}", "-" if r["front_cm"] is None else f"{r['front_cm']:.0f}",
                 "-" if r["contact_force_n"] is None else f"{r['contact_force_n']:.0f}", why]
        out.append("  ".join(f"{c:>7}" for c in cells[:-1]) + "  " + cells[-1])
    return "\n".join(out)


def summarize(rows: list[dict[str, Any]]) -> dict[str, Any]:
    """off / on ごとの点灯数と、配置ごとの比べ（off で NG・on で OK の数など）。"""
    out: dict[str, Any] = {}
    for key in (False, True):
        sub = [r for r in rows if r["scan"] == key]
        if sub:
            out["on" if key else "off"] = {"trials": len(sub), "lit": sum(r["lit"] for r in sub)}
    pairs: dict[tuple, dict[bool, bool]] = {}
    for r in rows:
        pairs.setdefault((r["x"], r["y"], r["target"]), {})[r["scan"]] = bool(r["lit"])
    both = {k: v for k, v in pairs.items() if len(v) == 2}
    if both:
        out["pairs"] = {"n": len(both), "off_ng_on_ok": sum((not v[False]) and v[True] for v in both.values()),
                        "off_ok_on_ng": sum(v[False] and not v[True] for v in both.values()),
                        "both_ok": sum(v[False] and v[True] for v in both.values()),
                        "both_ng": sum((not v[False]) and (not v[True]) for v in both.values())}
    fails: dict[str, int] = {}
    for r in rows:
        if r["scan"] and not r["lit"]:
            fails[r["reason"]] = fails.get(r["reason"], 0) + 1
    out["on_failure_reasons"] = fails
    out["waist_dev_max_deg"] = max((r["waist_dev_max_deg"] for r in rows if r["scan"]), default=None)
    out["waist_round_max_deg"] = max((r["waist_round_max_deg"] for r in rows if r["scan"]), default=None)
    out["contact_force_max_n"] = max((r["contact_force_n"] or 0.0 for r in rows), default=None)
    return out


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xs", type=float, nargs="+", default=[0.36], help="柱の面まで [m]")
    ap.add_argument("--ys", type=float, nargs="+", default=[0.24], help="▲▼ の中点の左右 [m]（左が +）")
    ap.add_argument("--targets", nargs="+", choices=["up", "down"], default=["up", "down"])
    ap.add_argument("--scan", choices=["on", "off", "both"], default="both", help="J1GEN_WAIST_SCAN（both は同じ配置を両方で回す）")
    ap.add_argument("--arm", default="auto", choices=["auto", "right", "left"], help="J1GEN_ARM")
    ap.add_argument("--set", default="basic", choices=["basic", "realistic"], help="評価セット（乱しの有無）")
    ap.add_argument("--scene", default="lab", choices=["lab"], help="置くボタン（今はラボの車いす用だけ）")
    ap.add_argument("--seed0", type=int, default=0)
    ap.add_argument("--override", action="append", default=[], help="メモリの設定の上書き（例 press.waist_scan.clear_x_m=0.2）")
    ap.add_argument("--tail-s", type=float, default=TAIL_S, help="点灯のあと、腰が戻るのを見るために続ける最大の時間 [s]")
    ap.add_argument("--out", default="")
    ap.add_argument("--workers", type=int, default=1)
    args = ap.parse_args(argv)

    if args.workers > 1:  # 並列のときは 1 試行ごとの BLAS のスレッドを 1 にする（取り合って遅くなるため）
        for k in ("OMP_NUM_THREADS", "OPENBLAS_NUM_THREADS", "MKL_NUM_THREADS"):
            os.environ.setdefault(k, "1")
    for o in args.override:
        parse_override(o)  # 形の確認だけ
    jobs = make_jobs(args.xs, args.ys, args.targets, args.scan, args.arm, args.set, args.seed0, tuple(args.override), args.tail_s)
    out = Path(args.out) if args.out else OUT_DIR / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)
    print(f"[waist_scan_sim] {len(jobs)} 試行（{args.workers} 並列）。保存先: {out}")
    rows: list[dict[str, Any]] = []
    t0 = time.time()
    if args.workers <= 1:
        for j in jobs:
            rows.append(_safe_run(j))
            _progress(rows[-1], len(rows), len(jobs))
    else:
        with ProcessPoolExecutor(max_workers=args.workers, mp_context=mp.get_context("spawn")) as ex:
            for r in ex.map(_safe_run, jobs):
                rows.append(r)
                _progress(r, len(rows), len(jobs))
    summary = summarize(rows)
    print(fmt_table(rows))
    print(f"[waist_scan_sim] まとめ: {json.dumps(summary, ensure_ascii=False)}（{time.time() - t0:.0f} 秒）")
    (out / "summary.json").write_text(json.dumps({"args": vars(args), "summary": summary, "trials": rows}, indent=2,
                                                 ensure_ascii=False, default=float))
    cols = [k for k in rows[0] if k != "notes"] if rows else []
    with open(out / "results.csv", "w", newline="", encoding="utf-8") as f:
        w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"[waist_scan_sim] 保存: {out}")
    return 0


def _safe_run(job: Job) -> dict[str, Any]:
    """1 回の例外で全体を止めない（記録して次へ）。"""
    try:
        return run_job(job)
    except Exception:
        import traceback

        tb = traceback.format_exc()
        print(tb)
        return {"x": job.x, "y": job.y, "target": job.target, "scan": job.scan, "arm_req": job.arm, "set": job.eval_set,
                "seed": job.seed, "lit": False, "ep_outcome": "script_error", "stage": "", "time_s": None,
                "ctl_outcome": "", "arm": None, "scan_tried": [], "found_waist_deg": None, "waist_cmd_max_deg": 0.0,
                "waist_dev_max_deg": 0.0, "waist_round_max_deg": 0.0, "clipped_rate_steps": 0, "clipped_limit_steps": 0,
                "waist_end_of_episode_deg": 0.0, "waist_final_deg": 0.0, "agent_done": False, "returned": False,
                "tail_steps": 0, "front_cm": None, "contact_force_n": None, "press_force_n": None,
                "error": tb.strip().splitlines()[-1], "reason": "スクリプトの例外", "wall_s": 0.0, "notes": []}


def _progress(r: dict[str, Any], i: int, n: int) -> None:
    print(f"[waist_scan_sim] ({i}/{n}) x={r['x'] * 100:.0f} y={r['y'] * 100:+.0f} {r['target']} scan={'on' if r['scan'] else 'off'}"
          f" → {'点灯' if r['lit'] else '失敗'}（{r['ep_outcome']}、腕 {r['arm']}、回した {r['scan_tried']}）", flush=True)


if __name__ == "__main__":
    sys.exit(main())
