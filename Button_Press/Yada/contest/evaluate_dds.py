"""実機と同じ口（DDS + ZMQ）で評価する。試行ごとに模擬 G1 を起動し、チームのコマンドを動かし、判定を集める。

    P=~/miniconda3/envs/lerobot/bin/python
    # 見本のエージェントを、実機用の経路（contest/run_dds.py）で動かす
    $P Button_Press/Yada/contest/evaluate_dds.py --seeds smoke \\
        --client "$P Button_Press/Yada/contest/run_dds.py --agent Button_Press/Yada/contest/example_agent"
    # 自分の実機用のプログラム（どんな作りでもよい）
    $P Button_Press/Yada/contest/evaluate_dds.py --seeds practice --client "<自分のコマンド>"

チームのコマンドへの渡し方:
- 指示は、模擬 G1 が書く task.json（パスは環境変数 YADA_TASK_FILE）。seed・target・instruction・DDS の口が入っている
- DDS の口は lo（domain 1）、カメラは 127.0.0.1:5556（深度付き）/ 5555（RGB 互換）
- コマンドは試行ごとに新しく起動する。判定が済んで模擬 G1 が止まったあと、まだ動いていれば止める

判定（success / wrong / timeout / no_command）は模擬 G1 が行う（sim/mujoco/g1_sim_server.py）。
結果は表で表示し、JSON を _local/button_press_yada/results/ に保存する。
"""

from __future__ import annotations

import argparse
import json
import os
import shlex
import signal
import subprocess
import sys
import time
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import FEATURE_DIR, REPO_ROOT, load_config  # noqa: E402

SERVER = FEATURE_DIR / "sim" / "mujoco" / "g1_sim_server.py"
SERVER_DIR = REPO_ROOT / "_local" / "button_press_yada" / "sim_server"
RESULTS_DIR = REPO_ROOT / "_local" / "button_press_yada" / "results"
LOG_DIR = REPO_ROOT / "_local" / "button_press_yada" / "logs" / "evaluate_dds"
READY_TIMEOUT_S = 60.0


def _stop(proc: subprocess.Popen, name: str) -> None:
    if proc.poll() is not None:
        return
    proc.send_signal(signal.SIGINT)
    try:
        proc.wait(timeout=10)
    except subprocess.TimeoutExpired:
        print(f"[eval_dds] {name} が止まらないので強制終了する")
        proc.kill()
        proc.wait()


def run_trial(seed: int, client: str, server_python: str, view: bool, time_limit: float) -> dict:
    """1 試行: 模擬 G1 を起動 → 準備できたらチームのコマンドを起動 → 模擬 G1 の判定を待つ。"""
    LOG_DIR.mkdir(parents=True, exist_ok=True)
    task_file = SERVER_DIR / "task.json"
    result_file = SERVER_DIR / f"result_seed{seed}.json"
    for f in (task_file, result_file):
        f.unlink(missing_ok=True)
    cmd = [server_python, str(SERVER), "--seed", str(seed)] + (["--view"] if view else [])
    s_log = open(LOG_DIR / f"server_seed{seed}.log", "w")
    c_log = open(LOG_DIR / f"client_seed{seed}.log", "w")
    server = subprocess.Popen(cmd, stdout=s_log, stderr=subprocess.STDOUT, cwd=REPO_ROOT)
    client_proc = None
    try:
        t0 = time.monotonic()
        while not task_file.exists():
            if server.poll() is not None:
                return {"seed": seed, "outcome": "server_error", "error": f"模擬 G1 が起動しなかった（{s_log.name}）"}
            if time.monotonic() - t0 > READY_TIMEOUT_S:
                return {"seed": seed, "outcome": "server_error", "error": "模擬 G1 の準備が終わらない"}
            time.sleep(0.2)
        env = {**os.environ, "YADA_TASK_FILE": str(task_file)}
        client_proc = subprocess.Popen(shlex.split(client), stdout=c_log, stderr=subprocess.STDOUT, cwd=REPO_ROOT,
                                       env=env)
        # 模擬 G1 は、判定のあと少しして自分で止まる。止まらなければ、制限時間 + 余裕で止める
        try:
            server.wait(timeout=time_limit + 120.0)
        except subprocess.TimeoutExpired:
            _stop(server, "模擬 G1")
    finally:
        if client_proc is not None:
            _stop(client_proc, "チームのコマンド")
        _stop(server, "模擬 G1")
        s_log.close()
        c_log.close()
    if not result_file.exists():
        return {"seed": seed, "outcome": "server_error", "error": f"判定の結果が無い（{s_log.name}）"}
    res = json.loads(result_file.read_text())
    res["client_exit_code"] = client_proc.returncode if client_proc is not None else None
    res["logs"] = {"server": s_log.name, "client": c_log.name}
    return res


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--client", required=True, help="試行ごとに起動するチームのコマンド（1 行の文字列）")
    ap.add_argument("--seeds", default="practice", help="seeds.yaml の組の名前")
    ap.add_argument("--seeds-file", default=str(Path(__file__).with_name("seeds.yaml")))
    ap.add_argument("--seed", type=int, action="append", help="種を直接指定する（複数可。--seeds より優先）")
    ap.add_argument("--server-python", default=sys.executable, help="模擬 G1 を動かす Python（既定はこの Python）")
    ap.add_argument("--view", action="store_true", help="模擬 G1 の画面を開く")
    ap.add_argument("--out", default="")
    args = ap.parse_args()

    seeds = args.seed if args.seed else load_config(args.seeds_file)[args.seeds]
    time_limit = float(load_config("contest.yaml")["time_limit_s"])
    print(f"[eval_dds] コマンド: {args.client}")
    print(f"[eval_dds] 試行: {len(seeds)}（ログは {LOG_DIR}）")
    results = []
    for seed in seeds:
        t0 = time.time()
        r = run_trial(seed, args.client, args.server_python, args.view, time_limit)
        results.append(r)
        print(f"[eval_dds] seed {seed:>3}  {r.get('target', '-'):<4}  {r['outcome']:<12}  時間 {r.get('time_s') or '-':>6}  "
              f"接触力 {r.get('max_contact_force_n', '-'):>6} N  （実時間 {time.time() - t0:.1f} s）"
              + (f"  {r['error']}" if r.get("error") else ""))
    ok = [r for r in results if r["outcome"] == "success"]
    counts: dict[str, int] = {}
    for r in results:
        counts[r["outcome"]] = counts.get(r["outcome"], 0) + 1
    summary = {"trials": len(results), "success_rate": round(len(ok) / len(results), 3) if results else 0.0,
               "outcomes": counts,
               "mean_time_s_success": round(sum(r["time_s"] for r in ok) / len(ok), 2) if ok else None}
    print(f"[eval_dds] 成功率 {summary['success_rate'] * 100:.0f}%（{counts}）、成功時の平均時間 {summary['mean_time_s_success']} s")
    out = Path(args.out) if args.out else RESULTS_DIR / f"dds_{time.strftime('%Y%m%d_%H%M%S')}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps({"client": args.client, "sim": "mujoco_dds", "summary": summary, "results": results},
                              ensure_ascii=False, indent=2))
    print(f"[eval_dds] 保存した: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
