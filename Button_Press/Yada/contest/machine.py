"""評価を回す PC の性能を調べ、並列数を決める。新しい PC では、見積もりの時間を示して試行回数を聞く。

PC ごとの設定（並列数、1 試行の時間の見積もり、試行回数の既定）は _local/button_press_yada/machine_profiles.json に
保存する（git の対象外）。2 回目からは聞かない。設定をやり直すときは evaluate.py --reset-machine を付ける。

並列数の決め方（MuJoCo。1 試行を 1 つのプロセスで動かす）:
- CPU: 使えるコアの数 − 1（画面やほかの処理のために 1 つ空ける）
- メモリ: 空いているメモリの 7 割 ÷ 1 プロセスのメモリ（実測で約 0.66 GB。余裕を見て 0.8 GB とする）
- このどちらか小さいほう。試行の数より多くはしない
"""

from __future__ import annotations

import json
import os
import platform
import socket
import sys
import time
from pathlib import Path
from typing import Any

from common.config import REPO_ROOT

PROFILE_FILE = REPO_ROOT / "_local" / "button_press_yada" / "machine_profiles.json"
# 1 プロセスのメモリ [GB]（2026-10-02 に、見本のエージェント + MuJoCo のロボット 1 つで約 0.66 GB を実測。余裕を見る）
MEM_PER_WORKER_GB = 0.8
MEM_FRACTION = 0.7
RESERVE_CORES = 1
DEFAULT_TRIALS = 20
# 1 試行のシミュレーションの時間の目安 [秒]（成功すると約 6 秒。失敗して制限時間まで続くと 30 秒）
SIM_SECONDS_PER_TRIAL = 8.0
# Isaac Sim の 1 試行の実時間（NVIDIA L40S で約 37 秒を実測。並列にしていない）
ISAAC_SECONDS_PER_TRIAL = 37.0


def _meminfo_gb() -> tuple[float, float]:
    """(全体, 空き) [GB]。Linux の /proc/meminfo から。読めなければ (0, 0)。"""
    try:
        info = {}
        for line in Path("/proc/meminfo").read_text().splitlines():
            k, v = line.split(":", 1)
            info[k] = float(v.split()[0]) / 1024 / 1024
        return info["MemTotal"], info.get("MemAvailable", info["MemFree"])
    except Exception:
        return 0.0, 0.0


def _cpu_model() -> str:
    try:
        for line in Path("/proc/cpuinfo").read_text().splitlines():
            if line.startswith("model name"):
                return line.split(":", 1)[1].strip()
    except Exception:
        pass
    return platform.processor() or "不明"


def _gpus() -> list[str]:
    import shutil
    import subprocess

    if not shutil.which("nvidia-smi"):
        return []
    try:
        out = subprocess.run(["nvidia-smi", "--query-gpu=name,memory.total", "--format=csv,noheader"],
                             capture_output=True, text=True, timeout=10).stdout
        return [s.strip() for s in out.splitlines() if s.strip()]
    except Exception:
        return []


def detect_spec() -> dict[str, Any]:
    try:
        cores = len(os.sched_getaffinity(0))
    except AttributeError:
        cores = os.cpu_count() or 1
    total, avail = _meminfo_gb()
    return {"host": socket.gethostname(), "cpu": _cpu_model(), "cores": cores, "mem_total_gb": round(total, 1),
            "mem_available_gb": round(avail, 1), "gpus": _gpus()}


def machine_id(spec: dict[str, Any]) -> str:
    """PC を見分ける名前（ホスト名、CPU、メモリの大きさ）。同じ名前の別の PC と混ざらないように。"""
    return f"{spec['host']}|{spec['cpu']}|{spec['cores']}cores|{round(spec['mem_total_gb'])}GB"


def recommend_workers(spec: dict[str, Any]) -> int:
    by_cpu = max(1, int(spec["cores"]) - RESERVE_CORES)
    avail = float(spec.get("mem_available_gb") or 0.0)
    by_mem = max(1, int(avail * MEM_FRACTION / MEM_PER_WORKER_GB)) if avail > 0 else by_cpu
    return max(1, min(by_cpu, by_mem))


def benchmark_mujoco(sim_seconds: float = 1.0) -> float | None:
    """MuJoCo のロボット 1 つで、シミュレーションの 1 秒を進めるのにかかる実時間 [秒]（描画込み）。測れなければ None。"""
    try:
        from common.config import load_config
        from contest.robots.mujoco_robot import MujocoRobot
        from contest.task import make_trial

        cfg = load_config("contest.yaml")
        trial = make_trial(0, cfg)
        robot = MujocoRobot(trial.scene_cfg, cfg)
        try:
            robot.advance(0.1)
            n = int(sim_seconds / 0.02)
            t0 = time.perf_counter()
            for k in range(n):
                robot.advance(0.02)
                robot.observe(k * 0.02)
            return (time.perf_counter() - t0) / sim_seconds
        finally:
            robot.close()
    except Exception as e:  # 公式モデルが無い、描画できない、など
        print(f"[machine] MuJoCo の速さを測れなかった: {e}")
        return None


def _load_all() -> dict[str, Any]:
    try:
        return json.loads(PROFILE_FILE.read_text())
    except Exception:
        return {}


def _save(mid: str, profile: dict[str, Any]) -> None:
    data = _load_all()
    data[mid] = profile
    PROFILE_FILE.parent.mkdir(parents=True, exist_ok=True)
    PROFILE_FILE.write_text(json.dumps(data, ensure_ascii=False, indent=2))


def estimate_minutes(profile: dict[str, Any], trials: int, sim: str) -> float | None:
    """trials 試行にかかる時間の見積もり [分]。"""
    if sim == "isaac":
        return trials * ISAAC_SECONDS_PER_TRIAL / 60.0
    workers = max(1, int(profile["workers"]))
    measured = profile.get("measured_worker_seconds_per_trial")
    if measured:
        # 前に評価したときの実測（1 プロセスあたり、1 試行にかかった実時間）。並列で回したときの無駄も入っている
        return trials * float(measured) / workers / 60.0
    s = profile.get("seconds_per_sim_second")
    if s is None:
        return None
    # 1 試行 = 準備（約 0.5 秒）+ シミュレーション。並列で回すと、描画などの取り合いで 1.5 倍くらい遅くなる
    return trials * (0.5 + s * SIM_SECONDS_PER_TRIAL) * 1.5 / workers / 60.0


def record_run(sim: str, n_trials: int, workers: int, wall_s: float) -> None:
    """評価にかかった実時間を、この PC の設定に記録する（次の見積もりに使う。MuJoCo だけ）。"""
    if sim != "mujoco" or n_trials < max(5, workers):
        return  # 試行が少ないと、準備の時間の割合が大きくて見積もりがずれる
    spec = detect_spec()
    mid = machine_id(spec)
    data = _load_all()
    if mid not in data:
        return
    per = wall_s * workers / n_trials
    old = data[mid].get("measured_worker_seconds_per_trial")
    # 試行の中身（エージェント、評価セット）で速さが変わるので、前の値と半々で混ぜる
    data[mid]["measured_worker_seconds_per_trial"] = round(per if old is None else 0.5 * (float(old) + per), 3)
    _save(mid, data[mid])


def format_minutes(m: float | None) -> str:
    if m is None:
        return "見積もれない"
    if m < 1.0:
        return f"約 {max(1, round(m * 60))} 秒"
    if m < 90:
        return f"約 {m:.0f} 分"
    return f"約 {m / 60:.1f} 時間"


def _ask_trials(profile: dict[str, Any], sim: str, multiplier: int) -> int:
    print("[machine] 試行回数の見積もり" + (f"（乱しの種類 × {multiplier} で回すので、合計はその {multiplier} 倍）" if multiplier > 1 else ""))
    for n in (20, 50, 100, 200):
        m = estimate_minutes(profile, n * multiplier, sim)
        print(f"[machine]   {n:>4} 試行: {format_minutes(m)}")
    while True:
        try:
            s = input(f"[machine] 試行回数を入力してください（既定 {DEFAULT_TRIALS}）: ").strip()
        except EOFError:
            return DEFAULT_TRIALS
        if not s:
            return DEFAULT_TRIALS
        if s.isdigit() and int(s) > 0:
            return int(s)
        print("[machine] 1 以上の整数を入力してください")


def ensure_profile(sim: str = "mujoco", ask: bool = True, reset: bool = False, multiplier: int = 1) -> dict[str, Any]:
    """この PC の設定を返す。初めての PC なら、性能を調べて並列数を決め、試行回数を聞いて保存する。

    ask: 試行回数を聞くか（試行を引数で決めているときは False）。画面に入力できない（端末でない）ときは聞かずに既定にする。
    multiplier: 1 回の指定で回る試行の倍数（--ablation なら乱しの種類の数）。見積もりの表示に使う。
    """
    spec = detect_spec()
    mid = machine_id(spec)
    profile = None if reset else _load_all().get(mid)
    if profile is not None:
        # 空きメモリは変わるので、並列数はその都度決め直す（保存した値より増やさない）
        profile["workers"] = min(int(profile.get("workers_max", profile["workers"])), recommend_workers(spec))
        if sim == "mujoco" and profile.get("seconds_per_sim_second") is None:
            # Isaac Sim で初めて評価した PC では、MuJoCo の速さをまだ測っていない
            profile["seconds_per_sim_second"] = benchmark_mujoco()
            _save(mid, profile)
        return profile

    print("[machine] このPCで評価するのは初めてです。性能を調べます")
    print(f"[machine]   CPU: {spec['cpu']}（使えるコア {spec['cores']}）")
    print(f"[machine]   メモリ: {spec['mem_total_gb']} GB（空き {spec['mem_available_gb']} GB）")
    print(f"[machine]   GPU: {', '.join(spec['gpus']) or 'なし'}")
    workers = recommend_workers(spec)
    print(f"[machine]   並列数: {workers}（コア {spec['cores']} − {RESERVE_CORES}、空きメモリ × {MEM_FRACTION} ÷ "
          f"{MEM_PER_WORKER_GB} GB の小さいほう）")
    # MuJoCo の速さは MuJoCo で評価するときだけ測る（Isaac Sim のアプリの中では測らない）
    sps = benchmark_mujoco() if sim == "mujoco" else None
    if sps is not None:
        print(f"[machine]   MuJoCo の速さ: シミュレーションの 1 秒に実時間 {sps:.2f} 秒（描画込み、1 プロセス）")
    profile = {"spec": spec, "workers": workers, "workers_max": workers, "seconds_per_sim_second": sps,
               "default_trials": DEFAULT_TRIALS, "created": time.strftime("%Y-%m-%d %H:%M:%S")}
    if ask and sys.stdin.isatty():
        profile["default_trials"] = _ask_trials(profile, sim, multiplier)
    elif ask:
        print(f"[machine] 画面から入力できないので、試行回数は既定の {DEFAULT_TRIALS} にする（--trials で変えられる）")
    _save(mid, profile)
    print(f"[machine] このPCの設定を保存した（試行回数 {profile['default_trials']}、並列数 {workers}）: {PROFILE_FILE}")
    return profile
