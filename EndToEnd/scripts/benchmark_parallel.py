"""実際に保存・flushした並列収集を逐次比較する。GPU使用量と起動込み速度も残す。"""

import argparse
import json
import os
from pathlib import Path
import shutil
import subprocess
import signal
import time


def monitor(process, gpu, samples):
    while process.poll() is None:
        status = subprocess.run(
            [
                "nvidia-smi",
                "-i",
                str(gpu),
                "--query-gpu=memory.used,utilization.gpu",
                "--format=csv,noheader,nounits",
            ],
            capture_output=True,
            text=True,
            check=False,
        )
        if status.returncode == 0:
            samples.append([int(part.strip()) for part in status.stdout.strip().split(",")])
        try:
            process.wait(timeout=0.5)
        except subprocess.TimeoutExpired:
            pass


def main():
    def interrupt(signum, frame):
        raise KeyboardInterrupt

    signal.signal(signal.SIGTERM, interrupt)
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--envs", type=int, nargs="+", default=[1, 4, 8, 16, 32])
    parser.add_argument("--steps", type=int, default=150)
    parser.add_argument("--gpu", type=int, default=1)
    parser.add_argument("--compression", choices=["lzf", "none"], default="lzf")
    parser.add_argument("--path-samples", type=int, default=8)
    parser.add_argument("--max-bounces", type=int, default=3)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    args.output.mkdir(parents=True, exist_ok=False)
    reports = []
    bytes_per_env = 200_000_000 * args.steps * 0.02
    for count in args.envs:
        if shutil.disk_usage(args.output).free < count * bytes_per_env + 4_000_000_000:
            reports.append(
                {"num_envs": count, "skipped": "insufficient free disk space for full fidelity benchmark"}
            )
            break
        trial = args.output / f"n{count}"
        command = [
            "bash",
            str(root / "parallel-patrol.sh"),
            "--headless",
            "--device",
            f"cuda:{args.gpu}",
            "--num-envs",
            str(count),
            "--steps",
            str(args.steps),
            "--output",
            str(trial.resolve()),
            "--compression",
            args.compression,
            "--path-samples",
            str(args.path_samples),
            "--max-bounces",
            str(args.max_bounces),
        ]
        samples = []
        started = time.perf_counter()
        with (args.output / f"n{count}.log").open("w") as log:
            process = subprocess.Popen(command, stdout=log, stderr=subprocess.STDOUT, start_new_session=True)
            try:
                monitor(process, args.gpu, samples)
            finally:
                if process.poll() is None:
                    os.killpg(process.pid, signal.SIGTERM)
                    try:
                        process.wait(timeout=10)
                    except subprocess.TimeoutExpired:
                        os.killpg(process.pid, signal.SIGKILL)
                        process.wait()
        elapsed = time.perf_counter() - started
        report = {
            "num_envs": count,
            "exit_code": process.returncode,
            "process_wall_seconds": elapsed,
            "gpu_peak_memory_mib": max((s[0] for s in samples), default=None),
            "gpu_mean_utilization_percent": sum(s[1] for s in samples) / len(samples) if samples else None,
        }
        if process.returncode == 0 and (trial / "summary.json").is_file():
            summary = json.loads((trial / "summary.json").read_text())
            report.update(summary)
            report["process_realtime_factor"] = summary["aggregate_simulated_seconds"] / elapsed
            paths = list(trial.glob("env_*/dataset.h5"))
            report["dataset_bytes"] = sum(path.stat().st_size for path in paths)
            bytes_per_env = report["dataset_bytes"] / count
            with (args.output / f"n{count}-audit.log").open("w") as log:
                result = subprocess.run(
                    ["bash", str(root / "check-parallel.sh"), str(trial)],
                    stdout=log,
                    stderr=subprocess.STDOUT,
                )
            report["sensor_audit_passed"] = result.returncode == 0
            if result.returncode != 0:
                report["accepted_realtime_factor"] = 0.0
        else:
            report["failure"] = (
                "collection did not produce a completed summary; inspect trial log/failure.json"
            )
        reports.append(report)
        (args.output / "benchmark.json").write_text(json.dumps(reports, indent=2) + "\n")
        print(json.dumps({k: v for k, v in report.items() if k != "environments"}), flush=True)
        if process.returncode != 0 or report.get("sensor_audit_passed") is False or report.get("failure"):
            break
    (args.output / "benchmark.json").write_text(json.dumps(reports, indent=2) + "\n")


if __name__ == "__main__":
    main()
