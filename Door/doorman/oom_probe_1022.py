"""映像の生徒の学習が GPU のメモリ不足で落ちる理由を調べる、軽い見張り（学習には触れない。OMEN の system の python3）。

  gpu.csv    時刻,GPU の使用量 [MiB]（nvidia-smi -lms 250 を 1 本だけ常駐させて 0.25 秒ごと）
  iters.log  時刻 と、最新の学習の train.log の「Learning iteration」「Computation」「Iteration time」「Traceback」「OutOfMemory」の行
             （学習の 1 回の区切りと、集める段・学習の段の長さが分かる。落ちた時刻も残る）

使い方: nice -n 19 python3 oom_probe_1022.py <出力先> <時間>
"""
from __future__ import annotations

import glob
import os
import re
import subprocess
import sys
import threading
import time

LOGS = os.path.expanduser("~/work_inoue/doorman6/tests/campitch_dagger16_*/train.log")
PAT = re.compile(r"Learning iteration|Computation|Iteration time|Traceback|OutOfMemory")
ANSI = re.compile(r"\x1b\[[0-9;]*m")


def read_gpu(out: str, stop: threading.Event) -> None:
    cmd = ["nvidia-smi", "--query-gpu=memory.used", "--format=csv,noheader,nounits", "-lms", "250"]
    with subprocess.Popen(cmd, stdout=subprocess.PIPE, text=True, bufsize=1) as p, open(f"{out}/gpu.csv", "a", buffering=1) as f:
        for line in p.stdout:
            f.write(f"{time.time():.2f},{line.strip()}\n")
            if stop.is_set():
                p.terminate()
                break


def follow_logs(out: str, end: float) -> None:
    cur, fh, last_scan = None, None, 0.0
    with open(f"{out}/iters.log", "a", buffering=1) as it:
        while time.time() < end:
            if time.time() - last_scan > 20:
                last_scan = time.time()
                logs = sorted(glob.glob(LOGS), key=os.path.getmtime)
                if logs and logs[-1] != cur:
                    cur = logs[-1]
                    fh = open(cur, errors="replace")
                    fh.seek(0, 2)
                    it.write(f"{time.time():.2f} FILE {cur}\n")
            if fh is not None:
                for line in fh.readlines():
                    if PAT.search(line):
                        it.write(f"{time.time():.2f} {ANSI.sub('', line).strip()}\n")
            time.sleep(0.2)


def main() -> None:
    out, hours = sys.argv[1], float(sys.argv[2])
    os.makedirs(out, exist_ok=True)
    stop = threading.Event()
    t = threading.Thread(target=read_gpu, args=(out, stop), daemon=True)
    t.start()
    follow_logs(out, time.time() + hours * 3600)
    stop.set()


if __name__ == "__main__":
    main()
