"""PC2 上のプロセスの CPU を、窓の中の平均で測る（標準ライブラリのみ・Python 3.8 可）。

`ps` の `%CPU` は**起動からの平均**なので、「いま何コア食っているか」には使えない。
`/proc/<pid>/stat` の utime+stime を窓の前後で差分する。系全体の負荷も同時に返す。

    scp pc2_cpu_probe.py g1:/tmp/
    ssh g1 'python3 /tmp/pc2_cpu_probe.py --seconds 30 --label "生 LiDAR のみ"'
    ssh g1 'python3 /tmp/pc2_cpu_probe.py --match mola-cli --seconds 30'

⚠️ **既定の照合が** **`lib/foxglove_bridge/foxglove_bridge`** **なのには理由がある。**
`ros2 run foxglove_bridge foxglove_bridge --ros-args ...` という**起動役の python も
同じ語を全部含む**。そちらを測ると **0.0%** ＝「タダ」という誤った結論が出る
（2026-09-13 に 1 度出した）。**実体は lib/ 下の実行ファイルの方。**

2026-09-13 の実測（Orin NX 8 コア・実機の生 LiDAR が流れている状態）:
購読者なし 3.1% / 生 LiDAR のみ 5.7% / RViz2 相当の全部 6.1%（= **0.06 コア**）。
"""
from __future__ import annotations

import argparse, os, sys, time

DEFAULT_MATCH = "lib/foxglove_bridge/foxglove_bridge"


def find_pid(match: str) -> int:
    for pid in filter(str.isdigit, os.listdir("/proc")):
        try:
            with open("/proc/%s/cmdline" % pid, "rb") as fh:
                cmdline = fh.read().decode("utf-8", "replace")
        except OSError:
            continue                       # 読む間に終わったプロセス
        if match in cmdline:
            return int(pid)
    raise SystemExit("該当プロセスが見つからない: %s" % match)


def proc_jiffies(pid: int) -> int:
    with open("/proc/%d/stat" % pid) as fh:
        # comm には空白や ')' が入りうるので、最後の ')' で切る
        fields = fh.read().rsplit(")", 1)[1].split()
    return int(fields[11]) + int(fields[12])          # utime + stime


def cpu_total() -> "tuple[int, int]":
    with open("/proc/stat") as fh:
        parts = [int(x) for x in fh.readline().split()[1:]]
    return sum(parts), parts[3] + parts[4]            # 合計, idle + iowait


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("--match", default=DEFAULT_MATCH, help="cmdline に含まれる文字列")
    ap.add_argument("--seconds", type=float, default=30.0)
    ap.add_argument("--label", default="")
    args = ap.parse_args()

    hz = os.sysconf("SC_CLK_TCK")
    ncpu = os.cpu_count() or 1
    pid = find_pid(args.match)

    before, (total0, idle0), t0 = proc_jiffies(pid), cpu_total(), time.monotonic()
    time.sleep(args.seconds)
    after, (total1, idle1), t1 = proc_jiffies(pid), cpu_total(), time.monotonic()

    wall = t1 - t0
    proc_pct = (after - before) / hz / wall * 100.0
    sys_pct = (1.0 - (idle1 - idle0) / (total1 - total0)) * 100.0 * ncpu

    print("%-28s pid %-7d %6.1f%%  (%.2f コア相当 / 全 %d コア)  系全体 %6.1f%%  窓 %.1fs"
          % (args.label or args.match, pid, proc_pct, proc_pct / 100, ncpu, sys_pct, wall))
    return 0


if __name__ == "__main__":
    sys.exit(main())
