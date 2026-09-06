"""建図が止まった瞬間を捕まえるための見張り。PC2で記録と並行して動かす。

## なぜ必要か

建図は放置すると自動停止する（2026-09-03に2回発生。約16分と20〜40分）。
停止時のロボットは `sportMode=-1` / `gaitType=-1` ＝ アクティブな動作モードに
入っていない状態だった。原因は未特定で、「時間で止まる」のか「静止で止まる」のかが
切り分けられていない。

⚠️ `sportmodestate` は**G1では読めない**。実機が配信しているのは
`unitree_hg/msg/SportModeState` だが、PC2 の `unitree_sdk2py` にこの型が無い
（`unitree_hg` にあるのは LowState_ など。IMU と同じ欠落。2026-09-06 実測）。
`unitree_go` の型で購読しても型名が違うので DDS がマッチしない。
代わりに `rt/lowstate`（`unitree_hg/LowState_`）の `mode_pr` / `mode_machine` を見る。
こちらが G1 の実際の FSM 状態で、型は SDK に在る。

`record_dds_to_bag.py` は受信件数を2秒ごとに出すが、**止まった瞬間のロボットの状態は
残らない**。切り分けに要るのはそこなので、点群が途切れた時刻と、そのときの
`mode` / `gait_type` / `slam_info` を並べて記録する。

購読するだけで、ロボットには何も指令しない。

    ssh g1 'python3 ~/mapping_tools/watch_mapping.py > ~/watch_mapping.log 2>&1 &'
    ssh g1 'tail -f ~/watch_mapping.log'
"""
import argparse
import json
import sys
import threading
import time
from datetime import datetime

sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber  # noqa: E402
from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import PointCloud2_  # noqa: E402
from unitree_sdk2py.idl.std_msgs.msg.dds_ import String_  # noqa: E402
from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_  # noqa: E402

POINTS_TOPIC = "rt/unitree/slam_mapping/points"
SLAM_INFO_TOPIC = "rt/slam_info"
LOWSTATE_TOPIC = "rt/lowstate"

# Hzは直近この秒数の受信で出す。建図点群は約10Hzなので5秒あれば十分に安定する
HZ_WINDOW_SEC = 5.0


def now_text():
    return datetime.now().strftime("%H:%M:%S")


class Watcher:
    """受信状況を集めるだけの器。値は書き換えず、必要なぶんだけ差し替える。"""

    def __init__(self):
        self.lock = threading.Lock()
        self.point_times = []          # 直近の点群受信時刻
        self.total_points = 0
        self.last_point_time = None
        self.mode = None               # (mode_pr, mode_machine, 受信時刻)
        self.slam = None               # (state, ctrName, 受信時刻)

    def on_points(self, msg):
        t = time.time()
        with self.lock:
            self.total_points += 1
            self.last_point_time = t
            self.point_times = [x for x in self.point_times if t - x <= HZ_WINDOW_SEC] + [t]

    def on_lowstate(self, msg):
        with self.lock:
            self.mode = (msg.mode_pr, msg.mode_machine, time.time())

    def on_slam_info(self, msg):
        try:
            payload = json.loads(msg.data)
            machine = payload.get("data", {}).get("stateMachine", {})
            value = (machine.get("state"), machine.get("ctrName"), time.time())
        except Exception:
            value = (None, None, time.time())
        with self.lock:
            self.slam = value

    def snapshot(self):
        with self.lock:
            times = list(self.point_times)
            return {
                "total": self.total_points,
                "last": self.last_point_time,
                "hz": (len(times) - 1) / max(times[-1] - times[0], 1e-6) if len(times) > 1 else 0.0,
                "mode": self.mode,
                "slam": self.slam,
            }


def describe(snap):
    mode = snap["mode"]
    slam = snap["slam"]
    mode_text = "mode_pr={} mode_machine={}".format(mode[0], mode[1]) if mode else "lowstate 未受信"
    slam_text = "slam={}/{}".format(slam[0], slam[1]) if slam else "slam_info 未受信"
    return "{}  {}".format(mode_text, slam_text)


def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--iface", default="eth0")
    parser.add_argument("--domain-id", type=int, default=0)
    parser.add_argument("--interval", type=float, default=5.0, help="状況を出す間隔（秒）")
    parser.add_argument("--stall", type=float, default=3.0,
                        help="この秒数だけ点群が途切れたら停止とみなす（約10Hz配信）")
    args = parser.parse_args()

    watcher = Watcher()
    ChannelFactoryInitialize(args.domain_id, args.iface)

    subscriptions = [
        (POINTS_TOPIC, PointCloud2_, watcher.on_points),
        (SLAM_INFO_TOPIC, String_, watcher.on_slam_info),
        (LOWSTATE_TOPIC, LowState_, watcher.on_lowstate),
    ]
    keep_alive = []
    for topic, msg_type, handler in subscriptions:
        subscriber = ChannelSubscriber(topic, msg_type)
        subscriber.Init(handler, 10)
        keep_alive.append(subscriber)

    print("[watch] 見張りを開始（iface={} 停止判定={}秒）".format(args.iface, args.stall), flush=True)
    print("[watch] 監視: {} / {} / {}".format(POINTS_TOPIC, LOWSTATE_TOPIC, SLAM_INFO_TOPIC), flush=True)

    started = time.time()
    stalled = False
    while True:
        time.sleep(args.interval)
        snap = watcher.snapshot()
        elapsed = time.time() - started
        gap = time.time() - snap["last"] if snap["last"] else None

        # まだ一度も届いていないうちは停止と呼ばない（建図の開始前でも動かせるようにする）
        if snap["last"] is None:
            print("{} [{:5.0f}s] 点群まだ来ていない   {}".format(
                now_text(), elapsed, describe(snap)), flush=True)
            continue

        if gap > args.stall and not stalled:
            stalled = True
            print("", flush=True)
            print("{} !!!! 建図が止まった（{:.1f}秒 途切れ）".format(now_text(), gap), flush=True)
            print("      止まった時点の状態: {}".format(describe(snap)), flush=True)
            print("      ここまでの受信 {} 件 / 経過 {:.0f}秒".format(snap["total"], elapsed), flush=True)
            print("", flush=True)
            continue

        if gap <= args.stall and stalled:
            stalled = False
            print("{} 復帰した（{:.2f}Hz）  {}".format(now_text(), snap["hz"], describe(snap)), flush=True)
            continue

        if stalled:
            print("{} [{:5.0f}s] 停止のまま {:.0f}秒   {}".format(
                now_text(), elapsed, gap, describe(snap)), flush=True)
        else:
            print("{} [{:5.0f}s] {:5.2f}Hz  計{:6d}件   {}".format(
                now_text(), elapsed, snap["hz"], snap["total"], describe(snap)), flush=True)


if __name__ == "__main__":
    main()
