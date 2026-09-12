#!/usr/bin/env python3
"""頭（LiDAR）の IMU と胴体の IMU の**ジャイロを同時記録**する。**PC2 で動かす。**

## なぜ要るのか

2026-09-12 の実機で、**その場旋回中だけ測位がすべる**ことが分かった
（純回転 0% の 2 本は到達、50% の 1 本は推定が 1.64 m の幻の並進を出して中止）。

調べると G1 固有の原因が論文に名指しで書かれていた
（arXiv 2604.17335 §III-D）:

> Since the neck joint of the G1 robot is passive, aggressive motions can
> induce head movement that degrades base pose estimation.

**LiDAR は頭に載っていて、首は受動関節。**だから頭が揺れると
`base_link -> livox_frame` の**静的**変換が動作中に嘘になり、
腕の長さ 1.228 m ぶん、向きの誤差が位置の誤差に化ける。

## なぜジャイロ同士を比べるのか

重力で調べる手は**歩行中は使えない**。足の接地加速度が重力の測定を汚すからで、
実際 2026-09-12 の記録では静止 0.31° に対し歩行中は中央 1.9〜3.6°・最大 10.6° 出た
（うちどれだけが首の揺れでどれだけが接地加速度かを分けられない）。

**ジャイロ同士なら重力も接地加速度も関係ない。**
首が剛体なら ω_頭 = ω_胴体、受動で揺れるなら差が出る。それだけを見る。

## 使い方

    # PC2 へ配る（Mapping/real から）
    scp quickstart/probe_neck_imu.py g1:~/mapping_tools/

    # 記録（機体を旋回させながら）
    ssh g1 'python3 ~/mapping_tools/probe_neck_imu.py --seconds 20 --out /tmp/neck.json'
    scp g1:/tmp/neck.json .

⚠️ **Python 3.8（PC2 の system python）で動かす。**`rclpy` とは同居できないので
ROS 側からは呼べない（`Navigation/real/README.md` の 2 プロセス構成と同じ理由）。
"""
import argparse
import json
import os
import sys
import threading
import time

sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import idl_imu  # noqa: F401,E402  sensor_msgs/Imu を SDK に注入する

HEAD_TOPIC = "rt/utlidar/imu_livox_mid360"      # LiDAR と同じ剛体（頭）
TORSO_TOPIC = "rt/secondary_imu"                # 胴体側


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=20.0)
    ap.add_argument("--out", default="/tmp/neck.json")
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--domain-id", type=int, default=0)
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_

    head, torso = [], []
    lock = threading.Lock()

    def on_head(m):
        # ⚠️ 頭側は **メッセージ内のスタンプ**を使う（LiDAR と同じ時計）
        t = m.header.stamp.sec + m.header.stamp.nanosec * 1e-9
        g = m.angular_velocity
        with lock:
            head.append([t, g.x, g.y, g.z])

    def on_torso(m):
        # ⚠️ IMUState_ にスタンプは無い。**受信時刻で代用する。**
        # 比較するのは「同じ時間帯の ω の大きさ」なので、数 ms のずれは効かない。
        # ただし**時刻ぴったりの引き算はしない**（下の解析でも帯で比べる）
        with lock:
            torso.append([time.time(), float(m.gyroscope[0]),
                          float(m.gyroscope[1]), float(m.gyroscope[2])])

    ChannelFactoryInitialize(a.domain_id, a.iface)
    sub_h = ChannelSubscriber(HEAD_TOPIC, Imu_)
    sub_h.Init(on_head, 50)
    sub_t = ChannelSubscriber(TORSO_TOPIC, IMUState_)
    sub_t.Init(on_torso, 50)

    print("[neck] {:.0f} 秒 記録する。**この間に機体を旋回させること**".format(a.seconds))
    t0 = time.time()
    while time.time() - t0 < a.seconds:
        time.sleep(0.5)
        with lock:
            nh, nt = len(head), len(torso)
        print("[neck] 頭 {:>6} 件 / 胴体 {:>6} 件".format(nh, nt), end="\r")
    print()

    with lock:
        out = {"head": head, "torso": torso,
               "head_topic": HEAD_TOPIC, "torso_topic": TORSO_TOPIC,
               "wall_start": t0, "seconds": a.seconds}
    if not head:
        print("[neck] ⚠️ 頭の IMU が 1 件も来ていない", file=sys.stderr)
    if not torso:
        print("[neck] ⚠️ 胴体の IMU が 1 件も来ていない（{} を確かめる）".format(TORSO_TOPIC),
              file=sys.stderr)
    with open(a.out, "w") as f:
        json.dump(out, f)
    print("[neck] -> {}  頭 {} 件 / 胴体 {} 件".format(a.out, len(head), len(torso)))
    return 0 if (head and torso) else 1


if __name__ == "__main__":
    sys.exit(main())
