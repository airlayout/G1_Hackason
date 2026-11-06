#!/usr/bin/env python3
"""立っている G1 の**どこが傾いているか**を、重力と関節角で切り分ける。**PC2 で動かす。**

## なぜ要るのか

2026-09-28 の実機で、base_link（LiDAR の取付から定義）が床に対して **5.6 度前傾**していた。
床平面フィット・LiDAR の IMU・FAST_LIO・タグの 4 つが一致したので傾き自体は本物。
残る問いは「**機体ごと（骨盤から）傾いているのか、腰から上か、頭だけか**」。

| 読むもの | トピック | どこの剛体か |
|---|---|---|
| 頭の IMU（LiDAR 内蔵） | `rt/utlidar/imu_livox_mid360` | 頭（LiDAR と同じ） |
| 胴の IMU | `rt/secondary_imu` | 腰より上の胴 |
| 骨盤の IMU ＋ 関節角 | `rt/lowstate` | 骨盤・各関節 |

⚠️ **静止中だけ使うこと。** 歩行中は接地加速度が重力を汚す（2026-09-12 に最大 10.6 度）。

## 使い方

    scp quickstart/probe_tilt.py g1:~/mapping_tools/      # idl_imu.py は既に置いてある
    ssh g1 'python3 ~/mapping_tools/probe_tilt.py --seconds 5 --out /tmp/tilt.json'

⚠️ Python 3.8（PC2 の system python）で動かす。`rclpy` とは同居できない。
**読むだけで、指令は一切出さない。**
"""
import argparse
import json
import math
import os
import sys
import threading
import time

sys.path.insert(0, "/home/unitree/unitree_sdk2_python")
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))

import idl_imu  # noqa: F401,E402  sensor_msgs/Imu を SDK に注入する

HEAD_TOPIC = "rt/utlidar/imu_livox_mid360"
TORSO_TOPIC = "rt/secondary_imu"
LOW_TOPIC = "rt/lowstate"

# G1 29 自由度の関節の並び（unitree_sdk2 の G1JointIndex）
JOINTS = ["L_hip_pitch", "L_hip_roll", "L_hip_yaw", "L_knee", "L_ankle_pitch", "L_ankle_roll",
          "R_hip_pitch", "R_hip_roll", "R_hip_yaw", "R_knee", "R_ankle_pitch", "R_ankle_roll",
          "waist_yaw", "waist_roll", "waist_pitch",
          "L_shoulder_pitch", "L_shoulder_roll", "L_shoulder_yaw", "L_elbow",
          "L_wrist_roll", "L_wrist_pitch", "L_wrist_yaw",
          "R_shoulder_pitch", "R_shoulder_roll", "R_shoulder_yaw", "R_elbow",
          "R_wrist_roll", "R_wrist_pitch", "R_wrist_yaw"]


def mean(rows):
    n = len(rows)
    return [sum(r[i] for r in rows) / n for i in range(len(rows[0]))] if n else None


def tilt_from_gravity(a):
    """加速度（静止なら重力の反力 = 上向き）から、その剛体の座標で見た傾き [deg]。"""
    x, y, z = a
    n = math.sqrt(x * x + y * y + z * z)
    return {"tilt": math.degrees(math.acos(abs(z) / n)),
            "pitch_like": math.degrees(math.atan2(x, abs(z))),
            "roll_like": math.degrees(math.atan2(y, abs(z))),
            "norm": n}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--seconds", type=float, default=5.0)
    ap.add_argument("--out", default="/tmp/tilt.json")
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--domain-id", type=int, default=0)
    a = ap.parse_args()

    from unitree_sdk2py.core.channel import ChannelFactoryInitialize, ChannelSubscriber
    from unitree_sdk2py.idl.sensor_msgs.msg.dds_ import Imu_
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import IMUState_, LowState_

    head, torso, pelvis_acc, pelvis_rpy, q = [], [], [], [], []
    lock = threading.Lock()

    def on_head(m):
        g = m.linear_acceleration
        with lock:
            head.append([g.x, g.y, g.z])

    def on_torso(m):
        with lock:
            torso.append([float(v) for v in m.accelerometer] + [float(v) for v in m.rpy])

    def on_low(m):
        with lock:
            pelvis_acc.append([float(v) for v in m.imu_state.accelerometer])
            pelvis_rpy.append([float(v) for v in m.imu_state.rpy])
            q.append([float(m.motor_state[i].q) for i in range(len(JOINTS))])

    ChannelFactoryInitialize(a.domain_id, a.iface)
    subs = []
    for topic, typ, cb in ((HEAD_TOPIC, Imu_, on_head), (TORSO_TOPIC, IMUState_, on_torso),
                           (LOW_TOPIC, LowState_, on_low)):
        s = ChannelSubscriber(topic, typ)
        s.Init(cb, 50)
        subs.append(s)

    print("[tilt] %.0f 秒 記録する（機体は静止させておくこと）" % a.seconds)
    time.sleep(a.seconds)

    with lock:
        h, t, pa, pr, qq = mean(head), mean(torso), mean(pelvis_acc), mean(pelvis_rpy), mean(q)
        counts = [len(head), len(torso), len(pelvis_acc)]
    out = {"counts": dict(zip(["head", "torso", "lowstate"], counts))}
    if h:
        out["head_acc"] = h
        out["head_tilt"] = tilt_from_gravity(h)
    if t:
        out["torso_acc"] = t[:3]
        out["torso_rpy_deg"] = [math.degrees(v) for v in t[3:6]]
        out["torso_tilt"] = tilt_from_gravity(t[:3])
    if pa:
        out["pelvis_acc"] = pa
        out["pelvis_rpy_deg"] = [math.degrees(v) for v in pr]
        out["pelvis_tilt"] = tilt_from_gravity(pa)
        out["joints_deg"] = {n: math.degrees(v) for n, v in zip(JOINTS, qq)}
    with open(a.out, "w") as f:
        json.dump(out, f, indent=1)

    print("[tilt] 件数 頭 %d / 胴 %d / lowstate %d" % tuple(counts))
    for key in ("head_tilt", "torso_tilt", "pelvis_tilt"):
        if key in out:
            v = out[key]
            print("[tilt] %-11s 傾き %5.2f 度（x成分 %+6.2f / y成分 %+6.2f）|a| %.3f"
                  % (key, v["tilt"], v["pitch_like"], v["roll_like"], v["norm"]))
    for key in ("torso_rpy_deg", "pelvis_rpy_deg"):
        if key in out:
            print("[tilt] %-14s roll %+6.2f pitch %+6.2f yaw %+7.2f" % ((key,) + tuple(out[key])))
    if "joints_deg" in out:
        j = out["joints_deg"]
        for n in ("waist_pitch", "waist_roll", "L_hip_pitch", "R_hip_pitch", "L_knee", "R_knee",
                  "L_ankle_pitch", "R_ankle_pitch", "L_shoulder_pitch", "R_shoulder_pitch",
                  "L_elbow", "R_elbow"):
            print("[tilt] %-17s %+7.2f 度" % (n, j[n]))
    print("[tilt] -> %s" % a.out)
    return 0 if all(counts) else 1


if __name__ == "__main__":
    sys.exit(main())
