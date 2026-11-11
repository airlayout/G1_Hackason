"""11-03: ライダーの追跡が旋回の回り始めで崩れる原因を調べるため、純正の歩行でその場の旋回だけを決まった速さで繰り返す。

腕は借りない（純正のまま）。歩きは LocoClient.SetVelocity(0, 0, wz, 0.5 秒) を 20 回/秒で送り、区間の終わりと止める時は
SetVelocity(0, 0, 0, 0.5) を 3 回送る（door_stock_io.VelocitySender と同じ作法）。
止まる: リモコンの B／select・Ctrl+C・傾き 10° 超・rt/lowstate の途切れ 0.1 s・SetVelocity の失敗 5 回続けて。脱力はしない
記録: logs/turn_probe_<日時>.npz（時刻・区間・送った wz・胴のジャイロ・pelvis の四元数・関節の角度・SetVelocity の応答）

使い方（PC2・機体は通常運転 501・扉の前）: python3 -u turn_probe.py [--plan "0.2:2,0:2,-0.2:2,0:2"]
  --plan は「wz [rad/s]:秒」を , でつなぐ。既定は ±0.2・±0.35・±0.5 を、行って戻る形で（各 ±23〜29°）
  11-09: 前後・左右の指令の掃引にも使う。区間を「vx/vy/wz:秒」と書く（例 "0.1/0/0:3,0:2,-0.1/0/0:3,0:2"）。
    小さい指令に純正の脚がどれだけ応えるか（シミュレータの層 v2 との答え合わせ）。正解は rec_follow.sh の生の点群と /dog_odom
"""
from __future__ import annotations

import argparse
import math
import signal
import sys
import time
from datetime import datetime
from pathlib import Path
from typing import List, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import door_run_real as drr  # noqa: E402
import door_stock_io as dsio  # noqa: E402

DEFAULT_PLAN = ("0:2,0.2:2,0:2,-0.2:2,0:2,-0.2:2,0:2,0.2:2,0:3,"
                "0.35:1.4,0:2,-0.35:1.4,0:2,-0.35:1.4,0:2,0.35:1.4,0:3,"
                "0.5:1,0:2,-0.5:1,0:2,-0.5:1,0:2,0.5:1,0:3")
MAX_ABS_WZ = 0.6  # [rad/s] これより速い指令は受け付けない
MAX_SEG_DEG = 35.0  # 1 区間で回す角度の上限
MAX_ABS_V = 0.5  # [m/s] 前後・左右の指令の上限（学習の前の上限と同じ）
MAX_SEG_M = 1.6  # 1 区間で送る距離（指令×秒）の上限。純正は前で 0.5 倍前後しか進まないので実際は 0.8 m 前後
STOP_TILT_DEG = 10.0
STATE_STALE_S = 0.1
VEL_FAIL_TRIP = 5
DT = 0.02


def parse_plan(text: str) -> List[Tuple[Tuple[float, float, float], float]]:
    """「wz:秒,...」か「vx/vy/wz:秒,...」→ [((vx, vy, wz), 秒)]。速すぎる・回し過ぎる・進み過ぎる区間は ValueError"""
    plan = []
    for part in text.split(","):
        vel, secs_text = part.split(":")
        comps = [float(v) for v in vel.split("/")]
        vx, vy, wz = (0.0, 0.0, comps[0]) if len(comps) == 1 else comps
        secs = float(secs_text)
        if abs(wz) > MAX_ABS_WZ or secs <= 0 or abs(math.degrees(wz * secs)) > MAX_SEG_DEG:
            raise ValueError(f"区間 {part} は上限（{MAX_ABS_WZ} rad/s・1 区間 {MAX_SEG_DEG}°）の外")
        if max(abs(vx), abs(vy)) > MAX_ABS_V or math.hypot(vx, vy) * secs > MAX_SEG_M:
            raise ValueError(f"区間 {part} は上限（{MAX_ABS_V} m/s・1 区間 {MAX_SEG_M} m）の外")
        plan.append(((vx, vy, wz), secs))
    return plan


def main(argv=None) -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--plan", default=DEFAULT_PLAN)
    ap.add_argument("--iface", default="eth0")
    ap.add_argument("--log-dir", default="logs")
    a = ap.parse_args(argv)
    plan = parse_plan(a.plan)

    io = drr.RobotIO(a.iface, send=False, use_odom=False)
    t_wait = time.monotonic()
    while io.snapshot()[0] is None:
        if time.monotonic() - t_wait > 5.0:
            print("[turn] rt/lowstate が来ない")
            return 1
        time.sleep(0.05)
    loco = dsio.make_loco_client(1.0)
    fsm = dsio.fsm_id(loco)
    if fsm not in dsio.WALKING_FSM_IDS:
        print(f"[turn] 起動しない: FSM={fsm} は通常運転（500/501）ではない")
        return 1
    sender = dsio.VelocitySender(loco)
    sender.start()

    stop = {"why": ""}
    signal.signal(signal.SIGINT, lambda *_: stop.update(why="Ctrl+C"))
    signal.signal(signal.SIGHUP, lambda *_: stop.update(why="ssh の切断"))
    print(f"[turn] FSM {fsm}・{len(plan)} 区間・合計 {sum(s for _, s in plan):.1f} 秒。止めるのはリモコンの B／select か Ctrl+C", flush=True)

    rows = {k: [] for k in ("t", "seg", "wz_cmd", "v_cmd", "gyro", "quat", "q", "rpc_ms", "code")}
    keys_prev = 0
    t0 = time.monotonic()
    next_t = t0
    seg_ends = np.cumsum([s for _, s in plan])
    try:
        while True:
            now = time.monotonic()
            el = now - t0
            seg = int(np.searchsorted(seg_ends, el, side="right"))
            if seg >= len(plan):
                break
            state, state_t, _, _ = io.snapshot()
            quat = np.array(state.imu_state.quaternion, dtype=float)
            g = dc.quat_wxyz_to_gravity(quat)
            tilt = math.degrees(math.acos(max(-1.0, min(1.0, -g[2]))))
            keys = drr.parse_keys(state.wireless_remote)
            edge = keys & ~keys_prev
            keys_prev = keys
            vst = sender.status()
            if edge & ((1 << drr.KEY["B"]) | (1 << drr.KEY["select"])):
                stop["why"] = "リモコンの B／select"
            elif tilt > STOP_TILT_DEG:
                stop["why"] = f"傾き {tilt:.1f}°"
            elif now - state_t > STATE_STALE_S:
                stop["why"] = f"rt/lowstate が {now - state_t:.2f} s 途切れた"
            elif vst.failed_in_row >= VEL_FAIL_TRIP:
                stop["why"] = f"SetVelocity が {vst.failed_in_row} 回続けて失敗（code={vst.last_code}）"
            if stop["why"]:
                break
            vx, vy, wz = plan[seg][0]
            sender.set((vx, vy, wz) != (0.0, 0.0, 0.0), vx, vy, wz)
            rows["t"].append(now)
            rows["seg"].append(seg)
            rows["wz_cmd"].append(wz)
            rows["v_cmd"].append((vx, vy, wz))
            rows["gyro"].append(np.array(state.imu_state.gyroscope, dtype=float))
            rows["quat"].append(quat)
            rows["q"].append(np.array([state.motor_state[j].q for j in range(dc.NUM_DOF)]))
            rows["rpc_ms"].append(vst.last_rpc_ms)
            rows["code"].append(np.nan if vst.last_code is None else vst.last_code)
            if len(rows["t"]) % 25 == 1:
                print(f"[turn] {el:5.1f} s 区間 {seg + 1}/{len(plan)} 指令 {vx:+.2f} {vy:+.2f} {wz:+.2f}・胴の回る速さ {rows['gyro'][-1][2]:+.2f} rad/s・"
                      f"傾き {tilt:.1f}°・RPC {vst.last_rpc_ms:.0f} ms", flush=True)
            next_t += DT
            slack = next_t - time.monotonic()
            if slack > 0:
                time.sleep(slack)
            else:
                next_t = time.monotonic()
    finally:
        sender.stop()
        time.sleep(0.3)
        sender.close()
        out = Path(a.log_dir) / f"turn_probe_{datetime.now().strftime('%Y%m%d_%H%M%S')}.npz"
        np.savez_compressed(out, **{k: np.array(v) for k, v in rows.items()},
                            plan=np.array([[*vel, secs] for vel, secs in plan]), reason=np.array(stop["why"] or "全区間を終えた"))
        print(f"[turn] 止めた（{stop['why'] or '全区間を終えた'}）。記録 {out}", flush=True)
    return 0


if __name__ == "__main__":
    sys.exit(main())
