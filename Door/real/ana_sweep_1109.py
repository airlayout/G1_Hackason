"""11-09: 純正の脚の指令の掃引（turn_probe.py の「vx/vy/wz:秒」の区間）を /dog_odom で測り、区間ごとに指令の何割進んだかを出す。

r1c7 が扉の 0.5 m 手前で留まった（2 回）とき、小さい前・左右の指令への応えがシミュレータの層 v2 より弱かった。その答え合わせ。

  Navigation/.venv/bin/python Door/real/ana_sweep_1109.py <turn_probe_*.npz> <…_odom のセッション> <時計の差 [s]> [<npz> <odom> <差> …]

時計の差は PC2 で time.time() − time.monotonic()（turn_probe の t は monotonic・db3 の時刻は time.time_ns()）。
進んだ量は区間の始めの胴の向きで前・左に分け、区間の終わりから TAIL_S の惰性も含める（純正は止めの指令から 0.5 s ほど遅れて減速する）。
⚠️ /dog_odom は距離を前後で 4〜6%・横で 7〜10% 短く出す（10-21 実測）。ここの割合はその補正をしていない
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import dog_odom_bag as dob  # noqa: E402

TAIL_S = 1.0  # 区間の終わりから足す惰性の秒
STEADY_S = 1.5  # 区間の最後のこの秒で、速さの落ち着いた値を出す
MOVE_MPS = 0.03  # 動き始めとみなす胴の速さ [m/s]


def body_disp(track: dob.OdomTrack, t0: float, t1: float) -> np.ndarray:
    """t0 の胴の向きで見た t0→t1 の動き（前・左 [m]）と向きの変化 [rad]"""
    i0, i1 = np.searchsorted(track.recv_s, [t0, t1])
    i1 = min(i1, len(track.recv_s) - 1)
    d = track.xyz[i1, :2] - track.xyz[i0, :2]
    c, s = math.cos(track.yaw[i0]), math.sin(track.yaw[i0])
    return np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1], track.yaw[i1] - track.yaw[i0]])


def start_lag(track: dob.OdomTrack, t0: float, t1: float, axis: int) -> float:
    """区間の始めから、指令の軸の速さが MOVE_MPS を越えるまでの秒（越えなければ NaN）"""
    i0, i1 = np.searchsorted(track.recv_s, [t0, t1])
    t = track.recv_s[i0:i1]
    if len(t) < 50:
        return math.nan
    c, s = math.cos(track.yaw[i0]), math.sin(track.yaw[i0])
    xy = track.xyz[i0:i1, :2]
    body = np.stack([c * xy[:, 0] + s * xy[:, 1], -s * xy[:, 0] + c * xy[:, 1]], axis=1)[:, axis]
    k = 100  # 約 0.1 s の差で速さを出す（1000 Hz）
    v = (body[k:] - body[:-k]) / (t[k:] - t[:-k])
    hit = np.nonzero(np.abs(v) > MOVE_MPS)[0]
    return float(t[hit[0]] - t0) if len(hit) else math.nan


def segments(npz: Path, session: Path, offset: float) -> list[dict]:
    d = np.load(npz)
    track = dob.load_odom(session)
    plan = d["plan"]  # (区間, [vx, vy, wz, 秒])
    t_wall = d["t"] + offset
    seg = d["seg"]
    out = []
    for k, (vx, vy, wz, secs) in enumerate(plan):
        if (vx, vy) == (0.0, 0.0):
            continue
        idx = np.nonzero(seg == k)[0]
        if len(idx) == 0:
            continue
        t0, t1 = t_wall[idx[0]], t_wall[idx[-1]]
        axis, cmd = (0, vx) if vx != 0.0 else (1, vy)
        whole = body_disp(track, t0, t1 + TAIL_S)
        steady = body_disp(track, t1 - STEADY_S, t1)
        out.append(dict(axis="前後" if axis == 0 else "左右", cmd=cmd, secs=secs, sent_m=cmd * secs,
                        moved_m=whole[axis], ratio=whole[axis] / (cmd * secs),
                        steady_mps=steady[axis] / STEADY_S, off_axis_m=whole[1 - axis],
                        yaw_deg=math.degrees(whole[2]), lag_s=start_lag(track, t0, t1 + TAIL_S, axis)))
    return out


def main(argv: list[str]) -> int:
    if len(argv) < 3 or len(argv) % 3:
        print(__doc__)
        return 2
    rows = []
    for i in range(0, len(argv), 3):
        rows += segments(Path(argv[i]), Path(argv[i + 1]), float(argv[i + 2]))
    print("軸    指令[m/s]  送った[m]  進んだ[m]  割合   落ち着いた速さ[m/s]（割合）  動き始め[s]  横へ[m]  向き[°]")
    for r in sorted(rows, key=lambda r: (r["axis"], abs(r["cmd"]), r["cmd"])):
        print(f"{r['axis']}  {r['cmd']:+.2f}      {r['sent_m']:+.2f}      {r['moved_m']:+.3f}    {100 * r['ratio']:4.0f}%  "
              f"{r['steady_mps']:+.3f}（{100 * r['steady_mps'] / r['cmd']:4.0f}%）          {r['lag_s']:.2f}        "
              f"{r['off_axis_m']:+.3f}   {r['yaw_deg']:+.1f}")
    return 0


if __name__ == "__main__":
    sys.exit(main(sys.argv[1:]))
