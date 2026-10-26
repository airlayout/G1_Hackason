"""段 1（2026-10-21）: 録った 1 本を、脚の odometry とリモコンの記録だけで素早く見る。止まっている区間を切り出し、区間ごとの位置と向きを並べる。

止まっている区間 = odometry の速さが STILL_SPEED 未満・角速度が STILL_YAW_RATE 未満で、MIN_STILL_S 以上続き、その間リモコンに触れていない所。
ここがライダーで正解を取る区間になる（odometry だけで決めるので、odometry が嘘をつけば区間も狂う。最後はライダーで確かめる）。

使い方: Navigation/.venv/bin/python Door/real/take_quicklook.py <odom のセッション> [<名前>_remote.log]
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from dog_odom_bag import OdomTrack, load_odom  # noqa: E402

STILL_SPEED = 0.03  # [m/s]
STILL_YAW_RATE = math.radians(2.0)  # [rad/s]
MIN_STILL_S = 2.0
SMOOTH_S = 0.5  # 速さを出す窓


def body_frame(track: OdomTrack) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
    """最初の姿勢から見た 前・左 [m] と 向き [°]"""
    yaw0 = track.yaw[0]
    c, s = math.cos(yaw0), math.sin(yaw0)
    d = track.xyz[:, :2] - track.xyz[0, :2]
    return c * d[:, 0] + s * d[:, 1], -s * d[:, 0] + c * d[:, 1], np.degrees(track.yaw - yaw0)


def remote_events(path: Path | None, t0: float) -> list[tuple[float, bool]]:
    """(時刻, 触れているか) の並び"""
    if path is None or not path.exists():
        return []
    out = []
    for line in path.read_text().splitlines():
        if "keys=" not in line:
            continue
        p = line.split()
        vals = dict(kv.split("=") for kv in p[2:])
        touched = int(vals["keys"], 16) != 0 or any(abs(float(vals[k])) > 0.02 for k in ("lx", "ly", "rx", "ry"))
        out.append((int(p[1]) / 1e9 - t0, touched))
    return out


def still_segments(track: OdomTrack, events: list[tuple[float, bool]]) -> list[tuple[float, float]]:
    t = track.recv_s - track.recv_s[0]
    k = max(1, int(SMOOTH_S * 1000))
    sl = slice(k, None)
    dt = t[k:] - t[:-k]
    v = np.hypot(*(track.xyz[k:, :2] - track.xyz[:-k, :2]).T) / dt
    w = np.abs(track.yaw[k:] - track.yaw[:-k]) / dt
    still = np.zeros(len(t), bool)
    still[sl] = (v < STILL_SPEED) & (w < STILL_YAW_RATE)
    still[:k] = still[k]
    touched = np.zeros(len(t), bool)
    for i, (te, on) in enumerate(events):
        t_end = events[i + 1][0] if i + 1 < len(events) else t[-1]
        if on:
            touched[(t >= te) & (t <= t_end)] = True
    ok = still & ~touched
    segs, start = [], None
    for i, flag in enumerate(ok):
        if flag and start is None:
            start = i
        if (not flag or i == len(ok) - 1) and start is not None:
            end = i if not flag else i
            if t[end] - t[start] >= MIN_STILL_S:
                segs.append((float(t[start]), float(t[end])))
            start = None
    return segs


def main() -> None:
    odom_dir = Path(sys.argv[1])
    remote = Path(sys.argv[2]) if len(sys.argv) > 2 else None
    tr = load_odom(odom_dir)
    t = tr.recv_s - tr.recv_s[0]
    fx, fy, yaw = body_frame(tr)
    events = remote_events(remote, tr.recv_s[0])
    segs = still_segments(tr, events)
    print(f"== {odom_dir.name}: {t[-1]:.1f} 秒・止まっている区間 {len(segs)} 個（{MIN_STILL_S:.0f} 秒以上・リモコンに触れていない）")
    prev = None
    for a, b in segs:
        m = (t >= a) & (t <= b)
        p = np.array([fx[m].mean(), fy[m].mean(), yaw[m].mean()])
        line = f"  {a:6.1f}〜{b:6.1f} s（{b - a:4.1f} 秒）  前 {p[0]:+.3f}  左 {p[1]:+.3f} m  向き {p[2]:+6.1f}°"
        if prev is not None:
            d = p - prev
            line += f"   前の区間から: 距離 {math.hypot(d[0], d[1]):.3f} m・回転 {d[2]:+.1f}°"
        print(line)
        prev = p
    moving = [te for te, on in events if on]
    if moving:
        print(f"  リモコンに触れていた: {moving[0]:.1f}〜{events[-1][0]:.1f} s のあいだ（{sum(on for _, on in events)} 回の変化）")


if __name__ == "__main__":
    main()
