"""実物の形のクローザー（0037）が Isaac Sim の中で本当に効いているかを、評価の記録から確かめる（計器の確かめ）。

1. 閉じた扉: 機体が触れる前（最初の 60 ステップ＝1.2 s）にレバーが勝手に回らず、扉が閉じたままか
   （閉じた位置で強く押すとラッチの斜面を押してレバーが回る。2026-10-07・0017）
2. 開いた扉: ++env.config.randomize_door_init_state=True で 1/3 の台は扉が 15〜100° 開いて始まる。最初の 1 s の
   閉まり方を、同じ台の値（static.npz の door_closer_*・重さ・幅）で 1 自由度の扉を計算したものと比べる。
   力が掛かっていない（向きが逆・ドライブの上限で切られた）なら大きく外れる。慣性は板の m w²/3 で近似（±3 割程度の差は許す）

使い方（OMEN の ~/work_inoue/doorman で。gr00t は conda doorman6 に入っている）:
    PAUSE_FILE=... EXTRA_ARGS="++env.config.task.name=door_en1154_g3_shaped ++env.config.randomize_door_init_state=True" \
      ./eval_from_start.sh ckpt/g4_pass_it34950/last.pt 32
    ~/miniforge3/envs/doorman6/bin/python check_closer_in_sim.py records/start_<日時>
"""

import glob
import math
import sys

import numpy as np
import torch

from gr00t.rl.isaac_utils.playground.env_rand.door_closer import closer_torque

PRE_TOUCH_STEPS = 60       # 機体は扉の 0.6〜1.5 m 手前から歩き出す
LEVER_SPIN_DEG = 5.0       # これより回ったら「勝手に回った」
CLOSED_DEG = 1.0
OPEN_START_DEG = 10.0
COMPARE_SECONDS = 1.0
PHYSICS_DT = 1.0 / 200.0
START_SEARCH_STEPS = 10   # 起動直後に 2 回リセットされる台がある（10-13 実測で 32 台中 5 台が 1 ステップ目に引き直し）
PARAM_NAMES = ("m0", "ratio_90", "efficiency", "preload", "close_damping", "backcheck_angle", "backcheck_damping")


def load(record: str) -> tuple[dict, np.ndarray, np.ndarray]:
    static = dict(np.load(f"{record}/static.npz", allow_pickle=True))
    files = sorted(glob.glob(f"{record}/states_*.npz"))
    if not files:
        raise FileNotFoundError(f"states_*.npz が無い: {record}")
    chunks = [np.load(f) for f in files]
    door = np.concatenate([c["door_dof_pos"] for c in chunks])  # (steps, envs, 扉の関節)
    ep_len = np.concatenate([c["episode_length"] for c in chunks])
    return static, door, ep_len


def first_episode_start(ep_len: np.ndarray, env: int) -> int:
    """最初の回が本当に始まった行。起動直後のリセットのやり直し（episode_length が 0 に戻る）の後から数える。"""
    resets = np.flatnonzero(ep_len[:START_SEARCH_STEPS, env] == 0)
    return int(resets[-1]) if len(resets) else 0


def predict(static: dict, env: int, theta0: float, steps: int, dt: float) -> np.ndarray:
    p = {n: torch.tensor([float(static[f"door_closer_{n}"][env])], dtype=torch.float64) for n in PARAM_NAMES}
    inertia = float(static["door_weight"][env]) * float(static["door_width"][env]) ** 2 / 3.0
    theta = torch.tensor([theta0], dtype=torch.float64)
    omega = torch.zeros(1, dtype=torch.float64)
    out = [theta0]
    sub = int(round(dt / PHYSICS_DT))
    for _ in range(steps - 1):
        for _ in range(sub):
            omega = omega + closer_torque(theta, omega, **p) / inertia * PHYSICS_DT
            theta = torch.clamp(theta + omega * PHYSICS_DT, min=0.0)
        out.append(theta.item())
    return np.array(out)


def main() -> int:
    if len(sys.argv) != 2:
        sys.exit(__doc__)
    static, door, ep_len = load(sys.argv[1])
    if "door_closer_shaped" not in static or not static["door_closer_shaped"].any():
        print("[check] 実物の形のクローザーの台が記録に無い（0037 の前の記録か、PD の扉）")
        return 2
    dt = float(static["dt"])
    hinge = np.degrees(np.abs(door[..., 0]))
    lever = np.degrees(door[..., 1])
    shaped = np.flatnonzero(static["door_closer_shaped"])
    ok = True

    start = {e: first_episode_start(ep_len, e) for e in shaped}
    redone = [int(e) for e in shaped if start[e] > 0]
    if redone:
        print(f"（起動直後にリセットがやり直された台 {redone} は、やり直しの後から数える）")
    closed = [e for e in shaped if hinge[start[e], e] < CLOSED_DEG]
    spun = [e for e in closed if lever[start[e]:start[e] + PRE_TOUCH_STEPS, e].max() > LEVER_SPIN_DEG]
    drift = [e for e in closed if hinge[start[e]:start[e] + PRE_TOUCH_STEPS, e].max() > CLOSED_DEG]
    print(f"1. 閉じて始まった台 {len(closed)}: 最初の {PRE_TOUCH_STEPS} ステップでレバーが {LEVER_SPIN_DEG:.0f}° 超 {len(spun)} 台・"
          f"扉が {CLOSED_DEG:.0f}° 超 {len(drift)} 台（どちらも 0 が合格）")
    ok &= not spun and not drift

    opened = [e for e in shaped if hinge[start[e], e] > OPEN_START_DEG]
    n = int(round(COMPARE_SECONDS / dt)) + 1
    if not opened:
        print("2. 開いて始まった台が無い（++env.config.randomize_door_init_state=True で回す）")
        return 0 if ok else 1
    print(f"2. 開いて始まった台 {len(opened)}: {COMPARE_SECONDS:.0f} s で閉じた角度（記録 / 1 自由度の計算）")
    ratios = []
    for e in opened:
        rec = hinge[start[e]:start[e] + n, e]
        pred = np.degrees(predict(static, e, math.radians(rec[0]), n, dt))
        moved_rec, moved_pred = rec[0] - rec[-1], pred[0] - pred[-1]
        ratio = moved_rec / moved_pred if moved_pred > 0.5 else float("nan")
        ratios.append(ratio)
        print(f"   台 {e:3d} m0 {static['door_closer_m0'][e]:5.1f} N·m {static['door_weight'][e]:5.1f} kg: "
              f"{rec[0]:5.1f}° → 記録 {rec[-1]:5.1f}° / 計算 {pred[-1]:5.1f}°（閉じた量の比 {ratio:4.2f}）")
    good = np.nanmedian(ratios)
    print(f"   閉じた量の比の中央値 {good:.2f}（0.7〜1.3 なら力が効いている。0 付近なら掛かっていない・負なら向きが逆）")
    ok &= 0.7 <= good <= 1.3
    print("合格" if ok else "不合格（評価の数字を使う前に原因を調べる）")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
