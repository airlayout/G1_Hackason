"""学習環境（rl/press_env.py）の確認: 速さと、成功・当たりの判定が働くか。学習の前に一度実行する。

    P=_local/rl/venv/bin/python
    $P Button_Press/J1-gen/rl/check_env.py                 # ランダムな行動と、IK で作った動きを 10 配置ずつ
    $P Button_Press/J1-gen/rl/check_env.py --episodes 30

1. ランダムな行動: 1 秒あたり何周期回せるか（学習にかかる時間の見積もりに使う）
2. IK で作った動き（方策の代わり）: 開始 → 経由の姿勢（configs/pipeline.yaml の via_pose）→ 手前 5 cm → 押す点の 1.5 cm 奥 を
   関節空間でまっすぐ結び、行動に変えて流す。成功・当たった・未完 の数と、押す力の最大を出す。
   経路の当たりは調べていない（本物の計画器 common/press_planner.py は調べて断る）ので、体の正面寄りの配置では
   「体に当たった」で終わるのが正しい。判定が働いているかを見るためのもの
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import load_config  # noqa: E402
from common.kinematics import ArmKinematics  # noqa: E402
from common.press_planner import finger_axis_for  # noqa: E402
from rl.press_env import PUSH_DIR, PressEnv  # noqa: E402

TILTS_DEG = (0, 15, 30, 45)


def random_rollouts(env: PressEnv, episodes: int) -> dict[str, int]:
    outs: dict[str, int] = {}
    n, t0 = 0, time.monotonic()
    for ep in range(episodes):
        env.reset(seed=ep)
        while True:
            _, _, te, tr, info = env.step(env.action_space.sample())
            n += 1
            if te or tr:
                outs[info["outcome"]] = outs.get(info["outcome"], 0) + 1
                break
    dt = time.monotonic() - t0
    print(f"[check_env] ランダムな行動: {episodes} 回、{n} 周期を {dt:.1f} 秒 → {n / dt:.0f} 周期/秒（1 プロセス）。終わり方 {outs}")
    return outs


def ik_goals(env: PressEnv, kin: ArmKinematics, via: np.ndarray) -> list[np.ndarray] | None:
    """経由の姿勢、手前 5 cm、押す点の 1.5 cm 奥 の右腕 7 関節。IK が解けなければ None。"""
    goals = [via]
    seed = env._start_q.copy()
    for depth in (-0.05, 0.015):
        for tilt in TILTS_DEG:
            r = kin.ik(env.target + depth * PUSH_DIR, seed, finger_axis_for(PUSH_DIR, tilt, 0))
            if r.success:
                break
        else:
            return None
        seed = seed.copy()
        seed[env.arm] = r.q[env.arm]
        goals.append(seed[env.arm].copy())
    return goals


def scripted_rollouts(env: PressEnv, episodes: int) -> dict[str, int]:
    kin = ArmKinematics(load_config("robot.yaml"), "right", load_config("press.yaml")["ik"])
    via = np.radians(load_config("pipeline.yaml")["via_pose"])
    step = float(env.cfg["action"]["max_step_rad"])
    outs: dict[str, int] = {}
    for ep in range(episodes):
        _, reset_info = env.reset(seed=ep)
        goals = ik_goals(env, kin, via)
        lay = ", ".join(f"{k} {v:+.3f}" for k, v in reset_info["layout"].items())
        if goals is None:
            outs["IK 失敗"] = outs.get("IK 失敗", 0) + 1
            print(f"[check_env] {ep:2d} IK 失敗（{lay}）")
            continue
        outcome, max_force, info = "", 0.0, {}
        for g in goals + [None]:
            for _ in range(env.max_steps):
                a = np.zeros(7) if g is None else np.clip((g - env._q_des[env.arm]) / step, -1.0, 1.0)
                _, _, te, tr, info = env.step(a)
                max_force = max(max_force, info["force"])
                if te or tr:
                    outcome = info["outcome"]
                    break
                if g is not None and np.abs(g - env._q_des[env.arm]).max() < 1e-9:
                    break
            if outcome:
                break
        outcome = outcome or "未完"
        outs[outcome] = outs.get(outcome, 0) + 1
        print(f"[check_env] {ep:2d} {outcome:9s} {env._steps:3d} 周期、押す力の最大 {max_force:5.1f} N、"
              f"横のずれ {info['lateral'] * 1000:5.1f} mm、当たった {info['hit'] or '-'}（{lay}）")
    print(f"[check_env] IK で作った動き: {outs}")
    return outs


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--episodes", type=int, default=10)
    args = ap.parse_args()
    env = PressEnv()
    random_rollouts(env, args.episodes)
    try:
        import pinocchio  # noqa: F401
    except ImportError:
        # 学習だけの PC（メンバーの PC）には pin を入れていない。学習には要らない
        print("[check_env] pin（Pinocchio）が無いので、IK で作った動きの確認は飛ばす")
        return 0
    outs = scripted_rollouts(env, args.episodes)
    return 0 if outs.get("success", 0) > 0 else 1


if __name__ == "__main__":
    sys.exit(main())
