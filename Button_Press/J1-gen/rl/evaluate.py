"""学習した方策を、決まった配置（seed）で評価する。画面で動きを見ることもできる。

    P=_local/rl/venv/bin/python
    $P Button_Press/J1-gen/rl/evaluate.py _local/rl/runs/<記録>/model_latest.zip
    $P Button_Press/J1-gen/rl/evaluate.py _local/rl/runs/<記録>/model_latest.zip --episodes 200 --out result.json
    $P Button_Press/J1-gen/rl/evaluate.py _local/rl/runs/<記録>/model_latest.zip --episodes 3 --view

配置は seed 10000 から順に引く（学習中の配置とは別。毎回同じ配置なので、方策どうしを比べられる）。
方策は決定的に動かす（学習中のような、行動にばらつきを足す試し方はしない）。設定は記録の config.yaml を使う。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path
from typing import Any

import numpy as np
import yaml

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from rl.press_env import PressEnv  # noqa: E402

EVAL_SEED0 = 10000


def load_policy(model_path: Path) -> tuple[Any, Any, dict[str, Any]]:
    """(方策, 観測の正規化, 設定)。正規化は model_<tag>.zip と同じ場所の vecnormalize_<tag>.pkl。"""
    import pickle

    from stable_baselines3 import PPO

    model = PPO.load(str(model_path), device="cpu")
    vn_path = model_path.with_name(model_path.name.replace("model_", "vecnormalize_")).with_suffix(".pkl")
    with open(vn_path, "rb") as f:
        vecnorm = pickle.load(f)
    cfg = yaml.safe_load((model_path.parent / "config.yaml").read_text(encoding="utf-8"))
    return model, vecnorm, cfg


def run_episode(env: PressEnv, model: Any, vecnorm: Any, seed: int, viewer: Any = None) -> dict[str, Any]:
    obs, reset_info = env.reset(seed=seed)
    max_force, info, t0 = 0.0, {}, time.monotonic()
    best_ct = float("inf")  # 押している間の、接触点から指先までの最短距離
    while True:
        action, _ = model.predict(vecnorm.normalize_obs(obs[None]), deterministic=True)
        obs, _, terminated, truncated, info = env.step(action[0])
        max_force = max(max_force, info["force"])
        best_ct = min(best_ct, info["contact_tip"])
        if viewer is not None:
            viewer.sync()
            rest = env._steps / env.cfg["control_hz"] - (time.monotonic() - t0)
            if rest > 0:
                time.sleep(rest)
        if terminated or truncated:
            break
    return {"seed": seed, "outcome": info["outcome"], "time_s": env._steps / env.cfg["control_hz"],
            "max_force_n": round(max_force, 2), "lateral_mm": round(info["lateral"] * 1000, 1),
            "dist_mm": round(info["dist"] * 1000, 1), "hit": info["hit"],
            "contact_tip_mm": round(best_ct * 1000, 1) if best_ct != float("inf") else None,
            "reach_x_mm": round(info["reach_x"] * 1000, 1),
            "layout": {k: round(float(v), 4) for k, v in reset_info["layout"].items()}}


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("model", type=Path, help="model_*.zip")
    ap.add_argument("--episodes", type=int, default=50)
    ap.add_argument("--view", action="store_true", help="MuJoCo の画面で、実時間で動きを見る")
    ap.add_argument("--out", type=Path, help="結果（配置ごと）を JSON で保存する")
    args = ap.parse_args()

    model, vecnorm, cfg = load_policy(args.model)
    cfg.setdefault("scene", {}).setdefault("edge_focus", {})["prob"] = 0.0  # 評価は配置を偏らせない
    env = PressEnv(cfg)
    viewer = None
    if args.view:
        import mujoco.viewer

        viewer = mujoco.viewer.launch_passive(env.model, env.data)
        cam = viewer.cam
        cam.lookat[:] = env.data.body("pelvis").xpos + np.array([0.3, -0.1, -0.05])
        cam.distance, cam.azimuth, cam.elevation = 1.6, 210.0, -25.0
    rows = []
    for k in range(args.episodes):
        r = run_episode(env, model, vecnorm, EVAL_SEED0 + k, viewer)
        rows.append(r)
        lay = ", ".join(f"{n} {v:+.3f}" for n, v in r["layout"].items())
        print(f"[evaluate] {k:3d} {r['outcome']:9s} {r['time_s']:4.1f} 秒、押す力の最大 {r['max_force_n']:5.1f} N、"
              f"横 {r['lateral_mm']:5.1f} mm、押す点まで {r['dist_mm']:5.1f} mm、接触点-指先 {r['contact_tip_mm']} mm、面まで {r['reach_x_mm']} mm、当たった {r['hit'] or '-'}（{lay}）")
    if viewer is not None:
        viewer.close()
    n = len(rows)
    counts = {o: sum(r["outcome"] == o for r in rows) for o in ("success", "collision", "overforce", "timeout")}
    ok = [r for r in rows if r["outcome"] == "success"]
    print(f"[evaluate] {n} 回: " + "、".join(f"{o} {c}（{100 * c / n:.0f}%）" for o, c in counts.items()))
    if ok:
        print(f"[evaluate] 成功したときの 時間 平均 {np.mean([r['time_s'] for r in ok]):.1f} 秒、押す力の最大 "
              f"{max(r['max_force_n'] for r in ok):.1f} N、横のずれ 最大 {max(r['lateral_mm'] for r in ok):.1f} mm")
    if args.out:
        args.out.write_text(json.dumps({"model": str(args.model), "counts": counts, "rows": rows}, indent=2,
                                       ensure_ascii=False), encoding="utf-8")
    return 0


if __name__ == "__main__":
    sys.exit(main())
