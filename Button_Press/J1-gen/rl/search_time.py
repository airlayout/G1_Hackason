"""押す点の近くに入ってから、押し終わるまでの時間（探す時間）を、評価と同じ 50 配置で測る。

    ~/rl_env/bin/python Button_Press/J1-gen/rl/search_time.py <model_latest.zip> <出力.json>

開始 → 5 cm 以内、5 cm 以内 → 成功（探す時間）、2 cm 以内 → 成功、触れる → 成功 の中央値・平均・最大を出す。
"""

import json
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from rl.evaluate import load_policy, EVAL_SEED0
from rl.press_env import PressEnv

model, vn, cfg = load_policy(Path(sys.argv[1]))
cfg.setdefault("scene", {}).setdefault("edge_focus", {})["prob"] = 0.0
env = PressEnv(cfg); hz = cfg["control_hz"]
rows = []
for k in range(50):
    seed = EVAL_SEED0 + k
    obs, ri = env.reset(seed=seed)
    d0 = None; t5 = t2 = tc = None; t = 0
    while True:
        a, _ = model.predict(vn.normalize_obs(obs[None]), deterministic=True)
        obs, _, term, trunc, info = env.step(a[0]); t += 1
        if d0 is None: d0 = info["dist"]
        if t5 is None and info["dist"] < 0.05: t5 = t
        if t2 is None and info["dist"] < 0.02: t2 = t
        if tc is None and info["force"] > 0.5: tc = t
        if term or trunc: break
    rows.append(dict(seed=seed, out=info["outcome"], end=t / hz, t5=None if t5 is None else t5 / hz,
                     t2=None if t2 is None else t2 / hz, tc=None if tc is None else tc / hz, d0=d0,
                     lay=ri["layout"]))
json.dump(rows, open(sys.argv[2], "w"), default=float, ensure_ascii=False)
ok = [r for r in rows if r["out"] == "success"]
f = lambda xs: f"中央値 {np.median(xs):.2f}、平均 {np.mean(xs):.2f}、最大 {np.max(xs):.2f}" if xs else "-"
print("成功", len(ok), "/ 50")
print("開始→5cm以内:", f([r["t5"] for r in ok if r["t5"] is not None]))
print("5cm以内→成功（探す時間）:", f([r["end"] - r["t5"] for r in ok if r["t5"] is not None]))
print("2cm以内→成功:", f([r["end"] - r["t2"] for r in ok if r["t2"] is not None]))
print("接触→成功:", f([r["end"] - r["tc"] for r in ok if r["tc"] is not None]))
print("全体の時間:", f([r["end"] for r in ok]))
long = [r for r in ok if r["t5"] is not None and r["end"] - r["t5"] > 1.0]
print("成功のうち探す時間が 1 秒超:", len(long), "件")
fl = [r for r in rows if r["out"] != "success"]
print("失敗", len(fl), "件: 5cm以内に入った", sum(r["t5"] is not None for r in fl), "件、入ってから終了までの秒:",
      [(r["seed"], r["out"], None if r["t5"] is None else round(r["end"] - r["t5"], 1)) for r in fl])
