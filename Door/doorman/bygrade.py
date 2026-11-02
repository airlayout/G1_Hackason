"""11-01（ユーザー「等級 2 ぐらいが実際の扉。それをベースに再検討して」）: 256 回の評価を、扉の等級（EN 1154 の閉じる力 m0:
等級 1 は 9〜13・2 は 13〜18・3 は 18〜26 N·m）ごとに分け、成功と失敗の型を数える。型の分け方は classify.py と同じ順番。
実機の扉（10-29 の遠隔操作から m0 ≈ 10〜12、幅 7〜16 N·m）に近い「m0 ≤ 16」の組も出す"""
import glob
import json
import sys

import numpy as np

LIMIT = (500, 200, 200, 200, 600, 400)
KINDS = ("成功", "70°未満で通り抜け", "持ち時間切れ", "途中の打ち切り", "扉が90°", "倒れ")


def kind_of(r: dict, summ: dict, e: int, T: int) -> str:
    if summ[e]["success"]:
        return "成功"
    ep = r["episode_length"][:, e]
    cut = np.where(ep[1:] < ep[:-1])[0]
    b = int(cut[0]) + 1 if len(cut) else T
    s = r["stage"][:b, e]
    last = int(s[-1])
    chg = np.where(s[1:] != s[:-1])[0]
    t_in = b - (int(chg[-1]) + 1 if len(chg) else 0)
    if r["root_pos"][:b, e, 2].min() < 0.5:
        return "倒れ"
    if t_in >= LIMIT[last] - 2:
        return "持ち時間切れ"
    if np.degrees(r["door_dof_pos"][b - 1, e, 0]) > 89.0:
        return "扉が90°"
    if last >= 4 and r["root_pos"][b - 1, e, 0] > 1.3:
        return "70°未満で通り抜け"
    return "途中の打ち切り"


for rec in sys.argv[1:]:
    files = sorted(glob.glob(f"{rec}/states_*.npz"))
    r = {k: np.concatenate([np.load(f)[k] for f in files]) for k in ("stage", "episode_length", "root_pos", "door_dof_pos")}
    st = np.load(f"{rec}/static.npz")
    m0 = st["door_closer_m0"]
    summ = {e["env"]: e for e in json.load(open(f"{rec}/summary.json"))["per_env"]}
    T, E = r["stage"].shape
    kinds = np.array([kind_of(r, summ, e, T) for e in range(E)])
    groups = {"等級1": m0 < 13.0, "等級2": (m0 >= 13.0) & (m0 < 18.0), "等級3": m0 >= 18.0, "実機に近い m0≤16": m0 <= 16.0, "全部": np.ones(E, bool)}
    print(f"== {rec.rstrip('/').split('/')[-1]}")
    for name, g in groups.items():
        n = int(g.sum())
        c = {k: int((kinds[g] == k).sum()) for k in KINDS}
        rest = "・".join(f"{k} {v}" for k, v in c.items() if k != "成功" and v)
        print(f"  {name:10s} {c['成功']:3d}/{n:3d} = {c['成功'] / max(n, 1):5.1%}  失敗: {rest}")
