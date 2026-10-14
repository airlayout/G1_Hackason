"""10-14 の段 2-4a（把手が左＋鏡写し）で、80.5%（反復 40,600・窓の報酬・立った姿勢から）に至った理由を分ける。

1. 失敗の型の分解（評価ごと）
2. 同じ台どうしの入れ替わり（窓の報酬の効き・出発の姿勢の効き・左の扉で学習した効き）。どの評価も乱数の種 0・256 台なので扉は同じ
3. 等級別・出だしの重さ別の通り抜け
4. 成功した回の扉の最大角（窓 70〜85° に止められているか）と回の長さ

使い方（OMEN の ~/work_inoue/doorman で）: ~/miniforge3/envs/doorman6/bin/python analyze_s24L_1014.py
"""

import glob
import json
import math

import numpy as np
import torch

from gr00t.rl.isaac_utils.playground.env_rand.door_closer import closing_moment

EVALS = {  # 名前: (記録, 説明)
    "L0_data": ("records/start_1014_0916", "39,150・データの姿勢から"),
    "L0_stand": ("records/start_1014_1602", "39,150・立った姿勢から"),
    "A4_data": ("records/start_1014_1129", "A 最良 41,100・データ"),
    "A4_stand": ("records/start_1014_1610", "A 最良 41,100・立った"),
    "B3_data": ("records/start_1014_1308", "B 最良 40,600・データ"),
    "B3_stand": ("records/start_1014_1606", "B 最良 40,600・立った"),
}
CATS = ["成功", "すぐ打ち切り", "レバーまで行けない", "レバーを下げたのに 10° 未満", "10〜70° で止まる", "70〜89° で抜けられない", "90° まで押し切り"]


def load(rec: str) -> dict:
    per = json.load(open(f"{rec}/summary.json"))["per_env"]
    s = np.load(f"{rec}/static.npz", allow_pickle=True)
    chunks = [np.load(f) for f in sorted(glob.glob(f"{rec}/states_*.npz"))]
    ep = np.concatenate([c["episode_length"] for c in chunks])
    env = np.array([e["env"] for e in per])
    lens = []
    for i in env:
        e = ep[:, i]
        t = [k for k in range(1, len(e)) if e[k] < e[k - 1]]
        lens.append(t[0] if t else len(e))
    a = np.array([e["max_hinge_deg"] for e in per])
    hd = np.array([e["max_handle_deg"] for e in per])
    ok = np.array([e["success"] for e in per])
    ln = np.array(lens)
    cat = np.full(len(per), -1)
    cat[ok] = 0
    f = ~ok
    cat[f & (ln < 60)] = 1
    rest = f & (ln >= 60)
    cat[rest & (a < 10) & (hd < 40)] = 2
    cat[rest & (a < 10) & (hd >= 40)] = 3
    cat[rest & (a >= 10) & (a < 70)] = 4
    cat[rest & (a >= 70) & (a < 89.5)] = 5
    cat[rest & (a >= 89.5)] = 6
    p = {n: torch.tensor(s[f"door_closer_{n}"][env], dtype=torch.float64) for n in ("m0", "ratio_90", "efficiency", "preload")}
    need = (closing_moment(torch.full_like(p["m0"], math.radians(3.0)), p["m0"], p["ratio_90"], p["preload"]) / p["efficiency"]).numpy()
    grade = np.digitize(s["door_hinge_max_force"][env], [13, 18]) + 1
    return {"env": env, "cat": cat, "ok": ok, "a": a, "len": ln, "need": need, "grade": grade, "m0": s["door_closer_m0"][env]}


def wilson(k: int, n: int) -> str:
    z = 1.96
    p = k / n
    d = 1 + z * z / n
    c = (p + z * z / (2 * n)) / d
    h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return f"{100 * (c - h):.1f}〜{100 * (c + h):.1f}%"


def main() -> None:
    D = {k: load(v[0]) for k, v in EVALS.items()}
    print("== 1. 失敗の型（256 回中の回数）")
    print("評価".ljust(26) + "".join(c[:8].rjust(10) for c in CATS) + "    95% の幅")
    for k, d in D.items():
        cnt = [int(np.sum(d["cat"] == i)) for i in range(len(CATS))]
        print(EVALS[k][1].ljust(26) + "".join(f"{c:10d}" for c in cnt) + f"    {wilson(cnt[0], len(d['ok']))}")
    same = all(np.allclose(D["L0_data"]["m0"], d["m0"]) and np.array_equal(D["L0_data"]["env"], d["env"]) for d in D.values())
    print("扉が全部の評価で同じ:", same)

    print("\n== 2. 同じ台どうしの入れ替わり（前 → 後）")
    for a_, b_, label in (("L0_data", "A4_data", "左の扉で学習（②のみ）"), ("A4_data", "B3_data", "窓の報酬（①）"),
                          ("B3_data", "B3_stand", "立った姿勢から（③）"), ("L0_data", "B3_stand", "全部まとめて")):
        x, y = D[a_], D[b_]
        gain = (~x["ok"]) & y["ok"]
        loss = x["ok"] & (~y["ok"])
        print(f"{label}: {EVALS[a_][1]} {x['ok'].sum()} → {EVALS[b_][1]} {y['ok'].sum()}（増 {gain.sum()}・減 {loss.sum()}・差 {gain.sum() - loss.sum():+d}）")
        print("   増えた台の前の失敗の型:", {CATS[i]: int(np.sum(gain & (x["cat"] == i))) for i in range(1, len(CATS)) if np.sum(gain & (x["cat"] == i))})
        print("   減った台の後の失敗の型:", {CATS[i]: int(np.sum(loss & (y["cat"] == i))) for i in range(1, len(CATS)) if np.sum(loss & (y["cat"] == i))})

    print("\n== 3. 等級別・出だしの重さ別の通り抜け")
    for k in ("L0_data", "A4_data", "B3_data", "B3_stand"):
        d = D[k]
        g = [f"{int(np.sum(d['ok'] & (d['grade'] == i)))}/{int(np.sum(d['grade'] == i))}" for i in (1, 2, 3)]
        bins = [(0, 24), (24, 30), (30, 36), (36, 40), (40, 60)]
        b = [f"{int(np.sum(d['ok'] & (d['need'] >= lo) & (d['need'] < hi)))}/{int(np.sum((d['need'] >= lo) & (d['need'] < hi)))}" for lo, hi in bins]
        print(f"{EVALS[k][1]:26s} 等級 1・2・3: {' '.join(g)}   出だし <24・24-30・30-36・36-40・40+ N·m: {' '.join(b)}")

    print("\n== 4. 成功した回の扉の最大角と回の長さ")
    for k in ("L0_data", "A4_data", "B3_data", "B3_stand"):
        d = D[k]
        s = d["ok"]
        a = d["a"][s]
        print(f"{EVALS[k][1]:26s} 最大角の中央値 {np.median(a):.1f}°  70〜85° {np.mean((a >= 70) & (a < 85)):.0%}  85° 以上 {np.mean(a >= 85):.0%}  回の長さの中央値 {np.median(d['len'][s]) * 0.02:.1f} s")


if __name__ == "__main__":
    main()
