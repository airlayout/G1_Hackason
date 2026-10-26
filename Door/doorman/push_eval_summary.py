"""押す間の扉の値の出どころを替えた評価（E1〜E3、10-20）を条件ごとにまとめる。

条件ごとの通り抜けの成功率（count_success_cutoff.py の summary_cutoff.json）と、区間ごとに
「生徒に実際に渡した値」と本当の値の差（vision の記録の used・pose・angle）を出す。

使い方: python push_eval_summary.py <名前>=<条件の記録の場所> [...]（場所は summary_cutoff.json と vision/ がある所）
"""
import glob
import json
import math
import sys

import numpy as np

STAGES = {0: "歩く", 1: "構える", 2: "手を掛ける", 3: "下げる", 4: "押す", 5: "通る"}


def wrap(a: np.ndarray) -> np.ndarray:
    return (a + np.pi) % (2 * np.pi) - np.pi


def load_vision(path: str) -> dict:
    files = sorted(glob.glob(f"{path}/vision/shard_*.npz"))
    if not files:
        return {}
    keys = ("pose", "angle", "stage", "est", "used")
    out: dict = {}
    for f in files:
        d = np.load(f)
        for k in keys:
            if k in d.files:
                out.setdefault(k, []).append(d[k])
    return {k: np.concatenate(v) for k, v in out.items()}


def errors(v: dict, key: str, stage: int):
    if key not in v:
        return None
    m = v["stage"] == stage
    if m.sum() == 0:
        return None
    truth = np.concatenate([v["pose"][m], v["angle"][m, None]], axis=1)
    e = np.abs(v[key][m] - truth)
    e[:, 2] = np.abs(wrap(v[key][m, 2] - truth[:, 2]))
    e[:, 3] = np.abs(wrap(v[key][m, 3] - truth[:, 3]))
    return np.median(e, axis=0), np.percentile(e, 90, axis=0), int(m.sum())


def fmt_err(r) -> str:
    if r is None:
        return "—"
    med, p90, _ = r
    return (f"{100 * med[0]:.1f}/{100 * med[1]:.1f} cm {math.degrees(med[2]):.1f}° 角度 {math.degrees(med[3]):.1f}°"
            f"（1 割: {100 * p90[0]:.1f} cm・{math.degrees(p90[3]):.1f}°）")


def main(pairs: list[tuple[str, str]]) -> list[dict]:
    rows = []
    for name, path in pairs:
        s = json.load(open(f"{path}/summary_cutoff.json"))
        v = load_vision(path)
        lo, hi = s["ci95"]
        print(f"== {name}: 通り抜け {s['success']}/{s['episodes']}（{100 * s['rate']:.1f}%、95% の幅 {100 * lo:.0f}〜{100 * hi:.0f}%）"
              f" 失敗 {s['fail_kinds']}")
        for st in (3, 4, 5):
            print(f"   区間 {st} {STAGES[st]}: 渡した値 {fmt_err(errors(v, 'used', st))} ／ 推定器 {fmt_err(errors(v, 'est', st))}")
        rows.append(dict(name=name, success=s["success"], episodes=s["episodes"], rate=s["rate"], ci95=s["ci95"],
                         fail_kinds=s["fail_kinds"]))
    return rows


if __name__ == "__main__":
    pairs = [tuple(a.split("=", 1)) for a in sys.argv[1:] if "=" in a and not a.startswith("--")]
    out = main(pairs)
    if "--json" in sys.argv:
        json.dump(out, open(sys.argv[sys.argv.index("--json") + 1], "w"), ensure_ascii=False, indent=1)
