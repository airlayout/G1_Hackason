"""評価の記録から、扉の強さ（クローザーの上限）ごとの成功率を出す（段 2-3: 開けられる扉の強さの限界）。

80% の合否は 1 点で決まり、等級の範囲（例 等級 1 は 9〜13 N·m）の中のどこまで開けられるかが分からない。
そこで台ごとのクローザーの上限（static.npz の door_hinge_max_force、パッチ doorman6/0033 から記録）と、
台ごとの成否（count_success.py の summary.json の per_env）を突き合わせ、
上限の区切りごとの成功率と、成功率が 50%・80% になる上限（ロジスティックの当てはめ）を出す。

使い方:
    python door_force_curve.py <記録のディレクトリ>... [--bins 9,10,11,12,13] [--json out.json]
記録を複数渡すと、まとめて数える（同じ方策を別の等級の扉で評価した記録など）。
"""

import argparse
import json
import sys

import numpy as np

LEVER_RADIUS_M = (0.75, 0.9)  # 蝶番からレバーの押す点まで [m]（扉の幅 0.8〜1.0 m − バックセット）。力の目安に使う


# 成功した台と失敗した台で平均を比べる扉の値（static.npz の名前・表示名・単位）
COMPARE_KEYS = (
    ("door_hinge_max_force", "クローザーの上限", "N·m"),
    ("door_hinge_stiffness", "クローザーのばね", "N·m/deg"),
    ("door_width", "扉の幅", "m"),
    ("door_weight", "扉の重さ", "kg"),
)


def load(record_dir: str) -> tuple[dict[str, np.ndarray], np.ndarray]:
    """({扉の値の名前: 台ごとの値}, 成否) を返す。最初の区間から始めて終わった回だけ（summary.json の per_env）"""
    static = np.load(f"{record_dir}/static.npz", allow_pickle=True)
    if "door_hinge_max_force" not in static.files:
        raise KeyError(f"door_hinge_max_force が無い（0033 より前の記録）: {record_dir}")
    per_env = json.load(open(f"{record_dir}/summary.json", encoding="utf-8"))["per_env"]
    env_ids = np.array([e["env"] for e in per_env], dtype=int)
    success = np.array([e["success"] for e in per_env], dtype=float)
    values = {key: static[key][env_ids] for key, _, _ in COMPARE_KEYS if key in static.files}
    return values, success


def fit_logistic(x: np.ndarray, y: np.ndarray) -> tuple[float, float]:
    """成功率 = 1 / (1 + exp(k (x − x50))) を最尤で当てはめ、(x50, k) を返す（scipy を使わず格子で探す）"""
    best = (float("nan"), float("nan"), -np.inf)
    for x50 in np.linspace(x.min() - 5.0, x.max() + 5.0, 401):
        for k in np.geomspace(0.05, 20.0, 120):
            p = np.clip(1.0 / (1.0 + np.exp(np.clip(k * (x - x50), -50.0, 50.0))), 1e-6, 1 - 1e-6)
            ll = float(np.sum(y * np.log(p) + (1 - y) * np.log(1 - p)))
            if ll > best[2]:
                best = (float(x50), float(k), ll)
    return best[0], best[1]


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record_dirs", nargs="+")
    ap.add_argument("--bins", help="上限の区切り [N·m]（, 区切り）。省くと 1 N·m 刻み")
    ap.add_argument("--json")
    args = ap.parse_args()
    try:
        pairs = [load(d) for d in args.record_dirs]
    except (FileNotFoundError, KeyError) as err:
        print(f"記録を読めない: {err}", file=sys.stderr)
        return 2
    values = {key: np.concatenate([p[0][key] for p in pairs]) for key in pairs[0][0]}
    force = values["door_hinge_max_force"]
    success = np.concatenate([p[1] for p in pairs])
    edges = (np.array([float(v) for v in args.bins.split(",")]) if args.bins
             else np.arange(np.floor(force.min()), np.ceil(force.max()) + 1.0, 1.0))
    print(f"台数 {len(force)}・成功 {int(success.sum())}（{success.mean():.1%}）・クローザーの上限 {force.min():.1f}〜{force.max():.1f} N·m")
    rows = []
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (force >= lo) & (force < hi) if hi < edges[-1] else (force >= lo) & (force <= hi)
        n = int(m.sum())
        rate = float(success[m].mean()) if n else float("nan")
        rows.append({"lo": float(lo), "hi": float(hi), "n": n, "success": int(success[m].sum()), "rate": rate})
        print(f"  {lo:5.1f}〜{hi:5.1f} N·m: {int(success[m].sum()):3d}/{n:3d}" + (f" = {rate:.0%}" if n else ""))
    out = {"n": int(len(force)), "success": int(success.sum()), "bins": rows}
    ok = success > 0.5
    if 0 < ok.sum() < len(ok):
        print("成功した台・失敗した台の平均:")
        for key, label, unit in COMPARE_KEYS:
            if key in values:
                v = values[key]
                print(f"  {label}: 成功 {v[ok].mean():.2f}・失敗 {v[~ok].mean():.2f} {unit}")
        x50, k = fit_logistic(force, success)
        if force.min() <= x50 <= force.max():
            x80 = x50 - np.log(4.0) / k  # 成功率 80% になる上限（強い扉ほど下がる向き）
            lever = [x50 / r for r in LEVER_RADIUS_M[::-1]]
            print(f"成功率 50% の上限 ≈ {x50:.1f} N·m（レバーの位置で約 {lever[0]:.0f}〜{lever[1]:.0f} N）・80% ≈ {x80:.1f} N·m")
            out.update({"x50_nm": x50, "x80_nm": float(x80), "slope": k})
        else:
            print(f"成功率はクローザーの上限では決まらない（当てはめの 50% の点 {x50:.1f} N·m が範囲の外）")
    if args.json:
        with open(args.json, "w", encoding="utf-8") as f:
            json.dump(out, f, ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
