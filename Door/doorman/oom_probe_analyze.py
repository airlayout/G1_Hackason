"""oom_probe_1022.py の記録（gpu.csv・iters.log）を、学習の 1 回ごとの「集める段」「学習の段」に切り分けて GPU の山を数える。

学習の 1 回の表は、その回の学習の段が終わった直後に書かれる。表の時刻 t・その回の長さ T（Iteration time）・集める段の長さ c
（Computation の Collection）から、集める段は [t−T, t−T+c]、学習の段は [t−T+c, t] とみなす（表の行は 0.2 秒おきに読むので ±0.2 秒）。

見るもの:
  - 学習の段の山の分布（256 MiB 刻み）。山が「1 区切りぶん（ResNet18・32 枚で約 1.1 GiB）」おきに固まっていれば、
    1 台の 32 ステップに回の区切りが何回あるかで山が段々に決まっている
  - 集める段の山（学習の段と比べて、どちらが上限に近いか）
  - 起動からの回数と山の関係（起動直後だけ高い＝cuDNN の作業領域の試し、だんだん上がる＝漏れ）

使い方: python oom_probe_analyze.py <gpu.csv> <iters.log> [--limit-mib 15830]
"""
from __future__ import annotations

import argparse
import bisect
import re
from collections import Counter

RE_ITER = re.compile(r"Learning iteration (\d+)")
RE_COLL = re.compile(r"Collection: ([\d.]+)s")
RE_TIME = re.compile(r"Iteration time: ([\d.]+)s")


def load_gpu(path: str) -> tuple[list[float], list[int]]:
    ts, mb = [], []
    for line in open(path):
        try:
            t, m = line.strip().split(",")
            ts.append(float(t))
            mb.append(int(m))
        except ValueError:
            continue
    return ts, mb


def load_iters(path: str) -> list[dict]:
    """表ごとに {it, t, coll, dur, file} をまとめる。落ちた行（OutOfMemory）は {oom: t}"""
    rows, cur, fname = [], None, None
    for line in open(path, errors="replace"):
        parts = line.split(" ", 1)
        if len(parts) < 2:
            continue
        t, text = float(parts[0]), parts[1]
        if text.startswith("FILE"):
            fname = text.split(" ", 1)[1].strip()
            continue
        if "OutOfMemory" in text:
            rows.append({"oom": t, "file": fname})
        m = RE_ITER.search(text)
        if m:
            if cur and "dur" in cur:
                rows.append(cur)
            cur = {"it": int(m.group(1)), "t": t, "file": fname}
        m = RE_COLL.search(text)
        if m and cur is not None:
            cur["coll"] = float(m.group(1))
        m = RE_TIME.search(text)
        if m and cur is not None:
            cur["dur"] = float(m.group(1))
    if cur and "dur" in cur:
        rows.append(cur)
    return rows


def window_max(ts: list[float], mb: list[int], a: float, b: float) -> int | None:
    i, j = bisect.bisect_left(ts, a), bisect.bisect_right(ts, b)
    return max(mb[i:j]) if j > i else None


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("gpu")
    ap.add_argument("iters")
    ap.add_argument("--limit-mib", type=int, default=15830, help="落ちたときの使用量（15.46 GiB）")
    a = ap.parse_args()
    ts, mb = load_gpu(a.gpu)
    rows = load_iters(a.iters)
    its = [r for r in rows if "it" in r and "coll" in r]
    learn, coll = [], []
    for r in its:
        start = r["t"] - r["dur"]
        mid = start + r["coll"]
        lm, cm = window_max(ts, mb, mid, r["t"] + 0.3), window_max(ts, mb, start, mid)
        if lm is not None and cm is not None:
            learn.append((r["it"], lm))
            coll.append(cm)
    if not learn:
        raise SystemExit("突き合わせられる回が無い（記録が短い）")
    print(f"回数 {len(learn)}（{learn[0][0]}〜{learn[-1][0]}）・GPU の記録 {len(ts)} 点・落ちた回 {sum(1 for r in rows if 'oom' in r)}")
    vals = sorted(v for _, v in learn)
    q = lambda p: vals[min(len(vals) - 1, int(p * len(vals)))]  # noqa: E731
    print(f"学習の段の山 [MiB]: 最小 {vals[0]}・中央 {q(0.5)}・90% {q(0.9)}・99% {q(0.99)}・最大 {vals[-1]}（上限の目安 {a.limit_mib}）")
    cv = sorted(coll)
    print(f"集める段の山 [MiB]: 中央 {cv[len(cv) // 2]}・最大 {cv[-1]}")
    hist = Counter((v // 256) * 256 for v in vals)
    print("学習の段の山の分布（256 MiB 刻み、回数）:")
    for k in sorted(hist):
        print(f"  {k:6d}〜 {'█' * min(60, hist[k])} {hist[k]}")
    top = sorted(learn, key=lambda x: -x[1])[:10]
    print("山の高い回: " + "・".join(f"{i}:{v}" for i, v in top))


if __name__ == "__main__":
    main()
