"""DAgger の学習の記録（train.log）を表にする。映像の生徒が真似を覚えているかを、学習の途中で見るため。

  真似の誤差      dagger_bc_loss（教師の動きとの差。下がり続ければ真似を覚えている）
  位置の誤差      obj_pred_loss（画像から扉の位置を当てる補助の誤差）
  届いた区間      Env/average_stage_reached（終わった回が平均でどこまで進んだか。0 扉へ歩く … 5 通り抜ける）
  回の長さ        Mean length（ステップ。50 で 1 秒。転ぶと短い）

--ref に別の学習の記録を渡すと、集めた歩数（Total timesteps）が同じところの値を横に並べる
（台数が違う学習は、回数でなく歩数でそろえないと比べられない。16 台は 512 台の 1/32 しか 1 回で集めない）。

使い方（OMEN）: python3 dagger_progress.py <train.log> [--every 200] [--ref <比べる train.log>]
"""
from __future__ import annotations

import argparse
import bisect
import re
from pathlib import Path

ANSI = re.compile(r"\x1b\[[0-9;]*m")
KEYS = {
    "steps": "Total timesteps",
    "bc": "dagger_bc_loss",
    "obj": "obj_pred_loss",
    "stage": "Env/average_stage_reached",
    "length": "Mean length",
}


def parse(path: Path) -> list[dict]:
    """1 回の学習ごとに {it, steps, bc, obj, stage, length} を返す（weighted_ の行は読まない）"""
    rows, cur = [], None
    for line in ANSI.sub("", path.read_text(errors="replace")).splitlines():
        m = re.search(r"Learning iteration (\d+)", line)
        if m:
            if cur:
                rows.append(cur)
            cur = {"it": int(m.group(1))}
            continue
        if cur is None:
            continue
        for k, name in KEYS.items():
            m = re.search(rf"(?<!weighted_){re.escape(name)}: *([-\d.eE+]+)", line)
            if m:
                cur[k] = float(m.group(1))
    if cur:
        rows.append(cur)
    return [r for r in rows if "steps" in r]


def at_steps(ref: list[dict], steps: float) -> dict | None:
    """ref の中で、集めた歩数が steps に一番近い回（steps を越えない最後の回）"""
    xs = [r["steps"] for r in ref]
    i = bisect.bisect_right(xs, steps) - 1
    return ref[i] if i >= 0 else None


def fmt(r: dict | None, k: str, spec: str) -> str:
    return "—" if r is None or k not in r else format(r[k], spec)


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("log", type=Path)
    ap.add_argument("--every", type=int, default=200)
    ap.add_argument("--ref", type=Path, default=None)
    a = ap.parse_args()
    rows = parse(a.log)
    if not rows:
        raise SystemExit("学習の回が 1 つも読めない（まだ始まっていないか、記録の形が違う）")
    ref = parse(a.ref) if a.ref else None
    pick = [r for r in rows if r["it"] % a.every == 0]
    if pick[-1:] != rows[-1:]:
        pick.append(rows[-1])
    head = "| 回 | 歩数（万） | 真似の誤差 | 位置の誤差 | 届いた区間 | 回の長さ（秒） |"
    sep = "|---|---|---|---|---|---|"
    if ref:
        head += " 比べる元：同じ歩数の回 | 真似の誤差 | 届いた区間 |"
        sep += "---|---|---|"
    print(head)
    print(sep)
    for r in pick:
        line = (f"| {r['it']} | {r['steps'] / 1e4:.1f} | {fmt(r, 'bc', '.2f')} | {fmt(r, 'obj', '.3f')} "
                f"| {fmt(r, 'stage', '.2f')} | {r.get('length', 0) / 50:.1f} |")
        if ref:
            q = at_steps(ref, r["steps"])
            line += f" {q['it'] if q else '—'} | {fmt(q, 'bc', '.2f')} | {fmt(q, 'stage', '.2f')} |"
        print(line)
    # 1 回ずつの値は振れるので、最後の 50 回の平均も出す
    tail = rows[-50:]
    mean = lambda k: sum(r[k] for r in tail if k in r) / max(1, sum(1 for r in tail if k in r))  # noqa: E731
    print(f"最後の {len(tail)} 回の平均: 真似の誤差 {mean('bc'):.2f}・位置の誤差 {mean('obj'):.3f}・"
          f"届いた区間 {mean('stage'):.2f}・回の長さ {mean('length') / 50:.1f} 秒")


if __name__ == "__main__":
    main()
