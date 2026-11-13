"""11-13 夜: 手首を下げてレバーに掛ける形（hook_*）の有無をシムで比べる。python3 s0_hook_cmp_1113.py <jsonl> [<jsonl> ...]
s0_chain_hook_1113.sh の結果（JSON Lines と、TRACE=32 の回ごとの毎周期 .trace_env*_ep*.npy）から、1 本ずつ:
  ① 何台が 2d・3a・扉 3°・3c・扉 10° まで行ったか・止まり方
  ② 3a・3c の間の手とレバーの当たり（接触センサ）: 触れている割合・前へ押す力・レバーを押さえる力・離れた回数・レバーの角度
  ③ 手の力の見積もり（手の先で解く）と接触センサの差（掛けた形でも合うか）
⚠️ シムの脚（HOMIE）は押し返されても実機の純正のようには下がらない。3c で扉が開くかはシムでは分からない（計画 16.3 の 5）
"""
import argparse
import collections
import glob
import json
import math
import statistics as stt

import numpy as np

# script_teacher.py（パッチ 0111）の trace の列
T, SUB, PALM, POSE, DOOR, F_EST, F_TRUE = 0, 1, slice(3, 6), slice(23, 26), slice(27, 29), slice(29, 32), slice(32, 35)
SUBS = ("2a", "2b", "2c", "2d", "3a", "3c", "done", "retreat", "failed")
TOUCH_N = 2.0       # 接触センサの力がこれより大きければ触れている [N]
LOST_S = 0.3        # 触れていた手がこれより長く離れたら「離れた」1 回


def med(xs):
    xs = [x for x in xs if x is not None and math.isfinite(x)]
    return stt.median(xs) if xs else float("nan")


def trace_stats(path: str) -> dict:
    a = np.load(path)
    out = {}
    # 3a から先（3a・3c・done）の本当の扉の角度（結果の door_max_deg は回の始めの置き直しで跳ねた値が混ざる・11-13 夜）
    late = np.isin(a[:, SUB], [SUBS.index("3a"), SUBS.index("3c"), SUBS.index("done")])
    out["door_late"] = float(np.degrees(np.max(np.abs(a[late, DOOR][:, 0])))) if late.any() else None
    for name in ("3a", "3c"):
        m = a[:, SUB] == SUBS.index(name)
        if m.sum() < 10:
            continue
        ft, fe = a[m, F_TRUE], a[m, F_EST]
        touch = np.linalg.norm(ft, axis=1) > TOUCH_N
        dt = float(np.median(np.diff(a[m, T]))) if m.sum() > 1 else 0.02
        lost, gap, was = 0, 0.0, False
        for on in touch:
            if on:
                if was and gap > LOST_S:
                    lost += 1
                gap, was = 0.0, True
            else:
                gap += dt
        if was and gap > LOST_S:
            lost += 1
        ok = np.isfinite(fe).all(axis=1) & touch
        out[name] = dict(
            secs=float(m.sum() * dt), touch=float(touch.mean()), lost=lost,
            push=float(np.median(-ft[touch, 0])) if touch.any() else float("nan"),     # 前へ押す力（手が受ける力の逆）
            press=float(np.median(ft[touch, 2])) if touch.any() else float("nan"),     # レバーを押さえる力（手が受ける上向き）
            lever=float(np.degrees(np.median(np.abs(a[m, DOOR][:, 1])))),
            pressed=float(np.mean(np.degrees(np.abs(a[m, DOOR][:, 1])) > 20.0)),   # レバーが 20° より下がっている割合
            door=float(np.degrees(np.max(np.abs(a[m, DOOR][:, 0])))),
            err_x=float(np.median(np.abs(fe[ok, 0] - ft[ok, 0]))) if ok.sum() > 5 else float("nan"),
            err_z=float(np.median(np.abs(fe[ok, 2] - ft[ok, 2]))) if ok.sum() > 5 else float("nan"),
        )
    return out


def one(path: str, hmin: float) -> None:
    rows = [json.loads(line) for line in open(path) if line.strip()]
    use = [r for r in rows if r.get("lever_height", 1.0) >= hmin]
    subs = [set(s for s, _ in r.get("subs", [])) for r in use]
    print(f"\n=== {path.split('/')[-1]}  回 {len(use)}（レバー {hmin} m 以上）")
    print(f"  2d {sum('2d' in s for s in subs)}・3a {sum('3a' in s for s in subs)}・扉 3° {sum(r['door_max_deg'] >= 3 for r in use)}・"
          f"3c {sum('3c' in s for s in subs)}・扉 10° {sum(r['door_max_deg'] >= 10 for r in use)}・30° {sum(bool(r.get('success')) for r in use)}")
    stops = collections.Counter((r.get("stop") or "（時間切れ）").split("（")[0][:40] for r in use)
    print("  止まり方: " + "・".join(f"{k} {v}" for k, v in stops.most_common(6)))
    per = collections.defaultdict(list)
    late = []
    for tp in sorted(glob.glob(path + ".trace_env*_ep*.npy")):
        for name, d in trace_stats(tp).items():
            if name == "door_late":
                if d is not None:
                    late.append(d)
            else:
                per[name].append(d)
    if late:
        print(f"  毎周期の記録で 3a 以降の本当の扉（{len(late)} 回）: 3° 以上 {sum(x >= 3 for x in late)}・10° 以上 {sum(x >= 10 for x in late)}・"
              f"最大の中央 {med(late):.1f}°")
    for name in ("3a", "3c"):
        d = per.get(name, [])
        if not d:
            print(f"  {name}: 記録なし")
            continue
        g = lambda k: med([x[k] for x in d])  # noqa: E731
        print(f"  {name}（{len(d)} 回・中央）: 長さ {g('secs'):.1f} 秒・触れている割合 {100 * g('touch'):.0f}%・離れた {g('lost'):.0f} 回・"
              f"前へ押す {g('push'):+.1f} N・押さえる {g('press'):+.1f} N・レバー {g('lever'):.0f}°（20° より下 {100 * g('pressed'):.0f}%）・扉の最大 {g('door'):.1f}°・"
              f"見積もりの差 前後 {g('err_x'):.1f} 上下 {g('err_z'):.1f} N")
    for sub in ("2d", "3a", "3c"):
        hf = [r["hand_force"][sub] for r in use if sub in r.get("hand_force", {})]
        if hf:
            print(f"  {sub} 手の力（回ごとの中央の中央）: 見積もり 前後 {med([x['est_med'][0] for x in hf]):+.1f}・上下 {med([x['est_med'][2] for x in hf]):+.1f}"
                  f"／接触 前後 {med([x['true_med'][0] for x in hf]):+.1f}・上下 {med([x['true_med'][2] for x in hf]):+.1f} N（{len(hf)} 回）")


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("jsonl", nargs="+")
    ap.add_argument("--hmin", type=float, default=0.95)
    a = ap.parse_args()
    for p in a.jsonl:
        one(p, a.hmin)


if __name__ == "__main__":
    main()
