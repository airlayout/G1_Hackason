"""レバーへの当たり方を評価の記録から数える（段 5、10-19。レバーにゆっくり近づける）。

評価は ++env.config.record_states.pregrasp_diag=True で回す（手のひらの位置 palm_used を毎ステップ残す）。
数えるのは各台の最初の 1 回（区間 0 から始まり終わった回。count_success.py と同じ）。

  触れた瞬間の力     初めて触れてから 0.1 秒（5 ステップ）の、手がレバーから受ける力の最大 [N]。目安 15 N 以下を 9 割
  触れる直前の速さ   初めて触れる前の 3 ステップの、手のひらの速さの平均 [m/s]
  近くの最大の速さ   レバーの当てる点から 10 cm 以内に入ってから初めて触れるまでの、手のひらの速さの最大 [m/s]
  下げきるまで       初めて触れてからレバーが 40° まで下がるまで [s]

10-19 夜に分け直した（0.1 秒の窓には性質の違う山が 2 つ混ざっていた。元の方策でレバーは 4〜8 ステップで止まりの 45° に着き、
着いてからも押すので 36〜61 N が出る。動かしている間は 6〜13 N で足りる）:
  当たりの山         初めて触れた 2 ステップ（0.04 秒）の力の最大 [N]。手がレバーにぶつかる強さ
  底突きの力         触れてから 1 秒の間、レバーが止まり（44° 以上）にある間の力の最大 [N]。止まりへ押し付ける強さ
  動かす間の力       触れてから 1 秒の間、レバーが 1〜44° を動いている間の力の中央 [N]。レバーを下げるのに使った力
  力の上がり方       2 ステップ目から 0.2 秒の、力の増え方の最大 [N/s]（1 ステップ目は当たりの山そのものなので除く）
  15 N 超えの内訳    0.1 秒の窓が 15 N を超えた回を「当たり（当たりの山が超えた）・底突き・押し（それ以外）」に分ける

力の取り方（--force。10-19 夜）: 制御の 1 ステップ（0.02 秒）は物理 4 回で、hand_lever_force は 4 回目の後の写し（snapshot）。
ぶつかった瞬間が 1〜3 回目だと残らない。lever_force_substeps=True で評価した記録には、物理 1 回ごとに読んだ力の
最大（peak）と平均（mean）がある。既定は peak があれば peak、無ければ snapshot

使い方: python touch_metrics.py <記録のディレクトリ> [--json out.json] [--force snapshot|peak|mean]
"""
import argparse
import glob
import json
import sys

import numpy as np

from count_success import LAST_STAGE, THROUGH_X_M

DT = 0.02
TOUCH_N = 1.0
IMPACT_STEPS = 5
IMPACT_MAX_N = 15.0
NEAR_M = 0.10
FIRST_STEPS = 2
LEVER_STOP_DEG = 44.0  # 学習の扉のレバーは 45° で止まる（10-19 の記録の最大 45.00°）
CONTACT_STEPS = 50  # 1 秒
RISE_STEPS = 10  # 0.2 秒


OPTIONAL_KEYS = ("hand_lever_force_peak", "hand_lever_force_mean")


def load(d: str) -> dict:
    files = sorted(glob.glob(f"{d}/states_*.npz"))
    if not files:
        raise FileNotFoundError(f"states_*.npz が無い: {d}")
    keys = ("stage", "episode_length", "root_pos", "door_dof_pos", "hand_lever_force", "palm_used", "grasp_target")
    chunks = [np.load(f, allow_pickle=True) for f in files]
    missing = [k for k in keys if k not in chunks[0].files]
    if missing:
        raise KeyError(f"記録に {missing} が無い（pregrasp_diag=True で評価する）")
    keys += tuple(k for k in OPTIONAL_KEYS if k in chunks[0].files)
    r = {k: np.concatenate([c[k] for c in chunks]) for k in keys}
    # 10-19 18:00 の試運転の記録は、最初の 1 行だけ peak・mean が欠けている（物理を回す前）。前を 0 で埋めて揃える
    for k in OPTIONAL_KEYS:
        if k in r and len(r[k]) == len(r["stage"]) - 1:
            r[k] = np.concatenate([np.zeros_like(r[k][:1]), r[k]])
    return r


def lever_force(r: dict, how: str) -> np.ndarray:
    """(ステップ, 台) の力の大きさ [N]"""
    if how == "auto":
        how = "peak" if "hand_lever_force_peak" in r else "snapshot"
    if how == "snapshot":
        return np.linalg.norm(r["hand_lever_force"], axis=-1)
    key = f"hand_lever_force_{how}"
    if key not in r:
        raise KeyError(f"記録に {key} が無い（lever_force_substeps=True で評価する）")
    return r[key]


def episodes(r: dict, how: str = "auto") -> list:
    stage, ep = r["stage"], r["episode_length"]
    force = lever_force(r, how)
    palm = r["palm_used"]
    speed = np.zeros(force.shape)
    speed[1:] = np.linalg.norm(np.diff(palm, axis=0), axis=-1) / DT
    dist = np.linalg.norm(palm - r["grasp_target"], axis=-1)
    lever = np.degrees(r["door_dof_pos"][..., 1])
    out = []
    for e in range(stage.shape[1]):
        ends = np.nonzero(ep[1:, e] < ep[:-1, e])[0]
        n = int(ends[0]) + 1 if len(ends) else stage.shape[0]
        st = stage[:n, e]
        success = bool(np.any((st == LAST_STAGE) & (r["root_pos"][:n, e, 0] > THROUGH_X_M)))
        finished = len(ends) > 0 or success
        if not finished or st[0] != 0:
            continue
        touch = np.nonzero(force[:n, e] > TOUCH_N)[0]
        item = {"env": e, "success": success, "touched": bool(len(touch))}
        if len(touch):
            k = int(touch[0])
            near = np.nonzero(dist[1:k + 1, e] < NEAR_M)[0]
            item.update(
                impact_n=float(force[k:k + IMPACT_STEPS, e].max()),
                speed_before=float(speed[max(k - 3, 1):k, e].mean()) if k > 1 else float("nan"),
                speed_near_max=float(speed[1 + near[0]:k + 1, e].max()) if len(near) else float("nan"),
                press_s=float(np.argmax(lever[k:n, e] >= 40.0) * DT) if (lever[k:n, e] >= 40.0).any() else float("nan"),
                hold_force_n=float(np.median(force[touch, e])),
                **split_contact(force[k:min(k + CONTACT_STEPS, n), e], lever[k:min(k + CONTACT_STEPS, n), e]),
            )
        out.append(item)
    return out


def split_contact(f: np.ndarray, lev: np.ndarray) -> dict:
    """初めて触れたステップから始まる力 f [N] とレバーの角度 lev [°] を、当たり・動かす間・底突きに分ける"""
    on = f > TOUCH_N
    moving = on & (lev > 1.0) & (lev < LEVER_STOP_DEG)
    bottom = on & (lev >= LEVER_STOP_DEG)
    rise = np.diff(f[:RISE_STEPS + 1])[1:] / DT if len(f) > 2 else np.array([])
    first_peak = float(f[:FIRST_STEPS].max())
    window = f[:IMPACT_STEPS]
    if window.max() <= IMPACT_MAX_N:
        cause = "なし"
    elif first_peak > IMPACT_MAX_N:
        cause = "当たり"
    elif np.any((window > IMPACT_MAX_N) & (lev[:IMPACT_STEPS] >= LEVER_STOP_DEG)):
        cause = "底突き"
    else:
        cause = "押し"
    return {
        "first_peak_n": first_peak,
        "moving_n": float(np.median(f[moving])) if moving.any() else float("nan"),
        "bottom_n": float(f[bottom].max()) if bottom.any() else float("nan"),
        "rise_n_per_s": float(rise.max()) if len(rise) else float("nan"),
        "reached_stop": bool(bottom.any()),
        "over15_cause": cause,
    }


def summarize(eps: list) -> dict:
    n = len(eps)
    t = [e for e in eps if e["touched"]]
    arr = lambda k: np.array([e[k] for e in t if not np.isnan(e[k])])  # noqa: E731
    q = lambda x: {"中央": round(float(np.median(x)), 2), "9割": round(float(np.percentile(x, 90)), 2)} if len(x) else {}  # noqa: E731
    imp = arr("impact_n")
    return {
        "回": n,
        "成功率": round(sum(e["success"] for e in eps) / max(n, 1), 3),
        "触れた回": len(t),
        "触れた瞬間の力が15N以下の割合": round(float(np.mean(imp <= IMPACT_MAX_N)), 3) if len(imp) else None,
        "触れた瞬間の力[N]": q(imp),
        "触れる直前の速さ[m/s]": q(arr("speed_before")),
        "近くの最大の速さ[m/s]": q(arr("speed_near_max")),
        "下げきるまで[s]": q(arr("press_s")),
        "押している力[N]": q(arr("hold_force_n")),
        "当たりの山が15N以下の割合": round(float(np.mean(arr("first_peak_n") <= IMPACT_MAX_N)), 3) if t else None,
        "当たりの山[N]": q(arr("first_peak_n")),
        "動かす間の力[N]": q(arr("moving_n")),
        "1秒以内に止まりへ着いた割合": round(sum(e["reached_stop"] for e in t) / max(len(t), 1), 3),
        "底突きの力[N]": q(arr("bottom_n")),
        "力の上がり方[N/s]": q(arr("rise_n_per_s")),
        "15N超えの内訳": {c: sum(e["over15_cause"] == c for e in t) for c in ("当たり", "底突き", "押し", "なし")},
    }


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record_dir")
    ap.add_argument("--json")
    ap.add_argument("--force", default="auto", choices=("auto", "snapshot", "peak", "mean"))
    a = ap.parse_args()
    eps = episodes(load(a.record_dir), a.force)
    s = summarize(eps)
    print(json.dumps(s, ensure_ascii=False))
    if a.json:
        json.dump({"summary": s, "per_episode": eps}, open(a.json, "w"), ensure_ascii=False, indent=1)
    return 0


if __name__ == "__main__":
    sys.exit(main())
