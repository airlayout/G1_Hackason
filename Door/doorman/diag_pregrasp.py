"""区間 1（手を構える）で、区間 2 へ進む条件のどの項目が満たせていないかを記録から数える。

記録は eval_from_start.sh に EXTRA_ARGS=++env.config.record_states.pregrasp_diag=True を付けて取る。
各 env の最初の 1 回のうち、区間 1 にいたステップだけを見る。条件は door_open_homie.py の
_stage_1_to_2_advance_condition と同じ（手のひらは door_open_lr で選ばれた側の手）:
  near   手のひらが構える点（既定は把手の 10 cm 上。pregrasp_height で変わり、記録はその点）から 0.1 m 以内
  above  手のひらが把手の高さ +0.05 m より上
  orient 手の向きの報酬 > 0.2（角度の誤差がおよそ 62° 未満）
  still  歩く指令の大きさ <= 0.1
（扉が 10° 開いても進めるが、評価では 1 回も開いていない）

使い方:
    python diag_pregrasp.py <記録のディレクトリ>
"""

import argparse
import glob
import sys

import numpy as np

STAGE_PREGRASP = 1
NEAR_M, ABOVE_M, ORIENT_MIN, STILL_MAX = 0.1, 0.05, 0.2, 0.1
KEYS = ("stage", "episode_length", "door_open_lr", "palm_to_pregrasp", "palm_above_handle",
        "hand_handle_orientation", "homie_cmd_norm", "cond_1_to_2", "palm_used", "pregrasp_target")
OPTIONAL_KEYS = ("hand_handle_contact_n",)  # 0013 の記録係から
CONTACT_N = 1.0  # 区間 2 の「握った」と同じ力のしきい値


def load(record_dir: str) -> dict:
    files = sorted(glob.glob(f"{record_dir}/states_*.npz"))
    if not files:
        raise FileNotFoundError(f"states_*.npz が無い: {record_dir}")
    chunks = [np.load(f) for f in files]
    missing = [k for k in KEYS if k not in chunks[0]]
    if missing:
        raise KeyError(f"pregrasp_diag の項目が無い（{missing}）。pregrasp_diag=True で記録し直す")
    keys = KEYS + tuple(k for k in OPTIONAL_KEYS if k in chunks[0])
    return {k: np.concatenate([c[k] for c in chunks]) for k in keys}


def first_episode_mask(ep_len: np.ndarray) -> np.ndarray:
    """各 env の最初の 1 回にあたるステップを True にする。"""
    reset = np.vstack([np.zeros((1, ep_len.shape[1]), bool), ep_len[1:] < ep_len[:-1]])
    return np.cumsum(reset, axis=0) == 0


def percentiles(values: list[float]) -> str:
    if not values:
        return "（無し）"
    p = np.percentile(values, [10, 50, 90])
    return f"10%={p[0]:.3f} 中央={p[1]:.3f} 90%={p[2]:.3f}"


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record_dir")
    ap.add_argument("--above-margin", default="0.05",
                    help="区間 1→2 の「把手より上」の余裕 [m]。none で見ない（pregrasp_above_margin: null の設定）")
    args = ap.parse_args()
    try:
        r = load(args.record_dir)
    except (FileNotFoundError, KeyError) as err:
        print(err, file=sys.stderr)
        return 2
    in_stage1 = (r["stage"] == STAGE_PREGRASP) & first_episode_mask(r["episode_length"])
    near = r["palm_to_pregrasp"] < NEAR_M
    above = (np.ones_like(r["palm_above_handle"], dtype=bool) if args.above_margin.lower() == "none"
             else r["palm_above_handle"] > float(args.above_margin))
    orient = r["hand_handle_orientation"] > ORIENT_MIN
    still = r["homie_cmd_norm"] <= STILL_MAX
    envs = [e for e in range(in_stage1.shape[1]) if in_stage1[:, e].any()]
    print(f"区間 1 に入った env: {len(envs)}/{in_stage1.shape[1]}（区間 1 にいたステップ {int(in_stage1.sum())}）")
    if not envs:
        print("区間 1 に入った env が無いので、条件の項目は数えられない（評価が途中で落ちていないか eval.log を見る）")
        return 2

    def ever(mask: np.ndarray) -> int:
        return sum(bool((mask[:, e] & in_stage1[:, e]).any()) for e in envs)

    n = len(envs)
    print("区間 1 の間に一度でも満たした env:")
    for name, mask in (("near", near), ("above", above), ("orient", orient), ("still", still),
                       ("near&above", near & above), ("near&above&orient", near & above & orient),
                       ("全部（near&above&orient&still）", near & above & orient & still),
                       ("cond_1_to_2（環境の判定そのもの）", r["cond_1_to_2"] > 0.5)):
        print(f"  {name}: {ever(mask)}/{n}")

    def per_env(values: np.ndarray, reduce) -> list[float]:
        return [float(reduce(values[in_stage1[:, e], e])) for e in envs]

    print("env ごとの区間 1 での値:")
    print(f"  手のひら→目標点の最小距離 [m]（0.1 未満が要る）: {percentiles(per_env(r['palm_to_pregrasp'], np.min))}")
    print(f"  手のひらの把手からの高さの最大 [m]（0.05 超が要る）: {percentiles(per_env(r['palm_above_handle'], np.max))}")
    print(f"  手の向きの報酬の最大（0.2 超が要る）: {percentiles(per_env(r['hand_handle_orientation'], np.max))}")
    print(f"  歩く指令の大きさの最小（0.1 以下が要る）: {percentiles(per_env(r['homie_cmd_norm'], np.min))}")
    lr = [float(r["door_open_lr"][in_stage1[:, e], e][0]) for e in envs]
    print(f"  door_open_lr（<0 は左手を使う）: 負 {sum(v < 0 for v in lr)} / 正 {sum(v > 0 for v in lr)}")
    # 最も近づいた瞬間に、手のひらが目標点からどちらへずれているか（env の原点の座標系）
    gaps = []
    for e in envs:
        idx = np.flatnonzero(in_stage1[:, e])
        t = idx[np.argmin(r["palm_to_pregrasp"][idx, e])]
        gaps.append(r["palm_used"][t, e] - r["pregrasp_target"][t, e])
    g = np.array(gaps)
    if "hand_handle_contact_n" in r:
        touch = r["hand_handle_contact_n"] > CONTACT_N
        print(f"  区間 1 の間に使う手が把手に {CONTACT_N} N 超で触れた env: {ever(touch)}/{n}"
              f"（触れていたステップの割合 {float((touch & in_stage1).sum() / max(in_stage1.sum(), 1)):.1%}）")
        print(f"  触れたまま手のひらが把手より下にいた env: {ever(touch & ~above)}/{n}")
    print(f"  最も近づいた瞬間の ずれ（手のひら−目標点, m）の中央: x={np.median(g[:, 0]):+.3f} "
          f"y={np.median(g[:, 1]):+.3f} z={np.median(g[:, 2]):+.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
