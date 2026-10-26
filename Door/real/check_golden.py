"""段 5: door_core.py が学習の環境と同じ計算をしているかを、Isaac Sim の正解の記録で確かめる。

正解の記録は VIRAL6 の golden_trace.py（パッチ 0050）で取る。観測の雑音は再現できないので切って評価する:
  EXTRA_ARGS="... ++obs.noise_scales.dof_pos=0.0 ++obs.noise_scales.dof_vel=0.0 ++obs.noise_scales.arm_dof_torque=0.0 \
    ++obs.noise_scales.e_homie_dof_pos=0.0 ++obs.noise_scales.f_homie_dof_vel=0.0 \
    ++algo.config.eval.golden_trace_path=<出力 .npz>" ./eval_from_start.sh <ckpt> 8

2 通りで見る（各台の最初の 1 回が終わるまで）:
  部品ごと（厳しく）: 前のステップの値を記録から入れて、観測の組み立て・HOMIE の 1 ステップと網が一致するか（許容 1e-5）。
                     生徒の網は記録の観測を入れて差を見る。GPU（cuDNN の LSTM）は TF32 で計算するので 1e-3 前後の差は出る
                     （2026-10-16: float64 で中央 9.9e-4、TF32 の丸めを真似ると 3.9e-4。差は時間で育たない）
  通しで（実機と同じ）: 記録の生の値（関節・IMU・扉の計測器）だけを入れ、door_core が自分の状態で回る。
                     積算 D に差が溜まるので許容は緩く（既定 0.05＝腕の目標で 0.0125 rad）。歩く／立つの切替は全ステップ一致を求める

直し B（腕の積み上げを今の角度 ±δ に収める）と 8（立つ HOMIE の指令を 0）は、記録を取った評価の設定に合わせる。
記録にその設定（golden_trace.py が 10-26 から書く zero_stand_cmd・arm_accum_guard_delta・arm_accum_guard_margin）が
無ければ、どちらも無かった頃（10-16 の golden_45150）の記録とみなして、door_core の側も切って比べる。

使い方: python check_golden.py <door_policy.npz> <golden.npz> [--tol-loop 0.05]
"""
from __future__ import annotations

import argparse
import math
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Optional, Tuple

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import door_core as dc  # noqa: E402
import target_guard as tg  # noqa: E402

CHECKS = ("obs", "a_pol", "homie_obs", "homie_a", "D", "cmd", "whole", "walk")


@dataclass(frozen=True)
class GoldenOptions:
    """記録を取った評価の設定（直し B・8 の有無）"""
    zero_stand_cmd: bool
    arm_guard: Optional[Tuple[float, float]]  # (delta, margin)。None は切らない


def golden_options(g) -> GoldenOptions:
    """記録に設定が無ければ 10-16 の評価（B も 8 も無い）とみなす"""
    zero = bool(float(g["zero_stand_cmd"]) > 0.5) if "zero_stand_cmd" in g else False
    delta = float(g["arm_accum_guard_delta"]) if "arm_accum_guard_delta" in g else math.nan
    margin = float(g["arm_accum_guard_margin"]) if "arm_accum_guard_margin" in g else math.nan
    return GoldenOptions(zero, None if math.isnan(delta) else (delta, margin))


def run_env(p: dc.Params, g, i: int, n_steps: int, opts: GoldenOptions):
    """台 i を最初の 1 回の終わりまで回し、項目ごとの差の最大と、最初に大きく外れたステップを返す"""
    s = dc.initial_state(p, first_height=0.0)  # 評価のプロセスの最初の観測は指令の高さが 0
    worst = {k: 0.0 for k in CHECKS}
    first_bad = {}
    for t in range(n_steps):
        inp = inputs_at(g, t, i)
        s, out = dc.step(p, s, inp, mode="door", zero_stand_cmd=opts.zero_stand_cmd, arm_guard=opts.arm_guard)
        if t == 0:
            # シミュレータはリセット直後の最初の観測を HOMIE の履歴に入れない（t=1 の履歴は全部 0 だった）。
            # リセットの作り物なので実機の door_core には入れず、ここでだけ真似る
            s = dc.replace(s, homie_hist=np.zeros_like(s.homie_hist))
        diffs = {
            "obs": np.abs(out.obs - g["actor_obs"][t, i]).max(),
            "a_pol": np.abs(out.a_pol - g["action_mean"][t, i]).max(),
            "homie_obs": np.abs(out.homie_obs - g["homie_obs"][t, i]).max(),
            "homie_a": np.abs(out.homie_a - g["homie_actions"][t, i]).max(),
            "walk": float(out.walk != bool(g["walk_mask"][t, i] > 0.5)),
        }
        if g["dones"][t, i] < 0.5:  # 終わったステップの後の値は環境の中でリセットされている
            diffs["D"] = np.abs(s.D - g["delta_actions_sum"][t, i]).max()
            diffs["cmd"] = np.abs(s.cmd_prev - g["homie_commands"][t, i]).max()
            diffs["whole"] = np.abs(s.whole_prev - g["whole_actions"][t, i]).max()
        for k, v in diffs.items():
            worst[k] = max(worst[k], float(v))
            if v > 1e-3 and k not in first_bad:
                first_bad[k] = t
    return worst, first_bad


def inputs_at(g, t: int, i: int) -> dc.Inputs:
    return dc.Inputs(
        q=g["dof_pos"][t, i].astype(float),
        dq=g["dof_vel"][t, i].astype(float),
        gyro=g["base_ang_vel"][t, i].astype(float),
        gravity=g["projected_gravity"][t, i].astype(float),
        door_angle=g["door_angle_lidar"][t, i].astype(float),
        door_frame_loc=g["door_frame_loc"][t, i].astype(float),
        door_geometry=g["door_geometry"][t, i].astype(float),
        complete=float(g["complete"][t, i, 0]),
    )


def check_parts(p: dc.Params, g, i: int, n_steps: int, opts: GoldenOptions) -> dict:
    """前のステップの値を記録から入れて、部品ごとに差の最大を見る"""
    worst = {"obs_build": 0.0, "homie_step": 0.0, "homie_net": 0.0, "actor_net": 0.0}
    h = np.zeros((dc.LSTM_LAYERS, dc.LSTM_HIDDEN))
    c = np.zeros((dc.LSTM_LAYERS, dc.LSTM_HIDDEN))
    s = dc.initial_state(p, first_height=0.0)
    for t in range(n_steps):
        if t > 0:
            s = dc.replace(
                s,
                whole_prev=g["whole_actions"][t - 1, i].astype(float),
                a_real_prev=p.act_mirror @ g["action_mean"][t - 1, i] if p.mirror else g["action_mean"][t - 1, i].astype(float),
                cmd_prev=g["homie_commands"][t - 1, i].astype(float),
                homie_a_prev=g["homie_actions"][t - 1, i].astype(float),
            )
        inp = inputs_at(g, t, i)
        worst["obs_build"] = max(worst["obs_build"], float(np.abs(dc.build_actor_obs(p, s, inp) - g["actor_obs"][t, i]).max()))
        one = dc.homie_one_step(p, s, inp)
        worst["homie_step"] = max(worst["homie_step"], float(np.abs(one - g["homie_obs"][t, i][-dc.HOMIE_STEP:]).max()))
        ho = g["homie_obs"][t, i].astype(float)
        if g["walk_mask"][t, i] > 0.5:
            a_homie = p.homie_walk(ho)
        else:
            a_homie = p.homie_stand(tg.zero_stand_command(ho, dc.HOMIE_HIST + 1) if opts.zero_stand_cmd else ho)
        worst["homie_net"] = max(worst["homie_net"], float(np.abs(a_homie - g["homie_actions"][t, i]).max()))
        a, h, c = dc.actor_forward(p, g["actor_obs"][t, i].astype(float), h, c)
        worst["actor_net"] = max(worst["actor_net"], float(np.abs(a - g["action_mean"][t, i]).max()))
    return worst


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("policy")
    ap.add_argument("golden")
    ap.add_argument("--tol-parts", type=float, default=1e-5)
    ap.add_argument("--tol-actor-net", type=float, default=1e-2)
    ap.add_argument("--tol-loop", type=float, default=0.05)
    a = ap.parse_args()
    p = dc.load_params(a.policy)
    with np.load(a.golden) as z:
        g = {k: z[k] for k in z.files}  # npz は添字のたびに配列を丸ごと展開するので、先に全部読む
    meta = dc.json.loads(str(np.load(a.policy)["meta_json"]))
    if list(g["dof_names"]) != list(meta["dof_names"]):
        print("[golden] ⚠️ 関節の並びが npz と正解の記録で違う")
        return 1
    if np.abs(g["default_dof_pos"] - p.q_default).max() > 1e-6:
        print("[golden] ⚠️ 既定の角度が npz と正解の記録で違う")
        return 1
    if not (g["mirror"] > 0.5).all() == p.mirror:
        print(f"[golden] ⚠️ 鏡写しの有無が違う（記録 {g['mirror'][0]}・npz {p.mirror}）")
    opts = golden_options(g)
    print(f"[golden] 記録の評価の設定: 立つ HOMIE の指令を 0 に {'する' if opts.zero_stand_cmd else 'しない'}・"
          f"腕の積み上げの切り方 {opts.arm_guard if opts.arm_guard else '無し'}（door_core も同じにして比べる）")
    n_env = g["actor_obs"].shape[1]
    total_ok = True
    for i in range(n_env):
        d = g["dones"][:, i]
        n = int(np.argmax(d > 0.5)) + 1 if (d > 0.5).any() else d.shape[0]
        parts = check_parts(p, g, i, n, opts)
        worst, first_bad = run_env(p, g, i, n, opts)
        ok_parts = max(parts["obs_build"], parts["homie_step"], parts["homie_net"]) <= a.tol_parts
        ok_net = parts["actor_net"] <= a.tol_actor_net
        ok_loop = worst["walk"] == 0.0 and all(v <= a.tol_loop for k, v in worst.items() if k != "walk")
        ok = ok_parts and ok_net and ok_loop
        total_ok &= ok
        print(
            f"[golden] 台 {i}: {n} ステップ {'OK' if ok else 'NG'}  部品 "
            + " ".join(f"{k} {v:.1e}" for k, v in parts.items())
            + "  ｜通し "
            + " ".join(f"{k} {v:.1e}" for k, v in worst.items())
        )
    print(f"[golden] {'一致した' if total_ok else '一致しない'}（許容 部品 {a.tol_parts:g}・生徒の網 {a.tol_actor_net:g}・通し {a.tol_loop:g}）")
    return 0 if total_ok else 1


if __name__ == "__main__":
    sys.exit(main())
