"""段 2.5（2026-10-15 夕）: DAgger で作った状態入力の生徒を、強化学習（PPO）で磨く出発の重みを組み立てる。

  actor  = 生徒（StateRecurrentActorObjPred）の LSTM と MLP を、教師の形（RecurrentActor: actor_module）に名前を直して入れる。
           腕の関節のトルク arm_dof_torque（14）の入力を、重み 0 で差し込む（並びは観測の名前の辞書順。最初の動きは生徒と同じ）
  critic = 教師の評価係（critic_obs は教師と同じなので、そのまま使える）
  std    = 教師が最後に学んだ値 × --std-scale（探索の揺らぎ。評価は平均の行動なので効かない）
  env の状態（報酬の段階の値など）と学習の状態は教師のものを使う。最適化の状態は入れない（学習率は exp の値で始まる）

最後に、生徒の元の重みと組み立てた重みで同じ入力を流し、行動が一致することを確かめる（トルクの列には乱数を入れる）。

使い方（OMEN の ~/work_inoue/doorman で、doorman6 の python）:
  python make_state_ppo_ckpt.py <生徒の last.pt> <教師の last.pt> <出力の .pt> [--std-scale 0.5]
"""

import argparse

import torch
import torch.nn as nn

# 生徒の actor_obs（obs/wbmanip/door_open_homie_29dof_state_dagger.yaml、0048 より前）と、PPO の actor_obs（_state_ppo）。
# 観測は名前の辞書順につながる（legged_robot_base._post_config_observation_callback）
STUDENT_TERMS = {
    "actions": 29, "b_homie_commands": 7, "base_ang_vel": 3, "complete": 1, "delta_actions": 17, "dof_pos": 29,
    "dof_vel": 29, "door_angle_lidar": 3, "door_frame_loc": 9, "door_geometry": 7, "projected_gravity": 3,
}
NEW_TERMS = {"arm_dof_torque": 14}


def layout(terms: dict) -> dict:
    out, start = {}, 0
    for name in sorted(terms):
        out[name] = (start, start + terms[name])
        start += terms[name]
    return out


def expand_columns(old: torch.Tensor, old_layout: dict, new_layout: dict, fill: float, dim: int) -> torch.Tensor:
    """old の dim 方向を new_layout の並びに並べ直す。新しい項目は fill で埋める"""
    n_new = max(e for _, e in new_layout.values())
    shape = list(old.shape)
    shape[dim] = n_new
    out = torch.full(shape, fill, dtype=old.dtype)
    for name, (s, e) in new_layout.items():
        if name in old_layout:
            os_, oe = old_layout[name]
            out.narrow(dim, s, e - s).copy_(old.narrow(dim, os_, oe - os_))
    return out


def forward(policy: dict, x: torch.Tensor, mlp_prefix: str) -> torch.Tensor:
    """RunningMeanStd → LSTM 2 層 → MLP（SiLU）を手で流す（1 ステップ、隠れ状態 0 から 3 ステップ）"""
    mean, var = policy["running_mean_std.running_mean"], policy["running_mean_std.running_var"]
    y = torch.clamp((x - mean) / torch.sqrt(var + 1e-5), -5.0, 5.0)
    in_dim = policy["memory.rnn.weight_ih_l0"].shape[1]
    lstm = nn.LSTM(in_dim, 256, num_layers=2)
    lstm.load_state_dict({k[len("memory.rnn."):]: v for k, v in policy.items() if k.startswith("memory.rnn.")})
    out, _ = lstm(y)  # [T, N, 256]
    h = out
    for i in (0, 2, 4, 6):
        h = h @ policy[f"{mlp_prefix}.module.{i}.weight"].T + policy[f"{mlp_prefix}.module.{i}.bias"]
        if i != 6:
            h = torch.nn.functional.silu(h)
    return h


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("student")
    ap.add_argument("teacher")
    ap.add_argument("out")
    ap.add_argument("--std-scale", type=float, default=0.5)
    args = ap.parse_args()

    student = torch.load(args.student, map_location="cpu", weights_only=False)
    teacher = torch.load(args.teacher, map_location="cpu", weights_only=False)
    sp = student["policy_state_dict"]
    old_layout = layout(STUDENT_TERMS)
    new_layout = layout({**STUDENT_TERMS, **NEW_TERMS})
    assert sp["memory.rnn.weight_ih_l0"].shape[1] == max(e for _, e in old_layout.values()), "生徒の入力の次元が違う"

    policy = {}
    for k, v in sp.items():
        if k.startswith("obj_pred_mlp."):
            continue  # 物の位置の当ては PPO では使わない
        if k.startswith("mlp_module."):
            policy["actor_module." + k[len("mlp_module."):]] = v.clone()
        else:
            policy[k] = v.clone()
    policy["memory.rnn.weight_ih_l0"] = expand_columns(sp["memory.rnn.weight_ih_l0"], old_layout, new_layout, 0.0, 1)
    policy["running_mean_std.running_mean"] = expand_columns(sp["running_mean_std.running_mean"], old_layout, new_layout, 0.0, 0)
    policy["running_mean_std.running_var"] = expand_columns(sp["running_mean_std.running_var"], old_layout, new_layout, 1.0, 0)
    policy["std"] = teacher["policy_state_dict"]["std"].clone() * args.std_scale

    # 確かめ: 同じ入力（新しい列は乱数）で、生徒と組み立てた重みの行動が一致する
    torch.manual_seed(0)
    x_old = torch.randn(3, 8, max(e for _, e in old_layout.values())) * 2.0
    x_new = torch.randn(3, 8, max(e for _, e in new_layout.values())) * 2.0
    for name, (s, e) in new_layout.items():
        if name in old_layout:
            os_, oe = old_layout[name]
            x_new[..., s:e] = x_old[..., os_:oe]
    a_old = forward(sp, x_old, "mlp_module")
    a_new = forward(policy, x_new, "actor_module")
    diff = (a_old - a_new).abs().max().item()
    print(f"行動の差の最大 {diff:.2e}（生徒 {tuple(x_old.shape)} → 組み立て {tuple(x_new.shape)}）")
    assert diff < 1e-5, "組み立てた重みの行動が生徒と一致しない"

    out = {
        "policy_state_dict": policy,
        "value_state_dict": teacher["value_state_dict"],
        "optimizer_state_dict": None,
        "lr_scheduler_state_dict": None,
        "state": teacher["state"],
        "env_state_dict": teacher.get("env_state_dict", {}),
    }
    torch.save(out, args.out)
    print(f"書いた: {args.out}（actor＝{args.student} の反復 {student['state'].global_step}、critic・env＝教師、"
          f"std＝教師 × {args.std_scale}: {[round(v, 3) for v in policy['std'][:5].tolist()]}…）")


if __name__ == "__main__":
    main()
