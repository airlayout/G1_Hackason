"""段 5（10-19）: 強化学習で磨いた状態入力の生徒（PPO の ckpt）に、観測の項目を重み 0 で差し込む。

今は「手のひらからレバーまでの位置の差」palm_lever_rel（3）。観測は名前の辞書順につながるので、door_geometry と
projected_gravity の間（148 列目）に入る。LSTM の入力の重み・正規化の平均（0）と分散（1）の列を差し込み、critic・std・
学習の状態はそのまま。最適化の状態は入れない（列の数が変わるため）。

最後に、元の重みと差し込んだ重みに同じ入力を流し（新しい列には乱数）、行動が一致することを確かめる。

使い方（OMEN の ~/work_inoue/doorman で、doorman6 の python）:
  python add_obs_term_ckpt.py <PPO の last.pt> <出力の .pt> [--term palm_lever_rel --dim 3]
"""

import argparse

import torch

from make_state_ppo_ckpt import STUDENT_TERMS, expand_columns, forward, layout

PPO_TERMS = {**STUDENT_TERMS, "arm_dof_torque": 14}  # obs/wbmanip/door_open_homie_29dof_state_ppo.yaml


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("src")
    ap.add_argument("out")
    ap.add_argument("--term", default="palm_lever_rel")
    ap.add_argument("--dim", type=int, default=3)
    args = ap.parse_args()

    ck = torch.load(args.src, map_location="cpu", weights_only=False)
    sp = ck["policy_state_dict"]
    old_layout = layout(PPO_TERMS)
    new_layout = layout({**PPO_TERMS, args.term: args.dim})
    n_old = max(e for _, e in old_layout.values())
    assert sp["memory.rnn.weight_ih_l0"].shape[1] == n_old, f"入力の次元が {sp['memory.rnn.weight_ih_l0'].shape[1]}（期待 {n_old}）"

    policy = {k: v.clone() for k, v in sp.items()}
    policy["memory.rnn.weight_ih_l0"] = expand_columns(sp["memory.rnn.weight_ih_l0"], old_layout, new_layout, 0.0, 1)
    policy["running_mean_std.running_mean"] = expand_columns(sp["running_mean_std.running_mean"], old_layout, new_layout, 0.0, 0)
    policy["running_mean_std.running_var"] = expand_columns(sp["running_mean_std.running_var"], old_layout, new_layout, 1.0, 0)

    torch.manual_seed(0)
    x_old = torch.randn(3, 8, n_old) * 2.0
    x_new = torch.randn(3, 8, max(e for _, e in new_layout.values())) * 2.0
    for name, (s, e) in new_layout.items():
        if name in old_layout:
            os_, oe = old_layout[name]
            x_new[..., s:e] = x_old[..., os_:oe]
    diff = (forward(sp, x_old, "actor_module") - forward(policy, x_new, "actor_module")).abs().max().item()
    print(f"行動の差の最大 {diff:.2e}（{n_old} → {x_new.shape[-1]} 列、{args.term} は {new_layout[args.term]}）")
    assert diff < 1e-5, "差し込んだ重みの行動が元と一致しない"

    out = dict(ck)
    out["policy_state_dict"] = policy
    out["optimizer_state_dict"] = None
    out["lr_scheduler_state_dict"] = None
    torch.save(out, args.out)
    print(f"書いた: {args.out}")


if __name__ == "__main__":
    main()
