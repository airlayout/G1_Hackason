# G1_Hackason（11-08、パッチ 0096）: 画像だけの生徒を PPO＋真似の錨＋追う段の補助の損失で磨く学習係の確かめ。Isaac Sim なしで動く。
#   1. 真似の錨の重み λ: 慣らしの間は 0・慣らしの後に start から end へ直線に下げ・その後は end
#   2. 損失の式: 真似の錨は DAgger の MSE と同じ・追う段の補助は DAgger の obj_pred の式と同じ・勾配が生徒の出力へ流れる
#   3. 学習係の損失の組み立て: 慣らしの間は評価係の損失だけ（真似と補助は足さない）・慣らしの後は PPO＋λ×真似＋係数×補助
#   4. 慣らしの間は KL で学習率を動かさない（actor が動かず KL≈0 で 1.5 倍ずつ上がり続けるのを防ぐ）
#   5. 生徒の ppo_mode の学習の段: 当て直した扉の値（last_obj_pred）が行動と同じ「台 × 歩」の並びに戻り、1 歩ずつ回した値と一致
#   6. 出発の重みの組み立て（make_vis_ppo_ckpt.build）: actor は生徒そのまま（追う段も）・std は教師 × 倍率・critic と env は教師・
#      強化学習を始めた反復を学習の状態に書く
#   7. count_success.py の passed（段を見ずに扉 70° 以上＋扉の先 1.5 m）
# 使い方（OMEN）: cd VIRAL6_vis && PYTHONPATH=. python -m pytest -q tests_g1/test_0096_ppo_distill.py
import importlib.util
import os
import sys
from types import SimpleNamespace

import numpy as np
import pytest
import torch

from gr00t.rl.trl.trainer.ppo_distill_trainer_homie_api import (
    TRLPPODistillTrainerHomieAPI,
    anchor_losses,
    bc_anchor_coef,
)
from gr00t.rl.trl.trainer.ppo_trainer_homie_api import TRLPPOTrainer

sys.path.insert(0, os.path.dirname(__file__))
from test_vision_door_student import F3SC, _make_obs, _student  # noqa: E402

COUNT_SUCCESS = os.environ.get("COUNT_SUCCESS", os.path.expanduser("~/work_inoue/doorman/count_success.py"))


def test_bc_anchor_coef_schedule():
    args = dict(warmup=50, start=1.0, end=0.1, decay_iters=2000)
    assert bc_anchor_coef(0, **args) == 0.0
    assert bc_anchor_coef(49, **args) == 0.0
    assert bc_anchor_coef(50, **args) == pytest.approx(1.0)
    assert bc_anchor_coef(1050, **args) == pytest.approx(0.55)
    assert bc_anchor_coef(2050, **args) == pytest.approx(0.1)
    assert bc_anchor_coef(10_000, **args) == pytest.approx(0.1)
    assert bc_anchor_coef(60, warmup=50, start=1.0, end=0.1, decay_iters=0) == pytest.approx(0.1)


def test_anchor_losses_match_dagger_formulas():
    gen = torch.Generator().manual_seed(0)
    mu = torch.randn(4, 6, 17, generator=gen, requires_grad=True)
    teacher = torch.randn(4, 6, 17, generator=gen)
    pred = torch.randn(4, 6, 17, generator=gen, requires_grad=True)
    gt = torch.randn(4, 6, 22, generator=gen)  # 答えの観測は 17 個より長くてよい（先頭 17 個を使う）
    w = torch.rand(17, generator=gen) + 0.5
    bc, obj = anchor_losses(mu, teacher, pred, gt, w)
    assert torch.allclose(bc, torch.nn.MSELoss()(mu, teacher))
    assert torch.allclose(obj, ((pred - gt[..., :17]) ** 2 * w).mean(dim=-1).mean())
    (bc + obj).backward()
    assert mu.grad is not None and pred.grad is not None and mu.grad.abs().sum() > 0


def _bare_trainer(global_step, rl_start_step, warmup=50):
    tr = object.__new__(TRLPPODistillTrainerHomieAPI)
    tr.state = SimpleNamespace(global_step=global_step, rl_start_step=rl_start_step)
    tr.critic_warmup_iters = warmup
    tr.bc_anchor_coef_start, tr.bc_anchor_coef_end, tr.bc_anchor_decay_iters = 1.0, 0.1, 2000
    tr.obj_pred_loss_coef = 0.2
    tr.policy_model = SimpleNamespace(num_actions=17, obj_pred_weight=torch.ones(17))
    tr.args = SimpleNamespace(vf_coef=0.5, learning_rate=1e-4)
    tr.desired_kl = 0.005
    return tr


def _loss_inputs():
    gen = torch.Generator().manual_seed(1)
    mu = torch.randn(3, 5, 17 + 12, generator=gen)  # 生徒の 17 個＋HOMIE の脚（学習係が先頭 17 個に切る）
    fr = {"policy_results": {"action_mean": mu}, "obj_pred": torch.randn(3, 5, 17, generator=gen)}
    mb = {"mb_teacher_actions": torch.randn(3, 5, 17, generator=gen), "mb_gt_obj_pos": torch.randn(3, 5, 17, generator=gen)}
    return fr, mb


@pytest.fixture
def fake_ppo_loss(monkeypatch):
    def _parent(self, forward_results, mb_rollout_data):
        return {"loss": torch.tensor(3.0), "ppo_loss_dict": {"vf_loss": torch.tensor(8.0)}}

    monkeypatch.setattr(TRLPPOTrainer, "_compute_loss", _parent)


def test_loss_is_critic_only_during_warmup(fake_ppo_loss):
    tr = _bare_trainer(global_step=71_620, rl_start_step=71_600)  # 慣らしの 20 反復目
    fr, mb = _loss_inputs()
    ret = tr._compute_loss(fr, mb)
    assert float(ret["loss"]) == pytest.approx(0.5 * 8.0)
    assert ret["bc_anchor_coef"] == 0.0
    assert float(ret["bc_anchor_loss"]) > 0 and float(ret["obj_pred_loss"]) > 0  # 記録には出す


def test_loss_adds_anchor_and_tracker_after_warmup(fake_ppo_loss):
    tr = _bare_trainer(global_step=71_600 + 50 + 1000, rl_start_step=71_600)
    fr, mb = _loss_inputs()
    ret = tr._compute_loss(fr, mb)
    bc, obj = anchor_losses(fr["policy_results"]["action_mean"][..., :17], mb["mb_teacher_actions"], fr["obj_pred"],
                            mb["mb_gt_obj_pos"], torch.ones(17))
    assert ret["bc_anchor_coef"] == pytest.approx(0.55)
    assert float(ret["loss"]) == pytest.approx(3.0 + 0.55 * float(bc) + 0.2 * float(obj), rel=1e-5)


def test_kl_learning_rate_frozen_during_warmup():
    opt = torch.optim.Adam([torch.nn.Parameter(torch.zeros(1))], lr=1e-4)
    tr = _bare_trainer(global_step=71_600, rl_start_step=71_600)
    tr._adjust_learning_rate_based_on_kl(torch.tensor(1e-6), opt)  # KL≈0（actor が動かない）
    assert opt.param_groups[0]["lr"] == pytest.approx(1e-4)
    tr.state.global_step = 71_600 + 50
    tr._adjust_learning_rate_based_on_kl(torch.tensor(1e-6), opt)
    assert opt.param_groups[0]["lr"] == pytest.approx(1.5e-4)


def test_rollout_clears_teacher_without_distribution(monkeypatch):
    """教師は act_inference だけで distribution を作らない。2 回目の反復の clear_rollout で落ちない（11-08 r1 の区切り 0 で踏んだ）"""
    from gr00t.rl.trl.modules.actor_critic_modules import Actor

    class _Teacher:
        def __init__(self):
            self.distribution = None  # Actor.__init__ と同じ
            self.detached = 0

        init_rollout = Actor.init_rollout

        def clear_rollout(self):
            Actor.clear_rollout(self)  # del self.distribution（2 回目から属性が無い）
            self.detached += 1  # RecurrentActor の隠れ状態の切り離しの代わり

    monkeypatch.setattr(TRLPPOTrainer, "_rollout_step", lambda self, model, obs: obs)
    tr = _bare_trainer(global_step=0, rl_start_step=0)
    tr.ref_model = _Teacher()
    for _ in range(3):
        assert tr._rollout_step(None, {"x": 1}) == {"x": 1}
    assert tr.ref_model.detached == 3


@pytest.mark.skipif(not os.path.exists(F3SC), reason="f3sc の重みが無い")
def test_ppo_mode_keeps_tracked_door_values_in_env_by_step():
    from gr00t.rl.trl.utils.rl import split_and_pad_trajectories

    stu, env_config = _student(door_tracker_hidden=32)
    with torch.no_grad():
        stu.tracker_out.weight.normal_(0.0, 0.3)  # 追う段が値を変える状態で確かめる
    stu.ppo_mode = True
    stu.eval()
    gen = torch.Generator().manual_seed(2)
    n, length = 3, 6
    seq = [_make_obs(stu, env_config, n, gen)[0] for _ in range(length)]
    dones = torch.zeros(length, n)
    dones[2, 1] = 1.0  # 台 1 は 3 歩目で回が終わる
    with torch.no_grad():
        acts, doors = [], []
        for t, o in enumerate(seq):
            a, d = stu._rollout_mean({"actor_obs": o})
            acts.append(a)
            doors.append(d)
            stu.reset(dones[t].long())
        padded, masks = split_and_pad_trajectories(torch.stack(seq, dim=0), dones)
        stu.update_distribution({"actor_obs": padded.transpose(0, 1)}, masks=masks.transpose(0, 1),
                                original_dones=dones.t())
    roll_d = torch.stack(doors, dim=1)
    assert stu.last_obj_pred.shape == roll_d.shape == (n, length, 17)
    assert torch.allclose(stu.last_obj_pred, roll_d, atol=1e-4), (stu.last_obj_pred - roll_d).abs().max()
    assert torch.allclose(stu.distribution.mean, torch.stack(acts, dim=1), atol=1e-4)


def test_build_ppo_start_from_student_and_teacher():
    from gr00t.rl.scripts.make_vis_ppo_ckpt import build, env_state_diff

    student = {
        "policy_state_dict": {"door_head.net.0.weight": torch.ones(2, 2), "tracker.rnn.weight_ih_l0": torch.full((4, 3), 2.0),
                              "std": torch.full((17,), 0.001)},
        "state": SimpleNamespace(global_step=12_000),
        "env_state_dict": {"termination_level": 1.0},
    }
    teacher = {
        "policy_state_dict": {"std": torch.full((17,), 0.2)},
        "value_state_dict": {"memory.rnn.weight_ih_l0": torch.zeros(3)},
        "state": SimpleNamespace(global_step=71_600),
        "env_state_dict": {"termination_level": 0.9},
    }
    out = build(student, teacher, 0.5)
    assert set(out["policy_state_dict"]) == set(student["policy_state_dict"])
    assert torch.equal(out["policy_state_dict"]["tracker.rnn.weight_ih_l0"], torch.full((4, 3), 2.0))
    assert torch.allclose(out["policy_state_dict"]["std"], torch.full((17,), 0.1))
    assert out["value_state_dict"] is teacher["value_state_dict"]
    assert out["state"].rl_start_step == 71_600 and out["optimizer_state_dict"] is None
    assert not hasattr(teacher["state"], "rl_start_step")  # 読んだ教師の状態は書き換えない
    assert env_state_diff(student["env_state_dict"], out["env_state_dict"]) == ["termination_level: 生徒 1.0 ・教師 0.9"]
    with pytest.raises(AssertionError):
        build({"policy_state_dict": {"std": torch.ones(17)}, "state": student["state"]}, teacher, 0.5)


@pytest.mark.skipif(not os.path.exists(COUNT_SUCCESS), reason="count_success.py が無い")
def test_count_success_passed_ignores_stage():
    spec = importlib.util.spec_from_file_location("count_success", COUNT_SUCCESS)
    cs = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(cs)
    steps = 6
    # 台 0: 段 0 のまま扉 80° で抜けた（passed・through ではない）。台 1: 段 5 で抜けた（両方）。
    # 台 2: 扉 50° のまま抜けた（どちらでもない）。台 3: 扉を 80° 開けたが 1.5 m に届かない（どちらでもない）
    stage = np.array([[0, 0, 0, 0], [0, 4, 0, 0], [0, 5, 0, 0], [0, 5, 0, 0], [0, 5, 0, 0], [0, 5, 0, 0]])
    x = np.array([[0.0, 0.0, 0.0, 0.0], [0.5, 0.5, 0.5, 0.5], [1.0, 1.0, 1.0, 1.0], [1.6, 1.6, 1.6, 1.1],
                  [1.7, 1.7, 1.7, 1.2], [1.8, 1.8, 1.8, 1.2]])
    hinge_deg = np.array([[0, 0, 0, 0], [40, 40, 30, 40], [80, 75, 50, 80], [80, 75, 50, 80], [80, 75, 50, 80],
                          [80, 75, 50, 80]], dtype=float)
    rec = {
        "stage": stage,
        "episode_length": np.tile(np.arange(1, steps + 1)[:, None], (1, 4)),
        "root_pos": np.stack([x, np.zeros_like(x), np.zeros_like(x)], axis=-1),
        "door_dof_pos": np.stack([np.radians(hinge_deg), np.zeros_like(x)], axis=-1),
    }
    eps = cs.split_episodes(rec)
    assert [cs.is_success(ep, "passed") for ep in eps] == [True, True, False, False]
    assert [cs.is_success(ep, "through") for ep in eps] == [False, True, False, False]
