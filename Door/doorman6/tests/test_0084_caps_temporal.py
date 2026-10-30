"""パッチ 0084（10-30・CAPS の時間の項 caps_temporal）の試験。
学習器は Isaac Sim と trl を読むので import せず、コードの組み立てを読んで確かめ、損失の関数 1 つだけを切り出して動かす。GPU も使わない。

OMEN で: cd ~/work_inoue/doorman6/VIRAL6_fix && PYTHONPATH=. ~/miniforge3/envs/doorman6/bin/python -m pytest -q <この試験>

- 損失は出力の平均の [start, end) の並びの、同じ行（1 台の続いた歩）の隣り合う時刻の差の 2 乗の和を、組で平均したもの
- 回の切れ目（dones[t] が真なら t と t+1 は別の回）と詰め物にかかる組は数えない
- 係数（algo.config.caps_temporal_coef）の既定は 0 で、そのときは損失に足さず勾配も作らない（元の学習は変わらない）。値は記録にだけ出す
- 損失の各項・KL・実際の学習率を train.log に出す
"""
from __future__ import annotations

import ast
from pathlib import Path

import pytest

TRAINER = Path("gr00t/rl/trl/trainer/ppo_trainer_homie_api.py")


def _class(name: str) -> ast.ClassDef:
    tree = ast.parse(TRAINER.read_text(encoding="utf-8"))
    return next(n for n in tree.body if isinstance(n, ast.ClassDef) and n.name == name)


def _method(cls: str, name: str) -> ast.FunctionDef:
    for node in _class(cls).body:
        if isinstance(node, ast.FunctionDef) and node.name == name:
            return node
    raise AssertionError(f"{cls}.{name} が {TRAINER} に無い")


def _caps_fn():
    torch = pytest.importorskip("torch")
    ns = {"torch": torch}
    fn = _method("TRLPPOTrainer", "_compute_caps_temporal_loss")
    exec(compile(ast.Module(body=[fn], type_ignores=[]), str(TRAINER), "exec"), ns)  # noqa: S102

    class Trainer:
        caps_temporal_start = 3
        caps_temporal_end = 17

    return torch, Trainer(), ns["_compute_caps_temporal_loss"]


def _inputs(torch, mu, dones, padding_mask=None):
    dones = torch.as_tensor(dones, dtype=torch.bool)
    if padding_mask is None:
        padding_mask = torch.zeros_like(dones)
    return (
        {"policy_results": {"action_mean": mu}},
        {"mb_dones": dones, "mb_padding_mask": torch.as_tensor(padding_mask, dtype=torch.bool)},
    )


def test_coef_defaults_to_zero_and_range_is_read_from_config():
    src = ast.unparse(_method("TRLPPOTrainer", "_init_config"))
    assert "self.caps_temporal_coef = float(self.config.get('caps_temporal_coef', 0.0))" in src
    assert "self.config.get('caps_temporal_start', 0)" in src
    assert "self.config.get('caps_temporal_end', self.policy_model.num_actions)" in src


def test_loss_is_added_only_when_coef_is_positive_and_no_grad_otherwise():
    src = ast.unparse(_method("TRLPPOTrainer", "_compute_loss"))
    assert "with torch.set_grad_enabled(self.caps_temporal_coef > 0):" in src
    lines = [line.strip() for line in src.splitlines()]
    i_if = lines.index("if self.caps_temporal_coef > 0:")
    assert lines[i_if + 1] == "loss += caps_temporal_loss * self.caps_temporal_coef"
    assert "ret_dict['caps_temporal_loss'] = caps_temporal_loss.detach()" in src


def test_mean_squared_change_of_the_arm_slice_within_rows():
    torch, trainer, fn = _caps_fn()
    gen = torch.Generator().manual_seed(0)
    mu = torch.randn(2, 6, 29, generator=gen)  # [台, 時刻, 生徒 17 + HOMIE 12]
    fr, rd = _inputs(torch, mu, torch.zeros(2, 6))
    got = fn(trainer, fr, rd)
    want = ((mu[:, 1:, 3:17] - mu[:, :-1, 3:17]) ** 2).sum(-1).mean()
    assert torch.allclose(got, want)
    # 歩く指令（0〜2）と HOMIE の出力（17〜）は変えても値は変わらない
    mu2 = mu.clone()
    mu2[..., :3] += torch.randn(2, 6, 3, generator=gen)
    mu2[..., 17:] += torch.randn(2, 6, 12, generator=gen)
    assert torch.allclose(fn(trainer, *_inputs(torch, mu2, torch.zeros(2, 6))), want)


def test_pairs_across_an_episode_boundary_are_not_counted():
    torch, trainer, fn = _caps_fn()
    mu = torch.zeros(1, 4, 17)
    mu[0, 2:, 3] = 100.0  # t=1 で回が終わり、t=2 から次の回（初めの姿勢へ戻るので出力が跳ぶ）
    mu[0, 1, 3] = 1.0
    got = fn(trainer, *_inputs(torch, mu, [[0, 1, 0, 0]]))
    # 数える組は (0,1)・(2,3) の 2 つ: (1² + 0²) / 2
    assert torch.allclose(got, torch.tensor(0.5))


def test_padded_steps_are_not_counted():
    torch, trainer, fn = _caps_fn()
    mu = torch.zeros(1, 4, 17)
    mu[0, 3, 3] = 100.0
    mu[0, 1, 3] = 2.0
    got = fn(trainer, *_inputs(torch, mu, [[0, 0, 0, 0]], padding_mask=[[0, 0, 0, 1]]))
    # 数える組は (0,1)・(1,2) の 2 つ: (2² + 2²) / 2
    assert torch.allclose(got, torch.tensor(4.0))


def test_constant_output_gives_zero_and_no_pairs_gives_zero_not_nan():
    torch, trainer, fn = _caps_fn()
    mu = torch.ones(3, 5, 17)
    assert fn(trainer, *_inputs(torch, mu, torch.zeros(3, 5))) == 0
    noisy = torch.randn(3, 5, 17)
    got = fn(trainer, *_inputs(torch, noisy, torch.ones(3, 5)))
    assert torch.isfinite(got) and got == 0


def test_gradient_reaches_the_output_mean():
    torch, trainer, fn = _caps_fn()
    mu = torch.zeros(1, 3, 17, requires_grad=True)
    with torch.no_grad():
        mu[0, 1, 5] = 1.0
    fn(trainer, *_inputs(torch, mu, [[0, 0, 0]])).backward()
    # L = ((1-0)² + (0-1)²) / 2 なので dL/dμ_1 = (2·1 + 2·1) / 2 = 2、両隣は −1
    assert torch.allclose(mu.grad[0, :, 5], torch.tensor([-1.0, 2.0, -1.0]))
    assert torch.count_nonzero(mu.grad[..., :3]) == 0


def test_stats_and_metrics_are_always_recorded():
    reg = ast.unparse(_method("TRLPPOTrainer", "_register_stats_buffer"))
    assert "self.caps_temporal_stats = torch.zeros(stats_shape, device=device)" in reg
    upd = ast.unparse(_method("TRLPPOTrainer", "_update_stats_buffer"))
    assert "self.caps_temporal_stats[ppo_epoch_idx, minibatch_idx, microbatch_idx] = loss_dict['caps_temporal_loss']" in upd
    met = ast.unparse(_method("TRLPPOTrainer", "_get_train_metrics"))
    for key in ("loss/caps_temporal_avg", "loss/weighted_caps_temporal_avg", "policy/learning_rate"):
        assert f"metrics['{key}']" in met
    assert "self.optimizer.param_groups[0]['lr']" in met


def test_printer_writes_loss_and_policy_keys_to_the_log():
    src = ast.unparse(_method("PrinterHVCallback", "on_log"))
    assert "if k.startswith(('loss/', 'policy/')):" in src
