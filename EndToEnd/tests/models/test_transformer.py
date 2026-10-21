"""単一系列での情報交換、最新Mの引継ぎ、非同期時刻RoPEの性質を検証する。"""

from dataclasses import replace

import pytest

from .support import ROOT, config, history_state, observations, torch
from g1_e2e.models import ModelConfig, RecurrentPerceiver, SensorTokens
from g1_e2e.models.position import RotaryTimeEmbedding
from g1_e2e.models.feedforward import SwiGLU


def test_latest_saved_m_is_the_residual_source_and_seed_is_only_for_empty_history():
    model = RecurrentPerceiver(config()).eval()
    # attentionとFFNを無効化すると、取り出される入力位置を直接検証できる。
    with torch.no_grad():
        for parameter in model.blocks.parameters():
            parameter.zero_()
        prior = history_state(model)
        current = model.encode_step(observations(), prior)
        expected = model.output_norm(prior.tokens[:, -1:] + model.memory_type)
        torch.testing.assert_close(current.latents, expected)
        fresh = model.encode_step(observations())
        torch.testing.assert_close(fresh.latents, model.output_norm(model.seed + model.memory_type))


def test_history_changes_observation_tokens_between_transformer_layers():
    model = RecurrentPerceiver(config(layers=2)).eval()
    prior = history_state(model, frames=2)
    changed = replace(prior, tokens=prior.tokens + torch.randn_like(prior.tokens))
    obs = observations()
    seen = []
    handle = model.blocks[1].register_forward_pre_hook(
        lambda module, args: seen.append(args[0].detach().clone())
    )
    with torch.inference_mode():
        first = model.encode_step(obs, prior)
        second = model.encode_step(obs, changed)
    handle.remove()
    # seed1 + history2 + null1 + 観測5。観測自身も履歴を参照して更新される。
    assert seen[0].shape == (1, 9, 16)
    assert not torch.allclose(seen[0][:, -5:], seen[1][:, -5:])
    assert not torch.allclose(first.latents, second.latents)


def test_history_and_async_observations_share_control_tick_time_axis():
    model = RecurrentPerceiver(config()).eval()
    prior = history_state(model, frames=2)
    prior = replace(prior, step=torch.tensor([4]), token_steps=torch.tensor([[0, 2]]))
    points = observations()["points"]
    obs = {"points": replace(points, age_s=torch.tensor([[0.0, 0.01, 0.02, 0.03, 0.04]]))}
    seen = []
    handle = model.blocks[0].register_forward_pre_hook(
        lambda module, args: seen.append(args[2].detach().clone())
    )
    model.encode_step(obs, prior)
    handle.remove()
    torch.testing.assert_close(seen[0], torch.tensor([[0.0, -4.0, -2.0, 0.0, 0.0, -0.5, -1.0, -1.5, -2.0]]))


def test_clock_origin_does_not_change_memory_and_observation_age_does():
    model = RecurrentPerceiver(config()).eval()
    prior = history_state(model, frames=2)
    shifted = replace(prior, step=prior.step + 10000, token_steps=prior.token_steps + 10000)
    obs = observations()
    original = model.encode_step(obs, prior).latents
    torch.testing.assert_close(original, model.encode_step(obs, shifted).latents)
    aged = {"points": replace(obs["points"], age_s=torch.tensor([0.04]))}
    assert not torch.allclose(original, model.encode_step(aged, prior).latents)


def test_rope_rotates_qk_and_preserves_relative_time_under_common_translation():
    rope = RotaryTimeEmbedding(8, 10000.0)
    q, k = torch.randn(2, 2, 3, 8), torch.randn(2, 2, 3, 8)
    times = torch.tensor([[0.0, 0.5, 4.0], [-1.0, 0.0, 2.0]])
    rotated_q, rotated_k = rope(q, times), rope(k, times)
    assert not torch.allclose(rotated_q, q)
    torch.testing.assert_close(rotated_q.norm(dim=-1), q.norm(dim=-1))
    expected = rotated_q @ rotated_k.transpose(-1, -2)
    translated = rope(q, times + 10) @ rope(k, times + 10).transpose(-1, -2)
    torch.testing.assert_close(expected, translated, atol=1e-5, rtol=1e-5)


def test_history_window_can_expand_at_inference_without_new_model_parameters():
    trained = RecurrentPerceiver(config(context_window=2)).eval()
    expanded = RecurrentPerceiver(replace(trained.config, context_window=8)).eval()
    expanded.load_state_dict(trained.state_dict())
    prior = history_state(expanded, frames=6)
    with torch.inference_mode():
        result = expanded.encode_step(observations(), prior)
    assert result.latents.shape == (1, 1, 16)
    assert result.state.tokens.shape == (1, 7, 16)
    assert torch.isfinite(result.latents).all()


@pytest.mark.parametrize(
    "changes", [{"heads": 16}, {"control_dt_s": 0}, {"rope_base": 1}, {"ff_multiple_of": 0}]
)
def test_invalid_time_rope_configuration_is_rejected(changes):
    with pytest.raises(ValueError):
        config(**changes)


def test_masked_nan_ages_do_not_affect_the_unified_sequence():
    model = RecurrentPerceiver(config()).eval()
    points = observations()["points"]
    valid = torch.tensor([[True, True, False, False, False]])
    ages = torch.tensor([[0.0, 0.01, float("nan"), float("nan"), float("nan")]])
    bad = SensorTokens(points.features, valid, points.positions, ages)
    clean = replace(bad, age_s=torch.nan_to_num(ages))
    torch.testing.assert_close(model({"points": bad}).latents, model({"points": clean}).latents)


@pytest.mark.parametrize("branch", ["gate", "up"])
def test_swiglu_requires_both_gate_and_value_branches(branch):
    ffn = SwiGLU(16, 24, dropout=0)
    with torch.no_grad():
        gate_weights, up_weights = ffn.gate_up.weight.chunk(2, dim=0)
        selected = gate_weights if branch == "gate" else up_weights
        selected.zero_()
    output = ffn(torch.randn(2, 5, 16))
    torch.testing.assert_close(output, torch.zeros_like(output))


def test_36_layer_swiglu_backpropagates_to_first_and_last_blocks():
    definition = ModelConfig.load(ROOT / "configs/model/recurrent_perceiver.yaml")
    model = RecurrentPerceiver(definition)
    features = torch.randn(1, 1, 14, requires_grad=True)
    memory = model.encode_step({"imu_pelvis": SensorTokens(features)})
    (memory.latents * torch.randn_like(memory.latents)).sum().backward()
    assert torch.isfinite(features.grad).all() and features.grad.abs().sum() > 0
    for block in [model.blocks[0], model.blocks[-1]]:
        for parameter in block.ff.parameters():
            assert parameter.grad is not None
            assert torch.isfinite(parameter.grad).all() and parameter.grad.abs().sum() > 0
