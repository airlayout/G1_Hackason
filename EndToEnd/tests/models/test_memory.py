"""再帰・欠損・query分割の意味を、Isaacに依存せずCPUで検証する。"""

from dataclasses import replace

import pytest

from .support import ROOT, config, history_state, observations, torch

from g1_e2e.models import (
    MemoryState,
    ModelConfig,
    QueryRequest,
    RecurrentPerceiver,
    SensorTokens,
)


def test_observation_only_phase_excludes_history_from_update_but_reads_it_for_queries():
    model = RecurrentPerceiver(config(recurrence_period=2)).eval()
    obs = observations()
    a = history_state(model, frames=1)
    b = replace(a, tokens=torch.randn_like(a.tokens))
    queries = {
        "velocity": QueryRequest(),
        "bev": QueryRequest(torch.randn(1, 2, 3)),
        "trajectory": QueryRequest(torch.ones(1, 3, 1)),
    }
    first, second = model(obs, a, queries), model(obs, b, queries)
    assert not first.memory.recurrent.any()
    torch.testing.assert_close(first.memory.latents, second.memory.latents)
    # Uの生成は履歴に依存しないが、全用途のqueryは履歴と今回のUの両方を読む。
    changed = model(observations(), a, queries)
    assert not torch.allclose(first.memory.latents, changed.memory.latents)
    for name in queries:
        assert not torch.allclose(first.predictions[name], second.predictions[name])
        assert not torch.allclose(first.predictions[name], changed.predictions[name])
    torch.testing.assert_close(first.memory.state.tokens[:, :1], a.tokens)
    torch.testing.assert_close(second.memory.state.tokens[:, :1], b.tokens)
    torch.testing.assert_close(first.memory.state.tokens, a.tokens)
    torch.testing.assert_close(changed.memory.state.tokens, a.tokens)
    assert first.memory.state.step.tolist() == [2]
    continued_a = model(obs, first.memory.state)
    continued_b = model(obs, second.memory.state)
    assert continued_a.memory.recurrent.all()
    assert not torch.allclose(continued_a.memory.latents, continued_b.memory.latents)
    # 前のUだけを変えても、次のMを生成する入力・出力は変わらない。
    continued_changed = model(obs, changed.memory.state)
    torch.testing.assert_close(continued_a.memory.latents, continued_changed.memory.latents)


def test_query_gradients_reach_current_u_and_history_without_recurrence():
    model = RecurrentPerceiver(config(recurrence_period=2)).eval()
    prior = history_state(model, frames=1)
    prior.tokens.requires_grad_()
    obs = observations(requires_grad=True)
    memory = model.encode_step(obs, prior)
    memory.latents.retain_grad()
    model.decode(memory)["velocity"].square().sum().backward()
    assert prior.tokens.grad.abs().sum() > 0
    assert memory.latents.grad.abs().sum() > 0
    assert obs["points"].features.grad.abs().sum() > 0


def test_recurrent_readout_reads_current_m_once_and_ignores_transient_slot():
    model = RecurrentPerceiver(config(recurrence_period=2)).eval()
    memory = model.encode_step(observations(), history_state(model, frames=2))
    assert memory.recurrent.all()
    queries = {
        "velocity": QueryRequest(),
        "bev": QueryRequest(torch.randn(1, 2, 3)),
        "trajectory": QueryRequest(torch.ones(1, 3, 1)),
    }
    history_only = model.decode(memory.state, queries)
    from_update = model.decode(memory, queries)
    masked_nan = model.decode(replace(memory, latents=torch.full_like(memory.latents, float("nan"))), queries)
    for name in queries:
        torch.testing.assert_close(from_update[name], history_only[name])
        torch.testing.assert_close(masked_nan[name], history_only[name])


def test_mixed_phase_readout_matches_individual_episodes():
    model = RecurrentPerceiver(config(recurrence_period=2)).eval()
    prior = history_state(model, frames=2, batch_size=2)
    prior = replace(prior, token_steps=prior.token_steps * 2, step=torch.tensor([4, 5]))
    obs = observations(batch_size=2)
    queries = {
        "velocity": QueryRequest(),
        "bev": QueryRequest(torch.randn(2, 2, 3)),
        "trajectory": QueryRequest(torch.ones(2, 3, 1)),
    }
    batch = model(obs, prior, queries)
    assert batch.memory.recurrent.tolist() == [True, False]
    for row in range(2):
        state = MemoryState(**{key: value[row : row + 1] for key, value in vars(prior).items()})
        points = obs["points"]
        single_obs = {
            "points": SensorTokens(points.features[row : row + 1], positions=points.positions[row : row + 1])
        }
        single_queries = {
            name: QueryRequest(
                request.coordinates[row : row + 1] if request.coordinates is not None else None
            )
            for name, request in queries.items()
        }
        single = model(single_obs, state, single_queries)
        for name in queries:
            torch.testing.assert_close(batch.predictions[name][row : row + 1], single.predictions[name])


def test_reset_is_per_environment_and_resets_phase_and_history():
    model = RecurrentPerceiver(config(recurrence_period=2)).eval()
    obs = observations(batch_size=2)
    prior = history_state(model, frames=3, batch_size=2)
    original = prior.tokens.clone()
    reset = model(obs, prior, reset_mask=torch.tensor([True, False]))
    fresh = model(obs)
    torch.testing.assert_close(reset.memory.latents[0], fresh.memory.latents[0])
    torch.testing.assert_close(reset.predictions["velocity"][0], fresh.predictions["velocity"][0])
    torch.testing.assert_close(reset.memory.state.tokens[1, -3:], prior.tokens[1])
    assert not reset.memory.state.valid[0, :3].any()
    assert reset.memory.state.valid[1].sum().item() == 3
    assert reset.memory.state.valid[0, -1:].all()
    assert reset.memory.state.token_steps[0, -1:].tolist() == [0]
    assert reset.memory.state.step.tolist() == [1, 4]
    assert reset.memory.recurrent.tolist() == [True, False]
    torch.testing.assert_close(prior.tokens, original)


def test_bptt_reaches_previous_observation_and_detach_cuts_history():
    model = RecurrentPerceiver(config())
    previous = observations(requires_grad=True)
    first = model.encode_step(previous)
    second = model(observations(), first.state)
    second.memory.latents.square().sum().backward()
    assert previous["points"].features.grad.abs().sum() > 0
    previous = observations(requires_grad=True)
    first = model.encode_step(previous)
    model(observations(), first.state.detach()).memory.latents.square().sum().backward()
    assert previous["points"].features.grad is None


def test_both_update_and_queries_can_read_older_m_with_identical_latest_m():
    model = RecurrentPerceiver(config()).eval()
    a = history_state(model)
    altered = a.tokens.clone()
    altered[:, :1] = torch.randn_like(altered[:, :1]) * 3
    b = replace(a, tokens=altered)
    obs = observations()
    assert not torch.allclose(model.encode_step(obs, a).latents, model.encode_step(obs, b).latents)
    queries = {
        "velocity": QueryRequest(),
        "bev": QueryRequest(torch.randn(1, 2, 3)),
        "trajectory": QueryRequest(torch.ones(1, 3, 1)),
    }
    first, second = model.decode(a, queries), model.decode(b, queries)
    for name in queries:
        assert not torch.allclose(first[name], second[name])


def test_time_encoding_distinguishes_the_order_of_past_m():
    model = RecurrentPerceiver(config()).eval()
    a = history_state(model, frames=2)
    b = replace(a, token_steps=1 - a.token_steps)
    assert not torch.allclose(model.decode(a)["velocity"], model.decode(b)["velocity"])
    obs = observations()
    assert not torch.allclose(model.encode_step(obs, a).latents, model.encode_step(obs, b).latents)


def test_invalid_history_tokens_are_ignored_in_both_update_and_readout():
    model = RecurrentPerceiver(config()).eval()
    a = history_state(model, frames=2)
    valid = a.valid.clone()
    valid[:, :1] = False
    a = replace(a, valid=valid)
    b = replace(a, tokens=torch.where(valid[..., None], a.tokens, float("nan")))
    obs = observations()
    torch.testing.assert_close(model.encode_step(obs, a).latents, model.encode_step(obs, b).latents)
    torch.testing.assert_close(model.decode(a)["velocity"], model.decode(b)["velocity"])


def test_only_recurrent_cycles_append_m_and_discard_oldest_tokens():
    model = RecurrentPerceiver(config(context_window=3, recurrence_period=2)).eval()
    state = model.initial_state(1)
    outputs, phases = [], []
    with torch.inference_mode():
        for index in range(9):
            old = state.tokens.clone()
            memory = model.encode_step(observations(), state)
            state = memory.state
            if memory.recurrent.item():
                outputs.append(memory.latents)
            phases.append(memory.recurrent.item())
            assert memory.latents.shape == (1, 1, 16)
            assert state.tokens.shape[1] == min((index // 2) + 1, 3)
            expected = torch.cat(outputs, dim=1)[:, -3:]
            torch.testing.assert_close(state.tokens, expected)
            if not memory.recurrent.item():
                torch.testing.assert_close(old, state.tokens)
            assert state.step.item() == index + 1
            assert state.valid.all()
    assert phases == [True, False, True, False, True, False, True, False, True]
    assert state.token_steps.tolist() == [[4, 6, 8]]


def test_full_size_model_updates_and_reads_4096_token_window():
    definition = ModelConfig.load(ROOT / "configs/model/recurrent_perceiver.yaml")
    model = RecurrentPerceiver(definition).eval()
    prior = history_state(model, frames=4096)
    prior = replace(prior, token_steps=prior.token_steps * 2, step=prior.step * 2)
    original = prior.tokens.clone()
    queries = {
        "velocity": QueryRequest(),
        "bev": QueryRequest(torch.randn(1, 3, 3)),
        "trajectory": QueryRequest(torch.ones(1, 4, 1)),
    }
    with torch.inference_mode():
        output = model(
            {"imu": SensorTokens(torch.randn(1, 1, 10))},
            prior,
            queries=queries,
        )
        off = model({"imu": SensorTokens(torch.randn(1, 1, 10))}, output.memory.state, queries)
        history_only = model.decode(off.memory.state, queries)
    assert output.memory.recurrent.all()
    state = output.memory.state
    assert state.tokens.shape == (1, 4096, 256)
    assert output.memory.latents.shape == (1, 1, 256)
    torch.testing.assert_close(state.tokens[:, :-1], original[:, 1:])
    torch.testing.assert_close(state.tokens[:, -1:], output.memory.latents)
    assert state.token_steps[0, 0].item() == 2
    assert state.token_steps[0, -1].item() == 8192
    assert state.step.item() == 8193
    torch.testing.assert_close(prior.tokens, original)
    assert not off.memory.recurrent.any()
    torch.testing.assert_close(off.memory.state.tokens, state.tokens)
    torch.testing.assert_close(off.memory.state.token_steps, state.token_steps)
    assert off.memory.state.step.tolist() == [8194]
    for name in queries:
        assert torch.isfinite(output.predictions[name]).all()
        assert torch.isfinite(off.predictions[name]).all()
        assert not torch.allclose(off.predictions[name], history_only[name], rtol=0, atol=1e-7)


def test_query_gradients_reach_older_m_independently_of_latest_m():
    model = RecurrentPerceiver(config()).eval()
    state = history_state(model)
    tokens = state.tokens.requires_grad_()
    model.decode(state)["velocity"].square().sum().backward()
    assert tokens.grad[:, :1].abs().sum() > 0
    assert tokens.grad[:, -1:].abs().sum() > 0


def test_reset_after_window_wrap_keeps_other_episode_history_and_phase():
    model = RecurrentPerceiver(config(context_window=3, recurrence_period=2)).eval()
    state = model.initial_state(2)
    with torch.inference_mode():
        for _ in range(5):
            state = model.encode_step(observations(batch_size=2), state).state
        saved = state.tokens.clone()
        reset = model.encode_step(observations(batch_size=2), state, reset_mask=torch.tensor([True, False]))
    assert reset.recurrent.tolist() == [True, False]
    assert reset.state.step.tolist() == [1, 6]
    assert reset.state.valid[0].tolist() == [False, False, True]
    torch.testing.assert_close(reset.state.tokens[1], saved[1])


def test_async_episodes_preserve_each_rows_full_window_of_m_updates():
    model = RecurrentPerceiver(config(context_window=3, recurrence_period=2)).eval()
    state = model.initial_state(2)
    expected_tokens, expected_steps = [[], []], [[], []]
    clocks = [0, 0]
    with torch.inference_mode():
        for cycle in range(12):
            reset = torch.tensor([cycle == 3, False])
            if reset[0]:
                expected_tokens[0], expected_steps[0], clocks[0] = [], [], 0
            memory = model.encode_step(observations(batch_size=2), state, reset_mask=reset)
            state = memory.state
            for row in range(2):
                if clocks[row] % 2 == 0:
                    expected_tokens[row].append(memory.latents[row])
                    expected_steps[row].append(clocks[row])
                clocks[row] += 1
                valid = state.valid[row]
                torch.testing.assert_close(
                    state.tokens[row, valid], torch.cat(expected_tokens[row][-3:], dim=0)
                )
                assert state.token_steps[row, valid].tolist() == expected_steps[row][-3:]
                assert state.step[row].item() == clocks[row]


def test_history_and_phase_can_be_checkpointed_without_hidden_module_state(tmp_path):
    model = RecurrentPerceiver(config(recurrence_period=2)).eval()
    state = model.initial_state(1)
    for _ in range(3):
        state = model.encode_step(observations(), state).state.detach()
    path = tmp_path / "state.pt"
    torch.save({"model": model.state_dict(), "history": vars(state)}, path)
    checkpoint = torch.load(path, weights_only=True)
    restored = RecurrentPerceiver(model.config).eval()
    restored.load_state_dict(checkpoint["model"])
    restored_state = MemoryState(**checkpoint["history"])
    obs = observations()
    first, second = model(obs, state), restored(obs, restored_state)
    torch.testing.assert_close(first.memory.state.tokens, second.memory.state.tokens)
    torch.testing.assert_close(first.predictions["velocity"], second.predictions["velocity"])
    assert first.memory.recurrent.tolist() == second.memory.recurrent.tolist() == [False]


def test_empty_history_and_invalid_window_or_metadata_are_rejected():
    model = RecurrentPerceiver(config())
    with pytest.raises(ValueError, match="正数"):
        config(context_window=0)
    with pytest.raises(ValueError, match="encode_step"):
        model.decode(model.initial_state(1))
    state = history_state(model)
    with pytest.raises(ValueError, match="state.valid"):
        model.decode(replace(state, valid=state.valid.float()))
    with pytest.raises(ValueError, match="token_steps"):
        model.decode(replace(state, token_steps=state.token_steps.float()))
    with pytest.raises(TypeError, match="MemoryState"):
        model.decode(state.tokens)
    memory = model.encode_step(observations(), state)
    with pytest.raises(ValueError, match="latents"):
        model.decode(replace(memory, latents=memory.latents[:, :0]))
    with pytest.raises(ValueError, match="recurrent"):
        model.decode(replace(memory, recurrent=memory.recurrent.float()))
