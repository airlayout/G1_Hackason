"""再帰・欠損・query分割の意味を、Isaacに依存せずCPUで検証する。"""

from dataclasses import replace

import pytest

from .support import ROOT, config, history_state, observations, torch

from g1_e2e.models import (
    ModelConfig,
    QueryRequest,
    RecurrentPerceiver,
    SensorTokens,
    grid_queries,
    image_patches,
)


def test_default_definition_supports_all_modalities_and_three_readouts():
    definition = ModelConfig.load(ROOT / "configs/model/recurrent_perceiver.yaml")
    assert definition.recurrence_period == 2
    assert definition.context_window == 4096
    assert definition.memory_tokens == 1
    model = RecurrentPerceiver(definition).eval()
    obs = {
        "rgb": image_patches(torch.rand(1, 3, 128, 128)),
        "depth": image_patches(torch.rand(1, 2, 128, 128)),
        "lidar": SensorTokens(torch.rand(1, 128, 6), positions=torch.rand(1, 128, 3)),
        "joints": SensorTokens(torch.rand(1, 29, 3)),
        "imu": SensorTokens(torch.rand(1, 1, 10)),
        "previous_action": SensorTokens(torch.rand(1, 29, 1)),
    }
    with torch.inference_mode():
        result = model(
            obs,
            queries={
                "velocity": QueryRequest(),
                "bev": QueryRequest(torch.randn(1, 11, 3)),
                "trajectory": QueryRequest(torch.linspace(0.1, 2.0, 20).reshape(1, -1, 1)),
            },
        )
    assert result.memory.latents.shape == (1, 1, 256)
    assert result.memory.state.tokens.shape == (1, 1, 256)
    assert {name: value.shape for name, value in result.predictions.items()} == {
        "velocity": (1, 1, 3),
        "bev": (1, 11, 32),
        "trajectory": (1, 20, 7),
    }
    for value in result.predictions.values():
        assert torch.isfinite(value).all()
    torch.testing.assert_close(result.predictions["trajectory"][..., 3:].norm(dim=-1), torch.ones(1, 20))


def test_missing_sensors_and_masked_nan_do_not_poison_memory_or_gradients():
    model = RecurrentPerceiver(config()).eval()
    x = torch.randn(1, 5, 3)
    positions = torch.randn_like(x)
    valid = torch.tensor([[True, True, False, False, False]])
    masked = torch.where(valid[..., None], x, float("nan")).requires_grad_()
    bad_positions = torch.where(valid[..., None], positions, float("nan"))
    result = model({"points": SensorTokens(masked, valid, bad_positions)})
    reference = model({"points": SensorTokens(x, valid, positions)})
    torch.testing.assert_close(result.memory.latents, reference.memory.latents)
    result.predictions["velocity"].sum().backward()
    assert torch.isfinite(masked.grad).all()
    assert (masked.grad[~valid] == 0).all()
    empty = model({}, model.initial_state(1))
    assert torch.isfinite(empty.predictions["velocity"]).all()


def test_point_order_does_not_change_memory():
    model = RecurrentPerceiver(config()).eval()
    obs = observations()
    points = obs["points"]
    permutation = torch.tensor([3, 0, 4, 1, 2])
    permuted = {
        "points": SensorTokens(points.features[:, permutation], positions=points.positions[:, permutation])
    }
    torch.testing.assert_close(model(obs).memory.latents, model(permuted).memory.latents)


@pytest.mark.parametrize("recurrent", [True, False])
def test_neural_field_is_independent_of_query_set_order_and_chunks(recurrent):
    model = RecurrentPerceiver(config(recurrence_period=2)).eval()
    prior = model.initial_state(1) if recurrent else history_state(model, frames=1)
    memory = model.encode_step(observations(), prior)
    saved_state = memory.state.tokens.clone()
    coordinates = torch.randn(1, 7, 3)
    full = model.decode(memory, {"bev": QueryRequest(coordinates)}, chunk_size=100)["bev"]
    split = model.decode(memory, {"bev": QueryRequest(coordinates)}, chunk_size=1)["bev"]
    permutation = torch.tensor([4, 2, 1, 6, 0, 3, 5])
    permuted = model.decode(memory, {"bev": QueryRequest(coordinates[:, permutation])})["bev"]
    one = model.decode(memory, {"bev": QueryRequest(coordinates[:, 3:4])})["bev"]
    torch.testing.assert_close(full, split)
    torch.testing.assert_close(full[:, permutation], permuted)
    torch.testing.assert_close(full[:, 3:4], one)
    torch.testing.assert_close(memory.state.tokens, saved_state)
    assert memory.state.step.tolist() == ([1] if recurrent else [2])


def test_time_queries_receive_gradients_and_quaternions_remain_finite():
    model = RecurrentPerceiver(config())
    times = torch.tensor([[[0.1], [0.5], [1.0]]], requires_grad=True)
    poses = model(observations(), queries={"trajectory": QueryRequest(times)}).predictions["trajectory"]
    loss = (poses - torch.randn_like(poses)).square().mean()
    loss.backward()
    assert torch.isfinite(times.grad).all() and times.grad.abs().sum() > 0
    torch.testing.assert_close(poses[..., 3:].norm(dim=-1), torch.ones(1, 3))
    for parameter in model.parameters():
        if parameter.grad is not None:
            assert torch.isfinite(parameter.grad).all()


def test_autocast_keeps_recurrent_state_compatible_across_steps():
    model = RecurrentPerceiver(config())
    with torch.autocast("cpu", dtype=torch.bfloat16):
        first = model(observations())
        second = model(observations(), first.memory.state)
        loss = second.predictions["velocity"].float().square().mean()
    loss.backward()
    assert second.memory.state.tokens.dtype == model.seed.dtype
    assert torch.isfinite(second.memory.state.tokens).all()
    assert torch.isfinite(model.seed.grad).all()


def test_zero_quaternion_uses_identity_and_backward_is_finite():
    model = RecurrentPerceiver(config())
    head = model.decoder.heads["trajectory"].projection[-1]
    with torch.no_grad():
        head.weight.zero_()
        head.bias.zero_()
    poses = model(observations(), queries={"trajectory": QueryRequest(torch.ones(1, 2, 1))})
    quaternion = poses.predictions["trajectory"][..., 3:]
    torch.testing.assert_close(quaternion, torch.tensor([[[0.0, 0.0, 0.0, 1.0], [0.0, 0.0, 0.0, 1.0]]]))
    quaternion.sum().backward()
    assert torch.isfinite(head.weight.grad).all()


def test_checkpoint_round_trip_preserves_predictions(tmp_path):
    model = RecurrentPerceiver(config()).eval()
    checkpoint = tmp_path / "weights.pt"
    torch.save(model.state_dict(), checkpoint)
    restored = RecurrentPerceiver(config()).eval()
    restored.load_state_dict(torch.load(checkpoint, weights_only=True))
    obs = observations()
    torch.testing.assert_close(model(obs).predictions["velocity"], restored(obs).predictions["velocity"])


def test_patches_and_grid_use_documented_coordinate_order():
    image = torch.arange(16.0).reshape(1, 1, 4, 4)
    patches = image_patches(image, 2)
    torch.testing.assert_close(patches.features[0, 0], torch.tensor([0.0, 1.0, 4.0, 5.0]))
    torch.testing.assert_close(
        patches.positions[0], torch.tensor([[-0.5, -0.5], [0.5, -0.5], [-0.5, 0.5], [0.5, 0.5]])
    )
    grid = grid_queries(torch.tensor([[0.0, 2.0], [0.0, 4.0], [0.0, 6.0]]), (3, 2, 2))
    volume = grid.reshape(1, 3, 2, 2, 3)
    torch.testing.assert_close(volume[0, 0, 0, 0], torch.tensor([0.5, 1.0, 1.0]))
    torch.testing.assert_close(volume[0, 2, 1, 1], torch.tensor([1.5, 3.0, 5.0]))


def test_task_context_accepts_global_and_per_query_values():
    definition = config()
    definition.queries["bev"] = replace(definition.queries["bev"], context_dim=2)
    model = RecurrentPerceiver(definition).eval()
    memory = model.encode_step(observations())
    coordinates = torch.randn(1, 5, 3)
    context = torch.randn(1, 2)
    global_value = model.decode(memory.state, {"bev": QueryRequest(coordinates, context)})["bev"]
    local_value = model.decode(
        memory.state,
        {
            "bev": QueryRequest(coordinates, context[:, None].expand(-1, 5, -1)),
        },
        chunk_size=2,
    )["bev"]
    torch.testing.assert_close(global_value, local_value)


@pytest.mark.parametrize(
    "query_requests, message",
    [
        ({"unknown": QueryRequest()}, "未定義のquery"),
        ({"bev": QueryRequest()}, "coordinates"),
        ({"bev": QueryRequest(torch.randn(1, 3, 2))}, "coordinates"),
        ({"velocity": QueryRequest(torch.randn(1, 1, 3))}, "learned query"),
    ],
)
def test_invalid_queries_fail_before_attention(query_requests, message):
    model = RecurrentPerceiver(config())
    with pytest.raises(ValueError, match=message):
        model(observations(), queries=query_requests)


def test_invalid_observation_masks_and_dimensions_are_rejected():
    model = RecurrentPerceiver(config())
    with pytest.raises(ValueError, match="未定義のmodality"):
        model({"unknown": SensorTokens(torch.zeros(1, 1, 3))})
    with pytest.raises(ValueError, match="features"):
        model({"imu": SensorTokens(torch.zeros(1, 1, 5))})
    with pytest.raises(ValueError, match="valid"):
        model({"imu": SensorTokens(torch.zeros(1, 1, 4), valid=torch.ones(1, 1))})
    with pytest.raises(ValueError, match="上限"):
        model({"imu": SensorTokens(torch.zeros(1, 2, 4))})
    with pytest.raises(ValueError, match="initial_state"):
        model({})


def test_empty_coordinate_request_is_supported():
    model = RecurrentPerceiver(config())
    result = model(observations(), queries={"bev": QueryRequest(torch.empty(1, 0, 3))})
    assert result.predictions["bev"].shape == (1, 0, 5)
