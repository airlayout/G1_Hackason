"""3出力の物理的な意味、Mへの学習経路、head間に接続がないことを検証する。"""

from dataclasses import replace

import pytest

from .support import config, observations, torch
from g1_e2e.models import QueryRequest, RecurrentPerceiver


def requests(batch_size=1):
    return {
        "velocity": QueryRequest(),
        "bev": QueryRequest(torch.randn(batch_size, 5, 3)),
        "trajectory": QueryRequest(torch.tensor([0.1, 0.5, 1.0]).reshape(1, 3, 1).expand(batch_size, -1, -1)),
    }


def test_velocity_head_cannot_change_field_or_trajectory_predictions():
    model = RecurrentPerceiver(config()).eval()
    memory = model.encode_step(observations())
    queries = requests()
    reference = model.decode(memory, queries)
    with torch.no_grad():
        for parameter in model.decoder.heads["velocity"].parameters():
            parameter.fill_(float("nan"))
        for parameter in model.decoder.sources["velocity"].parameters():
            parameter.fill_(float("nan"))
    changed = model.decode(memory, dict(reversed(list(queries.items()))))
    assert torch.isnan(changed["velocity"]).all()
    for name in ["bev", "trajectory"]:
        torch.testing.assert_close(reference[name], changed[name])
        alone = model.decode(memory, {name: queries[name]})[name]
        torch.testing.assert_close(reference[name], alone)


@pytest.mark.parametrize("task", ["velocity", "bev", "trajectory"])
def test_each_head_trains_current_m_and_observations_but_not_other_heads(task):
    model = RecurrentPerceiver(config())
    obs = observations(requires_grad=True)
    memory = model.encode_step(obs)
    memory.latents.retain_grad()
    output = model.decode(memory, {task: requests()[task]})[task]
    (output * torch.randn_like(output)).sum().backward()
    assert torch.isfinite(memory.latents.grad).all() and memory.latents.grad.abs().sum() > 0
    assert obs["points"].features.grad.abs().sum() > 0
    for name, head in model.decoder.heads.items():
        if name == task:
            assert all(parameter.grad is not None for parameter in head.parameters())
            assert all(torch.isfinite(parameter.grad).all() for parameter in head.parameters())
        else:
            assert all(parameter.grad is None for parameter in head.parameters())
            assert all(parameter.grad is None for parameter in model.decoder.sources[name].parameters())


def test_neural_field_probabilities_and_training_logits_describe_the_same_field():
    model = RecurrentPerceiver(config()).eval()
    queries = {"bev": requests()["bev"]}
    obs = observations(requires_grad=True)
    memory = model.encode_step(obs)
    probabilities = model.decode(memory, queries)["bev"]
    logits = model.decode(memory, queries, raw=True)["bev"]
    torch.testing.assert_close(probabilities, logits.sigmoid())
    # [占有, 観測済み]を独立に学習でき、未観測点は占有lossから除外できる。
    observed = torch.tensor([[True, True, False, False, True]])
    targets = torch.tensor([[1.0, 0.0, float("nan"), float("nan"), 1.0]])
    loss = torch.nn.functional.binary_cross_entropy_with_logits(logits[..., 0][observed], targets[observed])
    loss += torch.nn.functional.binary_cross_entropy_with_logits(logits[..., 1], observed.float())
    loss.backward()
    assert torch.isfinite(loss)
    assert torch.isfinite(obs["points"].features.grad).all()
    assert obs["points"].features.grad.abs().sum() > 0


@pytest.mark.parametrize("sign", [-1, 1])
def test_velocity_command_saturates_at_the_configured_asymmetric_limits(sign):
    model = RecurrentPerceiver(config()).eval()
    projection = model.decoder.heads["velocity"].projection[-1]
    with torch.no_grad():
        projection.weight.zero_()
        projection.bias.fill_(sign * 1000)
    memory = model.encode_step(observations())
    command = model.decode(memory)["velocity"]
    limits = model.config.queries["velocity"]
    expected = limits.output_min if sign < 0 else limits.output_max
    torch.testing.assert_close(command, torch.tensor(expected).reshape(1, 1, 3))


def test_field_coordinate_gradient_and_trajectory_chunking_are_preserved():
    model = RecurrentPerceiver(config()).eval()
    memory = model.encode_step(observations())
    xyz = torch.randn(1, 4, 3, requires_grad=True)
    probabilities = model.decode(memory, {"bev": QueryRequest(xyz)})["bev"]
    probabilities[..., 0].sum().backward()
    assert torch.isfinite(xyz.grad).all() and xyz.grad.abs().sum() > 0
    query = requests()["trajectory"]
    full = model.decode(memory, {"trajectory": query})["trajectory"]
    split = model.decode(memory, {"trajectory": query}, chunk_size=1)["trajectory"]
    torch.testing.assert_close(full, split)
    permutation = torch.tensor([2, 0, 1])
    permuted = model.decode(memory, {"trajectory": QueryRequest(query.coordinates[:, permutation])})
    torch.testing.assert_close(full[:, permutation], permuted["trajectory"])


def test_raw_forward_exposes_logits_without_changing_the_generated_m():
    model = RecurrentPerceiver(config()).eval()
    obs, query = observations(), {"bev": requests()["bev"]}
    normal, raw = model(obs, queries=query), model(obs, queries=query, raw=True)
    torch.testing.assert_close(normal.latents, raw.latents)
    torch.testing.assert_close(normal.predictions["bev"], raw.predictions["bev"].sigmoid())


@pytest.mark.parametrize("value", [float("nan"), float("inf"), -float("inf")])
@pytest.mark.parametrize("task, dimensions", [("bev", 3), ("trajectory", 1)])
def test_nonfinite_query_coordinates_are_rejected(value, task, dimensions):
    model = RecurrentPerceiver(config()).eval()
    with pytest.raises(ValueError, match="有限"):
        model(observations(), queries={task: QueryRequest(torch.full((1, 1, dimensions), value))})


def test_negative_future_time_is_rejected_and_zero_is_supported():
    model = RecurrentPerceiver(config()).eval()
    with pytest.raises(ValueError, match="未来時刻"):
        model(observations(), queries={"trajectory": QueryRequest(torch.tensor([[[-0.1]]]))})
    result = model(observations(), queries={"trajectory": QueryRequest(torch.zeros(1, 1, 1))})
    assert torch.isfinite(result.predictions["trajectory"]).all()


@pytest.mark.parametrize(
    "task, changes",
    [
        ("velocity", {"output_min": ()}),
        ("velocity", {"output_max": (-1.0, 0.3, 0.2)}),
        ("velocity", {"output_min": (-float("inf"), -0.3, -0.2)}),
        ("bev", {"output_dim": 32}),
        ("bev", {"coordinate_dim": 2}),
        ("trajectory", {"coordinate_dim": 3}),
        ("trajectory", {"output_dim": 6}),
    ],
)
def test_invalid_physical_output_configuration_is_rejected(task, changes):
    with pytest.raises(ValueError):
        replace(config().queries[task], **changes)
