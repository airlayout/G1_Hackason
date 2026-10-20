"""合成トークンでモデルのforwardを確認する。物理制御・学習・速度計測は行わない。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from g1_e2e.models import (
    ModelConfig,
    QueryRequest,
    RecurrentPerceiver,
    SensorTokens,
    grid_queries,
)


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "configs/model/recurrent_perceiver.yaml",
    )
    parser.add_argument("--steps", type=int, default=3)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("--stepsは正数です")
    torch.set_num_threads(1)
    torch.manual_seed(0)
    config = ModelConfig.load(args.config)
    model = RecurrentPerceiver(config).eval()
    # 20cm間隔の一例。出力は[B,Z,Y,X,C]にreshapeできる。
    xyz = grid_queries(torch.tensor([[-2.0, 2.0], [-2.0, 2.0], [-0.5, 0.5]]), (5, 20, 20))
    requests = {
        "velocity": QueryRequest(),
        "bev": QueryRequest(xyz),
        "trajectory": QueryRequest(torch.linspace(0.1, 2.0, 20).reshape(1, -1, 1)),
    }
    state = model.initial_state(1)
    with torch.inference_mode():
        for _ in range(args.steps):
            observations = {
                name: SensorTokens(
                    features=torch.randn(1, stream.max_tokens, stream.feature_dim),
                    positions=torch.randn(1, stream.max_tokens, stream.position_dim)
                    if stream.position_dim
                    else None,
                    age_s=torch.zeros(1),
                )
                for name, stream in config.modalities.items()
            }
            # query読出しの頻度は呼び出し側で選べる。stateだけを次周期へ渡す。
            memory = model.encode_step(observations, state)
            predictions = model.decode(memory, requests)
            state = memory.state
            print(
                json.dumps(
                    {
                        "step": state.step.item(),
                        "recurrent": memory.recurrent.item(),
                        "history_tokens": state.tokens.shape[1],
                        "outputs": {name: list(value.shape) for name, value in predictions.items()},
                    }
                )
            )
    print(json.dumps({"parameters": sum(parameter.numel() for parameter in model.parameters())}))


if __name__ == "__main__":
    main()
