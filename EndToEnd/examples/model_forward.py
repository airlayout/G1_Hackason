"""今回のMから、占有・観測済み確率、速度指令、監査用未来軌道を並列に読み出す。"""

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
    # 出力[B,Q,2]は[B,Z,Y,X,2]へreshapeできる。成分順は占有・観測済み確率。
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
            # 学習ではdecode(memory, {"bev": requests["bev"]}, raw=True)で場のlogitsを取得できる。
            state = memory.state
            print(
                json.dumps(
                    {
                        "step": state.step.item(),
                        "recurrent": memory.recurrent.item(),
                        "history_tokens": state.tokens.shape[1],
                        "latents": list(memory.latents.shape),
                        "outputs": {name: list(value.shape) for name, value in predictions.items()},
                        "velocity_command_vx_vy_wz": predictions["velocity"][0, 0].tolist(),
                    }
                )
            )
    print(json.dumps({"parameters": sum(parameter.numel() for parameter in model.parameters())}))


if __name__ == "__main__":
    main()
