"""収集済みHDF5を最新モデルの6入力へ変換し、順にメモリ更新・query読出しする。"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

import torch

from g1_e2e.data.dataset import DatasetReader
from g1_e2e.models import ModelConfig, ModelInputAdapter, QueryRequest, RecurrentPerceiver


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument(
        "--config",
        type=Path,
        default=Path(__file__).resolve().parents[1] / "configs/model/recurrent_perceiver.yaml",
    )
    parser.add_argument("--steps", type=int, default=3)
    args = parser.parse_args()
    if args.steps < 1:
        parser.error("stepsは1以上です")
    torch.set_num_threads(1)
    config = ModelConfig.load(args.config)
    model = RecurrentPerceiver(config).eval()
    memory = model.initial_state(1)
    queries = {
        "velocity": QueryRequest(),
        "bev": QueryRequest(torch.zeros(1, 1, 3)),
        "trajectory": QueryRequest(torch.tensor([[[0.1], [0.5], [1.0]]])),
    }
    episode = None
    with DatasetReader(args.dataset) as reader, torch.inference_mode():
        adapter = ModelInputAdapter(config, control_dt_s=reader.metadata["control_dt_s"])
        for i in range(min(args.steps, len(reader))):
            sample = reader.sample(i)
            current_episode = int(sample["state"]["episode_id"])
            reset = torch.tensor([episode is not None and current_episode != episode])
            inputs = adapter(sample)
            update = model.encode_step(inputs, memory, reset_mask=reset)
            predictions = model.decode(update, queries)
            memory, episode = update.state, current_episode
            print(
                json.dumps(
                    {
                        "row": i,
                        "episode": episode,
                        "inputs": {name: list(value.features.shape) for name, value in inputs.items()},
                        "valid_tokens": {name: int(value.valid.sum()) for name, value in inputs.items()},
                        "outputs": {name: list(value.shape) for name, value in predictions.items()},
                    }
                )
            )


if __name__ == "__main__":
    main()
