"""収集済みデータの6入力・3教師・実モデルforwardと損失計算をCPUで照合する。"""

import argparse
import json
from pathlib import Path

import numpy as np
import torch
import torch.nn.functional as F

from g1_e2e.data.dataset import DatasetReader
from g1_e2e.data.targets import SupervisionReader
from g1_e2e.models import ModelConfig, ModelInputAdapter, QueryRequest, RecurrentPerceiver


def check(path, config, *, index=None):
    torch.set_num_threads(1)
    with DatasetReader(path) as reader:
        if not reader.file.attrs.get("input_integrity_accepted", True):
            raise ValueError(
                "センサー入力の監査で不採用になったデータです。parallel-audit.jsonを確認してください"
            )
        if not reader.file.attrs.get("patrol_training_accepted", True):
            raise ValueError("巡回品質の監査で不採用になったデータです。summary.jsonを確認してください")
        if len(reader) < 2:
            raise ValueError("教師を検証するには2行以上のデータが必要です")
        index = min(25, max(5, len(reader) - 52), len(reader) - 2) if index is None else index
        sample = reader.sample(index)
        inputs = ModelInputAdapter(config, control_dt_s=reader.metadata["control_dt_s"])(sample)
        if set(inputs) != set(config.modalities):
            raise ValueError("モデルの全入力がありません")
        if any(not tokens.valid.any() for tokens in inputs.values()):
            raise ValueError("検証行の入力に欠測があります。LiDAR scan完成後の行を選んでください")
        source = SupervisionReader(reader)
        velocity, velocity_valid = source.velocity(index)
        if not velocity_valid:
            raise ValueError("速度指令の教師がありません。実速度で代用しません")
        trajectory = source.trajectory(index, [0.1, 0.5, 1.0])
        if not trajectory.valid.any():
            raise ValueError("同一episode内の未来軌道がありません")
        field = source.field(
            index,
            count=384,
            history_seconds=config.context_window * config.recurrence_period * config.control_dt_s,
        )
        if not field.occupied.any() or not (field.occupancy_valid & (field.occupied == 0)).any():
            raise ValueError("有効rayから表面・freeの両方の教師を作れません")
        model = RecurrentPerceiver(config).eval()
        queries = {
            "velocity": QueryRequest(),
            "bev": QueryRequest(torch.from_numpy(field.coordinates[None])),
            "trajectory": QueryRequest(torch.from_numpy(trajectory.times_s[None, :, None])),
        }
        with torch.inference_mode():
            memory = model.encode_step(inputs)
            predictions = model.decode(memory, queries)
            logits = model.decode(memory, {"bev": queries["bev"]}, raw=True)["bev"][0]
            known = torch.from_numpy(field.occupancy_valid)
            field_loss = F.binary_cross_entropy_with_logits(
                logits[known, 0], torch.from_numpy(field.occupied)[known]
            )
            field_loss += F.binary_cross_entropy_with_logits(logits[:, 1], torch.from_numpy(field.observed))
            velocity_loss = F.smooth_l1_loss(predictions["velocity"][0, 0], torch.from_numpy(velocity))
            valid = torch.from_numpy(trajectory.valid)
            poses = predictions["trajectory"][0, valid]
            targets = torch.from_numpy(trajectory.poses)[valid]
            trajectory_loss = F.smooth_l1_loss(poses[:, :3], targets[:, :3])
            trajectory_loss += (1 - (poses[:, 3:] * targets[:, 3:]).sum(-1).square()).mean()
        losses = {
            "field": float(field_loss),
            "velocity_command": float(velocity_loss),
            "trajectory": float(trajectory_loss),
        }
        if not all(np.isfinite(value) for value in losses.values()):
            raise ValueError("実モデルの損失に非有限値があります")
        result = {
            "accepted": True,
            "dataset": str(path.resolve()),
            "rows": len(reader),
            "checked_index": index,
            "schema_version": int(reader.file.attrs["schema_version"]),
            "recording_complete": bool(reader.file.attrs["complete"]),
            "inputs": {name: list(tokens.features.shape) for name, tokens in inputs.items()},
            "input_valid_tokens": {name: int(tokens.valid.sum()) for name, tokens in inputs.items()},
            "outputs": {name: list(value.shape) for name, value in predictions.items()},
            "velocity_command_vx_vy_wz": velocity.tolist(),
            "trajectory_valid": trajectory.valid.tolist(),
            "field_target_counts": {
                "occupied": int(field.occupied.sum()),
                "free": int((field.occupancy_valid & (field.occupied == 0)).sum()),
                "unknown": int((field.observed == 0).sum()),
            },
            "finite_losses_with_untrained_model": losses,
            "future_data_is_only_trajectory_label": True,
        }
    return result


def main():
    root = Path(__file__).resolve().parents[1]
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("dataset", type=Path)
    parser.add_argument("--model-config", type=Path, default=root / "configs/model/recurrent_perceiver.yaml")
    parser.add_argument("--index", type=int)
    args = parser.parse_args()
    report = args.dataset.parent / "dataset-audit.json"
    try:
        result = check(args.dataset, ModelConfig.load(args.model_config), index=args.index)
    except Exception as error:
        report.write_text(
            json.dumps(
                {"accepted": False, "dataset": str(args.dataset.resolve()), "error": str(error)},
                ensure_ascii=False,
                indent=2,
            )
            + "\n"
        )
        raise
    report.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
