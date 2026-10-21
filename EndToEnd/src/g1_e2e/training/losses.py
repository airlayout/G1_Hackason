"""速度・場・監査軌道を、今回のMから独立に読み出して教師と比較する。"""

import numpy as np
import torch
import torch.nn.functional as F

from g1_e2e.models import QueryRequest


def losses(model, memory, target, trajectory_times):
    device, batch_size = memory.latents.device, memory.latents.shape[0]

    def tensor(value):
        return torch.as_tensor(np.asarray(value).copy(), device=device)

    requests = {"velocity": QueryRequest()}
    if target["auxiliary"]:
        times = tensor(trajectory_times)[None, :, None].float().expand(batch_size, -1, -1)
        requests["trajectory"] = QueryRequest(times)
    predictions = model.decode(memory, requests)
    reference = tensor(target["velocity"]).float()
    velocity = predictions["velocity"][:, 0].float()
    valid_velocity = tensor(target["velocity_valid"]).bool()
    head = model.config.queries["velocity"]
    scale = velocity.new_tensor(head.output_max) - velocity.new_tensor(head.output_min)
    error = F.smooth_l1_loss(velocity / scale, reference / scale, beta=0.1, reduction="none")
    velocity_loss = (error * valid_velocity[:, None]).sum() / (3 * valid_velocity.sum()).clamp_min(1)
    result = {"velocity": velocity_loss}
    error = (velocity.detach() - reference).abs()
    metrics = {
        "velocity_normalized_mae": (error / scale).mean(),
        "vx_mae_m_s": error[:, 0].mean(),
        "vy_mae_m_s": error[:, 1].mean(),
        "yaw_rate_mae_rad_s": error[:, 2].mean(),
    }
    if target["auxiliary"]:
        coordinates = tensor(target["coordinates"]).float()
        logits = model.decode(memory, {"bev": QueryRequest(coordinates)}, raw=True)["bev"].float()
        observed, occupied = tensor(target["observed"]).float(), tensor(target["occupied"]).float()
        known = observed.bool()
        occupancy = F.binary_cross_entropy_with_logits(logits[..., 0], occupied, reduction="none")
        occupancy = (occupancy * known).sum() / known.sum().clamp_min(1)
        result["field"] = occupancy + F.binary_cross_entropy_with_logits(logits[..., 1], observed)
        metrics["observed_sampled_accuracy"] = ((logits[..., 1].detach() >= 0) == known).float().mean()
        if known.any():
            metrics["occupancy_sampled_accuracy"] = (
                ((logits[..., 0].detach() >= 0) == occupied.bool()) & known
            ).float().sum() / known.sum()
        valid = tensor(target["pose_valid"]).bool()
        poses, reference_poses = predictions["trajectory"].float(), tensor(target["poses"]).float()
        position_error = F.smooth_l1_loss(poses[..., :3], reference_poses[..., :3], reduction="none").mean(-1)
        quaternion = F.normalize(poses[..., 3:], dim=-1)
        rotation_error = 1 - (quaternion * reference_poses[..., 3:]).sum(-1).square().clamp_max(1)
        result["trajectory"] = ((position_error + rotation_error) * valid).sum() / valid.sum().clamp_min(1)
        metrics["trajectory_ade_m"] = (
            (poses[..., :3] - reference_poses[..., :3]).norm(dim=-1) * valid
        ).sum().detach() / valid.sum().clamp_min(1)
    return result, metrics
