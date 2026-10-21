"""今回のMから速度指令を読み出し、公開の低位歩行方策へ渡す。"""

from dataclasses import asdict
import hashlib

import torch

from g1_e2e.models import ModelConfig, ModelInputAdapter, QueryRequest, RecurrentPerceiver


class MemoryVelocityPolicy:
    def __init__(self, checkpoint, device):
        self.device = torch.device(device)
        if self.device.type == "cuda":
            torch.cuda.set_device(self.device)
        saved = torch.load(checkpoint, map_location=self.device, weights_only=True)
        self.config = ModelConfig.from_dict(saved["model_config"])
        self.model = RecurrentPerceiver(self.config).to(self.device).eval()
        self.model.load_state_dict(saved["model"])
        self.adapter = ModelInputAdapter(self.config, device=str(self.device))
        self.amp = self.device.type == "cuda" and torch.cuda.is_bf16_supported()
        self.metadata = {
            "checkpoint": str(checkpoint.resolve()),
            "sha256": hashlib.sha256(checkpoint.read_bytes()).hexdigest(),
            "model_config": asdict(self.config),
            "inputs": list(self.config.modalities),
            "readout": "velocity from current M only",
        }
        self.reset()

    def reset(self):
        self.state = self.model.initial_state(1)

    @torch.inference_mode()
    def command(self, sample):
        with torch.autocast(self.device.type, dtype=torch.bfloat16, enabled=self.amp):
            memory = self.model.encode_step(self.adapter(sample), self.state)
            velocity = self.model.decode(memory, {"velocity": QueryRequest()})["velocity"][0, 0]
        self.state = memory.state
        return velocity.float().cpu().numpy()
