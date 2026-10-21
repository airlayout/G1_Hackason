"""episodeの順序を守るTBPTT学習。M履歴を保持し、区間境界だけで勾配を切る。"""

from contextlib import nullcontext
from dataclasses import asdict, dataclass
import json
from pathlib import Path
import time

import numpy as np
import torch

from g1_e2e.models import ModelConfig, RecurrentPerceiver
from .cache import TrainingBatch
from .losses import losses


@dataclass(frozen=True)
class TrainingConfig:
    epochs: int = 1
    batch_size: int = 4
    unroll_steps: int = 8
    accumulation: int = 4
    learning_rate: float = 0.0003
    field_weight: float = 0.2
    trajectory_weight: float = 0.2
    seed: int = 42


class Statistics:
    def __init__(self):
        self.sums, self.counts = {}, {}

    def add(self, values):
        for name, value in values.items():
            self.sums[name] = self.sums.get(name, 0) + value.detach()
            self.counts[name] = self.counts.get(name, 0) + 1

    def result(self):
        return {name: float(value / self.counts[name]) for name, value in self.sums.items()}


class Trainer:
    def __init__(self, manifest, output, config, *, device="cuda:1", resume=None, initialize=None):
        self.manifest, self.output, self.config = manifest, Path(output), config
        self.device = torch.device(device)
        if self.device.type == "cuda":
            torch.cuda.set_device(self.device)
        torch.set_num_threads(1)
        torch.manual_seed(config.seed)
        np.random.seed(config.seed)
        self.model_config = ModelConfig.load(Path(manifest["model_config"]))
        self.model = RecurrentPerceiver(self.model_config).to(self.device)
        self.model.gradient_checkpointing = True
        self.optimizer = torch.optim.AdamW(
            self.model.parameters(), lr=config.learning_rate, weight_decay=0.01
        )
        self.update, self.start_epoch, self.start_episode = 0, 0, 0
        self.best_score = float("inf")
        self.amp = self.device.type == "cuda" and torch.cuda.is_bf16_supported()
        if initialize:
            saved = torch.load(initialize, map_location=self.device, weights_only=False)
            if saved["model_config"] != asdict(self.model_config):
                raise ValueError("初期重みのモデル構成が一致しません")
            self.model.load_state_dict(saved["model"])
        if resume:
            saved = torch.load(resume, map_location=self.device, weights_only=False)
            if saved["model_config"] != asdict(self.model_config):
                raise ValueError("再開checkpointのモデル構成が一致しません")
            if saved["manifest"] != manifest:
                raise ValueError("再開checkpointのデータmanifestが一致しません")
            for key in ("batch_size", "unroll_steps", "accumulation"):
                if saved["training_config"].get(key, 1) != getattr(config, key):
                    raise ValueError(
                        f"再開checkpointの{key}が一致しません。変更にはinitializeを使ってください"
                    )
            self.model.load_state_dict(saved["model"])
            self.optimizer.load_state_dict(saved["optimizer"])
            self.update, self.start_epoch, self.start_episode = (
                saved["update"],
                saved["next_epoch"],
                saved["next_episode"],
            )
            self.best_score = saved["best_score"]
            torch.set_rng_state(saved["torch_rng"].cpu())
            if self.device.type == "cuda" and saved["cuda_rng"] is not None:
                torch.cuda.set_rng_state(saved["cuda_rng"].cpu(), self.device)
        self.log = (self.output / "updates.jsonl").open("a")
        self.resuming = resume is not None
        self.started = time.perf_counter()

    def autocast(self):
        return torch.autocast("cuda", dtype=torch.bfloat16) if self.amp else nullcontext()

    def save(self, name, next_epoch, next_episode):
        path = self.output / name
        temporary = path.with_suffix(".partial.pt")
        torch.save(
            {
                "model": self.model.state_dict(),
                "optimizer": self.optimizer.state_dict(),
                "model_config": asdict(self.model_config),
                "training_config": asdict(self.config),
                "manifest": self.manifest,
                "update": self.update,
                "best_score": self.best_score,
                "next_epoch": next_epoch,
                "next_episode": next_episode,
                "torch_rng": torch.get_rng_state(),
                "cuda_rng": torch.cuda.get_rng_state(self.device) if self.device.type == "cuda" else None,
            },
            temporary,
        )
        temporary.replace(path)

    def episode(self, path, *, training):
        paths = [path] if isinstance(path, (str, Path)) else path
        episode = TrainingBatch(paths, self.device)
        if json.dumps(episode.metadata["model_config"], sort_keys=True) != json.dumps(
            asdict(self.model_config), sort_keys=True
        ):
            episode.close()
            raise ValueError("学習cacheとモデルの構成が一致しません")
        statistics = Statistics()
        self.model.train(training)
        state = self.model.initial_state(episode.batch_size)
        pending = 0
        try:
            with torch.enable_grad() if training else torch.inference_mode():
                for start in range(0, len(episode), self.config.unroll_steps):
                    grouped = {}
                    with self.autocast():
                        for index in range(start, min(start + self.config.unroll_steps, len(episode))):
                            memory = self.model.encode_step(episode.tokens(index), state)
                            state = memory.state
                            values, metrics = losses(
                                self.model,
                                memory,
                                episode.target(index),
                                episode.metadata["trajectory_times_s"],
                            )
                            statistics.add({**values, **metrics})
                            for key, value in values.items():
                                grouped.setdefault(key, []).append(value)
                        means = {key: torch.stack(value).mean() for key, value in grouped.items()}
                        total = (
                            means["velocity"]
                            + self.config.field_weight * means.get("field", 0)
                            + self.config.trajectory_weight * means.get("trajectory", 0)
                        )
                    if not torch.isfinite(total):
                        raise ValueError("学習損失に非有限値があります")
                    if training:
                        (total / self.config.accumulation).backward()
                        pending += 1
                        if pending == self.config.accumulation or start + self.config.unroll_steps >= len(
                            episode
                        ):
                            if pending < self.config.accumulation:
                                for parameter in self.model.parameters():
                                    if parameter.grad is not None:
                                        parameter.grad.mul_(self.config.accumulation / pending)
                            norm = torch.nn.utils.clip_grad_norm_(
                                self.model.parameters(), 1.0, error_if_nonfinite=True
                            )
                            lr = self.config.learning_rate * min(1.0, (self.update + 1) / 20)
                            for group in self.optimizer.param_groups:
                                group["lr"] = lr
                            self.optimizer.step()
                            self.optimizer.zero_grad(set_to_none=True)
                            self.update += 1
                            pending = 0
                            report = {
                                "update": self.update,
                                "episodes": [str(value) for value in paths],
                                "batch_size": episode.batch_size,
                                "control_step": min(start + self.config.unroll_steps, len(episode)),
                                "history_tokens": state.tokens.shape[1],
                                "loss": float(total.detach()),
                                "gradient_norm": float(norm),
                                "lr": lr,
                                "wall_seconds": time.perf_counter() - self.started,
                            }
                            self.log.write(json.dumps(report) + "\n")
                            self.log.flush()
                            if self.update % 10 == 0:
                                print(f"[TRAIN] {json.dumps(report)}", flush=True)
                    state = state.detach()
        finally:
            episode.close()
        return statistics.result()

    def batches(self, split, epoch=None):
        buckets = {}
        for record in self.manifest["splits"][split]:
            buckets.setdefault(record["rows"], []).append(record)
        rng = np.random.default_rng(self.config.seed + (epoch or 0))
        groups = []
        for records in buckets.values():
            if epoch is not None:
                rng.shuffle(records)
            groups.extend(
                records[start : start + self.config.batch_size]
                for start in range(0, len(records), self.config.batch_size)
            )
        if epoch is not None:
            rng.shuffle(groups)
        return groups

    def evaluate(self, split):
        values = []
        for records in self.batches(split):
            paths = [record["cache"] for record in records]
            metrics = self.episode(paths, training=False)
            values.append({"caches": paths, "episode_count": len(paths), **metrics})
            print(f"[EVAL] {split}: {json.dumps(values[-1])}", flush=True)
        total = sum(item["episode_count"] for item in values)
        aggregate = {
            name: float(sum(item[name] * item["episode_count"] for item in values) / total)
            for name in values[0]
            if name not in ("caches", "episode_count")
        }
        aggregate["score"] = (
            aggregate["velocity"]
            + self.config.field_weight * aggregate.get("field", 0)
            + self.config.trajectory_weight * aggregate.get("trajectory", 0)
        )
        return {"aggregate": aggregate, "episode_batches": values, "aggregation": "episode macro average"}

    def run(self):
        initial_path = self.output / "initial-validation.json"
        if self.resuming and initial_path.exists():
            initial = json.loads(initial_path.read_text())
        else:
            initial = self.evaluate("validation")
            initial_path.write_text(json.dumps(initial, indent=2) + "\n")
        epochs_path = self.output / "epochs.json"
        epochs = json.loads(epochs_path.read_text()) if self.resuming and epochs_path.exists() else []
        for epoch in range(self.start_epoch, self.config.epochs):
            groups = self.batches("train", epoch)
            train = []
            for index, records in enumerate(groups):
                if epoch == self.start_epoch and index < self.start_episode:
                    continue
                metrics = self.episode([record["cache"] for record in records], training=True)
                train.append({"caches": [record["cache"] for record in records], **metrics})
                self.save("last.pt", epoch, index + 1)
                print(f"[EPISODE] {index + 1}/{len(groups)} {json.dumps(train[-1])}", flush=True)
            validation = self.evaluate("validation")
            if validation["aggregate"]["score"] < self.best_score:
                self.best_score = validation["aggregate"]["score"]
                self.save("best.pt", epoch + 1, 0)
            self.save("last.pt", epoch + 1, 0)
            epochs.append({"epoch": epoch + 1, "training": train, "validation": validation})
            (self.output / "epochs.json").write_text(json.dumps(epochs, indent=2) + "\n")
        best = torch.load(self.output / "best.pt", map_location=self.device, weights_only=False)
        self.model.load_state_dict(best["model"])
        test = self.evaluate("test")
        report = {
            "complete": True,
            "updates": self.update,
            "parameters": sum(p.numel() for p in self.model.parameters()),
            "training_config": asdict(self.config),
            "model_config": asdict(self.model_config),
            "bf16": self.amp,
            "initial_validation": initial,
            "epochs": epochs,
            "test": test,
            "wall_seconds": time.perf_counter() - self.started,
            "cuda_peak_memory_bytes": torch.cuda.max_memory_allocated(self.device)
            if self.device.type == "cuda"
            else None,
        }
        (self.output / "summary.json").write_text(json.dumps(report, indent=2) + "\n")
        self.log.close()
        print(
            f"[TRAINED] {json.dumps({key: value for key, value in report.items() if key not in ('epochs', 'model_config', 'initial_validation', 'test')})}",
            flush=True,
        )
        return report
