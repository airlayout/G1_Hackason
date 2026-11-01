"""セットアップ・GPU診断・一度だけの実推論。実機へ指令を送らない。"""

from __future__ import annotations

import argparse
import json
import shutil
import time

import yaml

from .config import DEFAULT_CONFIG, ROOT, load_config


def main() -> None:
    parser = argparse.ArgumentParser(description="VLAモデルの準備・診断")
    parser.add_argument("command", choices=["download", "doctor", "benchmark"])
    parser.add_argument("--config", default=str(DEFAULT_CONFIG))
    parser.add_argument("--device")
    parser.add_argument("--robot", default="http://127.0.0.1:8766")
    parser.add_argument("--instruction")
    args = parser.parse_args()
    config = load_config(args.config)
    if args.device:
        config["device"] = args.device
    if args.command == "download":
        from .policy import download_model

        download_model(config)
        return
    if args.command == "doctor":
        import torch
        from .policy import model_directory

        directory = model_directory(config)
        model_error = ""
        if config["backend"] == "pi05":
            from .backends.pi05 import validate_runtime_files

            try:
                validate_runtime_files(directory)
            except (ValueError, OSError, KeyError) as error:
                model_error = str(error)
            model_ready = not model_error
        else:
            model_ready = (directory / "processor_config.json").is_file()
        print(
            json.dumps(
                {
                    "torch": torch.__version__,
                    "cuda": torch.version.cuda,
                    "available": torch.cuda.is_available(),
                    "gpus": [
                        {
                            "id": i,
                            "name": torch.cuda.get_device_name(i),
                            "capability": torch.cuda.get_device_capability(i),
                            "free_gb": round(torch.cuda.mem_get_info(i)[0] / 1024**3, 2),
                        }
                        for i in range(torch.cuda.device_count())
                    ],
                    "disk_free_gb": round(shutil.disk_usage(ROOT).free / 1024**3, 2),
                    "backend": config["backend"],
                    "model": config["model_id"],
                    "model_ready": model_ready,
                    "model_error": model_error,
                },
                indent=2,
            )
        )
        return
    import os
    from .http import ApiClient
    from .policy import create_policy, create_adapter

    client = ApiClient(args.robot, token=os.environ.get("G1_ARM_TOKEN", ""))
    names = yaml.safe_load((ROOT / "configs/robot/g1_29dof.yaml").read_text())["joint_names"]
    adapter = create_adapter(config, names)
    policy = create_policy(config)
    policy.load()
    for _ in range(9):
        observation = client.request("/api/observation")
        adapter.record(observation)
        time.sleep(0.1)
    begin = time.monotonic()
    instruction = args.instruction or config.get("default_instruction", "Move the blue box.")
    chunk = policy.predict(adapter.build(observation, instruction))
    print(
        json.dumps(
            {
                **policy.metrics(),
                "inference_ms": round((time.monotonic() - begin) * 1000),
                "shape": list(chunk.shape),
                "arm_target_rad": chunk[0].tolist(),
                "robot_commands_sent": 0,
            },
            indent=2,
        )
    )


if __name__ == "__main__":
    main()
