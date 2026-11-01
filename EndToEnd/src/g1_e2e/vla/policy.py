"""設定に対応する観測変換と推論器を、GPUを初期化せず選択する。"""

from __future__ import annotations

from pathlib import Path

from .config import ROOT


def model_directory(config: dict) -> Path:
    path = Path(config["model_directory"])
    return path if path.is_absolute() else ROOT / path


def create_adapter(config: dict, joint_names: list[str]):
    if config["backend"] == "pi05":
        from .backends.pi05 import Pi05ObservationAdapter

        return Pi05ObservationAdapter(config)
    from .backends.groot import ObservationAdapter

    return ObservationAdapter(config, joint_names)


def create_policy(config: dict, model_path: str | None = None):
    if config["backend"] == "pi05":
        from .backends.pi05 import Pi05Policy

        return Pi05Policy(config, model_path or str(model_directory(config)))
    from .backends.groot import GrootPolicy

    return GrootPolicy(config, model_path or str(model_directory(config)))


def download_model(config: dict) -> None:
    if config["backend"] == "pi05":
        from .backends.pi05 import download_model as download
    else:
        from .backends.groot import download_model as download
    download(config)
