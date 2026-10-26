# 段 6 (a)（10-21）: ~/Isaac-GR00T/gr00t/experiment/launch_finetune.py の写し。OMEN（16 GB）向けに 2 か所だけ足した（共用のコードは触らない）。
#   1. 言語モデルの上の層を学習するか（既定の設定は tune_top_llm_layers=4 で、学習するパラメータが 5.3 億個になり 1 歩目でメモリ不足）。
#      環境変数 TUNE_TOP_LLM_LAYERS（既定 0）
#   2. 勾配の checkpointing（GRAD_CKPT=1 で有効。既定は無効）
#   3. 1 を効かせるため、重みの読み込み（setup.py の AutoModel.from_pretrained）を包んで tune_top_llm_layers を渡す
# Launch finetuning for N1.6 on "single node".
# This script tries to provide a similar user experience as current OSS.

import os
from pathlib import Path

import tyro

from gr00t.configs.base_config import get_default_config
from gr00t.configs.finetune_config import FinetuneConfig
from gr00t.experiment.experiment import run


# Make sure the user provided modality config is registered.
def load_modality_config(modality_config_path: str):
    import importlib
    import sys

    path = Path(modality_config_path)
    if path.exists() and path.suffix == ".py":
        sys.path.append(str(path.parent))
        importlib.import_module(path.stem)
        print(f"Loaded modality config: {path}")
    else:
        raise FileNotFoundError(f"Modality config path does not exist: {modality_config_path}")


if __name__ == "__main__":
    # Set LOGURU_LEVEL environment variable if not already set (default: INFO)
    if "LOGURU_LEVEL" not in os.environ:
        os.environ["LOGURU_LEVEL"] = "INFO"
    # Use tyro for clean CLI
    ft_config = tyro.cli(FinetuneConfig, description=__doc__)
    embodiment_tag = ft_config.embodiment_tag.value

    # all rank workers should register for the modality config
    if ft_config.modality_config_path is not None:
        load_modality_config(ft_config.modality_config_path)

    config = get_default_config().load_dict(
        {
            "data": {
                "download_cache": False,
                "datasets": [
                    {
                        "dataset_paths": [ft_config.dataset_path],
                        "mix_ratio": 1.0,
                        "embodiment_tag": embodiment_tag,
                    }
                ],
            }
        }
    )
    config.load_config_path = None

    # overwrite with finetune config supplied by the user
    config.model.tune_llm = ft_config.tune_llm
    config.model.tune_visual = ft_config.tune_visual
    config.model.tune_projector = ft_config.tune_projector
    config.model.tune_diffusion_model = ft_config.tune_diffusion_model
    config.model.state_dropout_prob = ft_config.state_dropout_prob
    config.model.random_rotation_angle = ft_config.random_rotation_angle
    config.model.color_jitter_params = ft_config.color_jitter_params

    config.model.load_bf16 = False
    config.model.reproject_vision = False
    config.model.eagle_collator = True
    config.model.model_name = "nvidia/Eagle-Block2A-2B-v2"
    config.model.backbone_trainable_params_fp32 = True
    config.model.use_relative_action = True
    # OMEN 向け（上の 1・2）
    config.model.tune_top_llm_layers = int(os.environ.get("TUNE_TOP_LLM_LAYERS", "0"))
    config.training.gradient_checkpointing = os.environ.get("GRAD_CKPT", "0") == "1"
    print(f"[omen] tune_top_llm_layers={config.model.tune_top_llm_layers} gradient_checkpointing={config.training.gradient_checkpointing}")

    config.training.start_from_checkpoint = ft_config.base_model_path
    config.training.optim = "adamw_torch"
    config.training.global_batch_size = ft_config.global_batch_size
    config.training.dataloader_num_workers = ft_config.dataloader_num_workers
    config.training.learning_rate = ft_config.learning_rate
    config.training.gradient_accumulation_steps = ft_config.gradient_accumulation_steps
    config.training.output_dir = ft_config.output_dir
    config.training.save_steps = ft_config.save_steps
    config.training.save_total_limit = ft_config.save_total_limit
    config.training.num_gpus = ft_config.num_gpus
    config.training.use_wandb = ft_config.use_wandb
    config.training.max_steps = ft_config.max_steps
    config.training.weight_decay = ft_config.weight_decay
    config.training.warmup_ratio = ft_config.warmup_ratio
    config.training.wandb_project = "finetune-gr00t-n1d6"

    config.data.shard_size = ft_config.shard_size
    config.data.episode_sampling_rate = ft_config.episode_sampling_rate
    config.data.num_shards_per_epoch = ft_config.num_shards_per_epoch

    # OMEN 向け: 重みから始めると setup.py が AutoModel.from_pretrained に一部の tune_* しか渡さず、
    # tune_top_llm_layers は重みに付いた設定（4）のまま使われる（10-21 に 0 を入れても 5.3 億個のままだった）。
    # 読み込みの関数を包んで、tune_top_llm_layers も渡す（HF の from_pretrained は設定にある名前の引数で設定を上書きする）
    import gr00t.model.gr00t_n1d6.setup as _setup

    _orig_auto_model = _setup.AutoModel

    class _AutoModelOmen:
        @staticmethod
        def from_pretrained(*args, **kwargs):
            kwargs.setdefault("tune_top_llm_layers", config.model.tune_top_llm_layers)
            return _orig_auto_model.from_pretrained(*args, **kwargs)

    _setup.AutoModel = _AutoModelOmen
    run(config)
