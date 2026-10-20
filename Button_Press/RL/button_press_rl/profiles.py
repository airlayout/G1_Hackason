"""学習の設定（報酬の重み・乱し・試行の設定）を、各自の YAML（configs/profiles/<名前>.yaml）で変える。

    bash Button_Press/RL/scripts/train.sh --profile yada        # configs/profiles/yada.yaml で学習する
    BUTTON_PRESS_PROFILE=yada bash Button_Press/RL/scripts/eval_teacher.sh   # 評価も同じ乱しで

YAML の節:
- rewards: 報酬の重みと尺度（rewards.RewardWeights）
- rand:    乱し（press_env_cfg.DomainRandCfg）
- env:     試行の設定のうち、変えても観測・行動の形が変わらないもの（ENV_KEYS）

書いた項目だけが変わり、書かなかった項目は既定の値（= configs/profiles/default.yaml）のまま。
存在しない項目（打ち間違いなど）があれば、学習を始める前にエラーにする（黙って無視すると、変えたつもりで変わっていない）。
"""

from __future__ import annotations

import dataclasses
import os
from pathlib import Path
from typing import Any

import yaml

from .paths import CONFIG_DIR

PROFILE_DIR = CONFIG_DIR / "profiles"
ENV_VAR = "BUTTON_PRESS_PROFILE"
SECTIONS = ("rewards", "rand", "env")
# env の節で変えてよい項目（観測・行動の形や、評価と合わせている物理には触れないもの）
ENV_KEYS = ("episode_length_s", "terminate_on_failure", "force_abort_scale", "tip_gap_m", "tip_lateral_m")


class ProfileError(ValueError):
    pass


def profile_path(name_or_path: str) -> Path:
    """名前（configs/profiles/<名前>.yaml）か、YAML のパス。"""
    p = Path(name_or_path)
    if p.suffix in (".yaml", ".yml") or p.is_file():
        return p
    return PROFILE_DIR / f"{name_or_path}.yaml"


def profile_name(name_or_path: str) -> str:
    return profile_path(name_or_path).stem


def load_profile(name_or_path: str) -> dict[str, dict[str, Any]]:
    path = profile_path(name_or_path)
    if not path.exists():
        have = sorted(p.stem for p in PROFILE_DIR.glob("*.yaml"))
        raise ProfileError(f"学習の設定が無い: {path}（あるのは {have}。configs/profiles/default.yaml をコピーして作る）")
    data = yaml.safe_load(path.read_text(encoding="utf-8")) or {}
    if not isinstance(data, dict):
        raise ProfileError(f"{path}: 中身が辞書ではない")
    unknown = [k for k in data if k not in SECTIONS]
    if unknown:
        raise ProfileError(f"{path}: 知らない節 {unknown}（使えるのは {list(SECTIONS)}）")
    return {k: (data.get(k) or {}) for k in SECTIONS}


def _set(obj: Any, section: str, key: str, value: Any, allowed: tuple[str, ...] | None, path: Path) -> None:
    names = allowed if allowed is not None else tuple(f.name for f in dataclasses.fields(obj))
    if key not in names:
        raise ProfileError(f"{path}: {section}.{key} という項目は無い（使えるのは {sorted(names)}）")
    old = getattr(obj, key)
    if isinstance(old, tuple):
        if not isinstance(value, (list, tuple)) or len(value) != len(old):
            raise ProfileError(f"{path}: {section}.{key} は {len(old)} 個の値の並び（例 {list(old)}）")
        value = tuple(type(o)(v) for o, v in zip(old, value))
    elif isinstance(old, bool):
        if not isinstance(value, bool):
            raise ProfileError(f"{path}: {section}.{key} は true / false")
    elif isinstance(old, (int, float)):
        if isinstance(value, bool) or not isinstance(value, (int, float)):
            raise ProfileError(f"{path}: {section}.{key} は数: {value!r}")
        value = type(old)(value)
    setattr(obj, key, value)


def apply_profile(cfg: Any, name_or_path: str) -> dict[str, dict[str, Any]]:
    """cfg（ButtonPressEnvCfg）の rewards / rand / env の項目を、設定の値で上書きする。上書きした値を返す。"""
    path = profile_path(name_or_path)
    prof = load_profile(name_or_path)
    for key, value in prof["rewards"].items():
        _set(cfg.rewards, "rewards", key, value, None, path)
    for key, value in prof["rand"].items():
        _set(cfg.rand, "rand", key, value, None, path)
    for key, value in prof["env"].items():
        _set(cfg, "env", key, value, ENV_KEYS, path)
    return prof


def selected_profile() -> str | None:
    """環境変数 BUTTON_PRESS_PROFILE で選んだ設定（無ければ None = 既定の値のまま）。"""
    v = os.environ.get(ENV_VAR, "").strip()
    return v or None
