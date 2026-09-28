"""設定ファイル（YAML）の読み込みと、パスの解決。"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import yaml

# Button_Press/J1-gen/
FEATURE_DIR: Path = Path(__file__).resolve().parents[1]
# リポジトリ直下
REPO_ROOT: Path = FEATURE_DIR.parents[1]
CONFIG_DIR: Path = FEATURE_DIR / "configs"


def load_config(path: str | Path) -> dict[str, Any]:
    """YAML を辞書として読む。相対パスは configs/ 基準で探す。"""
    p = Path(path)
    if not p.is_absolute() and not p.exists():
        p = CONFIG_DIR / p
    with open(p, encoding="utf-8") as f:
        data = yaml.safe_load(f)
    if not isinstance(data, dict):
        raise ValueError(f"設定ファイルの中身が辞書ではない: {p}")
    return data


def resolve_repo_path(path: str | Path) -> Path:
    """設定ファイル内のパスを解決する。相対パスはリポジトリ直下基準。"""
    p = Path(path)
    return p if p.is_absolute() else REPO_ROOT / p
