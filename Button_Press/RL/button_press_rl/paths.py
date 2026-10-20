"""パスと、Button_Press/Yada（評価環境）の部品の読み込み。

学習環境は、乗り場の寸法・ボタン・G1 のモデル・PD の強さ・安全のための処理を、評価環境（Yada）と同じものにする。
そのため Yada のコードと設定を、複製せずにここから読み込む。

Yada のパッケージ名は `common` / `contest`（J1-gen も `common`）なので、Yada のフォルダを import の検索先の先頭に足す。
J1-gen の部品は Yada の common/j1gen_bridge.py が `j1gen_common` という別名で読み込む。
"""

from __future__ import annotations

import sys
from pathlib import Path
from typing import Any

# Button_Press/RL/
FEATURE_DIR: Path = Path(__file__).resolve().parents[1]
REPO_ROOT: Path = FEATURE_DIR.parents[1]
YADA_DIR: Path = REPO_ROOT / "Button_Press" / "Yada"
YADA_ISAAC_DIR: Path = YADA_DIR / "sim" / "isaac"
CONFIG_DIR: Path = FEATURE_DIR / "configs"
# 学習の出力（git の対象外）
LOCAL_DIR: Path = REPO_ROOT / "_local" / "button_press_rl"


def use_yada(isaac: bool = False) -> None:
    """Yada の common / contest を import できるようにする。isaac=True なら sim/isaac（isaac_hall など）も。"""
    for p in ([YADA_ISAAC_DIR] if isaac else []) + [YADA_DIR]:
        s = str(p)
        if s not in sys.path:
            sys.path.insert(0, s)


def yada_config(name: str) -> dict[str, Any]:
    """Yada/configs/<name> を読む（contest.yaml、elevator_prod.yaml など）。"""
    use_yada()
    from common.config import load_config

    return load_config(name)


def rl_config(name: str) -> dict[str, Any]:
    """Button_Press/RL/configs/<name> を読む。"""
    import yaml

    with open(CONFIG_DIR / name, encoding="utf-8") as f:
        return yaml.safe_load(f)
