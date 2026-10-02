"""Button_Press/J1-gen/common の部品（機体モデル、IK など）を読み込む。

J1-gen と Yada は、どちらもパッケージ名が `common` なので、そのまま import すると名前がぶつかる。
そこで J1-gen/common を `j1gen_common` という別名のパッケージとして読み込む
（J1-gen の perception_bridge.py と同じ方法）。

    from common.j1gen_bridge import j1gen, j1gen_config
    build_spec = j1gen("robot_model").build_spec
    robot_cfg = j1gen_config("robot.yaml")
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
from types import ModuleType
from typing import Any

from .config import FEATURE_DIR

PACKAGE = "j1gen_common"
J1GEN_DIR = FEATURE_DIR.parent / "J1-gen"
J1GEN_COMMON = J1GEN_DIR / "common"


def _load_package() -> ModuleType:
    if PACKAGE in sys.modules:
        return sys.modules[PACKAGE]
    spec = importlib.util.spec_from_file_location(
        PACKAGE, J1GEN_COMMON / "__init__.py", submodule_search_locations=[str(J1GEN_COMMON)]
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"J1-gen/common が読み込めない: {J1GEN_COMMON}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[PACKAGE] = module
    spec.loader.exec_module(module)
    return module


def j1gen(submodule: str) -> ModuleType:
    """J1-gen/common/<submodule> を返す（例: "robot_model"、"kinematics"）。"""
    _load_package()
    return importlib.import_module(f"{PACKAGE}.{submodule}")


def j1gen_config(name: str) -> dict[str, Any]:
    """J1-gen/configs/<name> を読む（機体の設定は J1-gen と同じものを使う）。"""
    return j1gen("config").load_config(J1GEN_DIR / "configs" / name)
