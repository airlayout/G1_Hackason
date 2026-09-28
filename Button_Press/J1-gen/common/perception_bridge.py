"""Perception/common の部品（FrameSource、ZmqFrameSource、YoloDetector など）を読み込む。

Perception と Button_Press は、どちらもパッケージ名が `common` なので、そのまま import すると
名前がぶつかる。そこで Perception/common を `perception_common` という別名のパッケージとして読み込む。

    from common.perception_bridge import perception
    FrameSource = perception("camera").FrameSource
"""

from __future__ import annotations

import importlib
import importlib.util
import sys
from types import ModuleType

from .config import REPO_ROOT

PACKAGE = "perception_common"
PERCEPTION_COMMON = REPO_ROOT / "Perception" / "common"


def _load_package() -> ModuleType:
    if PACKAGE in sys.modules:
        return sys.modules[PACKAGE]
    spec = importlib.util.spec_from_file_location(
        PACKAGE, PERCEPTION_COMMON / "__init__.py", submodule_search_locations=[str(PERCEPTION_COMMON)]
    )
    if spec is None or spec.loader is None:
        raise ImportError(f"Perception/common が読み込めない: {PERCEPTION_COMMON}")
    module = importlib.util.module_from_spec(spec)
    sys.modules[PACKAGE] = module
    spec.loader.exec_module(module)
    return module


def perception(submodule: str) -> ModuleType:
    """Perception/common/<submodule> を返す（例: "camera"、"detector"）。"""
    _load_package()
    return importlib.import_module(f"{PACKAGE}.{submodule}")
