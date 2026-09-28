"""テストの実行環境の判定。足りないものがあるテストは、理由を付けて「スキップ」する。

GitHub の CI（.github/workflows/ci.yml）は Python を用意するだけで、画面（OpenGL）も unitree_sdk2py も無い。
その環境でも失敗させず、何を確かめていないかが分かるようにする（run_tests.sh の最初にも一覧を表示する）。

    from _env import needs
    @needs("mujoco", "pin", "render")
    class TestSomething(unittest.TestCase): ...
"""

from __future__ import annotations

import functools
import importlib.util
import subprocess
import sys
import unittest
from collections.abc import Callable
from typing import Any

from common.config import REPO_ROOT

MODELS = REPO_ROOT / "_local" / "button_press" / "models" / "g1_description" / "g1_29dof_rev_1_0.xml"

REASONS = {
    "mujoco": "mujoco が無い（または読み込めない）",
    "pin": "pin（Pinocchio）が無い（または読み込めない）",
    "models": "公式モデルが無い（sim/fetch_models.sh を実行していない）",
    "render": "描画環境（OpenGL）が無い（MuJoCo のカメラの画像を作れない）",
    "sdk": "unitree_sdk2py が無い",
    "yolo": "ultralytics（YOLO）が無い",
}


def _has(module: str) -> bool:
    return importlib.util.find_spec(module) is not None


@functools.cache
def _imports(module: str) -> bool:
    """実際に読み込めるか（入っていても読み込めないことがある。例: MUJOCO_GL=osmesa で OSMesa が無いと
    import mujoco が失敗する）。失敗の仕方によってはプロセスごと落ちるので、別のプロセスで試す。"""
    if not _has(module):
        return False
    try:
        return subprocess.run([sys.executable, "-c", f"import {module}"], capture_output=True,
                              timeout=120).returncode == 0
    except Exception:  # noqa: BLE001
        return False


@functools.cache
def _render_ok() -> bool:
    """MuJoCo の描画ができるか。失敗するとプロセスごと落ちることがあるので、別のプロセスで試す。"""
    if not _imports("mujoco"):
        return False
    code = ("import mujoco; m = mujoco.MjModel.from_xml_string('<mujoco><worldbody><geom size=\"1\"/>"
            "</worldbody></mujoco>'); r = mujoco.Renderer(m, 8, 8); r.update_scene(mujoco.MjData(m)); r.render()")
    try:
        return subprocess.run([sys.executable, "-c", code], capture_output=True, timeout=60).returncode == 0
    except Exception:  # noqa: BLE001
        return False


@functools.cache
def available(what: str) -> bool:
    if what == "mujoco":
        return _imports("mujoco")
    if what == "pin":
        return _imports("pinocchio")
    if what == "models":
        return MODELS.exists()
    if what == "render":
        return _render_ok()
    if what == "sdk":
        return _imports("unitree_sdk2py")
    if what == "yolo":
        return _imports("ultralytics")
    raise ValueError(what)


def missing(*what: str) -> list[str]:
    # mujoco を使うテストは公式モデルも要る
    need = list(what) + (["models"] if ("mujoco" in what or "render" in what) and "models" not in what else [])
    return [REASONS[w] for w in need if not available(w)]


def needs(*what: str) -> Callable[[Any], Any]:
    """足りないものがあれば、テスト（クラスでも関数でも）を理由付きでスキップする。"""
    reasons = missing(*what)
    if reasons:
        return unittest.skip("スキップ: " + "、".join(reasons))
    return lambda x: x


def summary() -> str:
    return "、".join(f"{w} {'あり' if available(w) else 'なし'}" for w in REASONS)


if __name__ == "__main__":
    print(f"[tests] 実行環境: {summary()}")
    lacking = [REASONS[w] for w in REASONS if not available(w)]
    if lacking:
        print("[tests] ⚠️ 次のものが無いので、それを使うテストはスキップする（その部分は確かめていない）: " + "、".join(lacking))
