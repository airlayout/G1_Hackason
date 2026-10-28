"""乗り場（壁・扉・柱・ボタン）を USD のファイルに書き出す。Isaac Sim の起動後に使う（pxr が要る）。

評価環境（Yada）の sim/isaac/isaac_hall.py の build_hall をそのまま使うので、寸法・ボタンのばね・沈む量・
衝突の外し方は評価と同じ。学習環境はこのファイルを環境ごとに読み込む（tasks/press_env.py）。

ファイルの中は /World/Hall（pelvis の高さに置く）の下に乗り場がある。既定の prim は /World なので、
環境の <env>/Scene に読み込むと、乗り場は <env>/Scene/Hall、ボタンの関節機構は <env>/Scene/Hall/Buttons になる。
床の絨毯（見た目だけの 20 m 四方の板）は、環境ごとに重なるので作らない。
"""

from __future__ import annotations

import dataclasses
import hashlib
import json
from pathlib import Path
from typing import Any

import numpy as np

from .paths import LOCAL_DIR, use_yada

USD_DIR = LOCAL_DIR / "usd"
# 乗り場の作り方（Yada の isaac_hall.py）が変わったら上げる。ファイルの名前に入るので、古いファイルを使い続けない
# （2026-10-20: 柱の摩擦の材質を足したとき、設定が同じなので古いファイルが使われた）
BUILDER_VERSION = 2
# 環境の中での、乗り場とボタンの関節機構の場所（<env> からの相対パス）
SCENE_NAME = "Scene"
HALL_REL = f"{SCENE_NAME}/Hall"
BUTTONS_REL = f"{HALL_REL}/Buttons"


def hall_usd_path(scene_cfg: dict[str, Any]) -> Path:
    """乗り場の設定ごとのファイルの場所（設定が変われば別のファイル）。"""
    key = hashlib.sha1(json.dumps(scene_cfg, sort_keys=True).encode()).hexdigest()[:10]
    return USD_DIR / f"hall_v{BUILDER_VERSION}_{key}.usda"


def write_hall_usd(scene_cfg: dict[str, Any], overwrite: bool = False) -> Path:
    """乗り場を USD に書き出し、そのパスを返す（同じ設定のファイルがあれば作り直さない）。"""
    use_yada(isaac=True)
    from pxr import Usd, UsdGeom

    from common.scene_spec import build_hall_scene
    from isaac_hall import build_hall

    path = hall_usd_path(scene_cfg)
    if path.exists() and not overwrite:
        return path
    path.parent.mkdir(parents=True, exist_ok=True)
    scene = dataclasses.replace(build_hall_scene(scene_cfg), floor=None)
    stage = Usd.Stage.CreateNew(str(path))
    UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
    UsdGeom.SetStageMetersPerUnit(stage, 1.0)
    world = UsdGeom.Xform.Define(stage, "/World")
    stage.SetDefaultPrim(world.GetPrim())
    build_hall(stage, scene, np.array([0.0, 0.0, scene.pelvis_height]))
    stage.GetRootLayer().Save()
    print(f"[hall_usd] 乗り場を書き出した: {path}")
    return path
