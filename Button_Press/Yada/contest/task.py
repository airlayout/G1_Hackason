"""試行ごとの条件（ボタン盤の位置、押すボタン、指示の文、床の色）を、乱数の種から決める。

同じ種なら、どのシミュレーターでも、どのマシンでも同じ条件になる（numpy の default_rng を使う）。
"""

from __future__ import annotations

import copy
from dataclasses import dataclass
from typing import Any

import numpy as np

from common.config import load_config
from common.realism import Realism, sample_realism

from .interface import BUTTONS


@dataclass(frozen=True)
class Trial:
    seed: int
    target: str
    instruction: str
    scene_cfg: dict[str, Any]  # configs/elevator_hall.yaml の中身を、この試行の値に書き換えたもの
    eval_set: str = "basic"
    realism: Realism | None = None  # 評価セット realistic の乱し（basic では None）


def make_trial(seed: int, contest_cfg: dict[str, Any], base_scene_cfg: dict[str, Any] | None = None,
               eval_set: str = "basic") -> Trial:
    """種 seed の試行の条件を作る。eval_set は configs/contest.yaml の sets の名前。"""
    rng = np.random.default_rng(int(seed))
    scene = copy.deepcopy(base_scene_cfg if base_scene_cfg is not None else load_config("elevator_hall.yaml"))
    r = contest_cfg["randomize"]

    def uniform(key: str) -> float:
        lo, hi = r[key]
        return float(rng.uniform(lo, hi))

    # 引く順番を変えると、同じ種でも条件が変わるので注意（足すときは最後に足す）
    scene["wall"]["front_x"] = uniform("wall_front_x")
    scene["panel"]["center_y"] = uniform("panel_center_y")
    scene["panel"]["center_height"] = uniform("panel_center_height")
    if "floor" in scene:
        k = uniform("floor_brightness")
        scene["floor"]["rgb"] = [min(1.0, c * k) for c in scene["floor"]["rgb"]]
        scene["floor"]["seed"] = int(rng.integers(0, 2**31 - 1))
    target = BUTTONS[int(rng.integers(0, len(BUTTONS)))]
    texts = contest_cfg["instructions"][target]
    instruction = texts[int(rng.integers(0, len(texts)))]
    if eval_set not in contest_cfg.get("sets", {"basic": {}}):
        raise ValueError(f"評価セットが無い: {eval_set}（あるのは {list(contest_cfg.get('sets', {}))}）")
    realism = None
    if contest_cfg.get("sets", {}).get(eval_set, {}).get("realism"):
        realism = sample_realism(seed, contest_cfg["realism"])
    return Trial(seed=int(seed), target=target, instruction=instruction, scene_cfg=scene, eval_set=eval_set,
                 realism=realism)
