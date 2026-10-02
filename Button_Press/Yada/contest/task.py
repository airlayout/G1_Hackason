"""試行ごとの条件（柱やボタンの寸法と位置、押すボタン、指示の文、色）を、乱数の種から決める。

同じ種なら、どのシミュレーターでも、どのマシンでも同じ条件になる（numpy の default_rng を使う）。
"""

from __future__ import annotations

import copy
from dataclasses import dataclass, field
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
    params: dict[str, float] = field(default_factory=dict)  # randomize で引いた値（記録とレポート用）


def _set_path(cfg: dict[str, Any], path: str, value: float) -> None:
    keys = path.split(".")
    d = cfg
    for k in keys[:-1]:
        d = d[k]
    if keys[-1] not in d:
        raise KeyError(f"シーンの設定に {path} が無い（configs/contest.yaml の randomize を確かめる）")
    d[keys[-1]] = value


def _apply_special(scene: dict[str, Any], key: str, value: float, rng: np.random.Generator) -> None:
    """名前に「.」が無い条件（いくつかの値をまとめて決めるもの）。"""
    items = {b["name"]: b for b in scene.get("buttons", {}).get("items", [])}
    if key == "floor_brightness":
        if "floor" in scene:
            scene["floor"]["rgb"] = [min(1.0, c * value) for c in scene["floor"]["rgb"]]
            scene["floor"]["seed"] = int(rng.integers(0, 2**31 - 1))
    elif key == "column_brightness":
        if "column" in scene:
            c = scene["column"]["rgba"]
            scene["column"]["rgba"] = [min(1.0, v * value) for v in c[:3]] + [c[3]]
    elif key == "button_up_height":
        items["up"]["height"] = value
    elif key == "button_spacing":
        # ▼ は ▲ の下に間隔だけ。車いす用も同じ間隔（▲ と ▼ が逆にならないように、間隔で決める）
        items["down"]["height"] = float(items["up"]["height"]) - value
        if "wc_up" in items and "wc_down" in items:
            items["wc_down"]["height"] = float(items["wc_up"]["height"]) - value
    elif key == "button_wc_up_height":
        if "wc_up" in items:
            gap = float(items["wc_up"]["height"]) - float(items["wc_down"]["height"])
            items["wc_up"]["height"] = value
            items["wc_down"]["height"] = value - gap
            for dc in scene.get("decals", []):
                if dc["name"] == "wheelchair":
                    dc["height"] = value + 0.08
    # 前の形（elevator_hall.yaml）の名前
    elif key == "wall_front_x":
        scene["wall"]["front_x"] = value
    elif key == "panel_center_y":
        scene["panel"]["center_y"] = value
    elif key == "panel_center_height":
        scene["panel"]["center_height"] = value
    else:
        raise KeyError(f"条件の名前が分からない: {key}（configs/contest.yaml の randomize）")


def make_trial(seed: int, contest_cfg: dict[str, Any], base_scene_cfg: dict[str, Any] | None = None,
               eval_set: str = "basic") -> Trial:
    """種 seed の試行の条件を作る。eval_set は configs/contest.yaml の sets の名前。

    条件は configs/contest.yaml の randomize の順に引く（順番を変えると、同じ種でも条件が変わる。足すときは最後に足す）。
    """
    rng = np.random.default_rng(int(seed))
    if base_scene_cfg is None:
        base_scene_cfg = load_config(contest_cfg.get("scene_file", "elevator_hall.yaml"))
    scene = copy.deepcopy(base_scene_cfg)
    params: dict[str, float] = {}
    for key, (lo, hi) in contest_cfg["randomize"].items():
        v = float(rng.uniform(float(lo), float(hi)))
        params[key] = v
        if "." in key:
            _set_path(scene, key, v)
        else:
            _apply_special(scene, key, v, rng)
    target = BUTTONS[int(rng.integers(0, len(BUTTONS)))]
    texts = contest_cfg["instructions"][target]
    instruction = texts[int(rng.integers(0, len(texts)))]
    if eval_set not in contest_cfg.get("sets", {"basic": {}}):
        raise ValueError(f"評価セットが無い: {eval_set}（あるのは {list(contest_cfg.get('sets', {}))}）")
    realism = None
    if contest_cfg.get("sets", {}).get(eval_set, {}).get("realism"):
        realism = sample_realism(seed, contest_cfg["realism"])
    return Trial(seed=int(seed), target=target, instruction=instruction, scene_cfg=scene, eval_set=eval_set,
                 realism=realism, params=params)
