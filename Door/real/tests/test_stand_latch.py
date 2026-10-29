"""G2（10-29）の共通の関数 stand_latch.py の試験（機体なし・Mac）。

  python -m pytest G1_Hackason/Door/real/tests/test_stand_latch.py -q

- 入る: 胴と把手の水平距離 0.3 m 未満かつ扉 10° 未満。抜ける: 扉 10° 以上、または 0.5 m より遠い（0.3〜0.5 m は前のまま）
- once_opened: その回で扉が一度 10° に届いたら、もう入らない（押し開けの途中で扉が戻っても止めない）
- numpy と torch で同じ答え。入力は書き換えない。学習（VIRAL6）に置く写しはパッチ 0082 の新しいファイルと 1 字も違わない
"""
from __future__ import annotations

import ast
import sys
from pathlib import Path

import numpy as np
import pytest

HERE = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(HERE))
import stand_latch as sl
from tests.test_target_guard import PATCH_DIR, _new_file_from_patch


def _run(dist: list[float], deg: list[float], **kw) -> list[bool]:
    """1 台ぶんの時系列に掛け金を当てて、各周期の latched を返す"""
    latched, opened = np.zeros(1, bool), np.zeros(1, bool)
    out = []
    for d, g in zip(dist, deg):
        latched, opened = sl.update_stand_latch(latched, opened, np.array([d]), np.array([g]), **kw)
        out.append(bool(latched[0]))
    return out


def test_enters_near_the_handle_and_holds_until_the_door_opens():
    """近づいて 0.3 m を切ると止め、0.3〜0.5 m に戻っても止めたまま。扉が 10° に届いたら放す"""
    dist = [0.8, 0.4, 0.29, 0.35, 0.45, 0.2, 0.2]
    deg = [0.0, 0.0, 0.0, 0.0, 0.0, 9.9, 10.0]
    assert _run(dist, deg) == [False, False, True, True, True, True, False]


def test_leaves_when_walking_away_from_the_handle():
    """扉が閉じたままでも、0.5 m より離れたら放す（入る線と抜ける線の間は前のまま）"""
    assert _run([0.2, 0.49, 0.51, 0.45], [0.0] * 4) == [True, True, False, False]


def test_does_not_enter_while_the_door_is_open():
    """扉がもう 10° 以上開いているなら、把手のそばでも止めない"""
    assert _run([0.2, 0.1], [12.0, 30.0]) == [False, False]


def test_once_opened_never_enters_again():
    """一度 10° に届いた回は、扉が戻って 0.3 m を切っても止めない（once_opened=False なら止める）"""
    dist = [0.2, 0.2, 0.2, 0.2]
    deg = [0.0, 15.0, 5.0, 5.0]
    assert _run(dist, deg) == [True, False, False, False]
    assert _run(dist, deg, once_opened=False) == [True, False, True, True]


def test_thresholds_can_be_changed():
    assert _run([0.35, 0.55], [0.0, 0.0], enter_m=0.4, exit_m=0.6) == [True, True]
    assert _run([0.2], [6.0], door_deg_max=5.0) == [False]


def test_many_envs_at_once_and_inputs_are_not_written():
    rng = np.random.default_rng(0)
    latched, opened = rng.random(100) < 0.5, rng.random(100) < 0.3
    dist, deg = rng.uniform(0, 1, 100), rng.uniform(0, 20, 100)
    keep = [x.copy() for x in (latched, opened, dist, deg)]
    new_l, new_o = sl.update_stand_latch(latched, opened, dist, deg)
    for a, b in zip((latched, opened, dist, deg), keep):
        assert np.array_equal(a, b)
    # 1 台ずつ当てた答えと同じ
    for i in range(100):
        l1, o1 = sl.update_stand_latch(latched[i:i + 1], opened[i:i + 1], dist[i:i + 1], deg[i:i + 1])
        assert (l1[0], o1[0]) == (new_l[i], new_o[i])


def test_numpy_and_torch_give_the_same_answer():
    torch = pytest.importorskip("torch")
    rng = np.random.default_rng(1)
    latched, opened = rng.random(200) < 0.5, rng.random(200) < 0.3
    dist, deg = rng.uniform(0, 1, 200), rng.uniform(0, 20, 200)
    want = sl.update_stand_latch(latched, opened, dist, deg)
    got = sl.update_stand_latch(torch.tensor(latched), torch.tensor(opened), torch.tensor(dist, dtype=torch.float32),
                                torch.tensor(deg, dtype=torch.float32))
    for w, g in zip(want, got):
        assert g.dtype == torch.bool
        assert np.array_equal(g.numpy(), w)


def test_python38_syntax():
    ast.parse((HERE / "stand_latch.py").read_text(encoding="utf-8"), feature_version=(3, 8))


def test_training_copy_is_identical():
    patches = sorted(PATCH_DIR.glob("0082-*.patch"))
    if not patches:
        pytest.skip("パッチ 0082 がまだ無い")
    got = _new_file_from_patch(patches[0], "gr00t/rl/envs/base_task/stand_latch.py")
    assert got == (HERE / "stand_latch.py").read_text(encoding="utf-8")
