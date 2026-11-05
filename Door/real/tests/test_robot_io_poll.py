"""11-05 夜: door_run_real.LatestPoll（DDS の読み手から呼ばれた時に最新の 1 件だけ取る）の試験。DDS なしで回る"""
from __future__ import annotations

import sys
from pathlib import Path
from types import SimpleNamespace

import pytest

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
import door_run_real as drr  # noqa: E402

INVALID = object()


def _poller(batches, **kw):
    """batches を 1 回の take ごとに 1 つずつ返す LatestPoll。時計は take の回数"""
    it = iter(batches)
    clock = iter(range(100))
    return drr.LatestPoll(lambda: next(it), lambda m: m is not INVALID, clock=lambda: float(next(clock)), **kw)


def test_none_before_first_sample() -> None:
    assert _poller([[], []]).poll() == (None, None)


def test_keeps_last_value_and_time_when_nothing_new() -> None:
    p = _poller([["a"], [], []])
    assert p.poll() == ("a", 0.0)
    assert p.poll() == ("a", 0.0)  # 新しい件が無ければ時刻も進めない（途切れの見張りが効く）
    assert p.poll() == ("a", 0.0)


def test_takes_newest_and_skips_invalid() -> None:
    p = _poller([["a"], ["b", INVALID], [INVALID]])
    assert p.poll() == ("a", 0.0)
    assert p.poll() == ("b", 1.0)
    assert p.poll() == ("b", 1.0)


def test_convert_is_applied() -> None:
    p = _poller([[3]], convert=lambda m: m * 10)
    assert p.poll() == (30, 0.0)


def test_odom_xy_yaw() -> None:
    msg = SimpleNamespace(pose=SimpleNamespace(pose=SimpleNamespace(
        position=SimpleNamespace(x=1.5, y=-0.25, z=0.0),
        orientation=SimpleNamespace(w=0.7071067811865476, x=0.0, y=0.0, z=0.7071067811865476))))
    x, y, yaw = drr.odom_xy_yaw(msg)
    assert (x, y) == (1.5, -0.25)
    assert yaw == pytest.approx(1.5707963, abs=1e-6)
