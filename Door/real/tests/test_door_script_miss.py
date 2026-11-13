"""11-12 夕 空振りの押し直し（door_script.ScriptDoor の miss_shifts）: 偽の手のひらで状態の移りを確かめる。機体は要らない。

偽の世界は test_door_script.World に 2 つ足す: 本当のレバーは台本の思う当てる点から横へ lever_dy ずれた所にあり、
扉が開くまでは手の先が板（手の先の初めの位置の 2 cm 先。門番の見積もりは 1.7 cm）より前へ出られない。
⚠️ 状態の移りの確かめだけ。実機で空振りの時に手の先がどこで止まるかは入っていない
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
from test_door_script import World, make  # noqa: E402

BOARD_AHEAD_M = 0.02


class MissWorld(World):
    def __init__(self, p, lever_dy: float, **kw) -> None:
        super().__init__(p, **kw)
        self.board_x = float(self.ref[0]) + BOARD_AHEAD_M
        self.ref = self.ref + np.array([0.0, lever_dy, 0.0])  # 本当のレバー
        self.pose_at = []  # （小段, その小段に入った時の pelvis の x）

    def follow(self, goal: np.ndarray) -> None:
        self.stuck_x = math.inf if self.opened else self.board_x + 1e-4
        super().follow(goal)
        if self.subs and (not self.pose_at or self.pose_at[-1][0] != self.subs[-1]):
            self.pose_at.append((self.subs[-1], self.pose.x))


def test_hit_first_time_uses_no_retry():
    # Arrange
    p, s = make(goal_deg=15.0)
    w = MissWorld(p, lever_dy=p.above[1])

    # Act
    out = w.run(s)

    # Assert
    assert out.success, out.stop
    assert s.miss_i == 0 and "retreat" not in w.subs


def test_miss_toward_hinge_recovers_by_first_shift():
    # Arrange: 本当のレバーは手の下りる所より 9 cm 付け根の側（11-12 実機 2 回目の形）
    p, s = make(goal_deg=15.0)
    w = MissWorld(p, lever_dy=p.above[1] + 0.09)

    # Act
    out = w.run(s)

    # Assert
    assert out.success, out.stop
    assert s.miss_i == 1 and math.isclose(s.shift, p.miss_shifts[0])
    assert w.subs[:10] == ["2a", "2b", "2c", "2d", "3a", "retreat", "2a", "2b", "2c", "2d"]


def test_miss_toward_root_recovers_by_second_shift():
    # Arrange
    p, s = make(goal_deg=15.0)
    w = MissWorld(p, lever_dy=p.above[1] - 0.09)

    # Act
    out = w.run(s, seconds=120.0)

    # Assert
    assert out.success, out.stop
    assert s.miss_i == 2 and math.isclose(s.shift, p.miss_shifts[1])


def test_no_lever_uses_all_shifts_then_stops_as_before():
    # Arrange
    p, s = make(goal_deg=15.0)
    w = MissWorld(p, lever_dy=0.0, lever=False)

    # Act
    out = w.run(s, seconds=180.0)

    # Assert
    assert not out.success
    assert s.miss_i == len(p.miss_shifts)
    assert "掛け金が外れない" in out.stop


def test_walk_in_3a_miss_walks_back_before_retry():
    # Arrange
    p, s = make(goal_deg=15.0, walk_in_3a=True)
    w = MissWorld(p, lever_dy=p.above[1] + 0.09)
    x0 = w.pose.x

    # Act
    out = w.run(s, seconds=120.0)

    # Assert: 押し直しで掛け金は外れる（3c に入る）。その先は見ない: 歩いて押す 3a は、当たった回でも体が手に 5〜10 cm 寄り、
    # 3c の「手が体へ押し込まれた」の線（0.15 m）の際になる（押し直しと別の、walk_in_3a の余裕の話。機体で未確認）
    assert s.miss_i == 1 and w.subs[-2:] != ["retreat", "failed"] and "3c" in w.subs, out.stop
    k = [n for n, _ in w.pose_at].index("retreat")
    assert w.pose_at[k + 1][0] == "2a"
    assert w.pose_at[k][1] > x0 + 0.03                # 歩いて押した分だけ前に居た
    assert w.pose_at[k + 1][1] <= x0 + 0.03           # 押し直しの前に下がって戻った


def test_miss_shifts_out_of_range_is_rejected():
    # Arrange / Act / Assert
    try:
        make(miss_shifts=(0.2,))
    except ValueError as e:
        assert "miss_shifts" in str(e)
    else:
        raise AssertionError("±0.10 m の外のずらしを受け付けた")
