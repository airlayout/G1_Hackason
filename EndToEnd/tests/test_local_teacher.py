"""初期教師は取得済みLiDARに従い、非公開の巡回目標や未来scanへ依存しない。"""

from dataclasses import replace

import numpy as np
import pytest

from g1_e2e.navigation.local import LocalPatrolTeacher
from tests.test_data_collection import observation
from tests.test_patrol import grid, scene


def test_local_teacher_stops_for_missing_data_and_avoids_a_front_wall():
    teacher = LocalPatrolTeacher(grid(), scene())
    current = replace(observation(5), privileged={"base_position_world_m": np.array([1, 1, 0.8])})
    np.testing.assert_array_equal(teacher.command(current), 0)
    y, z = np.meshgrid(np.linspace(-2, 2, 100), np.linspace(0.2, 1.6, 10))
    points = np.column_stack((np.full(y.size, 1.8), 1 + y.reshape(-1), z.reshape(-1)))
    teacher.update_scan(points, np.ones(len(points), bool), 0.1)
    command = teacher.command(current)
    assert abs(command[1]) > abs(command[0])
    assert abs(command[1]) <= 0.3 and abs(command[2]) <= 0.2
    teacher.update_scan(points, np.ones(len(points), bool), 0.2)
    with pytest.raises(ValueError, match="未来"):
        teacher.command(current)
