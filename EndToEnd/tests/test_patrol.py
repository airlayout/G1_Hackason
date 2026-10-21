"""室内の通行領域、角の回避、巡回の継続性をGPUなしで検証する。"""

from dataclasses import replace
from pathlib import Path
from types import SimpleNamespace

import numpy as np
import pytest

from g1_e2e.data.scene import OfficeConfig
from g1_e2e.navigation.grid import NavigationGrid
from g1_e2e.navigation.patrol import PatrolTeacher
from scripts.fetch_office import mdl_dependencies

ROOT = Path(__file__).resolve().parents[1]


def scene(**changes):
    return replace(OfficeConfig.load(ROOT / "configs/scenes/office.yaml"), **changes)


def grid():
    free = np.ones((60, 60), bool)
    free[:2] = free[-2:] = False
    free[:, :2] = free[:, -2:] = False
    free[15:40, 28:32] = False
    return NavigationGrid(np.zeros(2), 0.1, free, np.where(free, 1.0, 0.0).astype(np.float32))


def test_astar_routes_around_a_wall_without_cutting_diagonal_corners():
    current = grid()
    route = current.path(np.array([1, 2.5]), np.array([5, 2.5]))
    assert len(route) > 40
    assert all(current.segment_walkable(a, b) for a, b in zip(route[:-1], route[1:]))
    assert not current.segment_walkable([1, 2.5], [5, 2.5])
    assert not current.segment_walkable([1, 1], [-1, 1])


def test_teacher_continues_to_new_regions_and_its_commands_remain_within_locomotion_limits():
    current = grid()
    teacher = PatrolTeacher(current, scene(cruise_speed_m_s=0.3, lookahead_m=0.5))
    xy, yaw = np.array([1.0, 1.0]), 0.0
    for step in range(5000):
        observation = SimpleNamespace(
            privileged={"base_position_world_m": np.array([*xy, 0.8])},
            orientation_xyzw=np.array([0, 0, np.sin(yaw / 2), np.cos(yaw / 2)]),
        )
        command = teacher.command(observation)
        assert np.isfinite(command).all()
        assert abs(command[0]) <= 0.300001 and abs(command[1]) <= 0.300001 and abs(command[2]) <= 0.200001
        xy += 0.02 * np.array(
            [
                np.cos(yaw) * command[0] - np.sin(yaw) * command[1],
                np.sin(yaw) * command[0] + np.cos(yaw) * command[1],
            ]
        )
        yaw += 0.02 * command[2]
        y, x = current.cell(xy)
        assert current.walkable[y, x]
    assert teacher.summary["goals_reached"] >= 2
    assert teacher.summary["distance_m"] > 5
    assert teacher.summary["visited_regions"] > 3
    teacher.reset()
    assert teacher.summary["replans"] == 0 and teacher.goal is None


def test_spawn_is_repeatable_and_inside_a_free_cell():
    teacher = PatrolTeacher(grid(), scene())
    assert teacher.spawn(42) == teacher.spawn(42)
    first, second = teacher.spawn(42), teacher.spawn(43)
    assert first != second
    y, x = teacher.grid.cell(first[:2])
    assert teacher.grid.walkable[y, x]


def test_office_grid_is_built_from_floor_and_obstacles_in_metres():
    pytest.importorskip("PIL")
    from sim.envs.office import navigation_grid

    vertices = np.array(
        [[0, 0, 0], [6, 0, 0], [6, 6, 0], [0, 6, 0], [2.8, 1, 0], [3.2, 1, 0], [3.2, 5, 1.5], [2.8, 5, 1.5]],
        np.float32,
    )
    faces = np.array([[0, 1, 2], [0, 2, 3], [4, 5, 6], [4, 6, 7]])
    config = scene(navigation_bounds_xy_m=((0, 6), (0, 6)), robot_radius_m=0.4)
    current = navigation_grid(vertices, faces, config)
    y, x = current.cell([3, 3])
    assert not current.walkable[y, x]
    assert current.clearance([3, 3]) == 0
    assert current.clearance([1, 3]) >= 0.4


def test_office_asset_hash_paths_and_relative_mdl_imports_are_checked():
    with pytest.raises(ValueError):
        scene(asset_path="../outside.usd")
    with pytest.raises(ValueError):
        scene(asset_sha256="unfixed")
    assert mdl_dependencies(
        'using .::OmniUe4Base import *; import ::math::*; import ..::Other::*; texture_2d("textures/a.png")'
    ) == {"./OmniUe4Base.mdl", "../Other.mdl", "textures/a.png"}
