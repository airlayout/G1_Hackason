"""`nav.mission` を偽サービス相手に走らせる。実機・PC2・DDS・numpyは不要。

`kinematics=False` がmock（プロトコルと状態遷移だけ）、
`kinematics=True` がsim（運動と障害物も含む）。同じMissionコードを流す。
"""

from __future__ import annotations

import unittest

from nav.geometry import Pose2D
from nav.mission import Mission, MissionOptions, Outcome
from nav.occupancy import build_grid
from nav.protocol import API_INIT_POSE, API_NAVIGATE_POSE, API_PAUSE, API_RESUME
from sim.fake_service import FakeOptions, FakeTransport, Obstacle

MAP = "/home/unitree/test1.pcd"


def floor_grid(half: float = 10.0, resolution: float = 0.2):
    """障害物のない床だけの部屋。"""

    step = resolution * 0.9
    cols = int(2 * half / step) + 1
    points = [
        (-half + col * step, -half + row * step, 0.0)
        for row in range(cols)
        for col in range(cols)
    ]
    return build_grid(points, resolution=resolution, inflation=0.0, margin=1.0)


def options(**overrides) -> MissionOptions:
    base = {"map_address": MAP}
    base.update(overrides)
    return MissionOptions(**base)


class MockModeTest(unittest.TestCase):
    """kinematics無し。プロトコルと状態遷移だけを見る。"""

    def setUp(self):
        self.grid = floor_grid()
        self.transport = FakeTransport(
            Pose2D(0.0, 0.0),
            FakeOptions(kinematics=False, known_maps=frozenset({MAP})),
        )

    def _run(self, waypoints, **overrides):
        return Mission(self.transport, self.grid, waypoints, options(**overrides)).run()

    def test_reaches_every_waypoint(self):
        report = self._run([Pose2D(0.0, 0.0), Pose2D(3.0, 0.0), Pose2D(3.0, 3.0)])
        self.assertIs(report.outcome, Outcome.COMPLETED)
        self.assertEqual(report.waypoints_reached, 2)

    def test_calls_init_pose_before_navigating(self):
        self._run([Pose2D(0.0, 0.0), Pose2D(3.0, 0.0)])
        api_ids = [api_id for api_id, _ in self.transport.calls]
        self.assertEqual(api_ids[0], API_INIT_POSE)
        self.assertTrue(all(api_id == API_NAVIGATE_POSE for api_id in api_ids[1:]))

    def test_init_pose_carries_the_start_pose_and_map(self):
        self._run([Pose2D(1.5, -2.5), Pose2D(3.0, 0.0)])
        _, request = self.transport.calls[0]
        self.assertEqual(request["data"]["address"], MAP)
        self.assertAlmostEqual(request["data"]["x"], 1.5)
        self.assertAlmostEqual(request["data"]["y"], -2.5)

    def test_long_leg_is_sent_as_several_commands(self):
        # 18m を8m刻み -> 3回
        self._run([Pose2D(-9.0, 0.0), Pose2D(9.0, 0.0)])
        navigates = [r for api_id, r in self.transport.calls if api_id == API_NAVIGATE_POSE]
        self.assertEqual(len(navigates), 3)

    def test_laps_repeat_the_waypoints(self):
        report = self._run([Pose2D(0.0, 0.0), Pose2D(3.0, 0.0), Pose2D(0.0, 0.0)], laps=3)
        self.assertIs(report.outcome, Outcome.COMPLETED)
        self.assertEqual(report.waypoints_reached, 6)

    def test_pause_and_resume_hit_the_right_apis(self):
        mission = Mission(self.transport, self.grid, [Pose2D(0.0, 0.0), Pose2D(1.0, 0.0)], options())
        mission.pause()
        mission.resume()
        self.assertEqual([api_id for api_id, _ in self.transport.calls], [API_PAUSE, API_RESUME])


class InitPoseTest(unittest.TestCase):
    def test_gives_up_after_the_configured_retries(self):
        transport = FakeTransport(Pose2D(0.0, 0.0), FakeOptions(known_maps=frozenset()))
        report = Mission(
            transport, floor_grid(), [Pose2D(0.0, 0.0), Pose2D(1.0, 0.0)], options()
        ).run()
        self.assertIs(report.outcome, Outcome.FAILED_INIT)
        self.assertEqual(sum(1 for api_id, _ in transport.calls if api_id == API_INIT_POSE), 3)

    def test_failure_message_points_at_pc1(self):
        transport = FakeTransport(Pose2D(0.0, 0.0), FakeOptions(known_maps=frozenset()))
        report = Mission(
            transport, floor_grid(), [Pose2D(0.0, 0.0), Pose2D(1.0, 0.0)], options()
        ).run()
        self.assertIn("192.168.123.161", report.message)
        self.assertIn("1802", report.message)

    def test_recovers_when_a_later_attempt_succeeds(self):
        transport = FakeTransport(
            Pose2D(0.0, 0.0),
            FakeOptions(kinematics=False, known_maps=frozenset({MAP}), init_failures=2),
        )
        report = Mission(
            transport, floor_grid(), [Pose2D(0.0, 0.0), Pose2D(2.0, 0.0)], options()
        ).run()
        self.assertIs(report.outcome, Outcome.COMPLETED)
        self.assertTrue(transport.initialized)

    def test_no_navigation_is_sent_when_init_fails(self):
        transport = FakeTransport(Pose2D(0.0, 0.0), FakeOptions(known_maps=frozenset()))
        Mission(transport, floor_grid(), [Pose2D(0.0, 0.0), Pose2D(1.0, 0.0)], options()).run()
        self.assertFalse(any(api_id == API_NAVIGATE_POSE for api_id, _ in transport.calls))


class KinematicsTest(unittest.TestCase):
    """kinematics有り。実際に動かして幾何を見る。"""

    def _transport(self, start=Pose2D(-5.0, 0.0), **fake):
        fake.setdefault("known_maps", frozenset({MAP}))
        return FakeTransport(start, FakeOptions(**fake))

    def test_ends_up_at_the_last_waypoint(self):
        transport = self._transport()
        goal = Pose2D(5.0, 0.0)
        report = Mission(transport, floor_grid(), [Pose2D(-5.0, 0.0), goal], options()).run()
        self.assertIs(report.outcome, Outcome.COMPLETED)
        self.assertLess(transport.pose.distance_to(goal), 0.2)

    def test_elapsed_time_matches_the_distance(self):
        transport = self._transport()
        report = Mission(
            transport, floor_grid(), [Pose2D(-5.0, 0.0), Pose2D(5.0, 0.0)], options()
        ).run()
        # 10m を 0.5m/s なので20秒前後。ポーリング粒度ぶんの余裕を見る
        self.assertGreater(report.elapsed_s, 18.0)
        self.assertLess(report.elapsed_s, 26.0)

    def test_waits_out_an_obstacle_that_goes_away(self):
        """人が横切っただけなら、待って再開する。迂回はしない。"""

        transport = self._transport(
            obstacles=(Obstacle(0.0, 0.0, radius=0.8, appears_at_s=0.0, disappears_at_s=12.0),)
        )
        report = Mission(
            transport, floor_grid(), [Pose2D(-5.0, 0.0), Pose2D(5.0, 0.0)], options()
        ).run()
        self.assertIs(report.outcome, Outcome.COMPLETED)
        self.assertEqual(report.detours, 0)

    def test_detours_around_an_obstacle_that_stays(self):
        """どかない障害物は地図に書き込んで迂回する。"""

        transport = self._transport(obstacles=(Obstacle(0.0, 0.0, radius=0.8),))
        report = Mission(
            transport,
            floor_grid(),
            [Pose2D(-5.0, 0.0), Pose2D(5.0, 0.0)],
            options(obstacle_wait_s=5.0, detour_limit=6),
        ).run()
        self.assertIs(report.outcome, Outcome.COMPLETED)
        self.assertGreaterEqual(report.detours, 1)
        self.assertLess(transport.pose.distance_to(Pose2D(5.0, 0.0)), 0.3)

    def test_detour_obstacle_never_traps_the_robot(self):
        """仮の障害物が機体自身の位置を塞ぐと「出発点が壁の中」になって経路が引けない。

        `detour_ahead_m` を極端に小さくしても、機体から離す最小距離が効くこと。
        """

        transport = self._transport(obstacles=(Obstacle(0.0, 0.0, radius=0.8),))
        report = Mission(
            transport,
            floor_grid(),
            [Pose2D(-5.0, 0.0), Pose2D(5.0, 0.0)],
            options(obstacle_wait_s=5.0, detour_limit=6, detour_ahead_m=0.1),
        ).run()
        self.assertIsNot(report.outcome, Outcome.FAILED_ROUTE)
        self.assertGreaterEqual(report.detours, 1)

    def test_gives_up_when_detours_run_out(self):
        """通路そのものが塞がっているなら、諦めて報告する。"""

        wall = tuple(
            Obstacle(0.0, -6.0 + index * 0.5, radius=0.6) for index in range(25)
        )
        transport = self._transport(obstacles=wall)
        report = Mission(
            transport,
            floor_grid(),
            [Pose2D(-5.0, 0.0), Pose2D(5.0, 0.0)],
            options(obstacle_wait_s=2.0, detour_limit=2),
        ).run()
        self.assertIn(report.outcome, (Outcome.FAILED_BLOCKED, Outcome.FAILED_ROUTE))
        self.assertNotEqual(report.message, "")


class FailureTest(unittest.TestCase):
    def test_rejected_navigation_stops_the_mission(self):
        transport = FakeTransport(
            Pose2D(0.0, 0.0),
            FakeOptions(kinematics=False, known_maps=frozenset({MAP}), reject_navigate=True),
        )
        report = Mission(
            transport, floor_grid(), [Pose2D(0.0, 0.0), Pose2D(3.0, 0.0)], options()
        ).run()
        self.assertIs(report.outcome, Outcome.FAILED_REJECTED)

    def test_frozen_robot_is_detected(self):
        transport = FakeTransport(
            Pose2D(-5.0, 0.0), FakeOptions(known_maps=frozenset({MAP}), freeze_after_s=2.0)
        )
        report = Mission(
            transport,
            floor_grid(),
            [Pose2D(-5.0, 0.0), Pose2D(5.0, 0.0)],
            options(stall_timeout_s=10.0, arrival_timeout_margin_s=5.0),
        ).run()
        self.assertIn(report.outcome, (Outcome.FAILED_STALL, Outcome.FAILED_TIMEOUT))

    def test_unreachable_waypoint_is_reported_before_moving(self):
        blocked = build_grid(
            [(x * 0.1 - 5.0, y * 0.1 - 5.0, 0.0) for x in range(100) for y in range(100)]
            + [(0.0, 0.0, 1.0)],
            resolution=0.1,
            inflation=0.4,
        )
        transport = FakeTransport(
            Pose2D(-2.0, 0.0), FakeOptions(kinematics=False, known_maps=frozenset({MAP}))
        )
        report = Mission(transport, blocked, [Pose2D(-2.0, 0.0), Pose2D(0.0, 0.0)], options()).run()
        self.assertIs(report.outcome, Outcome.FAILED_ROUTE)
        self.assertFalse(any(api_id == API_NAVIGATE_POSE for api_id, _ in transport.calls))


class ArrivalTest(unittest.TestCase):
    def test_stale_arrival_flag_does_not_end_the_next_segment(self):
        """前の区間の is_arrived が残っていても、次の区間を即完了にしない。"""

        transport = FakeTransport(
            Pose2D(-9.0, 0.0), FakeOptions(known_maps=frozenset({MAP}))
        )
        report = Mission(
            transport, floor_grid(), [Pose2D(-9.0, 0.0), Pose2D(9.0, 0.0)], options()
        ).run()
        self.assertIs(report.outcome, Outcome.COMPLETED)
        # 18m を実際に歩いた時間になっているか（即完了なら数秒で終わる）
        self.assertGreater(report.elapsed_s, 30.0)

    def test_report_records_progress_even_on_failure(self):
        transport = FakeTransport(
            Pose2D(0.0, 0.0),
            FakeOptions(kinematics=False, known_maps=frozenset({MAP}), reject_navigate=True),
        )
        report = Mission(
            transport, floor_grid(), [Pose2D(0.0, 0.0), Pose2D(3.0, 0.0)], options()
        ).run()
        self.assertFalse(report.succeeded)
        self.assertGreater(len(report.log), 0)
        # 1102が受理されなかった区間も「試した」として数える
        self.assertEqual(report.segments_executed, 1)
        self.assertEqual(report.waypoints_reached, 0)


if __name__ == "__main__":
    unittest.main()
