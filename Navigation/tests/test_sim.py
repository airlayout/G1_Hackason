"""歩行ポリシーを実際に回す検証。**mujoco / torch / 資産が要る。**

無ければ全部 skip する。CI は素の Python 3.12 で来るので常に skip され、
「何件 skip したか」が出力に残る（緑＝全部検証した、ではない）。
手元で回すには:

```bash
cd Navigation
bash sim/fetch_assets.sh
uv sync --group mujoco --group walk
bash tests/run_tests.sh
```

ここで固定するのは 2 つ:

1. **歩行ポリシーが実測どおりに歩くこと。** 資産の取得や環境が壊れたら気付く
2. **ナビ層と歩行が噛み合うこと。** 経路が引けるだけでなく、
   物理で歩いて実際に着く
"""

import math
import unittest

from nav.mission import Mission, MissionOptions, Outcome
from nav.protocol import Pose2D
from sim.rooms import EMPTY_ROOM, TEST_ROOM, DynamicObstacle, patrol_waypoints

try:  # 資産と重い依存が揃っているときだけ回す
    import mujoco  # noqa: F401
    import torch  # noqa: F401

    from sim.g1_walker import POLICY_PATH, G1Walker
    from sim.slam_service import SimOptions, SimTransport

    REASON = None if POLICY_PATH.exists() else "sim/assets が無い（sim/fetch_assets.sh）"
except ImportError as error:  # pragma: no cover - CI は必ずここに来る
    REASON = f"sim の依存が無い（{error}）"

needs_sim = unittest.skipIf(REASON is not None, REASON or "")

MAP = "/home/unitree/test1.pcd"


@needs_sim
class WalkerTest(unittest.TestCase):
    """公式 `deploy_mujoco.py` の移植が壊れていないことの確認。

    期待値は 2026-09-02 に手元の Mac で測った値。±30% で見るのは、
    物理の初期値と数値誤差で走るたびに少し動くため。
    """

    @classmethod
    def setUpClass(cls):
        cls.walker = G1Walker(EMPTY_ROOM.write_scene())

    def test_it_starts_at_the_spawn(self):
        self.assertAlmostEqual(self.walker.pose.x, EMPTY_ROOM.spawn.x, places=2)
        self.assertAlmostEqual(self.walker.pose.y, EMPTY_ROOM.spawn.y, places=2)

    def test_it_walks_forward_at_about_the_commanded_speed(self):
        walker = G1Walker(EMPTY_ROOM.write_scene())
        before = walker.pose
        walker.set_command(0.5, 0.0, 0.0)
        walker.step(5.0)
        moved = math.hypot(walker.pose.x - before.x, walker.pose.y - before.y)
        self.assertGreater(moved, 1.5, "前進していない（実測 2.22m）")
        self.assertLess(moved, 3.0)
        self.assertFalse(walker.has_fallen)

    def test_it_turns_in_place(self):
        """29DoF を捨てて 12DoF を選んだ理由がこれ。その場旋回ができること。"""

        walker = G1Walker(EMPTY_ROOM.write_scene())
        before = walker.pose.yaw
        walker.set_command(0.0, 0.0, 0.5)
        walker.step(5.0)
        turned = abs(math.degrees(_wrap(walker.pose.yaw - before)))
        self.assertGreater(turned, 90.0, "その場旋回が弱すぎる（実測 134°）")
        self.assertFalse(walker.has_fallen)

    def test_commands_are_clamped_to_the_trained_range(self):
        walker = G1Walker(EMPTY_ROOM.write_scene())
        walker.set_command(99.0, 99.0, 99.0)
        walker.step(3.0)
        self.assertFalse(walker.has_fallen, "範囲外の指令で転んだ。丸めが効いていない")

    def test_the_policy_output_does_not_diverge(self):
        """29DoF の失敗時は |action| が 2.6e+07 まで飛んだ。正常なら 3 前後。"""

        walker = G1Walker(EMPTY_ROOM.write_scene())
        walker.set_command(0.5, 0.0, 0.3)
        walker.step(10.0)
        self.assertLess(walker.peak_action, 10.0)

    def test_standing_still_does_not_fall_over(self):
        """指令 0 でも足踏みでバランスを取る。制御を止めると転ぶ。"""

        walker = G1Walker(EMPTY_ROOM.write_scene())
        walker.stop()
        walker.step(10.0)
        self.assertFalse(walker.has_fallen)

    def test_sim_time_tracks_the_steps(self):
        walker = G1Walker(EMPTY_ROOM.write_scene())
        walker.stop()
        walker.step(2.0)
        self.assertAlmostEqual(walker.sim_time, 2.0, places=2)


@needs_sim
class SlamServiceTest(unittest.TestCase):
    def make(self, **overrides):
        options = SimOptions(known_maps=frozenset({MAP}), **overrides)
        return SimTransport(EMPTY_ROOM, options)

    def test_1804_does_not_teleport_the_robot(self):
        """実機の 1804 は「いまここに居る」と教えるだけ。移動命令ではない。"""

        from nav.protocol import API_INIT_POSE, init_pose_request

        transport = self.make()
        before = transport.pose
        transport.call(API_INIT_POSE, init_pose_request(MAP, Pose2D(3.0, 3.0, 0.0)))
        self.assertAlmostEqual(transport.pose.x, before.x, places=3)
        self.assertAlmostEqual(transport.pose.y, before.y, places=3)

    def test_an_unknown_map_gives_507(self):
        from nav.protocol import API_INIT_POSE, init_pose_request

        transport = self.make()
        response = transport.call(API_INIT_POSE, init_pose_request("/nope.pcd", Pose2D(0, 0)))
        self.assertFalse(response.succeed)
        self.assertTrue(response.is_load_pcd_failure)

    def test_navigating_before_1804_is_refused(self):
        from nav.protocol import API_NAVIGATE_POSE, navigate_request

        transport = self.make()
        self.assertFalse(transport.call(API_NAVIGATE_POSE, navigate_request(Pose2D(0, 0))).succeed)

    def test_it_walks_to_a_commanded_target(self):
        from nav.protocol import API_INIT_POSE, API_NAVIGATE_POSE, init_pose_request, navigate_request

        transport = self.make()
        start = transport.pose
        transport.call(API_INIT_POSE, init_pose_request(MAP, start))
        target = Pose2D(start.x + 2.0, start.y, 0.0)
        transport.call(API_NAVIGATE_POSE, navigate_request(target))
        for _ in range(150):  # 最大 30 秒
            transport.sleep(0.2)
            if transport.take_task_results():
                break
        distance = math.hypot(transport.pose.x - target.x, transport.pose.y - target.y)
        self.assertLess(distance, 0.4, f"目標に着いていない（{distance:.2f}m 手前）")
        self.assertFalse(transport.has_fallen)

    def test_ctrl_info_says_not_init_before_1804(self):
        self.assertFalse(self.make().latest_ctrl_info().initialized)

    def test_the_clock_is_sim_time(self):
        transport = self.make()
        transport.sleep(1.0)
        self.assertAlmostEqual(transport.now(), 1.0, places=1)

    def test_by_default_it_runs_as_fast_as_it_can(self):
        """テストと CI で待たせる理由は無い。既定は実時間に縛らない。"""

        import time

        transport = self.make()
        started = time.monotonic()
        transport.sleep(2.0)
        self.assertLess(
            time.monotonic() - started, 1.0,
            "実時間を待ってしまっている（既定は realtime_factor=None のはず）",
        )

    def test_a_realtime_factor_slows_it_down(self):
        """`--viewer` のとき目で追えるようにするための仕掛け。"""

        import time

        transport = self.make(realtime_factor=4.0)
        started = time.monotonic()
        transport.sleep(2.0)   # sim 2 秒 / 4 倍速 = 実時間 0.5 秒のはず
        elapsed = time.monotonic() - started
        self.assertGreater(elapsed, 0.3, f"待っていない（{elapsed:.2f}秒）")
        self.assertLess(elapsed, 1.5, f"待ちすぎ（{elapsed:.2f}秒）")

    def test_the_frame_hook_is_called_on_the_calling_thread(self):
        """ビューアの描き替えはミッションと同じスレッドで呼ぶ。

        別スレッドから `viewer.sync()` すると `mj_step` と衝突して
        プロセスごと落ちる（実測・exit 133）ので、ここが崩れると GUI が死ぬ。
        """

        import threading

        transport = self.make()
        seen: list[int] = []
        here = threading.get_ident()
        transport.set_frame_hook(lambda: seen.append(threading.get_ident()))
        transport.sleep(0.5)
        self.assertGreater(len(seen), 0, "フックが一度も呼ばれていない")
        self.assertEqual(set(seen), {here}, "別スレッドから呼ばれている")

    def test_the_frame_hook_can_be_removed(self):
        transport = self.make()
        calls: list[int] = []
        transport.set_frame_hook(lambda: calls.append(1))
        transport.sleep(0.3)
        transport.set_frame_hook(None)
        before = len(calls)
        transport.sleep(0.3)
        self.assertEqual(len(calls), before)


@needs_sim
class PatrolTest(unittest.TestCase):
    """ナビ層 + 歩行を通しで回す。ここが Phase 3.5 の目的そのもの。"""

    def run_patrol(self, room, obstacles=(), **mission_overrides):
        transport = SimTransport(
            room, SimOptions(known_maps=frozenset({MAP}), obstacles=obstacles)
        )
        waypoints = patrol_waypoints(room)
        waypoints = [Pose2D(room.spawn.x, room.spawn.y, waypoints[0].yaw), *waypoints[1:]]
        mission = Mission(
            transport, room.grid(), waypoints,
            MissionOptions(map_address=MAP, **mission_overrides),
        )
        return mission.run(), transport

    def test_the_test_room_patrol_completes(self):
        report, transport = self.run_patrol(TEST_ROOM)
        self.assertEqual(report.outcome, Outcome.COMPLETED, report.message)
        self.assertEqual(report.waypoints_reached, 4)
        self.assertFalse(transport.has_fallen)

    def test_the_robot_ends_up_where_it_started(self):
        report, transport = self.run_patrol(TEST_ROOM)
        self.assertEqual(report.outcome, Outcome.COMPLETED, report.message)
        self.assertLess(
            math.hypot(
                transport.pose.x - TEST_ROOM.spawn.x, transport.pose.y - TEST_ROOM.spawn.y
            ),
            0.5,
        )

    def test_a_passing_obstacle_is_waited_out_not_detoured(self):
        """人が横切っただけで迂回すると巡回が無用に伸びる。"""

        crossing = (DynamicObstacle(-2.0, -1.62, 0.35, appears_at_s=3.0, disappears_at_s=14.0),)
        report, _ = self.run_patrol(TEST_ROOM, crossing, obstacle_wait_s=15.0)
        self.assertEqual(report.outcome, Outcome.COMPLETED, report.message)
        self.assertEqual(report.detours, 0)

    def test_a_parked_obstacle_is_detoured_around(self):
        parked = (DynamicObstacle(-2.0, -1.62, 0.35),)
        report, transport = self.run_patrol(TEST_ROOM, parked, obstacle_wait_s=15.0)
        self.assertEqual(report.outcome, Outcome.COMPLETED, report.message)
        self.assertGreater(report.detours, 0)
        self.assertFalse(transport.has_fallen)

    def test_one_detour_is_enough_when_the_position_is_measured(self):
        """実測の位置が使えると、迂回1回で回り込めること。

        **以前はここで `FAILED_BLOCKED` になっていた。** 位置が分からず
        「前方1.5mに半径1.0m」と当て推量で置いていたため、円が実物から
        0.35m ずれ、1回では回り込めなかった。生 LiDAR から位置を測るように
        してから、迂回上限1でも完走する（実測: 迂回2回→1回、sim 152→102秒）。
        """

        parked = (DynamicObstacle(-2.0, -1.62, 0.35),)
        report, transport = self.run_patrol(
            TEST_ROOM, parked, obstacle_wait_s=15.0, detour_limit=1
        )
        self.assertEqual(report.outcome, Outcome.COMPLETED, report.message)
        self.assertEqual(report.detours, 1)
        self.assertFalse(transport.has_fallen)

    def test_giving_up_is_reported_rather_than_walking_into_it(self):
        """通れないときは、理由を添えて止まる。転ばない。

        通路そのものを塞ぐので、位置を測れても回り込む先が無い。
        """

        # 柱と南壁の隙間、仕切りの西側の通路をまとめて塞ぐ
        wall = tuple(
            DynamicObstacle(0.0, y, 0.5) for y in (-2.6, -1.8, -1.0, -0.2, 0.6, 1.4, 2.2, 2.9)
        )
        report, transport = self.run_patrol(
            TEST_ROOM, wall, obstacle_wait_s=2.0, detour_limit=2
        )
        self.assertNotEqual(report.outcome, Outcome.COMPLETED)
        self.assertTrue(report.message, "理由が空のまま止まっている")
        self.assertFalse(transport.has_fallen, "諦めたのに転んでいる")

    def test_the_measured_position_is_close_to_the_truth(self):
        """推定した障害物の位置が、置いた真値に近いこと。

        sim で 6 通り測った実測は位置誤差 平均 0.100m / 最大 0.182m、
        半径誤差 平均 -0.048m（手前の面しか見えないので小さめに出る）。
        ここは代表 1 点だけ固定する。
        """

        from nav.protocol import (
            API_INIT_POSE,
            API_NAVIGATE_POSE,
            init_pose_request,
            navigate_request,
        )

        true_x, true_y, true_r = -2.0, -1.62, 0.35
        transport = SimTransport(
            TEST_ROOM,
            SimOptions(
                known_maps=frozenset({MAP}),
                obstacles=(DynamicObstacle(true_x, true_y, true_r),),
            ),
        )
        transport.call(API_INIT_POSE, init_pose_request(MAP, TEST_ROOM.spawn))
        transport.call(API_NAVIGATE_POSE, navigate_request(Pose2D(4.5, -2.0, 0.0)))
        observed = None
        for _ in range(300):
            transport.sleep(0.2)
            if transport.is_blocked:
                observed = transport.observe_obstacle()
                if observed is not None:
                    break
        self.assertIsNotNone(observed, "塞がれたのに位置を返していない")
        error = math.hypot(observed.x - true_x, observed.y - true_y)
        self.assertLess(error, 0.35, f"位置がずれすぎ（{error:.3f}m）")
        self.assertGreater(observed.radius, true_r - 0.20, "半径が小さすぎ")
        self.assertLess(observed.radius, true_r + 0.20, "半径が大きすぎ")
        self.assertGreater(observed.point_count, 20)

    def test_the_position_is_unknown_when_nothing_blocks(self):
        transport = SimTransport(TEST_ROOM, SimOptions(known_maps=frozenset({MAP})))
        self.assertIsNone(transport.observe_obstacle())


def _wrap(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


if __name__ == "__main__":
    unittest.main()
