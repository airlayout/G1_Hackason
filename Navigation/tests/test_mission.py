"""`nav/mission.py`: 1804 -> 1102 連打 -> 到達待ち、の状態機械。

ここだけは**台本どおりに答える相手**を使う。本物の相手（`sim/slam_service.py`）は
MuJoCo と torch が要るうえ物理で動くので、「507 を 2 回返す」「進捗を止める」
といった一点を狙った状況を作れない。台本は歩行の代わりではなく、
状態機械に特定の入力を与えるための道具（テストダブル）で、
これで sim の代わりにするつもりは無い。sim 込みの検証は `test_sim.py` にある。
"""

import json
import unittest

from nav.mission import Mission, MissionOptions, Outcome
from nav.occupancy import build_grid
from nav.protocol import (
    API_INIT_POSE,
    API_NAVIGATE_POSE,
    CTRL_NAME_NOT_INIT,
    ERROR_LOAD_PCD_FAILED,
    ERROR_OK,
    Pose2D,
    parse_ctrl_info,
    parse_response,
    parse_task_result,
)
from nav.transport import SlamTransport
from tests.test_occupancy import box_cloud

MAP = "/home/unitree/test1.pcd"


class ScriptedTransport(SlamTransport):
    """台本どおりに答える `SlamTransport`。時計は呼ばれた回数で進む。

    既定は「1804 は成功、1102 は `arrive_after_polls` 回のポーリングで到達」。
    そこから 1 つだけ崩して、状態機械がどう振る舞うかを見る。
    """

    def __init__(
        self,
        start: Pose2D,
        *,
        init_failures: int = 0,
        unknown_map: bool = False,
        reject_navigate: bool = False,
        arrive_after_polls: int = 2,
        blocked_from_poll: int | None = None,
        blocked_until_poll: int | None = None,
        freeze_progress: bool = False,
        freeze_progress_after_poll: int | None = None,
        emit_task_result: bool = True,
        drift_while_blocked: tuple[float, float] = (0.0, 0.0),
    ) -> None:
        self.pose = start
        self.calls: list[tuple[int, dict]] = []
        self._init_failures = init_failures
        self._unknown_map = unknown_map
        self._reject_navigate = reject_navigate
        self._arrive_after_polls = arrive_after_polls
        self._blocked_from = blocked_from_poll
        self._blocked_until = blocked_until_poll
        self._freeze_progress = freeze_progress
        self._freeze_after_poll = freeze_progress_after_poll
        self._emit_task_result = emit_task_result
        self._drift = drift_while_blocked
        self._time = 0.0
        self._polls = 0
        self._init_attempts = 0
        self._target: Pose2D | None = None
        self._arrived = False
        self._task_results: list = []
        self._progress = 0.0

    # -- SlamTransport --

    def call(self, api_id: int, request: dict):
        self.calls.append((api_id, request))
        data = request.get("data") or {}
        if api_id == API_INIT_POSE:
            self._init_attempts += 1
            if self._unknown_map or self._init_attempts <= self._init_failures:
                return _response(False, ERROR_LOAD_PCD_FAILED, "Load pcd failed.")
            return _response(True, ERROR_OK, "")
        if api_id == API_NAVIGATE_POSE:
            if self._reject_navigate:
                return _response(False, ERROR_OK, "rejected")
            target = data["targetPose"]
            self._target = Pose2D(float(target["x"]), float(target["y"]))
            self._arrived = False
            self._polls = 0
            self._progress = 0.0
            return _response(True, ERROR_OK, "")
        return _response(True, ERROR_OK, "")

    def latest_ctrl_info(self):
        return parse_ctrl_info(json.dumps(self._frame()))

    def take_task_results(self) -> list:
        taken, self._task_results = self._task_results, []
        return taken

    def now(self) -> float:
        return self._time

    def sleep(self, seconds: float) -> None:
        self._time += seconds
        self._polls += 1
        if self._target is None or self._arrived:
            return
        if self._is_blocked():
            # 塞がれていても機体は完全には止まらない（歩容の揺れ・定位のずれ）。
            # 迂回で置いた円の中へ入り込む状況をこれで作る。
            self.pose = Pose2D(
                self.pose.x + self._drift[0], self.pose.y + self._drift[1], self.pose.yaw
            )
            return
        if not self._is_frozen():
            self._progress = min(1.0, self._polls / max(1, self._arrive_after_polls))
        if self._polls >= self._arrive_after_polls:
            self.pose = self._target
            self._arrived = True
            self._target = None
            if self._emit_task_result:
                self._task_results.append(
                    parse_task_result(
                        json.dumps({"data": {"is_arrived": True, "targetNodeName": 0}})
                    )
                )

    # -- 内部 --

    def _is_frozen(self) -> bool:
        if self._freeze_progress:
            return True
        return self._freeze_after_poll is not None and self._polls >= self._freeze_after_poll

    def _is_blocked(self) -> bool:
        if self._blocked_from is None:
            return False
        if self._polls < self._blocked_from:
            return False
        return self._blocked_until is None or self._polls < self._blocked_until

    def _frame(self) -> dict:
        return {
            "type": "ctrl_info",
            "errorCode": 0,
            "info": "running" if self._init_attempts else CTRL_NAME_NOT_INIT,
            "data": {
                "stateMachine": {
                    "state": "follow" if self._target else "ready",
                    "ctrName": "pid" if self._init_attempts else CTRL_NAME_NOT_INIT,
                    "isPause": False, "vx": 0.004, "vy": 0.005, "vyaw": 0.065,
                },
                "currentPose": {"x": self.pose.x, "y": self.pose.y, "yaw": self.pose.yaw},
                "is_arrived": self._arrived,
                "obsInfo": {"state": self._is_blocked(), "time": 0.0},
                "progress": {"used_time": self._time, "completion_percentage": self._progress},
            },
        }


def _response(succeed: bool, error_code: int, info: str):
    return parse_response(
        json.dumps({"succeed": succeed, "errorCode": error_code, "info": info, "data": {}})
    )


def open_grid():
    return build_grid(box_cloud(-10, 10, -10, 10), inflation=0.0)


def make_mission(transport, waypoints, **overrides):
    options = MissionOptions(map_address=MAP, **overrides)
    return Mission(transport, open_grid(), waypoints, options)


SQUARE = [Pose2D(-5, -5), Pose2D(5, -5), Pose2D(5, 5), Pose2D(-5, -5)]


class InitTest(unittest.TestCase):
    def test_a_clean_run_completes(self):
        transport = ScriptedTransport(SQUARE[0])
        report = make_mission(transport, SQUARE).run()
        self.assertEqual(report.outcome, Outcome.COMPLETED)
        self.assertTrue(report.succeeded)

    def test_1804_comes_first(self):
        transport = ScriptedTransport(SQUARE[0])
        make_mission(transport, SQUARE).run()
        self.assertEqual(transport.calls[0][0], API_INIT_POSE)

    def test_1804_carries_the_configured_map_address(self):
        transport = ScriptedTransport(SQUARE[0])
        make_mission(transport, SQUARE).run()
        self.assertEqual(transport.calls[0][1]["data"]["address"], MAP)

    def test_507_is_retried_and_then_succeeds(self):
        transport = ScriptedTransport(SQUARE[0], init_failures=2)
        report = make_mission(transport, SQUARE).run()
        self.assertEqual(report.outcome, Outcome.COMPLETED)
        self.assertEqual(sum(1 for api, _ in transport.calls if api == API_INIT_POSE), 3)

    def test_retrying_is_bounded(self):
        """無限リトライは原因を隠すだけ。既定は 3 回で諦める。"""

        transport = ScriptedTransport(SQUARE[0], unknown_map=True)
        report = make_mission(transport, SQUARE).run()
        self.assertEqual(report.outcome, Outcome.FAILED_INIT)
        self.assertEqual(sum(1 for api, _ in transport.calls if api == API_INIT_POSE), 3)

    def test_the_init_failure_message_names_the_map_and_pc1(self):
        transport = ScriptedTransport(SQUARE[0], unknown_map=True)
        report = make_mission(transport, SQUARE).run()
        self.assertIn(MAP, report.message)
        self.assertIn("192.168.123.161", report.message)

    def test_nothing_is_navigated_when_init_fails(self):
        transport = ScriptedTransport(SQUARE[0], unknown_map=True)
        make_mission(transport, SQUARE).run()
        self.assertFalse([api for api, _ in transport.calls if api == API_NAVIGATE_POSE])


class SegmentTest(unittest.TestCase):
    def test_one_1102_per_segment(self):
        transport = ScriptedTransport(SQUARE[0])
        report = make_mission(transport, SQUARE).run()
        navigates = sum(1 for api, _ in transport.calls if api == API_NAVIGATE_POSE)
        self.assertEqual(navigates, report.segments_executed)

    def test_every_waypoint_is_reached(self):
        transport = ScriptedTransport(SQUARE[0])
        report = make_mission(transport, SQUARE).run()
        self.assertEqual(report.waypoints_reached, len(SQUARE) - 1)

    def test_a_rejected_1102_stops_the_mission(self):
        transport = ScriptedTransport(SQUARE[0], reject_navigate=True)
        report = make_mission(transport, SQUARE).run()
        self.assertEqual(report.outcome, Outcome.FAILED_REJECTED)

    def test_arrival_works_without_task_result(self):
        """`task_result` が実機で飛ばなくても `ctrl_info.is_arrived` で拾える。"""

        transport = ScriptedTransport(SQUARE[0], emit_task_result=False)
        report = make_mission(transport, SQUARE).run()
        self.assertEqual(report.outcome, Outcome.COMPLETED)

    def test_a_stale_arrival_flag_does_not_finish_the_next_segment(self):
        """前の区間の is_arrived が残っていても、一度 False を見るまで信じない。"""

        transport = ScriptedTransport(SQUARE[0], arrive_after_polls=3)
        report = make_mission(transport, SQUARE).run()
        navigates = sum(1 for api, _ in transport.calls if api == API_NAVIGATE_POSE)
        self.assertEqual(report.outcome, Outcome.COMPLETED)
        self.assertGreaterEqual(navigates, 3)

    def test_a_frozen_progress_is_reported_as_a_stall(self):
        transport = ScriptedTransport(
            SQUARE[0], arrive_after_polls=10_000, freeze_progress=True
        )
        report = make_mission(transport, SQUARE, stall_timeout_s=5.0).run()
        self.assertEqual(report.outcome, Outcome.FAILED_STALL)

    def test_earlier_blocked_time_does_not_hide_a_later_stall(self):
        """停滞の判定から引く塞がれ時間は、**最後に進捗が動いてから**の分だけ。

        区間開始からの累計を引いていたころは、序盤に長く塞がれた区間ほど
        後半の本物の停滞に気付けなくなっていた（塞がれた時間を二重に割り引くため）。

        台本: 5 秒塞がれる -> 解けて進捗が動く -> 進捗が止まる。
        停滞の上限 2 秒なので、進捗が止まってから 2 秒ちょっとで
        STALL になるはず。序盤の 5 秒を引いてしまうと 7 秒以上かかる。
        """

        transport = ScriptedTransport(
            SQUARE[0],
            arrive_after_polls=10_000,
            blocked_from_poll=1,
            blocked_until_poll=26,        # 25 ポーリング x 0.2s = 5.0 秒
            freeze_progress_after_poll=30,
        )
        report = make_mission(
            transport, SQUARE,
            stall_timeout_s=2.0,
            obstacle_wait_s=100.0,        # 塞がれても迂回に切り替えない
            arrival_timeout_margin_s=10_000.0,
        ).run()
        self.assertEqual(report.outcome, Outcome.FAILED_STALL)
        self.assertLess(
            report.elapsed_s, 10.0,
            f"序盤の塞がれ時間まで引いている（{report.elapsed_s:.1f}秒かかった）",
        )

    def test_blocked_time_is_still_excluded_from_the_stall_check(self):
        """引きすぎを直したついでに引かなくなった、では困る。

        ずっと塞がれている間は進捗が動かないが、それは停滞ではない。
        """

        transport = ScriptedTransport(
            SQUARE[0], arrive_after_polls=10_000, blocked_from_poll=1
        )
        report = make_mission(
            transport, SQUARE,
            stall_timeout_s=2.0,
            obstacle_wait_s=10_000.0,     # 迂回にも切り替えない
            arrival_timeout_margin_s=10_000.0,
        ).run()
        self.assertNotEqual(report.outcome, Outcome.FAILED_STALL)

    def test_a_segment_that_never_arrives_times_out(self):
        transport = ScriptedTransport(SQUARE[0], arrive_after_polls=10_000)
        report = make_mission(
            transport, SQUARE, stall_timeout_s=10_000.0, arrival_timeout_margin_s=2.0
        ).run()
        self.assertEqual(report.outcome, Outcome.FAILED_TIMEOUT)


class ObstacleTest(unittest.TestCase):
    def test_a_short_block_is_waited_out_without_detouring(self):
        """人が横切っただけで迂回すると巡回が無用に伸びる。"""

        transport = ScriptedTransport(
            SQUARE[0], arrive_after_polls=6, blocked_from_poll=2, blocked_until_poll=4
        )
        report = make_mission(transport, SQUARE, obstacle_wait_s=15.0).run()
        self.assertEqual(report.outcome, Outcome.COMPLETED)
        self.assertEqual(report.detours, 0)

    def test_a_lasting_block_causes_a_detour(self):
        transport = ScriptedTransport(
            SQUARE[0], arrive_after_polls=10_000, blocked_from_poll=1
        )
        report = make_mission(
            transport, SQUARE, obstacle_wait_s=1.0, detour_limit=2
        ).run()
        self.assertEqual(report.outcome, Outcome.FAILED_BLOCKED)
        self.assertEqual(report.detours, 2)

    def test_the_detour_limit_is_honoured(self):
        transport = ScriptedTransport(
            SQUARE[0], arrive_after_polls=10_000, blocked_from_poll=1
        )
        report = make_mission(
            transport, SQUARE, obstacle_wait_s=1.0, detour_limit=1
        ).run()
        self.assertEqual(report.detours, 1)

    def test_giving_up_says_why(self):
        transport = ScriptedTransport(
            SQUARE[0], arrive_after_polls=10_000, blocked_from_poll=1
        )
        report = make_mission(transport, SQUARE, obstacle_wait_s=1.0, detour_limit=1).run()
        self.assertIn("迂回", report.message)

    def test_waiting_time_is_not_counted_against_the_arrival_budget(self):
        """人待ちで巡回全体を落とすのは行き過ぎ。塞がれている間は時計を止める。"""

        transport = ScriptedTransport(
            SQUARE[0], arrive_after_polls=4, blocked_from_poll=1, blocked_until_poll=200
        )
        report = make_mission(
            transport, SQUARE, obstacle_wait_s=10_000.0, arrival_timeout_margin_s=1.0
        ).run()
        # 塞がれ続けているので到達しない。ただし TIMEOUT ではなく STALL でもなく、
        # 「待ち続けている」状態のまま打ち切り時間を使い切らないこと
        self.assertNotEqual(report.outcome, Outcome.FAILED_TIMEOUT)


class FootingTest(unittest.TestCase):
    def test_drifting_into_your_own_detour_guess_does_not_strand_the_robot(self):
        """迂回で置いた円の中へ入り込んでも、経路を引き直せること。

        sim で実測した状況の回帰（1 回目の迂回で置いた円の中心へ、17 秒後の
        自分が 0.95m まで入り、「出発点が障害物の中にある」で止まった）。
        """

        transport = ScriptedTransport(
            SQUARE[0],
            arrive_after_polls=10_000,
            blocked_from_poll=1,
            drift_while_blocked=(0.35, 0.0),
        )
        report = make_mission(
            transport, SQUARE, obstacle_wait_s=1.0, detour_limit=3
        ).run()
        self.assertNotEqual(report.outcome, Outcome.FAILED_ROUTE)
        self.assertTrue(any("足元" in line for line in report.log), "\n".join(report.log))

    def test_the_hole_is_punched_only_where_the_robot_stands(self):
        """憶測をまるごと取り下げると、真下の本物の障害物へ戻ってしまう。

        sim で実測: 取り下げる実装では同じ場所で迂回を繰り返して上限に達した。
        穴は足元だけに空け、その先の推定は残す。
        """

        transport = ScriptedTransport(
            SQUARE[0],
            arrive_after_polls=10_000,
            blocked_from_poll=1,
            drift_while_blocked=(0.35, 0.0),
        )
        mission = make_mission(transport, SQUARE, obstacle_wait_s=1.0, detour_limit=3)
        mission.run()
        # 憶測は 1 つも消えていない（消えるのは足元のセルだけ）
        self.assertEqual(len(mission._guessed_obstacles), 3)

    def test_a_footing_outside_every_guess_is_left_alone(self):
        transport = ScriptedTransport(
            SQUARE[0], arrive_after_polls=10_000, blocked_from_poll=1
        )
        report = make_mission(
            transport, SQUARE, obstacle_wait_s=1.0, detour_limit=2
        ).run()
        self.assertFalse(any("足元" in line for line in report.log))
        self.assertEqual(report.detours, 2)

    def test_the_measured_map_is_never_modified(self):
        """迂回は憶測。実測の壁を書き換えると取り返しがつかない。"""

        grid = open_grid()
        before = grid.blocked.copy()
        transport = ScriptedTransport(
            SQUARE[0], arrive_after_polls=10_000, blocked_from_poll=1
        )
        Mission(
            transport, grid, SQUARE,
            MissionOptions(map_address=MAP, obstacle_wait_s=1.0, detour_limit=2),
        ).run()
        self.assertTrue((grid.blocked == before).all())


class LapTest(unittest.TestCase):
    def test_three_laps_reach_three_times_as_many_waypoints(self):
        one = make_mission(ScriptedTransport(SQUARE[0]), SQUARE, laps=1).run()
        three = make_mission(ScriptedTransport(SQUARE[0]), SQUARE, laps=3).run()
        self.assertEqual(three.waypoints_reached, one.waypoints_reached * 3)

    def test_zero_laps_is_rejected(self):
        with self.assertRaises(ValueError):
            make_mission(ScriptedTransport(SQUARE[0]), SQUARE, laps=0)

    def test_a_single_waypoint_is_rejected(self):
        with self.assertRaises(ValueError):
            make_mission(ScriptedTransport(SQUARE[0]), [Pose2D(0, 0)])


class ReportTest(unittest.TestCase):
    def test_the_log_is_timestamped(self):
        report = make_mission(ScriptedTransport(SQUARE[0]), SQUARE).run()
        self.assertTrue(report.log)
        self.assertTrue(report.log[0].startswith("["))

    def test_elapsed_time_is_recorded_even_on_failure(self):
        transport = ScriptedTransport(SQUARE[0], unknown_map=True)
        report = make_mission(transport, SQUARE).run()
        self.assertGreater(report.elapsed_s, 0.0)

    def test_a_failure_keeps_the_partial_progress(self):
        """どこまで行けたかが分からないと、次に何をすればいいか決められない。"""

        transport = ScriptedTransport(SQUARE[0], arrive_after_polls=10_000)
        report = make_mission(transport, SQUARE, arrival_timeout_margin_s=1.0).run()
        self.assertFalse(report.succeeded)
        self.assertEqual(report.segments_executed, 1)


if __name__ == "__main__":
    unittest.main()
