"""モックの航法ソースと地図読み込みのテスト。

⚠️ **YOLO/torch を import しない。** この2つは重い上に GPU の有無で挙動が変わる。
ここで守りたいのは「UI が依存している契約」— つまり `NavState` の形と、モックの
状態機械が実機と同じ断り方をすること。映像側は Perception トラックのテストが見る。
"""
from __future__ import annotations

import sys
import time
import unittest
from pathlib import Path

MAIN = Path(__file__).resolve().parents[1] / "Main"
sys.path.insert(0, str(MAIN))

from mapimage import load_map                      # noqa: E402
from nav.base import COMMANDS, NavState, Pose      # noqa: E402
from nav.mock import MockNavSource                 # noqa: E402

MAP_YAML = (MAIN / "../../../Navigation/nav2_option/g1_ws/src/g1_navigation/maps"
                   "/room_a_map_20260911.yaml").resolve()


class MapTest(unittest.TestCase):
    """地図が読め、座標変換が ROS の規約どおりであること。"""

    def test_load(self) -> None:
        m = load_map(MAP_YAML)
        self.assertEqual((m.width, m.height), (476, 749))
        self.assertAlmostEqual(m.resolution, 0.05)
        self.assertTrue(m.png.startswith(b"\x89PNG"))

    def test_world_to_px(self) -> None:
        m = load_map(MAP_YAML)
        # 原点は左下。行は下向きなので、原点の row は height-1 になる
        self.assertEqual(m.world_to_px(m.origin_x, m.origin_y), (0, m.height - 1))
        # x が resolution ぶん増えると col が 1 増える
        col, _ = m.world_to_px(m.origin_x + m.resolution, m.origin_y)
        self.assertEqual(col, 1)


class NavStateTest(unittest.TestCase):
    """UI に渡す JSON の形。ここが変わると画面側が黙って壊れる。"""

    def test_keys(self) -> None:
        d = NavState(pose=Pose(1.0, 2.0, 30.0)).as_dict()
        self.assertEqual(
            set(d),
            {"connected", "pose", "goal", "plan", "route",
             "bridge_state", "patrol", "message"})
        self.assertEqual(d["pose"], {"x": 1.0, "y": 2.0, "yaw_deg": 30.0})
        self.assertEqual(set(d["patrol"]), {"state", "index", "total"})


class MockStateMachineTest(unittest.TestCase):
    """実機と同じ順序でしか進めないこと。"""

    def setUp(self) -> None:
        self.nav = MockNavSource(MAP_YAML, speed_mps=1.0)
        self.addCleanup(self.nav.close)
        # STANDBY から READY へ上がるのを待つ
        deadline = time.monotonic() + 6.0
        while self.nav.get_state().bridge_state == "STANDBY":
            if time.monotonic() > deadline:
                self.fail("READY に上がりませんでした")
            time.sleep(0.1)

    def test_is_marked_as_mock(self) -> None:
        # ⚠️ 画面に「モック」と出すための旗。落ちると作り物を実機と取り違える
        self.assertTrue(self.nav.is_mock)

    def test_patrol_needs_navigating(self) -> None:
        r = self.nav.send_command("patrol_start")
        self.assertFalse(r.ok)
        self.assertIn("NAVIGATING", r.message)

    def test_full_sequence(self) -> None:
        self.assertTrue(self.nav.send_command("enable_navigation").ok)
        self.assertEqual(self.nav.get_state().bridge_state, "NAVIGATING")
        self.assertTrue(self.nav.send_command("patrol_start").ok)
        self.assertEqual(self.nav.get_state().patrol_state, "RUNNING")
        self.assertTrue(self.nav.send_command("patrol_pause").ok)
        # ⚠️ 実機の patrol_node は一時停止で "HOLD" にする。ここを揃えておかないと
        # 画面のボタン活性判定がモックでだけ正しく動く、という食い違いが起きる
        self.assertEqual(self.nav.get_state().patrol_state, "HOLD")

    def test_moves_without_being_polled(self) -> None:
        """⚠️ 取得の回数で歩く速さが変わらないこと。

        以前は get_state() の中で時間を進めていたため、1Hz で問い合わせると
        速度が半分になっていた（2026-09-20 に実測して直した）。
        """
        self.nav.send_command("enable_navigation")
        self.nav.send_command("patrol_start")
        a = self.nav.get_state().pose
        time.sleep(2.0)                      # わざと1回も問い合わせない
        b = self.nav.get_state().pose
        moved = ((b.x - a.x) ** 2 + (b.y - a.y) ** 2) ** 0.5
        self.assertGreater(moved, 1.0, f"2秒で {moved:.2f}m しか進んでいない")

    def test_unknown_command_is_refused(self) -> None:
        self.assertFalse(self.nav.send_command("launch_missile").ok)

    def test_all_declared_commands_are_handled(self) -> None:
        """COMMANDS に並べた操作がすべて応答すること（綴り違いを拾う）。"""
        for name in COMMANDS:
            with self.subTest(name=name):
                r = self.nav.send_command(name)
                self.assertNotIn("知らない操作", r.message)


if __name__ == "__main__":
    unittest.main()
