"""`nav.protocol` のテスト。

`ctrl_info` と 507 の題材は、2026-09-01に実機から採取した実データを使う
（`Navigation/README.md`「実測で確定した挙動」節）。
仕様書ではなく実機が流したものでテストするのは、実機が仕様書の
スーパーセットを流すことが分かっているため。
"""

from __future__ import annotations

import json
import math
import unittest

from nav import protocol
from nav.geometry import Pose2D

# 実機の待機時（1804を投げる前）に rt/slam_info で実際に流れていたもの。
IDLE_CTRL_INFO = {
    "type": "ctrl_info",
    "errorCode": 0,
    "info": "not init",
    "data": {
        "stateMachine": {
            "state": "ready", "ctrName": "not init", "isOpenPlan": False,
            "isPause": False, "isBack": False, "isRotate": False,
            "isClimbStairs": False, "vx": 0.004, "vy": 0.005, "vyaw": 0.065,
        },
        "currentPose": {"x": 0, "y": 0, "z": 0, "roll": 0, "pitch": 0, "yaw": 0},
        "is_arrived": False,
        "targetNodeName": 0,
        "obsInfo": {"state": False, "time": 0.0},
        "progress": {"used_time": 0.0, "last_time": 0.0, "completion_percentage": 0.0},
        "total_distance": -1.0,
    },
}

# 1804に5パターンのaddressを与えて、すべて同一で返ってきたもの。
LOAD_PCD_FAILED = {"succeed": False, "errorCode": 507, "info": "Load pcd failed.", "data": {}}


class RequestTest(unittest.TestCase):
    def test_start_mapping_uses_fixed_slam_type(self):
        self.assertEqual(protocol.start_mapping_request()["data"]["slam_type"], "indoor")

    def test_navigate_uses_fixed_mode(self):
        request = protocol.navigate_request(Pose2D(2.0, 0.0))
        self.assertEqual(request["data"]["mode"], 1)

    def test_navigate_encodes_yaw_as_quaternion(self):
        target = protocol.navigate_request(Pose2D(1.0, 2.0, math.pi / 2.0))["data"]["targetPose"]
        self.assertAlmostEqual(target["x"], 1.0)
        self.assertAlmostEqual(target["y"], 2.0)
        self.assertAlmostEqual(target["q_z"], math.sin(math.pi / 4.0), places=9)
        self.assertAlmostEqual(target["q_w"], math.cos(math.pi / 4.0), places=9)

    def test_init_pose_carries_address_and_quaternion(self):
        data = protocol.init_pose_request("/home/unitree/test1.pcd", Pose2D(0.0, 0.0))["data"]
        self.assertEqual(data["address"], "/home/unitree/test1.pcd")
        self.assertAlmostEqual(data["q_w"], 1.0, places=9)

    def test_pause_and_resume_take_no_parameters(self):
        self.assertEqual(protocol.pause_request(), {"data": {}})
        self.assertEqual(protocol.resume_request(), {"data": {}})

    def test_requests_are_json_serializable(self):
        # DDS越しに文字列で飛ぶので、そのまま json.dumps できないと使えない
        for request in (
            protocol.start_mapping_request(),
            protocol.end_mapping_request("/home/unitree/test1.pcd"),
            protocol.init_pose_request("/home/unitree/test1.pcd", Pose2D(0.0, 0.0)),
            protocol.navigate_request(Pose2D(1.0, 1.0, 0.3)),
            protocol.close_slam_request(),
        ):
            json.dumps(request)


class ResponseTest(unittest.TestCase):
    def test_parses_the_measured_507(self):
        response = protocol.parse_response(json.dumps(LOAD_PCD_FAILED))
        self.assertFalse(response.succeed)
        self.assertEqual(response.error_code, protocol.ERROR_LOAD_PCD_FAILED)
        self.assertTrue(response.is_load_pcd_failure)

    def test_parses_success(self):
        response = protocol.parse_response({"succeed": True, "errorCode": 0, "info": "", "data": {}})
        self.assertTrue(response.succeed)
        self.assertFalse(response.is_load_pcd_failure)

    def test_accepts_bytes(self):
        self.assertTrue(protocol.parse_response(json.dumps(LOAD_PCD_FAILED).encode()).is_load_pcd_failure)

    def test_rejects_broken_json_instead_of_guessing(self):
        with self.assertRaises(ValueError):
            protocol.parse_response("{ not json")

    def test_rejects_non_object_json(self):
        with self.assertRaises(ValueError):
            protocol.parse_response("[1, 2, 3]")


class CtrlInfoTest(unittest.TestCase):
    def test_reads_the_measured_idle_frame(self):
        info = protocol.parse_ctrl_info(IDLE_CTRL_INFO)
        self.assertEqual(info.state, "ready")
        self.assertEqual(info.ctrl_name, "not init")
        self.assertFalse(info.is_arrived)
        self.assertFalse(info.obstacle.blocked)
        self.assertEqual(info.progress.completion_percentage, 0.0)

    def test_not_init_means_map_is_not_loaded(self):
        self.assertFalse(protocol.parse_ctrl_info(IDLE_CTRL_INFO).initialized)

    def test_initialized_once_controller_name_changes(self):
        frame = json.loads(json.dumps(IDLE_CTRL_INFO))
        frame["info"] = "running"
        frame["data"]["stateMachine"]["ctrName"] = "pid"
        self.assertTrue(protocol.parse_ctrl_info(frame).initialized)

    def test_reads_rpy_pose(self):
        frame = json.loads(json.dumps(IDLE_CTRL_INFO))
        frame["data"]["currentPose"] = {"x": 1.5, "y": -2.5, "z": 0.0,
                                        "roll": 0.0, "pitch": 0.0, "yaw": 1.2}
        pose = protocol.parse_ctrl_info(frame).current_pose
        self.assertIsNotNone(pose)
        self.assertAlmostEqual(pose.x, 1.5)
        self.assertAlmostEqual(pose.yaw, 1.2)

    def test_tolerates_missing_optional_blocks(self):
        # 仕様書に載っていて実機に無い、あるいはその逆のフィールドがある
        info = protocol.parse_ctrl_info({"type": "ctrl_info", "data": {}})
        self.assertEqual(info.state, "")
        self.assertEqual(info.progress.used_time, 0.0)
        self.assertIsNone(info.current_pose)

    def test_ignores_unknown_fields(self):
        # total_distance は仕様書に無いが実機は流してくる
        self.assertEqual(protocol.parse_ctrl_info(IDLE_CTRL_INFO).error_code, 0)

    def test_reads_obstacle_block(self):
        frame = json.loads(json.dumps(IDLE_CTRL_INFO))
        frame["data"]["obsInfo"] = {"state": True, "time": 12.5}
        obstacle = protocol.parse_ctrl_info(frame).obstacle
        self.assertTrue(obstacle.blocked)
        self.assertAlmostEqual(obstacle.blocked_seconds, 12.5)


class SlamInfoTest(unittest.TestCase):
    def test_splits_type_and_data(self):
        kind, data = protocol.parse_slam_info(IDLE_CTRL_INFO)
        self.assertEqual(kind, protocol.TYPE_CTRL_INFO)
        self.assertIn("stateMachine", data)


class TaskResultTest(unittest.TestCase):
    def test_reads_arrival(self):
        result = protocol.parse_task_result(
            {"type": "task_result", "errorCode": 0, "info": "",
             "data": {"targetNodeName": 9999, "is_arrived": True}}
        )
        self.assertTrue(result.is_arrived)
        self.assertEqual(result.target_node_name, 9999)

    def test_defaults_to_not_arrived(self):
        self.assertFalse(protocol.parse_task_result({"type": "task_result"}).is_arrived)


class ParsePoseTest(unittest.TestCase):
    def test_reads_quaternion_pose(self):
        pose = protocol.parse_pose({"x": 1.0, "y": 2.0, "z": 0.0,
                                    "q_x": 0.0, "q_y": 0.0, "q_z": 0.0, "q_w": 1.0})
        self.assertAlmostEqual(pose.yaw, 0.0, places=9)

    def test_returns_none_for_missing_pose(self):
        self.assertIsNone(protocol.parse_pose(None))
        self.assertIsNone(protocol.parse_pose({}))
        self.assertIsNone(protocol.parse_pose({"yaw": 1.0}))


if __name__ == "__main__":
    unittest.main()
