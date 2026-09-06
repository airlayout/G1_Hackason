"""`nav/protocol.py`: `slam_operate` の JSON 層。

固定しているのは 2 つ:

1. **実機の実測 JSON が読めること。** 実機は仕様書のスーパーセットを流す
   （`ctrl_info` に仕様書に無い `currentPose` と `total_distance` が入る）
2. **壊れた JSON を黙って通さないこと。** 握りつぶすと、通信が死んでいるのに
   「障害物なし・未到達」として巡回が進み続ける
"""

import json
import math
import unittest

from nav import protocol
from nav.protocol import Pose2D


class QuaternionTest(unittest.TestCase):
    def test_yaw_survives_a_round_trip(self):
        for yaw in (0.0, 0.5, -1.2, math.pi / 2, 3.0, -3.0):
            with self.subTest(yaw=yaw):
                q_x, q_y, q_z, q_w = protocol.yaw_to_quaternion(yaw)
                self.assertAlmostEqual(protocol.quaternion_to_yaw(q_x, q_y, q_z, q_w), yaw, places=9)

    def test_yaw_only_rotates_about_z(self):
        q_x, q_y, _, _ = protocol.yaw_to_quaternion(1.0)
        self.assertAlmostEqual(q_x, 0.0)
        self.assertAlmostEqual(q_y, 0.0)

    def test_unnormalized_quaternion_is_accepted(self):
        """実機の JSON をそのまま食わせるので、正規化されていなくても読めること。"""

        self.assertAlmostEqual(protocol.quaternion_to_yaw(0.0, 0.0, 0.0, 2.0), 0.0)

    def test_pi_and_minus_pi_are_the_same_heading(self):
        forward = protocol.yaw_to_quaternion(math.pi)
        backward = protocol.yaw_to_quaternion(-math.pi)
        self.assertAlmostEqual(
            abs(protocol.quaternion_to_yaw(*forward)),
            abs(protocol.quaternion_to_yaw(*backward)),
            places=9,
        )


class RequestTest(unittest.TestCase):
    def test_start_mapping_declares_indoor(self):
        """`slam_type` は公式に「固定值 indoor」と書かれている。"""

        self.assertEqual(
            protocol.start_mapping_request(), {"data": {"slam_type": protocol.SLAM_TYPE_INDOOR}}
        )

    def test_end_mapping_carries_the_address(self):
        self.assertEqual(
            protocol.end_mapping_request("/home/unitree/test1.pcd"),
            {"data": {"address": "/home/unitree/test1.pcd"}},
        )

    def test_init_pose_sends_a_quaternion_not_a_yaw(self):
        request = protocol.init_pose_request("/map.pcd", Pose2D(1.0, 2.0, 0.5))["data"]
        self.assertNotIn("yaw", request)
        self.assertAlmostEqual(
            protocol.quaternion_to_yaw(
                request["q_x"], request["q_y"], request["q_z"], request["q_w"]
            ),
            0.5,
        )
        self.assertEqual((request["x"], request["y"], request["address"]), (1.0, 2.0, "/map.pcd"))

    def test_navigate_mode_is_always_one(self):
        """G1 に绕障モードは無い。`mode` を可変にすると避けてくれると誤解する。"""

        self.assertEqual(protocol.navigate_request(Pose2D(0, 0))["data"]["mode"], 1)

    def test_navigate_puts_the_pose_under_target_pose(self):
        data = protocol.navigate_request(Pose2D(3.0, -4.0, 1.0))["data"]
        self.assertEqual(data["targetPose"]["x"], 3.0)
        self.assertEqual(data["targetPose"]["y"], -4.0)

    def test_navigate_does_not_reject_a_far_target(self):
        """10m 制限は現在位置との距離。リクエスト単体では判定できないので弾かない。"""

        self.assertTrue(protocol.navigate_request(Pose2D(999.0, 999.0)))

    def test_pause_resume_and_close_send_empty_data(self):
        for request in (
            protocol.pause_request(),
            protocol.resume_request(),
            protocol.close_slam_request(),
        ):
            self.assertEqual(request, {"data": {}})


class ResponseTest(unittest.TestCase):
    def test_success_is_read(self):
        response = protocol.parse_response('{"succeed":true,"errorCode":0,"info":"","data":{}}')
        self.assertTrue(response.succeed)
        self.assertEqual(response.error_code, 0)

    def test_507_is_recognised_as_a_load_failure(self):
        response = protocol.parse_response(
            '{"succeed":false,"errorCode":507,"info":"Load pcd failed."}'
        )
        self.assertTrue(response.is_load_pcd_failure)

    def test_missing_fields_do_not_look_like_success(self):
        response = protocol.parse_response("{}")
        self.assertFalse(response.succeed)
        self.assertEqual(response.error_code, -1)

    def test_bytes_are_accepted(self):
        self.assertTrue(protocol.parse_response(b'{"succeed":true}').succeed)

    def test_broken_json_raises(self):
        with self.assertRaises(ValueError):
            protocol.parse_response("{not json")

    def test_a_json_array_is_not_a_response(self):
        with self.assertRaises(ValueError):
            protocol.parse_response("[1,2,3]")


# 実機から実際に流れてきた 1 フレーム（`Navigation/README.md`「実測で確定した挙動」）。
# 仕様書に無い currentPose と total_distance が入っている点が肝。
REAL_CTRL_INFO = {
    "type": "ctrl_info",
    "errorCode": 0,
    "info": "not init",
    "data": {
        "stateMachine": {
            "state": "ready", "ctrName": "not init", "isOpenPlan": False, "isBack": False,
            "isRotate": False, "isClimbStairs": False, "isPause": False,
            "vx": 0.004, "vy": 0.005, "vyaw": 0.065,
        },
        "currentPose": {"x": 0.1, "y": -0.2, "z": 0.0, "roll": 0.0, "pitch": 0.0, "yaw": 0.3},
        "is_arrived": False,
        "targetNodeName": 0,
        "obsInfo": {"state": False, "time": 0.0},
        "progress": {"used_time": 1.5, "last_time": 0.0, "completion_percentage": 0.0},
        "total_distance": -1.0,
    },
}


class CtrlInfoTest(unittest.TestCase):
    def test_a_real_frame_is_read(self):
        info = protocol.parse_ctrl_info(json.dumps(REAL_CTRL_INFO))
        self.assertEqual(info.state, "ready")
        self.assertEqual(info.ctrl_name, "not init")
        self.assertFalse(info.is_arrived)
        self.assertEqual(info.current_pose, Pose2D(0.1, -0.2, 0.3))

    def test_not_init_means_no_map_is_loaded(self):
        self.assertFalse(protocol.parse_ctrl_info(json.dumps(REAL_CTRL_INFO)).initialized)

    def test_a_running_frame_counts_as_initialized(self):
        frame = json.loads(json.dumps(REAL_CTRL_INFO))
        frame["info"] = "running"
        frame["data"]["stateMachine"]["ctrName"] = "pid"
        self.assertTrue(protocol.parse_ctrl_info(frame).initialized)

    def test_either_field_saying_not_init_is_enough(self):
        """info と ctrName の片方だけが変わる可能性を潰しておく。"""

        frame = json.loads(json.dumps(REAL_CTRL_INFO))
        frame["info"] = "running"  # ctrName は "not init" のまま
        self.assertFalse(protocol.parse_ctrl_info(frame).initialized)

    def test_obstacle_state_and_elapsed_seconds_are_read(self):
        frame = json.loads(json.dumps(REAL_CTRL_INFO))
        frame["data"]["obsInfo"] = {"state": True, "time": 4.25}
        info = protocol.parse_ctrl_info(frame)
        self.assertTrue(info.obstacle.blocked)
        self.assertAlmostEqual(info.obstacle.blocked_seconds, 4.25)

    def test_missing_sections_fall_back_to_defaults(self):
        """仕様書にあるフィールドが欠けていても 1 フレームで巡回を落とさない。"""

        info = protocol.parse_ctrl_info('{"type":"ctrl_info","data":{}}')
        self.assertFalse(info.obstacle.blocked)
        self.assertIsNone(info.current_pose)
        self.assertEqual(info.progress.used_time, 0.0)

    def test_unknown_fields_are_ignored(self):
        frame = json.loads(json.dumps(REAL_CTRL_INFO))
        frame["data"]["something_new_from_a_firmware_update"] = {"a": 1}
        self.assertTrue(protocol.parse_ctrl_info(frame).current_pose is not None)

    def test_slam_info_is_split_into_type_and_data(self):
        kind, data = protocol.parse_slam_info(json.dumps(REAL_CTRL_INFO))
        self.assertEqual(kind, "ctrl_info")
        self.assertIn("stateMachine", data)


class PoseParsingTest(unittest.TestCase):
    def test_quaternion_form_is_accepted(self):
        pose = protocol.parse_pose({"x": 1.0, "y": 2.0, "q_x": 0, "q_y": 0, "q_z": 0, "q_w": 1})
        self.assertEqual(pose, Pose2D(1.0, 2.0, 0.0))

    def test_roll_pitch_yaw_form_is_accepted(self):
        pose = protocol.parse_pose({"x": 1.0, "y": 2.0, "roll": 0.1, "pitch": 0.2, "yaw": 0.3})
        self.assertEqual(pose, Pose2D(1.0, 2.0, 0.3))

    def test_something_without_x_and_y_is_not_a_pose(self):
        self.assertIsNone(protocol.parse_pose({"q_w": 1.0}))
        self.assertIsNone(protocol.parse_pose(None))
        self.assertIsNone(protocol.parse_pose({}))


class TaskResultTest(unittest.TestCase):
    def test_arrival_is_read(self):
        result = protocol.parse_task_result(
            '{"type":"task_result","errorCode":0,"data":{"is_arrived":true,"targetNodeName":3}}'
        )
        self.assertTrue(result.is_arrived)
        self.assertEqual(result.target_node_name, 3)

    def test_a_frame_without_is_arrived_is_not_an_arrival(self):
        self.assertFalse(protocol.parse_task_result('{"data":{}}').is_arrived)


if __name__ == "__main__":
    unittest.main()
