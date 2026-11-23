import unittest

from limits import default_limits
from rest import REST_ID, REST_JOINTS, rest_pose


def robot(**kw):
    base = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 5.0}
    return {**base, **kw}


class RestPoseTests(unittest.TestCase):
    def test_uses_the_taught_h2_without_gripper(self):
        teach = {REST_ID: {"robot": robot(shoulder_lift=-50.0, elbow_flex=60.0)}}
        got = rest_pose(teach, default_limits())
        self.assertEqual(set(got), set(REST_JOINTS))
        self.assertEqual((got["shoulder_lift"], got["elbow_flex"]), (-50.0, 60.0))

    def test_none_when_h2_is_not_taught(self):
        self.assertIsNone(rest_pose({}, default_limits()))
        self.assertIsNone(rest_pose({"H3": {"robot": robot()}}, default_limits()))

    def test_clamped_to_limits_but_saved_values_untouched(self):
        limits = default_limits()
        limits["shoulder_lift"].update(min=10.0, max=90.0)     # 内部の値 -80〜80
        teach = {REST_ID: {"robot": robot(shoulder_lift=-100.0)}}
        self.assertEqual(rest_pose(teach, limits)["shoulder_lift"], -80.0)
        self.assertEqual(teach[REST_ID]["robot"]["shoulder_lift"], -100.0)


if __name__ == "__main__":
    unittest.main()
