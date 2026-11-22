import unittest

from limits import default_limits
from teach_map import ARM_JOINTS, HUMAN, leave_one_out, parse_angles, solve


def pose(**kw):
    base = {"shoulder_pan": 0.0, "shoulder_lift": 0.0, "elbow_flex": 0.0, "wrist_flex": 0.0, "wrist_roll": 0.0, "gripper": 5.0}
    return {**base, **kw}


def angles(key):
    return {f: HUMAN[key][f] for f in ("pan", "lift", "elbow")}


TEACH = {
    "H3": {"robot": pose(shoulder_lift=-30.0, elbow_flex=20.0)},
    "H4": {"robot": pose(shoulder_lift=-3.0, elbow_flex=20.0, wrist_flex=-90.0)},
    "H9": {"robot": pose(shoulder_lift=-23.0, elbow_flex=80.0)},
}


class SolveTests(unittest.TestCase):
    def test_returns_the_taught_pose_at_a_taught_angle(self):
        for key, entry in TEACH.items():
            got = solve(TEACH, angles(key))
            self.assertEqual(got, {j: entry["robot"][j] for j in ARM_JOINTS})

    def test_gripper_is_not_part_of_the_result(self):
        self.assertNotIn("gripper", solve(TEACH, angles("H3")))

    def test_blend_stays_inside_the_taught_values(self):
        for query in ({"pan": 10, "lift": 70, "elbow": 130}, {"pan": -20, "lift": 30, "elbow": 60}, {"pan": 0, "lift": 150, "elbow": 20}):
            got = solve(TEACH, query)
            for j in ARM_JOINTS:
                values = [e["robot"][j] for e in TEACH.values()]
                self.assertTrue(min(values) - 1e-9 <= got[j] <= max(values) + 1e-9, (j, got[j]))

    def test_nearer_taught_pose_counts_more(self):
        near_h3 = solve(TEACH, {"pan": 0, "lift": 52, "elbow": 92})
        near_h4 = solve(TEACH, {"pan": 0, "lift": 88, "elbow": 168})
        self.assertLess(abs(near_h3["shoulder_lift"] + 30), abs(near_h4["shoulder_lift"] + 30))
        self.assertLess(abs(near_h4["wrist_flex"] + 90), abs(near_h3["wrist_flex"] + 90))

    def test_unmeasurable_pan_is_ignored(self):
        with_pan = solve(TEACH, {"pan": 0, "lift": 50, "elbow": 90})
        without = solve(TEACH, {"pan": None, "lift": 50, "elbow": 90})
        self.assertEqual(with_pan, without)         # 教示点の pan がすべて 0 なので、pan を使っても使わなくても同じ
        self.assertEqual(solve(TEACH, {"lift": 50, "elbow": 90}), without)

    def test_taught_point_without_pan_is_matched_on_lift_and_elbow(self):
        teach = {"H1": {"robot": pose(shoulder_lift=-34.0)}, "H4": TEACH["H4"]}
        self.assertEqual(solve(teach, {"pan": 25, "lift": 10, "elbow": 170})["shoulder_lift"], -34.0)

    def test_limits_clamp_the_result_but_not_the_saved_values(self):
        limits = default_limits()
        limits["wrist_flex"].update(min=50.0, max=100.0)       # 内部の値 0〜100
        got = solve(TEACH, angles("H4"), limits)
        self.assertEqual(got["wrist_flex"], 0.0)
        self.assertEqual(TEACH["H4"]["robot"]["wrist_flex"], -90.0)

    def test_bad_input_and_empty_teach_raise(self):
        for bad in (None, {}, {"lift": 1}, {"lift": float("nan"), "elbow": 1}, {"lift": "a", "elbow": 1}, {"pan": "x", "lift": 1, "elbow": 1}):
            with self.assertRaises(ValueError):
                parse_angles(bad)
        with self.assertRaises(ValueError):
            solve({}, angles("H3"))

    def test_pan_follows_a_line_between_taught_pans(self):
        teach = {
            "H3": {"robot": pose(shoulder_pan=-5.0, shoulder_lift=-30.0, elbow_flex=20.0)},
            "H7": {"robot": pose(shoulder_pan=-45.0, shoulder_lift=-3.0, elbow_flex=17.0)},     # 人の pan 40
        }
        for query_pan, expected in ((0, -5.0), (20, -25.0), (40, -45.0), (80, -45.0)):
            got = solve(teach, {"pan": query_pan, "lift": 150, "elbow": 150})
            self.assertAlmostEqual(got["shoulder_pan"], expected, delta=0.5, msg=query_pan)

    def test_pan_stays_put_when_human_pan_is_zero_even_near_a_far_pan_pose(self):
        teach = {
            "H4": {"robot": pose(shoulder_pan=-5.0, shoulder_lift=-3.0, elbow_flex=20.0)},       # 人の pan 0
            "H6": {"robot": pose(shoulder_pan=-90.0, shoulder_lift=-2.5, elbow_flex=17.0)},      # 人の pan 70、lift・elbow は H4 と同じ
        }
        self.assertEqual(solve(teach, angles("H4"))["shoulder_pan"], -5.0)
        self.assertEqual(solve(teach, angles("H6"))["shoulder_pan"], -90.0)
        self.assertAlmostEqual(solve(teach, {"pan": 0, "lift": 88, "elbow": 168})["shoulder_pan"], -5.0, delta=1.0)

    def test_leave_one_out_reports_each_point(self):
        errors = leave_one_out(TEACH)
        self.assertEqual(set(errors), set(TEACH))
        self.assertEqual(leave_one_out({"H3": TEACH["H3"]}), {})


if __name__ == "__main__":
    unittest.main()
