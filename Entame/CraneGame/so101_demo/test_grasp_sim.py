import unittest

try:
    import grasp_sim
    import sim_replay
    HAVE = True
except Exception:          # mujoco / rerun が無い環境では飛ばす
    HAVE = False


@unittest.skipUnless(HAVE, "mujoco / rerun が無い")
class GraspSimTests(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        cls.cal = sim_replay.load_calibration()

    def judged(self, x):
        model, steps, snack = grasp_sim.run_script(x, self.cal)
        return grasp_sim.judge(steps, snack), steps, snack

    def test_pinches_and_lifts_inside_the_working_range(self):
        for x in (0.25, 0.28, 0.31):
            result, _, _ = self.judged(x)
            self.assertTrue(result["ok"], (x, result))
            self.assertGreaterEqual(result["lift_height"], grasp_sim.LIFT_OK)

    def test_snack_stays_on_desk_before_the_fingers_close(self):
        _, steps, snack = self.judged(0.27)
        before = [s for s in steps if s["phase"] < 5]                   # 5 = 指を閉じる
        for s in before:
            self.assertLess(abs(s["snack"][0] - snack[0]), 0.01)         # 降ろすとき、爪で押しのけない
            self.assertLess(abs(s["snack"][1] - snack[1]), 0.01)

    def test_misses_far_outside_the_range(self):
        result, _, _ = self.judged(0.40)
        self.assertFalse(result["ok"], result)

    def test_close_open_events_follow_the_opening(self):
        steps = [dict(opening=o) for o in (1.0, 0.9, 0.1, 0.1, 0.8, 1.0)]
        self.assertEqual(grasp_sim.close_open_events(steps), [("閉じる", 2), ("離す", 4)])

    def test_pick_close_ignores_the_initial_closed_state(self):
        # 先頭が閉じ（録画の初期状態）→ 開く → 閉じる。選ぶのは先頭ではなく、開いてから閉じた瞬間
        frames = [dict(opening=o, z=z) for o, z in ((0.0, 0.0), (1.0, 0.1), (1.0, 0.1), (0.1, 0.1))]
        tip = lambda f: (0.25, 0.0, grasp_sim.reach_map.Z_DESK + f["z"])
        self.assertEqual(grasp_sim.pick_close(frames, tip), (3, False))   # 高さ 10 cm は条件外だが、候補はここだけ

    def test_pick_close_prefers_a_low_close_and_none_without_close(self):
        frames = [dict(opening=o, z=z) for o, z in ((1.0, 0.2), (0.1, 0.2), (1.0, 0.0), (0.1, 0.01))]
        tip = lambda f: (0.25, 0.0, grasp_sim.reach_map.Z_DESK + f["z"])
        self.assertEqual(grasp_sim.pick_close(frames, tip), (3, True))
        self.assertIsNone(grasp_sim.pick_close([dict(opening=1.0, z=0.0)] * 3, tip))


if __name__ == "__main__":
    unittest.main()
