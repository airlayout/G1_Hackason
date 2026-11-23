import unittest

from limits import default_limits
from replay import SimArm, replay, tracking

# 畳んだ姿勢から始め、手を上げる入力（lift）を 0 → 1 へ 4 秒かけて動かし続ける（実機ログと同じ形）
FOLDED = dict(shoulder_pan=0.0, shoulder_lift=-99.5, elbow_flex=94.0, wrist_flex=75.0, wrist_roll=0.0, gripper=20.0)
FOLLOW = ["shoulder_pan", "shoulder_lift", "elbow_flex", "gripper"]


def raising_hand(seconds=8.0, dt=0.08):
    samples, t = [], 0.0
    while t < seconds:
        t += dt
        lift = 0.0 if t < 1.0 else min(1.0, (t - 1.0) / 4.0)       # 最初の 1 秒は構えて止める（基準を取る）
        samples.append(dict(ts=t, follow=FOLLOW,
                            inp=dict(height=0.5, opening=0.5, pan=0.0, lift=lift, reach=0.0)))
    return samples


def run(step):
    limits = default_limits()
    limits["shoulder_lift"].update(step=step)
    start = dict(ts=0.0, pose=FOLDED, limits=limits, home=None)
    results = replay(start, raising_hand(), limits=limits, follow=FOLLOW)
    return results


class SimArmTests(unittest.TestCase):
    def test_dead_band_holds_small_commands_and_moves_large_ones(self):
        arm = SimArm(dict(shoulder_lift=0.0), dead=dict(shoulder_lift=3.0))
        arm.target["shoulder_lift"] = 2.0
        arm.advance(1.0)
        self.assertEqual(arm.pose["shoulder_lift"], 0.0)
        arm.target["shoulder_lift"] = 10.0
        arm.advance(1.0)
        self.assertGreater(arm.pose["shoulder_lift"], 5.0)

    def test_gravity_side_can_have_a_different_dead_band(self):
        arm = SimArm(dict(shoulder_lift=0.0), dead=dict(shoulder_lift=(5.0, 1.0)))
        arm.target["shoulder_lift"] = 3.0          # 上げる向きは 5 未満なので動かない
        arm.advance(1.0)
        self.assertEqual(arm.pose["shoulder_lift"], 0.0)
        arm.target["shoulder_lift"] = -3.0         # 下げる向きは 1 以上なので動く
        arm.advance(1.0)
        self.assertLess(arm.pose["shoulder_lift"], 0.0)


class ShoulderFollowTests(unittest.TestCase):
    """人が手を上げたとき、肩が実機の不感帯に詰まらず追従する（step 1.5% では詰まる）。"""

    def travelled(self, results):
        poses = [r["sim"]["shoulder_lift"] for r in results]
        return max(poses) - min(poses)

    def test_default_limits_let_the_shoulder_follow_the_raised_hand(self):
        results = run(step=default_limits()["shoulder_lift"]["step"])
        self.assertGreater(self.travelled(results), 40)
        self.assertLess(tracking(results, "shoulder_lift")["stall"], 0.1)

    def test_old_step_stalls_inside_the_dead_band(self):
        results = run(step=1.5)
        self.assertLess(self.travelled(results), 10)
        self.assertGreater(tracking(results, "shoulder_lift")["stall"], 0.3)


if __name__ == "__main__":
    unittest.main()
