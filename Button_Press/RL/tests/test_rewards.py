"""報酬（rewards.py）と、行動・観測の計算（proprio.py）のテスト。"""

from __future__ import annotations

import importlib.util
import unittest

import numpy as np

HAS_TORCH = importlib.util.find_spec("torch") is not None


@unittest.skipUnless(HAS_TORCH, "torch が無い")
class TestRewards(unittest.TestCase):
    def _inputs(self, **kw):
        import torch

        from button_press_rl.rewards import RewardInputs

        n = 1
        z = torch.zeros(n)
        f = torch.zeros(n, dtype=torch.bool)
        base = dict(
            tip=torch.tensor([[0.20, -0.05, 0.10]]), tip_vel=torch.zeros(n, 3),
            face=torch.tensor([[0.30, 0.0, 0.20]]), axis=torch.tensor([[1.0, 0.0, 0.0]]), tip_radius=0.008,
            depth=z.clone(), press_depth=0.002, force=z.clone(), max_force_until_lit=z.clone(),
            unwanted_force=z.clone(), in_post=f.clone(), lit_event=f.clone(), wrong_event=f.clone(),
            released_event=f.clone(), force_abort=f.clone(), wrong_lit=f.clone(), action=torch.zeros(n, 7),
            last_action=torch.zeros(n, 7), dq=torch.zeros(n, 7), at_limit=z.clone(),
            press_ok=(3.0, 8.0), release_n=0.3,
        )
        base.update(kw)
        return RewardInputs(**base)

    def test_closer_is_better_and_costs_are_negative(self) -> None:
        """近づくほど報酬が大きい。出来事が無ければ、毎周期の報酬は 0 以下。"""
        import torch

        from button_press_rl.rewards import RewardWeights, compute_rewards

        w = RewardWeights()
        far, _ = compute_rewards(self._inputs(), w)
        front, _ = compute_rewards(self._inputs(tip=torch.tensor([[0.27, 0.0, 0.20]])), w)
        touch, _ = compute_rewards(self._inputs(tip=torch.tensor([[0.292, 0.0, 0.20]]),
                                                depth=torch.tensor([0.001]), force=torch.tensor([1.0])), w)
        self.assertLess(far.item(), front.item())
        self.assertLess(front.item(), touch.item())
        self.assertLessEqual(touch.item(), 0.0)

    def test_reach_not_saturated_far_away(self) -> None:
        """手の初期位置くらい（約 0.6 m）離れていても、5 cm 近づけば「近づく」項がはっきり良くなる。"""
        import torch

        from button_press_rl.rewards import RewardWeights, compute_rewards

        w = RewardWeights()
        face = torch.tensor([[0.32, 0.0, 0.20]])
        _, a = compute_rewards(self._inputs(face=face, tip=torch.tensor([[-0.07, -0.30, -0.22]])), w)
        _, b = compute_rewards(self._inputs(face=face, tip=torch.tensor([[-0.04, -0.27, -0.19]])), w)
        self.assertGreater(b["reach"].item() - a["reach"].item(), 0.05)

    def test_success_quality_by_force(self) -> None:
        """点灯したときまでの力の最大が適正な範囲なら、ボーナスが大きい。"""
        import torch

        from button_press_rl.rewards import RewardWeights, compute_rewards

        w = RewardWeights()
        ev = torch.tensor([True])
        _, ok = compute_rewards(self._inputs(lit_event=ev, max_force_until_lit=torch.tensor([5.0])), w)
        _, strong = compute_rewards(self._inputs(lit_event=ev, max_force_until_lit=torch.tensor([15.0])), w)
        self.assertAlmostEqual(ok["success"].item(), w.success + w.success_quality)
        self.assertAlmostEqual(strong["success"].item(), w.success)

    def test_force_over_and_hold(self) -> None:
        import torch

        from button_press_rl.rewards import RewardWeights, compute_rewards

        w = RewardWeights()
        _, t5 = compute_rewards(self._inputs(force=torch.tensor([5.0])), w)
        _, t16 = compute_rewards(self._inputs(force=torch.tensor([16.0])), w)
        self.assertEqual(t5["force_over"].item(), 0.0)
        self.assertAlmostEqual(t16["force_over"].item(), -w.force_over * 1.0)  # (16 − 8) / 8 = 1
        post = torch.tensor([True])
        _, hold = compute_rewards(self._inputs(in_post=post, force=torch.tensor([2.0])), w)
        _, rel = compute_rewards(self._inputs(in_post=post, force=torch.tensor([0.0])), w)
        self.assertLess(hold["hold_after"].item(), 0.0)
        self.assertEqual(rel["hold_after"].item(), 0.0)
        self.assertEqual(rel["reach"].item(), 0.0)  # 点灯のあとは近づく項を止める

    def test_wrong_button_never_pays(self) -> None:
        """違うボタンを押すと、押さずに居続けるより必ず損（打ち切らないので、点灯している間は毎周期の減点が続く）。"""
        import torch

        from button_press_rl.rewards import RewardWeights, compute_rewards

        w = RewardWeights()
        on = torch.tensor([True])
        stay, _ = compute_rewards(self._inputs(), w)
        lit, _ = compute_rewards(self._inputs(wrong_lit=on), w)
        event, _ = compute_rewards(self._inputs(wrong_lit=on, wrong_event=on), w)
        self.assertLess(lit.item(), stay.item())
        self.assertLess(event.item(), lit.item())

    def test_impact(self) -> None:
        """当たる直前に速いと減点。触れたあとは見ない。"""
        import torch

        from button_press_rl.rewards import RewardWeights, compute_rewards

        w = RewardWeights()
        tip = torch.tensor([[0.285, 0.0, 0.20]])  # 面まで 7 mm（球の表面）
        fast = torch.tensor([[0.5, 0.0, 0.0]])
        _, t = compute_rewards(self._inputs(tip=tip, tip_vel=fast), w)
        self.assertAlmostEqual(t["impact"].item(), -w.impact * 0.4 ** 2, places=5)
        _, t = compute_rewards(self._inputs(tip=tip, tip_vel=fast, force=torch.tensor([1.0])), w)
        self.assertEqual(t["impact"].item(), 0.0)


@unittest.skipUnless(HAS_TORCH, "torch が無い")
class TestProprioSameInNumpyAndTorch(unittest.TestCase):
    def test_same(self) -> None:
        """学習環境（torch）と評価用の Agent（numpy）で、行動と観測の計算が同じ。"""
        import torch

        from button_press_rl.proprio import PROPRIO_DIM, proprio, step_command

        rng = np.random.default_rng(0)
        q, dq, qc, q0, a = (rng.normal(size=(3, 7)) for _ in range(5))
        lo, hi = -np.ones(7), np.ones(7)
        n_cmd = step_command(qc, a * 3, lo, hi, np.clip)
        t_cmd = step_command(torch.tensor(qc), torch.tensor(a * 3), torch.tensor(lo), torch.tensor(hi), torch.clamp)
        np.testing.assert_allclose(n_cmd, t_cmd.numpy())
        n_obs = proprio(q, dq, qc, q0, a, np.concatenate)
        t_obs = proprio(*(torch.tensor(v) for v in (q, dq, qc, q0, a)), torch.cat)
        self.assertEqual(n_obs.shape, (3, PROPRIO_DIM))
        np.testing.assert_allclose(n_obs, t_obs.numpy())


if __name__ == "__main__":
    unittest.main()
