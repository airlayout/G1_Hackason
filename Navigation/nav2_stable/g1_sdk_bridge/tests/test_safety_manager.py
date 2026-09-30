import unittest

import _pathfix  # noqa: F401
from safety_manager import NavState, SafetyLimits, SafetyManager, accel_limit, apply_deadband, clamp


class FakeClock:
    """モノトニック時計のテスト用スタブ。明示的にしか進まない。"""

    def __init__(self, t: float = 0.0):
        self.t = t

    def __call__(self) -> float:
        return self.t

    def advance(self, dt: float) -> None:
        self.t += dt


class TestPureFilters(unittest.TestCase):
    def test_clamp_limits_each_axis(self):
        limits = SafetyLimits(max_vx=0.2, max_vy=0.1, max_wz=0.3)
        self.assertEqual(clamp((1.0, 1.0, 1.0), limits), (0.2, 0.1, 0.3))
        self.assertEqual(clamp((-1.0, -1.0, -1.0), limits), (-0.2, -0.1, -0.3))
        self.assertEqual(clamp((0.05, 0.0, 0.1), limits), (0.05, 0.0, 0.1))

    def test_deadband_zeroes_small_values(self):
        limits = SafetyLimits(min_vx=0.03, min_wz=0.03)
        vx, vy, wz = apply_deadband((0.02, 0.05, 0.01), limits)
        self.assertEqual(vx, 0.0)
        self.assertEqual(wz, 0.0)
        self.assertEqual(vy, 0.05)  # vyはD-14の対象外(MVPではmax_vy=0で別途ゼロになる)

    def test_deadband_passes_through_values_above_threshold(self):
        limits = SafetyLimits(min_vx=0.03, min_wz=0.03)
        self.assertEqual(apply_deadband((0.05, 0.0, 0.10), limits), (0.05, 0.0, 0.10))

    def test_accel_limit_caps_rate_of_change(self):
        limits = SafetyLimits(max_ax=0.2, max_ay=0.15, max_awz=0.4)
        # 0 -> 1.0 へ一気に上げようとしても、dt=0.1sなら max_ax*dt=0.02までしか動けない
        out = accel_limit((0.0, 0.0, 0.0), (1.0, 1.0, 1.0), dt=0.1, limits=limits)
        self.assertAlmostEqual(out[0], 0.02, places=6)
        self.assertAlmostEqual(out[1], 0.015, places=6)
        self.assertAlmostEqual(out[2], 0.04, places=6)

    def test_accel_limit_does_not_overshoot_small_targets(self):
        limits = SafetyLimits(max_ax=0.2)
        out = accel_limit((0.0, 0.0, 0.0), (0.01, 0.0, 0.0), dt=0.1, limits=limits)
        self.assertAlmostEqual(out[0], 0.01, places=6)  # 上限より小さい変化はそのまま通る


class TestSafetyManagerStateMachine(unittest.TestCase):
    def setUp(self):
        self.clock = FakeClock()
        self.sent: list[tuple[float, float, float]] = []
        self.limits = SafetyLimits(cmd_timeout_s=0.30)
        self.mgr = SafetyManager(self.limits, ipc_send=lambda vx, vy, wz: self.sent.append((vx, vy, wz)), now_fn=self.clock)

    def _goto_navigating(self):
        self.mgr.on_bridge_connected()
        self.mgr.mark_ready()
        ok = self.mgr.enable_navigation(True)
        self.assertTrue(ok)
        self.assertEqual(self.mgr.state, NavState.NAVIGATING)

    def test_initial_state_is_disconnected(self):
        self.assertEqual(self.mgr.state, NavState.DISCONNECTED)

    def test_full_happy_path_transitions(self):
        self.mgr.on_bridge_connected()
        self.assertEqual(self.mgr.state, NavState.STANDBY)
        self.mgr.mark_ready()
        self.assertEqual(self.mgr.state, NavState.READY)
        self.assertTrue(self.mgr.enable_navigation(True))
        self.assertEqual(self.mgr.state, NavState.NAVIGATING)
        self.assertTrue(self.mgr.enable_navigation(False))
        self.assertEqual(self.mgr.state, NavState.READY)

    def test_enable_navigation_fails_outside_ready(self):
        # DISCONNECTED状態でいきなりNAVIGATINGにはできない(仕様書7章の遷移制約)
        self.assertFalse(self.mgr.enable_navigation(True))
        self.assertEqual(self.mgr.state, NavState.DISCONNECTED)

    def test_nonzero_velocity_only_sent_while_navigating(self):
        # READY状態でNav2からTwistが来ても、非ゼロ速度は送らない(仕様書8章の状態別許可表)
        self.mgr.on_bridge_connected()
        self.mgr.mark_ready()
        out = self.mgr.on_nav_twist(0.5, 0.0, 0.0)
        self.assertEqual(out, (0.0, 0.0, 0.0))
        self.assertEqual(self.sent[-1], (0.0, 0.0, 0.0))

    def test_nav_twist_is_clamped_and_forwarded_while_navigating(self):
        self._goto_navigating()
        limits = SafetyLimits()  # デフォルトのmax_vx=0.20, max_ax=0.20
        self.mgr.limits = limits
        # enable_navigation()からdt=1.0s経過させてから初回指令を送る。
        # max_ax*dt=0.2 == max_vxなので、accel_limitで頭打ちにならず、clampの上限にちょうど到達する。
        self.clock.advance(1.0)
        out = self.mgr.on_nav_twist(1.0, 0.0, 0.0)  # 上限超えの指令
        self.assertEqual(out, (limits.max_vx, 0.0, 0.0))
        self.assertEqual(self.sent[-1], out)

    def test_nav_twist_ramps_up_gradually_right_after_enable(self):
        # enable直後(dt≈0)に大きな指令が来ても、accel_limitでいきなり全開にはならないことを確認する。
        # これがバグ修正前は「常にゼロを返す」という別の理由でテストを素通りしていた箇所。
        self._goto_navigating()
        self.clock.advance(0.05)  # 20Hz相当の1周期分だけ進める
        out = self.mgr.on_nav_twist(1.0, 0.0, 0.0)
        expected_vx = self.limits.max_ax * 0.05  # 0.2 * 0.05 = 0.01
        self.assertAlmostEqual(out[0], expected_vx, places=6)
        self.assertLess(out[0], self.limits.max_vx)  # まだ上限には到達していない

        self.clock.advance(0.05)
        out2 = self.mgr.on_nav_twist(1.0, 0.0, 0.0)
        self.assertGreater(out2[0], out[0])  # 徐々に増えていく

    def test_e_stop_overrides_navigating_and_sends_zero(self):
        self._goto_navigating()
        self.mgr.e_stop()
        self.assertEqual(self.mgr.state, NavState.E_STOP)
        self.assertEqual(self.sent[-1], (0.0, 0.0, 0.0))

    def test_fault_does_not_override_e_stop(self):
        self._goto_navigating()
        self.mgr.e_stop()
        self.mgr.on_tf_stale()  # E_STOP中にFAULT条件が来ても上書きしない
        self.assertEqual(self.mgr.state, NavState.E_STOP)

    def test_clear_e_stop_requires_e_stop_state(self):
        self.assertFalse(self.mgr.clear_e_stop())  # DISCONNECTEDからは解除できない
        self._goto_navigating()
        self.mgr.e_stop()
        self.assertTrue(self.mgr.clear_e_stop())
        self.assertEqual(self.mgr.state, NavState.STANDBY)

    def test_clear_fault_requires_fault_state(self):
        self.assertFalse(self.mgr.clear_fault())
        self._goto_navigating()
        self.mgr.on_tf_stale()
        self.assertEqual(self.mgr.state, NavState.FAULT)
        self.assertTrue(self.mgr.clear_fault())
        self.assertEqual(self.mgr.state, NavState.STANDBY)

    def test_tick_watchdog_faults_on_cmd_timeout(self):
        # D-10: ROS側watchdog。指令が来なくなってからcmd_timeoutを超えたら明示的にゼロを送りFAULTへ
        self._goto_navigating()
        self.mgr.on_nav_twist(0.1, 0.0, 0.0)
        self.clock.advance(self.limits.cmd_timeout_s + 0.01)
        self.mgr.tick()
        self.assertEqual(self.mgr.state, NavState.FAULT)
        self.assertEqual(self.sent[-1], (0.0, 0.0, 0.0))

    def test_tick_does_not_fault_before_first_twist_even_after_timeout_elapsed(self):
        # 2026-09-09にNav2統合dry-runで発見: Nav2はGoal計画に数百ms〜数秒かかることがあり、
        # enable直後にcmd_timeoutのカウントを始めると最初の指令が届く前にFAULTへ誤って遷移する。
        # 最初の指令を受け取るまではtick()のタイムアウト判定を待機させる。
        self._goto_navigating()
        self.clock.advance(self.limits.cmd_timeout_s * 10)  # cmd_timeoutを大幅に超えて経過させる
        self.mgr.tick()
        self.assertEqual(self.mgr.state, NavState.NAVIGATING)  # まだFAULTにならない

        # 最初の指令が来た後は、通常どおりcmd_timeoutが効く
        self.mgr.on_nav_twist(0.1, 0.0, 0.0)
        self.clock.advance(self.limits.cmd_timeout_s + 0.01)
        self.mgr.tick()
        self.assertEqual(self.mgr.state, NavState.FAULT)

    def test_tick_does_not_fault_within_timeout(self):
        self._goto_navigating()
        self.mgr.on_nav_twist(0.1, 0.0, 0.0)
        self.clock.advance(self.limits.cmd_timeout_s - 0.05)
        self.mgr.tick()
        self.assertEqual(self.mgr.state, NavState.NAVIGATING)

    def test_bridge_disconnect_forces_disconnected_and_zero(self):
        self._goto_navigating()
        self.mgr.on_bridge_disconnected()
        self.assertEqual(self.mgr.state, NavState.DISCONNECTED)
        self.assertEqual(self.sent[-1], (0.0, 0.0, 0.0))


if __name__ == "__main__":
    unittest.main()
