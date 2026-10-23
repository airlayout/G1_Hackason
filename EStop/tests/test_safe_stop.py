import unittest

from estop.safe_stop import Observation, SafeStop, SafeStopConfig, State

STILL = Observation(2.0, 0.0, 0.02)
MOVING = Observation(3.0, 0.4, 0.5)
FALLING = Observation(50.0, 0.1, 2.0)
UNKNOWN = Observation(None, None, None)


class SafeStopTest(unittest.TestCase):
    def test_armed_passes_through(self):
        s = SafeStop()
        self.assertEqual(s.step(0.0, True, MOVING).locomotion, "run")
        self.assertTrue(s.permits_motion)

    def test_button_open_stops_motion_and_holds_arms_and_hands(self):
        s = SafeStop()
        cmd = s.step(0.0, False, MOVING)
        self.assertEqual((cmd.locomotion, cmd.arms, cmd.hands), ("stop_move", "hold", "hold"))
        self.assertFalse(s.permits_motion)

    def test_settles_to_safe_hold_keeping_torque(self):
        s = SafeStop()
        s.step(0.0, False, MOVING)
        s.step(0.2, False, STILL)
        cmd = s.step(0.8, False, STILL)
        self.assertEqual(s.state, State.SAFE_HOLD)
        self.assertEqual(cmd.locomotion, "stop_move")

    def test_motion_resets_settle_timer(self):
        s = SafeStop()
        s.step(0.0, False, MOVING)
        s.step(0.2, False, STILL)
        s.step(0.5, False, MOVING)
        s.step(0.8, False, STILL)
        self.assertEqual(s.state, State.STOPPING)

    def test_fall_is_unstable_and_never_auto_damps(self):
        s = SafeStop()
        s.step(0.0, False, MOVING)
        cmd = s.step(0.1, False, FALLING)
        self.assertEqual(s.state, State.UNSTABLE)
        self.assertTrue(s.alarm)
        self.assertEqual(cmd.locomotion, "stop_move")  # D-13: 自動で Damp しない
        s.step(100.0, False, FALLING)
        self.assertEqual(s.state, State.UNSTABLE)

    def test_timeout_is_unstable(self):
        s = SafeStop()
        s.step(0.0, False, MOVING)
        s.step(3.1, False, MOVING)
        self.assertEqual(s.state, State.UNSTABLE)

    def test_unknown_observation_never_counts_as_still(self):
        s = SafeStop()
        s.step(0.0, False, UNKNOWN)
        s.step(1.0, False, UNKNOWN)
        self.assertEqual(s.state, State.STOPPING)
        s.step(3.1, False, UNKNOWN)
        self.assertEqual(s.state, State.UNSTABLE)

    def test_unstable_recovers_to_safe_hold_when_still(self):
        s = SafeStop()
        s.step(0.0, False, MOVING)
        s.step(3.1, False, MOVING)
        s.step(3.2, False, STILL)
        s.step(3.8, False, STILL)
        self.assertEqual(s.state, State.SAFE_HOLD)

    def test_safe_hold_to_unstable_if_tilts_later(self):
        s = SafeStop()
        s.step(0.0, False, STILL)
        s.step(0.6, False, STILL)
        s.step(0.7, False, FALLING)
        self.assertEqual(s.state, State.UNSTABLE)

    def test_damp_only_by_operator(self):
        s = SafeStop()
        self.assertFalse(s.request_damp(0.0))  # ARMED では受け付けない
        s.step(0.1, False, FALLING)
        self.assertTrue(s.request_damp(0.2))
        self.assertEqual(s.step(0.3, False, FALLING).locomotion, "damp")

    def test_no_auto_resume_when_button_released(self):
        s = SafeStop()
        s.step(0.0, False, STILL)
        s.step(0.6, False, STILL)
        s.step(1.0, True, STILL)
        s.step(10.0, True, STILL)
        self.assertEqual(s.state, State.SAFE_HOLD)

    def test_reset_needs_ack_and_stable_button(self):
        s = SafeStop(SafeStopConfig(rearm_s=2.0))
        s.step(0.0, False, STILL)
        s.step(0.6, False, STILL)
        s.step(1.0, True, STILL)
        self.assertFalse(s.reset(1.5, operator_ack=True))   # 健全が短い
        self.assertFalse(s.reset(3.5, operator_ack=False))  # 人の確認なし
        self.assertTrue(s.reset(3.5, operator_ack=True))
        self.assertTrue(s.permits_motion)

    def test_button_chatter_restarts_rearm_wait(self):
        s = SafeStop(SafeStopConfig(rearm_s=2.0))
        s.step(0.0, False, STILL)
        s.step(1.0, True, STILL)
        s.step(2.5, False, STILL)
        s.step(2.6, True, STILL)
        self.assertFalse(s.reset(3.5, operator_ack=True))

    def test_events_record_reason(self):
        s = SafeStop()
        s.step(0.0, False, MOVING)
        events = s.pop_events()
        self.assertEqual([e[1] for e in events], [State.STOPPING])
        self.assertEqual(s.pop_events(), [])


if __name__ == "__main__":
    unittest.main()
