"""SwitchBot 電源操作のテスト。Bot も G1 も BLE も要らない（仮想時計つきの FakeBot と標準ライブラリのみ）。"""
import asyncio
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from switchbot_power import core as sp  # noqa: E402
from switchbot_power.core import BotPower, FakeBot, PowerError  # noqa: E402


def names(bot: FakeBot) -> list:
    return [n for _, n in bot.events if n != "get_info"]


def make():
    bot = FakeBot()
    return bot, BotPower(bot, sleep=bot.sleep, log=lambda m: None)


class PushOnceTest(unittest.IsolatedAsyncioTestCase):
    async def test_press_only(self):
        bot, power = make()
        await power.push_once()
        self.assertEqual(names(bot), ["press"])

    async def test_switch_mode_is_refused_without_pressing(self):
        bot, power = make()
        bot.info["switchMode"] = True
        with self.assertRaisesRegex(PowerError, "Switch モード"):
            await power.push_once()
        self.assertEqual(names(bot), [])

    async def test_nonzero_hold_setting_is_refused(self):
        bot, power = make()
        bot.info["holdSeconds"] = 3
        with self.assertRaisesRegex(PowerError, "長押し設定"):
            await power.push_once()
        self.assertEqual(names(bot), [])

    async def test_rejected_press_is_error(self):
        bot, power = make()
        bot.reject.add("press")
        with self.assertRaisesRegex(PowerError, "press"):
            await power.push_once()


class PushHoldHostTest(unittest.IsolatedAsyncioTestCase):
    async def test_down_wait_up_with_exact_duration(self):
        bot, power = make()
        await power.push_hold(2.5)
        self.assertEqual(names(bot), ["hand_down", "hand_up"])
        t = {n: t for t, n in bot.events}
        self.assertAlmostEqual(t["hand_up"] - t["hand_down"], 2.5)
        self.assertFalse(bot.arm_down)

    async def test_out_of_range_seconds_refused_before_touching_bot(self):
        for bad in (0, 0.05, sp.MAX_HOLD_S + 1, -1, True, "3"):
            bot, power = make()
            with self.assertRaises(PowerError, msg=repr(bad)):
                await power.push_hold(bad)
            self.assertEqual(bot.events, [], repr(bad))

    async def test_release_is_retried_after_comm_failure(self):
        bot, power = make()
        bot.fail["hand_up"] = 2
        await power.push_hold(1)
        self.assertEqual(names(bot).count("hand_up"), 3)
        self.assertFalse(bot.arm_down)

    async def test_release_waits_while_bot_is_busy(self):
        """実機: hand_down 直後の数秒は拒否される。待ちながら再送して成功する。"""
        bot, power = make()
        bot.reject.add("hand_up")
        orig = bot.hand_up

        async def busy_until_3s():
            bot.reject.discard("hand_up") if bot.now >= 3.0 else None
            return await orig()

        bot.hand_up = busy_until_3s
        await power.push_hold(1)
        self.assertFalse(bot.arm_down)
        self.assertGreater(names(bot).count("hand_up"), 3)

    async def test_release_failure_is_reported_loudly(self):
        bot, power = make()
        bot.fail["hand_up"] = 99
        with self.assertRaisesRegex(PowerError, "押下位置のまま"):
            await power.push_hold(1)
        self.assertEqual(names(bot).count("hand_up"), sp.RELEASE_RETRIES)

    async def test_hand_down_waits_while_bot_is_busy_then_counts_hold_from_acceptance(self):
        """実機: press の後 2.9 秒は hand_down が拒否される。受理された時点から保持時間を数える。"""
        bot, power = make()
        orig = bot.hand_down

        async def busy_until_2_9s():
            if bot.now < 2.9:
                bot._record("hand_down")
                bot.events[-1] = (bot.events[-1][0], "hand_down_busy")
                return False
            return await orig()

        bot.hand_down = busy_until_2_9s
        await power.push_hold(3.0)
        t = {n: t for t, n in bot.events}
        self.assertGreaterEqual(t["hand_down"], 2.9)
        self.assertAlmostEqual(t["hand_up"] - t["hand_down"], 3.0)

    async def test_hand_down_failure_does_not_send_release_noise(self):
        bot, power = make()
        bot.reject.add("hand_down")
        with self.assertRaisesRegex(PowerError, "hand_down"):
            await power.push_hold(1)
        self.assertNotIn("hand_up", names(bot))

    async def test_cancel_during_hold_still_releases(self):
        bot = FakeBot()
        gate = asyncio.Event()

        async def blocking_sleep(_s):
            await gate.wait()  # キャンセルされるまで進まない

        power = BotPower(bot, sleep=blocking_sleep, log=lambda m: None)
        task = asyncio.create_task(power.push_hold(5))
        for _ in range(5):
            await asyncio.sleep(0)
        self.assertTrue(bot.arm_down)
        task.cancel()
        with self.assertRaises(asyncio.CancelledError):
            await task
        self.assertFalse(bot.arm_down)
        self.assertEqual(names(bot), ["hand_down", "hand_up"])

    async def test_cancel_during_release_still_completes_release(self):
        bot = FakeBot()
        calls = {"n": 0}
        orig = bot.hand_up

        async def cancel_once():
            calls["n"] += 1
            if calls["n"] == 1:
                raise asyncio.CancelledError()
            return await orig()

        bot.hand_up = cancel_once
        power = BotPower(bot, sleep=bot.sleep, log=lambda m: None)
        with self.assertRaises(asyncio.CancelledError):
            await power.push_hold(1)
        self.assertFalse(bot.arm_down)
        self.assertEqual(calls["n"], 2)

    async def test_unknown_hold_mode(self):
        _, power = make()
        with self.assertRaisesRegex(PowerError, "hold-mode"):
            await power.push_hold(1, "x")


class PushHoldDeviceTest(unittest.IsolatedAsyncioTestCase):
    async def test_sets_presses_and_restores(self):
        bot, power = make()
        await power.push_hold(3, sp.HOLD_DEVICE)
        self.assertEqual(names(bot), ["set_hold_3", "press", "set_hold_0"])
        self.assertEqual(bot.info["holdSeconds"], 0)

    async def test_restores_even_if_press_fails(self):
        bot, power = make()
        bot.reject.add("press")
        with self.assertRaises(PowerError):
            await power.push_hold(3, sp.HOLD_DEVICE)
        self.assertEqual(bot.info["holdSeconds"], 0)

    async def test_restore_failure_is_reported(self):
        bot, power = make()
        bot.reject.add("set_hold_0")
        with self.assertRaisesRegex(PowerError, "元の 0 秒に戻せません"):
            await power.push_hold(3, sp.HOLD_DEVICE)

    async def test_restores_original_nonzero_setting(self):
        bot = FakeBot(info={"holdSeconds": 1})
        power = BotPower(bot, sleep=bot.sleep, log=lambda m: None)
        await power.push_hold(3, sp.HOLD_DEVICE)
        self.assertEqual(names(bot), ["set_hold_3", "press", "set_hold_1"])
        self.assertEqual(bot.info["holdSeconds"], 1)

    async def test_non_integer_seconds_refused(self):
        bot, power = make()
        with self.assertRaisesRegex(PowerError, "整数"):
            await power.push_hold(2.5, sp.HOLD_DEVICE)
        self.assertEqual(names(bot), [])


class G1SequenceTest(unittest.IsolatedAsyncioTestCase):
    async def test_power_on_is_tap_gap_hold(self):
        bot, power = make()
        await power.g1_power_on(gap=0.5, hold=3.0)
        self.assertEqual(names(bot), ["press", "hand_down", "hand_up"])
        t = {n: t for t, n in bot.events}
        self.assertAlmostEqual(t["hand_down"] - t["press"], 0.5)
        self.assertAlmostEqual(t["hand_up"] - t["hand_down"], 3.0)

    async def test_power_on_stops_if_tap_fails(self):
        bot, power = make()
        bot.reject.add("press")
        with self.assertRaises(PowerError):
            await power.g1_power_on()
        self.assertNotIn("hand_down", names(bot))

    async def test_power_on_validates_before_any_press(self):
        bot, power = make()
        with self.assertRaises(PowerError):
            await power.g1_power_on(hold=99)
        with self.assertRaises(PowerError):
            await power.g1_power_on(gap=-1)
        self.assertEqual(bot.events, [])

    async def test_power_off_requires_confirmation(self):
        bot, power = make()
        with self.assertRaisesRegex(PowerError, "confirm-damped"):
            await power.g1_power_off(confirm_damped=False)
        self.assertEqual(bot.events, [])

    async def test_power_off_holds_when_confirmed(self):
        bot, power = make()
        await power.g1_power_off(confirm_damped=True, hold=3.0)
        self.assertEqual(names(bot), ["hand_down", "hand_up"])

    async def test_defaults_meet_unitree_two_second_rule(self):
        self.assertGreaterEqual(sp.G1_HOLD_S, 2.0)


class VerifyReachableTest(unittest.IsolatedAsyncioTestCase):
    async def _wait(self, probe, want, timeout=10.0):
        clock = FakeBot()
        return await sp.wait_reachable("192.0.2.1", want, timeout, probe=probe, sleep=clock.sleep, interval_s=2.0), clock.now

    async def test_becomes_reachable(self):
        seq = iter([False, False, True])
        ok, waited = await self._wait(lambda h: next(seq), True)
        self.assertTrue(ok)
        self.assertAlmostEqual(waited, 4.0)

    async def test_never_reachable_times_out(self):
        ok, waited = await self._wait(lambda h: False, True, timeout=10)
        self.assertFalse(ok)
        self.assertAlmostEqual(waited, 10.0)

    async def test_becomes_unreachable_for_off(self):
        seq = iter([True, False])
        ok, _ = await self._wait(lambda h: next(seq), False)
        self.assertTrue(ok)


try:
    import switchbot as _sb
except ImportError:
    _sb = None


@unittest.skipIf(_sb is None, "PySwitchbot 未インストール")
class PySwitchbotSignatureTest(unittest.TestCase):
    """実 BLE 経路は自動テストできないので、呼び出し先の実シグネチャが合っているかだけ確かめる。"""

    def test_calls_match_installed_pyswitchbot(self):
        import inspect
        self.assertIn("scan_timeout", inspect.signature(_sb.GetSwitchbotDevices.discover).parameters)
        self.assertIn("interface", inspect.signature(_sb.GetSwitchbotDevices.__init__).parameters)
        base = inspect.signature(_sb.Switchbot.__mro__[-2].__init__).parameters  # Switchbot は *args を基底へ渡す
        for p in ("device", "password", "interface"):
            self.assertIn(p, base)
        for m in ("press", "hand_up", "hand_down", "set_long_press", "get_basic_info"):
            self.assertTrue(callable(getattr(_sb.Switchbot, m)), m)


class CliTest(unittest.TestCase):
    def setUp(self):
        from switchbot_power import cli  # noqa: PLC0415
        self.cli = cli

    def test_verify_ip_rejects_option_injection(self):
        for bad in ("-c1", "--help", "g1.local; rm", "999.1.1.1"):
            with self.assertRaises(PowerError, msg=bad):
                self.cli._verify_host(bad)
        self.assertEqual(self.cli._verify_host("192.168.123.164"), "192.168.123.164")
        self.assertEqual(self.cli._verify_host(""), "")

    def test_fake_g1_on_end_to_end(self):
        args = self.cli.build_parser().parse_args(["--fake", "g1-on"])
        self.assertEqual(asyncio.run(self.cli.run(args)), 0)

    def test_fake_g1_off_without_confirmation_fails_exit_code_1(self):
        sys.argv = ["x", "--fake", "g1-off"]
        self.assertEqual(self.cli.entry(), 1)

    def test_real_backend_requires_address(self):
        args = self.cli.build_parser().parse_args(["push-once"])
        args.address = ""
        with self.assertRaisesRegex(PowerError, "address"):
            asyncio.run(self.cli.run(args))


if __name__ == "__main__":
    unittest.main()
