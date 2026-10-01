"""SwitchBot Bot で G1 の電源ボタンを押すための本体ロジック（BLE に依存しない）。

BLE の実体は `switchbot_ble.py`。ここは `BotBackend` という窓口だけを知っているので、
偽の Bot（`FakeBot`）で押下・解放の順序と時間を自動テストできる。設計は docs/switchbot-power/DESIGN.md。
"""
import asyncio
import subprocess
from typing import Awaitable, Callable, Optional, Protocol

MIN_HOLD_S = 0.1  # これより短い押しっぱなしは意味がない
MAX_HOLD_S = 10.0  # 押しっぱなし時間の上限（誤指定で押し続けない）
G1_ON_GAP_S = 0.5  # 起動手順: 短押しを離してから長押しを始めるまで（実機で調整）
G1_HOLD_S = 3.0  # 起動・停止の長押し（手順は 2 秒以上）
RELEASE_RETRIES = 12  # アームを上げる命令の再送回数（実機: hand_down 後 2〜3 秒は「動作中 03」で拒否される）
RELEASE_RETRY_INTERVAL_S = 0.4  # 再送の間隔。合計で約 5 秒待つ
ACQUIRE_RETRIES = 30  # hand_down の再送回数（実機: press の後は約 2.9 秒「動作中 03」で拒否される）
ACQUIRE_RETRY_INTERVAL_S = 0.2  # 合計で約 6 秒待つ。受理された時点から保持時間を数える
VERIFY_TIMEOUT_S = 120.0  # G1 の起動は約 1 分
VERIFY_INTERVAL_S = 2.0
PING_TIMEOUT_S = 3.0
MAX_DEVICE_HOLD_S = 60  # Bot 本体の長押し設定の上限（整数秒）

HOLD_HOST = "host"  # hand_down → 待つ → hand_up
HOLD_DEVICE = "device"  # 長押し秒数を Bot に設定し、press で Bot 自身に離させる


class PowerError(Exception):
    """電源操作を実行できない／失敗した。メッセージはそのまま利用者に見せられる。"""


class BotBackend(Protocol):
    """Bot への命令。成功は True を返し、失敗は False か例外にする。"""

    async def press(self) -> bool: ...
    async def hand_down(self) -> bool: ...
    async def hand_up(self) -> bool: ...
    async def set_hold_seconds(self, seconds: int) -> bool: ...
    async def get_info(self) -> dict: ...


Sleep = Callable[[float], Awaitable[None]]
Log = Callable[[str], None]


def _log(msg: str) -> None:
    print("[switchbot] " + msg)


def validate_hold(seconds: float) -> float:
    if not isinstance(seconds, (int, float)) or isinstance(seconds, bool):
        raise PowerError("押しっぱなしの秒数は数値で指定してください")
    if not MIN_HOLD_S <= seconds <= MAX_HOLD_S:
        raise PowerError("押しっぱなしの秒数は %s〜%s 秒の範囲で指定してください（指定: %s）" % (MIN_HOLD_S, MAX_HOLD_S, seconds))
    return float(seconds)


class BotPower:
    def __init__(self, backend: BotBackend, sleep: Sleep = asyncio.sleep, log: Log = _log):
        self._b = backend
        self._sleep = sleep
        self._log = log

    async def check_ready(self, need_tap: bool = True) -> dict:
        """Press モード（と、タップなら長押し設定 0 秒）であることを読み戻して確認する。"""
        info = await self._b.get_info()
        if info.get("switchMode"):
            raise PowerError("Bot が Switch モードです。press がトグルになるので実行しません。アプリで Press モードにしてください")
        if need_tap and info.get("holdSeconds", 0) != 0:
            raise PowerError("Bot の長押し設定が %s 秒です。短押しにならないので 0 秒にしてください" % info.get("holdSeconds"))
        return info

    async def push_once(self) -> None:
        """押して、すぐ離す。"""
        await self.check_ready(need_tap=True)
        await self._cmd("press", self._b.press())
        self._log("push-once 完了")

    async def push_hold(self, seconds: float, mode: str = HOLD_HOST) -> None:
        """seconds 秒押し続けて離す。途中で失敗・中断されても解放を試みる。"""
        seconds = validate_hold(seconds)
        if mode == HOLD_HOST:
            await self._hold_host(seconds)
        elif mode == HOLD_DEVICE:
            await self._hold_device(seconds)
        else:
            raise PowerError("hold-mode は %s か %s です（指定: %s）" % (HOLD_HOST, HOLD_DEVICE, mode))
        self._log("push-hold %.1f 秒（%s）完了" % (seconds, mode))

    async def g1_power_on(self, gap: float = G1_ON_GAP_S, hold: float = G1_HOLD_S, mode: str = HOLD_HOST) -> None:
        """起動手順: 短押し → すぐ離す → 間隔 → 長押し。"""
        validate_hold(hold)
        if gap < 0:
            raise PowerError("間隔は 0 秒以上で指定してください")
        await self.push_once()
        await self._sleep(gap)
        await self.push_hold(hold, mode)

    async def g1_power_off(self, confirm_damped: bool, hold: float = G1_HOLD_S, mode: str = HOLD_HOST) -> None:
        """停止手順: 長押し。立位のまま切ると転倒するので、確認なしでは実行しない。"""
        if not confirm_damped:
            raise PowerError("電源 OFF はダンピング済み（座位か吊り下げで支えた状態）の確認が必要です。--confirm-damped を付けてください")
        await self.push_hold(hold, mode)

    async def _cmd(self, name: str, call: Awaitable[bool]) -> None:
        if not await call:
            raise PowerError("Bot が %s を受け付けませんでした" % name)

    async def _hold_host(self, seconds: float) -> None:
        await self.check_ready(need_tap=False)
        await self._acquire()
        try:
            await self._sleep(seconds)
        finally:
            await self._release()

    async def _acquire(self) -> None:
        """hand_down が受理されるまで、Bot が動作中の間は待ちながら再送する。"""
        for _ in range(ACQUIRE_RETRIES):
            if await self._b.hand_down():
                return
            await self._sleep(ACQUIRE_RETRY_INTERVAL_S)
        raise PowerError("Bot が hand_down を受け付けませんでした（動作中のまま。%d 回再送）" % ACQUIRE_RETRIES)

    async def _release(self) -> None:
        """hand_up を確実に送る。待機中にキャンセルされても送り、その後でキャンセルを伝える。"""
        cancelled = False
        released = False
        reason = "未実行"
        for _ in range(RELEASE_RETRIES):
            try:
                if await self._b.hand_up():
                    released = True
                    break
                reason = "hand_up が拒否された（Bot が動作中の可能性）"
            except asyncio.CancelledError:
                cancelled = True
                reason = "解放中にも中断された"
                continue
            except Exception as e:  # noqa: BLE001 — 失敗は握りつぶさず下で報告する
                reason = str(e)
            await self._sleep(RELEASE_RETRY_INTERVAL_S)
        if not released:
            raise PowerError("アームが押下位置のままの可能性があります。Bot を手動で確認してください（原因: %s）" % reason)
        if cancelled:
            raise asyncio.CancelledError()

    async def _hold_device(self, seconds: float) -> None:
        """Bot 本体に長押し秒数を持たせる。PC が落ちても Bot 自身が離す。設定は終了時に 0 へ戻す。"""
        n = int(round(seconds))
        if n < 1 or n > MAX_DEVICE_HOLD_S or abs(n - seconds) > 1e-9:
            raise PowerError("device 方式の秒数は 1〜%d の整数で指定してください（指定: %s）" % (MAX_DEVICE_HOLD_S, seconds))
        original = int((await self.check_ready(need_tap=False)).get("holdSeconds", 0))
        await self._cmd("set_hold_seconds", self._b.set_hold_seconds(n))
        try:
            await self._cmd("press", self._b.press())
            await self._sleep(n)
        finally:
            await self._restore_hold(original)

    async def _restore_hold(self, original: int) -> None:
        """長押し設定を実行前の値に戻す。戻せないと以後の押し方が変わるので、失敗は必ず報告する。"""
        msg = "長押し設定を元の %d 秒に戻せませんでした。アプリで確認してください" % original
        try:
            ok = await self._b.set_hold_seconds(original)
        except Exception as e:  # noqa: BLE001
            raise PowerError("%s（原因: %s）" % (msg, e)) from e
        if not ok:
            raise PowerError(msg)


def ping_once(host: str) -> bool:
    """ping を 1 回打つ。host は呼び出し側が IP アドレスとして検証済みであること。"""
    try:
        r = subprocess.run(["ping", "-c", "1", host], capture_output=True, timeout=PING_TIMEOUT_S)
    except subprocess.TimeoutExpired:
        return False
    return r.returncode == 0


async def wait_reachable(host: str, want: bool, timeout_s: float = VERIFY_TIMEOUT_S, probe: Callable[[str], bool] = ping_once,
                         sleep: Sleep = asyncio.sleep, interval_s: float = VERIFY_INTERVAL_S) -> bool:
    """G1 が到達できる（want=True）／できなくなる（want=False）まで待つ。時間内に達したら True。"""
    waited = 0.0
    while True:
        if await asyncio.to_thread(probe, host) == want:
            return True
        if waited >= timeout_s:
            return False
        await sleep(interval_s)
        waited += interval_s


class FakeBot:
    """Bot なしでの確認・自動テスト用。仮想時計に命令を記録し、アームの状態を模擬する。"""

    def __init__(self, info: Optional[dict] = None, hand_down_works: bool = True):
        self.now = 0.0
        self.events = []  # (時刻, 命令名)
        self.info = {"switchMode": False, "holdSeconds": 0, "battery": 100, "firmware": 6.3, "strength": 100, **(info or {})}
        self.arm_down = False
        self.fail = {}  # 命令名 → 残りの失敗回数（例外）
        self.reject = set()  # False を返す命令名
        self._hand_down_works = hand_down_works

    async def sleep(self, seconds: float) -> None:
        self.now += seconds
        await asyncio.sleep(0)

    def _record(self, name: str) -> bool:
        self.events.append((round(self.now, 6), name))
        if self.fail.get(name, 0) > 0:
            self.fail[name] -= 1
            raise ConnectionError("%s: 偽の通信失敗" % name)
        return name not in self.reject

    async def press(self) -> bool:
        ok = self._record("press")
        if ok and self.info["holdSeconds"] > 0:
            self.now += self.info["holdSeconds"]  # Bot 本体が押し続ける分の時間
        return ok

    async def hand_down(self) -> bool:
        ok = self._record("hand_down")
        self.arm_down = ok and self._hand_down_works
        return ok

    async def hand_up(self) -> bool:
        ok = self._record("hand_up")
        if ok:
            self.arm_down = False
        return ok

    async def set_hold_seconds(self, seconds: int) -> bool:
        ok = self._record("set_hold_%d" % seconds)
        if ok:
            self.info["holdSeconds"] = seconds
        return ok

    async def get_info(self) -> dict:
        self._record("get_info")
        return dict(self.info)
