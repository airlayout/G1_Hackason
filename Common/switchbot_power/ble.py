"""PySwitchbot（bleak 経由の BLE）で Bot を操作する実体。`BotBackend` を満たす。

PySwitchbot は `pip install pyswitchbot==2.10.0`。標準ライブラリだけで動く core を汚さないよう遅延 import する。
Linux は BlueZ が必要。macOS は端末にアドレスではなく UUID が見える（同じ文字列を --address に渡す）。
"""
from typing import Optional

from .core import PowerError

SCAN_TIMEOUT_S = 8
BLE_RETRY = 3


def _import():
    try:
        import switchbot  # noqa: PLC0415
    except ImportError as e:
        raise PowerError("PySwitchbot が入っていません。`pip install pyswitchbot==2.10.0` を実行してください") from e
    return switchbot


BOT_MODEL = "H"  # 広告の model 文字。H = WoHand（Bot）


async def _find_bots(sb, interface: int, scan_timeout: int) -> dict:
    """広告をスキャンして Bot だけ返す。{アドレス: SwitchBotAdvertisement}"""
    found = await sb.GetSwitchbotDevices(interface=interface).discover(scan_timeout=scan_timeout)
    return {a: v for a, v in found.items() if v.data.get("model") == BOT_MODEL}


async def scan_bots(interface: int = 0, scan_timeout: int = SCAN_TIMEOUT_S) -> list:
    """近くの Bot を広告から一覧する（接続しない）。[{address, rssi, switchMode, isOn, battery}]"""
    sb = _import()
    found = await _find_bots(sb, interface, scan_timeout)
    return [{"address": a, "rssi": v.rssi, **{k: v.data.get("data", {}).get(k) for k in ("switchMode", "isOn", "battery")}}
            for a, v in sorted(found.items(), key=lambda kv: -kv[1].rssi)]


class PySwitchbotBackend:
    def __init__(self, bot, adv_address: str):
        self._bot = bot
        self.address = adv_address

    @classmethod
    async def connect(cls, address: str, password: Optional[str] = None, interface: int = 0,
                      scan_timeout: int = SCAN_TIMEOUT_S) -> "PySwitchbotBackend":
        sb = _import()
        found = await _find_bots(sb, interface, scan_timeout)
        adv = next((v for a, v in found.items() if a.lower() == address.lower()), None)
        if adv is None:
            raise PowerError("Bot %s が見つかりません（電波圏外・他の機器が接続中・アドレス違いの可能性）。scan で確認してください" % address)
        bot = sb.Switchbot(adv.device, password=password, interface=interface, retry_count=BLE_RETRY)
        return cls(bot, adv.address)

    async def press(self) -> bool:
        return bool(await self._bot.press())

    async def hand_down(self) -> bool:
        return bool(await self._bot.hand_down())

    async def hand_up(self) -> bool:
        return bool(await self._bot.hand_up())

    async def set_hold_seconds(self, seconds: int) -> bool:
        return bool(await self._bot.set_long_press(seconds))

    async def get_info(self) -> dict:
        info = await self._bot.get_basic_info()
        if not info:
            raise PowerError("Bot の基本情報を読めませんでした（通信失敗）")
        return info
