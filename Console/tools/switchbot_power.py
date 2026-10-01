#!/usr/bin/env python3
"""SwitchBot Bot で G1 の電源ボタンを押す CLI。設計は docs/switchbot-power/DESIGN.md。

  scan                          近くの Bot を一覧（接続しない）
  info                          基本情報（モード・長押し設定・電池）
  set-hold --seconds N          Bot の長押し設定（保存される）を変える。短押しは 0
  push-once                     押して、すぐ離す
  push-hold --seconds N         N 秒押し続けて離す
  g1-on  [--verify-ip IP]       短押し → 間隔 → 長押し（G1 起動手順）
  g1-off --confirm-damped       長押し（G1 停止手順。ダンピング済みの確認が必須）

--fake は BLE を使わず押下・解放のタイムラインを表示する。パスワードは環境変数 SWITCHBOT_PASSWORD。
終了コード: 0 成功 / 1 失敗（電源操作の失敗・G1 の到達確認の不一致）/ 2 引数エラー
"""
import argparse
import asyncio
import ipaddress
import os
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent.parent))
from g1console import switchbot_power as sp  # noqa: E402


def build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--address", default=os.environ.get("SWITCHBOT_ADDRESS", ""), help="Bot の BLE アドレス（環境変数 SWITCHBOT_ADDRESS）")
    p.add_argument("--interface", type=int, default=0, help="Linux の BT アダプタ番号（hci<N>）")
    p.add_argument("--fake", action="store_true", help="BLE を使わず偽の Bot で実行する")
    p.add_argument("--hold-mode", choices=[sp.HOLD_HOST, sp.HOLD_DEVICE], default=sp.HOLD_HOST)
    sub = p.add_subparsers(dest="cmd", required=True)
    sub.add_parser("scan")
    sub.add_parser("info")
    sh = sub.add_parser("set-hold")
    sh.add_argument("--seconds", type=int, required=True)
    sub.add_parser("push-once")
    h = sub.add_parser("push-hold")
    h.add_argument("--seconds", type=float, required=True)
    for name in ("g1-on", "g1-off"):
        g = sub.add_parser(name)
        g.add_argument("--hold", type=float, default=sp.G1_HOLD_S, help="長押しの秒数")
        g.add_argument("--verify-ip", default="", help="G1 の IP。指定すると ping で起動／停止を自動確認する")
        g.add_argument("--verify-timeout", type=float, default=sp.VERIFY_TIMEOUT_S)
        if name == "g1-on":
            g.add_argument("--gap", type=float, default=sp.G1_ON_GAP_S, help="短押しの後の間隔（秒）")
        else:
            g.add_argument("--confirm-damped", action="store_true", help="ダンピング済みで支えていることの確認")
    return p


def _verify_host(value: str) -> str:
    """ping に渡すので IP アドレスだけ許す（先頭 `-` 等のオプション注入を防ぐ）。"""
    if not value:
        return ""
    try:
        return str(ipaddress.ip_address(value))
    except ValueError:
        raise sp.PowerError("--verify-ip は IP アドレスで指定してください（指定: %s）" % value)


async def _make_backend(args):
    if args.fake:
        bot = sp.FakeBot()
        return bot, bot.sleep
    from g1console.switchbot_ble import PySwitchbotBackend  # noqa: PLC0415
    if not args.address:
        raise sp.PowerError("--address（または環境変数 SWITCHBOT_ADDRESS）が必要です。scan で調べてください")
    backend = await PySwitchbotBackend.connect(args.address, os.environ.get("SWITCHBOT_PASSWORD"), args.interface)
    return backend, asyncio.sleep


async def _verify(args, want: bool, host: str, sleep) -> None:
    ok = await sp.wait_reachable(host, want, args.verify_timeout, sleep=sleep)
    state = "起動（到達可）" if want else "停止（到達不可）"
    if not ok:
        raise sp.PowerError("G1 の%sを %.0f 秒待っても確認できませんでした（%s）" % (state, args.verify_timeout, host))
    print("[switchbot] G1 の%sを確認しました（%s）" % (state, host))


async def run(args) -> int:
    if args.cmd == "scan":
        from g1console.switchbot_ble import scan_bots  # noqa: PLC0415
        bots = await scan_bots(args.interface)
        for b in bots:
            print("%s  rssi=%s  switchMode=%s  battery=%s" % (b["address"], b["rssi"], b["switchMode"], b["battery"]))
        print("[switchbot] Bot %d 台" % len(bots))
        return 0
    host = _verify_host(getattr(args, "verify_ip", ""))
    backend, sleep = await _make_backend(args)
    power = sp.BotPower(backend, sleep=sleep)
    if args.cmd == "info":
        print(await backend.get_info())
    elif args.cmd == "set-hold":
        if not 0 <= args.seconds <= sp.MAX_DEVICE_HOLD_S:
            raise sp.PowerError("--seconds は 0〜%d で指定してください" % sp.MAX_DEVICE_HOLD_S)
        if not await backend.set_hold_seconds(args.seconds):
            raise sp.PowerError("長押し設定を変更できませんでした")
        print("[switchbot] 長押し設定: %d 秒 → 読み戻し: %s" % (args.seconds, (await backend.get_info()).get("holdSeconds")))
    elif args.cmd == "push-once":
        await power.push_once()
    elif args.cmd == "push-hold":
        await power.push_hold(args.seconds, args.hold_mode)
    elif args.cmd == "g1-on":
        await power.g1_power_on(args.gap, args.hold, args.hold_mode)
        if host:
            await _verify(args, True, host, sleep)
    else:
        await power.g1_power_off(args.confirm_damped, args.hold, args.hold_mode)
        if host:
            await _verify(args, False, host, sleep)
    if args.fake:
        for t, name in backend.events:
            print("[fake] t=%6.2f  %s" % (t, name))
    return 0


def entry() -> int:
    args = build_parser().parse_args()
    try:
        return asyncio.run(run(args))
    except sp.PowerError as e:
        print("[switchbot] 失敗: %s" % e, file=sys.stderr)
        return 1


if __name__ == "__main__":
    sys.exit(entry())
