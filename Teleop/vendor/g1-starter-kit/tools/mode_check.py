#!/usr/bin/env python3
"""ロボットが腕の指令を受け付ける状態かを調べる／整える。

腕が動くには 3 つの条件が同時に要る。どれが欠けても **エラーを出さずに動かない**。

  1. 指令トピックが機体の状態と合っている
       モーションコントローラ稼働中 → rt/arm_sdk（teleop に --motion）
       デバッグ状態（停止）        → rt/lowcmd （teleop は --motion なし）
  2. 腕のモータが有効（rt/lowstate の motor_state[].mode == 1）
       FSM 0（ゼロトルク）だと mode=0 になり、何を送っても効かない。
       有効化はロボットの FSM 側（リモコンでダンピング = FSM 1）でしか行えず、
       指令メッセージの motor_cmd[].mode=1 では有効にならない。
       **テレオペや再生を終了するたびにゼロトルクへ戻る**ので、毎回確認する。
  3. （rt/arm_sdk の場合）モーションコントローラが制御権の委譲に応じる
       検証した機体では Regular モードでも応じなかった（tools/armsdk_probe.py で
       実測 0.000 rad）。応じない機体では 2 + rt/lowcmd が唯一の経路になる。

使い方:
    python3 tools/mode_check.py --iface enp129s0                 # 人が読む形で表示
    python3 tools/mode_check.py --iface enp129s0 --print         # 指令経路を 1 語で
    python3 tools/mode_check.py --iface enp129s0 --print-motors  # モータ状態を 1 語で
    python3 tools/mode_check.py --iface enp129s0 --release       # 解除してデバッグ状態へ
    python3 tools/mode_check.py --iface enp129s0 --restore       # ai モードへ戻す

--print の出力:        debug / <名前>（例: ai） / unknown
--print-motors の出力: enabled / disabled / unknown

なぜリトライするのか:
    MotionSwitcherClient の RPC は「[ClientStub] send request error」で散発的に
    失敗する。xr_teleoperate 同梱の MotionSwitcher は SetTimeout(1.0) かつ
    リトライ無しで、例外を握りつぶして (None, None) を返すため、実際には解除
    できる状況でも「Enter debug mode: Failed」と誤表示される。
"""

import argparse
import sys
import threading
import time

CHECK_ATTEMPTS = 6
CHECK_INTERVAL = 0.6
RELEASE_ATTEMPTS = 8
RPC_TIMEOUT = 5.0
STATE_WAIT = 5.0

# 23DoF / 29DoF の両方に存在する腕関節（肩 3 + 肘 + 手首ロール）。
# 29DoF だけにある手首ピッチ/ヨー(20,21,27,28)は判定に使わない。
ARM_CORE = (15, 16, 17, 18, 19, 22, 23, 24, 25, 26)


def _init(iface):
    from unitree_sdk2py.core.channel import ChannelFactoryInitialize
    ChannelFactoryInitialize(0, iface)


def _switcher():
    from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
    msc = MotionSwitcherClient()
    msc.SetTimeout(RPC_TIMEOUT)
    msc.Init()
    return msc


def check_mode(msc, attempts=CHECK_ATTEMPTS, verbose=False):
    """現在のモード名を返す。稼働中なら名前、停止なら ""、判定不能なら None。"""
    for i in range(attempts):
        try:
            status, result = msc.CheckMode()
            if status == 0 and result is not None:
                return result.get("name") or ""
            if verbose:
                print(f"  試行{i+1}: status={status} result={result}", file=sys.stderr)
        except Exception as e:
            if verbose:
                print(f"  試行{i+1}: {type(e).__name__}: {e}", file=sys.stderr)
        time.sleep(CHECK_INTERVAL)
    return None


def motor_state(wait=STATE_WAIT):
    """腕モータの有効状態を返す。'enabled' / 'disabled' / 'unknown'。

    rt/lowstate を 1 件受信して motor_state[].mode を見る。1 が有効。
    """
    from unitree_sdk2py.core.channel import ChannelSubscriber
    from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowState_
    box, lock = {}, threading.Lock()

    def cb(m):
        with lock:
            box["m"] = m

    sub = ChannelSubscriber("rt/lowstate", LowState_)
    sub.Init(cb, 10)
    t0 = time.time()
    while time.time() - t0 < wait:
        with lock:
            if "m" in box:
                modes = {box["m"].motor_state[i].mode for i in ARM_CORE}
                return "enabled" if modes == {1} else "disabled"
        time.sleep(0.05)
    return "unknown"


def release(msc):
    name = check_mode(msc, verbose=True)
    if name is None:
        print("  モードを判定できませんでした。")
        return False
    if name == "":
        print("  既にデバッグ状態です。解除の必要はありません。")
        return True
    print(f"  '{name}' を解除します ...")
    for attempt in range(1, RELEASE_ATTEMPTS + 1):
        try:
            msc.ReleaseMode()
        except Exception as e:
            print(f"  ReleaseMode 試行{attempt}: {type(e).__name__}: {e}")
        time.sleep(1.0)
        if check_mode(msc, attempts=3) == "":
            print("  解除しました。デバッグ状態です（rt/lowcmd が効きます）。")
            print("  ※ 解除後はモータがゼロトルクに落ちることがあります。")
            print("     動かない場合はリモコンでダンピングに入れてください。")
            return True
    print("  解除できませんでした。リモコン側での操作が必要かもしれません。")
    return False


def restore(msc):
    for attempt in range(1, RELEASE_ATTEMPTS + 1):
        try:
            status, _ = msc.SelectMode(nameOrAlias="ai")
            if status == 0:
                break
            print(f"  SelectMode 試行{attempt}: status={status}")
        except Exception as e:
            print(f"  SelectMode 試行{attempt}: {type(e).__name__}: {e}")
        time.sleep(1.0)
    time.sleep(1.5)
    name = check_mode(msc, attempts=3)
    if name:
        print(f"  '{name}' に戻りました。")
        return True
    print("  戻せませんでした。")
    return False


def main():
    p = argparse.ArgumentParser(description="腕の指令を受け付ける状態かを調べる／整える")
    p.add_argument("--iface", required=True, help="ロボットと繋がっている有線インターフェース名")
    g = p.add_mutually_exclusive_group()
    g.add_argument("--print", dest="print_only", action="store_true",
                   help="指令経路を 1 語で出す: debug / <名前> / unknown")
    g.add_argument("--print-motors", action="store_true",
                   help="腕モータの状態を 1 語で出す: enabled / disabled / unknown")
    g.add_argument("--release", action="store_true", help="解除してデバッグ状態にする")
    g.add_argument("--restore", action="store_true", help="ai モードへ戻す")
    args = p.parse_args()

    try:
        _init(args.iface)
    except Exception as e:
        if args.print_only or args.print_motors:
            print("unknown")
            return 0
        print(f"エラー: DDS を初期化できません ({type(e).__name__}: {e})")
        return 1

    if args.print_motors:
        print(motor_state())
        return 0

    try:
        msc = _switcher()
    except Exception as e:
        if args.print_only:
            print("unknown")
            return 0
        print(f"エラー: MotionSwitcherClient を初期化できません ({type(e).__name__}: {e})")
        return 1

    if args.release:
        return 0 if release(msc) else 1
    if args.restore:
        return 0 if restore(msc) else 1

    name = check_mode(msc, verbose=not args.print_only)
    if args.print_only:
        print("unknown" if name is None else (name or "debug"))
        return 0

    motors = motor_state()

    print("\n腕の指令を受け付ける状態か\n")
    print("1. モーションコントローラ")
    if name is None:
        print("   判定できませんでした（RPC が応答しません）。有線接続と電源を確認してください。")
    elif name == "":
        print("   停止（デバッグ状態） → 有効な経路: rt/lowcmd")
        print("   テレオペ: ./scripts/teleop.sh              （--motion を付けない）")
    else:
        print(f"   '{name}' が稼働中 → 規格上の経路: rt/arm_sdk（--motion）")
        print("   ⚠️ 検証した機体では Regular モードでも arm_sdk が効きませんでした。")
        print("      効くかは tools/armsdk_probe.py で 3 度だけ動かして確かめられます。")
        print("   確実なのは解除して rt/lowcmd を使うこと（必ず支持された状態で）:")
        print(f"      python3 tools/mode_check.py --iface {args.iface} --release")

    print("\n2. 腕のモータ")
    if motors == "enabled":
        print("   有効（mode=1）")
    elif motors == "disabled":
        print("   ❌ 無効（mode=0 = ゼロトルク）。この状態では何を送っても動きません。")
        print("      リモコンでダンピング（FSM 1）に入れてください。")
        print("      ※ テレオペ・再生を終了するたびにここへ戻るので、毎回必要です。")
    else:
        print("   判定できませんでした（rt/lowstate 未受信）。")

    ok = (name is not None) and (motors == "enabled")
    print("\n" + ("→ 腕を動かせる状態です。" if ok else "→ まだ動かせません。上の項目を整えてください。"))
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
