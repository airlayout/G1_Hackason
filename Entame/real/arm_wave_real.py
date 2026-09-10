#!/usr/bin/env python
"""実機G1にUnitree標準の腕ジェスチャー("high wave")をさせるスクリプト。

`Entame/README.md`の「実現方法の選択」でいうSDK方式（`sport_mode`）の実例。
Unitreeが工場出荷時から用意している定型モーションを`G1ArmActionClient.ExecuteAction()`
で呼び出すだけなので、G1本体側に追加のセットアップは不要（`lerobot`方式のような
独自ONNXポリシー・G1本体側conda環境は要らない）。

## 既知の落とし穴（実機確認済み）

`LocoClient.WaveHand()`は**RPCがcode=0(成功)を返すにもかかわらず、実際には
腕が動かない**。歩行系の`LocoClient`ではなく、専用の`G1ArmActionClient`の
`ExecuteAction()`を使うこと。この落とし穴は`FAILURES.md`にも記録している。

`ExecuteAction()`は動作名の文字列ではなく整数の action_id を取る。
以下は`unitree_sdk2py`の`g1_arm_action_client.py`にある対応表（抜粋、"high wave"=26)。
ダンス・パフォーマンス用の振り付けを増やす際は、まずこの中から使えるものを探すとよい。

    ACTION_IDS = {
        "release arm": 99, "two-hand kiss": 11, "left kiss": 12, "right kiss": 13,
        "hands up": 15, "clap": 17, "high five": 18, "hug": 19, "heart": 20,
        "right heart": 21, "reject": 22, "right hand up": 23, "x-ray": 24,
        "face wave": 25, "high wave": 26, "shake hand": 27,
    }

安全のため、歩行スクリプト（`SimpleWalk/real/walk_forward_real_sdk.py`）と
同様に実行前確認を挟む。腕の動作単体でも、モード遷移時や動作中に姿勢を崩す
リスクはゼロではないため、人間が支えられる状態で実行すること。

前提:
  - G1本体の電源が入り、転倒防止（人間が支える／自立スタンド）ができていること。
  - 操作側PCがG1と同一サブネット(192.168.123.x, x != 164)のstatic IPを持ち、
    Ethernetで直結されていること（`Common/network/setup_ethernet_for_g1.sh`参照）。
  - `SimpleWalk/real/walk_forward_real_sdk.py`など、高レベル制御(sport_mode)を
    使う他スクリプトと同時には実行しないこと（制御主体が競合する）。

使い方:
  python Entame/real/arm_wave_real.py --network-interface enp3s0
  python Entame/real/arm_wave_real.py --network-interface enp3s0 --action "shake hand"
"""
import argparse
import sys
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.arm.g1_arm_action_client import G1ArmActionClient

# unitree_sdk2py/g1/arm/g1_arm_action_client.py の action_map より。
ACTION_IDS = {
    "release arm": 99,
    "two-hand kiss": 11,
    "left kiss": 12,
    "right kiss": 13,
    "hands up": 15,
    "clap": 17,
    "high five": 18,
    "hug": 19,
    "heart": 20,
    "right heart": 21,
    "reject": 22,
    "right hand up": 23,
    "x-ray": 24,
    "face wave": 25,
    "high wave": 26,
    "shake hand": 27,
}


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--network-interface",
        default="enp3s0",
        help="G1と直結しているネットワークインターフェース名(`ip -br a`で確認)",
    )
    parser.add_argument(
        "--action",
        default="high wave",
        choices=sorted(ACTION_IDS),
        help="実行するジェスチャー(実機で確認済みなのは'high wave'のみ。他は未検証)",
    )
    parser.add_argument(
        "--yes",
        action="store_true",
        help="安全確認プロンプトをスキップする(自動実行用。通常は付けないこと)",
    )
    args = parser.parse_args()

    if not args.yes:
        print(
            "!!! 実機のG1を動かします。転倒防止(人間による保持／自立スタンド)が"
            "できていることを確認してください。 !!!",
            flush=True,
        )
        if input("続行しますか？ [y/N]: ").strip().lower() != "y":
            print("Aborted by user.", flush=True)
            sys.exit(1)

    print(f"Initializing DDS on interface {args.network_interface}...", flush=True)
    ChannelFactoryInitialize(0, args.network_interface)

    msc = MotionSwitcherClient()
    msc.SetTimeout(5.0)
    msc.Init()

    print("Selecting sport_mode ('ai')...", flush=True)
    code, _ = msc.SelectMode("ai")
    if code != 0:
        print(f"SelectMode failed (code={code}). Aborting.", flush=True)
        sys.exit(1)

    arm_client = G1ArmActionClient()
    arm_client.SetTimeout(10.0)
    arm_client.Init()

    action_id = ACTION_IDS[args.action]
    print(f"Executing action '{args.action}' (id={action_id})...", flush=True)
    code = arm_client.ExecuteAction(action_id)
    if code != 0:
        print(f"ExecuteAction failed (code={code}).", flush=True)
        sys.exit(1)

    # 動作が完了するまでの待ち時間の目安。実測に基づく厳密な同期ではないため、
    # 動作によってはもっと長く/短く調整が必要になる可能性がある。
    time.sleep(3.0)

    msc.ReleaseMode()
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
