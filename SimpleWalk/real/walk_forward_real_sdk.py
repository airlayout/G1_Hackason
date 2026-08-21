#!/usr/bin/env python
"""実機G1に接続し、Unitree SDK標準の高レベル歩行(sport_mode)で前進させるスクリプト。

walk_forward_real.py（lerobot + GrootLocomotionController、独自ONNXポリシーによる
低レベル関節制御）とは別方式。こちらは unitree_sdk2py が標準で提供する
G1_loco_client.LocoClient を使い、Unitree製品版の歩行コントローラをそのまま使う。
lerobot/torch/onnxruntimeは不要で、unitree_sdk2py + CycloneDDSだけで動く。

walk_forward_real.py との対応関係(パラメータは共通化している):
  --forward-duration / --forward-speed は同じ意味・同じデフォルト値。
  --robot-ip の代わりに --network-interface を使う: unitree_sdk2pyの直接DDS接続は
  IPで宛先を指定するのではなく、「どのネットワークインターフェース上でDDSの
  discoveryを行うか」を指定する方式のため(walk_forward_real.py は lerobot独自の
  ZMQブリッジ(run_g1_server.py)経由でIP指定するのに対し、こちらはG1本体に
  直接DDSで繋がるため run_g1_server.py は不要)。

安全のため3段階の確認を挟む(walk_forward_real.py と同じ考え方):
  1. 実行前 — 転倒防止の準備確認。
  2. 標準立位姿勢へ遷移させた後 — ここで一度停止し、安定して自立していることを
     目視確認してから前進コマンドを送るか判断する。
  3. 前進コマンド終了後 — Damp()(脱力)する前に、人間が支える準備ができたことを
     確認する。sport_mode自体はオンボードでバランスを取り続けるため
     walk_forward_real.py の hold_standing_until() のような常時コマンド送信は
     不要だが、脱力の瞬間の危険性は変わらないため確認ゲートは同様に設ける。

前提（安全上必須）:
  - G1本体の電源が入り、転倒しないよう安全確保された状態であること
    （人間が支える／自立スタンドを使う）。
  - 操作側PCが G1 と同一サブネット(192.168.123.x, x != 164)の
    static IP を持ち、Ethernetで直結されていること
    (Common/network/setup_ethernet_for_g1.sh 参照)。
  - run_g1_server.py は不要(直接DDS接続のため)。
  - 実行中は、このスクリプトが有効化する高レベル歩行サービス("ai"モード)が
    ロボットの制御を占有する。walk_forward_real.py 側の低レベル制御と同時には
    使えない。

使い方:
  python SimpleWalk/real/walk_forward_real_sdk.py --network-interface enp3s0
"""
import argparse
import sys
import time

from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--network-interface",
        default="enp3s0",
        help="G1と直結しているネットワークインターフェース名(`ip -br a`で確認)",
    )
    parser.add_argument("--forward-duration", type=float, default=5.0, help="前進を続ける秒数")
    # sim検証時の0.5より控えめな値をデフォルトにしている(walk_forward_real.pyと同じ理由)。
    parser.add_argument("--forward-speed", type=float, default=0.3, help="前進速度vx(m/s相当の指令値)")
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

    sport_client = LocoClient()
    sport_client.SetTimeout(10.0)
    sport_client.Init()

    # sportmode_test.py の例に合わせ、Damp(脱力/FSM待機状態)を経由してから
    # Squat2StandUp(しゃがみ姿勢→立位)を呼ぶ。ここが walk_forward_real.py の
    # connect()内reset()に相当する、標準立位姿勢への遷移。
    print("Standing up (Damp -> Squat2StandUp)...", flush=True)
    sport_client.Damp()
    time.sleep(0.5)
    sport_client.Squat2StandUp()
    time.sleep(3.0)  # 遷移が収まるまでの猶予(walk_forward_real.pyのconnect後1sより長めに取る)

    print(
        "\n!!! ここで一度停止します。ロボットが安定して自立していることを目視で確認してください。 !!!",
        flush=True,
    )
    if input("前進コマンドを送信して良ければ y、中断するなら他のキーを入力: ").strip().lower() != "y":
        print("前進コマンドを中断しました。安全に脱力します。", flush=True)
        sport_client.Damp()
        msc.ReleaseMode()
        print("DONE (aborted before walking)", flush=True)
        sys.exit(0)

    print(
        f"Moving forward (vx={args.forward_speed}) for {args.forward_duration}s...",
        flush=True,
    )
    # continous_move=Trueで指定秒数の間、速度指令を維持し続ける(内部的にはSetVelocityの
    # durationを長く取るだけ。オンボードのバランス制御自体は常時動いているので、
    # walk_forward_real.pyのような外部からの継続送信スレッドは不要)。
    sport_client.Move(args.forward_speed, 0.0, 0.0, continous_move=True)
    time.sleep(args.forward_duration)

    print("Stopping...", flush=True)
    sport_client.StopMove()
    time.sleep(1.0)

    print(
        "\n!!! Damp()すると関節が脱力します。支える準備ができてから Enter を押してください。 !!!",
        flush=True,
    )
    input("支える準備ができたら Enter: ")

    print("Damping (脱力します)...", flush=True)
    sport_client.Damp()
    # 高レベルモードを解放し、他の制御方式(walk_forward_real.py等)と衝突しない
    # 中立状態に戻す。
    msc.ReleaseMode()
    print("DONE", flush=True)


if __name__ == "__main__":
    main()
