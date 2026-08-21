#!/usr/bin/env python
"""実機G1に接続し、前進コマンド(remote.ly)を送って歩かせるスクリプト。

SimpleWalk/sim/release_band_and_walk_forward.py の「[実機でも共通]」部分
（robot.connect()/disconnect() と、send_action({"remote.ly": ...}) による
コマンド送信ループ)だけを残し、「[シミュレーション専用]」部分
（elastic band解除、mujoco内部状態(pelvis座標)の直接読み取り)を取り除いたもの。
歩行結果はpelvis座標のようなものでは確認できないため、目視で確認すること。

安全のため3段階の確認を挟む:
  1. connect()前(ロボットがまだ外部制御下にない段階) — 転倒防止の準備確認。
  2. connect()完了後(ロボットが標準立位姿勢へ遷移し、GrootLocomotionController
     による外部制御下でバランスを取り始めた段階) — ここで一度停止し、
     安定して自立していることを目視確認してから前進コマンドを送るか判断する。
     robot.connect() は内部の reset() が完了するまでブロックするため、
     connect() が返った時点で既に姿勢遷移と制御スレッド起動は完了している。
  3. 前進コマンド終了後 — disconnect()は実機の場合、関節をゼロトルク(脱力)に
     する仕様(unitree_g1.py の _send_zero_torque())。支えが無い状態で脱力すると
     そのまま倒れるため、disconnect()するまでは hold_standing_until() で
     待機コマンドを送り続けて直立姿勢を保持し、人間が支える準備を確認してから
     脱力させる。

前提（安全上必須）:
  - G1本体の電源が入り、転倒しないよう安全確保された状態であること
    （人間が支える／自立スタンドを使う。実機には elastic band に相当する
    ものが無いため、代わりにこれが必須)。
  - Unitree公式アプリでオンボードの高レベル制御(sport_mode)をオフにしておくこと
    (オンにしたままだと外部からのrt/lowcmdと衝突する)。
  - G1側で run_g1_server.py が起動済みであること:
      python src/lerobot/robots/unitree_g1/run_g1_server.py --camera
  - 操作側PCが G1 と同一サブネット(192.168.123.x, x != 164)の
    static IP を持ち、Ethernetで直結されていること。

使い方:
  python SimpleWalk/real/walk_forward_real.py --robot-ip 192.168.123.164
"""
import argparse
import os
import sys
import threading
import time

from lerobot.robots.unitree_g1 import UnitreeG1, UnitreeG1Config
from lerobot.robots.unitree_g1.g1_utils import default_remote_input


def run(robot, action_overrides, duration_s, control_hz=50):
    """action_overrides で指定した remote.* コマンド(例: {"remote.ly": 0.3})を
    duration_s 秒間送り続ける。[実機でも共通]"""
    action = default_remote_input()
    action.update(action_overrides)
    dt = 1.0 / control_hz
    for _ in range(int(duration_s * control_hz)):
        t0 = time.perf_counter()
        robot.send_action(dict(action))
        elapsed = time.perf_counter() - t0
        if elapsed < dt:
            time.sleep(dt - elapsed)


def hold_standing_until(robot, stop_event, control_hz=50):
    """stop_event がセットされるまで remote.* をゼロ(=待機)で送り続け、
    GrootLocomotionController に直立姿勢を保持させ続ける。[実機でも共通]

    disconnect() は関節をゼロトルク(脱力)にするため、脱力させてよいと
    人間が確認するまでは、この関数でバランス制御を止めないようにする。
    """
    action = default_remote_input()
    dt = 1.0 / control_hz
    while not stop_event.is_set():
        t0 = time.perf_counter()
        robot.send_action(dict(action))
        elapsed = time.perf_counter() - t0
        if elapsed < dt:
            time.sleep(dt - elapsed)


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--robot-ip", default="192.168.123.164", help="G1のIPアドレス")
    parser.add_argument("--forward-duration", type=float, default=5.0, help="前進コマンドを送る秒数")
    # sim検証時の0.5より控えめな値をデフォルトにしている(実機での初回動作確認のため)。
    parser.add_argument("--forward-speed", type=float, default=0.3, help="remote.ly の値(前進速度指令)")
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

    cfg = UnitreeG1Config(
        is_simulation=False,
        robot_ip=args.robot_ip,
        controller="GrootLocomotionController",
    )
    robot = UnitreeG1(cfg)
    print(f"Connecting to G1 at {args.robot_ip}...", flush=True)  # [実機でも共通]
    robot.connect()
    # connect() はブロッキングであり、戻ってきた時点で reset() による標準立位姿勢への
    # 3秒補間と、GrootLocomotionController の制御スレッド起動は完了している。
    # つまりここが「制御がこちら側(コントローラ)に渡った」瞬間。
    print("Connected. ロボットは標準立位姿勢へ遷移し、外部制御下でバランス保持を開始しました。", flush=True)
    time.sleep(1.0)  # 遷移直後の揺れが収まるまでの猶予

    print(
        "\n!!! ここで一度停止します。ロボットが安定して自立していることを目視で確認してください。 !!!",
        flush=True,
    )
    if input("前進コマンドを送信して良ければ y、中断するなら他のキーを入力: ").strip().lower() != "y":
        print("前進コマンドを中断しました。安全に切断します(ロボットは脱力状態になります)。", flush=True)
        try:
            robot.disconnect()
        except Exception as e:  # noqa: BLE001
            print(f"disconnect() error (ignored): {e}", flush=True)
        print("DONE (aborted before walking)", flush=True)
        sys.stdout.flush()
        sys.stderr.flush()
        os._exit(0)

    print(
        f"Sending forward command (remote.ly={args.forward_speed}) for {args.forward_duration}s...",
        flush=True,
    )
    run(robot, {"remote.ly": args.forward_speed}, duration_s=args.forward_duration)

    print(
        "\n前進コマンド終了。ここからは待機コマンドを送り続け、その場で直立姿勢を"
        "保持します(脱力はまだしません)。",
        flush=True,
    )
    stop_holding = threading.Event()
    hold_thread = threading.Thread(target=hold_standing_until, args=(robot, stop_holding), daemon=True)
    hold_thread.start()

    print(
        "!!! disconnect()すると関節が脱力します。支える準備ができてから Enter を押してください。 !!!",
        flush=True,
    )
    input("支える準備ができたら Enter: ")

    stop_holding.set()
    hold_thread.join(timeout=2.0)

    print("Disconnecting (脱力します)...", flush=True)  # [実機でも共通]
    try:
        robot.disconnect()
    except Exception as e:  # noqa: BLE001
        print(f"disconnect() error (ignored): {e}", flush=True)
    print("DONE", flush=True)
    sys.stdout.flush()
    sys.stderr.flush()
    os._exit(0)


if __name__ == "__main__":
    main()
