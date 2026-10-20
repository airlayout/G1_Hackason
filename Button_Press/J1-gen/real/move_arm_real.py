"""実機の腕を、指定の関節角へ動かして戻す（ラボ PC から DDS で直接送る）。

    # まず dry-run（既定）。lowstate を読んで計算・表示するだけで、何も送らない
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path arm_sdk

    # 実際に送る（確認モードで各段階で Enter を待つ）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path arm_sdk --execute

⚠️ 実機で動かす前に:
- 人がリモコンを持ち、緊急時は L2+B（ダンピング）で止める
- 腕モータを有効にする（リモコンでダンピング = FSM 1。ゼロトルクのままだと送信しても動かない）
- プランB（--path lowcmd）はデバッグモード（内蔵コントローラ停止）で使う。座った状態か吊り下げで行う
- 既定の動きは小さい（肩ピッチ -5°、肘 +5°）。大きく動かすのは小さい動きで確認してから
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.arm.run_move import add_common_args, run  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    add_common_args(p, default_delta=[-5.0, 0.0, 0.0, 5.0, 0.0, 0.0, 0.0])
    p.add_argument("--path", choices=["arm_sdk", "lowcmd"], required=True,
                   help="arm_sdk（プランA）か lowcmd（プランB: デバッグモード）")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は設定ファイル）")
    p.add_argument("--execute", action="store_true", help="実際に送信する（付けなければ dry-run）")
    p.add_argument("--no-confirm", action="store_true", help="各段階の Enter 待ちを省く（--execute 時の既定は確認あり）")
    args = p.parse_args()

    overrides = {}
    if args.network_interface:
        overrides["network_interface"] = args.network_interface
    if args.execute:
        print("[real] ⚠️ 実機に送信する。リモコンを持った人が立ち会っていること（緊急停止は L2+B）")
    else:
        print("[real] dry-run: lowstate を読んで計算・表示するだけで、送信しない（送るには --execute）")
    confirm = args.execute and not args.no_confirm
    return run(args, path=args.path, dry_run=not args.execute, confirm=confirm, overrides=overrides)


if __name__ == "__main__":
    sys.exit(main())
