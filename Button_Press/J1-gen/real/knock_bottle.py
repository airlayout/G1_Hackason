"""ボトルを倒す（実機）: 検出 → 目標 → IK → 手前の姿勢 → 胴の上のほうを前方へ深く押す → 戻る。ラボ PC で実行する。

    # dry-run（既定）: 検出・計画・指令の計算まで行い、送信しない。記録は残す
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/knock_bottle.py --path arm_sdk --gravity-scale 0.5
    # 実際に送る（確認モードで各段階 Enter）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/knock_bottle.py --path arm_sdk --gravity-scale 0.5 --execute
    # 深度が使えないとき: 教えた姿勢の中指の先 + 定規で測ったずれを対象にする（押す高さは --target-offset で上げない）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/knock_bottle.py --path arm_sdk --target manual --taught-pose touch_bottle --offset 0 0 0.058

real/press_bottle.py と同じ流れ・同じ引数。違うのは configs/knock.yaml の設定（深く押す・ボトルの上のほうを押す）で、
引数 --depth-mm（押し込み）、--target-offset X Y Z（押す点の上げ）、--speed-mm-s で上書きできる。
ボトルは机の上に置く。滑り止めのマットの上だと倒れやすい（sim/knock_sweep.py）。
倒れたかは、頭カメラの深度でボトルのいちばん高い点の高さを押す前と比べて自動で判定する（common/tip_check.py）。
終了コード: 0 倒れた / 20 倒れなかった / 21 判定できない（dry-run を除く。その他は press_bottle.py と同じ）。
ボトルは机の奥へ倒れる。腕は押し込みのあと机の上で止まる。人が離れていること。

⚠️ 実機で送る前に: 人がリモコンを持つ（緊急時は L2+B）。検出するときは腕をカメラの視野から外す。
   腕を伸ばしきる動きなので、最初は必ず dry-run で「届く」ことを確かめる。
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parent))
sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.tip_check import EXIT_NOT_TIPPED, EXIT_TIP_UNKNOWN  # noqa: E402
from press_bottle import main  # noqa: E402

OVERLAY = "knock.yaml"

if __name__ == "__main__":
    code = main(default_overlay=OVERLAY, description=__doc__, tip_check=True)
    if code == EXIT_NOT_TIPPED:
        print("[knock_bottle] 倒れなかった。押す点の上げや押し込みを調整する（real/KNOCK_PROCEDURE.md の表）")
    elif code == EXIT_TIP_UNKNOWN:
        print("[knock_bottle] 倒れたかを判定できなかった。目で確認する（dry-run、ボトルが見つからない・深度が取れない場合を含む）")
    sys.exit(code)
