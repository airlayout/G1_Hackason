"""腕の定型動作（敬礼・指さし）を実機または sim で動かす。

    # sim（MuJoCo）で動かす
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/run_action.py --action salute --path sim

    # 実機: まず dry-run（既定）。lowstate を読んで計算・表示するだけで、何も送らない
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/run_action.py --action salute --path arm_sdk --network-interface eth1

    # 実機に送る（確認モードで各段階で Enter を待つ。--execute は人が打つ）
    ... --path arm_sdk --network-interface eth1 --execute

動作: salute（敬礼・肘を横へ張る）/ salute_elbow_high（敬礼・肘を高く）/ salute_simple（敬礼の簡易版）/
      point（指さし・前下）/ point_level（指さし・水平）。定義は actions/poses.py。

腰の扱い（この動作の実行時だけ。configs/arm.yaml は変えない）:
- 腰が開始時から --waist-max-deg（既定 8°）ずれたら中止する。2026-11-13 の実機（腰の kp 60）で、point は 8°、point_level は 10° で最後まで動いた（kp 120 なら point_level も 8° で動いた）
- 腰を開始時の角度に保つ PD は既定 kp 120 / kd 2.5（arm.yaml の 60 / 1.5 より強い）。--waist-kp / --waist-kd で変えられる（上限 300 / 5）。
  kp 120 で point_level の腰 pitch の最大は +3.9〜4.1°（kp 60 では +9.2°）

⚠️ 実機で動かす前に:
- 人が G1 のそばでリモコンを持ち、緊急時は L2+B（ダンピング）で止める。腕を前へ出すと体が傾くことがあるので、横から支えられる位置に立つ
- 歩行コントローラは動かしたまま（rt/arm_sdk）。ReleaseMode は呼ばない。他の制御スクリプト（Entame の arm_wave_real.py など）は止めておく
- 腕モータを有効にする（リモコンでダンピング = FSM 1 を経由して立たせる。ゼロトルクのままだと送信しても動かない）
- 角度は計画値で、実機では未確認。最初は確認モードで 1 段ずつ進める
"""

from __future__ import annotations

import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from actions.runner import main  # noqa: E402

if __name__ == "__main__":
    sys.exit(main())
