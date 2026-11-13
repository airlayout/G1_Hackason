# Entame / real

実機G1上での実行コードをここに置く。

## `arm_wave_real.py`（SDK方式・実機確認済み）

`../README.md`の「実現方法の選択」でいうSDK方式（`sport_mode`）の実例。
`G1ArmActionClient.ExecuteAction()`でUnitree標準の定型モーション（"high wave"等）を
呼び出す。実機で"high wave"の動作を確認済み（`LocoClient.WaveHand()`はcode=0でも
実際には動かないという既知の落とし穴があるので、こちらを使うこと。詳細は
`FAILURES.md`とスクリプト冒頭のコメントを参照）。

```bash
python Entame/real/arm_wave_real.py --network-interface enp3s0
```

lerobot方式（独自振り付け）はまだ未着手。

## 音声ツール用の `arm_action_tool.py`

[Voice の手振り手順](../../Voice/README.md)から呼ぶ、非対話 CLI。
`--execute` を付けた場合だけ `G1ArmActionClient.ExecuteAction(26)` を呼ぶ。
登録済みの `high wave` のみ対応し、現在の FSM を読み取って SDK の対応モードを確認する。
`MotionSwitcherClient` によるモード変更・解除や起立・歩行指示は行わない。
動作 ID は [公式 Python SDK](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/unitree_sdk2py/g1/arm/g1_arm_action_client.py)、
FSM の制約は [公式 ROS 2 クライアント](https://github.com/unitreerobotics/unitree_ros2/blob/master/example/src/include/g1/g1_arm_action_client.hpp)を参照。

```bash
# 既定は dry-run。SDK のインストールも DDS 接続も不要。
python3 Entame/real/arm_action_tool.py --preset "high wave"
```

実行前に機体を停止し、巡回・歩行・他の腕制御を終了する。
RPC 受理後の 5 秒間はプロセス間ロックを保持するが、物理動作の完了通知ではない。
終了コード 0 と JSON の `accepted` は SDK に要求が受理されたことを表す。
音声経由の実機試験は未実施。
