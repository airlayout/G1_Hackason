# SimpleWalk（前進歩行）

G1にキーボード/スクリプトから前進コマンドを送って歩かせる、最初に動作確認できた機能。
`Perception/`・`Mapping/`・`SLAM/`と同じ「シムで作る→実機で動かす」の型のテンプレート元。

## 構成

- `sim/` — シミュレーション（MuJoCo, `lerobot/unitree-g1-mujoco`）上での検証
  - `verify_g1_sim_command.py` — remote入力がコントローラに反映されるか検証
  - `patch_mujoco_elastic_band.py` — MuJoCo環境のelastic bandバグへのパッチ
  - `release_band_and_walk_forward.py` — elastic band解除→前進歩行の検証
- `real/` — 実機G1での実行（2方式あり）
  - `walk_forward_real.py` — lerobot + 独自ONNXポリシー(GrootLocomotionController)による
    低レベル関節制御方式。`run_g1_server.py`(G1本体側のDDS-ZMQブリッジ)経由。
    3段階の安全確認ゲート付き。
  - `walk_forward_real_sdk.py` — Unitree SDK標準の高レベル歩行(sport_mode,
    `LocoClient`)をそのまま使う方式。lerobot/torch/onnxruntime不要、
    `run_g1_server.py`も不要（操作PCから直接DDS接続）。同じく3段階の安全確認ゲート付き。
    **両方式は同時には使えない**（片方が高レベルモードを占有する）。

## 環境

`G1_HuggingFace/venv/`（操作PC側）を使う。`walk_forward_real.py`（lerobot方式）は
さらにG1本体側のPython 3.12 conda環境（`lerobot`）と`run_g1_server.py`の起動が必要。
`walk_forward_real_sdk.py`（SDK方式）はG1本体側の追加セットアップ不要
（`unitree_sdk2py`が既にDDSでロボットと直接通信するため）。
セットアップ手順・動作確認結果は`G1_HuggingFace/README.md`と`G1_HuggingFace/Note`を参照。
ネットワーク接続・疎通確認は`Common/network/`を参照。

## 状態

- lerobot方式（`walk_forward_real.py`）: シミュレーション・実機ともに前進歩行の動作確認済み
- SDK方式（`walk_forward_real_sdk.py`）: 未検証（実機での動作確認はまだ行っていない）

失敗した内容は`FAILURES.md`に記録している（実機で歩行後にdisconnect()で脱力し
転倒した件など）。
