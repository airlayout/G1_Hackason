# SimpleWalk（前進歩行）

G1にキーボード/スクリプトから前進コマンドを送って歩かせる、最初に動作確認できた機能。
`Perception/`・`Mapping/`・`SLAM/`と同じ「シムで作る→実機で動かす」の型のテンプレート元。

## 構成

- `sim/` — シミュレーション（MuJoCo, `lerobot/unitree-g1-mujoco`）上での検証
  - `verify_g1_sim_command.py` — remote入力がコントローラに反映されるか検証
  - `patch_mujoco_elastic_band.py` — MuJoCo環境のelastic bandバグへのパッチ
  - `release_band_and_walk_forward.py` — elastic band解除→前進歩行の検証
- `real/` — 実機G1での実行
  - `walk_forward_real.py` — 実機向け前進歩行スクリプト（3段階の安全確認ゲート付き）

## 環境

`G1_HuggingFace/venv/`（操作PC側）・G1本体側のPython 3.12 conda環境（`lerobot`）を使う。
セットアップ手順・動作確認結果は`G1_HuggingFace/README.md`と`G1_HuggingFace/Note`を参照。
ネットワーク接続・疎通確認は`Common/network/`を参照。

## 状態

シミュレーション・実機ともに前進歩行の動作確認済み。

失敗した内容は`FAILURES.md`に記録している（実機で歩行後にdisconnect()で脱力し
転倒した件など）。
