# Perception（画像取得・認識）

G1のカメラ映像を取得し、認識処理（物体検出・セグメンテーション等、詳細は今後決定）を行う機能。

## 構成

- `sim/` — シミュレーション（`G1_HuggingFace/`と同じMuJoCo/lerobotスタック）上での
  カメラ取得・認識ロジックの検証
- `real/` — 実機G1のカメラ（`run_g1_server.py --camera`が配信するZMQストリーム、
  デフォルトポート5555）への接続・推論

## 環境

`G1_HuggingFace/venv/`（操作PC側）・G1本体側のPython 3.12 conda環境（`lerobot`）を
共通で使う想定。ネットワーク接続・疎通確認は`Common/network/`を参照。

## 進め方

`G1_HuggingFace/`で歩行機能を実装した際の流れ（シムで検証 → 実機で検証）を踏襲する:
1. `sim/`でロジックを作り、MuJoCo環境の`head_camera`で動作確認
2. `real/`で実機カメラに接続し、同じロジックが実機でも動くか確認

失敗した内容は`FAILURES.md`に記録する。

## 状態

未着手。
