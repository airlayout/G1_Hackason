# Mapping（G1によるMap計測・作成）

G1に搭載されたセンサー（LiDAR/カメラ等）を使って周囲の地図を計測・作成する機能。

## 構成

- `sim/` — シミュレーション上での地図計測ロジックの検証
- `real/` — 実機G1のセンサーから地図を作成する処理

## 環境

`G1_HuggingFace/venv/`（操作PC側）・G1本体側のPython 3.12 conda環境（`lerobot`）を
共通で使う想定。ネットワーク接続・疎通確認は`Common/network/`を参照。

## 注意

`IsaacSim_Env/`に2D LiDAR + SLAM/Nav2、`SimEnv3D/`に3D LiDAR + octomapの実装が
既にあるが、いずれも当面使わない環境という位置づけ。本機能は
`G1_HuggingFace/`と同じMuJoCo/lerobotスタックを前提に進める。
（MuJoCo環境側にLiDAR相当のセンサーが無い場合は、対応方法をここで検討する）

## 進め方

1. `sim/`でロジックを作り、シミュレーション上で動作確認
2. `real/`で実機センサーに接続し、同じロジックが実機でも動くか確認

失敗した内容は`FAILURES.md`に記録する。

## 状態

未着手。
