# SLAM（G1内SLAM機能の実装）

G1上でSLAM（自己位置推定＋地図作成）を行う機能。`Mapping/`で作成した地図を使った
自律ナビゲーションもここに含む想定（役割分担は`Mapping/`と要調整）。

## 構成

- `sim/` — シミュレーション上でのSLAMロジックの検証
- `real/` — 実機G1上でのSLAM実行

## 環境

`G1_HuggingFace/venv/`（操作PC側）・G1本体側のPython 3.12 conda環境（`lerobot`）を
共通で使う想定。ネットワーク接続・疎通確認は`Common/network/`を参照。

## 注意

`IsaacSim_Env/`に2D SLAM/Nav2の実装が既にあるが、当面使わない環境という位置づけ。
本機能は`G1_HuggingFace/`と同じMuJoCo/lerobotスタックを前提に進める。

## 進め方

1. `sim/`でロジックを作り、シミュレーション上で動作確認
2. `real/`で実機G1上で動くか確認

失敗した内容は`FAILURES.md`に記録する。

## 状態

未着手。
