# エレベーターのボタンの YOLO の重み

ラボ PC・メンバーに `git pull` だけで渡すために、リポジトリに入れた（2026-11-01）。学習データ・学習の途中の重みは `_local/`（git の対象外）のまま。

| ファイル | 中身 | 大きさ | sha256（先頭） |
|---|---|---|---|
| `button_final8.pt` | 最終版。8 本の動画（ラボのエレベーター）のフレーム 726 枚とシミュレーション 80 枚で学習（2026-10-31、11 エポック目）。クラスは `up` / `down` | 5.4 MB | `7795e946` |
| `button_final8.onnx` | 同じ重みを ONNX にしたもの（`real/export_button_onnx.py`、従来型の出力 `end2end=False`・opset 17）。torch・ultralytics なしで動く（G1 の PC2 用） | 9.8 MB | `d02a400a` |

元のファイル（ノート PC）: `_local/button_press/yolo_ft/runs/final8/weights/best.{pt,onnx}`。

## 使い方

`configs/elevator.yaml` の `yolo.weights` が `button_final8.pt` を指している（2026-11-01 から既定）。PC2 で動かすときは `.onnx` に書き換えるか、
評価のときだけなら環境変数で変える:

```bash
J1GEN_BUTTON_DETECTOR=yolo J1GEN_BUTTON_WEIGHTS=Button_Press/J1-gen/weights/button_final8.onnx <押すコマンド>
```

## 確かめたこと

- 学習・評価の詳細は [yolo_retrain_20261030.md](../docs/reports/yolo_retrain_20261030.md) の 11・12 章
  - シミュの確認用 30 枚: mAP50 0.995・mAP50-95 0.972。学習に使った 8 本の 746 枠で向きはすべて正しい（8 本とも学習に使ったので、未知の画像での正解率はこの重みでは測れない）
  - 評価環境（練習の 20 試行）: realistic 20/20・basic 20/20・模擬 G1（DDS）20/20
- ONNX（2026-11-01、検証用 30 枚＋ラボのエレベーターの写真 15 枚）:
  - .pt を同じ正方形（640×640）に入れた結果と **45/45 で一致**（確からしさの差 0.000）
  - 今の使い方（ultralytics は .pt を横長のまま入れる）との差: 個数と矢印が同じ 41/45。違った 4 枚は、ラボの写真 IMG_0924（消灯。大きさ違いの 3 枚）と
    シミュの 1 枚で、どれも ONNX が 1 つ少ない（.pt 4 個 / .onnx 3 個）。1 枚の時間はノート PC の CPU で .pt 238 ms・.onnx 96 ms
- **G1 の頭カメラ（実機）ではまだ確かめていない**
