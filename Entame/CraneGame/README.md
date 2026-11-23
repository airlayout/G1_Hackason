# CraneGame（クレーンゲーム）

G1 のエンタメ企画のうち、SO-101（小型ロボットアーム）で景品をつかむデモ。

## 中身

| 場所 | 内容 |
|---|---|
| [`index.html`](index.html) | 設計の軸と選択肢（初回方針、台の種類、比較案、評価観点、決定履歴）。ブラウザーで開く。 |
| [`so101_demo/`](so101_demo/README.md) | SO-101 の実機操作と、シミュレーション上のピック＆プレース。コードと試験は全部ここ。 |

## すぐ試す（シミュレーション）

```sh
cd Entame/CraneGame/so101_demo
source .venv/bin/activate      # 環境がなければ so101_demo/README.md の準備手順
python server.py
```

ブラウザーで開く:

- `http://localhost:8765/sim` … キーで操作
- `http://localhost:8765/sim/camera` … カメラの前の手で操作（左: カメラ、右: シミュレーション）

実機の操作画面や、準備・撮影の手順は [`so101_demo/README.md`](so101_demo/README.md) を参照。計画と進捗は [`so101_demo/CRANE_PLAN.md`](so101_demo/CRANE_PLAN.md)。

## 未確認のこと

- シミュレーションの手操作は、ブラウザーでの通し確認が残っている（試験は Python・Node とも通過）。
- 実機は、シミュレーションとは別の画面で動かす。シミュレーションの結果は実機では未確認。
