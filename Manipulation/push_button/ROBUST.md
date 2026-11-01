# MuJoCo事前探索と停止なしの押下

`real/run_push.py --task button|bottle --motion-planner robust` で選択する実験方式。
省略時は従来方式で、既存のfield設定・4段階の再計測は変更しない。
実機での動作試験は実施していない。

2026-11-01時点の標準設定は、**探索予算10秒・1候補あたり20種類の誤差条件**。
現環境で約10秒に20回の全軌道評価を完了した性能を基準にする。
20回は20本の異なる経路ではなく、1本の経路を20条件で試す回数。
予算内で全条件を通過した候補だけを採用し、完了しなければ実行しない。
1000回/10秒は現時点の運用条件にはしない。設定上の最大試行数1000は上限として残すが、
終了は10秒の時間予算を優先する。

[10秒・30秒の比較記録](reports/ROBUST_BUDGET_30S_20261101.md)では、検証を通過した候補は
1本から2～4本へ増えたが、ボトル1条件で画像補正の上限に達して復帰前に停止した。
動作完了は10秒で6/6、30秒で5/6だったため、標準設定は10秒・20条件を維持する。

初回の新しいRGB-D 3枚と撮影時の関節角・IMUから `SceneSnapshot` を作る。
右腕のIK初期値と中継点を変えた経路を、描画しないMuJoCoで誤差条件ごとに
接近・押下・退避・homeまで計算する。全条件を通過した候補同士の最小余裕を比較し、
`PlanResult` の軌道を50Hzでストリーム実行する。実行中は撮影を待たず、完成した
最新の3フレーム窓による計測を読み、小さな横方向の補正だけを加える。
ノイズ幅内の変化では目標を変えず、ボタンは手前5mm以降の横補正を固定する。
押下後の画像による結果確認は別に行う。

## ボトルのBモデル

ボトルの円筒の位置・半径・下端を初回RGB-Dから推定する。台の面・二辺・輪郭は測らない。
底の高さが決められない場合は計画を拒否する。`--bottle-base-height` で床からの高さ[m]を
明示指定できるが、ボトル胴体の有効な深度は引き続き必要。

初回のボトルの手前端を `x_wall`、底面高さを `z_bottom` とし、
`x >= x_wall AND z <= z_bottom` を腕の侵入禁止領域にする。
壁手前では底より低い腕を許す。壁は押下後も初回位置に固定し、ボトルには追従させない。
MuJoCoでは腕の作業範囲を覆う箱でこの領域を表し、上面でボトルを支持する。
ボタンは計測したパネル法線に合わせた壁と可動ボタンを使い、手先と対象ボタン以外の
腕接触を許さない。

実物の台は仮想壁より手前へ張り出さないように配置し、押す先まで支持面を確保する。
この方式は台の端からの落下、台上の別の物、PETの変形を予測しない。
`--table-height`、`--table-bounds`、`--track-table` はrobust設定には使用しない。

## シミュレーション

リポジトリ直下で実行する。既存のMuJoCo・NumPy・SciPy環境とMenagerieを使う。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 \
  _local/button_press_yada/venv/bin/python Manipulation/push_button/sim/run_push_mujoco.py \
  --task bottle --motion-planner robust \
  --report-out _local/robust_push/bottle.json \
  --video-out _local/robust_push/bottle.webm

# 上ボタン。下は --button-direction down を追加する。
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 \
  _local/button_press_yada/venv/bin/python Manipulation/push_button/sim/run_push_mujoco.py \
  --task button --motion-planner robust \
  --report-out _local/robust_push/button.json
```

標準の検出枠はセグメンテーション由来の **oracle bbox**。3D位置はRGB-Dから推定する。
`--weights` でYOLOを選べる。oracleとYOLOの結果は分けて扱う。
この新規シムは骨盤固定で、自由立位のバランスの検証ではない。
カメラは固定し、対象の真値を追尾しない。物体真値は描画・評価のみに使う。
5Hzのカメラサンプリングを模擬するが、画像処理の実時間遅延は物理時計へ注入していない。
制御と画像処理の実時間は `total_wall_s` に含み、探索の実時間と区別する。

探索だけの測定と、選択に使っていないseedでの物理評価:

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 \
  _local/button_press_yada/venv/bin/python Manipulation/push_button/sim/run_robust_push_mujoco.py \
  --task bottle --benchmark-only --holdout-scenarios 10 \
  --report-out _local/robust_push/benchmark.json
```

`--fault target-loss|stale-images|state-disconnect` で動作開始後の異常停止を試せる。
`--offset X Y Z`、`--calibration-offset X Y Z`、`--sensor-noise-m`、`--evaluation-seed` は
物理配置・観測誤差の比較用。探索用の誤差seedとは独立している。

新旧比較をまとめて実行する場合:

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=1 \
  _local/button_press_yada/venv/bin/python Manipulation/push_button/sim/run_robust_suite.py \
  --out _local/robust_push/comparison --seeds 10000 10001 --video
```

上下ボタン・ボトルに対し、同じ配置、校正ずれ、深度ノイズで新旧を実行する。
比較のstaged側は既存の `execute_alignment` / `BottlePushRoutine` を使い、
ボトルの台輪郭の検査だけBモデルへ置き換える。従来の台認識を含めた比較ではない。
`summary.json` は失敗ケースも除外せず、成否、停止回数、探索・全体の実時間、物理結果を残す。

## 探索設定と計測の有効性

| 引数 | 初期値 | 意味 |
|---|---:|---|
| `--planning-budget-s` | 10 | 環境構築からの探索時間予算[s] |
| `--planning-max-rollouts` | 1000 | 物理試行の最大開始数。目標値であり完了を保証しない |
| `--planning-scenarios` | 20 | 採用に必要な誤差条件数。1条件目は公称値 |
| `--planning-workers` | 8 | 物理計算のCPUプロセス数 |
| `--planning-max-candidates` | 50 | 経路候補数の上限 |
| `--planning-seed` | 0 | 探索の再現用seed |
| `--position-error-m` | 0.001 | 校正の位置不確かさに加える各軸の誤差範囲[m] |
| `--normal-error-deg` | 1 | ボタンパネルの向きの誤差範囲[deg] |
| `--tracking-error-rad` | 0.001 | 初期角と指令の偏りの範囲[rad] |
| `--mass-range` | 0.28 0.42 | ボトル質量の仮定範囲[kg] |
| `--friction-range` | 0.24 0.36 | ボトル・支持面の摩擦係数の仮定範囲 |
| `--gravity-scale` | 1 | 右腕の公称重力・速度補償倍率 |

質量・摩擦は画像から測定した値ではなく、設定した仮定の範囲。
物理刻み2ms、制御周期20ms、関節指令0.8rad/s・2rad/s²を使う。
ボタンの押下指令には従来と同じ追従補償上限を使い、シムで過押し量と40Nの力上限も検査する。
これは実機での接触力の測定・制御を意味しない。

各誤差条件で**全軌道の評価を終えたもの**が `completed_rollouts`。
衝突等で早期中止したものと、期限切れのものは `aborted_rollouts` に理由を残す。
候補の全条件が通過しなければ、その候補を採用しない。採用候補がなければ腕を動かさない。
候補を静的検査した回数は別計数であり、「1000回」に足さない。
期限は物理計算を短いブロックに区切って確認するため、終了処理等で小さな超過は起こり得る。
実測時間と超過を記録し、未検証の候補を埋め合わせに採用しない。

初回データに「計画後0.5秒で失効」は適用しない。計画中は画像と実測状態を監視し、
腕関節が0.01rad、IMUが0.5度、対象位置が設定の不確かさを超えて変われば破棄する。
取得時の画像の0.5秒、実行時の状態の0.1秒の鮮度検査は維持する。
静止中の検出できない機体並進は校正・配置の前提に依存する。

## 実機の計画表示

```bash
OPENBLAS_NUM_THREADS=1 _local/button_press_yada/venv/bin/python \
  Manipulation/push_button/real/run_push.py --task bottle --motion-planner robust \
  --model _local/bottle_push/menagerie/unitree_g1/g1.xml \
  --base-transform /path/to/calibration.json --state-json /path/to/state.json \
  --weights /path/to/bottle.pt --output _local/robust_push/preview.json
```

`--execute` がなければSDKへ接続しない。実行する場合も既存と同様に
`--execute --calibrated` とYES入力が必要。Navigationの移動指令は先に停止する。
ボタンも同じ入口で `--task button --weights /path/to/button.pt --stroke 実測値` を指定する。
robust専用の試行JSON例は `real/robust_trial.example.json`。CLIがJSONより優先する。
従来field用JSONには異なる設定項目があるため、そのまま流用せずrobust用の設定を用意する。
記録には観測・校正、全候補の評価と棄却理由、採用軌道、実際の指令・実測角、補正、
画像による成否確認、復帰と解除の結果を分けて保存する。過去の試行JSONは上書きしない。
