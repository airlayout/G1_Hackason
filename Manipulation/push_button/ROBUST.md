# MuJoCo事前探索と停止なしの押下

`real/run_push.py --task button|bottle --motion-planner robust` で選択する実験方式。
省略時は従来方式で、既存のfield設定・4段階の再計測は変更しない。
実機での動作試験は実施していない。

2026-11-01の現場準備記録は [現場ログ](LOG/robust-lab-20261101/README.md) を参照。
現場後の相対移動フィードバックは開発途中のため、`--bottle-feedback relative` の明示指定でのみ使う。
標準の `off` は基準軌道を使い、現場計測時の接近点・時間配分に戻している。

標準設定は、**探索予算は最大10秒・1候補あたり3条件**。
認識どおりの位置、押す方向の手前3mm、奥3mmで接近・押下・退避・homeまで評価する。
左右・上下の位置は変えず、2・3条件目の重さ・摩擦・関節誤差・ボタンの向きは従来のseedで生成する。
予算内に全条件を完走して成功した最初の候補を採用し、その時点で探索を終える。
プロセス数は設定値と条件数の小さい方を使う。最大試行数1000は上限として残す。
校正JSONの不確かさは変更せず、計画の確認幅3mmとは別に記録する。
3条件での合格は、校正の不確かさ15mmや任意の左右・上下ずれへの対応を保証しない。
保存現場シーンの比較と6ケースの実行結果は [3条件化の検証記録](reports/ROBUST_THREE_CONDITIONS_20261101.md) を参照。

[10秒・30秒の比較記録](reports/ROBUST_BUDGET_30S_20261101.md)では、検証を通過した候補は
1本から2～4本へ増えたが、ボトル1条件で画像補正の上限に達して復帰前に停止した。
動作完了は10秒で6/6、30秒で5/6だった。これは変更前の20条件方式の記録。

初回の新しいRGB-D 3枚と撮影時の関節角・IMUから `SceneSnapshot` を作る。
ボタン・ボトルとも画像の経過時間による棄却は行わない。画像ID・撮影時刻の順序と
3枚の位置の安定性は確認する。撮影時刻の姿勢履歴がない場合は最も近い姿勢を使う。
最後の計測は期限切れにならないため、画像の配信停止や遅延中も古い位置で動作しうる。
右腕のIK初期値と中継点を変えた経路を、描画しないMuJoCoで誤差条件ごとに
接近・押下・退避・homeまで計算する。全条件を通過した最初の候補の
`PlanResult` の軌道を50Hzでストリーム実行する。実行中は撮影を待たず、完成した
最新の3フレーム窓による計測を読む。ボトルの標準動作では画像による経路補正を加えない。
ボタンは小さな横補正を行い、手前5mm以降の横補正を固定する。
押下後の画像による結果確認は別に行う。

指先と選択した対象の接触は、接近・復帰を含む動作全体で許可する。
自己衝突・対象以外への衝突・指先以外の対象への衝突は許さず、障害物余裕は3mmを維持する。
`--stop-before-contact` の診断では往復とも接触を許可しない。
対象への接触余裕は実形状から求める参考値であり、採否・スコアには使わない。
条件に位置ずれを反映した後で校正の不確かさを再度差し引く計算は使わない。

## 動作時間と右腕の制限

robust方式は上下ボタン・ボトルとも、**全動作30秒を目標、許容差±3秒**にする。
計時は腕の制御開始から、接近・押下・home復帰・制御解除まで。
経路探索、初回認識、YES入力待ち、解除後の画像による結果確認は含めない。
開始2.02秒、home確認0.36秒、解除2.02秒を予約し、通常は軌道に25.6秒を割り当てる。
シムも開始・解除の時間中に物理を進め、開始中のRGB-D監視を継続する。
重み切替中の実機のサーボ特性を再現するものではない。

| 引数 | 初期値 | 意味 |
|---|---:|---|
| `--motion-duration-s` | 30 | 開始・home確認・解除を含む目標動作時間[s] |
| `--joint-speed-rad-s` | 1.2 | 右腕関節の指令角速度上限[rad/s] |
| `--joint-acceleration-rad-s2` | 4 | 右腕関節の指令角加速度上限[rad/s²] |

これらはハードウェアの定格ではなく、ソフトウェアの指令上限。
速度は1.2rad/s、加速度は4rad/s²までの正の値を設定できる。
計画軌道は上限の80%（既定で0.96rad/s・3.2rad/s²）以内とし、20%を補正用に残す。
スプラインの区間内部も含めて極値を調べ、必要なら許容差内で時間を延長する。
目標時間+3秒を超える候補は採用しない。小さい制限での比較では目標時間も延ばす必要がある。

ボトルは滑り接触を保つ押す区間の時間を確保し、接近・復帰の区間時間を
関節速度・加速度の制約下で最適化する。5cm押しの検証では押す区間に約17.6秒を使い、
開始・解除を含む全動作は32.96〜33.00秒。上下ボタンは30.00秒。
時間変更後に50Hzの軌道と速度補償トルクを生成し、その軌道全体を物理評価する。
計画・実行フィルター・実機の動作クラスで同じ制限を使う。
SDKブリッジには制御開始前に設定を送り、受信時の速度検査も同じ値へ更新するため、
SDK側にも更新した`arm_bridge.py`と共通の`motion_limits.py`が必要。
旧ブリッジが設定を確認できない場合は腕の制御開始前に停止する。

JSONには軌道の予測時間・速度/加速度上限使用率と、実行時の`enable_s`、`trajectory_s`、
`home_confirmation_s`、`release_s`、`motion_s`、指令速度/加速度の最大値を記録する。
`execution.duration_s`も解除完了までを含む定義へ統一した。
従来方式の0.8rad/s・2rad/s²、脚・IMUの監視、追従・接触・トルクの判定は維持する。

検証結果は [約30秒化の記録](reports/MOTION_30S_20261101.md) を参照。

## ボトルのBモデル

ボトルの円筒の位置・半径・下端を初回RGB-Dから推定する。台の面・二辺・輪郭は測らない。
底の高さが決められない場合は計画を拒否する。`--bottle-base-height` で床からの高さ[m]を
明示指定できるが、ボトル胴体の有効な深度は引き続き必要。
高さを指定した場合は、実測した底面と接触高さから胴体の範囲を選び、
斜めの検出枠に混ざる台面や肩の絞り部を円筒の推定から除く。
全高には推定円筒の内側にある細い首・キャップの深度も含める。

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

`--fault target-loss|state-disconnect` で動作開始後の異常停止を試せる。
`--fault stale-images` は画像時刻の遅延を模擬するが、経過時間だけでは停止しない。
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
| `--planning-scenarios` | 3 | 全条件の完走・成功が必要。基準・手前・奥の順。4条件目以降は確認幅内の各軸乱数 |
| `--planning-position-span-m` | 0.003 | 通常探索の位置ずれ幅[m]。2・3条件目は押す方向の−幅・＋幅 |
| `--planning-workers` | 8 | 物理計算のCPUプロセス数の上限。実際は条件数以下 |
| `--bottle-feedback` | off | off: 基準軌道、relative: 開発途中の相対移動補正 |
| `--planning-max-candidates` | 50 | 経路候補数の上限 |
| `--planning-seed` | 0 | 探索の再現用seed |
| `--position-error-m` | 0.001 | 追加評価で校正の不確かさに加える誤差、および相対補正の画像ノイズ幅[m] |
| `--normal-error-deg` | 1 | ボタンパネルの向きの誤差範囲[deg] |
| `--tracking-error-rad` | 0.001 | 初期角と指令の偏りの範囲[rad] |
| `--mass-range` | 0.28 0.42 | ボトル質量の仮定範囲[kg] |
| `--friction-range` | 0.24 0.36 | ボトル・支持面の摩擦係数の仮定範囲 |
| `--gravity-scale` | 1 | 右腕の公称重力・速度補償倍率 |

質量・摩擦は画像から測定した値ではなく、設定した仮定の範囲。
物理刻み2ms、制御周期20ms、関節指令の上限は既定1.2rad/s・4rad/s²を使う。
ボタンの押下指令には従来と同じ追従補償上限を使い、シムで過押し量と40Nの力上限も検査する。
これは実機での接触力の測定・制御を意味しない。

各誤差条件で**全軌道の評価を終えたもの**が `completed_rollouts`。
衝突等で早期中止したものと、期限切れのものは `aborted_rollouts` に理由を残す。
候補の全条件が通過しなければ、その候補を採用しない。採用候補がなければ腕を動かさない。
候補を静的検査した回数は別計数であり、「1000回」に足さない。
期限は物理計算を短いブロックに区切って確認するため、終了処理等で小さな超過は起こり得る。
実測時間と超過を記録し、未検証の候補を埋め合わせに採用しない。
追加の `--holdout-scenarios` は通常探索とは独立したseedで、校正の不確かさを含む
従来の全方向の誤差範囲を評価する。合否は別に記録し、3条件の採用判定へ混ぜない。

初回データに「計画後0.5秒で失効」は適用しない。計画中は画像と実測状態を監視し、
腕関節が0.01rad、IMUが0.5度、対象位置が設定の不確かさを超えて変われば破棄する。
画像の経過時間は検査せず、実行時の機体状態の0.1秒の鮮度検査は維持する。
実機の5Hz配信が処理周期よりわずかに早く届いた場合は、次の処理時刻まで
最新の未処理画像を保持する。撮影時刻は変更せず、処理前にも0.5秒以内を確認する。
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
既存の試行JSONに `planning_scenarios: 20` があれば、その明示値は維持される。
新しい標準を使う場合は3へ変更するか、CLIで `--planning-scenarios 3` を指定する。
校正の不確かさを小さく書き換えて採用させる必要はない。
従来field用JSONには異なる設定項目があるため、そのまま流用せずrobust用の設定を用意する。
記録には観測・校正、全候補の評価と棄却理由、採用軌道、実際の指令・実測角、補正、
画像による成否確認、復帰と解除の結果を分けて保存する。過去の試行JSONは上書きしない。
