# 外部アセット

`manifest.json`は取得先のcommit・個別ファイルのSHA-256を固定する。
`scripts/fetch_assets.py`で検証して`cache/`へ取得し、原本のURDFや重みは編集しない。
USDへの変換結果も`cache/usd/`へ保存する。

- 身体モデル: [unitree_ros](https://github.com/unitreerobotics/unitree_ros/tree/5994d4faef0a9cadd3287f8de0199a67eeb2a259/robots/g1_description)、`g1_29dof_rev_1_0.urdf`と参照mesh。上流の[LICENSE](https://github.com/unitreerobotics/unitree_ros/blob/5994d4faef0a9cadd3287f8de0199a67eeb2a259/LICENSE)を参照。
- 検証用ONNXとdeploy設定: [unitree_rl_lab](https://github.com/unitreerobotics/unitree_rl_lab/tree/4960b84732b0c2ec593dccbfe963fda1bcd7b1e3/deploy/robots/g1_29dof/config/policy/velocity/v0)。`--with-demo`指定時だけ取得する。上流READMEのライセンス表記はApache-2.0。

MID-360走査表は[Livox公式シミュレーター](https://github.com/Livox-SDK/livox_laser_simulation/tree/1cce1073633a062b92e30243a4c2920e45551bb5)の scan_mode/mid360.csv を使用する。MIT、著作権表示は licenses/livox_laser_simulation.txt。第1列はTime/sという見出しだが実値は1〜800000の点番号。点頻度200000/sを別に適用する。公開表は4秒分で、個体の取得時刻・ring・反射特性は含まない。既定では表の末尾で繰り返さず、4秒を超える収集を開始前に拒否する。

29関節の身体モデルは全コントローラーで共通。公開ONNXとその入力480次元・履歴5の仕様は、環境の必須条件ではない。

## Officeでの巡回収集

[NVIDIA公式Office](https://docs.isaacsim.omniverse.nvidia.com/6.2.0/assets/usd_assets_environments.html)を使う。
`configs/scenes/office.yaml`にIsaac Sim 6.2の取得先と入口USDのSHA-256を固定している。
`bash EndToEnd/prepare-office.sh`は、参照するUSD・画像・相対MDL moduleもローカルへ取得し、
`assets/cache/environments/isaac-6.2/office-assets.lock.json`に全ファイルのSHA-256を保存・再検証する。
アセットの権利は元のNVIDIA配布条件に従う。cacheのアセット原本を編集・Gitへ追加しない。

```bash
bash EndToEnd/prepare-office.sh
bash EndToEnd/patrol.sh --headless --device cuda:1 --steps 3000 --output EndToEnd/datasets/office-seed0
bash EndToEnd/check-dataset.sh EndToEnd/datasets/office-seed0/dataset.h5
```

巡回は高さ0mのフロアの最大連結領域で行う。物理collision付きの公式家具と同じシーン形状から
G1の半径を含む通行可能格子を作り、訪問数の少ない領域へA*経路を張って速度指令を出す。
経路には速度追従の余白も足す。公開v0歩行方策は並進ゼロの旋回にほぼ応答しないため、
教師はG1の前後・横移動を使って経路を追いながらheadingを合わせる。
公開ONNXがその速度指令を29関節の指令へ変換する。地図・真値姿勢・目標を使うのは収集用教師であり、
モデル入力はRGB、LiDAR、関節、骨盤・胸部・LiDAR位置IMUのまま。
開始場所・向きは`--seed`で再現できる。`--record-observer`で外部観測映像も保存できる。

描画・センサー保存を外して巡回だけを先に検証する場合:

```bash
bash EndToEnd/run.sh --headless --device cuda:1 --controller unitree --patrol \
  --scene-config EndToEnd/configs/scenes/office.yaml --env-config EndToEnd/configs/task/office_patrol.yaml \
  --steps 3000 --output EndToEnd/runs/office-motion-check
```

巡回試行は転倒、移動不足、指定した障害物距離への侵入を検査する。記録が完了しても巡回に失敗した
データには`patrol_training_accepted=false`を残し、速度の教師を無効化する。
`check-dataset.sh`も不採用データを拒否する。Officeを全センサー付きで収集する処理は実時間より低速なので、
最初は`--steps 250`（5秒）で入力と教師を確認してから長い収集へ進む。

`--profile`を付けると収集ループの`profile.txt`・`profile.pstats`を保存する。
`summary.json`の`collection_wall_seconds`は初期化を除いた収集時間、
`collection_realtime_factor`はシミュレーション時間÷収集時間を表す。
headless収集ではRGBの姿勢を設定して描画し、物理側の追加描画を省く。
600Hzの各IMUは制御行ごとに一括保存し、その行のcommit前にディスクへflushする。
センサーの取得周期・解像度・制御行とframeの因果関係は維持する。
この方式は収集内容の検証用でもあり、大量収集の速度を保証しない。

`patrol.sh`は長時間用に`--repeat-lidar-pattern`を明示している。MID-360の公開表を4秒周期で再生する近似であり、
実機の長時間非反復走査を再現したものではない。反復設定と未再現のセンサー特性はmetadataへ保存する。

HDF5 schema 5では、観測・関節指令に加えて`velocity_command/{requested,applied,valid}`を毎周期保存する。
`data.targets.SupervisionReader`は、過去の有効LiDAR rayを局所voxelへ統合して表面・free・未知の教師を作り、
速度指令と、現在base座標に対する実測未来姿勢XYZ+xyzwを返す。未知点の占有lossとepisode終了後の未来poseはmaskする。
`observed`は指定voxel解像度で蓄積したLiDAR ray evidenceであり、完全な幾何真値を意味しない。
監査用軌道へ速度指令を入力する処理はない。

収集先には`metadata.json`、`scene-assets.lock.json`、`mounting-audit.json`、`navigation-grid.npz`、
`summary.json`を残す。`check-dataset.sh`は6入力・3教師を実モデルへ接続し、損失の有限性を確認した
`dataset-audit.json`を作る。画角・取り付けは公称URDFと仕様書の内部校正であり、実機個体の校正ではない。

## RGB・LiDARの並列収集

```bash
bash EndToEnd/parallel-patrol.sh --num-envs 16 --steps 150 --device cuda:1 \
  --path-samples 1 --output EndToEnd/datasets/office-parallel-seed0
bash EndToEnd/check-parallel.sh EndToEnd/datasets/office-parallel-seed0
bash EndToEnd/check-dataset.sh EndToEnd/datasets/office-parallel-seed0/env_000/dataset.h5
python3 EndToEnd/scripts/benchmark_parallel.py --envs 4 16 32 --steps 150 \
  --path-samples 1 --output EndToEnd/runs/parallel-benchmark
```

複製したG1/Officeを一つの物理sceneで動かし、RGBをタイル描画、IMU・LiDARをGPU上で一括計算する。
圧縮・保存は専用workerへ渡し、queueを制限してディスクの遅延も測定時間へ含める。
収集CLIは`sim/batch_collect.py`、時計と制御ループは`sim/batch/{schedule,collection}.py`、
GPUセンサーは`sim/batch/sensors.py`、保存は`sim/batch/recording.py`に分離している。
保存に失敗した記録は完了扱いにせず、センサー整合性と巡回品質の監査結果も別々に保持する。
RGBは640×480・30Hz、LiDARは20,000点/scan・10Hz、3か所のIMUは600Hz、関節と指令は50Hz。
カメラはRGBのみで、モデルと保存schemaに深度画像入力は持たない。3D場の教師はLiDARから生成する。
36層・幅256・SwiGLU幅768・M履歴1024・2周期ごとの履歴追加、および今回のMだけを読む3出力は維持する。
既存の記録はそのまま保持し、Readerは旧schema 1〜4も読み出せる。古い7入力の学習済み重みは6入力モデルへそのままloadできない。

`--path-samples 1`は1画素1サンプルのPath Tracing。既定の8サンプルより描画noiseが増える。
入力解像度・取得周期は変えず、測定条件はmetadataへ残す。
`aggregate_realtime_factor`は全環境の合計シミュレーション時間÷収集・圧縮・fsync時間。
各環境が実時間で進む速度とは異なり、初期化時間は別記する。監査に落ちた環境はaccepted速度に含めない。
`check-parallel.sh`は全環境の取得周期、RGBの欠損、URDFのカメラ姿勢、LiDAR反射点と保存poseの整合性を照合する。
初期化も含めた速度、GPUメモリ使用量、最終保存容量は`benchmark.json`に記録する。

2026-10-21の整理前実装、RTX 5060 Ti 16GBでの実測（各環境3秒、1 spp、3 bounce、LZF、保存監査通過）:

| 並列数 | 合計収集速度 | 起動込み速度 | GPUメモリ最大 |
|---|---:|---:|---:|
| 4 | 0.635倍 | 0.252倍 | 6,360 MiB |
| 16 | 1.052倍 | 0.526倍 | 8,686 MiB |
| 32 | 1.142倍 | 0.624倍 | 13,028 MiB |

32環境では合計96秒分を84.05秒で保存した。短時間の試行なので初期化の負担は大きく、起動込みでは実時間未満。
長時間収集の持続速度は別途測定が必要。結果は`runs/parallel-rgb-only-20261021/benchmark.json`に保存した。
6入力・3出力モデルは33,551,628パラメータ。収集済みschema 5を使った実モデルforwardと3種類のlossも確認済み。

## 時系列モデルの学習

`--teacher local`は取得済みLiDARと姿勢から進行方向を選ぶ。地図はspawnと品質計測に用い、
行動には非公開の目標位置を渡さない。従来の全体巡回教師は`--teacher coverage`で選べる。

```bash
bash EndToEnd/parallel-patrol.sh --teacher local --num-envs 16 --steps 500 --seed 1000 \
  --device cuda:1 --path-samples 1 --output EndToEnd/datasets/patrol-pilot
bash EndToEnd/check-parallel.sh EndToEnd/datasets/patrol-pilot
bash EndToEnd/prepare-training.sh EndToEnd/datasets/patrol-pilot \
  --workers 4 --output EndToEnd/datasets/training-cache
bash EndToEnd/train.sh EndToEnd/datasets/training-cache/manifest.json \
  --device cuda:1 --batch-size 4 --output EndToEnd/checkpoints/patrol
```

cacheは生データを保持し、50Hzの6入力トークン・速度指令と5Hzの補助教師を保存する。
RGB等の特徴はfloat16でcacheし、モデルへ渡す際はfloat32へ戻す。3D場はLiDARの有効rayを20本に1本使い、
0.2mの時刻付きworld voxelへ一度ずつ統合する。教師の観測済みmaskはこの間引いたray evidenceを表す。
履歴窓は40.96秒。未知領域の占有lossは無効化する。未来情報を使うのは実測軌道のlabelだけ。

episode単位でtrain/validation/testを分離し、同じ長さのepisodeをbatch化する。
TBPTTは8制御周期、勾配蓄積は4区間。区間間もM履歴1024個と2周期ごとの追加は維持し、
区間境界で勾配だけを切る。36層の全パラメータと3出力をAdamWで更新する。
BF16とactivation checkpointingを使い、速度は各軸の出力範囲で正規化したHuber loss、
場はBCE、軌道は位置のHuber lossとquaternionの符号不変な回転lossで学習する。
補助lossの重みは各0.2。フレームを無作為にシャッフルして履歴を破壊しない。

`last.pt`はepisode batch境界でoptimizerも保存し、同じmanifestとbatch設定で`--resume`できる。
別の収集段階の重みを使う場合は`--initialize`を指定し、新しいoptimizerで開始する。
`best.pt`は検証lossで選び、最後に未使用test episodeを評価する。
`updates.jsonl`、`epochs.json`、`summary.json`に更新・評価・GPUメモリを残す。

学習済みモデルでの巡回確認は、低位の公開歩行方策を固定して実行できる。

```bash
bash EndToEnd/collect.sh --policy-checkpoint EndToEnd/checkpoints/patrol/best.pt \
  --controller unitree --scene-config EndToEnd/configs/scenes/office.yaml \
  --env-config EndToEnd/configs/task/office_patrol.yaml --device cuda:1 \
  --steps 500 --repeat-lidar-pattern --seed 3000 --output EndToEnd/runs/patrol-model-check
```

オンライン方策にも保存時と同じ取得済みframeだけを渡し、今回のMから速度を読む。
学習lossの低下と、モデルによる長時間巡回の成功は別に評価する。

## 重みの持ち出し

コードと設定はGit、重みはcheckpointとして別に配布する。`checkpoints/`はGitの管理対象外。
`best.pt`・`last.pt`には学習再開用のoptimizerも含まれる。現場への転送では、
PyTorchを使えるPython環境で以下を実行し、重みとモデル設定だけを取り出せる。

```bash
PYTHONPATH=EndToEnd/src python -m g1_e2e.inference.export \
  EndToEnd/checkpoints/patrol-batched-20261021/best.pt \
  --output EndToEnd/checkpoints/patrol-batched-20261021/inference
rsync -avP EndToEnd/checkpoints/patrol-batched-20261021/inference/ \
  user@JETSON_IP:~/g1-models/patrol/
```

転送先の親ディレクトリ`~/g1-models/`は先に作成する。USB媒体でも同じフォルダー一式をコピーすればよい。
`model.pt`には3出力の全重みと構成を格納し、量子化はしない。今回のモデルでは約128MiB。
`manifest.json`に元checkpointのSHA-256、学習更新回数、書き出し時のコードcommitとEndToEndの変更有無を記録する。
書き出しはEndToEndの変更をコミットした後に行い、現場にも対応するcommitのコードを用意する。
`SHA256SUMS`で転送後のファイルの一致を確認できる。

```bash
cd ~/g1-models/patrol
sha256sum -c SHA256SUMS
```

`MemoryVelocityPolicy(Path("model.pt"), "cuda:0")`はこの形式と元checkpointの両方を読み込める。
学習再開にはoptimizerを含む`last.pt`を残しておく。
重みの転送にはIsaac Simや学習データは不要だが、実機での実行には対応するPython/PyTorch環境、
実センサーを入力形式へ合わせる接続、低位歩行方策が別途必要。Jetson上の50Hz実行と障害物回避は未検証。
