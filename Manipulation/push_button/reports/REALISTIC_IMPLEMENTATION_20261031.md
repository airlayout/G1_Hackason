# realistic改善の実装とDeepSeekへの引き継ぎ（2026-10-31）

コードの実装と軽量な自動テストを担当した。今回、新しいシミュレーション試行と実機動作は実行していない。
成功率は未検証であり、以下の評価をDeepSeek側で実行する。
元の10回試験・映像・ログは変更していない。

軽量回帰テストは最終コードで158件通過（8.79秒）。ボトル・非同期処理・撮影時刻・台推定・
ボタン制御・カメラ・腕ブリッジ・復帰・接続確認を、ダミーI/Oと静的モデルで検査した。
Python構文検査と`git diff --check`も通過。実機の動作や物理シミュレーションの成功率を示す結果ではない。

## 実装した内容

- 実機入口の画像処理を専用ワーカーへ分離。カメラソケットと画像用FKモデルはワーカーだけが使う。
  台推定は最大10Hz、YOLOは最大5Hz、入力960・CPU・Torch 2スレッドが基準。
  台の幾何処理は640×480画像を2ピクセル間隔で標本化する。幾何の合否閾値は維持した。
- 制御は完成した最新結果を読む。段階切替前に開始した処理の結果は、新しい3枚の計測に流用しない。
  関節角・IMUは撮影時刻に対応させ、遠い時刻や空白のある履歴は拒否する。
- 接触中は可視の上部胴体から円筒を検証し、横方向だけを画像で補正する。
  初期横位置をy0、現在のボトル中心をyとすると、横目標はy+(y−y0)。
  画像補正とFK補償の合計5mm、中心線誤差8mm、指令差1cm、関節速度0.8rad/s・加速度2rad/s²を維持。
- 確認順序を「押下→後退→持ち上げ→home→画像確認」に変更。検出失敗を成功へ置き換えない。
- `--stop-before-contact`を追加。接触前の位置合わせ後に逆経路で復帰・解除する。
  `diagnostic_completed=true`でも押下の`completed`・`motion_completed`はfalse、画像成功はnull。
- 制御の時刻を状態取得後にそろえ、状態取得・指令計算・更新間隔の100ms超過を検出する。
  姿勢監視の各測定値・検査時刻・閾値を`controller.json`へ保存する。
- 評価器は同じ画像処理エンジンを完了時刻付きキューで再現する。
  画像処理中も50Hzの腕監視・送信と物理を継続し、画像処理時間を制御側へ二重加算しない。

## 実行環境

リポジトリのルートから実行する。既存venvが無い場合だけ作成する。

```bash
python3 -m venv --system-site-packages /tmp/opencode/g1realistic
/tmp/opencode/g1realistic/bin/pip install 'mujoco>=3.3.7' 'scipy>=1.10' \
  'pin==3.9.0' 'cmeel-urdfdom==4.0.1' 'cmeel-tinyxml2==10.0.0'
```

Menagerie・YOLO重み・J1-genのURDFは前回と同じものが必要。
以下は1本ずつ実行する。出力先が存在すると上書きを拒否するため、再試験は別名を使う。
変更や調整が必要になった場合は、勝手に条件・判定閾値を変更せず、失敗ログを残す。

## 評価の順番とコマンド

まず従来視点の開発seedで確認する。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
/tmp/opencode/g1realistic/bin/python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds 0 1 2 3 4 5 6 7 8 9 --camera-profile legacy --timing-mode measured \
  --gravity-scale 1 --detector-imgsz 960 --video \
  --out _local/bottle_push/async_20261031/legacy_dev_01
```

次に頭カメラ条件で同じseedを確認する。こちらが実機準備用の評価条件。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
/tmp/opencode/g1realistic/bin/python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds 0 1 2 3 4 5 6 7 8 9 --camera-profile head --timing-mode measured \
  --gravity-scale 1 --detector-imgsz 960 --video \
  --out _local/bottle_push/async_20261031/head_dev_01
```

頭カメラ条件で10/10を3反復確認し、コード・設定を凍結してから未使用seedを評価する。
反復は`head_dev_02`・`head_dev_03`など新しい出力先を使い、失敗した反復も保存する。
seed 100〜103は過去に使用・中断しているため、今回の未使用評価には使わない。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
/tmp/opencode/g1realistic/bin/python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds {200..219} --min-successes 18 --camera-profile head --timing-mode measured \
  --gravity-scale 1 --detector-imgsz 960 --video \
  --out _local/bottle_push/async_20261031/head_holdout_200_219
```

合格は前方50±10mm、横10mm以内、最終傾き15度以内、画像確認、home誤差0.05rad以内、
不要接触0.1N未満、送信重み・補償トルク0までの解除のすべて。閾値を緩めて合格にはしない。

異常試験は各コマンドを新しい出力先で実行する。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
/tmp/opencode/g1realistic/bin/python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds 0 --camera-profile head --profile miscalibrated \
  --out _local/bottle_push/async_20261031/miscalibrated

MUJOCO_GL=egl OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
/tmp/opencode/g1realistic/bin/python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds 0 --camera-profile head --fault target-loss \
  --out _local/bottle_push/async_20261031/target_loss
```

後者の`--fault`を`stale-images`・`unreachable`・`state-disconnect`に変えて、出力先も変える。
これらは`successes`ではなく`expected_outcomes`で期待した停止・復帰・解除を判定する。
故障注入地点まで到達できなかった場合、異常処理を検証できたとは扱わない。

接触前診断も別に確認する。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
/tmp/opencode/g1realistic/bin/python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds 0 1 2 3 4 --camera-profile head --stop-before-contact \
  --out _local/bottle_push/async_20261031/head_approach_only
```

処理時間の再生診断は、同一コード・モデル・重み・動作条件で取った新しい実測ログだけを使う。
旧版の10回試験は処理経路が異なるため再生対象にできない。条件不一致・時間列不足はエラーにする。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
/tmp/opencode/g1realistic/bin/python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds 0 --camera-profile head --timing-mode replay \
  --timing-log _local/bottle_push/async_20261031/head_dev_01 \
  --gravity-scale 1 --detector-imgsz 960 --video \
  --out _local/bottle_push/async_20261031/head_replay_seed0
```

再生結果は正式成功率へ混ぜない。実測時間のホスト差・負荷差も記録する。

## 返してほしい成果物

- `summary.json`・`results.json`・`manifest.json`・実行標準出力と使用venvの`pip freeze`。
- 各seedの映像、RGB-D、`controller.json`、`processing_times.json`、`states.json`。
- 停止段階と理由、横ずれ、画像確認、home、解除、処理時間の中央値・95パーセンタイル・最大値。
- 頭カメラでボトルや台二辺を確認できない場合は、その画像と棄却理由。従来視点へ切り替えて合格扱いにしない。

## 実機へ進む前の手順

1. 実機RGB-Dを腕指令なしで収集し、ボトル・台二辺の可視性、深度、時計同期、CPU処理時間を確認する。
2. カメラ・手先・台・ボトルの寸法を実測する。合成校正値を実機へ流用しない。
3. 既存の小動作確認を正常完了させる。中断時は今回追加した監視値・時刻と既存ログを比較する。
4. 実機入口の`--stop-before-contact --execute --calibrated`で、接近・再計測・逆経路復帰・解除を3回確認する。
   重力補償は無接触で段階確認し、動的補償は別に検証する。既定では両方無効。
5. その後、低速接触で前後・左右の移動量を実測し、画像判定と照合する。
   状態通信・姿勢監視異常では追加動作を止めて解除し、対象・台追跡の喪失では正常な状態と経路を確認できる場合だけ復帰する。

実行時の`YES`確認は維持している。自由立位・PET変形・実通信は別途実測で確認する。
