# ボタン／ペットボトルを選んで押す

既存の右腕FK/IK、関節補間、速度・加速度制限、SDK／ブリッジを使い、
台上の直立したペットボトルをロボット前方（+X）へ押す。
`--task button|bottle` で起動時に選ぶ。省略時は `button`。
既存のボタン専用コマンド・ストローク・Yada採点用処理は引き続き使用できる。

## 準備

画像処理・MuJoCo用のPython環境で、以下の依存を用意する。
既存の `_local/button_press_yada/venv/bin/python` でも検証できる。

```bash
python -m pip install -r Manipulation/push_button/sim/requirements_bottle.txt
git clone --depth 1 --filter=blob:none --sparse \
  https://github.com/google-deepmind/mujoco_menagerie.git _local/bottle_push/menagerie
git -C _local/bottle_push/menagerie sparse-checkout set unitree_g1
```

ボトル認識は既存のPerceptionの `YoloDetector` を使用する。
COCO学習済み `yolo26n.pt` の `bottle` クラスだけを検出する。
既定はリポジトリ直下にある同名の重み。無い場合はUltralytics経由で公式から取得する。
`--weights /path/to/model.pt` でローカルの重みも指定できる。クラス名 `bottle` が必要。
信頼度は既定0.25、ボトルの入力サイズは既定960（`--detector-imgsz`、32の倍数）。
`--device auto|cpu|cuda` は既存検出器に渡す。ボトル入口の既定はCPU、Torchは2スレッド。
共通検出器の既定640は変更しない。

## MuJoCoで実行

```bash
MUJOCO_GL=egl python Manipulation/push_button/sim/run_push_mujoco.py \
  --task bottle --model _local/bottle_push/menagerie/unitree_g1/g1.xml \
  --push-distance 0.05 \
  --report-out _local/bottle_push/results/yolo.json \
  --video-out _local/bottle_push/results/yolo.webm --video-fps 8 \
  --snapshot _local/bottle_push/results/yolo.png

# 同じ入口で従来の上／下ボタンを選択する。
MUJOCO_GL=egl python Manipulation/push_button/sim/run_push_mujoco.py \
  --task button --model _local/bottle_push/menagerie/unitree_g1/g1.xml \
  --weights /path/to/button-model.pt --button-direction up
```

ボトルの既定配置は中心X=0.33m、Y=-0.25m、台上面Z=0.75m。
`--bottle-x`・`--bottle-y` で変えられる。押す距離は `--push-distance`、0より大きく0.10m以下。
ボタン用の1.5～15mmストロークとは独立している。
初回・接近後・押した後の認識画像は `_before1`・`_before2`・`_after` を付けて保存する。

`--detector oracle` はセグメンテーションから正解枠を作る切り分け専用。
既定はYOLOで、検出に失敗しても正解枠へ切り替えない。
制御と位置推定はどちらの検出器でも同じ処理を通す。

ボトルは質量350g、胴体半径34mm、台面摩擦係数0.3の自由剛体。
カメラは固定し、動くボトルを真値で追尾しない。
制御へ渡すのはRGB-Dと機体関節角。ボトルの真の移動量・傾き・接触力は評価だけに使う。
物理時間の進行はシミュレーション側で行い、推論の実時間遅延はこの試験に含めない。
骨盤を固定した腕単体の試験であり、自由立位のバランスやPETの変形を検証するものではない。

## 実機の計画表示・実行入口

### 現場で調整して試す `field` モード

`--profile field` は頭カメラのボトル入口専用の試行設定。省略時の `strict` は従来の設定を使う。
現場設定は `real/field_trial.example.json` をコピーして編集できる。例の台高・範囲は
2026-10-31の台の値なので、使用する台と配置に合わせる。JSON内の `profile` でも選べる。
優先順は **明示したCLI引数 > JSON > プロファイル**。適用値は起動時に表示し、結果JSONにも保存する。

`field` の初期値は20mm押下、台の対辺許容差15mm、ボトルの位置安定幅8mm（新しい3枚）。
台追跡を有効にし、手前の `x_min` と右側の `y_min` を基準に固定する。
`--table-x-edge x_max` / `--table-y-edge y_max` またはJSONで変更できる。
見えない辺へ自動で切り替えず、計測を取り直す。台追跡を使わない切り分けは `--no-track-table`。

腰とIMUの傾きは、カメラ校正とは別に保存する試行開始姿勢から3度で一時停止する。
脚速度0.15rad/s・ジャイロ0.10rad/sの超過を検出すると移動指令を止めて保持する。
100ms未満で戻り、300ms安定した場合は計測・接近中に限り再計測から自動で再開する。
継続する変動、姿勢差、5秒以内に安定3枚を取れない場合は、次の操作を選ぶ。

- `r`: 現在姿勢を監視基準に取り直し、新しい計測と現在姿勢から接近を再計画する。
- `b`: 現在姿勢から経路を確認して復帰する。
- `q` または入力のEOF: 試行を終了して解除する。

入力待ちでも50Hzの状態監視・保持を継続する。再開・復帰には下半身の300msの安定が必要。
押下開始後の中断では `r` を表示せず、その試行の押下を打ち切る。復帰してから次の試行とする。
通信断、状態の不正値・鮮度異常、指令・追従・衝突の異常は終了する。
新しい試行基準を取っても、カメラ・手先の校正値は書き換えない。

```bash
# コピー先のJSONで台高・範囲などを編集する。
cp Manipulation/push_button/real/field_trial.example.json _local/bottle_field.json

# 初回は接触前まで。校正・モデル・ブリッジは従来のものを指定する。
python Manipulation/push_button/real/run_push.py --task bottle \
  --trial-config _local/bottle_field.json \
  --model _local/bottle_push/menagerie/unitree_g1/g1.xml \
  --base-transform /path/to/calibration.json --arm-socket /tmp/g1_button_arm.sock \
  --stop-before-contact --execute --calibrated --output _local/bottle_field_approach_01.json

# 接触前確認後、20mmの押下を試す。各試行の出力名を変えて3回記録する。
python Manipulation/push_button/real/run_push.py --task bottle \
  --trial-config _local/bottle_field.json \
  --model _local/bottle_push/menagerie/unitree_g1/g1.xml \
  --base-transform /path/to/calibration.json --arm-socket /tmp/g1_button_arm.sock \
  --execute --calibrated --output _local/bottle_field_push_01.json
```

対象を選ぶ場合は `--target-pixel U V`、押す量を変える場合は `--push-distance 0.03` を追加する。
監視条件を調整するCLIは `--position-stability-m`、`--waist-delta-deg`、`--imu-delta-deg`、
`--leg-speed-rad-s`、`--gyro-rad-s`、`--motion-persistence-s`、`--stable-hold-s`。
姿勢・速度監視の変更は `field` で指定する。共有のボタン制御には反映しない。

`field` の終了コード0は押下指令・home復帰・解除の処理完了（接触前診断は診断・home・解除の完了）を示す。
1は未完了・操作者終了、2は異常、Ctrl+Cは130。移動量の厳密な画像判定は別に残すため、
`visual_success: false/null` でも処理が完了すれば0になり得る。
ボトルが動いたか・直立を保ったかは解除後の入力で `operator_assessment` へ保存する。
`physical_push_success` は実機では常にnullであり、人の確認を物理計測と混同しない。

記録には適用設定・校正の内容とハッシュ・Gitのコミット（取得できる場合）・実行ソースとモデルのハッシュ・時計対応・29軸の実測角・
実際に送信できた腕目標とトルク・監視基準・停止再開イベントを含む。
RGB-Dは結果名に `_rgbd` を付けた別ディレクトリへ非同期で保存し、同名の過去記録は上書きしない。
各NPZは `color_bgr`、メートル単位の `depth_m`、時刻・内部パラメータ・撮影姿勢の `metadata_json` を持つ。
`np.load(path, allow_pickle=False)` で読める。保存キューの欠落数・保存エラーも結果へ記録する。
保存を切る場合は `--no-record-rgbd`、strictで保存する場合は `--record-rgbd` を指定する。

以下の従来コマンドは、プロファイルを省略すると引き続きstrictとして動作する。

カメラ配信・腕ブリッジの準備は既存の [CAMERA.md](CAMERA.md) と
[README.md](README.md) の手順を使う。
座標は停止時の骨盤の床上投影を原点とし、前方+X、左+Y、床上+Z。
既存の実測校正JSON（`g1-button-calibration-v1/v2`）を共用する。
ボトルモードではボタン半径は使わず、計測した胴体半径・手先半径・位置不確かさを検査する。

```bash
# 計画表示。DDS指令は送らない。台の範囲は実測値に置き換える。
python Manipulation/push_button/real/run_push.py --task bottle \
  --model _local/bottle_push/menagerie/unitree_g1/g1.xml \
  --base-transform /path/to/calibration.json --state-json /tmp/button_state.json \
  --table-height 0.75 --table-bounds 0.20 0.85 -0.60 0.02 \
  --push-distance 0.05 --output /tmp/bottle_preview.json

# 実機入口。既存と同じ校正指定とYES入力が必要。
python Manipulation/push_button/real/run_push.py --task bottle \
  --model _local/bottle_push/menagerie/unitree_g1/g1.xml \
  --base-transform /path/to/calibration.json --arm-socket /tmp/g1_button_arm.sock \
  --table-height 0.75 --table-bounds 0.20 0.85 -0.60 0.02 \
  --execute --calibrated --output /tmp/bottle_execution.json

# 接触前までの診断。同じ実測校正・台設定を指定し、押さずに復帰・解除する。
python Manipulation/push_button/real/run_push.py --task bottle \
  --model _local/bottle_push/menagerie/unitree_g1/g1.xml \
  --base-transform /path/to/calibration.json --arm-socket /tmp/g1_button_arm.sock \
  --table-height 0.75 --table-bounds 0.20 0.85 -0.60 0.02 \
  --stop-before-contact --execute --calibrated --output /tmp/bottle_approach_check.json
```

`--table-height` は実機側で必須。`--table-bounds X_MIN X_MAX Y_MIN Y_MAX` は台の実測平面範囲。
台は厚さ5cmの板として経路検査する。脚・周囲の障害物は含まれない。
接触高さは台面から8cm、`--contact-height` で4～15cmに変更できる。
到達範囲はボトル手前面のX=0.19～0.49m、Y=-0.45～-0.10m、Z=0.70～1.20m。
範囲内でも移動先までのIKや経路検査が通らなければ実行しない。
台の縁にはボトル半径に加えて1cmの余裕を要求する。
複数候補がある場合は `--target-pixel U V` で初回の対象を指定する。

ボタンを選ぶ場合は同じ入口に `--task button` と従来の
`real/align_button.py` の引数（`--weights`・`--stroke` 等）を渡す。
実機接触試験は今回実施していない。

### 任意の重力補償と台基準の追跡

`--gravity-scale 0..1` は右腕7軸へ送る重力補償の倍率。既定0で、
校正済みrealistic評価では明示的に1を指定する。計測関節角・IMUのroll/pitchを
画像処理と独立したMuJoCoモデルへ入れ、重力項を求める。
kp=40（手首15）、kd=1.5は維持し、有効化・解除で補償も増減する。
軸別トルク上限は25/25/25/25/25/5/5Nm。解除完了時にはトルクも0にする。
SDK側のPythonにはMuJoCo・NumPy・SciPyは不要。画像処理側に依存を入れる。
トルクを渡す場合は新しい腕ブリッジが必要で、未対応サーバーへの接続は解除して中止する。

追加の切り分け用オプションは次のとおり。実機側では両方とも既定無効。

- `--dynamics-feedforward`: 重力補償とともに、公称モデルの摩擦とkd×指令速度も補う。
  シミュレーションで生じた移動中の遅れへの対策。モータばらつきの真値は使わない。
- `--track-table`: 校正済みRGB-Dの台面・直交する二辺から機体の並進を求める。
  撮影時の関節角・IMUを状態履歴から補間し、現在のIMUと組み合わせる。
  台高・台範囲・カメラ外部パラメータの実測値が前提。二辺が見えない計測は棄却し、
  新しい機体位置を0.5秒以内に得られなければ押し動作を止める。

これらは静止再現で確認した重力補償だけでは残った移動遅れ・機体の揺れへの追加対策。
実機の頭カメラで台の二辺が見えるか、自由立位、実測校正の妥当性は未検証。

## 計測・動作・結果

1. YOLO枠内の深度を床基準へ変換し、接触高さ付近の胴体を鉛直円筒として推定する。
   台・キャップ・背景を除外し、深度欠損・曲面不足・背景混入は拒否する。
2. 撮影時刻とIDが異なる新しい3枚で位置ばらつき4mm以内を確認する。
   計測は取得時と推論後に0.5秒以内の鮮度を検査する。
3. 全段階と復帰の経路を事前検査し、接触直前から押し終点までは5mm以下に分割する。
   直前の解を次の初期値に使い、位置0.1mm・方向10度・関節限界・衝突検査を維持する。
   台の手前で腕を持ち上げてから5cm手前へ接近し、新しい3枚で再計測して経路を再検査する。
4. 前方の終点と接触高さを固定し、FKを使って連続接近・押し動作を行う。
   接近は最大10mm/s、押す間は最大4mm/s、関節速度0.8rad/s・加速度2rad/s²。
   7軸の速度・加速度・関節範囲を同時に制約した手先速度制御を使う。
   実機入口では押下中も胴体上部を画像で追跡し、横目標を「現在の中心＋開始直線からのずれ」にする。
   画像補正と横・高さのFK補償の合計は最大5mm。中心線誤差8mm・指令差1cmを超えたら中止する。
5. 元の手前位置へ後退し、持ち上げてhomeへ戻してから、予測移動先の同じボトルを新しい3枚で再認識する。
   移動誤差は前方・左右各1cm以内。未確認でも腕の復帰を試み、成功には数えない。
6. 接近後・押す前の失敗では、実測姿勢から記録経路への接続を検査し、正常な状態でだけ
   接近経路を逆順に戻す。押した後は後退・持ち上げ・homeを使う。
   通信・姿勢監視異常では追加動作を中止して解除する。

透明容器などで深度が得られなければ押さない。
`bottle` クラスは材質を判別しないため、対象にはペットボトルを配置する。
実機入口では独立した画像処理ワーカーが台推定を最大10Hz、ボトル検出を最大5Hzで行う。
制御側は完成した最新結果だけを読み、撮影時刻の関節角・IMUとの対応を検査する。
画像が0.5秒より古い場合や対象が消失した場合は押し動作を止める。
制御更新・状態取得・指令計算には100msの上限を維持する。
従来の固定骨盤試験 `run_bottle_mujoco.py` は同期計測・固定目標を使う比較用である。
自己衝突・台・推定した手前面への腕の侵入を経路で検査する。
ボタンと同じく、実機の接触力制御は行わない。

JSONは `motion_completed`（押す動作）、`home_completed`（復帰）、
`completed`（一連の動作・制御解除）、`visual_success`（画像での移動確認）を分ける。
`visual_success: null` は確認不能、`false` は計測できたが移動量が不適合。
実機の `physical_push_success` は常に `null`。模擬環境では物理評価を記録する。
動作と画像確認の両方を満たした場合だけ終了コード0。
`--stop-before-contact` は例外として、`diagnostic_completed`・home・解除の成立で終了コード0となる。
この診断の `completed`・`motion_completed` はfalse、`visual_success`はnullで、押下成功には数えない。
引数・初期化エラーは2、動作・検証失敗は1、中断は130。
`retreat_completed`・`release_completed`、失敗理由・復帰エラー・解除エラーも個別に保存する。
認識ログは未検出・深度/幾何棄却・不安定・鮮度超過・取得期限超過を区別する。

## 校正済みrealistic評価

頭部カメラ向けの四辺認識・経路検査付きIK再探索・配置検査を追加した。
頭カメラ用の標準配置は `(0.36, -0.12)` m。seed 0〜9の押下前後の画角、
台の直交する二辺、接近・押下・復帰の静的経路検査を通過した（最小画角余裕約25px）。
動的な評価結果は [画角内開始の試験記録](reports/VISIBLE_START_20261031.md) を参照。
保存結果の認識映像・集計方法は
[頭部カメラ改善の実装と評価手順](reports/CAMERA_RECOGNITION_IMPLEMENTATION_20261031.md) を参照。
realistic入口の `--bottle-x/--bottle-y` で配置を指定し、`--check-layout` で
描画・物理ステップなしの画角と全経路の静的検査だけを行える。
通常試験（`--fault none`）でもこの検査を行い、`layout_checks.json` に保存する。
不成立のseedは腕・シミュレーションを起動せず失敗として残し、再抽選しない。
明示したX/Yは各軸で優先し、省略した軸だけカメラ別の既定値を使う。
legacyの既定は従来の `(0.33, -0.25)` m。replayで配置を省略した場合は保存値を使い、
座標のない旧ログはカメラ指定によらず従来配置として扱う（ソース等の一致検査も必要）。

realistic評価では、初回の全経路計算後から腕を有効化する直前の計測期限を0.75秒とする。
実機入口の同じ判定は0.5秒。画像の受信・位置推定・押下中の追跡・台追跡は0.5秒を維持する。
使用値はmanifestと`controller.json`の`initial_plan_max_age_s`に記録する。
既存の画角内開始の20試行は変更前の0.5秒であり、0.75秒での動的評価は未実施。

```bash
MUJOCO_GL=egl OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds 0 1 2 3 4 5 6 7 8 9 --camera-profile head --gravity-scale 1 --detector-imgsz 960 --video \
  --out _local/bottle_push/calibrated_new

MUJOCO_GL=egl OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 \
python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds {100..119} --min-successes 18 --out _local/bottle_push/holdout_new

# 異常試験。誤校正は相応する不確かさを校正JSONへ入れ、腕有効化前に拒否する。
python Manipulation/push_button/sim/run_realistic_bottle.py \
  --seeds 0 --profile miscalibrated --out _local/bottle_push/miscalibration_new
# --fault target-loss|stale-images|unreachable|state-disconnect も指定できる。
```

出力先は毎回新しくする。既存結果を上書きしない。
元の5試行は `_local/bottle_push/realistic_20261030/batch_02/` に保存したまま。
評価条件はYadaのseed付きノイズ・遅延・モータばらつき・機体揺れを使い、
`calibrated`では合成した取付誤差を外部パラメータへ反映する。
`uncalibrated`は誤差を未反映、`miscalibrated`は位置不確かさ50mmを入力する異常試験。

実機入口の認識・IK・補間・監視条件・トルク・解除を使い、DDS・カメラ通信はプロセス内で置換する。
RGB-Dのワイヤ形式と鮮度検査は通すが、実ネットワークや実スレッドの検証ではない。
評価では台追跡と動的補償も有効。`--no-track-table`・`--no-dynamics-feedforward`で個別に比較できる。
`--gravity-scale 0`・`--detector-imgsz 640`・`--preview-step-m 1`も切り分け用に使える。
推論・深度推定・IK・衝突検査・台追跡・制約付き手先速度計算の実測時間だけ物理を進める。
描画の実時間はセンサ生成のコストなので遅延へ加えない。
画像処理は完了時刻付きのキューで再現し、その間も物理と50Hzの腕監視・送信を進める。
制御側へ画像処理時間を重ねて加算しない。

`--camera-profile legacy|head` は既定legacy。headはJ1-genの頭カメラ取り付け位置・画角を使う。
`--timing-mode measured|replay` は既定measured。replayは同一コード・モデル・重み・条件の
過去出力を `--timing-log` に指定する診断専用で、正式な成功率へ混ぜない。
接触前診断は `--stop-before-contact`。実行手順とDeepSeekへの引き継ぎは
[REALISTIC_IMPLEMENTATION_20261031.md](reports/REALISTIC_IMPLEMENTATION_20261031.md) を参照。

各seedにはRGB-Dパケットと撮影時の関節・IMU、50Hzの指令・トルク・状態、
IK残差、処理時間、認識の棄却理由、真値による評価を保存する。
最初の配信画像は `initial_rgb.jpg`、最初のYOLO結果の枠付き画像は
`initial_detection.jpg`、各回の検出枠は `detections.json` に保存する。
`summary.json` と後処理のレポートは成功数と手先の接触到達数を全試行数に対して表示し、
開始条件不成立・認識失敗・接触未到達・押下失敗・復帰失敗に分類する。
接触到達は採点用の手先接触力が0.1Nを超えたことを表し、押下成功とは別。
配置検査で止めたseedの移動量・home誤差はnullで、映像は生成しない。
実行ソースのコピー・ハッシュ、モデル・重みのハッシュ、引数はmanifestに保存する。
真値の位置・接触力は評価専用。台基準の推定はRGB-Dだけを使う。
合格には5cm±1cm、横ずれ1cm以内、最終傾き15度以内、画像確認、home誤差0.05rad以内、
不要な腕・台・自己接触0.1N未満、制御重み・トルク0までの解除がすべて必要。
固定骨盤の理想試験と異なり、自由立位は6軸の支持サーボで近似している。

2026-10-30時点の実装・中断時の試験結果・未達条件は
[REALISTIC_RESULTS_20261030.md](REALISTIC_RESULTS_20261030.md) を参照。

## 検証

```bash
G1_MUJOCO_MODEL="$PWD/_local/bottle_push/menagerie/unitree_g1/g1.xml" \
MUJOCO_GL=egl python -m pytest Manipulation/push_button/tests -q
```

モデルを指定すると、基準配置と前後左右各1cmの計5配置で、5cm±1cmの移動・傾き15度以内・
腕の復帰・台への不要接触なしを検査する。
対象消失後の復帰と、到達不能時に腕指令前で止まることも検査する。
ZMQを含む既存テストにはローカルソケットを作れる実行環境が必要。
既存のYada関連テストには公式モデルも必要で、未準備なら
`bash Button_Press/J1-gen/sim/fetch_models.sh` で用意する。

検証環境: MuJoCo 3.14.0、Ultralytics 8.4.182、NumPy 2.2.6、Torch 2.14.1 CPU。
Menagerieは `cda5236f98383f6fbaa4820c6b18dba892cd087f`。
結果・映像・認識画像は `_local/bottle_push/results/`、
今回の各配置のテスト結果は `_local/bottle_push/pytest_final/` に保存した。

2026-10-21の実行結果:

| 検証 | 結果 |
| --- | --- |
| 既存ボタン・ボトル・実機プレビューのテスト | 169件通過 |
| ボトル、正解枠、基準と前後左右各1cm | 5/5成功。移動・傾き・復帰・不要接触を検査 |
| ボトル、YOLO、基準配置 | 前方49.23mm、横ずれ4.29mm、最終傾き0.001度未満、台への不要接触0N、復帰完了 |
| 共通入口、上／下ボタン、正解枠 | 両方とも1.5mm押下成功 |

YOLOの成功記録は `results/yolo.json`、映像は `results/yolo.webm`。
いずれも上記の模擬条件での結果であり、実機は動かしていない。
