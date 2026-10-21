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
信頼度は既定0.25、`--device auto|cpu|cuda` は既存検出器に渡す。

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

## 計測・動作・結果

1. YOLO枠内の深度を床基準へ変換し、接触高さ付近の胴体を鉛直円筒として推定する。
   台・キャップ・背景を除外し、深度欠損・曲面不足・背景混入は拒否する。
2. 撮影時刻とIDが異なる新しい3枚で位置ばらつき4mm以内を確認する。
   計測は取得時と推論後に0.5秒以内の鮮度を検査する。
3. 全段階と復帰の経路を事前検査し、台の手前で腕を持ち上げてから5cm手前へ接近する。
   接近後に一度だけ、新しい3枚で再計測する。
4. 以降は目標を固定し、FKを使って連続接近・押し動作を行う。
   接近は最大25mm/s、押す間は最大5mm/s、関節速度0.8rad/s・加速度2rad/s²。
   横・高さのFK誤差だけを最大5mm補償し、ボトル中心線から8mmを超えたら中止する。
5. 元の手前位置へ後退して手を横へ退避し、予測移動先の同じボトルを新しい3枚で再認識する。
   移動誤差は前方・左右各1cm以内。未確認でも腕の復帰を試み、成功には数えない。

透明容器などで深度が得られなければ押さない。
`bottle` クラスは材質を判別しないため、対象にはペットボトルを配置する。
押している間は画像を更新しないので、その間の横滑りは押した後に判定する。
自己衝突・台・推定した手前面への腕の侵入を経路で検査する。
ボタンと同じく、実機の接触力制御は行わない。

JSONは `motion_completed`（押す動作）、`home_completed`（復帰）、
`completed`（一連の動作・制御解除）、`visual_success`（画像での移動確認）を分ける。
`visual_success: null` は確認不能、`false` は計測できたが移動量が不適合。
実機の `physical_push_success` は常に `null`。模擬環境では物理評価を記録する。
動作と画像確認の両方を満たした場合だけ終了コード0。
引数・初期化エラーは2、動作・検証失敗は1、中断は130。

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
