# Push Button

G1 にボタンを押させる動作の開発場所。

## 現在の実装

- `trajectory.py`: 右腕7軸の関節角検証と滑らかな補間。
- `sim/run_mujoco.py`: MuJoCo Menagerie の G1 29DoF モデルに、写真を参考にした
  暗色パネルと白い上下矢印ボタンを追加。固定されたプラスチック製手先を球状の接触点として扱い、
  腕の逆運動学で「待機→接触→押下→後退→初期姿勢」を生成・実行する。
  ボタンは可動ジョイントとばねを持ち、最大押下量・骨盤高・移動量・傾きで成否を判定する。
  既定は自由立位、`--fixed-base` は腕単体の切り分け用。
- `real/push_button.py`: シミュレーションで成功した関節経路を読み込み、
  歩行停止要求後に `rt/arm_sdk` で右腕を動かす。既定は経路表示のみ。
- `vision.py`: YOLOの検出枠とカラー画像に位置合わせした深度から、
  ボタン表面中心とパネル法線を推定する。深度欠損・面推定失敗は中止する。
- `sim/run_vision_mujoco.py`: MuJoCoのRGB-D画像で検出→3D推定→押下まで検証する。
- `real/localize_button.py`: RealSense + YOLOで3D目標を記録する。腕は動かさない。

ボタン前までの移動は、この段階では操作者または既存の Navigation 側で行う。
位置が分からない壁へ開ループで歩かせる機能は含めていない。

**MuJoCo 実験は Menagerie の簡易モデルと関節位置サーボによる検証。**
接触、到達性、模擬モデルでの立位安定を確認できるが、実物ボタンの押下力、
Unitree の歩行制御下でのバランス、実機のファームウェア互換性を証明するものではない。
実機はまだ動かしていない。

## MuJoCo 実験の準備と実行

共通の Python 環境（`G1_HuggingFace/venv/` など）を有効にし、`mujoco` と `numpy` を入れる。
MuJoCo Menagerie の `unitree_g1/g1.xml` と `assets/` を取得する。
モデルファイルとそのメッシュはこのリポジトリには複製しない。

```bash
git clone --filter=blob:none --sparse \
  https://github.com/google-deepmind/mujoco_menagerie.git /tmp/mujoco_menagerie
git -C /tmp/mujoco_menagerie sparse-checkout set unitree_g1
python -m pip install mujoco numpy
python Manipulation/push_button/sim/run_mujoco.py \
  --model /tmp/mujoco_menagerie/unitree_g1/g1.xml \
  --button-direction up --gif-out /tmp/button_up.gif \
  --plan-out /tmp/button_up_plan.json
python Manipulation/push_button/sim/run_mujoco.py \
  --model /tmp/mujoco_menagerie/unitree_g1/g1.xml \
  --button-direction down --gif-out /tmp/button_down.gif \
  --plan-out /tmp/button_down_plan.json
```

`RESULT_SUCCESS` と最大押下量、骨盤高、移動量、傾きが出れば成功。
`--viewer` を付けると物理時間に合わせて表示し、ボタンが十分押されると
押したボタンの白い面が淡い黄色に変わる。上と下のボタンは独立したスライドジョイントで、
非対象ボタンが押されていないことも成功条件に含める。`--fixed-base` は腕単体の接触試験に使う。
試験した Menagerie の
コミットは `2cd6b7be15440787d12f7eb4c34ae1a1a03237c`、MuJoCo は 3.14.0。
既定値は、ロボット前方 X=0.36 m、右側 Y=-0.25 m、上ボタン床上高さ=1.00 m、
下ボタン床上高さ=0.86 m、ボタン半径=0.032 m、ストローク=0.008 m、
ボタン手前の待機距離=0.030 m。`--button-x`、`--button-y`、`--height`、
`--stroke`、`--clearance` で変更できる。`--height` は上ボタンの中心高さを指定する。
これらは**模擬値**であり、
実物のエレベーターの寸法ではない。
`--tip-offset X Y Z` は右手首座標系から見たプラスチック手先の接触点。

### YOLO + 深度画像で押す

公開済みの[エレベーターボタンYOLOv8nモデル](https://github.com/Kshaw17-web/End-to-end-elevator-button-detection)
の `models/best.pt` を重みとして使える。ライブラリは `mujoco`、`numpy`、
`ultralytics`、`opencv-python`（画像保存時は `Pillow`）が必要。
既存の `Perception/common/detector/yolo_detector.py` で推論するため、
モデルをこのリポジトリへコピーする必要はない。

```bash
git clone --filter=blob:none --sparse \
  https://github.com/Kshaw17-web/End-to-end-elevator-button-detection.git \
  /tmp/elevator-button-yolo
git -C /tmp/elevator-button-yolo sparse-checkout set models
MUJOCO_GL=egl python Manipulation/push_button/sim/run_vision_mujoco.py \
  --model /tmp/mujoco_menagerie/unitree_g1/g1.xml \
  --weights /tmp/elevator-button-yolo/models/best.pt \
  --button-direction down \
  --snapshot /tmp/button_detection.png --gif-out /tmp/button_demo.gif \
  --plan-out /tmp/button_vision_plan.json
```

`--gif-out` は腕とボタンが見える視点から、待機→接触→押下→後退をGIFに保存する。

検出器を切り分けたい場合だけ `--detector oracle` を指定する。
このモードはMuJoCoのセグメンテーションIDから正解枠を作るため、YOLOの精度試験にはならない。
シミュレーションのカメラ位置・照明・ボタン形状は仮のもの。RGB-D カメラは
パネルが見える仮想位置にあり、G1 実機の搭載カメラ位置を再現していない。
MuJoCoで成功しても、
実機のカメラでの検出精度とボタン位置精度は別途測定が必要。
2026-09-29の実測では、Menagerie `2cd6b7b`、MuJoCo 3.3.7、公開重み `536e894`、
Ultralytics 8.4.165で、今回の上下ボタンをそれぞれ検出・押下できた。
公開モデルは `button` 1クラスなので、上下の指定は既知のパネル高さに基づいて行う。
ロボットの別部位の誤検出は3D位置・押下方向の検査で棄却した。
YOLO推定位置を使った模擬押下量は上8.2 mm、下7.4 mm（設定ストローク8 mm）。

実機ではRealSenseのカラーと深度を同一のframesetから取得し、深度をカラーへ位置合わせする。
既存のLeRobot ZMQ配信はカラーのみなので、この試験はカメラを持つG1 PC2上で行う。
カラー配信サーバーが同じRealSenseを使用中なら停止してから起動する。

```bash
python Manipulation/push_button/real/localize_button.py \
  --weights /path/to/elevator-model/models/best.pt --frames 10 \
  --preview /tmp/button_preview.png --output /tmp/button_target_camera.json
```

複数ボタンが写る場合は、プレビュー画像に描かれた候補中心の画素座標を確認し、
`--target-pixel U V` で押すボタンを指定して再実行する。
指定した画素を含む候補が複数ある場合や、3D条件を満たす候補が複数ある場合は中止する。
本実装は階数文字の自動選択にはまだ対応しない。

この出力の `frame` は `camera_optical` であり、腕の経路には渡せない。
ロボット基準の3D目標を出すには、実測した4×4同次変換 `T_base_optical` を
JSONに保存して `--base-transform /path/to/calibration.json` を指定する。
基準座標は**停止時の骨盤の床上投影を原点、前方+X、左+Y、床上+Z**とする。
カメラ光学座標は右+X、下+Y、前+Z。カメラの取り付けや姿勢が変わったら再校正する。
校正済み出力の `frame` は `robot_base` となり、次のMuJoCo試験へ渡せる。

```bash
python Manipulation/push_button/sim/run_mujoco.py \
  --model /tmp/mujoco_menagerie/unitree_g1/g1.xml \
  --button-direction down --target-json /tmp/button_target_base.json \
  --plan-out /tmp/button_plan.json
```

この試験では、計測位置に仮想ボタンを配置して押下可能性を調べる。
実物との位置誤差、押下力、接触安定性を証明するものではない。
現時点の模擬パネルは前方+Xに向けて押す配置だけに対応する。

### ボタン位置から右腕7軸の逆運動学へ

深度画像から求めたボタン表面中心を `p`、パネルへ押し込む単位方向を `n`、
手先の接触球の半径を `r` とする。`run_mujoco.py` は、待機点
`p - (r + clearance)n`、接触直前 `p - (r + 0.003)n`、
押下終点 `p - rn + stroke*n` を手先の目標として作る。

各目標に対して、MuJoCoの手先ヤコビアンを使う右腕7軸の数値IKで関節角を求める。
位置と手先の向きの誤差を同時に減らし、直前の解を次の初期値として使う。
位置誤差1.5 mm未満・向き誤差10度未満・関節範囲内に収束しなければ、
計画を出力せず中止する。各段階の誤差と関節限界までの余裕は
`[ik:approach]` などのログと、計画JSONの `ik_validation` に記録する。
このIKは目標姿勢と関節角の対応を求めるもので、実機の押下力制御にはならない。

## 実機での使い方

まず計画を表示する。これだけなら SDK や DDS 接続は不要。

```bash
python Manipulation/push_button/real/push_button.py --plan /tmp/button_plan.json
```

実機に送る前に、機体が **29DoF** であること、ファームウェアと `rt/arm_sdk` の使用可否、
手先の実際の接触点、ボタン中心と壁面方向を確認する。特に `--tip-offset` と
ボタン位置は現物に合わせて調整し、MuJoCo を再実行する。
床上高さだけでは位置合わせはできない。実機で停止した G1 の骨盤の床上投影を
XY 原点とし、前方を +X、左を +Y、床面を Z=0 としたボタン位置を測る。
ボタンとの接触を伴わない
空中動作から確認し、その後に低速で実物へ近づける。

位置合わせと周囲の安全確保を終えた後だけ実行する。`--execute --calibrated` と実行時の
`YES` 入力が必要。`StopMove()` と同じゼロ速度 API を呼び、応答コードと
操作者の目視で静止を確認してから腕を動かす。

```bash
python Manipulation/push_button/real/push_button.py \
  --plan /tmp/button_plan.json --network-interface enp3s0 \
  --execute --calibrated
```

実機側は腕の現在角を `rt/lowstate` から読み、左右の腕の現在姿勢を保持しながら
制御重みを 0→1 に上げる。右腕だけを経路に沿って動かし、最後は開始時の姿勢に戻して
重みを 1→0 に下げる。関節範囲、計画速度、状態受信の途絶、追従誤差を監視する。
通信途絶時も重みを下げる送信を試みるが、通信そのものが失われた場合に
送信が届く保証はない。手動の非常停止手段を確保すること。

本体の歩行制御が有効であることを前提とし、押下スクリプトはモードを切り替えない。
ファームウェアのバージョンは SDK の低レベル状態から自動判定していないため、
実機での無接触試験を先に行う。
押下成否は実機では自動判定せず、ボタンの点灯やエレベーターの反応を人が確認する。

## 制御方針

移動と押下を同時には行わない。下半身は Unitree の高レベル歩行制御 (`LocoClient`)
でボタン手前まで移動させ、停止を確認してから、上半身だけを `rt/arm_sdk` 経由で
指令する。既存の [`SimpleWalk/real/walk_forward_real_sdk.py`](../../SimpleWalk/real/walk_forward_real_sdk.py)
が歩行側の接続例になる。

腕を動かしている間も歩行用の制御モードは維持し、全身のモード切替を挟まない。
腕側は `rt/lowstate` から現在の関節角を読み、実機の機種で確認した腕関節だけを
`rt/arm_sdk` に送る。SDK の公式例に合わせて、制御重みは現在姿勢から徐々に有効化し、
押下後は腕を戻して滑らかに解除する。脚関節は送らず、腰関節も初期実装では対象外にする。
全身の `rt/lowcmd`、`Damp()`、`ReleaseMode()`、歩行モードの切替を押下ルーチンから呼ばない。

`rt/arm_sdk` と `LocoClient` を併用できる制御経路は公開されている。歩行中の腕制御は
この動作の対象外とし、停止状態で腕を動かすところから確認する。

## 押下動作

目標ボタン位置が決まった後、腕の動作を次の段階に分ける。

1. ボタン手前の待機姿勢へ移動
2. 低速で接触位置まで近づく
3. ボタンの必要ストロークだけ押す
4. 押下を確認して手先を戻す

目標位置、押下方向、必要ストローク、押下力、手先治具の形状は未確定。

## 実機運用前に確認する情報

- G1 の自由度構成 (23DoF / 29DoF など)
- 本体ソフトウェア／ファームウェアのバージョンと、使用できる制御モード
- 押すボタンの位置・寸法・必要ストローク・押下力
- 手先に取り付ける治具の有無

## 参考資料

- [Unitree SDK2 Python: G1 arm SDK DDS example](https://github.com/unitreerobotics/unitree_sdk2_python/blob/master/example/g1/high_level/g1_arm7_sdk_dds_example.py)
- [Unitree XR Teleoperate: Motion wiki](https://github.com/unitreerobotics/xr_teleoperate/wiki/Motion)
- [G1 29DoF で歩行中に arm SDK を使った際の安定性に関する報告](https://github.com/unitreerobotics/unitree_sdk2_python/issues/173)
