# Push Button

G1 にボタンを押させる動作の開発場所。

## 現在の実装

- `trajectory.py`: 右腕7軸の関節角検証と滑らかな補間。
- `sim/run_mujoco.py`: MuJoCo Menagerie の G1 29DoF モデルに、高さ1 m の
  エレベーター模擬ボタンを追加。固定されたプラスチック製手先を球状の接触点として扱い、
  腕の逆運動学で「待機→接触→押下→後退→初期姿勢」を生成・実行する。
  ボタンは可動ジョイントとばねを持ち、最大押下量・骨盤高・移動量・傾きで成否を判定する。
  既定は自由立位、`--fixed-base` は腕単体の切り分け用。
- `real/push_button.py`: シミュレーションで成功した関節経路を読み込み、
  歩行停止要求後に `rt/arm_sdk` で右腕を動かす。既定は経路表示のみ。

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
  --plan-out /tmp/button_plan.json
```

`RESULT_SUCCESS` と最大押下量、骨盤高、移動量、傾きが出れば成功。
`--viewer` を付けると物理時間に合わせて表示し、ボタンが十分押されると
上の表示灯が赤から緑になる。`--fixed-base` は腕単体の接触試験に使う。
試験した Menagerie の
コミットは `2cd6b7be15440787d12f7eb4c34ae1a1a03237c`、MuJoCo は 3.14.0。
既定値は、ロボット前方 X=0.36 m、右側 Y=-0.25 m、床上高さ=1.00 m、
ストローク=0.008 m、ボタン手前の待機距離=0.030 m。`--button-x`、`--button-y`、
`--height`、`--stroke`、`--clearance` で変更できる。これらは**模擬値**であり、
実物のエレベーターの寸法ではない。
`--tip-offset X Y Z` は右手首座標系から見たプラスチック手先の接触点。

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
