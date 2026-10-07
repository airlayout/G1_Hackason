# Push Button

G1 にボタンを押させる動作の開発場所。

リーダーのYada採点環境で評価する入口は [YADA.md](YADA.md)。
実機用のコマンドと設定を残し、Python API / DDS + ZMQ の模擬G1接続を追加した。

シミュレーションのボタンのストロークは上下とも既定 **1.5mm**（`--stroke 0.0015`）。
押下前に1回だけ再計測し、最後は停止区間を挟まず連続して接近・押下する。

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
- `alignment.py`: 対象ボタンの追跡、計測の鮮度検査、1回の再計測で固定した目標へのFK追従と法線方向の接近・押下。従来の反復再計測も選択可能。
- `kinematics.py`: シミュレーションと実機で共用する右腕のFK/IK、関節経路の自己衝突・パネル面検査。
- `real/align_button.py`: 到着後の再計測から位置合わせ、押下、後退までの実行時制御。既定は計画表示。
- `real/arm_bridge.py`: SDKと画像処理を別のPython環境で動かすローカルブリッジ。
- `sim/run_alignment_mujoco.py`: 到着ずれと接近後の追加ずれを与えるRGB-Dフィードバック試験。WebM・GIF・JSONを出力する。
- `sim/run_walk_and_press_mujoco.py`: 同じ29軸モデルで経由点に沿って歩行→停止確認→位置補正→押下→腕の後退を連続実行する。MuJoCo専用。
- `sim/walking.py`: Navigation側と同じ固定版の12軸歩行方策を脚の名前で対応付け、上半身17軸の位置制御と併用する。

実機のボタン前までの移動は操作者または既存のNavigation側で行う。
MuJoCoには、移動と押下をつなげて試す以下の実験を追加した。

## 歩行から押下までのMuJoCo統合試験

脚12軸にはNavigationの`motion.pt`と`g1.yaml`、腰・腕17軸にはMenagerieの位置サーボを使う。
歩行の観測は47次元、物理刻み2ms、方策の更新20ms、歩容の周期0.8秒。
設定・方策の取得元は[unitree_rl_gymの固定版](https://github.com/unitreerobotics/unitree_rl_gym/tree/276801e46c5d433564f24658bac64f254b7d2d4b)。
29軸全体を12軸用の観測に渡さず、脚だけを名前で選ぶ。関節の力の上限は維持する。

速度ゼロでも方策は足踏みを続けるため、最後の経由点に到着後、両足の接地荷重が各20N以上、
重心投影が接地点の凸包内で境界から5mm以上の状態で脚の位置制御へ切り替える。
切替時の実測角度を保持し、股関節のyawを最大0.1rad補正して向きを保持する。
骨盤の速度2cm/s以下、角速度0.05rad/s以下、脚関節速度0.05rad/s以下を両足支持で0.5秒確認してから撮影する。
引き継ぎ・静止確認にはそれぞれ5秒の制限があり、停止できなければ押下しない。
胴体の位置・速度は初期配置後に書き換えず、骨盤の固定や外力による支持は使わない。

RGB-Dカメラは仮想の取り付け位置で胴体に固定し、歩行・静止時の姿勢変化も画像に反映する。
ずれたボタンを真値で追尾するカメラにはしない。検出・深度推定後の腕の処理は従来と共通で、
途中の再計測は1回、接近・押下は連続移動、上下ボタンの機械ストロークは1.5mm。

Menagerieモデルと既存の画像処理依存に加えて`torch`・`PyYAML`が必要。
歩行資産はリポジトリに同梱せず、既存の取得スクリプトで用意する。

```bash
bash Navigation/sim/fetch_assets.sh

# 1.2m手前から2つの経由点を通り、上ボタンを押す。YOLO省略時は正解枠で切り分ける。
MUJOCO_GL=egl python Manipulation/push_button/sim/run_walk_and_press_mujoco.py \
  --model /tmp/g1_mujoco_menagerie/unitree_g1/g1.xml \
  --weights /tmp/g1_elevator_yolo/models/best.pt \
  --start-pose -1.2 0 0 --button-direction up --offset 0.03 -0.03 0 \
  --video-out /tmp/walk_press_up.webm --video-fps 12 \
  --report-out /tmp/walk_press_up.json

# 下ボタンは --button-direction down。映像は等速、960×540。
```

経路を指定する場合は次のJSONを`--route-json /tmp/route.json`で渡す。
座標はMuJoCoの世界座標[m]、向きはdeg。原点はボタン前の予定停止位置で、
初期姿勢は`--start-pose X Y YAW_DEG`。例えば`--start-pose -1.2 -0.7 90`で次の経路を試せる。

```json
[
  {"x_m": -1.2, "y_m": 0.0, "yaw_deg": 90},
  {"x_m": -0.6, "y_m": 0.0, "yaw_deg": 0},
  {"x_m": 0.0, "y_m": 0.0, "yaw_deg": 0}
]
```

経由点は1〜30個、最後は原点±5cm・正面±5度の範囲に指定する。
各点には位置2.5cm以内・向き3度以内で到着を判定し、停止後は最終点から8cm以内・5度以内を確認する。
速度指令は前進0.3m/s、後退・横移動0.15m/s、旋回0.4rad/sを上限とする。
歩行の制限時間は`--navigation-timeout`（既定40秒）。曲がる経路には60秒を指定できる。
JSONには経路、位置の履歴、停止時の荷重・重心余裕、停止確認、計測回数、実際の押下量を記録する。
`travel_distance_m`は出発位置から停止位置までの直線距離で、経路の長さではない。
物体IDと深度を計測する描画はMSAAを無効にし、境界画素で別の物体のIDが混ざらないようにしている。
鑑賞用カメラは元の描画品質を使う。

**この実験の経路追従はMuJoCoの真の位置・向きに対するフィードバック。**
ROS/Nav2、AMCL、実機の位置推定、障害物回避を一緒に実行した結果ではない。
既存のNav2巡回ノードや実機SDKには接続しない。扉は静止した形状で、開閉・乗り込みは扱わない。
実機で一連の動作を行うには、Navigationの到着・停止状態を腕の処理へ渡す接続と実機の立位制御の検証が必要。

2026-10-03、MuJoCo 3.3.7・Menagerie `2cd6b7b`・上記の固定版歩行方策で確認した。
表の時間は歩行・停止・撮影・腕の後退を含むシミュレーション内の時間。
パネルは前後・左右各3cmずらし、各ケースで途中の再計測は1回、非対象ボタンは0mm。

| 経路と押すボタン | 検出 | 所要時間 | 物理的な最大押下量 | 停止位置の誤差 |
| --- | --- | --- | --- | --- |
| 1.2m手前から、上 | YOLO | 30.64秒 | 1.571mm | 2.00cm |
| 1.2m手前から、下 | YOLO | 29.78秒 | 1.512mm | 2.00cm |
| 90度の向き変更を含む上記の経路、上 | YOLO | 35.36秒 | 1.570mm | 1.57cm |
| 2m手前・初期yaw 5度・3経由点、上 | 正解枠 | 35.26秒 | 1.539mm | 1.25cm |

押下時の自由立位も維持し、全ケースで骨盤の最大傾きは6度未満。
69件のテスト（脚の対応付け・停止失敗時の中止・連続歩行から押下までを含む）と、
既存の押下のみの位置ずれ34ケースも通過した。これらの条件外での歩行停止・押下を保証する結果ではない。

## 到着後の位置ずれを補正して押す

既存の経路JSON再生は残し、`real/push_button.py --align` または
`real/align_button.py` から再計測を使う方式を追加した。Navigationを停止してから使う。
移動指令を出すプロセスが残っている場合、押下側のゼロ速度要求だけでは競合を防げない。
このルーチンはNavigationのミッションを取り消すAPIを呼ばない。

1. 歩行停止を要求し、脚関節速度・骨盤の角速度・校正姿勢から停止を確認する。
2. RGB-Dの同じframesetからボタン位置とパネル法線を求め、3フレームの安定性を確認する。
   初回は一意な候補または`--target-pixel`で選択し、その後は3D位置で同じボタンを追跡する。
3. 初回に計測した位置を使い、9cm手前の中間点を通って手先球の表面をボタンの **5cm手前** へ運ぶ。
   中間点は遠い目標でIKが局所解に止まることを避けるために使う。関節経路は各区間で検査する。
4. **0.3秒静止し、1回だけ撮り直す**。この1回は新しい3フレームをまとめた計測で、
   初回のフレームを混ぜず、同じボタンの位置と法線を更新する。
5. 更新した位置へのIKを解き直し、1秒で位置を補正する。以後は画像を撮り直さず、
   最新の関節角からのFKで固定目標への追従を確認する。誤差3mm以内を3回確認して押下へ進む。
   残る追従誤差への補償は最大5mm刻み、指令とFK実測の差は10mm以内に制限する。
6. 固定した目標へ法線方向だけに**連続して接近・押下**する。
   区間ごとの停止を入れず、20msごとにFKを更新して速度を調整する。
   手前では最大25mm/s、最後の2cmで減速し、手前5mmでは最大5mm/s。
   そこから接触まで徐々に減速し、押下中は最大3mm/sまたはストロークの1/2毎秒の小さい方に抑える。
   1.5mmストロークでは押下中は最大0.75mm/s。空隙5mm全体をこの速度で進めることはしない。
   手先の指令速度の加速度は40mm/s²、関節指令は最大0.8rad/s・2rad/s²に制限する。
   接近から押下への切り替えでも速度をリセットしない。
   押込量が設定の `min(0.1mm, ストローク/30)` 以内で0.12秒安定し、速度も落ちたことを確認して終了する。
   1.5mmでは完了許容誤差0.05mm、FKによる過押しの許容量0.15mm、指令の追従補償上限3mm。
   一般の過押し許容量は `min(1mm, ストローク/10)`、追従補償は `min(4mm, max(3mm, ストローク))`。
   重力・接触による指令と実測の差を補う値であり、ボタンをその分余計に押す設定ではない。
   指令とFK実測の差は10mm以内に制限する。
7. 後退、開始時の腕姿勢への復帰、制御重みの解除を行う。

既定は `--alignment-mode single --normal-motion smooth`。画像による目標の更新は途中の1回だけで、
その後の関節角の追従補償は必要に応じて複数回行う。
機体・ボタンが静止していることを前提とし、最後の撮り直しの後に生じるボタンの移動は検出しない。
位置合わせ・接近・押下が制限時間内に収束しない場合は、撮り直しを増やさず中止する。
実機とMuJoCoは同じ制御器・時間設定を使い、腕の保持指令と状態監視は待機中も50Hzで続ける。
実機の実行記録には動作完了を記録するが、物理的な押下成功は点灯や反応で確認する。

| オプション | 既定値 | 意味 |
| --- | --- | --- |
| `--stroke` | 模擬機1.5mm、実機は指定必須 | ストローク。指定範囲1.5〜15mm、実機は実物に合わせる |
| `--alignment-mode` | `single` | `single`: 途中の1回再計測。`iterative`: 従来の反復再計測 |
| `--normal-motion` | `smooth` | `single`の接近・押下。`smooth`: 連続移動、`stepped`: 区間ごとに停止する従来方式 |
| `--remeasure-settle` | 0.3秒 | `single`で撮り直す前の静止待ち |
| `--feedback-timeout` | 20秒 | `single`のFK位置合わせ、連続接近・押下それぞれの制限時間 |
| `--correction-interval` | 2.0秒 | `iterative`の補正開始同士の最小間隔 |
| `--correction-duration` | 0.6秒 | `iterative`の1回の補正移動時間 |
| `--settle-time` | 0.6秒 | `iterative`の移動後の静止待ち |
| `--phase-timeout` | 60秒 | `iterative`の各段階の制限時間 |

`--normal-motion stepped`では従来どおり0.2秒移動＋0.1秒停止でFKを確認する。
`iterative`では`--normal-motion`にかかわらず従来の区間動作を使い、3つの新しい計測で収束を確認し、接近・押下も最低2秒間隔で行う。
補正間隔は「移動時間＋静止待ち」以上が必要。
この従来方式の押下完了の許容誤差は最大1mm（8mm未満のストロークではその1/8）。
両方式ともMuJoCoの物理押下量の成功条件は設定の90～110%、非対象ボタンは10%未満とする。

数センチの**停止位置誤差**は再計測したボタン位置からIKを解き直して吸収する。
手先位置は関節角とモデルから推定するため、カメラ・手先の系統的な校正誤差や治具のたわみを
画像で直接補正するものではない。手先マーカーや力センサはまだ使っていない。
押込量の監視はFKによる位置監視であり、実機の接触力制御ではない。

### 計測・動作の中止条件

- 計測が0.5秒より古い、同じフレームの再利用、2秒以内に安定した計測を取得できない。
- 追跡候補の取り違え・曖昧さ、25mmを超えるフレーム間の対象位置変化、初回位置から80mmを超える変化。
- 3フレームの位置ばらつき4mm超、法線ばらつき5度超、深度欠損やパネル面の推定失敗。
- 手先などで中央領域が隠れ、有効なボタン表面が60%未満になる。
  部分遮蔽は追跡中の面位置と深度外れ値を使って除外し、見えている表面から再計測する。
- 固定目標への補正移動が80mm超。位置合わせまたは連続接近・押下が20秒（`iterative`の各段階は60秒）以内に未収束、再目標化後の追従補償で指令とFK実測の差が10mm超、IK失敗や経路の衝突。
- 連続移動の状態更新間隔が100ms超、FKでの過押しや指令の追従補償が上記の上限を超える。
  1.5mm設定では推定押込量1.65mm超、または指令が4.5mmに達してもFKが追従しない場合。
- 状態受信の途絶、腕の継続した追従誤差、脚や骨盤の動き、校正姿勢からの大きな変化。

腕送信は画像処理と独立した50Hzのスレッドで行う。SDKを別プロセスで動かす場合も、
指令が0.5秒途絶したらSDK側が自分で制御重みを解除する。
自己衝突と計測したパネル平面は経路上で検査するが、実環境の全障害物を表す地図ではない。
静止状態と周囲の確認は別途必要。

### MuJoCoでの再現と映像

前述のMenagerieモデル、`mujoco`・`numpy`・`Pillow`が必要。YOLOを使う映像には
`ultralytics`・`opencv-python`とボタン用重みも必要。WebMの保存にはOpenCVのVP8 encoderを使う。

```bash
# 上下ボタン、前後左右±1/3/5cm、向き±5度、複合ずれ、接近後の追加ずれ: 34ケース
MUJOCO_GL=egl python Manipulation/push_button/sim/run_alignment_mujoco.py \
  --model /tmp/g1_mujoco_menagerie/unitree_g1/g1.xml \
  --alignment-mode single --suite --report-out /tmp/button_alignment_suite.json

# 実際のYOLO検出を使い、前後・左右3cmのずれを補正して押す映像
MUJOCO_GL=egl python Manipulation/push_button/sim/run_alignment_mujoco.py \
  --model /tmp/g1_mujoco_menagerie/unitree_g1/g1.xml \
  --weights /tmp/g1_elevator_yolo/models/best.pt \
  --button-direction up --offset 0.03 -0.03 0 \
  --alignment-mode single \
  --gif-out /tmp/button_alignment_up.gif --gif-fps 8 \
  --report-out /tmp/button_alignment_up.json

# 等速のWebM動画（再生位置を選んで確認できる）
MUJOCO_GL=egl python Manipulation/push_button/sim/run_alignment_mujoco.py \
  --model /tmp/g1_mujoco_menagerie/unitree_g1/g1.xml \
  --weights /tmp/g1_elevator_yolo/models/best.pt \
  --button-direction down --offset 0.03 -0.03 0 \
  --alignment-mode single --video-fps 8 \
  --video-out /tmp/button_alignment_down.webm \
  --report-out /tmp/button_alignment_down.json

# 回帰・異常系・モデル・模擬SDKブリッジのテスト
G1_MUJOCO_MODEL=/tmp/g1_mujoco_menagerie/unitree_g1/g1.xml \
  MUJOCO_GL=egl python -m pytest Manipulation/push_button/tests -q
```

`--button-direction down`で下ボタンを押す。
`--drift-after-approach 0 0.01 0.01`は待機姿勢へ動かした後に上下・左右各1cmのずれを追加する。
`--offset`はパネルをロボットに対して移した変位。ロボットの到着位置がずれる状況と
相対位置として等価だが、これは歩行の試験ではない。押下方向のずれはパネル全体を回して作る。
真のボタン位置は評価にのみ使い、制御目標は各回のRGB-D画像から求める。
`--weights`を省略した一括試験はセグメンテーションで検出枠を得るため、YOLOの精度試験にはならない。
WebM・GIFは左が物理動作、右が最後に計測したカラー画像と誤差・押下量。
映像は等速で、各フレームのシミュレーション時刻も表示する。
JSONには方式、計測回数、FK追従補償、時間設定、所要時間も保存する。
連続移動では各更新の速度と関節指令、接近・押下中の約50Hzの手先位置も記録する。
ストロークと過押し・追従補償の許容量もJSONに記録する。
`single`の成功判定には初回計測＋途中の再計測の計2回だけを使ったことも含める。
`iterative`では設定した最小開始間隔を守ったことも成功条件に含める。

既定は自由立位の関節位置サーボ。`--fixed-base`は切り分け用。
仮想カメラはボタンが見える位置にあり、実機搭載位置を再現していない。
シミュレーション結果は実機での検出精度、押下力、立位制御の保証にはならない。

#### 1.5mmストロークの実験記録

2026-10-01、MuJoCo 3.3.7、Menagerie `2cd6b7b`で、既定の1回再計測＋連続移動を検証。
上下ボタン、前後左右±1/3/5cm、向き±5度、複合ずれ、接近後の追加ずれを含む34ケースすべて成功。
全ケースで途中の再計測1回、物理的な最大押下量1.496〜1.555mm、非対象ボタン0mm。
全動作時間は20.00〜21.81秒。58個の回帰・異常系・速度・加速度・物理モデル・模擬SDKテストも通過した。

YOLO（重み `536e894`、Ultralytics 8.4.165）で前後・左右各3cmのずれを与えた等速動画も検証した。

| ボタン | ストローク設定 | 物理的な最大押下量 | 全動作時間 | 途中の再計測 |
| --- | --- | --- | --- | --- |
| 上 | 1.5mm | 1.545mm | 20.34秒 | 1回 |
| 下 | 1.5mm | 1.516mm | 20.38秒 | 1回 |

可動ジョイントの範囲は上下とも0〜1.5mm。
ばね350N/m・減衰3N·s/mに加え、機械終端の制約を `solreflimit="0.004 1"`、
`solimplimit="0.99 0.99 0.0001"` とし、従来の軟らかい終端で過剰に沈まないようにした。
時定数4msは物理刻み2msの2倍。制約に有限の柔らかさがあるため、物理変位には上記の小さな超過が残る。
制約の設定は[MuJoCoの公式説明](https://mujoco.readthedocs.io/en/3.3.7/modeling.html#solver-parameters)を参照。
真値やボタンのジョイント変位は評価にのみ使い、制御はRGB-Dと腕のFKで行う。
時間はシミュレーション内の時間。実機は未実行で、実物のストロークは引き続き指定必須。

#### 旧8mm設定・連続接近・押下方式の実験記録

以下は1.5mmへの変更前の記録。ストローク指定は `--stroke 0.008`。
2026-10-01、`--alignment-mode single --normal-motion smooth`で8mmストロークの34ケースすべて成功。
途中の再計測は全ケースで1回。物理押下量は7.34〜7.52mm、非対象ボタンは0mm、
全動作時間は20.41〜22.11秒だった。追加の下ボタン・ずれなし・2mmストロークも成功（2.01mm）。
46個の回帰・異常系・速度・加速度・モデル・模擬SDKテストが通過した。

同じYOLO・前後左右3cmずれで、区間ごとの停止方式も現行コードで再実行した。

| ボタン | 停止方式の全時間 | 連続方式の全時間 | 接近・押下中の指令停止区間 |
| --- | --- | --- | --- |
| 上 | 23.77秒 | 20.68秒 | 32→0回 |
| 下 | 29.03秒 | 20.72秒 | 50→0回 |

指令停止区間は約50Hzの記録で関節指令が60ms以上変わらなかった区間。
開始直後0.2秒と終点まで残り1mm以内の区間は除外している。物理的な完全停止の回数ではない。
全34ケースの指令記録で速度・加速度の上限を確認した。
時間はシミュレーション内の時間であり、実機は未実行。

#### 1回再計測＋区間ごとの停止方式の実験記録

当時の指定は `--stroke 0.008 --alignment-mode single --normal-motion stepped`。

2026-09-30、同じモデルで8mmストロークの一括34ケースすべて成功。
全ケースで初回計測と途中の再計測の計2回だけを使った。途中の再計測は1回。
計測目標へのFK誤差は最大2.89mm、物理押下量は7.37〜7.61mm、非対象ボタンは0mm。
全動作時間は23.64〜31.44秒だった。実際のYOLOを使う3cmずれの映像は上下とも成功。

| 同じYOLO・3cmずれの条件 | 従来の2秒方式 | 1回再計測方式 |
| --- | --- | --- |
| 上ボタン | 117.64秒 | 23.77秒 |
| 下ボタン | 123.18秒 | 29.03秒 |

全動作時間は約76〜80%短縮した。この時間はシミュレーション内時間であり、
実機での推論・通信時間を含む測定ではない。
変更には撮り直し回数の削減、接近・押下の2秒待機の廃止、初回計測を使う接近経路の変更を含む。
従来の模擬機は名目座標へ接近し、今回は実機と同じく初回計測で9cmの中間点と5cmの待機点を決める。
この比較は撮り直し回数だけの効果を分離した試験ではない。

YOLOの中心推定自体の真値誤差は上4.67mm・下5.25mm、計測目標へのFK誤差は上0.99mm・下0.38mm。
追加の下ボタン・ずれなし・2mmストロークも成功（押下量1.90mm）。
41個の回帰・異常系・モデル・模擬SDKテストが通過した。実機は未実行。

#### 従来の2秒反復方式の実験記録

当時の指定は `--stroke 0.008 --alignment-mode iterative --correction-interval 2.0`。
2026-09-30、MuJoCo 3.3.7、Menagerie `2cd6b7b`で2秒間隔を検証し、
設定8mmの一括34ケースすべて成功。補正の開始間隔は位置合わせ約2.204秒、
接近・押下約2.000～2.002秒で、全ケースが設定した下限を守った。
補正後の計測目標に対するFK誤差は最大2.97mm、押下量は7.77～8.49mm、
非対象ボタンの最大押下量は0mm。骨盤の最大移動は10.13mm、最大傾きは1.32度だった。

公開YOLO重み `536e894`、Ultralytics 8.4.165による前後・左右各3cmずれの映像でも上下とも成功。
補正後の計測目標に対するFK誤差は上0.50mm・下0.95mm、押下量は上8.48mm・下8.25mm。
**YOLOによる中心位置の推定自体には上5.29mm・下5.61mmの誤差が残った。**
この誤差と関節角から計算した位置合わせ誤差は別の値であり、実機での校正・精度評価が必要。

| 同じYOLO・3cmずれの条件 | 変更前の全動作時間 | 2秒間隔の全動作時間 |
| --- | --- | --- |
| 上ボタン | 28.55秒 | 117.64秒 |
| 下ボタン | 28.95秒 | 123.18秒 |

接近・押下も2秒間隔にしたため、全動作は約2分に延びる。
追加の下ボタン・ずれなし・設定2mmの試験では、補正量を0.25mmまでに抑えて成功
（最大押下量2.11mm）。これは追加の1条件であり、小ストロークの全ずれ条件は未検証。
回帰・時間制約・異常系・モデル・模擬SDKブリッジの35テストも通過した。

### 実機の校正データ

位置合わせ実行用のJSONは`g1-button-calibration-v1`を使用する。
下記の値は**すべて実測値で埋める**。`null`や未校正の模擬値では実行できない。

```json
{
  "schema": "g1-button-calibration-v1",
  "T_base_optical": null,
  "camera_body": "torso_link",
  "pelvis_height_m": null,
  "tip_offset_m": null,
  "tip_radius_m": null,
  "button_radius_m": null,
  "position_uncertainty_m": null,
  "reference_waist_q_rad": null,
  "reference_imu_rpy_rad": null
}
```

`T_base_optical`は従来と同じ4×4の変換。校正時の腰関節角とIMU姿勢も保存する。
`camera_body`はカメラを固定したMenagerieモデル上の胴体・骨盤のbody名を実物に合わせて指定する。
校正時の変換からカメラ取付先との固定変換を求め、実行時の腰角・骨盤のroll/pitchをFKに反映して
カメラ変換と手先位置を更新する。腕や可動治具に取り付けたカメラは対象外。
校正時から腰角が0.025rad、骨盤の傾きが2度を超えて変わった場合は再校正を要求する。
`position_uncertainty_m`はカメラ・モデル・治具を含む位置誤差を実測して決める。
ボタン半径から手先半径を引いた余裕が、不確かさと3mmの位置合わせ許容値を収めない場合は中止する。

### 実機での計画表示と実行

PC2ではSDKのPython環境と画像処理環境が異なるため、Unixソケットでつなぐ方式を用意した。
SDK側は標準ライブラリと既存のUnitree SDKだけを使い、MuJoCoやYOLOは読み込まない。
以下の`python3`はそれぞれのコマンドに必要な環境のPythonを使う。

```bash
# SDK環境: 現在関節角・IMUの保存だけ。腕や歩行は動かさない。
python3 Manipulation/push_button/real/arm_bridge.py \
  --network-interface enp3s0 --snapshot-out /tmp/button_state.json

# 画像処理環境: カメラで再計測して計画を表示するだけ。DDS接続不要。
python3 Manipulation/push_button/real/push_button.py --align \
  --model /path/to/unitree_g1/g1.xml --weights /path/to/best.pt \
  --base-transform /path/to/measured_calibration.json \
  --state-json /tmp/button_state.json --stroke 0.0015 \
  --target-pixel 320 240 --output /tmp/button_alignment_preview.json

# SDK環境: Navigationを停止した後に、別の端末でブリッジを起動する。
python3 Manipulation/push_button/real/arm_bridge.py \
  --network-interface enp3s0 --socket /tmp/g1_button_arm.sock --arm

# 画像処理環境: 実測校正・無接触試験の後だけ実行。実行時のYES入力も必要。
python3 Manipulation/push_button/real/push_button.py --align \
  --model /path/to/unitree_g1/g1.xml --weights /path/to/best.pt \
  --base-transform /path/to/measured_calibration.json \
  --arm-socket /tmp/g1_button_arm.sock --stroke 0.0015 \
  --alignment-mode single \
  --target-pixel 320 240 --execute --calibrated \
  --output /tmp/button_alignment_execution.json
```

例の`--stroke`と画素座標は実物に合わせて変更する。
同じ環境にSDKと画像処理が入っている場合は`--arm-socket`を省略して直接接続できる。
カメラはカラー・深度を直接取得するので、既存のカラー配信サーバーが同じRealSenseを使っている場合は
先に停止する。実行時JSONは従来のシミュレーション済み経路JSONとは別形式であり、
実機の押下成功は自動認定しない。制御重み解除の失敗も記録する。

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
コミットは `2cd6b7be15440787d12f7eb4c34ae1a1a03237c`、今回の検証は MuJoCo 3.3.7。
既定値は、ロボット前方 X=0.36 m、右側 Y=-0.25 m、上ボタン床上高さ=1.00 m、
下ボタン床上高さ=0.86 m、ボタン半径=0.032 m、ストローク=0.0015 m、
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
