# G1 デジタルツイン操作環境

Unitree G1 を Isaac Sim の Warehouse シーン上で動かす環境。
キーボード操作、地図作成（SLAM）、Nav2 による自律ナビゲーションに対応する。
将来の実機連携を見据え、コマンド送信部を差し替え可能な抽象層にしている。

**初めて動かす場合は [SETUP.md](SETUP.md) を先に読むこと。**
必要な環境、インストール手順、最初の動作確認までをまとめてある。

## できること

| 機能 | 起動方法 | 状態 |
|---|---|---|
| キーボード操作 | `bash run.sh` | 動作確認済み |
| 地図作成 | `src/build_map.py`（下記） | 動作確認済み（実形状とのずれ 0.36 m） |
| 自律ナビゲーション | `bash run_nav2.sh` + `bash run_rviz.sh` | 動作確認済み（短距離） |
| 手動操作 + 地図表示 | `bash run_nav2.sh --manual` | 地図と自己位置を RViz で見ながら手動操作 |

**既知の制約:** 距離が長いと成功率が下がる。実時間比が 0.23x（GUI 込み）まで
落ちること、地図の未探索領域が残ること、G1 が後退できないことが要因。

## 起動

用途に応じて 3 通りある。**手動と自律は排他で、実行中には切り替えられない。**

### 1. 手動操作だけ（一番軽い）

```bash
cd /home/spacedata/isaac_dev/G1/SimEnvTest
bash run.sh                # Warehouse シーン（GUI）
bash run.sh --flat         # 平地のみ（動作確認用）
```

Isaac Sim だけを起動する。地図も RViz も使わないので起動が速い。
歩行の挙動だけを見たいときはこれ。

### 2. 手動操作 + 地図表示

```bash
# 端末 1
bash run_nav2.sh --manual

# 端末 2（Nav2 の起動完了を待ってから）
bash run_rviz.sh
```

Nav2 と RViz も起動するので、地図と自己位置を見ながらキーボードで歩かせられる。
地図の妥当性を目で確かめたいときに使う。
2D Goal Pose を指定しなければ Nav2 は指令を出さないので競合しない。

### 3. 自律ナビゲーション

```bash
# 端末 1
bash run_nav2.sh
bash run_nav2.sh maps/other.yaml   # 地図を指定する場合

# 端末 2（Nav2 の起動完了を待ってから）
bash run_rviz.sh
```

RViz の「**2D Goal Pose**」で目標地点を指定すると、G1 が向き直ってから歩いて到達する。

**「2D Pose Estimate」は使わないこと。** 初期姿勢は `run_nav2.sh` が Isaac Sim の
真値から自動設定する。クリックで指定すると大きくずれる
（実測で位置 31 m / 向き 179 度）。失敗した場合は次でやり直す。

```bash
source env.sh && python3 src/set_initial_pose.py
```

### 共通の注意

- **起動に 2〜5 分かかる。** Isaac Sim の初期化とアセット取得のため。
  「反応が無い」と思っても待つこと。
- `run.sh` / `run_nav2.sh` が PYTHONPATH のブリッジを行う。
  `./isaaclab.sh` は**使えない**（理由は `../CLAUDE.md` 参照）。
- 前回のプロセスが残っていると TF が二重に配信されて RViz に現在地が出なくなる。
  `run_nav2.sh` は起動時に検出して止めるが、手動で確認するなら次を見る。

```bash
ps aux | grep -c "[r]un_g1_twin"        # 1 以下が正常
ros2 topic info /tf | grep Publisher    # 1 が正常
```

- Isaac Sim の起動が「`Waiting on global named semaphore`」で詰まる場合は
  古い共有メモリが残っている。`rm -f /dev/shm/sem.carbonite-sharedmemory`

### RViz でロボットの向きが分からないとき

RViz が表示するのは `robot_radius` から作られた**円形の footprint** なので、
回転しても形が変わらず向きが見えない。3D モデルを出す RobotModel は
`/robot_description` を必要とするが、本プロジェクトは配信していない。

向きを見たい場合は RViz に **TF** 表示を追加する。

1. 左パネル下部の **Add** をクリック
2. **TF** を選んで OK
3. 追加された TF の `Frames` で `base_link` と `laser` だけ有効にする（全部出すと見づらい）

座標軸（赤 = X 前方、緑 = Y 左）で向きが分かる。

なお `/odom` と TF 自体は正しく回転を報告している（実測で確認済み）。
表示上の問題であって、データやナビゲーションには影響しない。

## 操作

| キー | 動作 | 指令値 |
|---|---|---|
| W | 前進 | +0.6 m/s |
| S | 後退 | **-0.2 m/s**（下記参照） |
| A / D | 左移動 / 右移動 | ±0.4 m/s |
| Q / E | 左旋回 / 右旋回 | ±0.6 rad/s |
| SPACE | 停止 | 0 |
| SHIFT | 低速（微調整） | 0.35 倍 |

## 重要：後退は苦手（ポリシーの制約）

**このポリシーは前進のみで学習されている。** IsaacLab の `flat_env_cfg.py` に
`lin_vel_x = (0.0, 1.0)` と設定されており、負の値（後退）は学習範囲に入っていない。

実測した追従性能（`test_commands.py` による検証結果）。
**胴体座標系の速度**（ポリシーが追従すべき量そのもの）で評価している:

| 指令 | 胴体速度（実測） | 結果 |
|---|---|---|
| 静止 (0,0,0) | (-0.03, -0.01, -0.06) | OK（わずかに漂う） |
| 前進 +0.5 | **+0.38** | OK |
| **後退 -0.2** | **-0.21** | **OK** |
| 後退 -0.3 以上 | — | **転倒**（`cmd_test6.log`） |
| 左移動 +0.4 | **+0.38** | OK |
| 右移動 -0.4 | **-0.42** | OK |
| 左旋回 +0.3 | **+0.30 rad/s** | OK |
| 左旋回 +0.5 | **+0.51 rad/s** | OK |
| 左旋回 +1.0 | **+0.98 rad/s** | OK |
| 右旋回 -0.5 | **-0.50 rad/s** | OK |
| 右旋回 -1.0 | **-1.00 rad/s** | OK |
| 前進+旋回 (0.4, 0, 0.5) | (+0.31, 0.00, +0.52) | OK |

**旋回は非常に良く追従する**（指令とほぼ一致）。後退以外はすべて良好。

### 計測方法の注意（重要）
旋回の評価には**必ず胴体座標系の角速度 `root_ang_vel_b` を使うこと。**
ワールド座標の yaw を一定間隔の始点・終点で比較すると、回転が 2π を跨いだり
向きが一周して戻るため、**実際には正しく回っているのに 0 に近い値が出る**。
初期の検証でこの方法を使い「旋回の追従が弱い」と誤判定した。

### クランプ範囲
`command.py` のクランプ範囲を学習範囲に合わせている:
```python
LIN_VEL_X_RANGE = (-0.2, 1.0)   # 後退は -0.2 まで
LIN_VEL_Y_RANGE = (-0.5, 0.5)
ANG_VEL_Z_RANGE = (-1.0, 1.0)
```

後退を本格的に使いたい場合は、`lin_vel_x` の範囲を負まで広げて
IsaacLab で再学習する必要がある。

## 構成

```
SimEnvTest/
├── SETUP.md                # セットアップ手順（初めての人はこちら）
├── README.md               # このファイル
├── env.sh                  # Isaac Sim + IsaacLab + ROS 2 の共通環境設定
├── run.sh                  # キーボード操作で起動
├── run_nav2.sh             # Nav2 で自律走行させる
├── run_rviz.sh             # RViz を起動する
├── run_slam.sh             # slam_toolbox で地図を作る（現状は非推奨）
├── config/
│   ├── nav2.yaml           # Nav2 の設定（後退禁止など G1 向け調整）
│   ├── navigate_g1.xml     # Behavior Tree（BackUp を除いたもの）
│   ├── navigate_through_poses_g1.xml
│   └── slam_toolbox.yaml   # slam_toolbox の設定
├── src/
│   ├── run_g1_twin.py      # エントリポイント
│   ├── build_map.py        # 真値 odom から地図を作る（推奨）
│   ├── set_initial_pose.py # AMCL の初期姿勢を真値から設定
│   ├── publish_map_odom_tf.py  # map->odom を恒等変換で流す（AMCL の代替）
│   │
│   │   # --- 検証ツール ---
│   ├── check_map_alignment.py   # 地図と実シーンのずれを測る（重要）
│   ├── check_map.py             # 地図の埋まり具合
│   ├── check_scan.py            # スキャンの安定性
│   ├── check_scan_projection.py # スキャン+姿勢を自前で点群化
│   │
│   │   # --- 単体テスト（Isaac Sim 不要）---
│   ├── test_patrol.py           # 巡回ロジック（6件）
│   ├── test_build_map.py        # 地図生成（4件）
│   ├── test_scan_semantics.py   # LaserScan の規約（4件）
│   ├── test_commands.py         # コマンド追従の実測（Isaac Sim 必要）
│   │
│   │   # --- 調査用（記録として残しているもの）---
│   ├── inspect_warehouse.py     # Warehouse の prim 構造
│   ├── probe_raycaster.py       # RayCaster 版 LiDAR の検証
│   ├── probe_pose.py            # 姿勢データの形式確認
│   ├── probe_walking_yaw.py     # 歩行中の姿勢更新の確認
│   ├── probe_scan_angle.py      # スキャンの角度規約
│   ├── probe_torso_yaw.py       # torso と pelvis の姿勢差
│   ├── probe_lidar.py           # PhysX LiDAR の不具合記録（動作しない）
│   └── probe_spawn.py           # 開けた場所の探索（未完成）
│
│   └── g1_twin/
│       ├── checkpoint.py   # checkpoint の取得・キャッシュ
│       ├── command.py      # 速度コマンド + 送信先の抽象層
│       ├── keyboard.py     # キーボード入力 -> コマンド
│       ├── policy.py       # 学習済み歩行ポリシー
│       ├── runner.py       # シーン構築 + 実行ループ
│       ├── lidar.py        # 2D LiDAR（MultiMeshRayCaster）
│       ├── ros_bridge.py   # ROS 2 連携（/scan /odom /tf、/cmd_vel 購読）
│       └── patrol.py       # 自動巡回（地図作成用）
├── checkpoints/            # 学習済み checkpoint（自動ダウンロード）
├── maps/                   # 地図（.pgm / .yaml）
├── g1_joints.json          # G1 の関節順・既定姿勢・ゲイン（実測ダンプ）
└── logs/
```

## 地図作成と自律ナビゲーション

ROS 2 Jazzy + Nav2 で、地図作成から自律走行までを行う。

### 1. 地図を作る

**推奨: 真値 odom から直接作る（`build_map.py`）**

```bash
# 端末 1: 自動巡回で歩き回らせる
source env.sh
"$ISAAC_SIM/python.sh" src/run_g1_twin.py --viz none \
    --command-source patrol --max-steps 90000

# 端末 2: スキャンを集めて地図にする
source env.sh
python3 src/build_map.py --duration 1500 \
    --cache maps/scans.npz --output maps/warehouse
```

Isaac Sim は真値の姿勢を持っているため、SLAM の姿勢推定は本来不要で、
スキャンを姿勢どおりに重ねるだけでよい。`--cache` を付けるとスキャンを
`.npz` に保存でき、`--from-cache` で Isaac Sim 無しに再構築できる
（格子パラメータの調整用）。

**slam_toolbox 版（`run_slam.sh`）**

```bash
bash run_slam.sh 40000
```

実機を見据えるなら本来はこちら。ただし Warehouse は同じ形の棚が並び
2D スキャンでは特徴が乏しいため、相関スキャンマッチャが誤マッチして
地図が歪む（6 回試して安定しなかった）。設定は残してあるので、
実機連携時や別のシーンではこちらを使う。

### 2. 自律走行させる

起動方法は冒頭の「[起動](#起動)」を参照。指令の供給源は
`run_g1_twin.py` の `--command-source` で決まる。

| 値 | 指令元 | 使う場面 |
|---|---|---|
| `keyboard`（既定） | キーボード | 手動操作 |
| `patrol` | 自動巡回 | 地図作成 |
| `ros` | Nav2 の `/cmd_vel` | 自律走行 |

`runner.py` が `if/elif` で排他分岐しているため、実行中の切り替えはできない。

**動作確認済み（2026-08-10）。** RViz の 2D Goal Pose で目標を与えると、
G1 が向き直ってから歩き、目標に到達する。

    [controller_server]: Reached the goal!
    [bt_navigator]: Goal succeeded

実測: 総移動 5.24 m、経路計画の失敗 0 回、自己位置のずれ 約 0.6 m。

初期姿勢は run_nav2.sh が自動設定する（Isaac Sim の真値を使う）。
RViz の「2D Pose Estimate」は使わないこと。この地図は原点が
(-28.5, -25.4) にあり、画像上のどこがどの座標か直感的に分からないため
クリックでは大きくずれる。

### 現在の地図の品質

`maps/warehouse.pgm` は 7503 スキャン（シム内 20 分の巡回）から生成した。
721 x 1161 セル、探索面積 1430 m2。

実シーンとの照合結果（`src/check_map_alignment.py`）:

| 項目 | 実シーン | 地図 |
|---|---|---|
| X 範囲 | -26.5 〜 +5.5 m | -26.5 〜 +5.5 m |
| Y 範囲 | -23.4 〜 +30.6 m | -23.4 〜 +30.6 m |
| 実形状までの距離 | — | **中央値 0.36 m** |

範囲が完全に一致しており、Nav2 での自律走行にも成功している。

地図を作り直したら `check_map_alignment.py` で必ず検証すること。
以前 yaw の二重適用で 11 m ずれた地図を「正常」と誤認した。

### 構成

| 項目 | 選択 | 理由 |
|---|---|---|
| LiDAR | `MultiMeshRayCaster`（IsaacLab） | PhysX LiDAR がこのビルドで動かないため |
| 設置高さ | 地上 1.1 m | 低いと棚の脚だけを拾って地図が穴だらけになる |
| 姿勢追従 | `ray_alignment="yaw"` | `"base"` だと歩行の傾きでレイが上下を向く |
| オドメトリ | Sim の真値 | ドリフトが無い。地図生成もこれを直接使う |
| 時刻 | `/clock`（シム内時刻） | 実時間だとスキャンと姿勢の対応がずれる |
| ROS 接続 | `rclpy` を直接使用 | 既存のループ構造をそのまま保てる |
| 後退 | 原則禁止（`vx >= 0`） | ポリシーが -0.3 m/s 以上で転倒するため |

**後退の例外:** 巡回中に LiDAR に映らない低い障害物へ引っかかったときだけ、
転倒しない -0.2 m/s で後退して脱出する。旋回だけでは体が引っかかったままで、
実測では 21 回旋回しても 1 mm も動かなかった。

### ROS 2 のトピック

| トピック | 向き | 内容 |
|---|---|---|
| `/scan` | 配信 | 2D LiDAR（360 ビーム、10Hz、最大 30 m） |
| `/odom` | 配信 | オドメトリ（Sim の真値、50Hz） |
| `/tf` | 配信 | `odom` → `base_link`、`base_link` → `laser` |
| `/cmd_vel` | 購読 | Nav2 からの速度指令 |

## つまずいた点

開発中に踏んだ落とし穴。同じ環境で作業する人向け。

### 地図作成・SLAM 関連

**slam_toolbox はライフサイクルノード。** 起動しただけでは `/scan` を購読せず、
地図が全く作られない。`ros2 lifecycle set /slam_toolbox configure` と
`activate` が必要（`run_slam.sh` は実行済み）。エラーも出ないので気付きにくい。

**PhysX LiDAR は使えない。** `RangeSensorCreateLidar` が Imageable でない
Lidar prim の `visibility` 属性を設定しようとして例外になり、prim の作成に
失敗する（`Empty typeName for </World/Lidar.visibility>`）。加えて
`isaacsim.util.debug_draw` が undefined symbol で読み込めない
（ユーザーローカルに Isaac Sim 4.5 世代の拡張が残っており版が混在している）。
`RotatingLidarPhysX` クラスも `SimulationManager._get_backend_utils` が無く
動かない。調査の記録として `src/probe_lidar.py` を残してある。

**レイキャストが重い。** 1 回 74 ms かかり 50Hz（20 ms）に収まらない。
LiDAR の更新を配信周期（10Hz）に間引いて実時間比 0.07x → 0.5x に改善した。
ビーム数を 1/4 に減らしても 45 ms までしか下がらず、コストはビーム数ではなく
レイキャスト演算自体に支配される。SLAM はスキャンのタイムスタンプで処理する
ため、実時間より遅くても地図は正しく作られる。

**クォータニオンは `(x,y,z,w)` 順。** IsaacLab の `root_quat_w` は
`base_articulation_data.py` の docstring に "Root link orientation (x, y, z, w)"
と明記されている。ROS の `geometry_msgs` と同じ順序なのでそのまま渡せる。

これを `(w,x,y,z)` と誤解していたため、`/odom` が実際の向きをまったく
反映せず、Nav2 が旋回指令を出し続けても G1 が回らなかった。

**静止状態の検証では見つからない。** 以前 `probe_pose.py` で yaw=30 度の
静止状態を調べて「(w,x,y,z) 順」と誤った結論を出した。歩行中に初めて破綻する。
判別は初期姿勢（無回転）の生値を見るのが確実で、単位クォータニオンの
`w=1` がどの位置に来るかで分かる。

実測（500 step の旋回、期待 458 度）:

| 解釈 | root の回転量 |
|---|---|
| `(w,x,y,z)`（誤り） | +5.9 度 |
| `(x,y,z,w)`（正しい） | **+105.6 度**（458-360=98 度とほぼ一致） |

角速度 `root_ang_vel_b` は正常な値を返すので、**角速度は正しいのに姿勢が
変わらない**という矛盾がこのバグの指紋になる。

**LiDAR は `ray_alignment="yaw"` にする。** ここが今回いちばんはまった点。
`"base"`（胴体の姿勢に完全追従）にすると、歩行中の pitch/roll がレイに乗って
上下を向き、**同じ方向の距離が 1 スキャンごとに 10 m 近く暴れる**。SLAM は
スキャンを重ねられず、地図が放射状の縞模様になる。

実測（同一地点での連続スキャンの前方距離）:

| 設定 | 前方距離の推移 | 連続スキャンの差（平均） |
|---|---|---|
| `"base"` | 18.88 → 9.68 → 17.18 → 12.66 m | 1.143 m |
| `"yaw"` | 1.45 → 1.45 → 1.44 → 1.45 m | **0.033 m** |

`src/check_scan.py` でこの安定性を検査できる。地図がおかしいときは
まずこれを走らせること。

**地図は必ず画像で目視する。** `/map` は配信され、探索面積も増え続けるため、
数値だけ見ていると壊れた地図でも「正常に動いている」ように見える。実際に
`.pgm` を画像として開くまで壊れていることに気付けなかった。

```bash
python3 -c "from PIL import Image; Image.open('maps/warehouse.pgm').convert('L').save('/tmp/m.png')"
```

**自動巡回のデッドロック。** 前進の閾値（1.6 m）より旋回解除の閾値（2.2 m）が
大きいため、四方が 2.2 m 未満の狭い通路では、どちらを向いても旋回を抜けられず
その場で回り続ける。旋回は「最も開けた方向まで角度で回りきる」方式にしてある。

**近すぎる測距を inf にしてはいけない。** LaserScan の `inf` は
「最大距離まで何も無い」を意味する。近くの壁を `inf` にすると SLAM は
「30 m 先まで空き」と解釈し、地図が放射状に真っ白く塗り潰される。
実測では 360 ビーム中 89 本（25%）がこの状態だった。`range_min` 未満は
ROS の規約で「無効」として無視される `0.0` にする。

**占有格子は hit を大きく miss を小さくする。** 同じ場所を何度も通ると
レイの通過（空き）の回数が当たり（障害物）の回数を大きく上回るため、
`MISS_GAIN` が大きいと壁が消える。実測では 9385 スキャンから作った地図で
障害物が 833 セル（0.0%）しか残らなかった。`HIT_GAIN=2.0` /
`MISS_GAIN=0.05` とし、対数オッズに上下限を設けてある。

**LiDAR は足元の障害物を見ない。** 地上 1.1 m を見ているため、パレットや
棚の下段に引っかかっても「前方は開いている」と判断してしまう。実測では
前方 1.44 m と報告しながら位置が 1 mm も動かない状態が続いた。
位置が動いているかを別途監視し、動かなければ後退して脱出する。

**その場旋回は「足踏み」判定に引っかからない。** 前進指令中しか足踏みを
検知しないため、旋回を繰り返している間は見逃す。実測では全スキャンの 77% が
半径 5 cm の一点から取られ、地図は 1 スキャンが回転しただけの楕円になった。
半径 1.5 m から 15 秒出られなければ「閉じ込め」として別途検知する。

**進捗の判定に粗いログを使わない。** 状態ログは 5 秒ごとにしか出ないため、
その中のユニーク位置を数えると正常な移動でも「停滞」に見える。実際に一度
これで誤判定した。移動距離の合計など、意味のある量で判断すること。


### 基本操作・実行時の注意

**ループ内で `simulation_app.update()` を必ず呼ぶ。** これを呼ばないと
Kit の UI イベントが処理されず、**キーボードのコールバックが一切発火しない**
（キーを押しても無反応になる）。`sim.step()` と `sim.render()` だけでは足りない。

**エントリポイントを import してはいけない。** `run_g1_twin.py` はモジュール直下で
`AppLauncher` を起動するため、他のスクリプトから import すると
**アプリが二重起動して即座に落ちる**（エラーも出ない）。
共有したい処理は `g1_twin/` 配下へ置くこと。

**`finally` で `simulation_app.close()` を呼ばない。** 例外を隠して
「エラーも無く終了した」ように見える。`main()` の外で閉じる。

**起動に 2〜5 分かかる。** Isaac Sim 本体の初期化に加え、G1 の USD をリモートの
アセットサーバーから取得するため。タイムアウトを短く設定すると
「エラーも出ずに落ちた」ように見えるので、待ち時間は十分に取ること。
2 回目以降は `~/.cache/ov` にキャッシュされる。

**起動時に segfault することがある。** `blas_thread_shutdown` / `__libc_fork` で
落ちる。`OPENBLAS_NUM_THREADS=1` 等で回避する（`env.sh` / `run.sh` は設定済み）。

**静止コマンド (0,0,0) でもゆっくり漂う**（実測 約 0.04 m/s）。
このポリシーは速度追従で学習されており位置保持の項が無いため、仕様上の挙動。

**キーボード操作は GUI でしか使えない。** ヘッドレス（`--viz none`）では
`omni.appwindow` が import できずエラーになる。

## 歩行ポリシー

NVIDIA 公開の学習済み checkpoint をそのまま使う（**自前学習は不要**）。

```
.../Assets/Isaac/6.0/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl/Isaac-Velocity-Flat-G1-v0/checkpoint.pt
```

- obs **123** 次元 / action **37** 次元 / actor [256,128,128] / 非リカレント ELU MLP
- ライセンス BSD-3-Clause（商用可）
- 初回起動時に `checkpoints/` へ自動ダウンロードしてキャッシュする

### 観測の構成（順序を変えると歩かなくなる）

IsaacLab の `velocity_env_cfg.py` の `PolicyCfg` と一致させている。

| 範囲 | 内容 |
|---|---|
| `[0:3]` | `base_lin_vel` 胴体座標系の並進速度 |
| `[3:6]` | `base_ang_vel` 胴体座標系の角速度 |
| `[6:9]` | `projected_gravity` 胴体座標系の重力方向 |
| `[9:12]` | `velocity_commands` (vx, vy, yaw_rate) |
| `[12:49]` | `joint_pos - default` (37) |
| `[49:86]` | `joint_vel` (37) |
| `[86:123]` | `last_action` (37) |

**注意点:**
- **観測にスケーリングは掛けない。** IsaacLab の `PolicyCfg` にスケール指定が無い。
  Unitree 公式リポジトリのポリシーは 0.25 / 0.05 等を掛けるので混同しないこと。
- アクションは **scale 0.5** を既定関節位置に加算する（`use_default_offset=True` 相当）。
- Flat タスクは `height_scan` を使わない（obs 123）。Rough は +187 で 310 になる。

### 関節順（37 自由度）

`g1_joints.json` に実測値をダンプしてある。**解剖学的な並びではなく
アーティキュレーション木の幅優先順**なので、手で書き起こさずこの JSON を参照すること。

```
 0 left_hip_pitch    1 right_hip_pitch   2 torso        3 left_hip_roll
 4 right_hip_roll    5 left_sho_pitch    6 right_sho_pitch  ...
23..36 ハンド関節（five/three/zero/six/four/one/two）
```

action 37 はハンド関節を含む全身の関節位置制御。12 自由度の脚のみではない。

## 実機連携

`command.py` の `CommandSink` を実装すれば送信先を差し替えられる。

| 実装 | 状態 |
|---|---|
| `SimCommandSink` | 利用可能（シミュレーション内） |
| `DdsCommandSink` | **未実装**（unitree_sdk2py 経由、実機接続時に実装） |
| `Ros2CommandSink` | **未実装**（ROS2 Twist、必要になった時点で実装） |

未実装のものはコンストラクタで `NotImplementedError` を投げる。
`unitree_sdk2py` は未インストールなので、実機連携時は導入も必要。

## コマンド追従の自動検証

キーボードを使わずに、各方向の指令へ追従しているかを確認できる。

```bash
cd /home/spacedata/isaac_dev/G1/SimEnvTest
S=/home/spacedata/isaacSim6.0dev2/_build/linux-x86_64/release
SP=/home/spacedata/IsaacLab/env_isaaclab/lib/python3.12/site-packages
PP=$(ls -d /home/spacedata/IsaacLab/source/*/ | tr '\n' ':')
PYTHONPATH="$PWD/src:$PP$SP" OPENBLAS_NUM_THREADS=1 \
  $S/python.sh src/test_commands.py --viz none
```

前進・後退・左右移動・左右旋回を順に与え、指令値と実測値を並べて出力する。

## キー操作の使い方

1. **GUI ウィンドウをクリックしてフォーカスを当てる**（これをしないとキーが届かない）
2. 上記のキーを押す。押している間だけ有効で、離すと停止する。
3. 同時押しも可能（W+Q で「前進しながら左旋回」など）

キーを押すとログに `[G1] 指令変更: vx=+0.60 ...` が出る。
**入力が効いているかはこのログで判別できる。**

動作確認済み（2026-08-04、GUI で実操作）: W / S / A / D / Q / E および
同時押し（W+D, W+E）が正しく指令に反映され、操作中に転倒しないことを確認。

## 既知の問題

- **`isaacsim.util.debug_draw` が読み込めない。** ユーザーローカル
  （`~/.local/share/ov/data/exts/v2/`）に Isaac Sim 4.5 世代（cp311）の拡張が
  残っており、6.0（cp312）と版が混在しているため。歩行・地図作成には影響しないが、
  PhysX LiDAR が使えない原因の一つになっている。

- **長距離のナビゲーションは成功率が下がる。** 実時間比 0.23x（GUI 込み）で
  Nav2 の制御周期と噛み合わないこと、地図に未探索領域が残ること、
  G1 が後退で立て直せないことが要因。改善するなら headless で動かし、
  巡回時間を延ばして地図の探索範囲を広げる。

- **slam_toolbox では安定した地図が作れなかった。** Warehouse は同じ形の棚が
  並び 2D スキャンでは特徴が乏しいため、相関スキャンマッチャが誤マッチする。
  設定は `config/slam_toolbox.yaml` に残してあるので、実機連携時や
  別のシーンでは再挑戦する価値がある。なお当時は yaw の二重適用バグ
  （下記）が入っていたため、修正後なら結果が変わる可能性がある。
