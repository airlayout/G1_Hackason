# G1 デジタルツイン操作環境

Unitree G1 を Warehouse シーン上でキーボード操作する環境。
将来の実機連携を見据え、コマンド送信部を差し替え可能な抽象層にしている。

## 起動

```bash
cd /home/spacedata/isaac_dev/G1/SimEnvTest
bash run.sh                # Warehouse シーン（GUI）
bash run.sh --flat         # 平地のみ（動作確認用）
bash run.sh --viz none     # ヘッドレス
```

`run.sh` が PYTHONPATH のブリッジと `--viz kit` の付与を行う。
`./isaaclab.sh` は**使えない**（理由は `../CLAUDE.md` 参照）。

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
├── run.sh                  # 起動スクリプト（キーボード操作）
├── run_slam.sh             # 自動巡回で地図を作る（SLAM）
├── run_nav2.sh             # 作った地図で自律走行する（Nav2）
├── env.sh                  # Isaac Sim + IsaacLab + ROS 2 の共通環境設定
├── config/
│   ├── slam_toolbox.yaml   # slam_toolbox の設定
│   └── nav2.yaml           # Nav2 の設定（後退禁止など G1 向け調整）
├── src/
│   ├── run_g1_twin.py      # エントリポイント
│   ├── test_commands.py    # コマンド追従の自動検証（キーボード不要）
│   ├── build_map.py        # 真値 odom から地図を作る（推奨）
│   ├── check_map.py        # 地図の埋まり具合を数値で確認する
│   ├── check_scan.py       # スキャンの安定性を検査する
│   ├── check_scan_projection.py # スキャン+姿勢を自前で点群化（切り分け用）
│   ├── test_patrol.py      # 巡回ロジックの単体テスト
│   ├── test_build_map.py   # 地図生成の単体テスト
│   ├── test_scan_semantics.py # LaserScan の規約の回帰テスト
│   ├── inspect_warehouse.py # Warehouse の prim 構造調査（調査用）
│   ├── probe_raycaster.py  # RayCaster 版 LiDAR の検証（調査用）
│   ├── probe_pose.py       # 姿勢データの形式確認（調査用）
│   ├── probe_scan_angle.py # スキャンの角度規約の検証（調査用）
│   ├── probe_lidar.py      # PhysX LiDAR の不具合記録（調査用・動作しない）
│   ├── probe_spawn.py      # 開けた場所の探索（調査用・未完成）
│   └── g1_twin/
│       ├── checkpoint.py   # checkpoint の取得・キャッシュ
│       ├── command.py      # 速度コマンド + 送信先の抽象層
│       ├── keyboard.py     # キーボード入力 -> コマンド
│       ├── policy.py       # 学習済み歩行ポリシー
│       ├── runner.py       # シーン構築 + 実行ループ
│       ├── lidar.py        # SLAM 用 2D LiDAR（MultiMeshRayCaster）
│       ├── ros_bridge.py   # ROS 2 連携（/scan /odom /tf、/cmd_vel 購読）
│       └── patrol.py       # 自動巡回（地図作成用）
├── checkpoints/            # 学習済み checkpoint（自動ダウンロード）
├── maps/                   # SLAM で作った地図（.pgm / .yaml）
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

```bash
bash run_nav2.sh                 # Isaac Sim + Nav2
bash run_rviz.sh                 # 別ターミナルで RViz
```

RViz の「2D Pose Estimate」で現在位置を教えてから「2D Goal Pose」で
目標を指定すると、G1 が自律的に歩いて到達する。

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

### つまずいた点

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

**クォータニオンの順序が違う。** IsaacLab の `root_quat_w` は `(w,x,y,z)`、
ROS の `geometry_msgs` は `(x,y,z,w)`。取り違えると地図が回転して壊れる。

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

## 実装上の注意（つまずいた点）

- **ループ内で `simulation_app.update()` を必ず呼ぶ。** これを呼ばないと
  Kit の UI イベントが処理されず、**キーボードのコールバックが一切発火しない**
  （キーを押しても無反応になる）。`sim.step()` と `sim.render()` だけでは足りない。
  IsaacLab の teleop スクリプト（`teleop_se3_agent.py` 等）も同様に呼んでいる。

- **エントリポイントを import してはいけない。** `run_g1_twin.py` はモジュール直下で
  `AppLauncher` を起動するため、他のスクリプトから import すると
  **アプリが二重起動して即座に落ちる**（エラーも出ない）。
  共有したい処理は `g1_twin/` 配下へ置くこと（`checkpoint.py` がその例）。
- **`finally` で `simulation_app.close()` を呼ばない。** 例外を隠して
  「エラーも無く終了した」ように見える。IsaacLab の参考実装と同じく
  `main()` の外で閉じる。
- 起動時に `blas_thread_shutdown` / `__libc_fork` で segfault することがある。
  `OPENBLAS_NUM_THREADS=1` 等で回避する（`run.sh` は設定済み）。

## 実行時の注意

- **起動に 2〜5 分かかる。** Isaac Sim 本体の初期化に加え、G1 の USD をリモートの
  アセットサーバーから取得するため。タイムアウトを短く設定すると
  「エラーも出ずに落ちた」ように見えるので、待ち時間は十分に取ること。
- Warehouse シーン（`full_warehouse.usd`）は本体 6.8MB だが、マテリアル・テクスチャを
  多数の個別ファイルとして参照するため初回ロードは特に時間がかかる。
  2 回目以降は `~/.cache/ov` にキャッシュされる。
- **静止コマンド (0,0,0) でもゆっくり漂う**（実測 約 0.04 m/s）。
  このポリシーは速度追従で学習されており位置保持の項が無いため、仕様上の挙動。
  完全に止めたい場合は別途位置制御を重ねる必要がある。

## 既知の問題

- `isaacsim.util.debug_draw` が `libtbb.so.2` 不足で読み込めない。
  歩行には影響しないが、センサー可視化に使う場合は `libtbb2` の導入が必要（sudo 権限）。
