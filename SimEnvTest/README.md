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
├── run.sh                  # 起動スクリプト（PYTHONPATH ブリッジ）
├── src/
│   ├── run_g1_twin.py      # エントリポイント
│   ├── test_commands.py    # コマンド追従の自動検証（キーボード不要）
│   └── g1_twin/
│       ├── checkpoint.py   # checkpoint の取得・キャッシュ
│       ├── command.py      # 速度コマンド + 送信先の抽象層
│       ├── keyboard.py     # キーボード入力 -> コマンド
│       ├── policy.py       # 学習済み歩行ポリシー
│       └── runner.py       # シーン構築 + 実行ループ
├── checkpoints/            # 学習済み checkpoint（自動ダウンロード）
├── g1_joints.json          # G1 の関節順・既定姿勢・ゲイン（実測ダンプ）
└── logs/
```

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
