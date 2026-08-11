# セットアップ手順

このプロジェクトをゼロから動かすための手順。所要時間は環境構築込みで 1〜2 時間、
初回のアセット取得に数分〜十数分かかる。

## 必要なもの

| 項目 | 要件 | 備考 |
|---|---|---|
| OS | Ubuntu 24.04 | 開発環境。他の版は未検証 |
| GPU | NVIDIA、VRAM 8GB 以上 | 検証環境は L40S (46GB) |
| Isaac Sim | 6.0.0-rc.22 または 6.0.1.0（pip 版） | いずれも動作確認済み。5.1 以下は非対応 |
| IsaacLab | `release/3.0.0-beta2` 以降 | `source/` が editable install されていること。`main`（v2.3.x）は Isaac Sim 6.0 と非互換 |
| ROS 2 | Jazzy | Python 3.12 であることが重要（後述） |
| ディスク | 20GB 以上 | Isaac Sim のアセットキャッシュを含む |

ネットワーク接続が必要。Warehouse シーンと歩行ポリシーの checkpoint を
NVIDIA のアセットサーバーから取得する。

## 1. Isaac Sim と IsaacLab

インストール手順は本プロジェクトの範囲外。NVIDIA の公式手順に従うこと。

**Isaac Sim の入手方法（ソースビルド版 / pip 版 / バイナリ版）やインストール先は
ユーザーごとに異なる。** 以下は実際に動作確認できた環境の例であり、
これ以外の構成を否定するものではない。`env.sh` の3行（`ISAAC_SIM` /
`ISAACLAB` / `ROS_SETUP`）を自分の環境に合わせて書き換えれば、
どの入手方法でも基本的には動くはず。

インストール後、以下のパスを控えておく。

```bash
# 例（検証環境のパス。ソースビルド版 Isaac Sim の場合）
ISAAC_SIM=/home/devuser/isaacSim6.0dev2/_build/linux-x86_64/release
ISAACLAB=/home/devuser/IsaacLab
```

### 動作確認

```bash
"$ISAAC_SIM/python.sh" -c "import isaacsim; print('Isaac Sim OK')"
ls "$ISAACLAB/source"   # isaaclab, isaaclab_assets, isaaclab_tasks などがあること
```

### 注意: `isaaclab.sh` は使えないことがある

検証環境では isaacsim と isaaclab が別々の Python に入っており、
`isaaclab.sh` が選ぶ venv では `isaacsim` を import できなかった。

本プロジェクトの `env.sh` は PYTHONPATH で両方を繋ぐ方式を取っている。
同じ問題が起きていなければそのまま動く。

**`isaaclab.sh` は Isaac Sim が見つからなくても終了コード 0 を返す。**
成功したように見えるので、必ずログ本文でエラーを確認すること。

### 環境の例②: pip 版 Isaac Sim を使う場合（2026-08-11 検証済み）

上記はソースビルド版 Isaac Sim を使う環境の例だったが、**別のユーザーが
別マシンで pip 版 Isaac Sim を使ってこのプロジェクトを動かすケースもある。**
実際に `pip install "isaacsim[all,extscache]==6.0.1.0"` で入れた Isaac Sim +
IsaacLab `release/3.0.0-beta2` の組み合わせでも、`SimEnvTest/run.sh`
（Warehouse, 自動巡回 2000 step）の完走を確認済み。この場合は以下の
追加対応が必要になる（ソースビルド版では不要な対応も含む）。

**1. `python.sh` が存在しない。** pip 版は venv に `isaacsim` パッケージが
入っているだけで、ソースビルド版のような `python.sh` ランチャーが無い。
venv 直下に薄いラッパーを置けば `env.sh` の仕組みがそのまま使える。

```bash
cat > /path/to/env_isaaclab/python.sh << 'EOF'
#!/bin/bash
exec "$(dirname "${BASH_SOURCE[0]}")/bin/python" "$@"
EOF
chmod +x /path/to/env_isaaclab/python.sh
```

`env.sh` の `ISAAC_SIM` にはこの venv のパスをそのまま指定する
（`isaacsim` と `isaaclab` が同じ venv に同居していれば `ISAACLAB` は
IsaacLab リポジトリのパスのまま、`LAB_SITE_PACKAGES` は `env.sh` が
自動的に `$ISAAC_SIM/lib/python3.12/site-packages` を見るようになっている）。

**2. `isaacsim.core.utils` / `isaacsim.storage.native` が import できない。**
Isaac Sim 6.0.0 で非推奨（`isaacsim.core.experimental.utils` に統合）となり、
IsaacLab 標準の `.kit` アプリ構成では既定で読み込まれなくなった
（ソースビルド版でも起きるはずだが、`devuser` 環境では別の理由で
気付かれなかった可能性がある）。`g1_twin/runner.py` は次のように
IsaacLab 側の同等 API に置き換え済み:

| 旧（動かない） | 新（動く） |
|---|---|
| `isaacsim.core.utils.stage.add_reference_to_stage(usd_path=, prim_path=)` | `isaaclab.sim.add_reference_to_stage(usd_path=, path=)`（引数名が変わる） |
| `isaacsim.storage.native.get_assets_root_path()` | `isaaclab.utils.assets.NUCLEUS_ASSET_ROOT_DIR`（関数ではなく定数） |
| `isaacsim.core.utils.prims`（`runner.py` では未使用のimportだった） | 削除 |

自分で別のスクリプトを書く場合も同じ置き換えが必要になる可能性が高い。

**3. `/tmp/Assets` の権限エラー。** IsaacLab はダウンロードしたアセットを
`$TMPDIR`（既定 `/tmp`）配下の固定パス `/tmp/Assets/...` にキャッシュする。
共有マシンで別ユーザーが先に `/tmp/Assets` を作っていると
`PermissionError: [Errno 13] Permission denied` になる。
`env.sh` は自分専用のキャッシュディレクトリを `TMPDIR` に設定することで
回避している（他ユーザーのファイルには一切触れない）。

**4. Isaac Sim 6.0 系は IsaacLab `main`（v2.3.x 系）と非互換。**
`main` ブランチは PhysX の内部 API（`omni.physics.tensors.impl` や
`PhysxSchema.PhysxDeformableBodyAPI` など）が Isaac Sim 6.0 で変更された
影響を受け、起動直後にクラッシュする。**必ず `release/3.0.0-beta2` 以降を
使うこと。**

## 2. ROS 2 Jazzy

```bash
# インストール（公式手順に従う）
sudo apt install ros-jazzy-desktop

# 本プロジェクトが使うパッケージ
sudo apt install \
    ros-jazzy-navigation2 \
    ros-jazzy-nav2-bringup \
    ros-jazzy-slam-toolbox \
    ros-jazzy-rviz2
```

### 重要: Python のバージョンを合わせる

Isaac Sim と ROS 2 が**同じ Python バージョン**である必要がある。
検証環境ではどちらも 3.12 で、これにより Isaac Sim の `python.sh` から
`rclpy` を直接 import できている。

```bash
# 両方が 3.12 であることを確認
"$ISAAC_SIM/python.sh" -c "import sys; print('Isaac Sim:', sys.version.split()[0])"
python3 -c "import sys; print('ROS 2:', sys.version.split()[0])"
```

バージョンが違う場合、`rclpy` の import で ABI 不整合が起きる。
その場合は ROS 2 を別プロセスにして DDS 経由で通信する構成に変える必要があり、
本プロジェクトの `ros_bridge.py` は使えない。

### 疎通確認

```bash
source /opt/ros/jazzy/setup.bash
"$ISAAC_SIM/python.sh" -c "
import rclpy
from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
import tf2_ros
print('rclpy OK:', rclpy.__file__)
"
```

## 3. Python パッケージ

地図の生成と検証に使う。ROS 2 側の Python に入れる。

```bash
python3 -c "import numpy, scipy, PIL, yaml" || \
    pip install numpy scipy pillow pyyaml
```

`torch` は Isaac Sim に同梱されているので別途インストールは不要。

## 4. このプロジェクトの設定

`env.sh` の先頭にあるパスを自分の環境に合わせる。

```bash
cd <このリポジトリ>/SimEnvTest
vi env.sh
```

```bash
ISAAC_SIM=/home/devuser/isaacSim6.0dev2/_build/linux-x86_64/release  # ← 変更
ISAACLAB=/home/devuser/IsaacLab                                      # ← 変更
ROS_SETUP=/opt/ros/jazzy/setup.bash                                    # ← 必要なら
```

**書き換えが必要なのはこの 3 行だけ。** 他のスクリプトは `env.sh` を
source するか、自分のファイル位置から相対でパスを求めるので、
リポジトリをどこに置いても動く。

`config/nav2.yaml` に Behavior Tree の絶対パスが書いてあるが、
`run_nav2.sh` が起動時に実際の場所へ書き換えるため触らなくてよい
（Nav2 は設定内の環境変数を展開しないため絶対パスが必要）。

### 動作確認

```bash
source env.sh
echo "$ISAAC_SIM"                    # パスが表示されること
echo "$PYTHONPATH" | tr ':' '\n' | head -5   # IsaacLab と ROS が入っていること
```

パスが間違っていると `env.sh` が検出して止まる。

```
[NG] Isaac Sim が見つかりません: /opt/nonexistent/isaacsim
     env.sh の ISAAC_SIM を自分の環境に合わせてください。
     探す場合: find / -name python.sh -path '*isaac*' 2>/dev/null
```

ROS 2 が無い場合は警告だけ出して続行する（キーボード操作のみ使える）。

## 5. 歩行ポリシーの checkpoint

リポジトリに含まれている（`checkpoints/g1_flat_checkpoint.pt`、2.0MB）。

無い場合は初回実行時に自動でダウンロードされる。取得元は
IsaacLab の PretrainedCheckpoints（`Isaac-Velocity-Flat-G1-v0`）。

```
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl/Isaac-Velocity-Flat-G1-v0/checkpoint.pt
```

## 6. 最初の動作確認

### キーボード操作（GUI）

```bash
bash run.sh
```

起動に 2〜5 分かかる（アセット取得を含む）。Warehouse シーンに G1 が現れ、
W / S / A / D / Q / E で操作できれば成功。

**キーボード操作は GUI（`--viz kit`）でしか使えない。** ヘッドレスでは
`omni.appwindow` が import できずエラーになる。

### 単体テスト（Isaac Sim 不要）

```bash
source env.sh
python3 src/test_patrol.py          # 巡回ロジック（6件）
python3 src/test_build_map.py       # 地図生成（4件）
python3 src/test_scan_semantics.py  # LaserScan の規約（4件）
```

すべて成功すればロジック部分は正常。

### コマンド追従の検証（GUI 不要）

```bash
source env.sh
"$ISAAC_SIM/python.sh" src/test_commands.py --viz none
```

前進・横移動・旋回の追従性能を実測する。10 分程度かかる。

## 7. 地図を作る

```bash
# 端末 1: 自動巡回
source env.sh
"$ISAAC_SIM/python.sh" src/run_g1_twin.py --viz none \
    --command-source patrol --max-steps 60000

# 端末 2: スキャンを集めて地図にする（起動完了後）
source env.sh
python3 src/build_map.py --duration 1200 \
    --cache maps/scans.npz --output maps/warehouse
```

20 分程度。`--viz none`（ヘッドレス）で回すこと。GUI 付きだと
実時間比が 0.23x まで落ち、同じ時間で集まるデータが半分以下になる。

### 地図の検証（必須）

```bash
source env.sh
"$ISAAC_SIM/python.sh" src/check_map_alignment.py
```

「実形状までの距離 中央値 0.36 m」程度なら良好。1 m を超えていたら作り直す。

**必ず画像でも目視すること。** 数値だけでは壊れた地図を検出できない。

```bash
python3 -c "
from PIL import Image
im = Image.open('maps/warehouse.pgm').convert('L')
im.resize((im.width//2, im.height//2)).save('/tmp/map.png')
"
```

壁が直線として出ていれば正常。放射状の縞や楕円になっていたら壊れている。

## 8. 自律走行させる

```bash
# 端末 1
bash run_nav2.sh              # 自律走行
bash run_nav2.sh --manual     # 手動操作（地図を見ながらキーボードで歩かせる）

# 端末 2（Nav2 の起動完了後、2〜5 分かかる）
bash run_rviz.sh
```

手動と自律は排他で、実行中には切り替えられない。起動時に決めること。

RViz で「2D Goal Pose」をクリックし、地図上の白い場所へドラッグする。
G1 が向き直ってから歩き出せば成功。

**RViz でロボットの向きが分からない場合**は、表示が円形の footprint だけで
向きが見えないため。Add → TF を追加すると座標軸で向きが分かる。
詳細は [README.md](README.md) の「起動」を参照。

**「2D Pose Estimate」は使わないこと。** 初期姿勢は `run_nav2.sh` が
Isaac Sim の真値から自動設定する。クリックで指定すると大きくずれる
（実測で位置 31 m / 向き 179 度）。

## つまずきやすい点

### 起動に失敗する

```bash
# ログを確認する
tail -50 logs/g1_twin.log      # キーボード操作
tail -50 logs/nav2.log         # Nav2
tail -50 logs/nav2_sim.log     # Nav2 使用時の Isaac Sim
```

### 起動時に segfault する

OpenBLAS と Isaac Sim の fork の相性問題。`env.sh` で以下を設定済み。

```bash
export OPENBLAS_NUM_THREADS=1
export OMP_NUM_THREADS=1
export MKL_NUM_THREADS=1
```

### Nav2 のノードが起動しない

`lifecycle_manager` が「Failed to bring up all requested nodes」で
全体の起動を中止している可能性がある。

```bash
grep -E "not initialized|Failed to bring up" logs/nav2.log
```

不足パラメータのあるノード名が出る。`config/nav2.yaml` に設定を追加する。
標準設定（`/opt/ros/jazzy/share/nav2_bringup/params/nav2_params.yaml`）から
流用するのが早い。

### RViz に「navigate_to_pose action server is not available」

初期姿勢とは無関係で、Nav2 のノードが起動していない。上の項目を確認する。

### G1 が動かない / その場で回り続ける

```bash
source /opt/ros/jazzy/setup.bash
# 指令が届いているか
ros2 topic echo /cmd_vel --once
# 自己位置が合っているか
ros2 topic echo /odom --once --field pose.pose.position
ros2 topic echo /amcl_pose --once --field pose.pose.position
```

自己位置が大きくずれていたら初期姿勢を設定し直す。

```bash
source env.sh
python3 src/set_initial_pose.py
```

### GPU メモリが足りない

Isaac Sim のプロセスが残っていることが多い。

```bash
ps aux | grep -E "run_g1_twin|isaac"
nvidia-smi --query-compute-apps=pid,used_memory --format=csv
```

## 参考

- 実装の詳細と、開発中に踏んだ落とし穴は [README.md](README.md) を参照
- プロジェクト全体の方針は `../CLAUDE.md` を参照
