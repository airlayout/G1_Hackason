# G1 プロジェクト

Unitree G1 をデジタルツイン上で操作する環境を構築するプロジェクト。

## 環境（このプロジェクト専用）

**このプロジェクトでは以下の Isaac Sim を使用する。**
親ディレクトリ `~/isaac_dev/CLAUDE.md` に記載のパスではなく、こちらを優先すること。

- Isaac Sim: `/home/devuser/isaacSim6.0dev2/_build/linux-x86_64/release`
  - バージョン: 6.0.0-rc.22
- Python: `/home/devuser/isaacSim6.0dev2/_build/linux-x86_64/release/python.sh` で実行
- IsaacLab: `/home/devuser/IsaacLab`（`isaaclab.sh`）
- GPU: NVIDIA L40S (46GB)

※ この指定は G1 プロジェクト配下のみに適用される。`~/isaac_dev/` 配下の他プロジェクト
（Cosmos, Gr00t, IsaacLab_dev など）はそれぞれの環境設定に従う。

## 重要：編集してよいファイル
- `~/isaac_dev/` 以下のみ編集すること
- `/home/devuser/isaacSim6.0dev2` 配下は絶対に編集しない
- `/home/devuser/IsaacLab` 配下は絶対に編集しない（設定は本プロジェクト内に置く）

## 別環境での動作実績（2026-08-11、`ubuntu` ユーザーで確認済み）

上記は `devuser` ユーザー環境（ソースビルド版 Isaac Sim）専用の設定。
**このリポジトリは複数のユーザー・複数の環境（ソースビルド版 / pip 版など）で
使われることを想定しており、上記はその一つの事例に過ぎない。**
別ユーザー・別マシンで動かす場合は、`SimEnvTest/SETUP.md` の
「環境の例②: pip 版 Isaac Sim を使う場合」を参照すること。実際に以下の
組み合わせで `SimEnvTest/run.sh`（Warehouse, 自動巡回, 2000 step）の
完走を確認済み:

- Isaac Sim: **6.0.1.0**（pip 版、`pip install "isaacsim[all,extscache]==6.0.1.0"`）
- IsaacLab: **3.0.0-beta2**（`release/3.0.0-beta2` ブランチ。main/v2.3.2 は
  Isaac Sim 6.0 系と非互換なので使わないこと）
- 詳細な移行手順・つまずいた点は `SimEnvTest/README.md` の
  「pip 版 Isaac Sim（6.0.1）固有のつまずき」を参照。

## プロジェクト方針（2026-08-04 決定）

### 目的
実機 G1 の遠隔操作・可視化。デジタルツイン上で操作し、将来は実機と連携する。

### 制御レベル
上位コマンド（速度指令 `vx` / `vy` / `yaw_rate`）を与えて歩行させる。
関節レベルの制御は歩行ポリシーが担当する。

### 歩行の実現方法
**IsaacLab で G1 の歩行ポリシーを自分で学習する。**

理由: Isaac Sim 6.0 同梱の学習済みポリシー（`isaacsim.robot.policy.examples`）は
anymal / spot / h1 / franka の4つのみで、**G1 用は存在しない**。
一方 IsaacLab には G1 の速度追従タスクが標準で用意されている:
- `source/isaaclab_tasks/isaaclab_tasks/manager_based/locomotion/velocity/config/g1/`
  - `flat_env_cfg.py` / `rough_env_cfg.py` / `agents/rsl_rl_ppo_cfg.py`
- ロボット定義: `source/isaaclab_assets/isaaclab_assets/robots/unitree.py` の
  `G1_CFG` / `G1_MINIMAL_CFG`

学習したポリシーは Isaac Sim 側で `PolicyController`（H1 の実装が参考になる:
`exts/isaacsim.robot.policy.examples/isaacsim/robot/policy/examples/robots/h1.py`）
と同じ形で読み込んで使う。

### 入力デバイス
キーボード（WASD 等で `vx` / `vy` / `yaw_rate` を与える）。

### シーン
Isaac Sim 標準の Simple Warehouse を使用。
`{assets_root}/Isaac/Environments/Simple_Warehouse/full_warehouse.usd`

### 実機連携
現時点で実機は接続できないため、**送信部を抽象層（インターフェース）にしておき、
Sim / unitree_sdk2 (DDS) / ROS2 を差し替えで選べる形**にする。
- 実機は存在するが今は繋がらない
- `unitree_sdk2py` は未インストール

### 実行形式
**スタンドアロン**（`python.sh` で起動）。
親 CLAUDE.md の「Script Editor 主流」とは異なり、本プロジェクトはスタンドアロンを採用する。
キーボード入力の扱いとデバッグのしやすさを優先。

```bash
cd /home/devuser/isaacSim6.0dev2/_build/linux-x86_64/release && ./python.sh <スクリプト>
```

## アセット（2026-08-04 疎通確認済み）

アセットルート（`isaacsim.storage.native` の既定値、`extension.toml` で定義）:
```
https://omniverse-content-staging.s3-us-west-2.amazonaws.com/Assets/Isaac/6.0
```
`get_assets_root_path()` がこの URL を返すため、コード内ではハードコードせず
`get_assets_root_path()` を使う。

### G1 の USD（存在を HTTP 200 で確認済み）
- `/Isaac/Robots/Unitree/G1/g1.usd` … 基本モデル
- `/Isaac/Robots/Unitree/G1/configuration/g1_29dof_with_hand_rev_1_0_robot.usd` … 29自由度＋ハンド
- `/Isaac/Robots/Unitree/G1/configuration/backpack/g1_29dof_NVBP.usd` … バックパック付き
- ハンドの選択肢: `inspire_hand/`（Inspire ハンド）、`three_finger_hand/`（3指ハンド）

### 学習済みポリシー

`/Isaac/Samples/Policies/`（Isaac Sim の PolicyController 用）配下は
Anymal / Franka / H1 / Spot / go2 / h1 のみで **G1 用は無い**。

しかし **IsaacLab の PretrainedCheckpoints には G1 が存在する**（2026-08-04 実測検証済み）。
ベース URL:
```
https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.0/Isaac/IsaacLab/PretrainedCheckpoints/rsl_rl/<task>/checkpoint.pt
```

| Task | サイズ | iter | obs | action | actor 構成 |
|---|---|---|---|---|---|
| `Isaac-Velocity-Flat-G1-v0` | 2,027,170 B | 1499 | **123** | **37** | [256, 128, 128] |
| `Isaac-Velocity-Rough-G1-v0` | 7,842,530 B | 2999 | **310** | 37 | [512, 256, 128] |

実際に `torch.load` してテンソル形状を確認済み（推測ではない）:
`actor.0.weight (256,123)` → `(128,256)` → `(128,128)` → `(37,128)`、**非リカレント MLP**。

- obs 123 の内訳: `base_lin_vel(3) + base_ang_vel(3) + projected_gravity(3)
  + velocity_commands(3) + joint_pos(37) + joint_vel(37) + last_action(37)`
- obs 310 = 123 + height_scan 187（11×17 raycast）。`flat_env_cfg.py` は
  `height_scan = None` を設定するため Flat は height_scan 不要。
- **action = 37**: `G1_CFG` はハンド関節を含む `g1.usd` を使うため 37 DOF。12 DOF の
  脚のみポリシーではなく全身の関節位置制御。
- ライセンス: BSD-3-Clause（IsaacLab）＝商用可

利用方法:
```bash
cd /home/devuser/IsaacLab
./isaaclab.sh -p scripts/reinforcement_learning/rsl_rl/play.py \
  --task Isaac-Velocity-Flat-G1-v0 --use_pretrained_checkpoint --num_envs 16
```

### 検証結果（2026-08-04）：**この checkpoint で歩行することを確認済み**

GUI で 16 体を動かし、目視で正常に歩行することを確認した。
**したがって当初の「IsaacLab で自分で学習する」方針は不要。この checkpoint を使う。**

- checkpoint は **6.0 のパスから取得された**（`Assets/Isaac/6.0/...`）。
  Isaac Sim 6.0 と同一バージョンなので版ずれの心配はない。
- 物理 200Hz（step 0.005）/ 環境・描画 50Hz（step 0.02）
- 検証コマンド:
  ```bash
  $S/python.sh scripts/reinforcement_learning/rsl_rl/play.py \
    --task Isaac-Velocity-Flat-G1-v0 --use_pretrained_checkpoint \
    --num_envs 16 --viz kit
  ```
- ログ: `SimEnvTest/logs/g1_play_gui.log`

### 代替候補（公開ポリシー）
1. `unitreerobotics/unitree_rl_lab` — Apache-2.0、IsaacLab 2.3.0 ネイティブ。
   `deploy/robots/g1_29dof/config/policy/velocity/v0/exported/policy.onnx`
   （obs **480** = history 5×96、action **29**）。`deploy.yaml` に obs 定義・kp/kd
   ・コマンド範囲が全記載。`base_lin_vel` を使わないため実機向けに堅牢。
   ただし `joint_ids_map` による関節順の並び替えが必要。
2. `unitreerobotics/unitree_rl_gym` — BSD-3-Clause、`deploy/pre_train/g1/motion.pt`
   （obs 47 / action **12** の脚のみ）。ただし config が `ActorCriticRecurrent`
   （LSTM）なので隠れ状態の持ち回りが必要な可能性。要検証。
3. `LeCAR-Lab/ASAP` — MIT。ONNX 15 本同梱だが大半は motion tracking。移植コスト高。

重みが無く自前学習が必要なもの（却下）: `fan-ziqi/robot_lab`（コードのみ）、
`HumanoidVerse`（重み0）、`NVlabs/ProtoMotions`（LFS ポインタのみ）。

## IsaacLab の実行方法（重要：環境が分裂している）

**`./isaaclab.sh` は使えない。** この環境では isaacsim と isaaclab が別々の Python に
入っており、`isaaclab.sh` が選ぶ `env_isaaclab` では Isaac Sim が import できない。

- `/home/devuser/IsaacLab/env_isaaclab/bin/python` … **isaaclab のみ**（`isaacsim` なし）
  - `/usr/bin/python3.12` 上の venv で `include-system-site-packages = false`
- Isaac Sim の `python.sh` … **isaacsim のみ**（`isaaclab` なし）
- `/home/devuser/IsaacLab/_isaac_sim` → `isaacSim6.0dev2` へのシンボリックリンク（正しい）

### 正しい起動方法（PYTHONPATH でブリッジする）
```bash
cd /home/devuser/IsaacLab
S=/home/devuser/isaacSim6.0dev2/_build/linux-x86_64/release
SP=/home/devuser/IsaacLab/env_isaaclab/lib/python3.12/site-packages
PP=$(ls source/*/ -d | sed "s|^|/home/devuser/IsaacLab/|" | tr '\n' ':')
export PYTHONPATH="$PP$SP"
export DISPLAY=:1        # GUI 表示用
$S/python.sh scripts/reinforcement_learning/rsl_rl/play.py \
  --task Isaac-Velocity-Flat-G1-v0 --use_pretrained_checkpoint --num_envs 16
```
- `source/*/`（editable install の実ソース）と venv の `site-packages`（`warp`,
  `rsl_rl` 等の依存）の**両方**を PYTHONPATH に入れる必要がある。
- `VIRTUAL_ENV` / `CONDA_PREFIX` を `unset` すると `isaaclab.sh` が
  `python3` フォールバックに落ちて失敗するので注意。

### 落とし穴
`isaaclab.sh` は Isaac Sim が見つからなくても**終了コード 0 を返す**。
成功したように見えるので、必ずログ本文でエラーを確認する。

## 実装済みの環境: SimEnvTest/

キーボードで G1 を操作する環境を `SimEnvTest/` に実装済み。詳細は
`SimEnvTest/README.md` を参照。起動は `bash SimEnvTest/run.sh`。

### ポリシーの制約（実測で確認済み・重要）
**この歩行ポリシーは前進のみで学習されている。**
IsaacLab の `flat_env_cfg.py` が `lin_vel_x = (0.0, 1.0)` と設定しているため、
後退は学習範囲外。実測では **-0.2 m/s までは歩けるが -0.3 m/s 以上で転倒する**。

後退以外は良好（胴体座標系での実測値）:
前進 +0.5→+0.38、横移動 ±0.4→+0.38/-0.42、
**旋回 ±0.3/±0.5/±1.0 → +0.30/+0.51/+0.98 とほぼ一致**。
静止指令でも約 0.04 m/s 漂う（位置保持の項が無いため仕様）。

### 追従性能の計測方法（はまった点）
**旋回の評価には胴体座標系の角速度 `root_ang_vel_b` を使う。**
ワールド座標の yaw を始点・終点で比較すると、回転が 2π を跨ぐため
正しく回っていても 0 に近い値になり「追従が弱い」と誤判定する。
実際に一度この方法で誤った結論を出した。

### 実装時にはまった点
- **Isaac Sim 6.0 の `Articulation.data` は warp 配列**。`data.joint_pos[0]` は
  `RuntimeError: Item indexing is not supported on wp.array objects` になる。
  `wp.to_torch()` で変換してから添字を取る（IsaacLab の `mdp/observations.py` と同じ）。
- **エントリポイントを他スクリプトから import してはいけない。** モジュール直下で
  `AppLauncher` を起動しているため、アプリが二重起動して無言で落ちる。
- **`finally` で `simulation_app.close()` を呼ばない。** 例外が隠れて
  「エラー無く終了した」ように見え、原因究明が困難になる。
- 起動時に `blas_thread_shutdown` / `__libc_fork` で segfault することがある。
  `OPENBLAS_NUM_THREADS=1` 等で回避（`run.sh` は設定済み）。
- 起動に 2〜5 分かかる（アセット取得を含む）。タイムアウトは長めに取る。

## 注意点
- アセットはリモートから取得されるため、初回ロード時はネットワーク接続が必要。

## コーディング規約
親ディレクトリの `~/isaac_dev/CLAUDE.md` の規約に従う（コメントは日本語、型ヒント必須、
ログは `print("[タグ] メッセージ")` 形式）。
