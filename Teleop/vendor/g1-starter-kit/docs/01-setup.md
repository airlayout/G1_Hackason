# 導入

初回だけの作業です。回線次第で 30〜60 分かかります。

---

## 前提

| | |
|---|---|
| OS | Ubuntu **22.04**（x86-64） |
| ディスク空き | 12GB 以上 |
| RAM | 8GB 以上（16GB 推奨） |
| ネット | 導入中のみ必要（運用時はオフラインで動きます） |

> **Ubuntu 24.04 では動きません。** `xr_teleoperate` が 20.04 / 22.04 でのみ検証されているためです。
> OS からアップグレードを促されても拒否してください。

---

## 手順

```bash
git clone https://github.com/yokoshaan/g1-starter-kit.git
cd g1-starter-kit

bash setup/install_apt.sh      # ① 要 sudo
bash setup/install_env.sh      # ② root 不要
bash setup/install_livox.sh    # ③ root 不要（LiDAR を使う場合）
```

各スクリプトは**冪等**です。途中で失敗しても、原因を直して同じコマンドをもう一度実行すれば、済んでいる部分は飛ばします。

### ① `install_apt.sh` — apt で入れるもの

ビルドツール、ROS 2 Humble desktop、PlotJuggler、PCL を入れます。

**⚠️ ネイティブ端末で実行してください。** VSCode の統合ターミナルでは `sudo` が通らない環境があります（`no_new_privs`）。

```bash
grep NoNewPrivs /proc/self/status     # 0 なら OK、1 なら別の端末で
```

**⚠️ 長いコマンドを手で貼らないでください。** 端末の設定によって先頭に `^[[200~` が混ざり、`sudo: command not found` で失敗します。だからスクリプトにしてあります。実行するときも上記の 1 行を手で打ってください。

### ② `install_env.sh` — テレオペ環境

作られるもの:

| | |
|---|---|
| conda 環境 `tv` | python 3.10 / pinocchio 3.1.0 / numpy 1.26.4 |
| `~/xr_teleoperate` | テレオペ本体（Unitree 公式）+ サブモジュール 3 つ |
| `~/cyclonedds` | DDS の C 実装（0.10.2 をソースからビルド） |
| `~/unitree_sdk2_python` | ロボットとの通信 |
| Quest 用の自己署名証明書 | `~/xr_teleoperate/teleop/televuer/{cert,key}.pem` |

最後に `setup/apply_patches.py` が自動で走り、テレオペ本体に必要な修正を当てます（後述）。

### ③ `install_livox.sh` — LiDAR ドライバ

| | |
|---|---|
| `~/.local/lib/liblivox_lidar_sdk_*` | Livox-SDK2 |
| `~/ws_livox/` | `livox_ros_driver2` の ROS 2 ワークスペース |

**sudo を使いません。** Livox-SDK2 は既定で `/usr/local` に入りますが、`~/.local` に入れて `CMAKE_LIBRARY_PATH` で参照させています。実行時は `LD_LIBRARY_PATH` が必要で、`scripts/lib.sh` が自動設定します。

---

## バージョンを固定している理由

**これらは変更しないでください。** `xr_teleoperate` はこの組み合わせでのみ検証されています。

| | 値 | 理由 |
|---|---|---|
| Ubuntu | 22.04 | 20.04 / 22.04 でのみ検証済み |
| Python | 3.10 | 依存パッケージの前提 |
| pinocchio | 3.1.0 | 上げると逆運動学が動かない |
| numpy | 1.26.4 | pinocchio との組み合わせ |
| cyclonedds | 0.10.2 | `unitree_sdk2_python` の要求バージョン。C コアと Python バインディングを厳密に一致させる |
| params_proto | 2.13.2 | `vuer` が 2.x の API を使う。3.x が入ると `ImportError: cannot import name 'Vuer'` になる |

`teleimager` を `pip install --no-deps` で入れているのも同じ理由です（依存解決を任せると環境が壊れます）。

---

## テレオペ本体に当てる修正

`xr_teleoperate` は Unitree の公式リポジトリなので fork せず、手元のクローンに修正を当てる方式にしています。patch ファイルではなく文字列置換なので、上流が更新されて行番号がずれても壊れません。

```bash
python3 setup/apply_patches.py --list        # 何を当てるか
python3 setup/apply_patches.py --dry-run     # 当てずに確認
python3 setup/apply_patches.py               # 当てる
python3 setup/apply_patches.py --revert      # 元に戻す
```

| 名前 | 内容 | 当てないとどうなるか |
|---|---|---|
| `safe-startup` | 起動時の腕の速度を 2 rad/s に落とす（上流既定は 30） | **起動した瞬間に両腕が高速で振られる**（危険） |
| `speed-restore` | 追従開始（`r`）で 30 rad/s に戻す | テレオペ中もずっと 2 rad/s に制限され、操作にならない |
| `head-reference` | 腕の基準を `head_yaw` → `head_position` | 操作中によそを向くと左右がズレる |
| `record-camera` | カメラ画像の中身も確認する | **記録開始（`s`）で teleop ごと落ちる**（頭部カメラが無い環境） |
| `motion-switcher-retry` | モード判定 RPC のタイムアウトを伸ばしてリトライ | `Enter debug mode: Failed` が誤表示され、切り分けを妨げる |

> `safe-startup` と `speed-restore` は**必ず対で**当ててください。上流から
> `speed_gradual_max()` が削除され、初期化後に速度を戻す処理が無くなったためです。
> `--only` で片方だけ指定した場合は、もう片方が自動で補完されます。

初回適用時に `*.orig` のバックアップを作ります。冪等なので何度実行しても安全です。

> `git submodule update` などで `xr_teleoperate` を巻き戻すと修正は消えます。その場合は再度実行してください。

---

## 導入できたかの確認

```bash
# conda 環境と python 側
source ~/miniforge3/etc/profile.d/conda.sh && conda activate tv
python -c "import pinocchio, numpy, unitree_sdk2py, vuer; print('OK')"

# ROS と LiDAR ドライバ
source scripts/lib.sh && load_config && use_ros
ros2 pkg executables livox_ros_driver2

# パッチ
python3 setup/apply_patches.py --dry-run

# 実機なしで点群表示を試す
./scripts/lidar_view.sh --fake
```

---

## 次にやること

1. `cp config/g1.env.example config/g1.env` してネットワークを設定 → [02-network.md](02-network.md)
2. `./scripts/preflight.sh` で疎通と機体構成を確認
3. `./scripts/teleop.sh` でテレオペ → [03-teleop.md](03-teleop.md)

つまずいたら [06-troubleshooting.md](06-troubleshooting.md) を見てください。
