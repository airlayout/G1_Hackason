# G1_HuggingFace

[HuggingFace LeRobot の Unitree G1 ドキュメント](https://huggingface.co/docs/lerobot/unitree_g1) に基づいて、
Unitree G1 を Unitree SDK (`unitree_sdk2py`) 経由で操作するための環境。
`G1_Hackason/` の他ディレクトリ（`IsaacSim_Env/`, `SimEnv3D/`）は Isaac Sim ベースのデジタルツイン
アプローチだが、こちらは LeRobot 公式サポートの DDS ベースのアプローチ。

## セットアップ済み環境（2026-08-20 構築・動作確認済み）

- **Miniconda**: `/home/ubuntu/miniconda3`（このマシンには元々 conda が無かったため新規インストール）
- **conda 環境 `lerobot`**（Python 3.12）
- **CycloneDDS C ライブラリ**: `G1_HuggingFace/cyclonedds/install`
  - pip 版 `cyclonedds==0.10.2` は C ライブラリを自前でビルドせず、`CYCLONEDDS_HOME` /
    `CMAKE_PREFIX_PATH` で既存の C ライブラリを探しにいく。そのため `releases/0.10.x`
    ブランチを clone → cmake で `cyclonedds/install` にインストール済み。
- **`unitree_sdk2_python`**: `G1_HuggingFace/unitree_sdk2_python`（editable install 済み）
- **`lerobot`**: `G1_HuggingFace/lerobot`（`pip install -e '.[unitree_g1]'` 済み）
- **pinocchio 3.9.0**（conda-forge 版、CasADi バインディング付き。アーム IK 用）
- **ffmpeg 8.0.1**（conda-forge 版）
- **torch 2.11.0+cu128**（GPU: NVIDIA L40S、ドライバ 570.211.01 / CUDA 12.8 に合わせて
  cu128 版を明示的に指定してインストール。pip 既定の cu130 版はこのドライバでは
  `CUDA available: False` になるため注意）
- シミュレーション動作確認用: `mujoco`, `loguru`, `msgpack`, `msgpack-numpy`

### 使い方

```bash
source ~/miniconda3/etc/profile.d/conda.sh
conda activate lerobot
cd ~/NVIDIA/G1_Hackason/G1_HuggingFace/lerobot
```

## 動作確認済み: シミュレーションでの teleoperate

以下のコマンドで、HuggingFace Hub 上の `lerobot/unitree-g1-mujoco`（MuJoCo シーン）と
`nepyope/GR00T-WholeBodyControl_g1`（ONNX 歩行ポリシー、GrootLocomotionController が
自動ダウンロード）を使って 90 秒間動作確認済み。カメラ・表示なしでコントローラループが
約 59〜60Hz で安定動作することを確認した（`--display_data=true` にすれば rerun で
可視化も可能。このマシンには `DISPLAY=:1` が存在する）。

```bash
lerobot-teleoperate \
  --robot.type=unitree_g1 \
  --robot.is_simulation=true \
  --teleop.type=unitree_g1 \
  --teleop.id=wbc_unitree \
  --robot.cameras='{"global_view": {"type": "zmq", "server_address": "localhost", "port": 5555, "camera_name": "head_camera", "width": 640, "height": 480, "fps": 30, "warmup_s": 5}}' \
  --display_data=true \
  --robot.controller=GrootLocomotionController
```

ガンパッドを接続すればロコモーションを操作可能（`9`=リリース、`7`/`8`=腰の高さ）。
`--robot.controller=HolosomaLocomotionController` にも切り替え可能（同様に ONNX を
HF Hub から自動取得する実装のため、追加のリポジトリ clone は不要）。

## 動作確認済み: ゲームパッド無しでスクリプトから指令を送る

`lerobot-teleoperate` はゲームパッド/exo が無いと `remote.*` の指令が常にゼロになる
（`get_action()` がその入力元しか持たないため）。スクリプトから直接コマンドを送って
G1（MuJoCo）が反応することを検証したい場合は、`UnitreeG1` を直接使い
`robot.send_action({"remote.ly": 0.4, ...})` のように呼び出せばよい。
`send_action()` は `controller_input` を更新するだけで、実際の反映はバックグラウンドの
`_controller_thread`（`GrootLocomotionController` など、50Hz）が行う。

`scripts/verify_g1_sim_command.py` で検証済み（2026-08-20）: `remote.ly` / `remote.lx` /
`remote.rx` を送ると、`GrootLocomotionController.cmd`（`[vx, vy, theta_dot]`）に
期待通りの符号・値（`ly→cmd[0]`, `lx→-cmd[1]`, `rx→-cmd[2]`）で反映され、ゼロに戻すと
`cmd` も正しく `0` に戻ることを確認した。

```bash
python G1_HuggingFace/scripts/verify_g1_sim_command.py
```

なお、脚関節の角速度（`dq`）や IMU の `gyro.z` を直接比較して「歩行で有意に変化するか」
を見る素朴な検証は最初に行ったが、**この標準立位バランス方策自体の揺れがコマンドによる
変化より大きく、有意差は出なかった（inconclusive）**。理由は下記の「elastic band」を
参照（そもそも接地しておらず物理的な前進が起きようがなかった）。

## 既知のバグ: elastic band（吊りバンド）とその対処（2026-08-20 調査・パッチ済み）

シミュレーション上の G1 は素の状態だと**地面から浮いた状態**になる。原因は
`lerobot/unitree-g1-mujoco`（HF Hub 上の trust_remote_code 環境）が同梱する
[unitree_mujoco](https://github.com/unitreerobotics/unitree_mujoco) 由来の
**elastic band**（骨盤を `z=1.0m` 付近へ強い PD 力 `kp_pos=10000` で引き上げ続ける仮想バンド）
が `config.yaml` の `ENABLE_ELASTIC_BAND: True` によりデフォルト有効になっているため
（本家ドキュメントにあった「9でリリース、7/8で腰の高さ調整」の正体）。
**現在の LeRobot 側 `UnitreeG1` 実装には、このバンドを解除する経路が実装されていない**
（キーボードコールバック経由のみ）ため、`lerobot-teleoperate` を素で使う限り常に浮いた
ままになる。

さらに、`robot.sim_env.simulator.sim_env.elastic_band.enable = False` で直接解除しようと
すると、以下のバグを踏むことがある（非決定的）:

```
Exception in thread Thread-3 (_subscribe_lowstate):
  File ".../sim/unitree_sdk2py_bridge.py", line 439, in Advance
    rot = scipy.spatial.transform.Rotation.from_quat(quat)
ValueError: Found zero norm quaternions in `quat`.
```

これは物理演算（`sim_env.step()`）を回している非デーモンスレッドの中で起きる例外のため、
**プロセスは落ちずスレッドだけが黙って死に、以後シミュレーションが完全に停止する**。
teleop ループやコントローラループは何事もなく動き続けるため非常に気づきにくい。

`scripts/patch_mujoco_elastic_band.py` で、HF Hub キャッシュ内の
`unitree_sdk2py_bridge.py`（`ElasticBand.Advance()`）にゼロノルムガードを当てて修正済み
（何度実行しても安全な idempotent スクリプト。まだ一度もシミュレーションを起動していない
環境ではキャッシュが無いため先に一度 `lerobot-teleoperate` 等でダウンロードさせてから
実行すること）:

```bash
python G1_HuggingFace/scripts/patch_mujoco_elastic_band.py
```

⚠️ このパッチは `~/.cache/huggingface/hub/models--lerobot--unitree-g1-mujoco/blobs/...`
に直接当てているため、キャッシュを消す／別マシンで動かす際は再実行が必要。また
パッチ後もこのクラッシュ自体が非決定的なので、まれに接続直後に無言で止まることがある
（その場合は再実行すれば大抵通る）。

### 実際に前進歩行することを確認（`scripts/release_band_and_walk_forward.py`）

パッチ適用後、`elastic_band.enable = False` でバンドを解除し、`remote.ly=0.5`（前進）を
5 秒間送って MuJoCo 側の pelvis 座標（`mj_data.qpos[0:3]`）を直接読んで検証した:

| 段階 | pelvis (x, y, z) |
|---|---|
| バンド有効時（浮いている） | `(-0.001, 0.000, 0.910)` |
| バンド解除 + 2 秒静止後 | `(0.040, 0.010, 0.751)` |
| 前進指令を 5 秒送信後 | `(1.892, 0.222, 0.750)` |

5 秒間で水平方向に **1.86m 前進**（約 0.37 m/s）、z 座標は `0.751→0.750` とほぼ一定
（転倒・沈み込みではない）。スクリプトから送ったコマンドで実際に二足歩行が起きることを
座標レベルで確認済み。

```bash
python G1_HuggingFace/scripts/patch_mujoco_elastic_band.py  # 未適用なら先に
python G1_HuggingFace/scripts/release_band_and_walk_forward.py
```

## 実機への接続（未検証・このマシンからは実施不可）

**このマシンは AWS 上のクラウド VM で、G1 実機と直接 Ethernet 接続できる NIC が無い**
（`enp39s0` は VPC 内部 NW `10.0.1.0/24` のみ）。実機に接続する場合は、実機と
Ethernet ケーブルで直結できる別のマシン（ノート PC 等）側でこの環境構築手順
（あるいはこのディレクトリを rsync 等でコピー）をやり直す必要がある。

実機側の手順は [公式ドキュメント](https://huggingface.co/docs/lerobot/unitree_g1) の
"Connect to the Physical Robot" 以降を参照。要点のみ記載:

- G1 の固定 IP: `192.168.123.164`。操作側 PC は同一サブネット `192.168.123.x`（`x≠164`）に
  static IP を設定する。
- `ssh unitree@192.168.123.164`（パスワード `123`）
- G1 側にもインターネット共有 + `unitree_sdk2_python` + `lerobot[unitree_g1]` の
  インストールが必要（手順は本ドキュメント最上部と同様）。
- G1 側で `python src/lerobot/robots/unitree_g1/run_g1_server.py --camera` を起動し、
  操作側 PC から `--robot.robot_ip=<ROBOT_IP> --robot.is_simulation=false` を付けて
  `lerobot-teleoperate` を実行する。

## 既知の注意点

- pip 既定の `torch` は環境の GPU ドライバ（CUDA 12.8）より新しい CUDA 13.0 向けにビルドされて
  おり `torch.cuda.is_available()` が `False` になる。`--index-url
  https://download.pytorch.org/whl/cu128` で明示的に入れ直す必要がある
  （`pip install "torch>=2.7,<2.12" "torchvision>=0.22,<0.27" --index-url
  https://download.pytorch.org/whl/cu128`）。
- `cyclonedds` の pip パッケージは C ライブラリを同梱しないため、`CYCLONEDDS_HOME` を
  設定してから `unitree_sdk2py` をインストールする必要がある（上記参照）。
- `G1_HuggingFace/unitree_sdk2_python`, `lerobot`, `cyclonedds` はそれぞれ独自の `.git` を
  持つ外部クローンのため、`G1_Hackason` リポジトリの `.gitignore` で除外済み。
