# ボタン押し（Button_Press/J1-gen）の環境構築

ボタン押しのコードを動かすための Python 環境の作り方。共通の `G1_HuggingFace/venv/`
（Python 3.12）を使う。基本はリポジトリ直下の [SETUP.md](../../SETUP.md) に従い、
**ボタン押しに要る部分だけ**を入れる。

コマンドはすべてリポジトリ直下（`G1_Hackason/`）で、1行ずつ実行する。

## どのマシンで何を入れるか

| マシン | 用途 | 入れるもの |
|---|---|---|
| 手元のノート PC（WSL2 Ubuntu 26.04, ARM64, GPU なし） | コード作成、単体テスト、MuJoCo の画面なし検証 | この手順（lerobot は入れない） |
| ラボ PC（Ubuntu, x86_64） | 実機日に G1 へ DDS で直接指令を送る | SETUP.md のフルセット（lerobot 込み）が入っている前提。足りなければこの手順の 3 と 4 を追加する |
| G1 の PC2（Jetson, aarch64） | 深度付きカメラサーバ（タスク3） | 別手順（タスク3の README に書く）。**OS は絶対に書き換えない** |

## 1. Python 3.12 の venv を作る

SETUP.md の 1.1（`python3 -m venv venv`）は、`python3` が 3.12 のマシン（Ubuntu 24.04 など）の手順。
`python3 --version` が 3.12 でないマシン（Ubuntu 26.04 は 3.14）では、**uv** で 3.12 を入れて venv を作る。
uv は、Python 本体と venv をまとめて扱えるツール（sudo 不要で、ホームフォルダの中だけに入る）。

```bash
python3 --version
```

3.12 と出たら SETUP.md の 1.1 のとおりに作る。それ以外なら次の4行:

```bash
curl -LsSf https://astral.sh/uv/install.sh | sh
```
```bash
~/.local/bin/uv python install 3.12
```
```bash
~/.local/bin/uv venv --python 3.12 --seed G1_HuggingFace/venv
```
```bash
G1_HuggingFace/venv/bin/python --version
```

`--seed` を付けると venv に `pip` が入り、以降は SETUP.md と同じく `G1_HuggingFace/venv/bin/pip` で入れられる。

## 2. ビルドに使う道具

CycloneDDS のビルドに `cmake`・`gcc`・`g++` が要る。

```bash
cmake --version
```

無ければ（sudo のパスワードを聞かれるので、Claude Code の `!` ではなく普通のターミナルで実行する）:

```bash
sudo apt install -y cmake gcc g++
```

## 3. CycloneDDS（SETUP.md 1.2 と同じ）

DDS（ロボットとの通信の仕組み）のライブラリ。unitree_sdk2py がこれを使う。

```bash
git clone --depth 1 -b releases/0.10.x https://github.com/eclipse-cyclonedds/cyclonedds G1_HuggingFace/cyclonedds
```
```bash
cmake -S G1_HuggingFace/cyclonedds -B G1_HuggingFace/cyclonedds/build -DCMAKE_INSTALL_PREFIX=$PWD/G1_HuggingFace/cyclonedds/install -DCMAKE_BUILD_TYPE=Release
```
```bash
cmake --build G1_HuggingFace/cyclonedds/build --target install -- -j$(nproc)
```

## 4. unitree_sdk2_python（SETUP.md 1.3 と同じ）

```bash
git clone https://github.com/unitreerobotics/unitree_sdk2_python.git G1_HuggingFace/unitree_sdk2_python
```
```bash
export CYCLONEDDS_HOME=$PWD/G1_HuggingFace/cyclonedds/install
```
```bash
G1_HuggingFace/venv/bin/pip install -e G1_HuggingFace/unitree_sdk2_python
```

確認（`sdk ok` と出れば成功）:

```bash
G1_HuggingFace/venv/bin/python -c "from unitree_sdk2py.idl.unitree_hg.msg.dds_ import LowCmd_; print('sdk ok')"
```

## 5. ボタン押しに要るパッケージ

Perception の部品（`YoloDetector`、`ZmqFrameSource`）も使うので、両方の requirements を入れる。

```bash
G1_HuggingFace/venv/bin/pip install -r Perception/requirements.txt -r Button_Press/J1-gen/requirements.txt
```

lerobot（SETUP.md 1.4）は**入れなくてよい**。ボタン押しの MuJoCo は、lerobot 経由の環境ではなく
公式モデルを直接読むため。

## 6. 公式モデルの取得と動作確認

G1 の公式モデル（URDF と MJCF）を `_local/button_press/models/` に取り出す（約19MB）。

```bash
bash Button_Press/J1-gen/sim/fetch_models.sh
```

テストを実行する（最後に `OK` と出れば成功）:

```bash
PYTHON=G1_HuggingFace/venv/bin/python bash Button_Press/J1-gen/tests/run_tests.sh
```

## 動作確認済みの組み合わせ

| 日付 | マシン | Python | 主なパッケージ |
|---|---|---|---|
| 2026-09-28 | WSL2 Ubuntu 26.04.1, aarch64, GPU なし | 3.12.14（uv） | CycloneDDS 0.10.5（C）/ cyclonedds 0.10.2（Python）、unitree_sdk2py 1.0.1、numpy 2.5.3、opencv-python 5.0.0、mujoco 3.14.0、pyzmq 27.2.0、torch 2.14.0、ultralytics 8.4.164 |

GPU の無い WSL2 でも、MuJoCo のカメラ描画（`mujoco.Renderer`）は `MUJOCO_GL` を指定せずに動いた。
