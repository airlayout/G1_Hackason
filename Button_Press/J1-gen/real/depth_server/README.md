# 深度付きカメラサーバ（PC2 で動かす）

G1 の頭カメラ（RealSense D435i）を PC2 で直接読み、ZMQ で配信する。`run_g1_server.py`（lerobot 側のファイル）は変更しない。

| ポート（既定） | 中身 | 受け取る側 |
|---|---|---|
| 5556 | 深度付き: カラー（JPEG）＋ カラーに位置合わせした深度（16bit のまま）＋ カメラの内部パラメータ | `common/camera_rgbd.py` の `RgbdZmqSource` |
| 5555 | RGB 互換: `run_g1_server.py --camera` と同じ形式 | 既存の `ZmqFrameSource`（Perception）がそのまま動く |

- 配信は既定で **10 fps** に間引く（カメラは 30 fps で読み続ける。PC2 の CPU とネットワークの負担を減らすため）。
- ポート番号、RealSense のシリアル番号、画像の大きさ、配信の上限 fps は `configs/depth_server.yaml` で変える
  （外付けカメラの配信と同時に動かすときに、ポートがぶつからないようにする）。
  受け取る側の接続先は `configs/camera.yaml`。
- 形式の詳細は `common/rgbd_protocol.py` の先頭のコメント。
- 用語: **位置合わせ（align）** = 深度カメラとカラーカメラは少し離れて付いているので、深度の画素を
  カラー画像の画素に対応するように並べ直すこと。これで「カラー画像のこの画素の距離」がそのまま読める。

コマンドはすべて 1 行ずつ実行する。「PC2 で」とあるものは `ssh g1` などで PC2 にログインして実行する。

> ⚠️ **PC2 の OS は絶対に書き換えない。** 下の手順で `sudo apt` が必要になった場合は、実行する前にチームに確認する。

---

## 1. RealSense が PC2 につながっているか確かめる（PC2 で）

機体のロットによっては、頭カメラが PC1（ユーザーはアクセスできない）につながっている。

```bash
lsusb | grep -i intel
```

`Intel Corp. Intel(R) RealSense(TM) Depth Camera 435i` のような行が出れば PC2 につながっている。
**何も出なければ、このサーバは使えない。** その場合は、タスク6の「対象の位置を設定ファイルの値で与えるモード」を使う。

## 2. pyrealsense2 が入っているか確かめる（PC2 で）

まず Python の版と、pyrealsense2 がすでに入っているかを確かめる。PC2 に conda の `lerobot` 環境
（SETUP.md 3.2、Python 3.12）があればそれを使う。無ければ PC2 の `python3` を使う。

```bash
source ~/miniforge3/bin/activate lerobot
```
```bash
python3 --version
```
```bash
python3 -c "import pyrealsense2 as rs; print(rs.__version__)"
```

版の番号が出れば入っているので、3 へ進む。`ModuleNotFoundError` なら、次の 2-1 か 2-2 で入れる。

ほかに使うパッケージ（numpy、opencv、pyzmq、PyYAML）が入っているかも確かめる:

```bash
python3 -c "import numpy, cv2, zmq, yaml; print('ok')"
```

### 2-1. インターネットにつながっている場合

```bash
python3 -m pip install pyrealsense2
```

### 2-2. インターネットにつながっていない場合（オフラインで入れる）

PC2 はインターネットにつながっていない可能性が高い。そこで、**事前にネットにつながったマシンで**必要なファイルを
ダウンロードしておき、当日 PC2 にコピーして入れる。sudo は使わず、システムも変えない。

用意するもの（`fetch_offline_packages.sh` がまとめてダウンロードする。合計約 17 MB）:

| ファイル | 用途 |
|---|---|
| `pyrealsense2-*-cp38-*-manylinux2014_aarch64.whl` | Python 3.8 用（2.55.1） |
| `pyrealsense2-*-cp310-*-manylinux2014_aarch64.whl` | Python 3.10 用（2.58.4） |
| `pyrealsense2-*-cp312-*-manylinux2014_aarch64.whl` | Python 3.12 用（2.58.4。SETUP.md の conda 環境の版） |
| `libusb-1.0-0_1.0.23-2build1_arm64.deb` | pyrealsense2 が使う USB のライブラリ（Ubuntu 20.04 用）。PC2 に無い場合だけ使う |
| `install_offline.sh`、`SHA256SUMS` | PC2 で入れるスクリプトと、ファイルが壊れていないかを確かめる値 |

**事前（ネットにつながったマシン。ラボ PC でも手元の PC でもよい）:**

```bash
bash Button_Press/J1-gen/real/depth_server/fetch_offline_packages.sh
```

`_local/button_press/offline/` にファイルができる。手元の PC で用意した場合は、このフォルダをラボ PC にも持っていく。

**当日（ラボ PC → PC2 へコピー）:**

```bash
ssh unitree@192.168.123.164 mkdir -p button_press
```
```bash
scp -r _local/button_press/offline unitree@192.168.123.164:button_press/
```

**当日（PC2 で入れる）:** 使う Python の環境に入ってから（conda なら `source ~/miniforge3/bin/activate lerobot`）:

```bash
bash ~/button_press/offline/install_offline.sh
```

- Python の版に合う wheel を選んで `pip install --no-index` で入れる。conda を使わず PC2 の別の Python を
  使うときは、`PYTHON=/usr/bin/python3 bash ~/button_press/offline/install_offline.sh` のように指定する。
- libusb-1.0 がシステムに無ければ、.deb を**インストールせず、中身だけ**を `~/button_press/libusb_local/` に
  取り出す（`dpkg -x`）。サーバは 4 の `start_rgbd_server.sh` で起動すれば、自動でここを使う。
- 最後に `[install] pyrealsense2 ... OK、RealSense 1 台` と出れば成功。

2026-09-28 に、PC2 と同じ条件（aarch64、Python 3.12、libusb がシステムに無い）の手元のマシンで、
`install_offline.sh` → `start_rgbd_server.sh` の流れを確かめた（RealSense は無いので `--source dummy` で起動）。

## 4. 起動する（PC2 で）

**RealSense は 1 つのプログラムしか開けない。** `run_g1_server.py` がカメラを開いていると、このサーバは起動できない。
すでに `run_g1_server.py --camera` が動いていれば止めて、**`--camera` なしで**起動し直す
（lowcmd / lowstate の中継はそのまま使える）。動いているかは次で確かめる:

```bash
ps aux | grep run_g1_server
```

`run_g1_server.py` を `--camera` なしで起動する（SETUP.md 3.6 と同じ。1 つ目のターミナル）:

```bash
source ~/miniforge3/bin/activate lerobot
```
```bash
cd ~/lerobot
```
```bash
export CYCLONEDDS_HOME=~/cyclonedds/install
```
```bash
export LD_LIBRARY_PATH=~/cyclonedds/install/lib:$LD_LIBRARY_PATH
```
```bash
python -u src/lerobot/robots/unitree_g1/run_g1_server.py
```

深度付きカメラサーバを起動する（2 つ目のターミナル）:

```bash
source ~/miniforge3/bin/activate lerobot
```
```bash
cd ~/button_press
```
```bash
bash real/depth_server/start_rgbd_server.sh --list-devices
```
```bash
bash real/depth_server/start_rgbd_server.sh
```

（`start_rgbd_server.sh` は、2-2 で libusb を取り出していればその場所を設定してから `rgbd_server.py` を起動する。
引数はそのまま `rgbd_server.py` に渡る。conda を使わないときは `PYTHON=/usr/bin/python3` を前に付ける）

`[rgbd_server] RealSense 開始: ...` と内部パラメータが表示され、5 秒ごとに配信したフレーム数と fps が出れば動いている。
Ctrl+C で止まる。SSH を切っても動かし続けたいときは:

```bash
nohup bash real/depth_server/start_rgbd_server.sh > ~/rgbd_server.log 2>&1 &
```

### よく使うオプション

| オプション | 意味 |
|---|---|
| `--serial <番号>` | RealSense をシリアル番号で選ぶ（`--list-devices` で表示される番号） |
| `--rgbd-port <番号>` / `--rgb-port <番号>` | ポートを変える（設定ファイルより優先） |
| `--no-legacy-rgb` | RGB 互換ストリーム（5555）を出さない |
| `--max-fps <数>` | 配信の上限 [fps]（既定 10。`configs/depth_server.yaml` の `publish.max_fps`）。0 なら間引かない |
| `--source dummy` | RealSense を使わず、作り物の画像と深度を出す（配信経路だけを確かめる） |

## 5. 届いているか確かめる（ラボ PC で）

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/probe_rgbd.py
```

fps、内部パラメータ、画像中央の深度、深度が測れなかった画素の割合が表示される。`--save` を付けると、
カラー（PNG）・深度（16bit PNG）・内部パラメータ（JSON）を `_local/button_press/probe/` に保存する。

RGB 互換ストリームは、既存の Perception の確認スクリプトで確かめられる:

```bash
G1_HuggingFace/venv/bin/python Perception/real/run_real.py --server-address 192.168.123.164
```

## うまくいかないとき

| 症状 | 原因と対処 |
|---|---|
| `--list-devices` で見つからない | 1 の `lsusb` で見えるか。見えるのに見つからないなら、ほかのプログラム（`run_g1_server.py --camera` など）が開いていないか |
| 起動時に `Device or resource busy` など | ほかのプログラムが RealSense を開いている。`run_g1_server.py` を `--camera` なしで起動し直す |
| `ImportError: libusb-1.0.so.0` | libusb が見つからない。2-2 の `install_offline.sh` で取り出し、`start_rgbd_server.sh` で起動する |
| 権限のエラー（permission denied） | RealSense の USB に触る権限が無い。udev ルールの追加が必要な場合がある。**PC2 の設定を変える前にチームに確認する** |
| ラボ PC でタイムアウト | アドレス（`configs/camera.yaml`）とポートが合っているか。PC2 側で `ss -ltnp \| grep 5556` で待ち受けているか |
| 深度が 0 ばかり | 近すぎる（D435 はおよそ 0.2 m 未満を測れない）、または透明・黒い物。ボトルはラベル付きを使う |

## 確かめたこと（2026-09-28、実機なし）

- ダミーカメラで、サーバ → `RgbdZmqSource` と、サーバ → 既存の `ZmqFrameSource` の両方で受け取れた。
  配信の上限は 10 / 5 / 0（なし）で、受信側で 10.0 / 5.0 / 30.0 fps。
  赤い物が赤のまま届く（色の順番が入れ替わらない）ことも確かめた。
- 深度は送った値と完全に一致する（16bit のまま、zlib で圧縮しても値は変わらない）。
- 実際に近い深度（640x480）で、1 フレームは圧縮なし 606 KB、zlib で約 201 KB（30 fps で約 49 Mbps）。
- 本物の pyrealsense2（2.58.4）を読み込み、サーバで使っている関数や項目（`enable_device`、`get_depth_scale`、
  内部パラメータの `ppx` / `ppy` など）がすべてあることを確かめた。
- **本物の RealSense をつないでの確認はまだ**（実機日に行う）。
