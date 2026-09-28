# 深度付きカメラサーバ（PC2 で動かす）

G1 の頭カメラ（RealSense D435i）を PC2 で直接読み、ZMQ で配信する。`run_g1_server.py`（lerobot 側のファイル）は変更しない。

| ポート（既定） | 中身 | 受け取る側 |
|---|---|---|
| 5556 | 深度付き: カラー（JPEG）＋ カラーに位置合わせした深度（16bit のまま）＋ カメラの内部パラメータ | `common/camera_rgbd.py` の `RgbdZmqSource` |
| 5555 | RGB 互換: `run_g1_server.py --camera` と同じ形式 | 既存の `ZmqFrameSource`（Perception）がそのまま動く |

- ポート番号、RealSense のシリアル番号、画像の大きさなどは `configs/depth_server.yaml` で変える
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

PC2 では、SETUP.md の 3.2 で作った conda の `lerobot` 環境（Python 3.12）を使う。

```bash
source ~/miniforge3/bin/activate lerobot
```
```bash
python -c "import pyrealsense2 as rs; print(rs.__version__)"
```

版の番号が出れば入っている。`ModuleNotFoundError` なら、次で入れる:

```bash
pip install pyrealsense2
```

- 2026-09-28 に、PyPI に aarch64（Jetson と同じ CPU）用の配布物があることを確認した
  （Python 3.12 用は 2.58.4、Python 3.8 用は 2.55.1）。ビルドは要らない。
- `ImportError: libusb-1.0.so.0` が出たら、USB を扱うライブラリが無い。次で確かめる:

  ```bash
  ldconfig -p | grep libusb-1.0
  ```

  何も出なければ `sudo apt install libusb-1.0-0` が必要（**実行する前にチームに確認する**）。

ほかに使うパッケージ（numpy、opencv、pyzmq、PyYAML）は lerobot と一緒に入っているはず。確かめる:

```bash
python -c "import numpy, cv2, zmq, yaml; print('ok')"
```

## 3. ファイルを PC2 に置く（ラボ PC で）

サーバに要るのは `Button_Press/J1-gen/` の `common/`・`configs/`・`real/` だけ。PC2 の `~/button_press/` に置く:

```bash
ssh unitree@192.168.123.164 mkdir -p button_press
```
```bash
scp -r Button_Press/J1-gen/common Button_Press/J1-gen/configs Button_Press/J1-gen/real unitree@192.168.123.164:button_press/
```

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
python real/depth_server/rgbd_server.py --list-devices
```
```bash
python real/depth_server/rgbd_server.py
```

`[rgbd_server] RealSense 開始: ...` と内部パラメータが表示され、5 秒ごとに配信したフレーム数と fps が出れば動いている。
Ctrl+C で止まる。SSH を切っても動かし続けたいときは:

```bash
nohup python -u real/depth_server/rgbd_server.py > ~/rgbd_server.log 2>&1 &
```

### よく使うオプション

| オプション | 意味 |
|---|---|
| `--serial <番号>` | RealSense をシリアル番号で選ぶ（`--list-devices` で表示される番号） |
| `--rgbd-port <番号>` / `--rgb-port <番号>` | ポートを変える（設定ファイルより優先） |
| `--no-legacy-rgb` | RGB 互換ストリーム（5555）を出さない |
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
| 権限のエラー（permission denied） | RealSense の USB に触る権限が無い。udev ルールの追加が必要な場合がある。**PC2 の設定を変える前にチームに確認する** |
| ラボ PC でタイムアウト | アドレス（`configs/camera.yaml`）とポートが合っているか。PC2 側で `ss -ltnp \| grep 5556` で待ち受けているか |
| 深度が 0 ばかり | 近すぎる（D435 はおよそ 0.2 m 未満を測れない）、または透明・黒い物。ボトルはラベル付きを使う |

## 確かめたこと（2026-09-28、実機なし）

- ダミーカメラで、サーバ → `RgbdZmqSource` と、サーバ → 既存の `ZmqFrameSource` の両方で受け取れた（約 30 fps）。
  赤い物が赤のまま届く（色の順番が入れ替わらない）ことも確かめた。
- 深度は送った値と完全に一致する（16bit のまま、zlib で圧縮しても値は変わらない）。
- 実際に近い深度（640x480）で、1 フレームは圧縮なし 606 KB、zlib で約 201 KB（30 fps で約 49 Mbps）。
- **本物の RealSense では未確認**（pyrealsense2 の呼び方は、偽物のモジュールを使ったテストで確かめただけ）。
