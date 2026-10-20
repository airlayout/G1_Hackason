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
PC2 の `lsusb` は名前の表が古く、D435i（`8086:0b3a`）を `Intel Corp. 4-Port USB 3.0 Hub` と表示する（2026-09-29）。
`8086:0b3a` の行があればつながっている。
**何も出なければ、このサーバは使えない。** その場合は、タスク6の「対象の位置を設定ファイルの値で与えるモード」を使う。

## 2. pyrealsense2 を入れる（PC2 で。システムの Python 3.8 に入れる）

**配信サーバは、PC2 のシステムの Python（`/usr/bin/python3`、3.8）で動かす。conda の `lerobot` 環境（3.12）では動かない。**
2026-09-29 に PC2 で確かめた: Python 3.10 / 3.12 用の pyrealsense2（aarch64）は、名前に `manylinux2014` と付いていても
中身が glibc 2.34 / 2.38 を必要とし、PC2（Ubuntu 20.04、glibc 2.31）では `GLIBC_2.38 not found` で読み込めなかった。
Python 3.8 用の 2.55.1 は glibc 2.17 で足りる。

まず conda の環境から抜けて、システムの Python で pyrealsense2 が入っているかを確かめる:

```bash
conda deactivate
```
```bash
/usr/bin/python3 -c "import pyrealsense2; print('pyrealsense2 あり')"
```

`ModuleNotFoundError` なら、次のオフラインの手順で入れる（PC2 はインターネットにつながっていない可能性が高いため）。
sudo は使わず、システムも変えない（`pip --user` で `~/.local` の下に入れる）。

用意するもの（`fetch_offline_packages.sh` がまとめてダウンロードする。合計約 47 MB）:

| ファイル | 用途 |
|---|---|
| `pyrealsense2-2.55.1.6486-cp38-cp38-manylinux2014_aarch64.whl` | pyrealsense2（Python 3.8 用。必要な glibc 2.17） |
| `deps_cp38/`（numpy 1.24.4、opencv-python-headless 4.8.1、pyzmq 27.1.0、PyYAML 6.0.3） | 配信サーバが使うもの。システムの Python に無いときだけ入れる |
| `libusb-1.0-0_1.0.23-2build1_arm64.deb` | USB のライブラリ（Ubuntu 20.04 用）。PC2 のシステムに無いときだけ、中身を取り出して使う |
| `install_offline.sh`、`SHA256SUMS` | PC2 で入れるスクリプトと、ファイルが壊れていないかを確かめる値 |

**事前（ネットにつながったマシン。ラボ PC でも手元の PC でもよい）:**

```bash
bash Button_Press/J1-gen/real/depth_server/fetch_offline_packages.sh
```

`_local/button_press/offline/` にファイルができる（`_local/` は git に入らないので、ほかの PC で使うときはコピーして持っていく）。

**PC2 へコピー**（有線のラボ PC なら 192.168.123.164、無線なら PC2 の無線の IP）。コマンドが長いと貼り付けで
2 行に分かれて失敗するので、1 つずつ送る:

```bash
ssh unitree@192.168.123.164 mkdir -p button_press
```
```bash
scp -r _local/button_press/offline unitree@192.168.123.164:button_press/
```

**PC2 で入れる**（conda の環境から抜けた状態で）:

```bash
bash ~/button_press/offline/install_offline.sh
```

- ファイルを確かめ、**wheel が必要とする glibc が PC2 にあるかを比べてから**入れる（足りなければ入れずに止まる）。
- システムの Python なので `--user` で入れる。numpy などは、読み込めないものだけを `deps_cp38/` から入れる。
- libusb はシステムにあればそれを使う（PC2 にはある）。無ければ `.deb` の中身を `~/button_press/libusb_local/` に取り出すだけ。
- 最後に `[install] pyrealsense2 2.55.1.6486 OK、RealSense 1 台` と `numpy ... OK` が出れば成功。
- 以前に conda の `lerobot` 環境へ入れてしまった pyrealsense2（動かない）は、`lerobot` 環境で `pip uninstall -y pyrealsense2` で消せる。

2026-09-29 に、手元のマシンで Python 3.8 の空の環境から `install_offline.sh` → `start_rgbd_server.sh`（`--source dummy`）
の流れを確かめた。glibc を 2.31 と見せかけて、Python 3.12 用の 2.58.4 が入らずに止まることも確かめた。

## 3. サーバのファイルを PC2 に置く（ラボ PC / ノート PC で）

サーバに要るのは `Button_Press/J1-gen/` の `common/`・`configs/`・`real/` だけ。PC2 の `~/button_press/` に 1 つずつ送る:

```bash
scp -r Button_Press/J1-gen/common unitree@192.168.123.164:button_press/
```
```bash
scp -r Button_Press/J1-gen/configs unitree@192.168.123.164:button_press/
```
```bash
scp -r Button_Press/J1-gen/real unitree@192.168.123.164:button_press/
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

深度付きカメラサーバを起動する（2 つ目のターミナル。conda の環境には入らない。入っていたら `conda deactivate`）:

```bash
cd ~/button_press
```
```bash
bash real/depth_server/start_rgbd_server.sh --list-devices
```
```bash
bash real/depth_server/start_rgbd_server.sh
```

（`start_rgbd_server.sh` は、システムの Python（`/usr/bin/python3`）で `rgbd_server.py` を起動する。2 で libusb を
取り出していれば、その場所も設定する。引数はそのまま `rgbd_server.py` に渡る）

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
| `--source videohub` | カラーは Unitree の videohub から受け取り、深度だけを RealSense から開く（videohub_pc4 がカラーを開いているとき用。下の「videohub から受け取る」） |

### videohub から受け取る（`--source videohub`）

Unitree の `videohub_pc4` が RealSense のカラー（`/dev/video4`）を開いていると、ふつうの起動は
`Device or resource busy` で止まる。そのときは videohub を止めずに、次で起動する（PC2 で）:

```bash
bash real/depth_server/start_rgbd_server.sh --source videohub
```

- カラー: videohub に 1 枚ずつ頼んで受け取る（`unitree_sdk2py` の Go2 用 `VideoClient`、DDS は `eth0`）。
  1920x1080 を縦横比を保って **640x360** に縮めて送る（ふつうの起動は 640x480）
- 深度: RealSense から深度だけを開く（640x480、30 fps）。起動直後の 15 枚は捨てる
- 位置合わせ: pyrealsense2 の `align` はカラーを開いていないと使えないので、`common/depth_align.py` で計算する。
  カラーの内部パラメータと深度 → カラーの位置関係は、カラーを開かずに RealSense の設定から読む
- カラーと深度は別の経路なので、撮った瞬間は揃わない。5 秒ごとの表示に、時刻の差と位置合わせの時間が出る
- 設定は `configs/depth_server.yaml` の `camera.videohub`（ネットワークの口、送る幅、videohub の画像の大きさ）
- `unitree_sdk2py` が要る（PC2 のシステムの Python には `~/unitree_sdk2_python` が入っている）
- 受け取る側（`probe_rgbd.py` など）は変えなくてよい（送る形式は同じで、大きさと内部パラメータが中に書いてある）

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
| 起動時に `Device or resource busy` など | ほかのプログラムが RealSense を開いている。`run_g1_server.py` なら `--camera` なしで起動し直す。Unitree の `videohub_pc4` が `/dev/video4`（カラー）を開いているときは止めない（`REAL_DAY_PROCEDURE.md` の「Device or resource busy」の節） |
| `ImportError: libusb-1.0.so.0` | libusb が見つからない。2 の `install_offline.sh` で取り出し、`start_rgbd_server.sh` で起動する |
| `GLIBC_2.38 not found` など | Python 3.10 / 3.12 用の pyrealsense2 を入れた。システムの Python 3.8 で 2 をやり直す |
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

## 確かめたこと（2026-09-29、PC2 で無線から）

- `install_offline.sh` で、システムの Python 3.8 に pyrealsense2 2.55.1 が入り、RealSense 1 台が見えた。
- `--source dummy` で、無線（ノート PC）への配信・`probe_rgbd`・`locate_bottle --detector color`・`record` が動いた。
- 本物のカメラでは、Unitree の `videohub_pc4` が `/dev/video4`（カラー）を開いていて、`Device or resource busy` で起動できなかった。
- 見るだけ・受け取るだけの確認スクリプトで次がわかった（詳細は `REAL_DAY_PROCEDURE.md`）:
  - `v4l2_list.py`: 深度は video0、カラーは video4 だけ（USB の別の部分）
  - `depth_only_check.py`: 深度だけなら videohub が動いたままでも 28 fps で取れる
  - `videohub_check.py`: カラーは videohub から 1920x1080 の JPEG で受け取れる（Go2 用 `VideoClient`、52 枚/秒）
- 次回、`rgbd_server.py` に `--source videohub`（カラーは videohub、深度は直接、位置合わせは自前）を足して確かめる。

## 確かめたこと（2026-09-30、実機なし）

- `--source videohub` を足した。pyrealsense2 と VideoClient の代わりを使うテスト（`tests/test_depth_align.py`）で:
  - カラーは開かず（開こうとすると失敗する代わりを使った）、深度だけを開く。カラーの内部パラメータは設定から読む
  - 位置合わせ: 同じカメラどうしなら入力と完全に一致する。箱の角が計算で投影した位置の 1 画素以内に来て、
    箱の中に穴が空かない。手前の物が奥の物を隠す。librealsense の回転の並び（列の順）どおりに読む
  - サーバの配信ループを通して、受け取る側に 640x360 のカラー（赤が赤のまま）と深度が届く
- PC2 と同じ Python 3.8・numpy 1.24.4（オフラインのファイル）でもテストが通り、位置合わせは 1 フレーム約 28 ms（ノート PC）。
- **本物の videohub と RealSense での確認はまだ**（次回 G1 につないだときに行う）。
