# 実機日の手順書（ボトル押しと、エレベーターの呼びボタン）

コマンドはすべてラボ PC のリポジトリ直下（`G1_Hackason/`）で、1行ずつ実行する。
`<NIC>` は G1 につないでいる有線 NIC の名前（`ip -br a` で確認し、`configs/arm.yaml` の `network_interface` にも書く）。

> **最新の記録と再開手順: [DAILY_LOG_20261014_labpc.md](DAILY_LOG_20261014_labpc.md)**（ラボ PC・有線。段階0〜3の途中まで。arm_sdk が効く・実機では `--gravity-scale 0.5`）。同じ日のノート PC 側（無線）の記録は [DAILY_LOG_20261014_laptop.md](DAILY_LOG_20261014_laptop.md)。

## 時間の目安（2026-09-29 のリハーサル。ループバックの模擬ロボット）

機械が動いている時間の実測。人の作業（接続、ロボットの姿勢を整える、腕を手で動かす、ボトルを置き直す）は含まない。

| 段階 | 実測 | 内容 |
|---|---|---|
| 0 接続確認 | 約 7 秒 | `check_connection.py --rgbd`、`probe_rgbd.py --save` |
| 1 収録 | 約 45 秒 | ボトル 20 秒、ボタン 1 秒に 1 枚 × 15 秒、Enter × 3（本番のボタン 150〜300 枚は 1 秒に 1 枚で 3〜5 分） |
| 2 経路の判定 | 約 15 秒 | `move_arm_real.py` の dry-run と送信（各 8 秒）。プランB への切り替えを含めると +13 秒 |
| 3 ティーチング・FK・重力補償 | 約 25 秒 | 重力補償 0 / 0.5 / 1.0 の比較が各 6 秒 |
| 4 較正 | 約 6 秒 | 3 か所（人が指先を触れさせる時間は含まない） |
| 5 押し込み | 約 108 秒 | dry-run 54 秒 + 送信 54 秒（1 回の押し込みは、経由の姿勢を通る行き来を含めて約 54 秒） |
| 6 繰り返し | 約 53 秒 / 回 | 置き場所を変えるたびに 1 回 |

リハーサル全体（故障の場面を含む）は約 8 分。

## 安全のルール（全段階で守る）

- **人が常にリモコンを持つ。緊急時は L2+B（ダンピング）。**
- 腕を手で動かす（ティーチング・較正）ときはダンピング状態にする。**ダンピング中は全身が脱力するので、
  必ず座った状態か吊り下げた状態で行う**（`teach.py` / `calibrate.py` は開始時に確認して Enter を待つ）。
- プランB（デバッグモード + lowcmd）もバランス制御が止まるので、座った状態か吊り下げた状態で行う。
- 実機に送るスクリプトは、まず dry-run（既定）→ `--execute`（確認モードで各段階 Enter）の順。
- 実機で撮ったデータは取り直せない。収録が終わるたびにバックアップする。

## 当日の流れ（HANDOFF 8章）

| 段階 | 内容 | 使うもの（この手順書の節） | 失敗したときの対応 |
|---|---|---|---|
| 0 | 接続確認（RGB・深度・lowstate、`mode_machine` = 5） | 「段階0: 接続確認」「深度付きカメラサーバ」 | 深度がだめなら「pyrealsense2 が使えなかった場合の切り替え」 |
| 1 | ボタン撮影とボトル周りの収録 | 「収録（タスク5）」 | 必ずやり切る |
| 2 | 経路の判定（arm_sdk が効くか） | 「段階2: 経路の判定」 | 効かなければ座った状態（吊り下げ）でプランB |
| 3 | ティーチング、FK と実物の比較 | 「段階3: ティーチングと FK の確認」「重力補償の確認」 | URDF と関節の対応、指先の点を確かめる |
| 4 | 較正 | 「段階4: 較正」 | 一定のずれなら補正値を入れる |
| 5 | dry-run → 確認モードで低速 → 押し込み | 「段階5・6: 押し込み」（タスク6） | 手前の姿勢まで行ければ成功 |
| 6 | 置き場所を変えて繰り返す | 同上 | 全試行を収録する |
| 7 | エレベーターの呼びボタン: 写真で確かめる → dry-run → 押す | 「段階7: エレベーターの呼びボタン」 | 見つけられなければ写真にラベルを付けて次の回に。arm_sdk が効かなければ行わない |

## 事前準備（ラボ PC）

⚠️ ラボ PC では `~/.bashrc` が別の古い CycloneDDS を `CYCLONEDDS_HOME` に入れている。そのままだと DDS の初期化で
`*** buffer overflow detected ***` で落ちるので、ターミナルを開くたびに最初に実行する（2026-10-14）:

```bash
export CYCLONEDDS_HOME=$PWD/G1_HuggingFace/cyclonedds/install
```

ラボ PC（x86_64）には SETUP.md のフルセット（lerobot 込み）が入っている前提。
ボタン押し用のパッケージ（IK 用の Pinocchio = `pin` など）を追加で入れる:

```bash
G1_HuggingFace/venv/bin/pip install -r Button_Press/J1-gen/requirements.txt
```

公式モデルを取得する（初回だけ。ネットワークが必要）:

```bash
bash Button_Press/J1-gen/sim/fetch_models.sh
```

入ったかの確認（最後に `OK`）:

```bash
PYTHON=G1_HuggingFace/venv/bin/python bash Button_Press/J1-gen/tests/run_tests.sh
```

G1 につないでいる有線 NIC の名前を調べ、`configs/arm.yaml` の `network_interface` に書く:

```bash
ip -br a
```

## プランB（デバッグモード + lowcmd）を使う場合

- 座った状態か、吊り下げた状態で行う（開始時にスクリプトが確認を求める）。
- **開始直後に、まず腰の角度を lowstate で確かめる。** 腰の3関節は Kp=300 / Kd=3.0 で開始時の角度に
  保持している（MuJoCo では、公式 low_level サンプルの Kp=40 だと上体の重さで腰ピッチが約 19° 倒れた）。
  開始時から `safety.waist_max_deviation_rad`（既定 0.05 rad ≈ 2.9°）以上ずれると自動で中止する。
  スクリプトを動かしている間に、別のターミナルで腰の角度を表示して確かめる（何も送信しない）:

  ```bash
  G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_connection.py --seconds 1
  ```

  「腰 yaw / roll / pitch」が開始前とほぼ同じ（1° 以内）なら OK。倒れていくなら、すぐに止める（Ctrl+C、危なければ L2+B）。

## 重力補償の確認

腕を同じ目標へ動かし、重力補償の倍率を **0 → 0.5 → 1.0** の順に変えて、腕の下がり方（追従誤差）を比べる。
表示される「最大の追従誤差」を記録する。

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path lowcmd --execute --gravity-scale 0
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path lowcmd --execute --gravity-scale 0.5
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path lowcmd --execute --gravity-scale 1.0
```

- MuJoCo（プランB の条件）では、倍率 0 / 0.5 / 1.0 で腕の追従誤差が 1.74° / 0.87° / 0.01°。
- 腰の重力補償（`--waist-gravity-scale`、プランB のときだけ効く）は、腕の確認のあとで必要なら試す。
- lowstate の IMU が pelvis のものかは未確認。倍率 1.0 で逆に下がり方が大きくなる場合は、IMU の向きを疑う。

## 無線でノート PC から行う準備（有線のラボ PC を使う前に）

G1 が無線 LAN につながっていれば、カメラまわり（PC2 の準備、深度付きの配信、検出、収録）はノート PC から進められる。
**無線では DDS が届かないので、lowstate の読み取りと腕の制御はしない**（それは有線のラボ PC で行う）。

- PC2 の無線の IP は `configs/camera_wifi.yaml` の `pc2_host`（2026-09-29 は 192.168.0.82。DHCP なので変わることがある。
  変わったら `pc2_host` を直すか、各スクリプトに `--host <IP>` を付ける）。以下では `<PC2>` と書く。
- **PC2 で変えてよいのは `~/button_press/` の下と、pip でのパッケージのインストールだけ。** sudo は使わない
  （libusb は `.deb` の中身を `~/button_press/libusb_local/` に取り出すだけ）。Unitree のシステムのサービスや設定には触れない。
- 「PC2 で」とあるコマンドは、ノート PC から `ssh unitree@<PC2>` で入って 1 行ずつ実行する。

### 1. PC2 の確認（PC2 で。どれも表示するだけ）

```bash
lsusb | grep -i intel
```
```bash
python3 --version
```
```bash
ls ~/miniforge3/envs 2>/dev/null
```
```bash
python3 -m pip --version
```
```bash
ldconfig -p | grep libusb-1.0
```
```bash
python3 -c "import pyrealsense2 as rs; print(rs.__version__)"
```
```bash
ps aux | grep -v grep | grep run_g1_server
```

- `lsusb` に RealSense が出なければ、深度はこの PC2 では使えない（PC1 につながっている）。
  ただし PC2 の `lsusb` は名前の表が古く、D435i（`8086:0b3a`）を **`Intel Corp. 4-Port USB 3.0 Hub`** と表示する
  （2026-09-29。ほかの班が「見えていない」と誤判定した）。`8086:0b3a` の行があれば D435i はつながっている。
  確実なのは `bash ~/button_press/real/depth_server/start_rgbd_server.sh --list-devices`（pyrealsense2 を入れたあと）。
- 配信サーバと pyrealsense2 は、**PC2 のシステムの Python（3.8）**で動かす。conda の `lerobot` 環境（3.12）用の
  pyrealsense2 は PC2 の glibc では動かなかった（2026-09-29）。conda の環境に入っていたら `conda deactivate` で抜ける。
- **`run_g1_server.py` を起動したコマンド（`ps` に出た行）を控えておく。** 最後に同じコマンドで元に戻すため。

### 2. オフラインのファイルを送って pyrealsense2 を入れる

ノート PC で（リポジトリ直下）:

```bash
ssh unitree@<PC2> mkdir -p button_press
```
長いコマンドは貼り付けで 2 行に分かれて失敗する（送り先が抜けると、ノート PC の中でコピーしてしまう）ので、1 つずつ送る:

```bash
scp -r _local/button_press/offline unitree@<PC2>:button_press/
```
```bash
scp -r Button_Press/J1-gen/common unitree@<PC2>:button_press/
```
```bash
scp -r Button_Press/J1-gen/configs unitree@<PC2>:button_press/
```
```bash
scp -r Button_Press/J1-gen/real unitree@<PC2>:button_press/
```

PC2 で（conda の環境から抜けた状態で）:

```bash
bash ~/button_press/offline/install_offline.sh
```

- システムの Python 3.8 に `--user`（`~/.local` の下）で pyrealsense2 2.55.1 を入れる。入れる前に、必要な glibc が
  PC2 にあるかを比べる。numpy などは足りないものだけ入れる。libusb はシステムのもの（PC2 にはある）を使う。
- 最後に `pyrealsense2 2.55.1.6486 OK、RealSense 1 台` と出れば成功。

### 3. PC2 で配信サーバを起動する

RealSense は 1 つのプログラムしか開けないので、`run_g1_server.py` を `--camera` なしで起動し直す。
1 で控えた行が `--camera` 付きなら、そのプロセスを止めて（起動したターミナルで Ctrl+C。`nohup` で起動していれば
`kill <PID>`）、同じコマンドから `--camera` を外して起動する（DDS の中継は、有線のラボ PC から使うときのために残す）。

配信サーバを、無線に合わせて 5 fps で起動する（PC2 で。別のターミナル）:

```bash
cd ~/button_press
```
```bash
bash real/depth_server/start_rgbd_server.sh --list-devices
```
```bash
bash real/depth_server/start_rgbd_server.sh --max-fps 5
```

### 4. ノート PC で受け取って保存する

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/probe_rgbd.py --camera-config camera_wifi.yaml --save
```

fps（5 前後）、内部パラメータ、画像中央の深度、深度が 0 の画素の割合を見る。保存先は `_local/button_press/probe/`。

### 5. ボトルを検出して 3 次元の位置を出す（lowstate は使わない）

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --live --no-lowstate --camera-config camera_wifi.yaml --frames 5
```

- カメラ座標（x 右、y 下、z 前）と、腰を 0° と仮定した pelvis 座標が出る（「腰の角度は仮定」と警告し、結果の JSON にも残す）。
- YOLO が見つけなければ `--detector color`（`configs/localize.yaml` の色の範囲をボトルのラベルの色に合わせる）。
- 枠と基準点が胴に乗っているかを、保存された画像（`_local/button_press/locate/`）で見る。

### 6. RGB と深度を収録する（lowstate は記録しない）

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label bottle_wifi --no-lowstate --camera-config camera_wifi.yaml --duration-s 60
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button_wifi --no-lowstate --camera-config camera_wifi.yaml --mode enter
```

終わったらバックアップする（取り直せない）。

### 7. PC2 を元の状態に戻す

1. PC2 で配信サーバを止める（起動したターミナルで Ctrl+C）。
2. `run_g1_server.py` を止め、**1 で控えたコマンドのまま（`--camera` あり）** 起動し直す。
3. ノート PC で、RGB が元どおり届くかを確かめる:

   ```bash
   G1_HuggingFace/venv/bin/python Perception/real/probe_zmq_camera.py --host <PC2> --timeout 60
   ```

`~/button_press/` と pip で入れた pyrealsense2 は、そのまま残してよい（有線のラボ PC から使うときにも使う）。

## 事前準備: PC2 用のオフラインのファイル（実機日の前に、ネットにつながったマシンで）

PC2 はインターネットにつながっていない可能性が高いので、pyrealsense2（PC2 のシステムの Python 3.8 用）と、
配信サーバが使う numpy などと、libusb-1.0 の .deb を事前にダウンロードしておく（約 47 MB）:

```bash
bash Button_Press/J1-gen/real/depth_server/fetch_offline_packages.sh
```

`_local/button_press/offline/` にできる。手元の PC で用意した場合は、このフォルダをラボ PC に持っていく
（`_local/` は git に入らないので、push しても届かない）。

## 深度付きカメラサーバ（PC2）

詳しい手順は [depth_server/README.md](depth_server/README.md)。流れ:

1. RealSense が PC2 につながっているかを確かめる（PC2 で）:

   ```bash
   lsusb | grep -i intel
   ```

2. Python の版を確かめ、pyrealsense2 が無ければオフラインのファイルで入れる。ラボ PC から PC2 へコピー:

   ```bash
   ssh unitree@192.168.123.164 mkdir -p button_press
   ```
   ```bash
   scp -r _local/button_press/offline unitree@192.168.123.164:button_press/
   ```

   PC2 で（conda の環境から抜けた状態で）:

   ```bash
   python3 --version
   ```
   ```bash
   bash ~/button_press/offline/install_offline.sh
   ```

3. サーバのファイルを PC2 に置き（README の 3）、`run_g1_server.py` を **`--camera` なしで**起動し（README の 4）、
   `start_rgbd_server.sh` でサーバを起動する。
4. ラボ PC で届いているかを確かめて保存する:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/probe_rgbd.py --save
   ```

## 「Device or resource busy」でカメラを開けない場合（2026-09-29 に起きた）

`start_rgbd_server.sh` が `xioctl(VIDIOC_S_FMT) failed, errno=16 Last Error: Device or resource busy` で止まるのは、
ほかのプロセスが RealSense を開いているため。2026-09-29 には、G1 の電源が入り直したあとに、Unitree の
システムのサービス **`/unitree/module/video_hub_pc4/videohub_pc4 /dev/video4`**（root で動く。ロボットの起動と一緒に
立ち上がる）が `/dev/video4` を開いていた。見るだけで確かめるコマンド（PC2 で）:

```bash
ps -eo user,pid,etime,cmd | grep -i -E "realsense|camera|video|g1_server|rs-|python" | grep -v grep
```

- videohub は Unitree のサービスなので、**勝手に止めない**。止めてよいか・元に戻す方法は、ロボットの管理者に確かめる。
  PC2 の中の問題なので、**有線のラボ PC から使うときも同じく起きる**
- `run_g1_server.py --camera` も既定で `/dev/video4` を開くので、同じく開けない
- カメラ以外（無線の経路・配信・位置を求める処理・収録）は、PC2 で `--source dummy --no-legacy-rgb` を付けて
  配信サーバを起動し、ノート PC で `locate_bottle.py --detector color` などを動かせば確かめられる（ダミーの赤い箱を検出する）

### videohub が動いたままでも、カラーと深度は取れる（2026-09-29 に確かめた）

PC2 には `v4l2-ctl` が無いので、`real/depth_server/v4l2_list.py`（形式の一覧を聞くだけ。映像は開かない）で
各 `/dev/video` の中身を調べた。D435i は USB の中で 2 つに分かれている:

| USB の部分 | /dev | 中身 |
|---|---|---|
| 1.0（深度側） | video0 | 深度（Z16） |
| | video2 | IR（GREY / Y8I / Y12I。UYVY も出せるが IR カメラの口） |
| | video1, video3 | 補助データ（映像ではない） |
| 1.3（カラー側） | **video4** | **カラー（YUYV）← videohub が開いている** |
| | video5 | 補助データ |

- **カラーのカメラの口は video4 だけ。** ほかの班の「内蔵 video2」（Mapping の `dual_webcam_server.py`）は深度側の IR の口。
- **深度だけなら開ける。** `depth_only_check.py`（pyrealsense2 で深度だけを開く。カラーは開かない）で、
  videohub が動いたまま 30/30 枚・28 fps で取れた（1 枚目は有効な画素 45%、30 枚目は 96%。最初の数枚は捨てる）。
- **カラーは videohub に頼めば受け取れる。** videohub のプログラムの中に `rt/api/videohub/request` / `response` があり
  （DDS。設定は `eth0`）、`unitree_sdk2py` の Go2 用 `VideoClient`（サービス名 `videohub`、API 1001 `GetImageSample`）が
  そのまま使えた。`videohub_check.py` で **1920x1080 の JPEG（約 133 KB）を 30/30 枚、52 枚/秒**で受け取れた。
  PC2 のシステムの Python に `unitree_sdk2py` が入っている（`~/unitree_sdk2_python`）。DDS はノート PC まで届かないので、
  PC2 の中で受け取る。
- 位置合わせに使う値（カラーを開かずにカメラの設定から読めた）:
  カラー 640x480 の内部パラメータ fx=605.5 fy=605.1 cx=318.6 cy=254.8、深度 fx=384.8 fy=384.8 cx=325.4 cy=242.1、
  深度 → カラーの平行移動 [0.015, -0.0004, 0.0003] m。1920x1080 用の内部パラメータはまだ読んでいない。
- シリアル番号は 2 つある: pyrealsense2 の番号 `250122075509`（`--serial` に使うのはこちら）と、
  `/dev/v4l/by-id` の名前に入る USB の番号 `254843066801`。同じ 1 台。

見るだけ・受け取るだけの確認（PC2 で。どれも videohub を止めない）:

```bash
python3 ~/button_press/real/depth_server/v4l2_list.py
```
```bash
python3 ~/button_press/real/depth_server/depth_only_check.py
```
```bash
python3 ~/button_press/real/depth_server/videohub_check.py
```

## 次回やること（2026-09-29 の無線の回の続き）

今の `rgbd_server.py` は、カラーと深度を 1 つの pipeline で開くので、videohub が動いていると起動できない。
次回は「**カラーは videohub から、深度は RealSense から直接**」の配信を作って確かめる。

G1 につなぐ前（ノート PC だけでできる）: **2026-09-30 に済んだ**（`tests/test_depth_align.py`。実機ではまだ）

1. `rgbd_server.py` に `--source videohub` を足す（今の `realsense` と `dummy` は残す）。
   - カラー: `VideoClient.GetImageSample()` の 1920x1080 JPEG を、縦横比を保って 640x360 に縮める
     （縮める倍率に合わせて、1920x1080 用のカラーの内部パラメータを縮める）
   - 深度: pyrealsense2 で深度だけを開く（`depth_only_check.py` と同じ）
   - 位置合わせ: 深度の内部パラメータ・深度 → カラーの位置関係・カラーの内部パラメータで、深度をカラーの画素に並べ直す
     （pyrealsense2 の `align` はカラーを同じ pipeline で開いていないと使えないので、自分で計算する）
   - カラーと深度は別の経路なので、撮った瞬間が揃わない（止まっている物なら問題ない。受け取った時刻の差を記録しておく）
   - 送る形式（`common/rgbd_protocol.py`）は変えない。受け取る側（probe_rgbd / locate_bottle / record）はそのまま使う
2. 位置合わせの計算を、ダミーの値で単体テストする

G1 につないだら（無線でよい。PC2 で `~/button_press/real/` を送り直してから）:

1. 1920x1080 の画像が頭カメラの映像で、上下・左右が逆でないかを見る（`videohub_check.py` が保存する
   `~/button_press/videohub_sample.jpg` をノート PC に `scp` で持ってきて見る）
   - 2026-09-29 の 1 枚は床（木目の床と、右上の端に椅子の車輪）で、下向きに付いた頭カメラの映像と合う。
     上下・左右の向きは、ボトルなど向きのわかる物を前に置いて確かめる
2. PC2 で配信する（`~/button_press` で。conda の環境には入らない）:

   ```bash
   bash real/depth_server/start_rgbd_server.sh --source videohub --max-fps 5
   ```

   「カラー: videohub の 1920x1080 を 640x360 に縮めて送る」と内部パラメータが出て、5 秒ごとに
   「カラーと深度の時刻の差 … ms、位置合わせ … ms」が出れば動いている。
   そのあと「無線でノート PC から行う準備」の 4〜6（probe_rgbd → locate_bottle（YOLO）→ record）を行う。
   - 深度がカラーの物の輪郭に重なっているかを保存画像で確かめる（ずれていれば、深度 → カラーの位置関係か、
     videohub の画像の向きを疑う）
   - 赤い物が赤く写るか（videohub の JPEG の色の並び）も見る
3. PC2 を元に戻す: 配信サーバを止めるだけ（2026-09-29 は `run_g1_server.py` が動いていなかったので、起動し直さない）

## pyrealsense2 が使えなかった場合の切り替え

次のどれかになったら、深度はあきらめて切り替える（RGB の収録と、ボトル押しそのものは続ける）:

- `lsusb` で RealSense が PC2 に見えない（PC1 につながっている）
- `install_offline.sh` が失敗する（Python の版に合う wheel が無い、など）
- `start_rgbd_server.sh` が起動しない、または `probe_rgbd.py` で深度が届かない

切り替えの手順:

1. PC2 で `rgbd_server.py` を止める（動いていれば Ctrl+C）。RealSense は 1 つのプログラムしか開けないので、
   止めないと次の手順でカメラを開けない。
2. PC2 で `run_g1_server.py` を止め、**`--camera` を付けて**起動し直す（RGB は OpenCV 経由で、既存と同じ形式・ポート 5555）:

   ```bash
   python -u src/lerobot/robots/unitree_g1/run_g1_server.py --camera
   ```

   画像が来ないときは、カメラのデバイス番号（既定 4 = `/dev/video4`）が違う可能性がある。PC2 で一覧を見て、
   `--camera-device <番号>` で指定する:

   ```bash
   ls -l /dev/v4l/by-id/
   ```

3. ラボ PC で RGB が届いているかを確かめる（既存の Perception の確認スクリプト）:

   ```bash
   G1_HuggingFace/venv/bin/python Perception/real/probe_zmq_camera.py --host 192.168.123.164 --timeout 60
   ```

4. 対象（ボトル）の位置は、「定規で測った位置」モード（`--target manual`）で与える。段階3で、ボトルの表面に
   中指の先を触れさせた姿勢を `touch_bottle` などの名前で記録しておき、ボトルを動かしたら動かした量
   （pelvis の向き。x 前、y 左、z 上 [m]）を `--offset` に書く:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk --target manual --taught-pose touch_bottle --offset 0 0 0 --no-camera
   ```

## ボトルの位置を求める（タスク4）

- **検出するときは、腕をカメラの視野から外す**（手がボトルを隠さないように）。
- ボトルはラベル付きのもの（透明な PET ボトルは深度が取れない）。
- ライブで確かめる（腰の角度は lowstate から読む）:

  ```bash
  G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --live --frames 5
  ```

- YOLO がボトルを検出するか、枠と基準点が胴に乗っているかを、保存された画像（`_local/button_press/locate/`）で見る。
- 深度が使えないときは、`probe_zmq_camera.py` などで撮った RGB で YOLO の検出だけを確かめる。

## 収録（タスク5）

「必ずやり切る」段階（HANDOFF 8章の段階1）。lowstate も一緒に記録する（NIC は `configs/arm.yaml`）。
`--note` に、置いた場所・照明・ボトルの種類などを書いておく。

ボトルのまわり（全フレーム。Ctrl+C で止める）:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label bottle --note "机の上、ラベル付きボトル"
```

ボタンの撮影（①の学習用。150〜300 枚。背景を変えたもの 2〜3 割、ボタンが写っていないもの 1 割）。
構図を決めてから撮るなら `enter`、動かしながら撮るなら `interval`:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button --mode enter
```
```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button --mode interval --interval-s 1
```

深度が使えないときは `--rgb-only` を付ける（`run_g1_server.py --camera` の 5555 から受信する）。

**収録が終わるたびに、バックアップする**（取り直せないため）。終了時に表示される `cp -r ...` を実行する。

## 段階0: 接続確認

何も送信しない。lowstate の頻度、`mode_machine`（5 であること）、腰・腕のモータの mode、腰の角度、
指先の位置（FK）を表示する。`--rgbd` で深度付きストリームも確かめる:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_connection.py --rgbd
```

- `mode_machine` が 5 でなければ中止（機体構成が違う。IK のモデルが合わない）。
- モータの mode が 0 なら、リモコンでダンピング（FSM 1）に入れて有効化する（指令側からは有効化できない）。
  プログラムを終了するたびにゼロトルクに戻ることがあるので、**実行の前に毎回確かめる**。

## 段階2: 経路の判定（Dev/Navigation の g1-starter-kit を直接実行する）

g1-starter-kit はこのブランチに取り込まず、Dev/Navigation を別のフォルダ（`../G1_nav`）に出して実行する。
今の作業フォルダには影響しない（`git worktree`: 同じリポジトリの別のブランチを、別のフォルダに取り出す仕組み）:

```bash
git fetch origin
```
```bash
git worktree add --detach ../G1_nav origin/Dev/Navigation
```

1. モーションコントローラの状態と、どちらの経路が効くかを見る（何も動かさない）:

   ```bash
   G1_HuggingFace/venv/bin/python ../G1_nav/Teleop/vendor/g1-starter-kit/tools/mode_check.py --iface <NIC>
   ```

2. arm_sdk が効くかを、関節を 1 つだけ約 3° 動かして確かめる（右肩ピッチ = motor 22。確認を求められる）:

   ```bash
   G1_HuggingFace/venv/bin/python ../G1_nav/Teleop/vendor/g1-starter-kit/tools/armsdk_probe.py --iface <NIC> --joint 22
   ```

   - 「✅ arm_sdk が効いています」→ プランA（`--path arm_sdk`）で進める
   - 「❌ arm_sdk が効いていません」→ **座った状態か吊り下げた状態にしてから**、デバッグ状態へ切り替えてプランB:

     ```bash
     G1_HuggingFace/venv/bin/python ../G1_nav/Teleop/vendor/g1-starter-kit/tools/mode_check.py --iface <NIC> --release
     ```

     終わったら元の状態へ戻す:

     ```bash
     G1_HuggingFace/venv/bin/python ../G1_nav/Teleop/vendor/g1-starter-kit/tools/mode_check.py --iface <NIC> --restore
     ```

3. 選んだ経路で、腕を小さく（肩ピッチ −5°、肘 +5°）動かして戻す。まず dry-run、次に `--execute`:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path arm_sdk
   ```
   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path arm_sdk --execute
   ```

   プランB なら `--path lowcmd`。**開始直後に腰の角度を確かめる**（別のターミナルで `check_connection.py`。
   上の「プランB（デバッグモード + lowcmd）を使う場合」）。

実機日が終わったら、取り出したフォルダを片付ける:

```bash
git worktree remove ../G1_nav
```

## 段階3: ティーチングと FK の確認

⚠️ 腕を手で動かすので、ダンピング状態にする。**全身が脱力するので、必ず座った状態か吊り下げた状態で行う。**

1. 押し込み姿勢を記録する（腕を手で動かし、中指の先がボトルに触れる少し手前で Enter）。
   `configs/taught_poses.yaml` に保存され、タスク6で IK の初期値に使う:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/teach.py --name press_bottle --arm right
   ```

2. FK と実物を比べる。FK で求めた中指の先を頭カメラの画像に丸で描いて保存する（`_local/button_press/fk_check/`）:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/fk_check.py --overlay
   ```

   - 丸が画像の中の**実物の中指の先**に重なっていれば OK（FK とカメラの取り付け位置の両方が合っている）。
   - ずれていたら、指先の点（`configs/robot.yaml` の `end_effector`。今は公式メッシュの中指の先）を
     `--ee-offset X Y Z` で試しに変えて、丸が中指の先に重なる値を探す。見つかったら robot.yaml に書き写す:

     ```bash
     G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/fk_check.py --overlay --ee-offset 0.17 0.028 0.007
     ```

   - 腕をいくつかの姿勢にして、どの姿勢でも同じようにずれるか（一定のずれ）、姿勢で変わるか（関節の対応や
     モデルの違い）を記録する。

### 机の障害物の箱を作る（段階3の続き。段階5の前に必ず）

机はロボットのモデルに無いので、箱を作っておかないと「手が机の縁の下をくぐる」経路を計画で見つけられない
（MuJoCo では、両腕を下ろした姿勢から手を上げる途中に机に当たった）。腕を手で動かすので、座った状態か吊り下げで行う。

中指の先で、机の天板の**手前の縁の上**を 1 か所ずつ触って Enter。q で終わる。机が大きくて角に届かなければ、
ロボットの正面あたりの縁の上を 20〜30 cm 離して 2 点触ればよい（1 点だけでもよい）:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/teach.py --obstacle table --arm right
```

- 触った点から箱を作り、`configs/obstacles.yaml` に保存する。
  - 左右: ロボットの正面（y = 0）から左右に 1 m ずつ（`--width-m 2.0`。触った点がもっと外ならそこまで）
  - 手前の縁: 1 点なら、縁はロボットの正面に平行だと仮定する。2 点以上なら、触った点を通る直線を左右の端まで
    延ばし、一番手前になる位置にする（縁が 10° を超えて傾いていたら警告が出る。ロボットを机に正対させるとよい）
  - 奥へ 0.6 m、天板の上面から下へ 0.8 m、まわりに 3 cm の余裕（`--depth-m` / `--below-m` / `--margin-m`）
- 表示された箱の範囲が、机と合っているかを見る。
- 全体をつなぐスクリプトは、この箱も衝突の確認に使う（腕が箱に近づく・入る経路は、送る前に拒否する）。
- 机を動かしたら作り直す。

## 段階4: 較正

⚠️ 指先で触れるときは腕を手で動かすので、座った状態か吊り下げた状態で行う。
押す位置の近くで、ボトルの置き場所を変えて 3〜5 か所。各場所で、**先に腕をどけてカメラで測り、そのあとで
中指の先を触れさせる**（触れている状態で撮ると、手が写り込んで深度が狂う）:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/calibrate.py --arm right
```

- 最後に、場所ごとの補正値の候補（差）と、平均からのずれが表で出る。ばらつきが数 mm なら一定のずれなので、
  `--write` を付けてやり直すか、`configs/localize.yaml` の `calibration.offset_pelvis_m` に平均を書く。
- ばらつきが 1 cm を超える、またはずれが 5 cm を超えるときは、補正では直らない。段階3の結果（FK）と、
  頭カメラの取り付け位置（`configs/robot.yaml` の `head_camera`）を疑う。

## 作業空間の箱の調整

`configs/press.yaml` の `workspace`（手先が入ってよい箱、pelvis 座標）は仮の値。段階3で記録した押し込み姿勢の
指先の位置（`configs/taught_poses.yaml` の `fingertip_pelvis_m`）と、段階4のボトルの位置が箱の中に入っているかを見て、
必要なら広げる（押す位置のまわり ±10〜15 cm 程度にとどめる）。

## 段階5・6: 押し込み

事前に `configs/obstacles.yaml`（机の箱。段階3の `teach.py --obstacle` で作る）と、`configs/pipeline.yaml` の
`via_pose`（机の縁より手前で手を上げる経由の姿勢）を確かめておく。検出するときは腕をカメラの視野から外す。

立つ位置とボトルの置き場所（シミュレーションで確かめた範囲。`configs/pipeline.yaml` の `stand`。README の
「立つ位置（ボトル）」）:

- **押す点（ボトルの手前の面）まで 30〜40 cm、押す点が体の中心から右に 15〜25 cm、机の手前の縁まで 29 cm 以上**
  （pelvis から。左腕なら左右を反転）。ボトルは机の縁の近く（10 cm 以内）に置く
- 近い側はボトルではなく**机の縁**で決まる（手を上げる途中で手首が縁に当たる）。机が高いほど縁から離れる
- 押す点が体の正面寄り（右に 5 cm 以内）だと、右肩が胴体に当たって計画できない
- 範囲の外なら「押す点まで 47.1 cm と遠すぎる。あと 8 cm 以上近づく」のように出す（計画できればそのまま押す）

1. dry-run（何も送らない。検出・計画・指令の計算と記録だけ）。計画で拒否されたら、表示された理由
   （届かない / ぶつかる / 作業空間の外）を見て、ボトルの置き場所や `workspace` / `obstacles` を見直す:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk --label dry
   ```

2. 確認モードで送る（各段階で Enter。危なければ q か Ctrl+C、さらに危なければ L2+B）。
   q・Ctrl+C・安全確認での中止のあとは、**その場で止めて、来た経路を逆にたどって開始姿勢へ戻ってから** weight を下げる
   （段階ごとの Enter は聞かない）。戻る動きが危ないと思ったら、**もう一度 Ctrl+C**（戻らずにその場で weight を下げる。
   机のそばだと腕が下がって当たることがある）。lowstate が途切れたときは戻らない。
   「手前の姿勢まで行ければ成功」とする:

   ```bash
   G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk --execute --label try1
   ```

   プランB なら `--path lowcmd`（座った状態か吊り下げ。開始時に確認される）。押し込みの深さは `--depth-mm`（上限 30 mm）。

3. ボトルの置き場所を変えて繰り返す（段階6）。**別のターミナルで `record.py` を動かしたまま**押し、全試行を収録する。
   各回の記録は `_local/button_press/runs/<日時>_<ラベル>/`（RGB・深度・lowstate・検出・IK・送った指令）。

4. 各回の終わりに出る「体（pelvis）の動き」を書き留める（記録の `body_motion.csv` にも残る）。シミュレーションは腰を
   固定しているので、**足側のバランス制御が腰をどれだけ動かすかは、実機でしか分からない**（README の
   「押している間の体（pelvis）の動き」）。特に見るのは:
   - **狙いのずれの最大**（mm）と、そのときの段階（「押し込み」「保持・戻り」なら押した反動、「手前へ移動」なら腕を伸ばしたことによる）。
     x が + なら、押す向きに体が逃げて浅くしか押せていない。押せなかった回の原因を切り分けるのに使う
   - 「足を踏み替えた ⚠️」が出た回は、そのあとの値は当てにならない
   - 傾きの変化で IMU と脚の FK が大きく違う回は、足が浮いたか滑った
   あとから読み直すとき: `G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/body_motion.py _local/button_press/runs/<日時>_<ラベル>`

終了コード: 0 成功 / 2 目標を拒否（何も送っていない、または動かしたあと安全に止めた）/ 3 動いていない /
4 lowstate・相手・mode_machine・モータの問題 / 5 腰が倒れた / 130 中止（q、Ctrl+C）

## 段階7: エレベーターの呼びボタン

エージェントは Yada さんの評価環境のもの（`contest_agent/`、中身は `common/elevator_press.py`）を、評価環境の
実機用の口（`Button_Press/Yada/contest/run_dds.py` + `contest/robots/real_g1.py`）で動かす。シミュレーションで
確かめたことは README の「エレベーターの呼びボタン」を参照（MuJoCo・模擬 G1 で 20 / 20。実機は未確認）。

### 始める前に（条件）

- **段階2で arm_sdk が効くこと。** 立ったまま（LocoClient のバランス制御が動いたまま）で腕を動かすので、
  プランB（lowcmd、座った状態か吊り下げ）では行えない
- 段階0（接続）・段階3（FK）・段階4（較正）が済んでいること。段階4の `configs/localize.yaml` の
  `calibration.offset_pelvis_m` は、ボタンの位置にも使われる（`configs/elevator.yaml` の `calibration`）
- ⚠️ 実機用の口（`real_g1.py`）にあるのは、weight の上げ下げ、lowstate の途切れでの停止、Ctrl+C での安全な終了、
  関節の可動範囲と速さ（1.0 rad/s）の丸めだけ。**`press_bottle.py` と違い、段階ごとに Enter で止まらず、
  腕を戻し終わるまで約 13〜23 秒で最後まで動く。** 作業空間の箱・腕と体や柱との衝突は、エージェントの計画（動く前）で確かめている
- ⚠️ 実機用の口は、画像を撮った時刻と IMU の向きを入れない。時刻は 0.1 秒ごとに新しい画像とみなす
  （`image_interval_s`）。重力の補正は pelvis がまっすぐ立っているとみなす（立っていれば数度の差）
- ⚠️ 腰の PD（評価環境の `configs/contest.yaml` の `gains.waist`、kp 300）は実機で未確認（Yada さんのメモ）。
  最初の送信のときは、腰が急に動かないかを特に見る

### 1. 写真を撮って、見つけられるかを確かめる（その日のうちに）

ロボットを押す位置（下の 2.）に立たせ、段階1と同じ `record.py` でボタンを撮る。距離（25〜50 cm）・左右・高さを
少しずつ変える。**手でボタンを押して点灯した状態も**何枚か撮る（点灯の確かめ方の確認に使う）:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button_hall --mode interval --interval-s 1 \
    --note "<階・乗り場・照明>"
```

撮った収録で、classic と YOLO を並べて比べる（左 classic・右 YOLO の画像と summary.csv。5 枚に 1 枚）:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_button_photos.py \
    _local/button_press/recordings/<日時>_button_hall --every 5
```

| 結果 | 次 |
|---|---|
| classic で「pair: OK」が多い | `configs/elevator.yaml` の `detector: classic` のまま進む |
| YOLO だけ「pair: OK」が多い | 下のコマンドの前に `J1GEN_BUTTON_DETECTOR=yolo` を付ける |
| どちらもだめ（枠が無い・▲▼ の取り違え・1 組を選べない） | 押すのはやめる。写真にラベルを付けて学び直す（README の「実機の写真を学習データに足す」）。次の回に持ち越す |

「pair: NG」の理由は summary.csv に出る（例: 間隔が 5〜12 cm の外 → `configs/elevator.yaml` の `pair_spacing_m` を、
本物のボタンの間隔に合わせて直す）。

### 2. ロボットを立たせる位置

シミュレーションで確かめた範囲（`configs/elevator.yaml` の `press.stand`。README の「立つ位置」）に入れる:

- 柱の正面に向けて立たせ、**pelvis から柱の面まで 29〜40 cm**、▲▼ の中点は体の中心から左右 6 cm 以内。
  まずは真ん中の **32 cm 前後**から試す（近いほど、カメラの角度のずれによる位置の推定の横ずれが小さい）
  - 遠すぎると、41 cm 以上で高い ▲（床から 1.10 m）が頭カメラの上に切れて見つからない
  - 近すぎると、27〜28 cm で高い ▲ への腕の計画が立たないことがある（腕が胴体に当たる・肘の範囲を超える）
  - 範囲の外なら、エージェントが「柱まで 43.2 cm と遠すぎる。あと 4 cm 以上近づく」のように、どちらへ何 cm
    動けばよいかを出す（計画できればそのまま押す）。「見つけられない」で止まったときも、立つ位置を確かめるよう出る
- 一般用の ▲ の高さは床から 1.03〜1.10 m を想定（本番の写真からの推定。実測していない）。
  これより高い・低いときは、3. の dry-run で計画が立つかで判断する
- 腕は下ろしておく（右腕が視野に入ると、ボタンを隠す）

### 3. dry-run（指令を送らない）

```bash
G1_HuggingFace/venv/bin/python Button_Press/Yada/contest/run_dds.py --agent Button_Press/J1-gen/contest_agent \
    --network-interface <NIC> --camera-host <PC2 の IP> --target up --instruction "上の階" --no-loco --dry-run
```

- `--no-loco`: J1-gen のエージェントは足を動かさないので、LocoClient につながない（立つのはリモコンで）
- 見るところ:
  - `[elevator] look: 見つけた: up=[x, y, z] …、down=[…]` — x がメジャーで測った柱までの距離と 1〜2 cm 以内か。
    ▲ と ▼ の z の差が本物の間隔と合うか
  - `[elevator] look: 計画（…）` — 計画が立った。「届かない」「当たる」が出たら、2. の位置を直す
  - dry-run では腕が動かないので、そのあとの「点灯していない → 押し直す → 確かめられない」と、最後の
    「エージェント側の結果: gave_up」は気にしない（模擬 G1 で確かめた。2026-10-12）
- 「見つけられない」で止まるときは、1. に戻る

### 4. 送る（押す）

**人がリモコンを持ち、危なければ L2+B。** 別のターミナルで `record.py` を動かしたまま行う（あとで見直すため）。
最初の 1 回は、押し直しを止めておく（点灯の確かめ方が本物のボタンで合っているかが分からないため）:
`configs/elevator.yaml` の `press.max_retries` を 0 にする。

```bash
J1GEN_BODY_LOG=_local/button_press/body_log \
G1_HuggingFace/venv/bin/python Button_Press/Yada/contest/run_dds.py --agent Button_Press/J1-gen/contest_agent \
    --network-interface <NIC> --camera-host <PC2 の IP> --target up --instruction "上の階" --no-loco
```

- `J1GEN_BODY_LOG`: 毎周期の関節角・IMU・段階を `_local/button_press/body_log/<日時>_<up|down>/` に記録する
  （押している間に足側のバランス制御が腰をどれだけ動かしたかを見るため。ボトルの段階5・6の 4. と同じ）。
  終わったら、各回で次を実行して「狙いのずれの最大」とその段階を書き留める:

  ```bash
  G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/body_motion.py _local/button_press/body_log/<日時>_<up|down>
  ```

  点灯しなかった回で、狙いのずれの x が + に数 mm 以上あれば、押した反動で体が逃げて浅くしか押せていない
  （較正のずれとは別の原因）

- 動き（シミュレーションの実測）: 見る 0.3 秒 → 手前の姿勢へ 6 秒（経由の姿勢を通る）→ 止まって補正 1 秒 →
  押し込み 2.5 秒 → 0.6 秒保つ → 戻る 1 秒 → 点灯を確かめる →（押し直すなら、もう一度押し込みから）→
  **来た経路を逆にたどって、腕を始めの姿勢へ戻す（手前の姿勢へ来たときの時間 + 1.5 秒ほど。約 8 秒）** → 終わり。
  始めから終わりまでは約 13〜23 秒（点灯で終わらせない診断、2026-10-14）
- 腕を戻してから終わるので、終わったあとに weight が 0 に戻っても、腕は柱のそばから動かない。
  **腕が戻る前に Ctrl+C で止めると、腕は柱のそばから元の姿勢へ動く**（柱に当たることがある）。
  止めるときは、腕の位置を見てから
- **点灯したかは人が見て記録する**（実機ではプログラムは読めない）。エージェントの「点灯していない」の表示と
  食い違ったら、`press.lit_brightness_rise`（今は明るさ +25）を見直す
- 点灯しなかったとき: 指先がボタンに届いていないなら、カメラの位置の推定のずれ（シミュレーションでも、カメラの
  取り付けのずれで押す向きに最大 19 mm ずれた）。段階4の較正をやり直すか、`press.planner.press_depth_m` を
  5 mm ずつ深くする（上限 `max_press_depth_m` 30 mm）
- ボタンの端を押した・外したとき: 左右・上下のずれ。シミュレーションでも最大 13.5 mm ずれた。本物のボタンが
  小さい（半径 15 mm 未満）と外れやすい。較正で直す
- `▼` も同じく `--target down` で試す

止めるとき: Ctrl+C（腕の weight を 2 秒で 0 に戻す）。動きがおかしいとき（腕が柱に向かって速く動く、腰が回る）は
すぐ L2+B。

## 前日までのリハーサル

手元の PC かラボ PC で、この手順書の流れを模擬ロボットで通しで確かめられる（約 8 分。G1 はつながなくてよい）:

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/rehearsal.py
```

模擬ロボットはこのマシンの中（口 `lo`、domain 1）だけで動くので、G1 を LAN につないだままでも混ざらない。

呼びボタン（段階7）は、Yada さんの評価環境の模擬 G1（実機と同じ DDS + ZMQ の口）で通しで確かめられる
（20 試行で約 7 分。dry-run の表示を見るなら `--seed 0` と、`--client` の中に `--no-loco --dry-run` を足す）:

```bash
P=G1_HuggingFace/venv/bin/python
$P Button_Press/Yada/contest/evaluate_dds.py --seeds practice \
    --client "$P Button_Press/Yada/contest/run_dds.py --agent Button_Press/J1-gen/contest_agent --no-loco"
```

2026-10-12: classic 20 / 20（平均 8.7 秒）、YOLO（`J1GEN_BUTTON_DETECTOR=yolo` を前に付ける）20 / 20（平均 10.2 秒）。
結果は `_local/button_press/rehearsal/<日時>/report.md`。

## 実機日が終わったら

- 収録（`_local/button_press/recordings/`）と、較正・FK 確認の画像（`_local/button_press/`）をバックアップする。
- `configs/taught_poses.yaml`、`configs/obstacles.yaml`（机の箱）、`configs/localize.yaml`（補正値）、`configs/robot.yaml`（指先の点を直した場合）、
  `configs/arm.yaml`（NIC 名など）の変更をコミットして残す。
- `git worktree remove ../G1_nav`
