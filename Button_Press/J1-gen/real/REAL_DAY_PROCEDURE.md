# 実機日の手順書（下書き）

> ⚠️ **下書き**。タスク7で完成させる。ここには、それまでのタスクで「当日手順書に入れる」と決めた項目を
> 忘れないように集めている。

コマンドはすべてラボ PC のリポジトリ直下（`G1_Hackason/`）で、1行ずつ実行する。

## 事前準備（ラボ PC）

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
  （具体的なコマンドはタスク7で追加する）

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

## 事前準備: PC2 用のオフラインのファイル（実機日の前に、ネットにつながったマシンで）

PC2 はインターネットにつながっていない可能性が高いので、pyrealsense2（Python 3.8 / 3.10 / 3.12 用）と
libusb-1.0 の .deb を事前にダウンロードしておく（約 17 MB）:

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

   PC2 で（conda を使うなら先に `source ~/miniforge3/bin/activate lerobot`）:

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

4. 対象（ボトル）の位置は、タスク6の「定規で測った位置」モードで与える
   （設定のキー名と具体的なコマンドは、タスク6の実装後にここへ追記する）。

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

## タスク7で追加する項目

- 接続確認（RGB・深度・lowstate、`mode_machine` = 5）
- 経路の判定: Dev/Navigation の `mode_check.py` と `armsdk_probe.py` を直接実行する手順
- ティーチング、FK の確認（手先の点 `end_effector` の実測との比較）、較正
- 作業空間の箱（`configs/press.yaml` の `workspace`、仮の値）の調整
