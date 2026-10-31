# Perception / real

実機G1上での実行コード。ここには2種類のスクリプトがある:

- `run_real.py` — `common/`のYOLO検出パイプライン一式(FrameSource -> YoloDetector ->
  ResultWriter)を、ZMQストリーム経由で実機G1に接続して動かすもの
- `check_yolo.py` — YOLO(人の検出)がそのマシンで動くか・速さを確かめる。G1のPC2の上で、torchを使わずONNXで動かす
  (`export_onnx.py` で書き出し、PC2用の配布物は `fetch_pc2_wheels.sh`)
- `probe_zmq_camera.py` — 認識処理を含まない、画像取得だけの動作確認スクリプト
- `person_alert.py` / `person_alert_receiver.py` — 人を検出したら写真を G1 に残し、PC へも送る（下記）

## person_alert.py（人を検出したら写真を残して PC へ送る） ※2026-10-31 実機で確認

G1 の PC2 で動かす。人が 3 フレーム続けて写ったら「出来事」にして、枠を描いた写真と記録（`events.jsonl`）を
`~/perception/_local/perception/person_alert/` に保存し、ZMQ（5560 番）で配信する。そのあと 10 秒は出さない（人がいる間は 10 秒ごと）。
写真は 1000 枚まで（超えたら古いものから消す。記録は消さない）。**PC がつないでいなくても G1 には必ず残る**
（つないでいなかった間の写真は PC には届かないので、要るなら `scp` で持ってくる）。準備（エンジン・`~/perception/` への配置）は
`Perception/docs/report/2026-10-31_g1_webcam_yolo.md` を参照。

```bash
# G1（PC2）: GPU 版。前方のカメラ（Innomaker）を Mapping の配信（5555 番）から受け取る。Mapping を止めずに使える（5 枚/秒）
cd ~/perception && /usr/bin/python3 real/person_alert.py --zmq localhost:5555 --zmq-bgr --rotate 180 --backend tensorrt --model yolo26n.engine
```

**PC2 の Web カメラは 2 つある**（2026-10-31）: 前方の **Innomaker U20CAM** と、腕の**リストカメラ**（SunplusIT の Full HD webcam）。
`/dev/videoN` の番号は再起動で入れ替わる（再起動の前は Innomaker が video6、後はリストカメラが video6 になり、
`--webcam 6` のままリストカメラの映像を配信していた）。直接開くときは `--webcam /dev/v4l/by-id/...` で指定する（`ls /dev/v4l/by-id/`）。

| 前方のカメラの取り方 | オプション | 速さ |
|---|---|---|
| Mapping の `~/mapping_tools/innomaker_camera_server.py` の配信から（BGR・反時計回りに回し済み） | `--zmq localhost:5555 --zmq-bgr --rotate 180` | 5 枚/秒（Mapping の設定） |
| 直接（Mapping の配信が止まっているときだけ） | `--webcam /dev/v4l/by-id/usb-Innomaker_Innomaker-U20CAM-1080p-S1_SN0001-video-index0 --rotate cw` | 約 30 枚/秒 |
```bash
# PC: 受け取って _local/perception/person_alert_received/ に保存し、表示する（PC から G1 へつなぐので WSL でも設定は要らない）
G1_HuggingFace/venv/bin/python Perception/real/person_alert_receiver.py --host 192.168.0.82
```

**認識した映像をブラウザで見る**: G1 側に `--stream 8080` を足し、ブラウザ（Windows でよい）で `http://192.168.0.82:8080/` を開く。
枠・人数・fps を描いた映像が、見ている人がいるときだけ 10 枚/秒（`--stream-fps`）で届く。VLC などでは `/stream`、最新の 1 枚は `/snapshot.jpg`。
同じネットワークにいる人なら誰でも見られる（人が写るので注意）。

- 2026-10-31 の確認（Wi-Fi）: 10.0 枚/秒、1 枚約 52 KB、約 0.5 MB/s、届く間隔の最大 155 ms。2 人同時に見ても同じ。
  配信しながらでも検出は約 28 fps のまま（PC2 の CPU はこのプロセスで約 50%）
- G1 は 5 秒ごとに生きている合図を送り、PC は 15 秒届かなければ警告する
- 2026-10-31 の確認: GPU 版で約 28 fps（カメラの 30 fps でほぼ頭打ち）。60 秒で出来事 4 件、PC に 4 件とも届いた。PC を止めた状態でも
  G1 に保存され、`--max-photos 3` で古い写真から消えた
- 写真の時刻は G1 の時計（その日はラボPC より約 1 時間遅れていた）
- カメラは同時に 1 つのプロセスしか開けない。`check_yolo.py` などと同時には動かせない

## run_real.py（YOLO検出パイプライン）

### 前提

実機G1側で、カメラ配信付きのサーバを起動しておくこと（`G1_HuggingFace/README.md`の
「実機への接続」を参照）:

```bash
# G1本体側(SSH接続後)
cd ~/lerobot
python src/lerobot/robots/unitree_g1/run_g1_server.py --camera
```

操作PC側は`configs/config.yaml`の`source.zmq.server_address`をG1のIP
（既定`192.168.123.164`）に合わせる。ネットワーク疎通確認は`Common/network/`を参照。

### 実行

```bash
source ../../G1_HuggingFace/venv/bin/activate  # 未activateの場合
pip install -r ../requirements.txt             # 未インストールの場合

python run_real.py --server-address 192.168.123.164
# フレーム数を絞って動作確認する場合: python run_real.py --max-frames 30
```

検出結果は`outputs/`にJSON Lines(`run.jsonl`)・CSV(`run.csv`)として出力される
（`.gitignore`で追跡対象外）。

## G1 の PC2 の上で YOLO を動かす（check_yolo.py） ※2026-10-22 に準備、実機未確認

人の認識を G1 の中（PC2: Jetson Orin NX、Ubuntu 20.04、aarch64、glibc 2.31）で動かすための確認。
PC2 には torch を入れず、ラボPCで書き出した ONNX を **onnxruntime（CPU）** で動かす（`common/detector/onnx_yolo.py`。
同じ ONNX を ultralytics で動かした結果と一致することを確かめてある）。カメラはまず **G1 の頭カメラ**（PC2 で動いている配信の
5555 番を、PC2 の中から localhost で受け取る）。頭カメラは約 45° 下を向いているので、**人はかがんで、G1 の前 0.5〜1 m で写り込む**。
あとで Web カメラ（前向き）を PC2 の USB に挿したら `--webcam <番号>` で同じことを確かめる。

> ⚠️ PC2 の OS は書き換えない（`sudo apt` をしない）。入れるのは `~/perception/` の下の venv だけ。conda の `lerobot` 環境には足さない。
> ⚠️ `run_g1_server.py` は起動時に歩行の制御を解除する（立っていると倒れるおそれ）。カメラの配信は ButtonPress の
> `rgbd_server.py`（5555 番に同じ形式で出す。`Button_Press/J1-gen/real/depth_server/README.md`）を使う。

### 事前（ラボPC。実機なしでできる）

```bash
G1_HuggingFace/venv/bin/python Perception/real/export_onnx.py
```

→ `_local/perception/models/yolo26n.onnx`（約 10 MB、end-to-end）。サンプル画像で ultralytics と一致するかも確かめる。
PC2 がインターネットにつながらない場合（`Button_Press/J1-gen/real/pc2_survey.sh` の結果で分かる）は、配布物も集める:

```bash
bash Perception/real/fetch_pc2_wheels.sh
```

→ `_local/perception/pc2_wheels/`（約 73 MB。PC2 の glibc 2.31 / GLIBCXX 3.4.28 で動くかも確かめる。2026-10-22 はすべて OK）

### 当日 1. PC2 へ写す（ラボPC、有線）

```bash
ssh g1 'mkdir -p ~/perception/real'
```
```bash
scp -r Perception/common Perception/requirements_onboard.txt _local/perception/models/yolo26n.onnx g1:~/perception/
```
```bash
scp Perception/real/check_yolo.py g1:~/perception/real/
```

インターネットにつながらないときは、配布物も: `scp -r _local/perception/pc2_wheels g1:~/perception/`
（サンプル画像も写しておくと、まず 1 枚で確かめられる: `G1_HuggingFace/venv/lib/python3.12/site-packages/ultralytics/assets/bus.jpg`）

### 当日 2. PC2 で venv を作る（PC2。conda の環境から抜けた状態で）

Python 3.12 は conda の `lerobot` 環境のものを借りて venv を作る（`lerobot` 自体には何も足さない）。場所は `pc2_survey.sh` の
「conda の場所」で分かる（例: `~/miniconda3`）:

```bash
~/miniconda3/envs/lerobot/bin/python -m venv ~/perception/venv
```
```bash
~/perception/venv/bin/pip install -r ~/perception/requirements_onboard.txt
```

インターネットにつながらないときは `~/perception/venv/bin/pip install --no-index --find-links ~/perception/pc2_wheels -r ~/perception/requirements_onboard.txt`。

### 当日 3. 動かす（PC2）

画像 1 枚で動くか → 頭カメラの配信で連続、の順:

```bash
cd ~/perception && venv/bin/python real/check_yolo.py --image bus.jpg --model yolo26n.onnx
```
```bash
cd ~/perception && venv/bin/python real/check_yolo.py --zmq localhost:5555 --model yolo26n.onnx --frames 100 --threads 4
```

- 配信（`rgbd_server.py --source videohub`）を動かしたままで測る（PC2 の CPU を分け合うので）。人は G1 の前 0.5〜1 m でかがむ
- `--threads` を 2・4・6 と変えて、1 枚の時間と CPU 使用率（1 コア = 100%）を比べる。ラボPC では 4 スレッドで約 19 ms・325%
- 結果: `~/perception/_local/perception/check_yolo/<日時>/`（`summary.json` と、人が写った画像）。ラボPC に持ち帰る:
  `scp -r g1:~/perception/_local/perception/check_yolo _local/perception/pc2_check_yolo`
- 比べるため、同じ配信をラボPC から受け取っても測る: `G1_HuggingFace/venv/bin/python Perception/real/check_yolo.py --zmq 192.168.123.164:5555 --model _local/perception/models/yolo26n.onnx --frames 100`

### 記録すること

1 枚の時間（平均・90%）、毎秒の枚数、CPU 使用率（スレッド数ごと）、かがんだ人が `person` と出たか（距離と姿勢）、配信サーバの
CPU 使用率が変わったか（`top`）。うまくいかなければ `FAILURES.md` に書く。

## probe_zmq_camera.py（画像取得のみの動作確認） ※実機未検証(2026-09-01時点)

G1の一人称視点(head_camera)をZMQ経由で受け取り、解像度・受信レート・遅延を実測して
PNGに保存する。認識処理は含まない。

`sim/probe_zmq_camera.py`と処理は同一で、**既定値だけが違う**:

| | sim版 | real版(これ) |
|---|---|---|
| `--host` | `localhost` | `192.168.123.164` |
| `--out-dir` | `_local/perception/sim` | `_local/perception/real` |

配信側の実装はシムと実機で異なるが、メッセージ形式は共通なので処理は変える必要がない
(詳細は後述)。**片方のロジックを直したらもう片方にも反映すること。**

## 当日の手順(コマンド早見表)

### 事前(前日までに、実機なしでできること)

```bash
cd ~/Robot/G1_Hackason
git pull
./G1_HuggingFace/venv/bin/python Perception/real/probe_zmq_camera.py --help
```

`--help`が正しく出れば、当日オプションを思い出せる。

### 0. 安全確保

ロボットを吊る／支える／座らせる。**この手順を飛ばさないこと**(理由は「安全上の注意」参照)。

### 1. 操作PC — ネットワーク設定と疎通確認

```bash
cd ~/Robot/G1_Hackason
bash Common/network/setup_ethernet_for_g1.sh
python3 Common/network/check_g1_connectivity.py
```

`READY`が出ればOK。

### 2. G1本体 — カメラのデバイス番号を確認

```bash
ssh unitree@192.168.123.164     # または ssh g1
ls /dev/video*
```

`run_g1_server.py`の既定は`/dev/video4`。無ければ次の手順で`--camera-device <番号>`を足す。

### 3. G1本体 — 配信サーバーを起動

```bash
source ~/miniforge3/bin/activate lerobot
cd ~/lerobot
export CYCLONEDDS_HOME=~/cyclonedds/install
export LD_LIBRARY_PATH=~/cyclonedds/install/lib:$LD_LIBRARY_PATH
python -u src/lerobot/robots/unitree_g1/run_g1_server.py --camera
```

`bridge running`と`Camera server started on port 5555`が出ればOK。
この端末は占有されるので、以降は操作PC側の別端末で作業する。

### 4. 操作PC — ポートの到達性確認

```bash
python3 Common/network/check_g1_connectivity.py --check-bridge-ports
```

5555が到達可能になっていればOK。

### 5. 操作PC — まず30枚で動作確認

```bash
./G1_HuggingFace/venv/bin/python Perception/real/probe_zmq_camera.py \
  --host 192.168.123.164 --timeout 60
```

`RESULT_OK`を確認。解像度・受信レート・カメラ名をメモする。

### 6. 色順の判定

`_local/perception/real/`の`_asis`と`_swapped`を見比べ、どちらが自然な色かを確認する。
実装上はシムと同じくRGB(=`_swapped`が正しい)のはずだが、実物で確認すること。

`run_real.py`(common/camera/zmq_camera.py)側は、シムでの実測(RGB)を前提に
`cv2.cvtColor(..., cv2.COLOR_RGB2BGR)`を内部で適用済み。実機で色順が異なると
判明した場合は、まずここで確認してから`zmq_camera.py`の修正要否を判断すること。

### 7. 大量収集(最重要)

```bash
./G1_HuggingFace/venv/bin/python Perception/real/probe_zmq_camera.py \
  --host 192.168.123.164 --frames 500 --save 500
```

いろいろな場所・明るさ・対象を撮ること。

### 8. 撤収

```bash
# G1側の端末で Ctrl-C（サーバー停止）
bash Common/network/setup_ethernet_for_g1.sh --revert   # 必要なら元のネットワークに戻す
```

## 当日メモすべきこと

| 項目 | 値 |
|---|---|
| カメラ名の一覧(`head_camera`以外があるか) | |
| 解像度 / データ型 | |
| チャンネル順(`_asis` / `_swapped` のどちらが正しいか) | |
| 受信レート(実測 fps) | |
| 空メッセージの警告は出たか | |
| カメラのデバイス番号(`/dev/videoN`) | |

**遅延の値は無視してよい。** タイムスタンプはG1本体の時計で打たれ、操作PCの時計とは
ずれているため、マイナスの値が出ることもある。故障ではない。代わりに受信間隔を見る。

## 安全上の注意

`run_g1_server.py`は起動時に`MotionSwitcherClient.ReleaseMode()`を実行し、
オンボードの高レベル制御(sport_mode)を解除する。**自立している状態で起動すると
脱力して倒れる恐れがある。**

`SimpleWalk/FAILURES.md`に記録された「disconnect()で脱力し転倒」と同じ構図。
制御状態が変わる瞬間はすべて転倒リスクがある。

画像取得だけならロボットを歩かせる必要は無いので、静止状態で行うこと。

## 前提

- `Common/network/`の手順でEthernet設定と疎通確認が済んでいること
- G1本体側にPython 3.12のconda環境(`lerobot`)が構築済みであること(`SETUP.md`の3章)

## シミュレーションとの違い

配信側の実装が異なる(シム: HFキャッシュ内の`SensorServer` / 実機: lerobotの
`lerobot/cameras/zmq/image_server.py`の`ImageServer`)。`timestamps`と`images`を持つ
JSONという形式は共通だが、以下の違いがある(コードを読んで確認、2026-08-30):

| | シミュレーション | 実機 |
|---|---|---|
| 画像の出どころ | MuJoCoの描画 | `/dev/video4`の**単眼カラーカメラ** |
| **深度画像** | 無し | **無し**(この経路では配信されない) |
| トップレベルのカメラ名キー | 有り | **無し**(`images`の中だけ) |
| 空メッセージ | 来ない | **来る**(新しいフレームが無いとき`images`が空で送られる) |
| 配信レート | 約2.5Hz(GPU無し環境) | 30fps(`--camera-fps`の既定値) |

**深度が無いということは、画像だけでは対象までの距離が分からない。** 距離が必要な場合は
別の手段(対象の実サイズを既知とする、複数視点を使う等)を検討すること。

**空メッセージは実機では実際に飛んでくる。** `probe_zmq_camera.py`の
「`images`が空なら警告して次へ」の処理と、`common/camera/zmq_camera.py`の
`ZmqFrameSource.read()`が`images`が空の場合に`None`を返す処理は、どちらも
実機で実際に必要になる。

## 実機の時間で優先すべきこと

**実機で撮った画像を大量に持ち帰ること。**

実データがあれば、実機が無い期間も本物の映像で認識処理を開発・検証できる。
シミュレーションのチェッカー模様の床と実環境はまったく違うため、認識が実環境で
通用するかは実データでしか分からない。

⚠️ **撮った画像は必ず別途バックアップすること。** 保存先の`_local/`は
「再取得できるもの」を置く前提でGitの管理外にしてある。しかし実機の画像は
実機に触れる機会が限られる以上、**失うと取り返しがつかない**。
Gitに入れるとリポジトリが恒久的に重くなるため、クラウドストレージや外付けドライブなど
別の手段で退避しておく。

実機に触れる時間は貴重なので、**その場でしかできないこと(接続確認・データ収集)に集中し、
後でできること(認識ロジックの開発)は持ち帰る**。
