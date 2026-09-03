# Perception / real

実機G1上での実行コードをここに置く。

## probe_zmq_camera.py ※実機未検証(2026-09-01時点)

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
「`images`が空なら警告して次へ」の処理は、実機で実際に必要になる。

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
