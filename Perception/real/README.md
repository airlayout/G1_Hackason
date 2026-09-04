# Perception / real

実機G1上での実行コード。ここには2種類のスクリプトがある:

- `run_real.py` — `common/`のYOLO検出パイプライン一式(FrameSource -> YoloDetector ->
  ResultWriter)を、ZMQストリーム経由で実機G1に接続して動かすもの
- `probe_zmq_camera.py` — 認識処理を含まない、画像取得だけの動作確認スクリプト

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

### 端末の準備

**操作PCで端末を2つ開く。** コマンドはすべて操作PCのキーボードから打つが、
実行される場所が2つに分かれる。

| | 役割 | 実行される場所 |
|---|---|---|
| **端末A** | `ssh`でG1にログインして使う | **G1本体** |
| **端末B** | 操作PCのまま使う | **操作PC** |

`ssh`でログインした時点から、端末Aで打つコマンドはG1上で動く。
**迷ったらプロンプトを見る**:

| プロンプト | どこ |
|---|---|
| `unitree@` で始まる | G1本体(端末A) |
| それ以外(自分のユーザー名) | 操作PC(端末B) |

手順3でサーバーを起動すると端末Aは占有され、戻ってこなくなる。以降は端末Bで作業する。

実行順:

```
0（安全確保）→ 1（端末B）→ 2・3（端末A）→ 4〜7（端末B）→ 8（端末A→端末B）
```

### 事前(前日までに、実機なしでできること)

**【端末B｜操作PC】**

```bash
cd ~/Robot/G1_Hackason
git pull
./G1_HuggingFace/venv/bin/python Perception/real/probe_zmq_camera.py --help
```

`--help`が正しく出れば、当日オプションを思い出せる。

### 0. 安全確保

ロボットを吊る／支える／座らせる。**この手順を飛ばさないこと**(理由は「安全上の注意」参照)。

### 1. 【端末B｜操作PC】ネットワーク設定と疎通確認

```bash
cd ~/Robot/G1_Hackason
bash Common/network/setup_ethernet_for_g1.sh
python3 Common/network/check_g1_connectivity.py
```

`READY`が出ればOK。

⚠️ **操作PCがWSL2の場合、`setup_ethernet_for_g1.sh`は使えない**(物理NICを持たないため)。
代わりにWindows側でEthernetアダプタに静的IP(`192.168.123.200` /
サブネット`255.255.255.0` / ゲートウェイ空欄)を設定し、`.wslconfig`に
`networkingMode=mirrored`を書いて`wsl --shutdown`する必要がある。
WSL内で`ip -br a`に`192.168.123.x`が見え、G1にpingが通ることを確認してから先へ進む。

### 2. 【端末A｜G1本体】カメラのデバイス番号を確認

```bash
ssh unitree@192.168.123.164     # または ssh g1
ls /dev/video*
```

`run_g1_server.py`の既定は`/dev/video4`。無ければ次の手順で`--camera-device <番号>`を足す。

### 3. 【端末A｜G1本体】配信サーバーを起動

```bash
source ~/miniforge3/bin/activate lerobot
cd ~/lerobot
export CYCLONEDDS_HOME=~/cyclonedds/install
export LD_LIBRARY_PATH=~/cyclonedds/install/lib:$LD_LIBRARY_PATH
python -u src/lerobot/robots/unitree_g1/run_g1_server.py --camera
```

`bridge running`と`Camera server started on port 5555`が出ればOK。
**この端末はここで占有される。以降は触らず、端末Bで作業する。**

### 4. 【端末B｜操作PC】ポートの到達性確認

```bash
python3 Common/network/check_g1_connectivity.py --check-bridge-ports
```

5555が到達可能になっていればOK。

### 5. 【端末B｜操作PC】まず30枚で動作確認

```bash
./G1_HuggingFace/venv/bin/python Perception/real/probe_zmq_camera.py \
  --host 192.168.123.164 --timeout 60
```

`RESULT_OK`を確認。解像度・受信レート・カメラ名をメモする。

### 6. 【端末B｜操作PC】色順の判定

`_local/perception/real/`の`_asis`と`_swapped`を見比べ、どちらが自然な色かを確認する。
実装上はシムと同じくRGB(=`_swapped`が正しい)のはずだが、実物で確認すること。

`run_real.py`(common/camera/zmq_camera.py)側は、シムでの実測(RGB)を前提に
`cv2.cvtColor(..., cv2.COLOR_RGB2BGR)`を内部で適用済み。実機で色順が異なると
判明した場合は、まずここで確認してから`zmq_camera.py`の修正要否を判断すること。
### 7. 【端末B｜操作PC】大量収集(最重要)

```bash
./G1_HuggingFace/venv/bin/python Perception/real/probe_zmq_camera.py \
  --host 192.168.123.164 --frames 500 --save 500
```

いろいろな場所・明るさ・対象を撮ること。

### 8. 撤収

**【端末A｜G1本体】** サーバーを停止してログアウトする:

```bash
# Ctrl-C でサーバーを停止
exit
```

**【端末B｜操作PC】** 必要なら元のネットワーク設定に戻す:

```bash
bash Common/network/setup_ethernet_for_g1.sh --revert
```

撮った画像のバックアップも忘れないこと(「実機の時間で優先すべきこと」参照)。

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
