# Perception / real

実機G1上での実行コード。ここには3種類のスクリプトがある:

| スクリプト | 何をするか | いつ使うか |
|---|---|---|
| `probe_zmq_camera.py` | 認識せず、画像取得だけを確認・計測する | **実機で最初に使う**。接続確認・画像の収集 |
| `run_real_visual.py` | 検出＋**検出枠を描いた画像の保存・表示** | **開発・検証**(目視で確かめる)、リアルタイム確認 |
| `run_real.py` | 検出のみ(console/JSONL/CSV)。軽い | 本番運用(実機のCPU向け) |

依存の重さもこの順で、`probe_zmq_camera.py`はzmq/cv2/numpyだけで動く
(ultralytics/torchを必要としない)。**実機では、まず`probe_zmq_camera.py`で
画像経路だけを確定させてから認識に進むと、失敗の切り分けが楽になる。**

## run_real.py（YOLO検出パイプライン）

### 前提

実機G1側で、カメラ配信付きのサーバを起動しておくこと（`G1_HuggingFace/README.md`の
「実機への接続」を参照）:

```bash
# G1本体側(SSH接続後)。conda環境を有効化しないとpython3.8で起動して失敗する
source ~/miniforge3/bin/activate lerobot
cd ~/lerobot
export CYCLONEDDS_HOME=~/cyclonedds/install
export LD_LIBRARY_PATH=~/cyclonedds/install/lib:$LD_LIBRARY_PATH
python src/lerobot/robots/unitree_g1/run_g1_server.py --camera
```

詳細な手順と、環境が見つからないように見えるときの確認方法は、下の
「当日の手順」の手順2.5・3を参照。

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

## run_real_visual.py（検出結果を画像として保存・表示する版）

`run_real.py`との違いは「検出枠を描いた画像を保存・表示できるかどうか」。数値だけでは
検出が正しいか人間が判断できないため、**目視で確認するために画像を残す、またはその場で
ウィンドウに表示する**。

`sim/run_sim_visual.py`と処理は同一で、参照するconfigが違うだけ（接続先が実機G1の
IPになる）。**片方を直したらもう片方にも反映すること。**

### 実行

```bash
cd Perception/real
# 画像を保存して後から確認する
../../G1_HuggingFace/venv/bin/python run_real_visual.py --max-frames 30
# 実機カメラの映像をリアルタイムで表示する(q または Esc で終了)
../../G1_HuggingFace/venv/bin/python run_real_visual.py --show
```

`--show`は**操作PC側にディスプレイがあり、GUI対応のOpenCVが入っている場合のみ**使える。
起動直後に「ウィンドウを開けませんでした」と出たら、
[../sim/README.md](../sim/README.md)の「--show が使えないとき」を参照。

### 追加のオプション

`run_real.py`のオプション（`--server-address`を含む）に加えて:

| オプション | 既定値 | 説明 |
|---|---|---|
| `--save-images` | `detections_only` | `none` / `all` / `detections_only` |
| `--run-name` | 実行時の日時 | この実行の名前。`visual_dir/<run-name>/`に画像と数値がまとまる |
| `--visual-dir` | `../../_local/perception/visual/real` | 実行フォルダを作る親ディレクトリ |
| `--show` | 無効 | 検出枠付きの映像をウィンドウに表示（`q`/`Esc`で終了） |

⚠️ 実機で撮った画像は**別途バックアップすること**（`_local/`はGit管理外のため。
詳細は「実機の時間で優先すべきこと」を参照）。

評価としての使い方（`--run-name`を分ける理由、`run.csv`の数え方）、終了時に表示される
処理時間の内訳の読み方、実装上の注意（描画時の`copy()`、`bbox`の`int()`変換、
チャンネル順を変換しないこと等）は[../sim/README.md](../sim/README.md)を参照。

## probe_zmq_camera.py（画像取得のみの動作確認） ※実機未検証(2026-09-01時点)

G1の一人称視点(head_camera)をZMQ経由で受け取り、解像度・受信レート・遅延を実測して
PNGに保存する。認識処理は含まない。

`sim/probe_zmq_camera.py`と処理は同一で、**既定値だけが違う**:

| | sim版 | real版(これ) |
|---|---|---|
| `--host` | `localhost` | `192.168.123.164` |
| `--out-dir` | `_local/perception/probe/sim` | `_local/perception/probe/real` |

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
0（安全確保）→ 1（端末B）→ 2・2.5・3（端末A）→ 4〜7（端末B）→ 8（端末A→端末B）
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

**G1の電源を入れた直後は、ネットワークが繋がるまで1〜2分待つ。** 2026-09-09の作業では、
リンクは確立しているのにARPが解決せずSSHもタイムアウトし、数回リトライした後に
繋がった。PC2の起動完了待ちだった可能性が高い。すぐ繋がらなくても設定を疑う前に
少し待ってから再試行すること。

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

### 2.5 【端末A｜G1本体】作業前チェック: 環境が実在するか

**`conda`や`python3.12`が見つからないのは正常。環境が無いと判断しないこと。**

G1側のMiniforgeは`bash Miniforge3-Linux-aarch64.sh -b -p ~/miniforge3`
(`-b`=バッチモード)でインストールしており、この方式では`.bashrc`にcondaの設定が
書き込まれない。そのためログイン直後のシェルでは次のようになるが、**環境が
インストールされていても同じ結果になる**:

| コマンド | 結果 | 意味 |
|---|---|---|
| `which conda` | 見つからない | condaがPATHに無いだけ |
| `which python3.12` | 見つからない | Python 3.12はconda環境の中にしか無い |
| `python3 --version` | 3.8.10 | システム標準のPython |

環境の有無は、**ディレクトリの実在で**確認する:

```bash
ls -d ~/miniforge3 ~/lerobot ~/cyclonedds ~/unitree_sdk2_python
ls ~/miniforge3/envs/          # lerobot があればOK
```

- **すべて存在する** → 環境は無事。手順3の`source ~/miniforge3/bin/activate lerobot`で
  有効化してから使う。有効化後に`python --version`が3.12になることを確認する
- **存在しない** → 本当に環境が無い。別個体に接続していないか、PC2がリセットされて
  いないかをチームに確認したうえで、`SETUP.md`の3章の手順で再構築する

`run_g1_server.py`のプロセスが動いていないのも正常(`nohup`で起動していても、G1を
再起動すれば消える)。毎回手順3で起動する。

(2026-09-09の作業では、上表の3つの結果から「環境が存在しない」と判断して作業を
停止した。ディレクトリの確認は行っておらず、実際に環境が失われていたかは未確認。
`Perception/FAILURES.md`の同日のエントリも参照)

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

`_local/perception/probe/real/`の`_asis`と`_swapped`を見比べ、どちらが自然な色かを確認する。
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
