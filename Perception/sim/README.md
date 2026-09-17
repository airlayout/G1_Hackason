# Perception / sim

MuJoCoシミュレーション上での検証コード。ここには3種類のスクリプトがある:

| スクリプト | 何をするか | いつ使うか |
|---|---|---|
| `probe_zmq_camera.py` | 認識せず、画像取得だけを確認・計測する | **接続確認**、画像の収集 |
| `run_sim_visual.py` | 検出＋**検出枠を描いた画像の保存・表示** | **開発・検証**(目視で確かめる)、リアルタイム確認 |
| `run_sim.py` | 検出のみ(console/JSONL/CSV)。軽い | 本番運用(実機のCPU向け) |

依存の重さもこの順で、`probe_zmq_camera.py`はzmq/cv2/numpyだけで動く
(ultralytics/torchを必要としない)。**画像が来ないのか、認識が動かないのかを
切り分けたいときは、まず`probe_zmq_camera.py`で画像経路だけを確定させるとよい。**

## run_sim.py（YOLO検出パイプライン）

### 前提

MuJoCoシム側で、`head_camera`をZMQ配信する状態になっていること。
`G1_HuggingFace/README.md`の`lerobot-teleoperate`の例のように、
`--robot.cameras='{"...": {"type": "zmq", "server_address": "localhost", "port": 5555, "camera_name": "head_camera", ...}}'`
を付けて起動すればよい。

### 実行

```bash
source ../../G1_HuggingFace/venv/bin/activate  # 未activateの場合
pip install -r ../requirements.txt             # 未インストールの場合

python run_sim.py
# 別のconfigを使う場合: python run_sim.py --config configs/config.yaml
# フレーム数を絞って動作確認する場合: python run_sim.py --max-frames 30
```

検出結果は`outputs/`にJSON Lines(`run.jsonl`)・CSV(`run.csv`)として出力される
（`.gitignore`で追跡対象外）。

`common/camera/zmq_camera.py`の`ZmqFrameSource`は、下記「メッセージ形式」「実測値」の
チャンネル順(RGB)を踏まえてBGRに変換してから返す（詳細はコード内コメントを参照）。

## run_sim_visual.py（検出結果を画像として保存・表示する版）

`run_sim.py`との違いは「検出枠を描いた画像を保存・表示できるかどうか」。
`bbox=(120, 45, 260, 430)`という数値だけでは検出が正しいか人間が判断できないため、
**目視で確認するために画像を残す、またはその場でウィンドウに表示する**。

`common/`の部品はそのまま再利用し、繋ぐループだけを独自に持つ。`run_sim.py`・
`common/pipeline.py`は変更していないので、実機向けの軽い経路には影響しない。

使い方は目的によって2通りある:

| 目的 | 入力 | 主なオプション |
|---|---|---|
| **評価**（正解が分かっている動画で検出性能を測る） | `--source video` | `--run-name` / `--save-images` |
| **リアルタイム確認**（カメラの前で枠が追従するか見る） | `--source webcam` / `zmq` | `--show` |

動画での評価は、リアルタイムで動かす結果を信用するための土台になる（正解が分かっていて、
同じ条件で何度でも繰り返せるため）。順番は評価が先。

### 実行

```bash
cd Perception/sim
../../G1_HuggingFace/venv/bin/python run_sim_visual.py --max-frames 30
```

シムの映像には人が写らないため、初回は`--save-images all`を付けて
「検出0でも画像が保存されること」を確認するとよい。

### 評価として使う（動画・条件ごとに出力を分ける）

```bash
../../G1_HuggingFace/venv/bin/python run_sim_visual.py \
  --source video --video-path ../../_local/perception/videos_640/dist_5m.mp4 \
  --save-images all --run-name dist_5m
```

**1回の実行 = 1フォルダ**。`--run-name`で付けた名前のフォルダに、画像も数値もまとまる:

```
_local/perception/visual/sim/dist_5m/
├── images/          検出枠を描いた画像
├── run.csv
└── run.jsonl
```

`--run-name`を省略すると日時(例: `2026-09-17_1830`)が名前になるので、
**何も指定しなくても実行ごとに別フォルダ**になる。

⚠️ **`run.jsonl` / `run.csv`は追記方式**。同じ`--run-name`で実行し直すと前回の結果に
積み重なる(その場合は起動時に警告が出る)。条件ごとに比較するなら名前を変えること。

`run.csv`は**1検出=1行**で、検出0件のフレームも`class_name`が空の行として出力される。
検出率を出すときは行数ではなく、**`frame_index`の重複を除いた数**で数えること
（1フレームに2人写ると2行になるため）。

### リアルタイムで表示する

```bash
../../G1_HuggingFace/venv/bin/python run_sim_visual.py --source webcam --show
```

検出枠を描いた映像がウィンドウに表示され、左上に`fps`・推論時間・検出数が出る
（この文字は表示用で、保存する画像には描かない）。**`q`または`Esc`で終了**
（ウィンドウを閉じても終了する）。`--max-frames`を付けなければ終了操作まで動き続ける。

⚠️ **Webカメラは遅延が溜まることがある。** `WebcamSource`はOpenCVの`VideoCapture`を
そのまま使っており、カメラ側がフレームを溜め込む。処理がカメラのフレームレートより遅いと
古いフレームから順に処理され、**表示がどんどん遅れていく**。ZMQ(`CONFLATE`で最新だけを取る)
では起きない。GPUのあるPCでは処理が速いので問題になりにくい。

### --show が使えないとき

起動直後に「ウィンドウを開けませんでした」と出て終了する場合、ほとんどは
**表示機能を持たない`opencv-python-headless`が読み込まれている**のが原因。

```bash
../../G1_HuggingFace/venv/bin/python -c "import cv2; print(cv2.__version__); print(cv2.getBuildInformation())" | grep -A3 "GUI:"
../../G1_HuggingFace/venv/bin/pip list | grep -i opencv
```

`GUI:`の欄が`NONE`で、`opencv-python`と`opencv-python-headless`が両方入っていれば該当する。
両者は同じ`cv2`という名前で入るため、後から入った方が優先される（2026-09-15、WSLの
`G1_HuggingFace/venv`でこの状態を確認。lerobot等の依存でheadless版が入ったと思われる）。

対処（**表示用の`opencv-python`だけを残す。バージョンは今読まれているheadless版に揃える**）:

```bash
# いま読まれている版を確認(例: 4.13.0 なら opencv-python-headless 4.13.0.92)
../../G1_HuggingFace/venv/bin/pip show opencv-python-headless | grep Version

../../G1_HuggingFace/venv/bin/pip uninstall -y opencv-python-headless opencv-python
../../G1_HuggingFace/venv/bin/pip install opencv-python==4.13.0.92   # 上で確認した版に合わせる

# GUI 欄が NONE 以外(GTK など)になっていれば成功
../../G1_HuggingFace/venv/bin/python -c "import cv2; print(cv2.__version__); print(cv2.getBuildInformation())" | grep -A3 "GUI:"
```

**バージョンを揃える理由**: 何も指定せず`pip install opencv-python`すると最新版(5.x)が入り、
シム(lerobotのMuJoCo環境)が使っていたOpenCVの版が変わってしまう。表示機能の有無だけを
変えたいので、版は据え置く。

⚠️ `opencv-python-headless`は`lerobot`の依存に指定されているため、入れ替え後に`pip check`で
警告が出る。`cv2`というモジュール自体は同じなので動作には支障ないが、lerobotを入れ直した
拍子にheadless版が戻ってくることがある。**再発したら同じ手順で戻す**。
入れ替えた後は、シム(`release_band_and_walk_forward.py`)と`probe_zmq_camera.py`が
これまでどおり動くことを一度確認しておくと安心。

元に戻す場合:

```bash
../../G1_HuggingFace/venv/bin/pip uninstall -y opencv-python
# 2つは同じ cv2 フォルダに入り、後から入れた方が優先される。元の状態(headless版が読まれる)に
# 戻すため、必ずこの順番で別々に実行する。
../../G1_HuggingFace/venv/bin/pip install opencv-python==5.0.0.93
../../G1_HuggingFace/venv/bin/pip install --force-reinstall --no-deps opencv-python-headless==4.13.0.92
```

表示機能の無い環境（SSH越し、ディスプレイの無いサーバー等）では`--show`は使えない。
その場合は`--save-images`で保存して後から確認する。

### 処理時間の内訳

終了時に、1フレームあたりの平均時間が段階別に表示される:

```
1フレームあたりの平均時間:
  取得   :     2.7 ms  (カメラ/動画から1枚取り出す。待ち時間を含む)
  推論   :   742.2 ms
  後処理 :     2.6 ms  (結果の書き出し・描画・保存・表示)
  合計   :   747.5 ms  → 実効 1.3 fps
```

（上の数値はWSL2・GPU無しでの例。推論が支配的なのが分かる）

リアルタイムで意味があるのは推論時間だけでなく**合計**。どこが遅いかはこの内訳で判断する。
ZMQやWebカメラでは「取得」に**次の1枚が届くまでの待ち時間**が含まれるため、
処理が十分速ければ取得時間がカメラのフレーム間隔(30fpsなら約33ms)に近づく。

### 追加のオプション

`run_sim.py`のオプションに加えて:

| オプション | 既定値 | 説明 |
|---|---|---|
| `--save-images` | `detections_only` | `none` / `all` / `detections_only` |
| `--run-name` | 実行時の日時 | この実行の名前。`visual_dir/<run-name>/`に画像と数値がまとまる |
| `--visual-dir` | `../../_local/perception/visual/sim` | 実行フォルダを作る親ディレクトリ |
| `--show` | 無効 | 検出枠付きの映像をウィンドウに表示（`q`/`Esc`で終了） |

既定の`detections_only`は「**検出が1つ以上あったフレームだけ保存**」。警備用途では
何も起きていない時間が大半なので、この方が現実的。設定は`configs/config.yaml`の
`output.save_images` / `output.visual_dir`にもある（`run_sim.py`はこの項目を読まない）。

保存先を`_local/`配下にしているのは、画像が再取得できるローカルデータであり、
上流の`.gitignore`が`_local/`を除外済みのため。データの置き場全体は
[../README.md](../README.md)の「ローカルデータの置き場」を参照。

### 実装上の注意

- 描画時は`frame.copy()`してから描く（`cv2.rectangle`は配列を直接書き換えるため）
- `bbox`は`float`なので`int()`に変換してから`cv2`へ渡す
- **チャンネル順の変換はしない**。`FrameSource`の契約がBGRで、`cv2.imwrite`も
  BGR前提なので、そのまま保存すれば正しい色になる
  （`probe_zmq_camera.py`は生データを扱うので変換が必要。逆なので混同しないこと）
- ラベルは英数字のみ（`cv2.putText`は日本語を描けない）

## probe_zmq_camera.py（画像取得のみの動作確認）

G1の一人称視点(head_camera)をZMQ経由で受け取り、解像度・受信レート・遅延を実測して
PNGに保存する。認識処理は含まない。「画像が取れる」ことを確定させるためのスクリプト。

処理そのものは実機と共通(違いは`--host`だけ)。実機用は既定値を変えたコピーを
[../real/probe_zmq_camera.py](../real/probe_zmq_camera.py)に置いている。

### 実行手順(シミュレーション)

**端末を2つ使う。** 配信する側(シム)と受け取る側(このスクリプト)を同時に動かす必要が
あるため。

端末1 — シミュレーションを起動する:

```bash
cd ~/Robot/G1_Hackason
export CYCLONEDDS_HOME=$(pwd)/G1_HuggingFace/cyclonedds/install
./G1_HuggingFace/venv/bin/python SimpleWalk/sim/release_band_and_walk_forward.py
```

起動に約1分かかる。`Camera images publishing on tcp://localhost:5555`が
**配信開始の合図**。

端末2 — 画像を受け取る:

```bash
cd ~/Robot/G1_Hackason
./G1_HuggingFace/venv/bin/python Perception/sim/probe_zmq_camera.py
```

端末2では`CYCLONEDDS_HOME`は不要(DDSを使わないため)。

`--timeout 120`を付ければ端末2を先に起動して待たせておけるので、
端末1の起動完了を見計らう必要がなくなる。

### 実機で使う場合

同じスクリプトを`--host`だけ変えて使う。実機側の手順・注意点・仕様の違いは
[../real/README.md](../real/README.md)を参照。

## 出力

標準出力に1枚ごとの受信ログと、最後に集計結果(解像度・データ型・受信枚数・
平均受信間隔・平均遅延)を表示する。成功すると`RESULT_OK`で終わる。

画像は`--out-dir`(既定`_local/perception/probe/sim/`)に保存される。1枚につき2種類:

| ファイル名 | 内容 |
|---|---|
| `frame_001_asis.png` | 受け取った配列をそのまま保存したもの |
| `frame_001_swapped.png` | 赤と青のチャンネルを入れ替えて保存したもの |

**`_swapped`のほうが正しい色**(2026-08-30 実測で確認)。詳細は下記「チャンネル順」を参照。

出力先を`_local/`配下にしているのは、上流の`.gitignore`が`_local/`を
「このPCだけに置くローカル専用データ(共有しない・再取得できるもの)」として
除外済みのため。新たな除外設定は不要。

## オプション

| オプション | 既定値 | 説明 |
|---|---|---|
| `--host` | `localhost` | 配信元。**実機に切り替える唯一の窓口** |
| `--port` | `5555` | ZMQのポート |
| `--frames` | `30` | 受信するフレーム数 |
| `--save` | `3` | PNGとして保存する枚数 |
| `--out-dir` | `_local/perception/probe/sim` | 保存先ディレクトリ |
| `--timeout` | `20.0` | 1枚あたりの受信待ち時間(秒) |
| `--camera` | (自動) | カメラ名。省略時は最初に見つかったものを使う |

## 実測値(2026-08-30, WSL2 / GPU無し)

| 項目 | 値 |
|---|---|
| カメラ名 | `head_camera` |
| 解像度 / データ型 | 640×480×3 / `uint8`(0〜255) |
| チャンネル順 | **RGB** |
| 受信レート | 約1.2〜1.3 fps(シム側の配信は約2.5Hz) |
| 遅延 | 約3〜5ms |

**チャンネル順がRGBである点は重要。** cv2の関数はBGRを前提とするため、`cv2.imwrite`や
`cv2.imshow`に渡す前に`cv2.cvtColor(img, cv2.COLOR_RGB2BGR)`が必要になる。一方、
一般的な認識モデルはRGB入力を期待するのでそのまま渡せる。

`common/camera/zmq_camera.py`の`ZmqFrameSource`は、`FrameSource`インターフェース全体の
契約(BGRを返す。`webcam.py`/`video_file.py`と揃える)に合わせるため、この変換を内部で
行ってから返す。

受信レートが低いのはGPU非搭載の環境でMuJoCoの描画がCPU処理になっているため。
シミュレータ全体が実時間の約1/12の速さで動いている。実機のカメラは30fps配信なので、
この値は実機の性能とは無関係。

遅延が数msなのは配信側と受信側が同一マシンにあるため。実機では両者の時計がずれるので、
この計算方法では正しい遅延が測れない。

## つまずいた点

**シムがsegfaultで落ちた後、再実行すると必ず`Address already in use (5555)`で失敗する。**
画像配信のサブプロセス(multiprocessingのspawn子)が親の異常終了後も生き残り、
ポート5555と共有メモリを保持し続けるため。次の手順で片付けてから再実行する:

```bash
ss -lntp | grep 5555        # 誰が掴んでいるか確認
pkill -f spawn_main         # 残った配信サブプロセスを止める
rm -f /dev/shm/psm_*        # 漏れた共有メモリを削除
```

「昨日は動いたのに今日は動かない」の正体がこれだった。スクリプトの変更を疑う前に、
前回の実行が残した状態を疑うこと。

## メッセージ形式

HFキャッシュ内の`sim/sensor_utils.py`(`SensorServer` / `ImageUtils`)より。
ZMQのPUB/SUBで、中身はJSON文字列:

```
{"timestamps": {"head_camera": <UNIX時刻>},
 "images":     {"head_camera": "<base64のJPEG>"},
 "head_camera": "<同じもの。トップレベルにも入る>"}
```

購読側は`CONFLATE=True`(最新の1通だけ保持)で受けるので、処理が遅い場合、
**古いフレームは警告なしに捨てられる**。

実機側は別実装だが、この形式は共通なのでスクリプトはそのまま動く。
細かい違いは[../real/README.md](../real/README.md)を参照。
