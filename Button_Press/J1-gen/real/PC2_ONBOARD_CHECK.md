# PC2 で動かす準備の確認（次の実機日）

目的: (1) 押す処理（ボトル・ボタン）を G1 の PC2 の中で動かし、最終的にはノート PC（WSL2）から無線で開始・停止・状態だけを
指示する。(2) Perception の人の認識のために、まず YOLO が PC2 の上で動くかを確かめる。
この日は**見るだけの調査**と、**YOLO が動くかの確認**まで。押す処理を PC2 で動かすのは、調査の結果で準備してから。

> ⚠️ PC2 の OS は書き換えない（`sudo apt`・systemd の設定はしない）。入れるものは、すべてユーザーの場所（`~/` の下の venv）に。
> conda の `lerobot` 環境はチームのものなので、パッケージを足さない（読むだけ）。

## 分かっていること（2026-10-22 時点）

| 項目 | 値 | 出典 |
|---|---|---|
| PC2 | Jetson Orin NX、Ubuntu 20.04.6、aarch64、glibc 2.31、システムの Python 3.8 | 10/14 の記録、`depth_server/README.md` |
| conda の `lerobot` 環境 | Python 3.12（中身は未確認） | `depth_server/README.md` |
| 押す処理のライブラリ（aarch64・Python 3.12 用） | numpy・opencv（headless）・pyzmq・MuJoCo 3.3.7・Pinocchio 4.1.0（関連 14 個）・onnxruntime は、要求する glibc が 2.28 以下・GLIBCXX が 3.4.22 以下で **PC2 で動く見込み**（ラボ PC で配布物の中を調べた） | 2026-10-22 |
| cyclonedds 0.10.2 | **aarch64・Python 3.12 用の配布物が無い**。PC2 の `lerobot` 環境にあるものを使うか、PC2 で組み立てる → **10/31: `lerobot` に入っていた**（下の「2026-10-31 の調査の結果」） | 2026-10-22 |
| torch | Jetson の GPU 用は JetPack ごとの Python 3.8 用だけ。Python 3.12 なら CPU 用 | — |
| 無線 | PC2 の無線は 192.168.0.82（DHCP。変わることがある）。DDS（lowstate・腕）は無線に流れない | `real_robot_notes.md` |

## 当日の手順

### 1. つないで、調査スクリプトを流す（ラボ PC、5 分）

```bash
cd ~/dev/G1_jigen && export CYCLONEDDS_HOME=$PWD/G1_HuggingFace/cyclonedds/install
```
```bash
mkdir -p _local/pc2_survey
```
```bash
ssh -o PubkeyAuthentication=yes -o PreferredAuthentications=publickey -i ~/.ssh/id_ed25519_humanoid_expo g1 'bash -s' < Button_Press/J1-gen/real/pc2_survey.sh > _local/pc2_survey/$(date +%Y%m%d_%H%M).txt 2>&1
```

`real/pc2_survey.sh` は見るだけ（OS・glibc・CPU とメモリ・ディスク・Jetson の CUDA / TensorRT・システムの Python・conda の各環境の
パッケージ（torch・ultralytics・onnxruntime・cyclonedds・unitree_sdk2py・pinocchio・mujoco など）・gcc / cmake・ネットワークと
インターネット・カメラと常駐のサービス・時刻）。ラボ PC で試して、最後まで流れることを確かめてある（値はラボ PC のもの）。

### 2. 結果を読んで、次を決める

| 見るところ | こうなら | 次 |
|---|---|---|
| `pypi.org: HTTP 200` | PC2 がインターネットにつながる | PC2 で直接 `pip install` できる。オフラインの準備は要らない |
| 同上がタイムアウト | つながらない | ラボ PC で aarch64 用の配布物を集めて持っていく（`depth_server/fetch_offline_packages.sh` と同じやり方） |
| `lerobot` の `cyclonedds`・`unitree_sdk2py` | 入っている | PC2 で押す処理の DDS はそれを使う（venv を `--system-site-packages` で作る） |
| 同上 | 無い | PC2 で CycloneDDS（C）と Python 部分を組み立てる（`gcc`・`cmake` があるか、`~/work/unitree_sdk2/thirdparty` を見る） |
| `lerobot` の `torch` | 入っている（`cuda=False` でも） | YOLO は「やり方 A」（ultralytics）を試せる |
| 同上 | 無い | YOLO は「やり方 B」（ONNX）だけ |
| `tensorrt`（システムの Python 3.8） | 入っている | GPU で速くしたくなったら、後で TensorRT を使える（今回はしない） |
| `free -h`・`df -h`・CPU の上位 | 余裕が少ない | 押す処理・YOLO を足したときに、配信サーバ（1 コアの約 75%）や Unitree のサービスと取り合わないかを、YOLO の確認で測る |
| `date` | ラボ PC と数分以上ずれている（10/21 は約 54 分） | 記録の時刻を比べるときに注意する（直すのは OS の設定なのでしない） |

### 3. YOLO が PC2 で動くかを確かめる（人の認識）

**手順とコマンドは `Perception/real/README.md` の「G1 の PC2 の上で YOLO を動かす（check_yolo.py）」**（2026-10-22 に準備した。
ONNX と、PC2 用の配布物はラボ PC で用意済み: `_local/perception/models/yolo26n.onnx`、`_local/perception/pc2_wheels/`）。
次の実機日は、まず G1 の頭カメラで確かめる（Web カメラが使えるかは当日次第）。人はかがんで、G1 の前 0.5〜1 m で写り込む。

PC2 の頭カメラの配信（`rgbd_server.py`、5555 / 5556）はそのまま使う。YOLO は**配信を localhost で受け取って**動かす。
見るもの: 1 枚にかかる時間（ms）、毎秒の枚数、CPU の使用率、人（`person`）の検出がラボ PC と同じように出るか。

- **やり方 A（ultralytics + torch）**: `lerobot` に torch があれば、それを借りた venv（`--system-site-packages`）に ultralytics だけを入れて、
  `Perception/common/detector` の `YoloDetector` をそのまま使う。コードの変更が少ない。torchvision が無い・版が合わないと入らないことがある
- **やり方 B（ONNX + onnxruntime）**: ラボ PC で `yolo26n.pt` を ONNX に変換して持っていき、onnxruntime（CPU）で動かす。torch が要らず軽い。
  YOLO の出力の後処理を自分で書く（Perception に同じ形の検出器を足す）。押す処理を PC2 で動かすときにも同じものを使える
- どちらも、まず保存した画像 1 枚 → 次に配信からの連続、の順で確かめる
- **人がどう写るかも確かめる**: 頭カメラは約 45° 下を向いているので、立った人の全身はほとんど写らない見込み（下の計算）。
  G1 の前 0.5・1・2・3 m に人に立ってもらい、画像を保存して、どこまで写るか・YOLO が `person` と出すかを見る

  計算（すべて実測の値から: 配信の内部パラメータ fy 453.8・高さ 360 で上下の画角 ±21.6°、10/14 の向き 45.9°、10/21 に深度で
  測ったカメラから床までの 1.21 m）: 写るのは水平から下 24〜68° の範囲で、床が写るのは前 約 2.7 m まで。
  人は 0.5 m 先で床から 0〜0.98 m、1 m 先で 0〜0.76 m、2 m 先で 0〜0.31 m（足もとだけ）、3 m 先では写らない。
  警備の見回りのように離れた人を見つけるには、頭カメラの向きを上げる（首の傾きを変えられるなら。ボタン押しとは向きが
  逆の要求になる）か、前を向いた別のカメラが要る

### 3'. Web カメラ（人の認識用。頭カメラとは別に付ける予定、2026-10-22）

頭カメラ（RealSense）はボタン押しのために下向きのまま使い、人の認識は前向きの Web カメラで行う。**挿した状態で 1. の調査を流す**
（`/dev/video*` の名前・`lsusb` に出る）。見るもの:

- PC2 が認識するか（新しい `/dev/videoN` と名前）。Unitree の videohub が新しいカメラを掴まないか（`ps` の `videohub_pc4*`）
- OpenCV で読めるか、解像度・fps・MJPEG で受け取れるか（RealSense と同じ USB なら、帯域のため MJPEG にする）
- 取り付けの高さ・向きをメジャーで測る（人までの距離を、足もとが床のどこにあるかで求めるときに使う。Web カメラには深度が無い）
- 決めておくこと: カメラの型番・画角、付ける場所（頭・胸）と向き、つなぐ先（PC2 の USB を想定）

### 4. 押す処理を PC2 で動かす準備（2 の結果をもとに、別の日に）

見積もりは下の「見積もり」。この日の調査で、cyclonedds と YOLO の分の幅が決まる。

## 2026-10-31 の調査の結果（ノート PC から Wi-Fi の ssh で）

ノート PC（WSL2）から `ssh unitree@192.168.0.82`（無線）で入り、1. の `pc2_survey.sh` を流した（結果 `_local/pc2_survey/20261031_1727_laptop.txt`）。
そのあと PC2 に自分用の venv を作り、**計画だけ**と **lowstate の受け取りだけ**を試した（腕には何も送っていない）。

| 項目 | 結果 |
|---|---|
| DDS のライブラリ | `lerobot`（Python 3.12）・`tv`（3.10）・システムの 3.8 の**どれにも cyclonedds 0.10.2 と unitree_sdk2py 1.0.1（`~/unitree_sdk2_python`）が入っている**。押す処理が使う import（`ChannelPublisher`・`LowCmd_`・`LowState_`・`CRC` など）はすべて通る。**PC2 で組み立てる必要はない** |
| インターネット | `pypi.org: HTTP 200`。PC2 で直接 `pip install` できる |
| venv | `~/button_press/venv`（`lerobot` の Python から `--system-site-packages` で作った。`lerobot` 自体には何も足していない）。足したのは **pin 4.1.0・mujoco 3.14.0**（ノート PC と同じ版）と、その関連だけ（519 MB）。numpy は `lerobot` の 2.2.6 のまま |
| コード | `~/button_press/repo/` にリポジトリと同じ形で置いた（`Button_Press/J1-gen/{common,configs,sim/reach_map.py}`、`_local/button_press/models`。コミット 467a708 の時点）。10/14 に置いた `~/button_press/{common,configs,real}` は古いまま |
| 計画の時間（`real/plan_bench.py`、10/31 の配置 A〜F） | **PC2 で 1 回 0.17〜0.32 秒**（ノート PC は 0.06〜0.14 秒。約 2.5 倍遅いが十分速い）。準備（モデルの読み込み）は 1.0 秒。通る / 断るは両方で同じ（A・F は通る、B〜E は断る。10/31 の地図の予想どおり） |
| lowstate（`common/dds.py` の `LowStateReader`、受け取るだけ） | PC2 の eth0 で **約 1,040 通 / 秒**届く。**それだけで 1 コアの約 73% を使う**（1 通ごとに `parse_lowstate` で変換しているため）。送る周期は 50 Hz なので、最新の 1 通だけを残して読むときに変換すれば減らせる（段階 2 で直す） |
| 同時に動いていたもの | 別の班の地図づくりの記録（`record_onboard_with_camera.py` 50%・`sensor_stream.py` 46%・`innomaker_camera_server.py` 23%）、`videohub_pc4` 21%。メモリの空き 11 GB、ディスクの空き 1.7 TB |
| カメラ | 頭の RealSense（video0〜5）のほかに、Web カメラが 2 台（video6 `Full HD webcam`、video8 `Innomaker-U20CAM-1080p-S1`）。10/31 の Perception の記録とは番号が変わっている |
| 時計 | PC2 の時間帯は `Asia/Shanghai`（CST、日本より 1 時間遅い）。NTP で合っている |

PC2 で計画の時間を測る手順（ノート PC から。venv とコードは上のとおり置いてある。コードを更新したら `rsync` からやり直す）:

```bash
rsync -a --exclude __pycache__ Button_Press/J1-gen/common Button_Press/J1-gen/configs unitree@192.168.0.82:button_press/repo/Button_Press/J1-gen/
```
```bash
rsync -a Button_Press/J1-gen/sim/reach_map.py Button_Press/J1-gen/real/plan_bench.py --relative unitree@192.168.0.82:button_press/repo/
```
```bash
ssh unitree@192.168.0.82 'OPENBLAS_NUM_THREADS=1 ~/button_press/venv/bin/python ~/button_press/repo/Button_Press/J1-gen/real/plan_bench.py'
```

（2 つ目の `--relative` は、リポジトリの根で実行したときのパスをそのまま PC2 の `~/button_press/repo/` の下に作る）

## 見積もり（2026-10-31 に更新。10/22 の見積もりは 4.5〜8 日・3〜4 回だった）

10/31 の調査で、cyclonedds を組み立てる必要がないこと、計画が PC2 で十分速いことが分かり、段階 0・1 はほぼ済んだ。
残りは、PC2 の中で実際に腕を動かすこと（段階 2）と、PC から指示する仕組み（段階 3）。

| 段階 | 内容 | 作業 | 実機 |
|---|---|---|---|
| 0・1. 調査・Python 環境 | **10/31 に済んだ**（上の表） | — | — |
| 2. PC2 の中で押す | lowstate の受け取りを軽くする（上の 73%）。通信先（eth0・カメラは localhost）の設定、`auto_press.py` を PC2 の venv で通す。ボタンの YOLO を ultralytics から ONNX へ（ボトルの方は `detector.backend: onnx` で済む）。PC2 で 50 Hz の送信が乱れないかを測る。dry-run → ボトル | 1〜1.5 日 | 1 回 |
| 3. 常駐サービスと PC からの操作 | `press_service.py`・`press_ctl.py`（Perception の `person_alert` の ZMQ・心拍、`mjpeg.py` の映像の配信を流用）。模擬ロボットで二重の開始・停止・心拍切れ、実機で Wi-Fi を切って止まるか | 1.5〜2 日 | 1 回 |
| 4. エレベーター | 同じサービスから `run_dds.py` の経路も（Yada さんのコードは変えずに置く） | 0.5〜1 日 | 0.5〜1 回 |
| （任意）ブラウザの操作画面 | 開始・停止のボタンを MJPEG の画面に付ける | +0.5〜1 日 | — |
| 合計 | | **3〜4.5 日**（任意を入れて 3.5〜5.5 日） | **約 3 回** |

ずれやすいところ: PC2 で別の班の処理（地図づくり・カメラの配信）と同時に動かしたときの CPU の取り合い（+0.5〜1 日）、
押す処理で GPU（TensorRT）を使いたくなったとき（システムの Python 3.8 でしか動かないので、検出だけ別のプロセスにする。+1 日）。

## 見積もり（2026-10-22 の版。上の 10/31 の版に置き換えた）

### 押す処理を PC2 で動かし、ノート PC から無線で開始・停止・状態を指示する

```
ノート PC（WSL2）            無線            PC2（G1 の中）
  press_ctl.py  ──── start / stop / status ────▶  press_service.py（常駐、Python 3.12 の venv）
                                                     │ 押す仕事は 1 件ずつ、子プロセスで
                                                     ├─ auto_press.py → press_bottle.py / run_dds.py（今のまま）
                                                     │      DDS は eth0（G1 の中の有線）、カメラは localhost
                                                     └─ rgbd_server.py（今のまま。システムの Python 3.8）
```

停止は子プロセスに Ctrl+C と同じ合図を送る（今の安全な止め方を使う）。動いている間にノート PC からの心拍が途切れたら止めて戻る。
緊急停止は今までどおりリモコンの L2+B。電源を入れたときの自動起動は systemd（OS の変更）が要るので入れない（ssh で起動する）。

| 段階 | 内容 | 作業 | 実機 |
|---|---|---|---|
| 0. 現地調査 | このファイルの 1・2 | 0.5 日 | 0.5 日 |
| 1. PC2 の Python 環境 | venv（3.12）にライブラリを入れる。cyclonedds は既存を使うか組み立てる | 0.5〜1.5 日 | （0 と同じ日） |
| 2. PC2 の中で押す | 設定（eth0・localhost）、YOLO は ONNX、計画の時間と制御の周期を測る、dry-run → ボトル | 1〜2 日 | 1 日 |
| 3. 常駐サービスと無線の操作 | `press_service.py`・`press_ctl.py`、模擬ロボットで二重の開始・停止・心拍切れ、実機で Wi-Fi を切って止まるか | 2〜3 日 | 1 日 |
| 4. エレベーター | 同じサービスから `run_dds.py` の経路も（Yada さんのコードは変えずに置く） | 0.5〜1 日 | 0.5〜1 日 |
| 合計 | | 4.5〜8 日 | 3〜4 回 |

### 人の認識（Perception）を PC2 で

| 段階 | 内容 | 作業 | 実機 |
|---|---|---|---|
| W0. Web カメラを PC2 で読む | 認識されるか、OpenCV で読めるか、解像度・fps・MJPEG、videohub が掴まないか（3'.） | 0.5 日 | 0.5 日 |
| R0. YOLO が PC2 で動く | やり方 A なら ultralytics を入れて既存の `YoloDetector`（0.5 日）。やり方 B なら ONNX に変換し、同じ形の検出器を Perception に足して、ラボ PC で ultralytics と結果が合うかを確かめる（1〜1.5 日）。確認用のスクリプト（画像 1 枚・配信の連続・時間の計測） | 0.5〜1.5 日 | 0.5 日（調査と同じ日） |
| R1. PC2 で人を検出し続け、ノート PC で見る | Web カメラを PC2 で直接読んで検出し続ける常駐、結果（枠・確信度・時刻、間引いた画像）を無線で配る（今の RGB の配信と同じ形式で別のポート。`ZmqFrameSource` で受けられる）、ノート PC（WSL2）で枠を描いて見る | 1〜2 日 | 0.5〜1 日 |
| R2'. 人までの距離（深度なし） | Web カメラの較正（チェッカーボード）、取り付けの高さ・向きの測定、足もとを床に当てて距離を求める | 1〜1.5 日 | 0.5 日 |
| R3. 用途に合わせた処理 | 追跡（同じ人か）・人数の変化・見回りでの通知・ナビとの連携など。何をしたいかを決めてから | 1〜3 日 | 未定 |
| GPU（TensorRT） | CPU で足りないときだけ | +1〜2 日 | +0.5 日 |
| 合計（W0〜R2'） | | 3〜5.5 日 | 1.5〜2 回 |

頭カメラ（約 45° 下）では、立った人は 1 m 先でも膝くらいまで、3 m 先では写らない見込み（3. の計算）なので、人の認識は前を向いた
Web カメラで行う（2026-10-22 にユーザーが決めた）。

## 記録すること

- 調査の結果のファイル（`_local/pc2_survey/<日時>.txt`）
- YOLO: やり方、1 枚の時間、毎秒の枚数、CPU 使用率（配信サーバを動かしたまま）、人の検出の様子（画像を保存）
- 日ごとの記録（`real/DAILY_LOG_<日付>_*.md`）に、2 の表のどの行だったかを書く
