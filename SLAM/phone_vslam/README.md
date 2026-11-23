# phone_vslam — スマホの映像で visual SLAM を回し、ウェブ画面で見せる

スマホ（iPhone の Safari・Pixel の Chrome）のカメラ映像を、アプリなし・無線で OMEN に送り、
OMEN の GPU で MASt3R-SLAM を回して、地図（色つきの点群）と軌跡が育つ様子をブラウザに出す。

計画書: `physical_ai/docs/plan/2026-11-17-iphone-webrtc-visual-slam.md`

```
スマホの Safari ─ WebRTC（WHIP）・Physical_AI_5G ─→ OMEN 192.168.0.167
  https://192.168.0.167:8443/            送信ページ（serve_publish.py。背面・H.264 で始まる）
                                          └→ MediaMTX :8889・:8189（受け口）
                                               └→ rtsp://127.0.0.1:8554/cam1
                                                    └→ run_phone_slam.py（ffmpeg で解く → MASt3R-SLAM）
                                                         └→ Rerun のウェブ表示 :9090（地図・軌跡・いまのこま・状態）
```

## 置き場所（OMEN）

| 物 | 場所 |
|---|---|
| この台本 | `~/work_inoue/phone_slam/phone_vslam/`（このフォルダを rsync で置く） |
| MediaMTX v1.21.4・証明書・記録 | `~/work_inoue/phone_slam/mediamtx/`（`mediamtx.log`・`publish_page.log`） |
| MASt3R-SLAM（上流 e6f4e3d＋`patches/`）・重み | `~/work_inoue/phone_slam/MASt3R-SLAM/` |
| Python 環境 | conda `mast3r`（Python 3.11・torch 2.7.0+cu128・nvcc 12.8・numpy 1.26.4・rerun-sdk 0.22.1） |
| **SLAM の結果** | `~/work_inoue/phone_slam/results/<日時>_slam1/`（`map.ply`・`traj.txt`。**止めた時に**書く） |
| SLAM の記録 | `~/work_inoue/phone_slam/logs/slam1_<日時>.log`（`slam1_latest.log` が最新） |
| 録画（`record.sh` を使った時だけ） | `~/work_inoue/phone_slam/rec/<名前>_<日時>.mkv`・`.report.json` |

## 1. 準備（1 度だけ）

```bash
# Mac から台本を置く
rsync -a --exclude __pycache__ G1_Hackason/SLAM/phone_vslam/ ubuntu@100.99.102.70:work_inoue/phone_slam/phone_vslam/
ssh ubuntu@100.99.102.70
cd ~/work_inoue/phone_slam
# MediaMTX（https://github.com/bluenviron/mediamtx/releases の linux_amd64）
mkdir -p mediamtx && cd mediamtx
curl -fsSLO https://github.com/bluenviron/mediamtx/releases/download/v1.21.4/mediamtx_v1.21.4_linux_amd64.tar.gz
tar xzf mediamtx_v1.21.4_linux_amd64.tar.gz && cd ..
# MASt3R-SLAM（上流の取得・環境・重み 2.7 GB・試しの録画・組み立て。30 分ほど）
bash phone_vslam/setup_omen.sh all
```

上流への手当て（`patches/mast3r_slam_rtx50.patch`）: RTX 5060 Ti（sm_120）向けに

- `setup.py` に `sm_120` を足す（上流は sm_86 まで）
- torch 2.7 で消えた `torch::linalg::linalg_norm` → `at::norm`、`.type()` → `.scalar_type()`（SLAM の核と、MASt3R の curope の 2 か所）
- `pyproject.toml` から lietorch を外す（`setup_omen.sh` が版を固定して先に入れる）
- torch 2.6 から `torch.load` の既定が `weights_only=True` で MASt3R の重みが読めない → 走らせるときに `TORCH_FORCE_NO_WEIGHTS_ONLY_LOAD=1`（`start_slam.sh` が付ける）

## 2. 毎回の起動

```bash
ssh ubuntu@100.99.102.70
cd ~/work_inoue/phone_slam/phone_vslam
bash start_mediamtx.sh          # 受け口と送信ページ（起きていれば何もしない）
bash start_slam.sh              # SLAM と表示（既定の源は rtsp://127.0.0.1:8554/cam1）。重みを読んで映像を待つ
```

スマホ（Wi-Fi は **Physical_AI_5G**）:

1. Safari で `https://192.168.0.167:8443` を開く（ホーム画面に追加しておくと 1 押しで開く）
2. 証明書の警告を越える（iPhone は「詳細を表示」→「このWebサイトを閲覧」、Pixel は「詳細設定」→「アクセスする」）
3. 背面カメラの映像が出たら「送信開始」。左下が「送信中 0:xx」で数え続ければ届いている
4. 送っている間はブラウザを前面に置く（画面は消えないようにしてある。裏に回って戻ると自動で送り直す）
5. 最初の数秒は模様のある物に 1〜2 m 離れて向け、ゆっくり動かす

見る（Mac は Tailscale でも、同じ Wi-Fi でもよい）:

| 見る物 | URL |
|---|---|
| 地図と軌跡（SLAM） | `http://100.99.102.70:9090/?url=ws://100.99.102.70:9877` |
| 保存した結果（`start_slam.sh view`） | 同じ `:9090`（別の記録として並ぶ） |
| 生の映像 | `https://100.99.102.70:8889/cam1` |

同じ Wi-Fi なら `100.99.102.70` を `192.168.0.167` に替える。SLAM を起こし直したら表示のタブも再読み込みする
（前の回の表示が残る）。

止める: `bash start_slam.sh stop`（軌跡と点群を `results/<日時>_slam1/` に保存してから止まる）。

## 3. 結果を後から開く

```bash
bash start_slam.sh view                      # 最新の results/ を表示に送る（固定の口 :9090）
bash start_slam.sh view ~/work_inoue/phone_slam/results/1117_120510_slam1
```

- `map.ply` は色つきの点群（全部の画素。数百万〜数千万点）。表示では 200 万点に間引く。
  Mac に持ってきて CloudCompare・MeshLab で開いてもよい
- `traj.txt` はキーフレームの軌跡（TUM の形: 時刻 x y z qx qy qz qw。単眼なので大きさは実寸ではない）
- 録画があれば、同じ流れで SLAM を回し直せる: `bash start_slam.sh ~/work_inoue/phone_slam/rec/cam1_<日時>.mkv --realtime`

## 3b. 2 台で 1 つの地図（両手に 1 台ずつ持って歩いてよい。Multicam-MASt3R-SLAM）

2 台の位置関係を決めなくてよい（固定も較正もいらない）。2 台目は「地図の中の見たことのある所」を写すと地図に重なり、
その後は別々に歩いてよい。大きさは単眼と同じく実寸ではない（2 台の大きさは揃う）。

**よく使うコマンド**（OMEN で。Mac からは `ssh ubuntu@192.168.0.167`、網が違うときは Tailscale の `100.99.102.70`）

```bash
cd ~/work_inoue/phone_slam/phone_vslam

# 撮る（生）
bash start_mediamtx.sh stop && bash start_mediamtx.sh   # 表示の前の記録を全部捨てる（任意。ブラウザは再読み込み）
FORCE=1 bash start_multicam.sh       # cam1・cam2 の生の SLAM と 2 台の録画を起こす（机に自分がいるときは FORCE=1）
bash start_multicam.sh log           # 記録の最後を見る（合流・見失い・fps）
bash start_multicam.sh stop          # 止めて保存（results/<日時>_slammc/ に traj_cam1.txt・traj_cam2.txt・map.ply）。録画も止まる

# 見る
bash start_multicam.sh view                                       # 最新の結果を :9090 に出す
bash start_multicam.sh view ~/work_inoue/phone_slam/results/<日時>_slammc --conf-thresh 3   # 確からしさ 3 以上だけ

# 録画からきれいに回し直す（512・こまを捨てない。rec/ の同じ回の 2 本を指定。2 分の録画で 7 分ほど）
R=~/work_inoue/phone_slam/rec
ls -t $R/cam1_*.mp4 $R/cam2_*.mp4 | head -4                       # 最新の回の 2 本を探す
FORCE=1 bash start_multicam.sh $R/cam1_<日時>.mp4,$R/cam2_<日時>.mp4 --names cam1,cam2
#   ↑ 11-18 10:30 以降の録画は 15 fps なので全部のこまを使う。それより前の 30 fps の録画は --stride 2 を付ける
```

- 生の既定は長い辺 384（速さ優先）・緩い再測位。録画から回すときは長い辺 512。`--reloc-strict` で上流の厳しい再測位、
  `--img-size`・`--gn-iters` などは `python run_multicam_slam.py -h`
- 表示は `http://192.168.0.167:9090/?url=ws://192.168.0.167:9877`。回ごとに日時入りの新しい記録になる。前の地図が見えたら、
  右上の 3 つの四角の左端で左の窓を出し、Recordings で新しい日時（世界標準時なので日本時間 −9 時間）を選ぶ
- `start_multicam.sh` は机で人が使っていると GPU を使わずに止まる。机にいるのが自分なら `FORCE=1`

**撮り方**

1. iPhone は `https://192.168.0.167:8443/?cam=cam1`、Pixel は `?cam=cam2` で送信開始（2 台とも同じ向き。両方縦か両方横）。
   ページは開くたびに再読み込みする（fps を保つ指定・送る量の上限が入った新しいページになる）。
   送る量は 1 台あたり 15 fps・2.5 Mbps（11-18 に 30 fps・4 Mbps×2 台で包みがまとめて落ち、崩れたこまで見失った）。
   変えるときは URL に `&fps=30&kbps=4000` などを足す
2. **始めは 2 台で同じ所を写す**（模様のある所に 1〜2 m）。表示の状態に 2 台とも `TRACKING` と出たら重なった
3. 後は両手に持って別々の向き・道を歩いてよい。片方が見失ったら（`LOST`）、そのスマホで前に写した所を写す
4. **出入り口や角では止まるくらいゆっくり向きを変え、扉や壁に 1 m より近づかない**（11-18 に会議室の出口で 2 台とも
   ぶれて見失い、机に戻るまでの 20 秒が地図に入らなかった。見失っている間は新しい場所を地図に足さない）
5. **最後は始めの場所に戻って数秒写す**（前に見た所として重なり直し、1 周のループが閉じる）

表示（`:9090`）: 点は撮った色、軌跡は cam1 赤・cam2 青、黄緑の線は 2 台のキーフレームがつながった所（合流・相互のループ）。
録画から回し直す: `bash start_multicam.sh ~/work_inoue/phone_slam/rec/cam1_….mp4,~/work_inoue/phone_slam/rec/cam2_….mp4 --realtime`

中身: 派生版（johnkimdy/Multicam-MASt3R-SLAM 71058ea。上流との違いは `frame.py`（キーフレームにカメラの番号）と
`tracker.py`（同じカメラの直前のキーフレームと比べる）だけ）を `~/work_inoue/phone_slam/Multicam-MASt3R-SLAM/` に置き、
その `main.py` の代わりに `run_multicam_slam.py`（主ループ）・`multicam_core.py`（後ろの最適化）を使う。派生版の `main.py` の
不具合 5 つを直してある（`multicam_core.py` の頭に一覧）: 2 台目が照合の割合を見ずに追跡へ進む・2 台目の姿勢が毎こま単位行列から・
2 台目は見失っても再測位しない・別のカメラのキーフレームとの「直前」のつなぎが照合が悪くても残る・再測位で入れたキーフレームの
カメラの番号が 0 になる。基準のカメラは決めず、最初にこまが来た方が地図を始める。

TUM の机の 2 本（fr1_desk・fr1_desk2。同じ机を別の道で撮った物、正解は同じ座標）で試した（11-17・1,228 こま）:

| | 1 つの Sim3 で 2 本を合わせた誤差 | 1 本ずつ合わせた誤差 |
|---|---|---|
| desk（キーフレーム 13） | 4.0 cm | 3.9 cm |
| desk2（キーフレーム 23） | 5.7 cm | 5.7 cm |

2 本をまとめて合わせても誤差が増えない＝desk2 は desk の地図の正しい所に載った（始めの 10 こまほどで合流・見失い 0 回）。
比べ方は `tests/eval_multicam.py`。

**速さ**（11-17 に上げた。測りは `tests/bench_mast3r.py`・`--source` に録画を 2 本・`--realtime` で生と同じ速さ）:

| 手 | 中身 | 効き |
|---|---|---|
| fp16（`fast_mast3r.enable`・既定で入る。`--no-fp16` で外す） | MASt3R の読み取り部・復号部・頭（DPT と MLP）を fp16 で、指数を取る後処理は fp32 | 1 こまの追跡 92.6→73.1 ms・後ろの照合 4 組 424→323 ms・点のずれ 0.09% |
| 直前のキーフレームを照合しない（`--recent-skip 4`） | 検索で見つけた相手のうち、同じカメラの直前 4 枚は前進の鎖でつながっているので照合しない | 後ろの照合が 1 枚あたり 380〜440→220〜330 ms |
| 生の間の反復を絞る（`--gn-iters 5`・止めるとき `--final-gn-iters 20`） | 全体の最適化は前の答えから始めるので少ない反復で収まる。遅れたらまとめて 1 回 | 地図が育つほど重くなる最適化（60 枚で 1 回 613 ms）を抑える |
| 画を小さく（`--img-size 384`。**生の既定**） | MASt3R に渡す画の長い辺 512→384（画素 0.56 倍） | 速さ 1.4〜1.6 倍。TUM の 2 本の誤差 4.0/5.5→6.1/5.7 cm |

こまを全部順に回したときの 1 台あたりの fps（2 台同時）: TUM の 2 本で 2.5（最初）→ 3.7（fp16 など）→ 5.8（384）、
スマホの録画で 4.5（512）→ 6.3（384）。生と同じ速さで今日の録画を流すと、最初の作りは 1.6 fps で広間で見失ったまま戻れず
（キーフレーム 26）、384 では追跡中 6 fps 前後・キーフレーム 68〜77。生は 384 で回し、きれいな地図は録画から 512 で回し直す。

## 4. ステレオ（iPhone と Pixel を並べて固定する。DROID-SLAM・実寸）

⚠️ 2 台を**互いに動かないよう固定した時だけ**使える（11-17 に両手で別々に持って較正したら、2 台の向きの違いが組ごとに
5.5° ばらつき、解けなかった）。手で持つなら 3b 節。

単眼の MASt3R-SLAM の代わりに、2 台を並べた両目で DROID-SLAM を回す。基線から実寸（メートル）が出る。
環境は別（conda `droid`。`bash setup_droid.sh all`）。単眼の SLAM と同じ口（9090）なので同時には回さない。

1. **並べる**: 2 台をテープなどで**互いに動かないように**固定し、10〜15 cm 離して同じ向き（両方縦か両方横）にする。
   後ろから見て iPhone が左（逆でも較正が気づいて入れ替える）。2 台の**レンズの中心の間の距離**を定規で測る（mm）
2. **送る**: iPhone は `https://192.168.0.167:8443/?cam=cam1`、Pixel は `https://192.168.0.167:8443/?cam=cam2` で送信開始。
   Pixel で「publisher.js を読めない」と出たら、先に `https://192.168.0.167:8889/cam2/publish` を開いて警告を越える
3. **較正（景色で。市松模様は使わない）**: `bash start_stereo.sh calib --baseline-mm <レンズの間の mm>`
   → 様子は `http://<OMEN>:9090/?url=ws://<OMEN>:9877（表示の口は固定）`。2 台を模様のある所（机・棚・ポスター・本）に向け、
   少し止める → 向きや場所を少し変える、を繰り返す（30 組集まると COLMAP で解いて保存。1 分ほど）
4. **時刻合わせ（動きで）**: `bash start_stereo.sh sync` → 2 台を景色に向けたまま、一緒に「止める・振る・止める…」を 20 秒
5. **回す**: `bash start_stereo.sh start` → 表示は単眼と同じ `:9090`。止めるのは `bash start_stereo.sh stop`
   （全体の最適化をかけて `results/<日時>_stereo/` に保存。`start_slam.sh view` で開ける）

較正の結果は `calib/<日時>_scene/calib.json`（`rectified_check.jpg` は平行化した 1 組に水平線を重ねた物、
`images/` は集めた画、`colmap/` は復元）、時刻のずれは `calib/sync_<日時>.json`。
集めた組から解き直すには `bash start_stereo.sh solve calib/<日時>_scene --baseline-mm <mm>`。

較正の中身: COLMAP（pycolmap 4.2.1）で 2 台の画をまとめて復元し、各カメラのレンズの値（SIMPLE_RADIAL: f・cx・cy・k1）と、
同時に写した組ごとの「左 → 右」の姿勢を出す。外れを除いて平均し、レンズの間の距離で実寸にする。
景色からは大きさが決まらないので、距離の測り違いはそのまま大きさの違いになる（120 mm で 5 mm 違うと約 4%）。

合成の試し（電話なし。11-17。正解は fx 820・基線 0.12 m）:

| 試し | 結果 |
|---|---|
| 景色の較正（30 秒の部屋から 45 組） | fx 820.7・平行化の上下のずれ 中央値 0.16 px（9 割 0.72 px）・左右の向き正しい・45 秒で解けた |
| 起動の台本から集めて解く（止める・動くの映像） | 20 組・fx 819.6・上下のずれ 中央値 0.19 px |
| ステレオ SLAM（景色の較正で合成の部屋 30 秒） | 15〜24 fps・縮尺の誤差 0.06%・位置の誤差 1.4 cm（道のり 4.3 m） |
| 時刻（動き・急に変わる映像を 0.2 秒ずらす） | +200 ms（相関の山 1.00）。滑らかに動き続ける映像では 0.15 秒を +120 ms と読んだ → 止める・振るをはっきり |

## 4b. 2 台を別々に回す（単眼を 2 つ）

- 送る側: 2 台目のスマホで `https://192.168.0.167:8443/?cam=cam2`
- SLAM はカメラごとに別に回す（地図は 1 つに合わせない）: `SLAM_ID=2 bash start_slam.sh rtsp://127.0.0.1:8554/cam2 --kf-buffer 256`
  （表示は同じ `:9090` に別の記録として並ぶ。止めるのは `SLAM_ID=2 bash start_slam.sh stop`）
- ⚠️ 1 台分で GPU 8.2 GB（キーフレームの箱 512 個）を使う。2 台を同時に回すなら両方 `--kf-buffer 256` にする（未試験）

## 5. 道具

| 台本 | 用途 |
|---|---|
| `start_mediamtx.sh [start\|stop\|status\|urls]` | 受け口・送信ページ（WHIP を中継）・表示のサーバー（口は 9090 に固定）。`status` で届いている流れ（コーデック・受けたバイト数）を見る。`MTX_LOGLEVEL=debug` で詳しい記録 |
| `start_slam.sh [源] [引数…]` / `stop` / `log` / `reset` / `view` | SLAM と表示。机で人が使っている間は起こさない（`FORCE=1` で越える）。記憶は `MEM_MAX`（既定 8G）で囲う。`reset` で地図を捨てて作り直す |
| `publish/index.html`・`serve_publish.py` | 送信ページ（MediaMTX 付属の `publisher.js` を使う。背面・H.264・720p で始まる） |
| `record.sh [名前] [秒]` | 届いている流れを符号を解かずに録る（mkv）。終わると `stream_report.py` が解像度の動き・fps・こまの抜けをまとめる |
| `feed_test.sh [名前] [秒]` | スマホの代わりに TUM の録画を流す（電話なしで通しを試す） |
| `view_result.py` | 保存した `map.ply`・`traj.txt` を表示のサーバーに送る |
| `viewer_hub.py` | 表示の口の固定（どの道具も 127.0.0.1:9876 に送る。ブラウザは :9090） |
| `run_phone_slam.py --source …` | 本体。源は `rtsp://…`・録画のファイル（`--realtime` で生と同じく遅れたこまを捨てる）・上流の dataset のフォルダ |
| `start_multicam.sh [源] / stop / log / reset / view / rec on\|off` | 2 台で 1 つの地図（3b 節）。`start_slam.sh` を `RUNNER=run_multicam_slam.py`・`SLAM_ID=mc` で回し、MediaMTX の録画を API で入れ切りする |
| `run_multicam_slam.py --source a,b` | 2 台以上の本体（主ループ）。`multicam_core.py`（後ろの最適化・再測位・保存）・`multicam_viz.py`（表示） |
| `fast_mast3r.py` | MASt3R の推論を fp16 にする差し込み・画の長い辺を 512 以外にする差し込み（上流の中身は書き換えない） |
| `video_file.py` | 録画を ffmpeg で読む口（途中で解像度が変わる iPhone の録画も読める） |
| `tests/bench_mast3r.py <録画>` | 1 こまの追跡と後ろの照合の時間を、fp32 と fp16 で測り比べる |
| `rig/` | 2 台を胸に載せる板（Bambu Lab A2L で刷る。カバー付きの iPhone 11 Pro と Pixel 7a を縦に横並び・互いに動かないのでステレオにも使える・2 台の間 28 mm でカメラの間 約 11.7 cm・左右は外向き 0/10/20°・上下は台ごと −20〜+10° を 5° 刻み）。作り方は `rig/README.md` |
| `start_stereo.sh calib\|solve\|sync\|start\|stop\|log` | ステレオ（4 節）。`scene_calib.py`・`measure_sync.py`・`run_stereo_slam.py` を回す |
| `scene_calib.py capture\|solve` | 景色の較正（集める・COLMAP で解く） |
| `stereo_core.py` | 平行化（stereoRectify・左右の入れ替え）・組ごとの姿勢の平均・時刻のずれ（動きの相互相関・点滅） |
| `rtsp_grabber.py` | RTSP を ffmpeg で解く読み口（MASt3R にも DROID にも依らない） |
| `setup_droid.sh` | DROID-SLAM の環境（conda `droid`・torch 2.7.0+cu128。直し無しで組めた） |
| `tests/make_synthetic_room.py`・`make_synthetic_flash.py`・`eval_traj.py` | 電話なしでステレオを通す合成の映像と、正解との比べ |

## 6. 試し

```bash
~/miniforge3/envs/mast3r/bin/python -m pytest -q tests   # 部品（GPU・MediaMTX は要らない）
bash feed_test.sh cam1 &  bash start_slam.sh             # 通し（電話なし）
```

## 7. 罠（11-17 に踏んだ物）

- **iPhone の送信が 4〜8 秒ごとに切れた** → 受け口の ICE の候補を全部の網の口から出していた
  （`webrtcIPsFromInterfaces: yes`）。口ごとに別のソケットになり、iPhone との組に `127.0.0.1` や
  Tailscale の口が選ばれた。候補を LAN と Tailscale だけにして 1 つのソケットにしたら切れなくなった
  （54 秒続いた後に手で止めた）。同じ Wi-Fi の Mac の Chrome では元から切れなかった
- **iPhone は始めの約 2 秒を 360x640、その後 720x1280 で送る** → OpenCV の RTSP の読み口はこの変化を追えず、
  壊れたこまを渡すか `_step >= minstep` で落ちる。生の流れは ffmpeg の子プロセスで解き、決まった大きさに揃える
- **上流は見失うと前に見た所への再測位しか試さない** → キーフレーム 1 枚の地図で 2,400 回失敗し続けた。
  小さい地図（10 枚未満）は 3 秒見失ったら作り直す
- **見失った状態で映像が止まると、後ろのプロセスが再測位を空回りして GPU を使い続ける** → 映像が来ない間は一時停止する
- **rerun 0.22 の `serve_web` は、その前に何かを送ると戻らないことがある**（rerun-io/rerun#8417。
  `default_blueprint` も中で送る）→ 先に `serve_web`、後で `send_blueprint`。念のため表示が 60 秒止まったら起こし直す
- **rerun は 0.22.1 に固定**（0.23 から numpy 2 が要り、上流の numpy 1.26 と並べられない）。
  0.22 は後からつないだブラウザに最初から全部を流し直すので、表示へ送る量を絞ってある（約 220 kB/s）
- **record.sh の mkv は fps の欄が 1000**（時間の目盛り）→ 録画はこまごとの時刻で読む
- **MediaMTX 付属の送信ページは前面カメラ・AV1/VP9 で始まる** → 自前の送信ページ（8443）を使う。
  URL を手で打つと Safari の自動修正で `cam1` が `calm` になった
- **OMEN は共用機**。机で人が使っている間は GPU を使わない（`start_slam.sh` が見る）。ufw は無効（`ENABLED=no`）
- **単眼なので地図は実寸にならない**。大きさは MASt3R の見積もりで、だいたいメートルだが流れる
- **キーフレームは 512 まで**（GPU の記憶 約 3.3 GB）。超えると SLAM は止まる（`--kf-buffer` で増やせる）
- **表示の文字は英語**。Rerun のウェブ表示は日本語の字を持たない
- **表示の口は 9090 に固定**（ユーザーの依頼 11-17）。前は道具ごとに口を開き（9090・9092・9093）、道具を起こし直すと
  ブラウザが古い口につながったまま「Waiting for data」で止まった。いまは表示のサーバーを 1 つ常に動かし、道具はそこへ送る。
  新しい記録が来るとブラウザの表示がそちらに替わる（開き直さなくてよい）
- **送信ページは WHIP を 8443 で中継する**。8889 に直接送ると、Safari は 8889 の証明書の警告も別に越える必要があり、
  その許可が切れて「TypeError: Load failed」で送れなかった（11-17 の iPhone）
- **MASt3R-SLAM と MASt3R の重みは非商用**（CC BY-NC-SA 4.0）。デモ・研究の用途に限る（DROID-SLAM は BSD-3）
- **DROID-SLAM のステレオは基線を 0.1 と決め打ち**（`projective_ops.py` の `[-0.1, 0, 0]`）→ 出てきた位置と点に基線 / 0.1 を掛けて実寸にする
- **較正に市松模様は使わない**（ユーザーの決定 11-17）。部屋の景色を COLMAP で復元して解く。白い壁だけの所では解けない
- **droid 環境は numpy 2.4.6**（pycolmap 4.2.1 が引き上げた）。rerun 0.22.1・opencv 4.11・DROID はそのまま動いた。
  mast3r 環境は numpy 1.26.4 のまま（上流の固定）
- **2 台は両手に持つので、固定ステレオではなく Multicam**（ユーザーの決定 11-17）。並べて固定しないと、組ごとの向きが 5.5° ばらつき較正が解けない
- **見失っている間は新しい場所を地図に足さない**（MASt3R-SLAM の作り。前に見た所が見えるのを待つだけ）。11-17 にロボットの周りを
  回ったのは 2 台とも見失った後で、何も入らなかった。速く歩く・向きを急に変える・人が多い・30 cm まで寄る、で見失う。
  2 台同時だと 1 台あたり約 2.8 fps（1 台なら約 7）なので、ゆっくり歩く。撮る物の前で地図を始め、1.5〜2 m 離れて回る
- **表示のプロセスが Xid 31（0 番地への書き込み）で落ちた**（11-17 の 2 台の実機）→ 上流の `get_dirty_idx` は GPU の上で
  `torch.where(is_dirty)` をとる。nonzero は「数える → 書き出す」の 2 段で、その間に別のプロセスの `is_dirty[i] = True` が
  GPU で走るとはみ出す（数えた数が 0 なら書き先は 0 番地）。錠は命令を出す所しか守らない → CPU に写してから探す（`rerun_viz.take_dirty`）。
  表示のプロセスが落ちたら 10 秒以内に起こし直す
- **伏せたスマホの真っ暗な画で地図を始め、作り直しを 9 回繰り返した** → 暗い・模様の無い画では地図を始めない・重ねない（`view_problem`）
- **表示のサーバーが新しいブラウザに応えなくなった**（`Waiting for data`）→ 前に Tailscale で開いたタブへの送りが 3 MB 詰まっていた。
  古いタブを閉じ、`start_mediamtx.sh stop` → 起こし直す
- **iPhone の録画は途中で解像度が変わる**（360x640 → 720x1280）。OpenCV では読めないので録画も ffmpeg で読む（`video_file.py`）。
  送信ページは、はじめ `contentHint = "detail"`・`degradationPreference: "maintain-resolution"`（解像度を保つ）を頼んでいた。
  11-18 に Wi-Fi で包みが 43 個落ちた後、iPhone が約 20 秒 1 秒 0.2〜2 こましか送らず（解像度を保つ代わりに fps を下げた）、
  SLAM が見失った。いまは `contentHint = "motion"`・`maintain-framerate`（fps を保ち、混んだら解像度を下げる。ユーザーの決定 11-18）。
  効いた値は送信ページが `encoder:` の行で知らせる
- **2 台の表示は点を 3 つおきに間引く**（2 だと TUM の 2 本で 30 秒に 26〜43 MB を送った。各カメラの最新のキーフレームは育つので 3 秒おきに）
- **録画を「生と同じ速さ」で読む口が早送りになっていた**（11-17）→ 呼ばれるたびに少なくとも 1 こま進めていたので、主ループが
  空回りで速く呼ぶと 67 秒の録画が 35 秒で終わった。まだ時刻の来ていないこまは渡さず取っておく形に直した。速さを比べるときは
  「読み始めから何秒」が録画の長さと合っているかを先に見る
- **生と同じ速さの比べは、見失うかどうかで fps が大きく振れる**（見失っている間は再測位を 1 秒に 1 回しか試さない）。
  速さそのものは、こまを全部順に回す測り（`--realtime` なし）で比べる
- **送信ページは状態を OMEN に知らせる**（`mediamtx/publish_page.log` の `[スマホ]` の行）。11-17 に「2 台がそろって 8 秒で切れた」
  と見えたのは、2 台とも「止める」が押されていた
