# Button_Press / J1-gen（ボタン押し：ボトル押しプロトタイプ）

G1 の頭カメラ（RGB＋深度）で対象を見つけ、IK で腕を動かして押す機能。
最初の目標はボトル押し、最終的な目標はエレベーターのボタン。

- 全体の説明: [PROJECT_OVERVIEW.md](PROJECT_OVERVIEW.md)
- 設計と実装順: [HANDOFF_button_press.md](HANDOFF_button_press.md)
- 環境構築: [SETUP_button_press.md](SETUP_button_press.md)

## 構成

- `common/` — sim / real 共通のロジック
  - `config.py` — 設定ファイル（YAML）の読み込み
  - `robot_model.py` — 関節の並び（motor 番号）、公式モデルの読み込みと頭カメラの追加
  - `arm/` — 腕の指令部分（タスク1）。`ArmCommander` と、送り先（バックエンド）の切り替え
    - `commander.py` — 開始（weight 0→1）、移動、動いたかの確認、安全な終了
    - `safety.py` — 関節リミット、1ステップの最大移動量、作業空間の箱、補間
    - `backend_sim.py` — MuJoCo（胴体固定）。arm_sdk の weight ブレンドを近似する
    - `backend_dds.py` — 実機。`rt/arm_sdk`（プランA）/ `rt/lowcmd`（プランB）へ DDS で直接送る
  - `kinematics.py` — 片腕の FK / IK（URDF + Pinocchio）。指の向きを押す方向にそろえる
  - `collision.py` — 腕と体がぶつからないかを MuJoCo で確かめる（ハンドには衝突判定用の箱を足す）
  - `press_planner.py` — 押し込みの軌道（手前の姿勢 → 直線で押し込む → 戻る）。届かない・ぶつかる目標は拒否
  - `press.py` — 軌道を ArmCommander で実行する
  - `rgbd_protocol.py` — 深度付きストリームの形式（サーバとクライアントで共有）
  - `camera_rgbd.py` — 深度付きストリームの受信側 `RgbdZmqSource`（Perception の FrameSource を継承）
  - `perception_bridge.py` — Perception/common を別名で読み込む（どちらもパッケージ名が `common` のため）
  - `camera_geometry.py` — 頭カメラの座標 → pelvis 座標（腰 3 関節の FK 込み）
  - `localize.py` — 検出（YoloDetector）→ 枠の中の深度の中央値 → 逆投影 → pelvis 座標
  - `rgbd_io.py` — RgbdFrame のファイルへの保存と読み込み（カラー PNG、深度 16bit PNG、JSON）
  - `dds.py` — DDS の初期化（1 プロセス 1 回）と lowstate の受信
  - `sim_scene.py` / `sim_camera.py` — MuJoCo の机とボトル（倒れる自由物体にもできる）、頭カメラの RGB + 深度、セグメンテーションの検出器
  - `recording.py` / `recorder.py` — 収録（RGB・深度・lowstate を時刻付きで保存）と再生
  - `realday.py` — 実機日用のツールの部品（指先の FK、画像への投影、教えた姿勢、支持の確認）
  - `pipeline.py` / `pipeline_cli.py` — 全体をつなぐ流れ（検出 → 目標 → IK → 手前の姿勢 → 押し込み → 戻る）
  - `run_logger.py` — 1 回分の記録（RGB・深度・lowstate・検出・IK・送った指令。dry-run でも）
  - `tip_check.py` — ボトルが倒れたかの自動判定（いちばん高い点の高さの変化。深度）
  - `post_press.py` — 押したあとの確認の差し込み口（ボタン版で「点灯しなければ深く押し直す」に使う）
  - `elevator_buttons.py` — エレベーターの呼びボタン（一般用の ▲▼）を見つける。画像の候補 → ▲▼ の記号 → 3D → 柱の面の向き → 上の 2 つ
  - `elevator_press.py` — 呼びボタンを押す制御（50 Hz で 1 歩ずつ）。Yada の評価環境のエージェントの中身
- `sim/` — MuJoCo での検証
  - `fetch_models.sh` — 公式モデル（unitree_ros の `g1_29dof_rev_1_0`）を取得する
  - `move_arm_sim.py` — 腕を指定の関節角へ動かして戻す
  - `press_sim.py` — 指定した点（pelvis 座標）を押す
  - `locate_sim.py` — 頭カメラの画像と深度からボトルの位置を求め、正解と比べる
  - `press_bottle_sim.py` — 全体をつなぐスクリプト（MuJoCo、同じプロセス）
  - `knock_bottle_sim.py` / `knock_sweep.py` — ボトルを倒す動作（MuJoCo）と、その条件の掃引（自動判定との一致も見る）
  - `sim_robot_server.py` — ループバックの模擬ロボット（実機と同じ DDS とカメラ。故障をわざと起こせる）
  - `make_button_dataset.py` — ボタンの YOLO の学習データを、Yada の評価環境の画像から作る（正解の枠は自動）
  - `train_button_yolo.py` — ボタンの YOLO（クラス up / down）を学習する
  - `record_trial_video.py` — 評価環境の 1 試行を、実際の速さで再生される動画にする
  - `merge_button_datasets.py` — シミュレーションの学習データと、枠を付けた実機の写真を混ぜる
- `contest_agent/` — Yada の評価環境（`Button_Press/Yada/contest`）で動かすエージェント（中身は `common/elevator_press.py`）
  - `rehearsal.py` — 実機日のリハーサル（当日手順書の段階0〜6と故障の場面を、模擬ロボットで通しで実行）
- `real/` — 実機用
  - `move_arm_real.py` — 同じことを実機で行う（既定は dry-run。送るには `--execute`）
  - `REAL_DAY_PROCEDURE.md` — 実機日の手順書（段階0〜6）
  - `depth_server/` — PC2 で動かす深度付きカメラサーバ（`rgbd_server.py`）と、その使い方（`README.md`）
  - `probe_rgbd.py` — 深度付きストリームが届くかを確かめる（ラボ PC で）
  - `locate_bottle.py` — ボトルの位置を pelvis 座標で求める（ライブ、保存したファイル、収録から）
  - `record.py` — 収録ツール（全フレーム / N 秒ごと / Enter を押したとき、深度付き / RGB だけ）
  - `check_connection.py` — 段階0の接続確認（lowstate、`mode_machine`、モータの mode、腰、指先、深度）
  - `teach.py` — ティーチング（手で動かした腕の姿勢を `configs/taught_poses.yaml` に記録）
  - `fk_check.py` — FK の確認（指先の位置の表示、頭カメラの画像への投影、指先の点を試しに変える）
  - `calibrate.py` — 較正（カメラで求めた位置と、指先で触れた位置の差を複数か所で取る）
  - `press_bottle.py` — 全体をつなぐスクリプト（実機。既定は dry-run）
  - `knock_bottle.py` — ボトルを倒す（実機）。`press_bottle.py` に `configs/knock.yaml` を重ねる
  - `check_button_photos.py` — 撮った写真で、呼びボタンを classic と YOLO で見つけられるかを並べて比べる
  - `label_button_photos.py` — 撮った写真に、呼びボタンの正解の枠を付ける（YOLO の結果を下書きにして直す）
- `configs/` — 設定ファイル。当日はコードを編集せず、ここだけを変える
  - `robot.yaml` — モデルのパス、頭カメラの取り付け位置
  - `arm.yaml` — 経路（sim / arm_sdk / lowcmd）、NIC 名、ゲイン、安全の上限、重力補償
  - `press.yaml` — IK、衝突の確認、押し込み（手前の距離・深さ・速さ）、作業空間の箱
  - `depth_server.yaml` — 深度付きカメラサーバ（PC2）: RealSense のシリアル番号、ポート（深度付き 5556 / RGB 互換 5555）
  - `camera.yaml` — 受信側（ラボ PC）の接続先とポート
  - `localize.yaml` — 検出器（YOLO の重み・クラス）、深度を取る基準点と領域、較正の補正
  - `sim_scene.yaml` — MuJoCo の机とボトル、検出するときの姿勢
  - `record.yaml` — 収録の保存先、カラーの形式、lowstate の周期、間引きの間隔
  - `taught_poses.yaml` — ティーチングで記録した姿勢（`teach.py` が書く。実機日のあとコミットする）
  - `pipeline.yaml` — 全体の流れ: 対象の与え方（depth / manual）、押す方向、経由の姿勢、IK の初期値、記録の保存先
  - `camera_sim.yaml` — 模擬ロボット用のカメラの接続先（127.0.0.1）
  - `elevator.yaml` — エレベーターの呼びボタン: 探し方（classic / yolo）、大きさ・間隔、押し方、壁の箱
- `real/check_head_tilt.py` — 頭カメラの向き（首の傾き）を、深度の床・机の面から確かめる（実機日の最初。10/14 の深度で 46.8°）
- `real/measure_gap.py` — 横から撮った画像・動画のフレームで、指先とボトルの面のすき間を、定規やボトルの太さを物差しにして測る（クリックか座標）
- `real/make_table_boxes.py` — 立った状態の深度から、机の障害物の箱を左右 9 cm ごとに作る（既定は確認だけ。`--save` で保存）
- `tests/` — `run_tests.sh`（CI が自動で見つけて実行する）

## 機体モデル

- 機体構成は `g1_29dof_rev_1_0`（`mode_machine = 5`）。unitree_ros の
  `robots/g1_description/README.md` の表で対応を確認した。
- IK 用の URDF とシミュレーション用の XML は、同じコミットの同じ構成からそろえて取る
  （`sim/fetch_models.sh`。取り出し先の `_local/` は git 管理外）。
- 公式 XML には頭カメラが無いので、読み込み時に `configs/robot.yaml` の値で `torso_link` に追加する。
  値は `g1_29dof_rev_1_0.urdf` の `d435_joint`（z = 0.42987）。
  `SimEnv3D/src/g1_twin/sensor_rig.py` の値（z = 0.41987）は旧版 `g1_29dof.urdf` 基準のため使わない。
  **実機日の較正で確認する。**
- シミュレーションでは、`floating_base_joint` を消して pelvis をワールドに固定する
  （MuJoCo に内蔵の歩行コントローラが無いため。腕の IK と軌道の確認には十分）。

## 状態

| # | タスク | 状態 |
|---|---|---|
| 0 | 土台づくり（構成、環境構築、モデル取得、CI 登録） | 済み |
| 1 | 腕の指令部分（sim / arm_sdk / lowcmd） | 済み（実機は未確認） |
| 2 | FK / IK と軌道生成 | 済み |
| 3 | 深度付きの配信サーバ（PC2） | 済み（本物の RealSense では未確認） |
| 4 | ボトル検出 → 3D 座標 → pelvis 座標 | 済み（YOLO は実機の画像で未確認） |
| 5 | 収録ツール | 済み |
| 6 | 全体をつなぐスクリプト | 済み（MuJoCo と模擬ロボットで確認。実機では未確認） |
| 7 | 実機日用のツールと手順 | 済み（実機では未確認） |

## 腕の指令部分（タスク1）

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/move_arm_sim.py
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/move_arm_sim.py --realtime   # 途中で Ctrl+C を試せる
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path arm_sdk              # dry-run
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/move_arm_real.py --path arm_sdk --execute    # 実際に送る
```

- 開始時に、`mode_machine` が 5 であること、指令する関節のモータが有効（mode=1）であることを確認し、
  違えば何も送らずに中止する。
- 移動後に lowstate を読み、指令した変化の 3 割未満しか動いていなければ「送信したのに動かない」として止める。
- Ctrl+C / SIGTERM / 例外のどれでも、プランA は現在の姿勢のまま weight を 1→0、
  プランB は現在の姿勢を保持してから送信をやめる。
- 押し込み（`execute_press`）の途中で中止したときは、その前に**その場で止めて、来た経路を逆にたどって開始姿勢へ戻る**
  （`arm.yaml` の `safety.retreat`）。腰のずれ・動かない・目標の拒否・q・Ctrl+C のどれでも。lowstate が途切れたときは
  腕の位置が分からないので戻らない。**戻る途中でもう一度 Ctrl+C を押すと、戻るのをやめてその場で weight を下げる。**
  戻る途中で腰のずれが 1° 以上増える、または腕が指令から 14° 以上離れる（何かに当たっている）と、戻るのをやめる。
- 腰は開始時の角度を保持する指令を送る（プランA でも `include_waist_hold: true`）。実測の腰の角度が
  開始時から `waist_max_deviation_rad`（既定 0.05 rad ≈ 2.9°）以上ずれたら中止する。
- プランB（実機の lowcmd）は、開始前に必ず「座った状態、または吊り下げた状態か」を聞き、Enter を待つ。
- 重力補償: `gravity_compensation.scale`（0.0〜1.0、既定 0）× 腕の重力トルクを `tau` で送る。
  1 関節あたり `tau_max_nm` で上限をかける。重力トルクは公式モデルと IMU の姿勢から計算する。
  `--gravity-scale 0.5` のようにコマンドラインでも変えられる。

### シミュレーションで分かったこと（2026-09-28）

- 公式モデルは関節のアーマチュアと減衰が 0 で、手首ロールの慣性が非常に小さい（約 0.00037 kg·m²）。
  PD の Kd 項をそのまま計算すると数値的に発散したため、MuJoCo の関節減衰と implicitfast 積分器で計算している。
- 重力補償なし（実機のプランB と同じ条件）では、Kp=60 で腕が 1.7〜2.5° 下がる。
  IK の精度（手先 1cm）に効くので、タスク2で扱う。
- 右腕の肩ロールを正（内向き）に動かすと、上腕が胴体（`torso_link`）に当たる。
- 重力補償（MuJoCo 側の補償を切った、プランB と同じ条件）: 倍率 0 / 0.5 / 1.0 で腕の誤差が
  1.74° / 0.87° / 0.01°。
- プランB の腰の保持が公式 low_level サンプルの Kp=40 だと、上体の重さで腰ピッチが約 19° 前に倒れた。
  そのため**腰の3関節だけ Kp=300 / Kd=3.0（xr_teleoperate の値）**にした（2026-09-28 決定）。このとき約 1.3°。
- 腰の重力補償（`gravity_compensation.waist_scale`、既定 0 = オフ）も用意した。プランB のときだけ効く。

## FK / IK と押し込みの軌道（タスク2）

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/press_sim.py --target 0.40 -0.20 0.05 --push-dir 1 0 0
```

- 手先の点は**中指の先**（`robot.yaml` の `end_effector`）。公式のハンドのメッシュで、指の先が並ぶ向き（z）に
  親指・人差し指・中指・薬指・小指があり、中指が最も長い（x = 0.1733 m）。実機日に `fk_check.py --overlay` で確認する。
  2026-09-28 に、y の符号を左右で取り違えていた（手の外の点になっていた）のを直した。
- IK は減衰付き擬似逆行列の微分 IK。動かすのは片腕 7 関節だけで、腰と反対の腕は固定。
  指の向き（手先リンクの x 軸）を押す方向にそろえ、指の軸まわりの回転は自由にする。
  余った自由度で初期値（ティーチングで記録した姿勢）に近づける。
- 送る前に（dry-run でも）、次のどれかに引っかかる目標は拒否する（`UnreachableError`）:
  IK が収束しない、関節リミットに当たる、作業空間の箱の外、腕が体にぶつかる
  （手前の姿勢、そこへの移動の途中、押し込みの直線上の全点）、1 周期の関節の動きが上限を超える。

### 確かめたこと（2026-09-28）

- 公式 URDF（Pinocchio）と公式 XML（MuJoCo）で、手先の位置の差は最大 0.001 mm 未満（左右の腕、ランダムな姿勢）。
- MuJoCo で押し込みを実行すると、手先の誤差は手前の点で 0.3 mm、押し込み終わりで 0.4〜0.5 mm。
- 公式モデルのハンドには衝突判定の形状が無いため、衝突の確認ではメッシュを包む箱を足している。
- 肩のリンクはゼロ姿勢でも胴体から約 9 mm しか離れていないので、肩はめり込みだけ、
  肘から先とハンドは 1 cm 未満に近づいたら「ぶつかる」とする。
- 手前の姿勢に着いた時点で lowstate から実際の腰の角度を読み直し、その角度で押し込みの軌道を計算し直す
  （`execute_press(..., planner=...)`）。プランB の条件（MuJoCo 側の補償なし、腰 Kp=300）での押し込み終わりの誤差:

  | 条件 | 誤差 |
  |---|---|
  | 重力補償なし | 41.8 mm |
  | 腕の重力補償 1.0 | 8.5 mm |
  | + 実測の腰で計算し直し | 1.8 mm |
  | + 腰の重力補償 1.0 | 0.7 mm |

  計算し直したあとに残る約 2 mm は、押し込みで腕が前に伸びる間に腰がさらに少し倒れる分。
- 真下に押す姿勢は手首が特異姿勢に近く、1 周期の関節の動きの上限で拒否された（横から押すボトルでは問題ない）。

## 深度付きの配信サーバ（タスク3）

PC2 で RealSense を pyrealsense2 で直接読み、深度をカラー画像に位置合わせして 16bit のまま、内部パラメータと
一緒に配信する（ポート 5556）。既存と同じ形式の RGB（ポート 5555）も出すので、既存の `ZmqFrameSource` もそのまま動く。
使い方・PC2 の準備・うまくいかないときは [real/depth_server/README.md](real/depth_server/README.md)。

- `run_g1_server.py` は変更しない。RealSense は 1 つのプログラムしか開けないので、このサーバを使うときは
  `run_g1_server.py` を `--camera` なしで起動する。
- 配信は既定で 10 fps に間引く（`configs/depth_server.yaml` の `publish.max_fps`）。
- 配信サーバは PC2 のシステムの Python 3.8 で動かす（Python 3.10 / 3.12 用の pyrealsense2 は glibc 2.34 / 2.38 を必要とし、
  PC2 の glibc 2.31 では動かなかった。2026-09-29）。
- PC2 はインターネットにつながっていない可能性が高いので、pyrealsense2（Python 3.8 用）と numpy などと
  libusb-1.0 の .deb を事前にダウンロードして持っていく（`real/depth_server/fetch_offline_packages.sh` →
  PC2 で `install_offline.sh`。sudo は使わず、libusb は中身を取り出すだけ）。
- 既存の `run_g1_server.py --camera` は、RealSense を pyrealsense2 ではなく OpenCV で `/dev/video4` として開いている
  （lerobot の `ImageServer`、2026-09-28 に GitHub の main で確認）。

## ボトル検出 → 3D 座標 → pelvis 座標（タスク4）

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/locate_sim.py --save
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --live
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --files _local/button_press/probe/<stem>
```

1. Perception の `YoloDetector`（事前学習済み、COCO の `bottle`）で枠を得る
2. 枠の中の基準点（`localize.yaml` の `depth.anchor`）のまわりの深度から、0 と範囲外を除いて中央値を取る
3. 内部パラメータで逆投影してカメラ座標（x 右、y 下、z 前）の点にする
4. `d435_link` → `torso_link`（`robot.yaml` の取り付け位置）→ 腰 3 関節の FK → pelvis 座標。較正の補正を足す

### 確かめたこと（2026-09-28）

- 座標変換は URDF の `d435_link` と一致（誤差 0）。腰を動かした姿勢でも同じ。
- MuJoCo の深度から逆投影した点と、同じ画素へ飛ばした光線が実際に当たる点の差は 0.5 mm 未満（画像の隅、腰を動かした姿勢を含む）。
- MuJoCo のボトル（腰 0°）で、求めた点は胴の前面から x 方向 0.9 mm、ボトルの中心から左右 5.9 mm。
- 頭カメラは斜め上から見下ろすので、ボトルの枠の中心には肩が写る。胴の前面を狙うため、基準点を縦 0.7 にした
  （ボタンのように正面を向いた対象は 0.5）。
- **検出するときは、腕をカメラの視野から外す**（肘を曲げた姿勢だと手がボトルを隠す）。シミュレーションでは
  `sim_scene.yaml` の `detection_pose_deg`（両腕を下ろした姿勢）。
- 腰を回すとボトルを斜めから見るので、狙う点が円柱の横寄りになる（腰ヨー 10° で中心から 26 mm）。押すときは
  腰を保持するので、腰がほぼ 0° のまま検出する前提。
- 事前学習済みの YOLO は、MuJoCo の円柱を組み合わせた作り物のボトルを検出しなかった。シミュレーションでは
  セグメンテーション（画素ごとに写っている物の番号）で枠を作る。**YOLO は実機日に撮った画像で確かめる。**

## 収録ツール（タスク5）

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label bottle                                  # 全フレーム
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button --mode interval --interval-s 1   # 1 秒に 1 枚
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label button --mode enter                     # Enter で 1 枚
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/record.py --label bottle --rgb-only                       # 深度なし
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --recording _local/button_press/recordings/<フォルダ> --every 10
```

- 保存先は `_local/button_press/recordings/<日時>_<ラベル>/`。カラー（PNG）、深度（16bit PNG）、
  フレームごとの時刻と腰の角度（`frames.jsonl`）、lowstate（`lowstate.csv`、100 Hz、関節角・速度・IMU）、
  内部パラメータと使った設定（`meta.json`）。途中で落ちてもそれまでの分が残るよう、1 行ずつ追記する。
- 時刻はラボ PC の時計。lowstate は DDS で直接受信する（`--no-lowstate` で記録しない）。
- 再生（`common/recording.py` の `Recording`）は、フレームと、そのときの腰の角度を返す。
  `locate_bottle.py --recording` で、収録したデータからオフラインでボトルの位置を求められる。
- **実機のデータは取り直せない。終わったら必ずバックアップする**（終了時にコマンドを表示する）。

## 実機日用のツール（タスク7）

手順は [real/REAL_DAY_PROCEDURE.md](real/REAL_DAY_PROCEDURE.md)。どのツールも lowstate と頭カメラを読むだけで、何も送信しない。

- `check_connection.py`: 段階0。`mode_machine` = 5、モータの mode = 1 を確かめる
- `teach.py`: 段階3。腕を手で動かした押し込み姿勢を記録する（IK の初期値に使う）
- `fk_check.py --overlay`: 段階3。FK の中指の先を頭カメラの画像に投影する。実物の中指の先に重なれば、
  FK とカメラの取り付け位置の両方が合っている。ずれていれば `--ee-offset` で指先の点を試しに変える
- `calibrate.py`: 段階4。押す位置の近くの複数か所で、カメラで測った位置（先に腕をどけて測る）と、
  中指の先で触れた位置（FK）の差を取り、場所ごとの差と平均からのずれを表示する。平均を補正値にする
- 腕を手で動かす `teach.py` / `calibrate.py` は、ダンピング状態（全身が脱力する）で使うので、開始時に
  「座った状態か吊り下げた状態か」を確認して Enter を待つ
- 経路の判定（段階2）は、Dev/Navigation の g1-starter-kit の `mode_check.py` / `armsdk_probe.py` を
  `git worktree` で別のフォルダに出して、そのまま実行する（このブランチには取り込まない）

確かめたこと（2026-09-28、実機なし）: 投影と逆投影が往復で一致する。MuJoCo で、FK の中指の先を頭カメラに
投影した画素に右手が写っている（取り違えていた値では写っていなかった）。教えた姿勢の保存と読み込み、
補正値の書き込み（コメントは残る）。

## 全体をつなぐスクリプト（タスク6）

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/press_bottle_sim.py                     # MuJoCo（同じプロセス）
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/press_bottle_sim.py --view              # 画面で動きを実時間で見る（WSL は WSLg で表示）
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk             # 実機、dry-run
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk --execute   # 実機、送信（確認モード）
```

流れ（`common/pipeline.py`）: lowstate と相手の確認 → 対象の位置（depth: 複数フレームの中央値、較正値を必ず足す。
manual: 設定の点、または教えた姿勢の中指の先 + 定規のずれ）→ 計画（IK の初期値は 教えた姿勢 → 今の姿勢 → 腕のゼロ姿勢 の順に試す。
届かない・ぶつかるならここで中止し、何も送らない）→ 経由の姿勢 → 手前の姿勢 → **実測の腰の角度で必ず計算し直す** → 押し込み →
保持 → 戻り → 押したあとの確認（押し直しは上限付き）→ 経由の姿勢 → 開始姿勢。

- 記録: `_local/button_press/runs/<日時>_<ラベル>/`（dry-run でも）。形式は `common/run_logger.py` の先頭のコメント。
- 机などの障害物は `configs/press.yaml` の `obstacles` に箱で書く。両腕を下ろした姿勢から手を上げると、手が机の縁に
  当たった（MuJoCo。腰のずれの監視で止まった）。経由の姿勢（`pipeline.yaml` の `via_pose`）で、机の縁より手前で手を上げる。
- 「送信したのに動かない」の判定は、腕全体の動き（指令の向きへの射影）で見る。押し当てで 1 つの関節だけ押し戻されても
  誤判定しない（模擬ロボットで、手首ピッチだけ押し戻されて誤判定したため変えた）。
- **押し込みは、指先が進まなくなったところで止める**（`common/press.py` の `ContactStop`、`press.yaml` の
  `press.contact_stop`。エレベーターの呼びボタンと同じ値）: 指令した指先が実際の指先より押す向きに、**押し込みを始めたときの遅れから** 5 mm 以上増えたら、
  残りの直線は送らず、実際の指先の約 2 mm 先の点まで直線をさかのぼって保持し、そこから戻る。
  - **遅れの絶対値ではなく増え方で見る**（2026-10-21）: 実機は重力補償 0.5 だと押す前から 4.5 mm 遅れていて、絶対値 5 mm では約 1 mm
    進んだだけで止まり、ボトルの約 5 cm 手前で止まった。押し込みのたびに「指先が実際に進んだ距離」を表示し、
    `events.jsonl` に `press_result` として残す（届いたかは、これで見る）
  - 以前は面の 15 mm 奥まで押し続けた。MuJoCo のボトル（固定されていて動かない）では遅れが 25 mm まで増え、その反動で
    腰が 2.9° ねじれて安全確認（腰のずれ）で中止し、中止で腕の指令が止まったあと、手が机の天板に最大 37 N で当たった
    （2026-10-15、621 通りの配置のうち 7 通り。多くは机 0.79 m・ボトルが右 25 cm）
  - 止めるようにして、机との接触は 0 N、腰のずれでの中止は 7 → 2 通り（残りは押す点 28 cm・机の縁 25 cm の、勧める範囲の外）、
    ボトルを押す力は最大 19.4 → 10.4 N。触れる前の遅れは 2 mm 以下
  - 実機のボトルは柔らかいので、押し込みの途中で遅れがゆっくり増える。遅れ 5 mm で止まったときの押す力は実機で確かめる
- **安全確認で中止したら、来た経路を逆にたどって開始姿勢へ戻ってから weight を下げる**（`common/press.py` の
  `PressRetreat`）。以前は、その場で weight を下げていたので、腕が内蔵コントローラの姿勢へまっすぐ下がって机に当たった
  （上の 37 N はこれ。中止そのものは正しい）
  - 押し当てているときは、指令を押し込みの直線の点を逆にたどって、実際の指先の位置に当たる点まで戻してから止める
    （押す力が抜ける）。そのあと直線を手前の姿勢までさかのぼり、経由の姿勢 → 開始姿勢。押し当てていないときは、
    今の指令のまま止める。実測の姿勢をそのまま指令にすると、重力で下がっている分だけさらに下がった（手首が机の縁に 4.4 N）。
    関節ごとに速さの上限で寄せると、指先が直線から外れてボトルを押した
  - MuJoCo（2026-10-15、押し込みを止める処理を外して腰のずれで中止させた配置と、今の設定で中止する配置）:
    机 30.5 → 0 N・35.5 → 0 N（机 0.79 m）、24.1 → 8.0 N（机 0.74 m・縁 27 cm。中止の前から手首が縁に触れていて、
    戻り始めの 1 周期だけ）、ボトル 29.6 → 13.2 N・36.9 → 14.6 N（机 0.69 m・押す点 28 cm）。どれも開始姿勢まで戻った
- **立つ位置（ボトル）**: 勧める範囲は、**押す点（ボトルの手前の面）まで 30〜40 cm、押す点が体の中心から右に 15〜25 cm、
  机の手前の縁まで 29 cm 以上**（`pipeline.yaml` の `stand`。左腕なら左右を反転）。ボトルは机の縁の近く（10 cm 以内）に置く。
  範囲の外なら「押す点まで 47.1 cm と遠すぎる。あと 8 cm 以上近づく」のように出す。計画できなければ中止の理由に添え、
  計画できれば注意（⚠️）だけ出して続ける。エレベーターの呼びボタン（柱まで 29〜40 cm）とは、範囲を決めているものが違う
  - 近い側は**ボトルではなく机の縁**で決まる: 経由の姿勢へ手を上げる途中で、手首が机の縁に当たる（計画で断る）。
    縁までの最小は、机の高さ（床から）0.69 m で 25 cm、0.74 m で 27 cm、0.79 m で 29 cm
  - 遠い側は腕の届く距離: 押す点まで 0.69 m の机で 38〜41 cm、0.74・0.79 m の机で 43〜45 cm
  - 押す点が体の正面寄り（右に 5 cm 以内）だと、右肩が胴体に当たって計画できない
  - 調べ方（2026-10-15）: 計画だけの網羅（約 2600 通り）と、`press_bottle_sim.py` の流れ全体（検出 → 計画 → 押す → 戻る）を
    机の高さ 3 通り × 縁からボトルまで 3 通り × 押す点まで 26〜48 cm × 右に 15・20・25 cm の 621 通り。
    勧める範囲は 3 つの高さのどれでも押せる値（机が 0.74 m 前後なら、押す点 43 cm・縁 27 cm まで押せた）

### 押している間の体（pelvis）の動き（実機で見る）

実機の G1 は、腕を伸ばしたり押した反動を受けたりすると、**足側のバランス制御が腰の位置を直す**。シミュレーション
（J1-gen も Yada さんの評価環境も）は腰を固定しているので、この動きは見えない。対象の位置は押す前に pelvis 座標で
測っているので、そのあとに腰が動くと、そのぶん狙いがずれる（押した反動で体が後ろへ逃げると、腕は計画どおりでも
部屋から見ると浅くしか押せない）。

- 全体をつなぐスクリプトは、終わったときに体の動きを表示し、記録に `body_motion.csv` と出来事 `body_motion` を残す。
  対象を測るときの lowstate も `lowstate.csv` に入れるようにした（基準にするため）
- 求め方（`common/body_motion.py`）: 両足が床に着いて滑らないとして、**脚の関節角（lowstate の q0〜q11）の FK** で
  足首から見た pelvis の位置と向きを出し、対象を測った時刻からの変化を見る。新しいトピックは使わない
  （G1 の位置の推定 `sportmodestate` は PC2 の SDK に型が無く読めない。Mapping 班の記録）
- 出すもの（基準の時刻の pelvis 座標）: pelvis の移動 [mm] と回転 [°]、**狙いのずれ** [mm]（測った対象の点が今の pelvis
  から見てどこへ動いたか）、IMU の傾きの変化（脚の FK と大きく違えば、足が浮いたか滑った）、左右の足の位置関係の変化
  （10 mm を超えたら足を踏み替えた。そのあとの値は当てにならない）
- エレベーター（`contest_agent`）は、環境変数 `J1GEN_BODY_LOG` にフォルダを渡すと毎周期の関節角・IMU・段階を記録する
  （基準は ▲▼ を探していた最後の周期）。Yada さんの `run_dds.py` には手を入れていない
- 同じフォルダに、押す前と点灯の確認のときの画像（`lit_00_before.png`、`lit_01_check.png` …、枠と明るさを描いた
  `*_annotated.jpg`、`lit_log.json`）も残る。手が枠に被さって点灯の確認が狂っていないかを、あとで画像で見るため
  （確認のときは指先がボタンの 6 cm 手前にある）。コントローラーの `on_snapshot` が口で、記録に失敗しても押す動きは止めない。
  シミュレーションの評価は点灯した時点で終わるので、確認の画像は押し直した・点灯が遅れたときだけ増える
- 点灯の判定の回帰テスト（`tests/test_lit_check.py`）は、ラボのエレベーターの写真の切り出し（`tests/data/lab_elevator/`）で、
  明るい部分の値が点灯（224〜230）と消灯（67〜131）で分かれることと、金属面が反射で白い消灯（190〜215）が現状の限界であることを確かめる

```bash
P=G1_HuggingFace/venv/bin/python
$P Button_Press/J1-gen/real/body_motion.py _local/button_press/runs/<日時>_<ラベル>        # ボトル（あとから読み直す）
J1GEN_BODY_LOG=_local/button_press/body_log $P Button_Press/Yada/contest/run_dds.py --agent Button_Press/J1-gen/contest_agent ...
$P Button_Press/J1-gen/real/body_motion.py _local/button_press/body_log/<日時>_<up|down>   # エレベーター
```

- シミュレーションでの値: ボトル（腰を固定）は 0 mm。評価環境は脚を PD で保つだけなので、腕の反動で 0.2 mm・0.07°。
  評価環境の「体の揺れ」は腰を直接動かしているので、IMU には出るが脚の FK には出ない（実機では両方に出るはず）

### ループバックの模擬ロボット

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/sim_robot_server.py
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/press_bottle.py --path arm_sdk --network-interface lo --camera-config camera_sim.yaml --detector color --sim-scene-obstacles --execute
```

- MuJoCo の G1 を、実機と同じ DDS のトピックと ZMQ のカメラで見せる。実機用のスクリプトを、当日と同じコマンドのまま試せる。
- **実機と混ざらないように固定している**: DDS は口 `lo` と domain 1（G1 は domain 0）、カメラは 127.0.0.1 だけ。
  lowstate の `reserve[0]` に目印を入れ、実機用のスクリプトは開始時に口と相手（実機 / 模擬ロボット）を表示し、食い違えば中止する。
- `--fault` でわざと故障を起こせる: `ignore_arm_sdk`（arm_sdk が効かない）/ `motor_mode0`（ゼロトルク）/
  `lowstate_dropout`（途切れる）/ `mode_machine`（機体構成の違い）/ `waist_sag`（腰が倒れる）。
  それぞれ決まった終了コード（3 / 4 / 4 / 4 / 5）で安全に止まることを `tests/test_sim_robot.py` で確かめている。
- 模擬ロボットは domain 1 なので、domain 0 で動く g1-starter-kit の `mode_check.py` / `armsdk_probe.py` は試せない
  （この 2 つは `--help` と、使うモジュールが venv で読み込めることだけを 2026-09-29 に確かめた）。

### リハーサル

```bash
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/rehearsal.py
```

当日手順書の段階0〜6と、故障の場面（arm_sdk が効かない → プランB、ゼロトルク、機体構成の違い、押し込み中の
lowstate の途切れ、押し込み中に腰が倒れる）を、実機用のスクリプトで通しで実行する（約 8 分）。結果は
`_local/button_press/rehearsal/<日時>/report.md`（各コマンドの所要時間・終了コード・期待どおりか、段階ごとの合計）。

2026-09-29 の結果: 26 / 26 件が期待どおり。1 回目は、経由の姿勢（肩ピッチ 17.2°）で手を上げる途中に机の 4〜8 mm まで
近づき、計画の段階で拒否された（何も送っていない）ので、肩ピッチを 28.6° にした（180 mm 以上離れる）。

## ボトルを倒す（`real/knock_bottle.py`、`sim/knock_bottle_sim.py`）

机の手前の縁に置いた 500 mL の PET ボトル（水 50 mL ほど）を、立った状態の右腕で前方（机の奥）へ押して倒す。
流れは「全体をつなぐスクリプト」と同じ（`common/pipeline.py`）で、設定の上書き `configs/knock.yaml`（`--overlay`）だけが違う。

```bash
# シミュレーション（倒れるボトルは configs/sim_scene_knock.yaml）
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/knock_bottle_sim.py [--view] [--water-ml 100 --friction 0.3]
# 条件の掃引（深さ・押す高さ・水・摩擦。1 回約 10 秒）
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/knock_sweep.py --depth-mm 100 125 --raise-mm 40 58 --friction 0.3 0.5
# 実機（dry-run が既定。送るときは --execute）
G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/knock_bottle.py --path arm_sdk --gravity-scale 0.5
```

- 実機日の手順書: [real/KNOCK_PROCEDURE.md](real/KNOCK_PROCEDURE.md)。
- **倒れたかの自動判定**（`common/tip_check.py`）: 頭カメラの深度で、ボトルの**いちばん高い点の高さ**を押す前と押したあとで比べる
  （約 6 cm 以上下がれば倒れた、3 cm 以内なら立っている、間は傾いている）。終了コードは 倒れた 0 / 倒れなかった 20 / 判定できない 21。
  枠の縦横比は使えない（ボトルは机の奥へ倒れるので、寝ても画像の上では縦に長いまま。MuJoCo の画像で確かめた）。
  シミュレーションでは、MuJoCo のボトルの傾き（最後に 60° 以上）が正解で、自動判定と食い違わないかを `knock_sweep.py` の最後の列で見る
  （2026-10-22、深さ・高さ・水・摩擦を振った 16 通りで、ボトルが見えなくなった 2 通り（机から落ちた。判定できない）を除いて一致）。
  実機の YOLO が寝たボトルを検出できるか、PET の深度が取れるかは未確認（取れなければ「判定できない」で、目で見る）。dry-run では判定しない。
- 手の動かし方: 測った面の点から `pipeline.target.depth.offset_m` だけ上を、水平に前方へ `press_depth_m` 押す
  （`--target-offset X Y Z`、`--depth-mm`、`--speed-mm-s` で上書き）。押し込みの上限は腕が届く約 13 cm。

### 分かったこと（2026-10-22、MuJoCo、机の手前の縁のボトル、右腕）

- **倒れきる角度は、重心の高さで決まる。** 水 50 mL は底にたまり、重心が底から約 3 cm まで下がる。倒れきる角度は約 45°（空のボトルは約 20°）。
  手先は 9 cm 以上進める必要がある。最初の見積もり（22°・5 cm）は誤りで、押し込み 8 cm では最大 44° まで傾いて戻った。
- **押す点が低いと滑る。** 底から `r/μ`（半径 3.25 cm、摩擦 0.4 で約 8 cm）より低いと、倒れずに机の上を滑る。
  ハンドの当たり判定（手を包む箱）は、中指の先より約 4.5 cm 下まで広がるため、指先を胴の中ほどに合わせると滑った。
- **窓が狭い。** 押し込み 125 mm・押す点の上げ 58 mm（既定）は、摩擦 0.3〜1.0、水 25〜100 mL ですべて倒れた。
  一方、摩擦 0.25 では滑り、上げが 65 mm だと届かず倒れない（高く押すほど、倒すのに必要な押し込みが増える）。± 5 mm で外れる。
- **滑り止めのマット（摩擦 0.8 以上）を敷くと窓が広がる**: 上げ 0〜40 mm・押し込み 100 mm で倒れた。実機ではこれを勧める。
- 押し込みが 14 cm 以上は、腕が伸びきって計画できない。
- ⚠️ 実機のハンドの当たる高さ・机の摩擦は未確認。dry-run と実機で、上げの量を調整する。
  押し込みを止める条件（`contact_stop`）は残してある（固い物に当たったときの安全装置。ボトルでは遅れが出ない）。

## エレベーターの呼びボタン（Yada の評価環境）

Yada さんの評価環境（`Button_Press/Yada/contest`。本番に似た乗り場の黒い柱の ▲▼）で、J1-gen の部品で押す。
使い方は評価環境の `Button_Press/Yada/contest/GUIDE.md` と同じ（B. Python の API）。

```bash
P=G1_HuggingFace/venv/bin/python
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seeds smoke
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seeds practice --set realistic --report
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seed 1 --view   # 画面で見る（計算が間に合わずスローモーションになる）
$P Button_Press/J1-gen/sim/record_trial_video.py --seed 1          # 実際の速さの動画（_local/button_press_yada/videos/）
```

- 準備: `pip install scipy`（評価環境の見本が使う）。J1-gen の側は scipy を使わない
- 中身は `common/elevator_press.py`（`contest_agent/agent.py` は評価環境の決まりにつなぐだけ）。設定は `configs/elevator.yaml`
- シミュレーターの正解の値（ボタンの座標など）は使わない。観測（RGB・深度・内部パラメータ・関節の角度・IMU）と、
  機体のモデル（`robot.yaml`）だけを使う

### 流れ

1. **見つける**（`common/elevator_buttons.py`）
   - 画像で候補を探す: まわりより明るい、色の無い丸（classic）か、自分で学習した YOLO（yolo）
   - 丸の中の黒い記号から ▲ か ▼ かを読む（三角形の重心は底辺の側に寄る）。縁の影を避け、内側だけを見る
   - 深度で pelvis 座標にし、まわりの柱の面に平面を当てはめて押す向きを求める
   - 柱ごとに上から並べ、**いちばん上の 2 つ**の間隔（5〜12 cm）と記号（上が ▲、下が ▼）が合うときだけ採用する。
     ▲ が画面の外に切れていると、▼ と車いす用の ▲（20 cm 以上離れている）が上の 2 つになるが、間隔が合わないので押さない
   - 新しいフレーム 3 つで、位置のばらつき 5 mm 以内・向き 5° 以内を確かめる
2. **計画**（`PressPlanner`。ボトルと同じ）: IK、腕と体の衝突、作業空間の箱、押し込みの直線
   - 指は押す向きから左・上に傾ける（右下から斜めに伸ばす）。傾きの候補を順に試す
   - **柱・壁との衝突**: 柱の面の向きに回した箱を置き、開始 → 手前の姿勢の移動で腕が当たらないかを確かめる。
     当たるなら、壁から離れた右側の経由点を通る（指の向きを手前の姿勢とそろえて解く）
   - **立つ位置**: 勧める範囲は **pelvis から柱の面まで 29〜40 cm、▲▼ の中点が体の中心から左右 6 cm 以内**
     （`elevator.yaml` の `press.stand`）。範囲の外なら「柱まで 43.2 cm と遠すぎる。あと 4 cm 以上近づく」のように、
     どちらへ何 cm 動けばよいかを出す。計画できなければ中止の理由に添え、計画できれば注意（⚠️）だけ出して押す
     （範囲の外でも押せることは多く、推定の距離は realistic で最大 19 mm 手前にずれるので、これだけでは止めない）。
     ▲▼ が見つからずに中止するときも、立つ位置を確かめるよう出す
   - 範囲の決め方（2026-10-15）:
     - 計画だけの網羅（柱まで 18〜58 cm・▲ の高さ 0.95〜1.15 m・左右 ±10 cm・▲▼、3444 条件）: 評価環境の
       高さ（1.03〜1.10 m）・左右 ±3 cm では、29〜46 cm で全条件の計画が立つ。近い側の失敗は「腕が体に当たる・
       肘の範囲」、遠い側は「押す途中で関節が速く回りすぎる（腕が伸びきる特異姿勢の近く）」
     - 評価環境（basic）で押す（点灯で終わらせず、腕を戻すまで）: 29 cm と 40 cm は、高さ 1.03・1.10 m、
       左右 ±3・±6 cm、▲▼ のすべてで点灯、接触 0 N。27・28 cm では高い ▲ が計画できないことがある。
       **41 cm 以上では、高い ▲（1.10 m）が頭カメラの上に切れて見つからない**（▼ と車いす用の ▲ が上の 2 つになり、
       間隔 22.5 cm で断る）。46 cm 以上では低い ▼ の計画も立たない
     - 近いほど位置の推定の横ずれ（カメラの角度のずれ × 距離）は小さい。迷ったら 32 cm 前後に立つ
   - 遠いときに自分で近づく（歩く）ことはしない。J1-gen は足を動かさない（`--no-loco`）。人がリモコンで近づける
3. **押す**: 関節空間で手前の姿勢へ → 止まって指先のずれを補正（FK で積分）→ 押す向きに直線で押し込む → 戻る
   - 腕の PD が弱く（kp 60）重力を補償しないので、重力のトルク ÷ kp だけ目標をずらす（`common/arm/gravity.py`、IMU の向きを使う）
   - **指先が進まなくなったら、そこで押し込みを止める**: 指令した指先（目標の角度の FK）が実際の指先より押す向きに
     5 mm 以上（押し込みを始めたときの遅れからの増え方）先に出たら、ボタンが底に着いたとみなし、実際の指先の
     「押し込みを始めたときの遅れ + 2 mm」先を押したまま保持する（押し直しのときは 8 mm。腕は目標からその遅れの分ずれて
     落ち着くので、遅れを足さないと指先がボタンから離れる。2026-10-22）。
     関節の角度だけで判断するので、実機でもそのまま使える（`elevator.yaml` の `press.contact_stop`）
     - 以前は面の 15 mm 奥まで押し続けたので、底に着いたあとも遅れ（＝押す力）が最大 26 mm まで増え、ボタンの
       中心からずれた位置を押していると指先が横に滑り、柱の穴の縁を押した。点灯で終わらせない診断
       （realistic 20 配置、2026-10-15）で、縁への力は最大 21.6 N（seed 0）・19.3 N（seed 6）・9.5 N（seed 7）
       → 止めるようにして最大 5.0 N（seed 0）、ほかは 1.9 N 以下。20 配置すべて点灯
     - 評価（classic、練習用の 20 試行）は変わらない: basic 20/20・8.5 秒、realistic 20/20・9.07 秒、
       模擬 G1（DDS）20/20・8.72 秒、すべて 0 N
     - 評価環境の触れる前の遅れは最大 1.9 mm。実機で腕の重力の補正がずれていると、触れる前から遅れが出て早く止まる
       おそれがある（ログの「指先が進まなくなった（遅れ … mm）」で確かめる）。→ 2026-10-21 のボトルの実機で起きた
       （押す前から 4.5 mm 遅れていて触れる前に止まった）ので、10-22 から遅れの増え方で見る
     - 2026-10-22（増え方で止める・保持に遅れを足す・点灯の第 2 の条件を入れたあと、classic、練習用の 20 試行）:
       realistic 20/20・9.08 秒、basic 20/20・8.48 秒、模擬 G1（DDS）20/20・8.71 秒、すべて 0 N。点灯で終わらせない
       診断（realistic・basic 各 20 配置）で点灯の判定の誤り 0 回、壁・柱への力は最大 5.64 N（seed 0。保持に遅れを足す前は
       5.04 N）・2.34 N（seed 6。前は 1.92 N）、ほかは 0 N。YOLO はボタンの重みがこの PC に無く未確認
     - 残る課題: 触れた時点で、指先がボタンの中心から最大 16.5 mm ずれている（カメラの取り付けの角度のずれによる
       位置の推定の横ずれ）。seed 6（半径 19.7 mm で 16.4 mm ずれ）は、止めても縁に軽く触れる
4. **確かめる**: 押したボタンの明るい部分が明るくなったか（点灯したか）を見る。点灯していなければ 1 cm 深くして 1 回だけ押し直す
   - 見る値: 枠に内接する楕円を 1.15 倍に広げた範囲の、明るい側から 90% の値（`elevator.yaml` の `press.lit_check`）。
     見つけたとき（look の画像）と、押したあと手前の姿勢へ戻って 0.3 秒待った最初の画像 1 枚で比べ、+30 以上で点灯
   - 以前は枠全体の平均（+25）で見ていた。ラボのエレベーター（2026-10-15、スマホの写真）は**縁の細い輪だけが光る**ので、
     枠全体の平均では +21〜+31 しか上がらず、ぎりぎりだった。明るい部分なら +51〜+95
   - 評価環境（点灯で終わらせない診断、basic・realistic × classic・YOLO の 80 試行）: 点灯したとき +39〜+55、
     点灯していないとき 1 以下。YOLO でも、押したあとに枠を見つけ直して判定できた
   - ⚠️ ▲ を押したあと、手前の姿勢の手が下の ▼ の前に重なり、▼ の明るい部分が最大 +163 上がった（評価環境の手は
     明るい色）。押したボタンの前に手が重なると、点灯していなくても「点灯した」と見るおそれがある。評価環境で
     押したボタンが点灯していなかった確認は 4 回だけで、どれも 1 以下（手で上がった様子は無い）。実機の手の色でも変わる
5. **腕を戻す**: 来た経路（手前の姿勢 →（経由の姿勢 →）始めの姿勢）を逆にたどって戻してから終わる
   - 実機・模擬 G1 では、エージェントが終わると腕の weight が 0 に戻り、腕が柱の 6 cm 手前から元の姿勢へ動くため
   - 来た経路は、動く前に体・柱との衝突を確かめてある。押す向きと逆にまっすぐ離れる方法は、肘がたたまれて
     右の手首が胴体に入った（柱まで 31 cm で、面から 10 cm 以上離れると）ので使わない
   - 評価環境は点灯した瞬間に終わるので、評価の結果（成功率・時間）は変わらない。点灯で終わらせない診断
     （basic 2・realistic 20 配置、2026-10-14）で、全配置が戻り終わり（最長 23.3 秒）、戻る間の壁への接触は 0 N、
     体・柱との 1 cm 以内の接近も無かった

### 確かめたこと（2026-10-07、MuJoCo、練習用の 20 試行）

| 評価セット | 成功 | 点灯までの平均 | 壁・柱との接触力 |
|---|---|---|---|
| basic | 20 / 20 | 8.5 秒 | すべて 0 N |
| realistic | 20 / 20 | 9.0 秒 | すべて 0 N |
| 模擬 G1（DDS + ZMQ、実機と同じ口。`evaluate_dds.py` + `run_dds.py`） | 20 / 20 | 8.7 秒 | すべて 0 N |

参考: 評価環境の見本のエージェントは basic 100%、realistic 95%（GUIDE.md）。seed 0 の ▲ では、見本は柱の横の壁に 40 N で当たった。

途中で直したこと:
- ▲▼ の記号を読み違えた（縁の影）→ ボタンの内側だけを見る
- seed 1 の ▼: まっすぐ手前の姿勢へ動くと、手首が柱の 1.9 mm 手前を通った → 経由点（指の向きもそろえて解く）
- realistic で 20 試行のうち 13 試行が動く前に中止: 柱の面の向きが 3° ほど傾いて推定され、軸にそろえた壁の箱が
  4 cm 手前に出た → 箱を面の向きに回して置く（`common/collision.py` の障害物に `x_axis` を足した）
- realistic の seed 8: 柱が近く面が斜めで、手前の姿勢で手首が胴体に 9.9 mm まで近づいた → 指の傾きの候補を順に試す

- 模擬 G1（DDS）で 2 試行とも「▲▼ を見つけられない」で中止: 評価環境の実機用の口（`contest/robots/real_g1.py`）は
  画像を撮った時刻 `image_t` を入れず常に 0 なので、最初の 1 枚しか新しい画像とみなさず、3 フレームそろわなかった
  → 時刻が無いときは `image_interval_s`（0.1 秒）ごとに新しい画像とみなす。同じ口は IMU の向きも入れない（腰が固定の
  模擬 G1 では影響なし。実機では重力の向きの計算に IMU が使われない点に注意）

模擬 G1 で動かす（実機と同じ口。1 試行に実時間で約 18 秒）:

```bash
P=G1_HuggingFace/venv/bin/python
$P Button_Press/Yada/contest/evaluate_dds.py --seeds practice \
    --client "$P Button_Press/Yada/contest/run_dds.py --agent Button_Press/J1-gen/contest_agent"
```

まだ確かめていないこと: Isaac Sim、練習用以外の種、実機。

### ボタンの YOLO

公開されているエレベーターボタンの YOLO（Kshaw17-web/End-to-end-elevator-button-detection）は使わない。
リポジトリにライセンスの記載が無く、使用の許可が無いため（2026-10-07 確認。GitHub の API でも license が null。
学習データの Roboflow のデータセット `acc-stwam/elevator-button-jinxe-qb0gs` は CC BY 4.0）。
代わりに、評価環境の画像から自分で学習する（元にするのはリポジトリの `yolo26n.pt`。Ultralytics、AGPL-3.0）。

```bash
$P Button_Press/J1-gen/sim/make_button_dataset.py      # 学習データ（正解の枠は MuJoCo のセグメンテーションから自動）
$P Button_Press/J1-gen/sim/train_button_yolo.py        # 学習 → _local/button_press/button_yolo/weights/best.pt
J1GEN_BUTTON_DETECTOR=yolo $P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seeds practice
```

- クラスは `up`（▲）と `down`（▼）。一般用か車いす用かは、YOLO ではなく位置の並びで決める
- 学習データは、試行の種ごとの乗り場の違い、半分は realistic、腰の角度の違い、3 枚に 1 枚は右腕が写り込んだ画像
- ⚠️ シミュレーションの画像だけで学習した重みは、本物のエレベーターではそのままでは使えない見込み。
  実機の日に `real/record.py` で本物のボタンを撮り、`real/check_button_photos.py` で確かめる。見つけられなければ、
  手でラベル（`up` / `down` の枠）を付けて学習データに足し、学習し直す

確かめたこと（2026-10-07）:
- 学習データ: `--seeds 300 --start 1000`（練習用の種 0〜19 と重ならない）で 300 枚（学習 270、検証 30）、枠 1,174 個。
  このノート PC（12 コア）で 2 分 48 秒。手に隠れたボタンの切れ端（幅か高さが 8 px 未満）の枠が 0 個であることを数えて確かめた
- YOLO の結果をボタンの候補に変える部分: テスト（偽の検出器）。重みの読み込みと推論が動くことも確かめた
- CPU（このノート PC）では 1 エポック 3〜5 分（40 エポックで 2〜3 時間）かかるので、GPU で学習した（次）

#### GPU のある PC で学習する（2026-10-12、RTX 5060 Ti）

班員の PC（Ubuntu 22.04、Python 3.10）に SSH して学習した。その PC で変えたのは、作業用のフォルダと venv だけ。

```bash
# このノート PC: 学習に要るものだけをまとめる（73 MB）
tar czf button_yolo_bundle.tgz Button_Press/J1-gen/sim/train_button_yolo.py Button_Press/J1-gen/yolo26n.pt \
    _local/button_press/button_yolo/dataset
# GPU の PC: 展開して、data.yaml の path をその PC の場所に直し、venv に入れる（約 7 GB）
python3 -m venv ~/yolo_env && ~/yolo_env/bin/pip install torch torchvision --index-url https://download.pytorch.org/whl/cu128
~/yolo_env/bin/pip install ultralytics
# 学習（GPU の番号は下の注意を参照）
CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES=1 ~/yolo_env/bin/python \
    Button_Press/J1-gen/sim/train_button_yolo.py --device 0
# できた _local/button_press/button_yolo/weights/best.pt を、このノート PC の同じ場所に写す
```

- ⚠️ **GPU の番号が nvidia-smi と PyTorch で逆だった**: nvidia-smi では 0 = GTX 1660 Ti、1 = RTX 5060 Ti だが、
  PyTorch の既定（速い順）では 0 = 5060 Ti。`CUDA_DEVICE_ORDER=PCI_BUS_ID` で nvidia-smi と同じ番号にそろえてから選ぶ
- RTX 50 系（Blackwell）には CUDA 12.8 版の torch（`cu128`）が要る
- WSL2 からはその PC に届かなかったので、ファイルの受け渡し（scp）は Windows の PowerShell から
  `\\wsl.localhost\Ubuntu\home\...` のパスで行った
- 後片付け: `rm -rf ~/yolo_env ~/G1_Hackason ~/.cache/pip`

結果（40 エポック、batch 16）:
- **1.4 分**（1 エポック約 1.2 秒）。GPU のメモリは 2.7 GB
- 検証用（30 枚、枠 119 個）: mAP50 **0.995**、mAP50-95 0.981、適合率 0.998、再現率 1.0。▲▼ の取り違え・見落とし 0
- 学習・検証の損失はどちらも最後まで下がっていて、過学習の様子は無い
- 確信度が低めだったのは、小さい車いす用の ▼ と、手の近くのボタン（0.7〜0.9）
- ⚠️ この検証用の 30 枚は、分け方の誤りで **basic だけ**だった（`make_button_dataset.py` を直した。
  学習データを作り直して学習し直せば、realistic の画像での成績が分かる）

評価環境（`J1GEN_BUTTON_DETECTOR=yolo`、2026-10-12）:

| 評価セット | 成功 | 点灯までの平均 | 壁・柱との接触力 |
|---|---|---|---|
| smoke | 2 / 2 | — | 0 N |
| realistic（練習用の 20 試行） | 20 / 20 | 9.05 秒（classic 9.04 秒） | すべて 0 N |
| basic（練習用の 20 試行） | 20 / 20 | 8.49 秒（classic 8.5 秒） | すべて 0 N |
| 模擬 G1（DDS + ZMQ、実機と同じ口） | 20 / 20 | 10.19 秒（classic 8.71 秒） | すべて 0 N |

- 試行ごとの時間も classic とほぼ同じ。評価環境では、どちらの見つけ方でも結果は変わらない
- 模擬 G1 は実際の時刻で動くので、YOLO の計算の時間がそのまま遅れになる。このノート PC の CPU では 1 枚 約 0.17 秒
  （classic は 0.02 秒）で、見つけるのに新しい画像が 3 枚要る。さらに**最初の 1 回だけ 1.1 秒**かかり、
  はじめは平均 11.17 秒だった → 読み込んだときに 1 回空の画像で推論しておく（`YoloButtonDetector`）ようにして 10.19 秒
- ⚠️ **YOLO で並列に評価すると、メモリが足りなくなる**: 1 試行のプロセスが約 2.5 GB（classic は約 1.7 GB）。
  このノート PC（WSL2、メモリ 15 GB + スワップ 4 GB）で `--workers 11` にすると WSL2 ごと落ちた。
  `--workers 4` なら落ちず、速かった（16 分 → 5 分）:

```bash
J1GEN_BUTTON_DETECTOR=yolo OMP_NUM_THREADS=2 $P Button_Press/Yada/contest/evaluate.py \
    --agent Button_Press/J1-gen/contest_agent --seeds practice --set realistic --report --workers 4
```

#### 撮った写真で確かめる（`real/check_button_photos.py`）

実機の日に撮った写真で、classic と YOLO がボタンを見つけられるかを並べて比べる。シミュレーションの画像だけで
学習した重みが本物のボタンで使えるかは、これで確かめる。

```bash
$P Button_Press/J1-gen/real/check_button_photos.py _local/button_press/recordings/<日時>_button   # 実機の収録
$P Button_Press/J1-gen/real/check_button_photos.py ~/photos/elevator/                           # ただの写真のフォルダ
```

- 出力は `_local/button_press/button_check/<日時>/`: 1 枚ごとに左が classic・右が YOLO の画像（緑 = ▲、赤 = ▼、
  黄 = 記号を読めない）と、`summary.csv`
- 実機の収録（`real/record.py`）で深度と腰の角度があれば、▲▼ の 1 組を選ぶ（押す相手を決める）ところまで試す
- 正解のラベル（YOLO の形式。`images/` と同じ並びの `labels/` の .txt）があれば、正解と比べて数える
  （記号まで正しい・▲▼ の取り違え・見落とし・余分）。手でラベルを付けた実機の写真でも、同じように数えられる
- `--every N` で N 枚に 1 枚、`--no-yolo` で classic だけ、`--confidence` で YOLO の確信度の下限を変える

シミュレーションの検証用の 30 枚（枠 119 個。車いす用も含む）で試した結果（2026-10-12、9 秒）:

| | 記号まで正しい | ▲▼ の取り違え・読めない | 見落とし | 余分 |
|---|---|---|---|---|
| classic | 97（82%） | 13 | 9 | 25 |
| YOLO | 119（100%） | 0 | 0 | 1 |

classic の余分は、ほとんどが白い腕（明るく色が無い）。小さい車いす用のボタンの記号も読めないことが多い。
それでも評価環境で classic が 20 / 20 なのは、深度での大きさ・柱ごとの並び・3 フレームの確認で、
一般用の ▲▼ の 1 組だけが残るため。

#### 実機の写真を学習データに足す

撮った写真で YOLO がボタンを見つけられなかったときに、その写真に正解の枠を付けて、学習し直す。
ラベルを付ける道具はこのリポジトリのもの（OpenCV の画面。追加のインストールは要らない）。

```bash
# 1. 枠を付ける（今の YOLO の結果が下書きとして入っているので、直すだけ。1 秒ごとの収録なら 3 枚に 1 枚で十分）
$P Button_Press/J1-gen/real/label_button_photos.py _local/button_press/recordings/<日時>_button_hall --every 3
# 2. シミュレーションの学習データと混ぜる（実機の 5 枚に 1 枚を検証用、学習用は 3 回ずつ入れる）
$P Button_Press/J1-gen/sim/merge_button_datasets.py _local/button_press/button_real/<日時>_button_hall
# 3. GPU のある PC で学習する（「GPU のある PC で学習する」の手順で、dataset の代わりに dataset_mixed を持っていく）
CUDA_DEVICE_ORDER=PCI_BUS_ID CUDA_VISIBLE_DEVICES=1 ~/yolo_env/bin/python Button_Press/J1-gen/sim/train_button_yolo.py \
    --device 0 --data _local/button_press/button_yolo/dataset_mixed/data.yaml \
    --out _local/button_press/button_yolo/weights/best_mixed.pt
# 4. 実機の検証用の写真だけで確かめる（学習に使っていない写真）
$P Button_Press/J1-gen/real/check_button_photos.py _local/button_press/button_yolo/dataset_mixed/images/val_real
```

- 操作: 左ドラッグで枠を足す（1 = up ▲、2 = down ▼）、右クリックで消す、c で ▲▼ を入れ替える、Space で保存して次へ、
  x でその写真を使わない、b で戻る、q でやめる（続きから再開できる）
- 枠は、一般用も車いす用も ▲▼ のボタンすべてに付ける。半分以上隠れた・画面の端で切れたボタンには付けない。
  **ボタンが写っていない写真も、枠 0 個のまま保存する**（見間違いを減らす）
- 4. で良ければ、`configs/elevator.yaml` の `yolo.weights` を `best_mixed.pt` にする。評価環境でも 20 / 20 のままかを確かめる
  （`J1GEN_BUTTON_DETECTOR=yolo` で basic・realistic。シミュレーションのボタンを見つけられなくなっていないか）
- 実機の検証用は、1 秒ごとの収録の中から選ぶので、学習用とよく似た写真になる（成績は甘めに出る）。
  別の日・別の乗り場の写真があれば、それで確かめるほうが確か
- ⚠️ 撮った写真には、建物の中が写る。外のサービス（Roboflow など）に上げない。GPU のある PC に持っていくときも、
  終わったら消す

試したこと（2026-10-12）: 実機の写真はまだ無いので、シミュレーションの写真で、枠の下書き・保存・読み直し・
混ぜ方をテストで確かめた（画面の操作は未確認）。
