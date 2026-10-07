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
  - `sim_scene.py` / `sim_camera.py` — MuJoCo の机とボトル、頭カメラの RGB + 深度、セグメンテーションの検出器
  - `recording.py` / `recorder.py` — 収録（RGB・深度・lowstate を時刻付きで保存）と再生
  - `realday.py` — 実機日用のツールの部品（指先の FK、画像への投影、教えた姿勢、支持の確認）
  - `pipeline.py` / `pipeline_cli.py` — 全体をつなぐ流れ（検出 → 目標 → IK → 手前の姿勢 → 押し込み → 戻る）
  - `run_logger.py` — 1 回分の記録（RGB・深度・lowstate・検出・IK・送った指令。dry-run でも）
  - `post_press.py` — 押したあとの確認の差し込み口（ボタン版で「点灯しなければ深く押し直す」に使う）
  - `elevator_buttons.py` — エレベーターの呼びボタン（一般用の ▲▼）を見つける。画像の候補 → ▲▼ の記号 → 3D → 柱の面の向き → 上の 2 つ
  - `elevator_press.py` — 呼びボタンを押す制御（50 Hz で 1 歩ずつ）。Yada の評価環境のエージェントの中身
- `sim/` — MuJoCo での検証
  - `fetch_models.sh` — 公式モデル（unitree_ros の `g1_29dof_rev_1_0`）を取得する
  - `move_arm_sim.py` — 腕を指定の関節角へ動かして戻す
  - `press_sim.py` — 指定した点（pelvis 座標）を押す
  - `locate_sim.py` — 頭カメラの画像と深度からボトルの位置を求め、正解と比べる
  - `press_bottle_sim.py` — 全体をつなぐスクリプト（MuJoCo、同じプロセス）
  - `sim_robot_server.py` — ループバックの模擬ロボット（実機と同じ DDS とカメラ。故障をわざと起こせる）
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

## エレベーターの呼びボタン（Yada の評価環境）

Yada さんの評価環境（`Button_Press/Yada/contest`。本番に似た乗り場の黒い柱の ▲▼）で、J1-gen の部品で押す。
使い方は評価環境の `Button_Press/Yada/contest/GUIDE.md` と同じ（B. Python の API）。

```bash
P=G1_HuggingFace/venv/bin/python
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seeds smoke
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seeds practice --set realistic --report
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/J1-gen/contest_agent --seed 1 --view   # 画面で見る
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
3. **押す**: 関節空間で手前の姿勢へ → 止まって指先のずれを補正（FK で積分）→ 押す向きに直線で押し込む → 戻る
   - 腕の PD が弱く（kp 60）重力を補償しないので、重力のトルク ÷ kp だけ目標をずらす（`common/arm/gravity.py`、IMU の向きを使う）
4. **確かめる**: 押したボタンの枠が明るくなったか（点灯したか）を見る。点灯していなければ 1 cm 深くして 1 回だけ押し直す

### 確かめたこと（2026-10-07、MuJoCo、練習用の 20 試行）

| 評価セット | 成功 | 点灯までの平均 | 壁・柱との接触力 |
|---|---|---|---|
| basic | 20 / 20 | 8.5 秒 | すべて 0 N |
| realistic | 20 / 20 | 9.0 秒 | すべて 0 N |

参考: 評価環境の見本のエージェントは basic 100%、realistic 95%（GUIDE.md）。seed 0 の ▲ では、見本は柱の横の壁に 40 N で当たった。

途中で直したこと:
- ▲▼ の記号を読み違えた（縁の影）→ ボタンの内側だけを見る
- seed 1 の ▼: まっすぐ手前の姿勢へ動くと、手首が柱の 1.9 mm 手前を通った → 経由点（指の向きもそろえて解く）
- realistic で 20 試行のうち 13 試行が動く前に中止: 柱の面の向きが 3° ほど傾いて推定され、軸にそろえた壁の箱が
  4 cm 手前に出た → 箱を面の向きに回して置く（`common/collision.py` の障害物に `x_axis` を足した）
- realistic の seed 8: 柱が近く面が斜めで、手前の姿勢で手首が胴体に 9.9 mm まで近づいた → 指の傾きの候補を順に試す

まだ確かめていないこと: 模擬 G1（DDS、A の使い方）、Isaac Sim、練習用以外の種、実機。

