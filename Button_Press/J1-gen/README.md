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
- `sim/` — MuJoCo での検証
  - `fetch_models.sh` — 公式モデル（unitree_ros の `g1_29dof_rev_1_0`）を取得する
  - `move_arm_sim.py` — 腕を指定の関節角へ動かして戻す
  - `press_sim.py` — 指定した点（pelvis 座標）を押す
- `real/` — 実機用
  - `move_arm_real.py` — 同じことを実機で行う（既定は dry-run。送るには `--execute`）
  - `REAL_DAY_PROCEDURE.md` — 実機日の手順書（下書き。タスク7で完成させる）
  - `depth_server/` — PC2 で動かす深度付きカメラサーバ（`rgbd_server.py`）と、その使い方（`README.md`）
  - `probe_rgbd.py` — 深度付きストリームが届くかを確かめる（ラボ PC で）
- `configs/` — 設定ファイル。当日はコードを編集せず、ここだけを変える
  - `robot.yaml` — モデルのパス、頭カメラの取り付け位置
  - `arm.yaml` — 経路（sim / arm_sdk / lowcmd）、NIC 名、ゲイン、安全の上限、重力補償
  - `press.yaml` — IK、衝突の確認、押し込み（手前の距離・深さ・速さ）、作業空間の箱
  - `depth_server.yaml` — 深度付きカメラサーバ（PC2）: RealSense のシリアル番号、ポート（深度付き 5556 / RGB 互換 5555）
  - `camera.yaml` — 受信側（ラボ PC）の接続先とポート
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
| 4 | ボトル検出 → 3D 座標 → pelvis 座標 | これから |
| 5 | 収録ツール | これから |
| 6 | 全体をつなぐスクリプト | これから |
| 7 | 実機日用のツールと手順 | これから |

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

- 手先の点は、ハンドのメッシュで最も前に出ている点（`robot.yaml` の `end_effector`。実機日に確認）。
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
- PC2 はインターネットにつながっていない可能性が高いので、pyrealsense2（Python 3.8 / 3.10 / 3.12 用）と
  libusb-1.0 の .deb を事前にダウンロードして持っていく（`real/depth_server/fetch_offline_packages.sh` →
  PC2 で `install_offline.sh`。sudo は使わず、libusb は中身を取り出すだけ）。
- 既存の `run_g1_server.py --camera` は、RealSense を pyrealsense2 ではなく OpenCV で `/dev/video4` として開いている
  （lerobot の `ImageServer`、2026-09-28 に GitHub の main で確認）。
