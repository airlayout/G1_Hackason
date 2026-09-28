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
- `sim/` — MuJoCo での検証
  - `fetch_models.sh` — 公式モデル（unitree_ros の `g1_29dof_rev_1_0`）を取得する
- `real/` — 実機用（これから）
- `configs/` — 設定ファイル。当日はコードを編集せず、ここだけを変える
  - `robot.yaml` — モデルのパス、頭カメラの取り付け位置
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
| 1 | 腕の指令部分（sim / arm_sdk / lowcmd） | これから |
| 2 | FK / IK と軌道生成 | これから |
| 3 | 深度付きの配信サーバ（PC2） | これから |
| 4 | ボトル検出 → 3D 座標 → pelvis 座標 | これから |
| 5 | 収録ツール | これから |
| 6 | 全体をつなぐスクリプト | これから |
| 7 | 実機日用のツールと手順 | これから |
