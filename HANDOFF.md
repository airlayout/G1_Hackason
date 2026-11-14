# G1 プロジェクト開発引き継ぎ

2026-11-13（JST）更新。Unitree G1 のデジタルツイン、実機の認識・移動・操作を扱うリポジトリ。直近の作業は、頭部 RGB-D と実測関節角を使ったエレベーターの呼びボタン押下。

## 作業の再開

- 作業コピーは `/home/kanto/dev/G1_Hackason`。引き継ぎ作成前は `push-button` の `2d80adb` で、作業ツリーはクリーン。手元の `origin/push-button` と同じコミットだった。手元の `origin/main` は `27fde70` で、このブランチはそこから44コミット先。件数と SHA は手元のリモート追跡参照で確認した値。
- [AGENTS.md](AGENTS.md) が Git 運用の基準。ボタン押下の継続先は `push-button`。各班の担当ブランチは `Dev/Mapping2`、`Dev/Navigation`、`Dev/Perception`。依頼から担当を特定できない場合はブランチを選ばず、現在の差分と参照を確認する。
- 実装を再開するときは `git status --short --branch` で変更を確認し、既存の差分を保護する。担当ブランチで `git fetch origin` のあと `git merge origin/main`。`main` への取り込みは PR の **Create a merge commit**。Update branch もマージコミット版を使い、機能ブランチは維持する。コミットの依頼だけならコミットまで。push は内容を確認してから明示的に別操作で行う。
- リポジトリ全体の入口は [README.md](README.md)、操作 PC・G1 搭載 PC（PC2）の共通セットアップは [SETUP.md](SETUP.md)。2026年8月の環境記述や各機能の進捗表は、対象ディレクトリの手順と検証日付を照合する。

## 機能別の入口

| 対象 | 開発・運用資料 |
| --- | --- |
| 共通環境・実機接続 | [SDK・LeRobot 環境](G1_HuggingFace/README.md)、[PC2](Common/g1-onboard-pc/README.md)、[ネットワーク](Common/network/README.md) |
| 地図作成 | `Mapping/real/mapctl`。内蔵 LIO と raw LiDAR / FAST-LIO2。実機側は ROS 2 Humble の Docker 環境。[概要](Mapping/README.md)、[現場手順](Mapping/FIELD_RUNBOOK.md) |
| 自律移動・巡回 | `Navigation/navctl` は Unitree 純正 `slam_operate` を使う系統。`Navigation/nav2_stable/` は ROS 2 / Nav2 と SDK ブリッジを使う系統。後者の2026-09-25の実機6点巡回と2周目の中断は専用の引き継ぎに記録。[純正 API](Navigation/README.md)、[Nav2 構成](Navigation/nav2_stable/README.md)、[Nav2 引き継ぎ](Navigation/nav2_stable/HANDOVER.md)、[実機手順](Navigation/nav2_stable/safety/runbook_nav2_session.md) |
| カメラ・認識 | ZMQ カメラ配信と YOLO。PC2 向けに ONNX Runtime 経路もある。[Perception](Perception/README.md)、[実機カメラ](Perception/real/README.md) |
| 歩行・発話・デモ・UI | [SimpleWalk](SimpleWalk/README.md)、[Voice](Voice/README.md)、[Entame](Entame/README.md)、[操作 UI](G1_HuggingFace/UI/README.md) |
| シミュレーション環境 | [Isaac Sim](IsaacSim_Env/README.md)、[3D LiDAR の試作](SimEnv3D/README.md) |
| ボタン押下の関連実装 | [J1-gen の視覚・腕制御](Button_Press/J1-gen/README.md)、[Yada の評価環境](Button_Press/Yada/README.md)、[RL の学習環境](Button_Press/RL/README.md)。直近の実装は [Manipulation/push_button](Manipulation/push_button/README.md) にある。 |

## 直近のボタン押下開発

[BUTTON_FIELD.md](Manipulation/push_button/BUTTON_FIELD.md) の冒頭「関節角を補正しながら実機で押す」が現行手順。[CAMERA.md](Manipulation/push_button/CAMERA.md) に頭部 RGB-D の配信・校正、[BOTTLE.md](Manipulation/push_button/BOTTLE.md) に台上のボトル押し、[ROBUST.md](Manipulation/push_button/ROBUST.md) に MuJoCo による事前探索の概要がある。

- **11月3日 `1f6e7ec`**。15 mm のボタンに対して、標準モードの基準・奥・左・右の4条件で事前検証。比較試験の結果と旧設定は [11月3日の実装・評価記録](Manipulation/push_button/reports/BUTTON_CONTROL_POLICY_PLAN_20261101.md) にある。
- **11月9日 `9c33e44`、`15dca6d`**。右腕の実測角を初期姿勢から帰還まで読み、重力補償に上限付きの積分トルクを加える。腰角と IMU も FK に反映。追従が遅いときは経路の進行を保留する。起動前監視はボトルと共有し、収集モードでの対象位置変化は記録しながら上限付きで補正する。
- **11月13日 `2d80adb`**。11月11日に 10 mm にした次指令先の障害物余裕を 3 mm に戻した。余裕を下回る場合は減速指令で再検査する。設定箇所は `COMMAND_CLEARANCE_M`。[共通コントローラ](Manipulation/push_button/button_tracking.py)、[補正テスト](Manipulation/push_button/tests/test_button_tracking.py)

シェル入口の `real/run_robust_button.sh` は `collection` の起動前監視とボタン検証、`joint` の補正、120秒の計画予算が既定。収集モードの物理検証は基準位置の1条件。標準ボタン検証は基準・奥3 mm・左3 mm・右3 mm の4条件。Python の直接起動ではボタン検証 `standard`、補正 `off` が既定。補正付きではストロークの既定が15 mmで、旧来の追加押し込みを加えない。

| 調べたい箇所 | ファイル |
| --- | --- |
| 起動・事前確認・試行記録 | [robust_session.py](Manipulation/push_button/real/robust_session.py)、[check_robust_readiness.py](Manipulation/push_button/real/check_robust_readiness.py)、[button_on_g1.py](Manipulation/push_button/real/button_on_g1.py) |
| 画像からの対象推定・経路採否 | [robust_perception.py](Manipulation/push_button/robust_perception.py)、[robust_planner.py](Manipulation/push_button/robust_planner.py)、[robust_conditions.py](Manipulation/push_button/robust_conditions.py) |
| 補正・実機とシムの共有処理 | [button_tracking.py](Manipulation/push_button/button_tracking.py)、[button_tracking_runtime.py](Manipulation/push_button/button_tracking_runtime.py)、[button_collection.py](Manipulation/push_button/button_collection.py)、[arm_bridge.py](Manipulation/push_button/real/arm_bridge.py) |
| 保存ログでの回帰 | [verify_button_tracking.py](Manipulation/push_button/sim/verify_button_tracking.py)、[試験用の保存観測](Manipulation/push_button/tests/data/button_tracking_nov03.json) |

### 実機で分かっていること

11月3日の旧制御による実機試行4件はパネル穴の下縁との衝突判定で停止した。保存した実測角・IMUを用いた FK 再解析では、右肩 pitch の指令との差3.9〜4.5°が手先の下ずれ約24〜28 mmに相当し、腰・骨盤姿勢も約4〜6 mm寄与した。右肩の追従不足の物理的な原因は未特定。当時のログには `dq`、`ddq`、`tau_est` がなく、現行の補正付き経路では SDK が提供する値を記録する。解析資料は `_local/robust_button/analysis_20261108_tracking/README.md`。

11月8日の補正評価では骨盤固定の MuJoCo で押下・退避・帰還を確認。実機での補正ゲイン、立位バランス、補正付きでの押下成功は未確認。現場での次の確認は `check` → `precontact` → `push` の順。`check` は重み0の出力確認で、`precontact` と `push` は腕が動く。帰還、解除、押下後の異なる3枚による点灯確認をログで追う。[実機試験の手順](Manipulation/push_button/BUTTON_FIELD.md)

## ローカル環境と検証

既存チェックアウトでは Python 3.10 の `_local/button_press_yada/venv/bin/python` と Menagerie モデル `_local/bottle_push/menagerie/unitree_g1/g1.xml` を使用。別 PC で再構築する場合は [MuJoCo の準備手順](Manipulation/push_button/README.md) と [Yada の準備手順](Manipulation/push_button/YADA.md) を参照。一部の回帰は `G1_MUJOCO_MODEL` に加えて上記の固定相対パスを参照する。歩行方策やカメラの URDF が必要なテストもある。

リポジトリ直下で実行する。現行の押下テストはローカルソケットや MuJoCo を使う。共通 CI の自動テスト対象外なので、押下コードを変えた場合は pytest を個別に実行する。[CI 定義](.github/workflows/ci.yml)、[共通テスト入口](scripts/ci/run_all_tests.sh)

```bash
G1_MUJOCO_MODEL="$PWD/_local/bottle_push/menagerie/unitree_g1/g1.xml" \
OPENBLAS_NUM_THREADS=2 OMP_NUM_THREADS=2 MUJOCO_GL=egl \
LIBGL_ALWAYS_SOFTWARE=1 CUDA_VISIBLE_DEVICES='' \
_local/button_press_yada/venv/bin/python -m pytest -q -rs Manipulation/push_button/tests
```

11月13日の引き継ぎ時は、サンドボックスでローカルソケットが使えず全体回帰が途中終了。直近の補正・起動前監視・標準ポリシーの重点試験は93件合格、1件不合格、ソケットを使う1件を除外。3 mm の障害物余裕のテスト2件は合格した。不合格は標準ポリシーの全候補比較で、30秒の上限内で候補34件までを試行し、期待する候補36に到達しなかった。11月9日の保存済み全体回帰は652件合格・40件スキップ。

`_local/` は Git 対象外。同じ作業コピーなら既存の環境・ログを参照できる。別のチェックアウトへ移るときは、必要なモデル・設定・実測校正・ログの所在を確認する。G1 上で実行する構成では、設定と実測校正は G1 側から読める場所に置く。Git に含まれるボタン検出重みは [button_final8.pt](Button_Press/J1-gen/weights/button_final8.pt)。

- `_local/robust_button/` に `active_setup.json`、`trial.json`、校正、実行ログ `runs/`、11月8日の `feedback_implementation_20261108/`、11月9日の `stop_policy_20261109/pytest.log` がある。実測校正の `position_uncertainty_m` と出典を保持する。
- `_local/robust_push/` にボトルの探索・異常系・動作時間の比較結果。`_local/wrist_press/` と `_local/arrival_measurement/` には下記の別ブランチの実験資料がある。

## 別ブランチで残している実験

`dev/button_list-cam`（手元では `47777d3`）の `Manipulation/wrist_press/` と `Navigation/arrival/` は、Git の参照を指定して確認できる。手首の視覚補正は既定 OFF。新方式の20条件は全工程成功・点灯とも0件で、従来方式の追加比較は6件で中断。到着計測は合成データで解析を検証済みで、実機30試行は未取得。

```bash
git show dev/button_list-cam:Manipulation/wrist_press/VERIFICATION.md
git show dev/button_list-cam:Navigation/arrival/VERIFICATION.md
```
