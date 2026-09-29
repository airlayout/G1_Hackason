# プロジェクト概要：G1 Physical AI ハッカソン

新しい Claude Code セッション向けの前提説明です。まずこのファイルを読み、次に作業内容に応じて `HANDOFF_button_press.md`（ボタン押しタスクの詳細）を読んでください。
リポジトリ内の詳しい情報は、ルートの `README.md`、`SETUP.md`、各フォルダの `README.md` と `FAILURES.md` にあります。

---

## 1. ハッカソンの概要

- 約2か月間の Physical AI ハッカソン。**Unitree G1（ヒューマノイドロボット）で自律警備パトロールを実現する**。
- 目標：**10月末までに自律警備パトロールを動かす**。
- 想定する警備パトロールの流れ：作成済みの地図上を自律的に巡回し、人を認識して撮影・報告する。エレベーターのボタンを押すなど、手を使う作業にも対応する。
- チームは約8人で、機能ごとに班に分かれている。毎週水曜の夜にチームミーティングがある。

## 2. チームと役割

| 担当 | 内容 |
|---|---|
| Jigen（このセッションの利用者） | **Perception（画像認識）**。ボタン押し機能の認識と動作も担当 |
| Watanabe | Perception（Jigen と共同） |
| Miyoshi | カメラ画像の取得（`run_g1_server.py` まわり） |
| Hayashi、Inagaki、Inoue | 地図作成（SLAM）、自律移動（Navigation） |

他の班のコードを変更する必要がある場合は、変更する前に確認すること（例：カメラのサーバ側に深度を追加する場合は、カメラ担当と相談が必要）。

## 3. リポジトリ

- `github.com/airlayout/G1_Hackason`
- Jigen の作業ブランチ：**`Dev/ButtonPress`**（ボタン押し。Perception 全般は `Dev/Perception`）。他に `Dev/Navigation`、`Dev/Mapping2`、`Dev/Common` などがある。

| フォルダ | 内容 | 状態（README より） |
|---|---|---|
| `G1_HuggingFace/` | LeRobot と `unitree_sdk2py` の共通 Python 環境、G1 との ZMQ⇔DDS ブリッジ（`run_g1_server.py`） | 動作確認済み |
| `Common/` | ネットワーク設定、疎通確認などの共通スクリプト | 運用中 |
| `SimpleWalk/` | 前進歩行 | シミュレーション・実機で確認済み |
| `Perception/` | 画像取得・認識（このセッションの主な作業場所） | 実機での確認待ち |
| `Mapping/` | G1 内蔵の LIO / FAST-LIO2 による3D地図作成 | 実機で初回試験済み |
| `Navigation/` | 作成済みの地図を使った自律移動・巡回（Unitree 純正の `slam_operate` を利用） | シミュレーションで巡回を完走 |
| `Entame/` | ダンス、ジェスチャーなどの動作 | 未着手 |
| `IsaacSim_Env/`、`SimEnv3D/` | Isaac Sim 環境、3D LiDAR シミュレーション | **当面使わない** |

- 各機能は **`sim/`（MuJoCo で検証）→ `real/`（実機で実行）** の順に開発する。
- 失敗と反省は、各フォルダの `FAILURES.md` に記録する。同じ失敗を繰り返さないよう、作業前に読むこと。
- ⚠️ ルートの `CLAUDE.md` は、主に Isaac Sim 環境（特定ユーザーのパス、GPU L40S）について書かれている。**今回の作業（MuJoCo、実機、Perception）には当てはまらない部分が多い**ので、書かれているパスや環境をそのまま前提にしないこと。

### Perception の既存実装（`Perception/`）
- `common/camera/`：`ZmqFrameSource`（ZMQ ストリーム。sim と real で共通）、`VideoFileSource`、`WebcamSource`
- `common/detector/`：`YoloDetector`（ultralytics の YOLO による物体検出）
- `common/output/`：`ResultWriter`（console / JSON / CSV に出力）
- `common/pipeline.py`：`FrameSource → YoloDetector → ResultWriter` をつないでループ実行する
- `sim/`：MuJoCo シミュレーションの `head_camera` が配信する ZMQ ストリームに接続する
- `real/`：実機のカメラ（`run_g1_server.py --camera`、ポート 5555）に接続する
- `tests/run_tests.sh`：CI が自動で見つけて実行する
- ローカルデータはリポジトリ直下の `_local/perception/`（git 管理外）に置く。**実機で撮った画像は取り直せないので、必ずバックアップする**。

**新しいコードは、これらの既存の部品（`ZmqFrameSource`、`YoloDetector` など）を再利用・拡張する形で作ること。** 詳細は `Perception/README.md` を参照。

## 4. 開発環境

- Python 環境：**`G1_HuggingFace/venv/`（Python 3.12）** を使う。conda は使わない。Docker を使うのは `Mapping/` だけ。
- G1 への指令は DDS で送る。歩行（`LocoClient`）やナビゲーション（`slam_operate`）も `unitree_sdk2py` から直接呼べるので、**制御だけなら ROS 2 は不要**。
- 開発マシン：
  - Jigen のノート PC：Windows（Snapdragon ARM64）＋ WSL2 Ubuntu。NVIDIA GPU はない。
  - ラボ PC：RTX 5060 Ti、Ubuntu。学習や重い推論はここで行う。
  - Claude Code を Web 版で使う場合、クラウドからは G1 やラボ PC に接続できない。コード作成、テスト、MuJoCo のヘッドレス検証までをクラウドで行う。
- 流れ：コードを書く → GitHub に push → ラボ PC / G1 で実行する。
- 複数行のシェルコマンドは、1行ずつ実行できる形で示すこと（貼り付けで壊れるため）。
- 初めての環境構築は `SETUP.md` を参照。

## 5. Git ルール（厳守）

- 作業を始める前に、main を取り込む：`git fetch origin` → `git merge origin/main`
- **commit と push は分ける**（`git commit && git push` のようにまとめない）。push は内容を確認してから明示的に行う。
- push の前に `bash scripts/ci/run_checks.sh` と `bash scripts/ci/run_all_tests.sh` を実行する。
- main には PR 経由で入れる。**マージは「Create a merge commit」のみ**（squash と rebase は使わない）。
- **マージ後もブランチを削除しない**（各班が同じブランチを使い続けるため）。
- GitHub の「Update branch」を使うときは、merge commit 版を選ぶ（rebase 版は使わない）。
- 理由などの詳細は `README.md` の「Git運用」を参照。

## 6. G1 の基本情報

- **機体構成：`g1_29dof_rev_1_0`**（`mode_machine = 5`）。脚 6×2、腰 3DoF、腕 7DoF×2。
- 2台のコンピュータを内蔵している：
  - PC1（運動制御）：`192.168.123.161`。ユーザーはアクセスできない。
  - PC2（開発用、Jetson Orin NX）：`192.168.123.164`。SSH 可能。**PC2 の OS を書き換えると起動しなくなるおそれがあるので、絶対にやらない**。
- 通信：CycloneDDS（0.10.2）、G1 用の `unitree_hg` IDL。
- センサー：Livox Mid-360 LiDAR（`192.168.123.20`）、頭部に RealSense D435i（下向きに付いている）。
- カメラは ZMQ 経由で取得する（`unitree_sdk2py` の `VideoClient` には G1 用のカメラモジュールがないため）。現在は RGB のみで、深度は未配信。
  - 2026-09-29 追記: G1 用のモジュールは無いが、**Go2 用の `VideoClient`（`unitree_sdk2py.go2.video`）で PC2 の中から頭カメラのカラー（1920x1080 の JPEG）を受け取れた**。Unitree の `videohub_pc4` が `/dev/video4` を開いているため、ボタン押しは「カラーは videohub、深度は RealSense から直接」にする予定（`real/REAL_DAY_PROCEDURE.md` の「次回やること」）。
- 手の指は動かせない。
- ナビゲーションは、Nav2 から Unitree 純正の `slam_operate` API に方針を変更した。
- カメラの使い分け：歩行は LiDAR のみで行う。人の認識・撮影・報告は外付けの RGB カメラで行う。ボタン押しには G1 の頭カメラを使う。

### 実機を扱うときの安全ルール
- 低レベル制御のコードを初めて実機で動かすときは、吊り下げ（ガントリー）などの物理的な保護を用意する。
- 緊急時はリモコンの **L2+B（ダンピング）** ですぐに止める。実機を動かすときは、人が常にリモコンを持っておく。
- すべての実機用スクリプトに、dry-run モードと安全な終了処理を入れる。

## 7. 現在の状況とスケジュール

| 期間 | 内容 |
|---|---|
| 〜9/30 | ボタン押しのプロトタイプ（ボトル押し）のコードを、実機なしで作り終える |
| 9/30 までの1日 | **実機を使える唯一の日**。ボトル押しの実機テスト、データ収録、ボタンの撮影 |
| 10/1〜13 | **実機は使えない**。ボタン認識の学習、収録データを使った改善 |
| 10/14〜 | ボタン押しの実機テスト |
| 10月末 | 自律警備パトロールの完成 |

- ボタン押しの詳細な設計、タスクの一覧、リスク（arm_sdk が効かない可能性など）は **`HANDOFF_button_press.md`** にまとめてある。

## 8. 作業の進め方のお願い

- 既存のコードや README を先に確認し、既存の構成と書き方に合わせる。
- 分からないことや、ドキュメント間で食い違う情報は、推測で埋めずに確認する。
- 専門用語が新しく出てきたら、短い説明を添える（利用者は大学1年生で、ロボティクスは学習中）。
- 実機は使える日が限られているので、「実機でしか確認できないこと」と「事前に確認できること」を常に区別して進める。
