# G1 ハッカソン 実機エンタメ統合デモ

2026-09-29動作確認版の固定snapshotを保存する統合です。今回の作業では実機接続・DDS command・歩行・Arm motion・SSH・deploymentを実行していません。実機動作を再検証した版ではありません。

## Source snapshots

| Source | 固定commit |
|---|---|
| g1-bottle-reaction | `506d6cbd849502139be223ec7def74b3d75feaaa` |
| motiondecode-test | `78bacb68caa620797042703444a8286223663128` |

両方とも`recovery/hackathon-runtime-20260929`指定。Git履歴・submoduleは含めず、ソース・設定・必要資産を直接収録。差分はパス可搬性、統合説明、生成物除外です。詳細は[INTEGRATION_REPORT.md](INTEGRATION_REPORT.md)と[SNAPSHOT_MANIFEST.json](SNAPSHOT_MANIFEST.json)を参照。

## 主要機能

- G1 camera、YOLO、person detection、banana detection、plushie detection。
- person → FOUND、banana → SURPRISE、plushie → SURPRISE、音声リアクション。
- MotionDecode上半身リアクション、Patrol、stop → react → q0 / weight 0 → resume。
- odometry、IMU、LiDAR guard、locomotion relay、safety interlock。

判定・制御ロジック、motion振幅、interlock、既存の明示実機gateは変更していません。residentのSURPRISEは元snapshotの吊り下げ用profileをそのまま保存しています。一般の自立歩行で安全という意味ではありません。

## ディレクトリ構造

```text
integrated_demo/
├── README.md / INTEGRATION_REPORT.md / SNAPSHOT_MANIFEST.json
├── g1-bottle-reaction/
│   ├── src/ config/ assets/audio/ motions/ tests/ docs/ tools/
│   ├── patrol/          # guard・IMU/odometry・relay・interlock
│   ├── robot_side/      # G1側コード
│   └── scripts/         # 既存の起動・診断スクリプト
└── motiondecode-test/
    ├── scripts/ config/ tests/
    ├── data/            # 固定revisionのmotion CSV・metadata
    ├── external/        # SDK・CycloneDDS・モデル・各ライセンス
    └── output/          # 参照されるtrajectoryと必要な安全検証資料のみ
```

既存の`../arm_wave_real.py`は変更していません。runtimeログ・socket・PID・venv・キャッシュはGitへ追加しないでください。保存した`output/`資料は過去の記録であり、現在のq0・ownership・preflightとして再利用しないでください。

## 必要環境とsetup

操作PCはLinux、Python 3.12、GPU/CUDA、GStreamer、SSH接続環境が本番前提。Windows Python 3.13ではmock・単体テストを実行できます。Dockerは使用しません。通常はリポジトリの`G1_HuggingFace/venv/`を使い、9/29実機residentのMuJoCo 3.1.6環境とは分離してください。G1 PC2はLinux aarch64、既存Unitreeサービスと対応DDS/native libraryが必要です。

以下は操作PCでのsetup例（ロボットへ接続しません）。`DEMO`をこのディレクトリの絶対パスに設定します。

```bash
export DEMO=/absolute/path/G1_Hackason/Entame/real/integrated_demo
export G1_PYTHON=/absolute/path/G1_Hackason/G1_HuggingFace/venv/bin/python
"$G1_PYTHON" -m pip install -e "$DEMO/g1-bottle-reaction[dev]"
export PYTHONPATH="$DEMO/motiondecode-test/external:$DEMO/g1-bottle-reaction/src${PYTHONPATH:+:$PYTHONPATH}"
```

MotionDecodeのdesktop用依存は`motiondecode-test/requirements.lock.txt`に保存しています。CycloneDDS同梱C libraryはLinux x86_64用で、Windows/aarch64へ流用しないこと。Windows offline検証はnumpy・pytest・MuJoCo 3.13.0のみで可能な範囲を実行しました。実機residentは元スクリプトがMuJoCo 3.1.6を厳密に要求します。desktop lockをG1へそのまま適用してはいけません。

YOLO11n重みはGit管理外ですが、元の`docs/G1_PERSON_YOLO.md`に公式release・サイズ・SHA256が記録されています。**YOLO WEIGHT RECONSTRUCTIBLE**。今回は公式URLから取得し、その記録と一致することを確認しました。モデルのload/inferenceは実行していません。

```bash
"$G1_PYTHON" "$DEMO/setup/fetch_yolo.py"
```

期待値は5,613,764 bytes、SHA256 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`。固定v8.3.0公式releaseから取得して両方を検証し、相違時は書き込みません。重みはGit管理外。Ultralyticsの[公式ライセンス](https://www.ultralytics.com/license)を確認し、AGPL-3.0の公開条件または適切なEnterprise契約を満たしてください。

motion CSVはChingmuの非商用研究・個人学習・prototyping条件で保存した10候補です。出典は[CMRobot/MotionDecode](https://huggingface.co/datasets/CMRobot/MotionDecode)、固定revisionは`988decc8724ad686ace4b003851506e694000acc`。`external/MotionDecode_LICENSE.md`を保持しています。商用配布・転売・競合datasetとしてのhostingは許可されていません。公開push前にはこの用途での再配布条件を確認してください。欠落した候補は`motiondecode-test/scripts/download_candidates.py`で固定revision・SHA256検証付きで取得できます（setup時のネットワーク取得。今回実行していません）。SDK/モデルはBSD-3-Clause、CycloneDDSは同梱EPL-2.0/EDL-1.0表示を保持。

## 接続しないdry-run

```bash
cd "$DEMO/g1-bottle-reaction"
"$G1_PYTHON" -m g1_bottle_reaction --simulate --robot mock --speech console
"$G1_PYTHON" patrol/run_patrol.py --mode dry-run
"$G1_PYTHON" -m pytest
cd "$DEMO/motiondecode-test"
"$G1_PYTHON" -m pytest
"$G1_PYTHON" scripts/play_g1_arms.py output/selected_arms.csv --dry-run
```

最後のdry-runはソース検査とテストでsocket作成・SDK importなしを確認しました。`scripts/run_integrated_demo.py --no-locomotion`は実カメラ・route・relay確認を伴うため、offline dry-runとして実行しないでください。`start-integrated-demo.sh --dry-run`にもroute調査があるため今回未実行です。

## 実機準備と起動順（オペレーター向け・今回未実行）

1. 会場・吊り下げ支持・補助者・非常停止・安全な旋回範囲を確認する。既存Arm writer、Wander、Patrol、relayの競合がないことを確認する。接続設定はローカル環境へ保存し、credentialをGitへ入れない。
2. 両snapshotを同じ`integrated_demo`配置でG1側へ準備する。Python SDKは同梱`motiondecode-test/external`を使用し、PC2に適合するCycloneDDS/native librariesを準備する。配置・接続は別途承認された実機作業でのみ行う。
3. residentはG1側`/tmp/motiondecode-current`にMotionDecodeコード、`/tmp/motiondecode-hold-deps`にMuJoCo 3.1.6等を要求する。[setup/README.md](setup/README.md)の固定依存・native build・manifest生成手順でbundleを再構築する。SDK/native libraryは生成したcode内から解決する。original runtime bundle not archivedは既知制約であり、当日bundleとのbyte一致は保証しない。再構築は別のLinux aarch64/Python 3.8 build hostで行い、今回G1に接続しない。
4. 当日のruntimeを別途準備・検証後、操作PCで`G1_SSH_TARGET`、`G1_SSH_CONTROL`、`G1_PYTHON`を設定し、`scripts/start-motiondecode-resident.sh`を実行する。strict Arm PID probe、READY、IDLE、weight 0、ownership safeを確認する。
5. G1側では同梱`g1-bottle-reaction/patrol/lidar_guard_relay.py`、次に`patrol/locomotion_relay.py`を起動する。`PYTHONPATH`には同梱SDKを設定する。具体的な引数は[Patrol README](g1-bottle-reaction/patrol/README.md)を参照し、旧独立deploymentパスは同梱`patrol/`へ読み替える。guard-pathには同梱`patrol`ディレクトリを指定する。
6. 操作PCでは次の有限Patrol統合経路を使用する。これは実機コマンドであり、今回実行していない。

```bash
cd "$DEMO/g1-bottle-reaction"
bash scripts/check-demo-ready.sh
"$G1_PYTHON" scripts/run_integrated_demo.py \
  --real-patrol --operator-approved-real-patrol --headless \
  --duration 600 \
  --ssh-target "$G1_SSH_TARGET" --ssh-control "$G1_SSH_CONTROL"
```

**CANONICAL VALIDATED PATH**: `scripts/run_integrated_demo.py`のPatrol + Reaction + MotionDecode。`reaction_command()`は`--with-wander`を含まず、Patrol IPC pause/resumeを使い、preflightは`WANDER=OFF`を表示します。`docs/G1_INTEGRATED_DEMO_20260926.md`も実機検証済み構成をwander offと明記しています。

**LEGACY / ALTERNATIVE PATH**: `scripts/start-integrated-demo.sh`は`--with-wander`を使う古い経路です。固定snapshotに`scripts/g1-wander-reactive-mvp.py`はありませんが、正規Patrol経路はそのファイルを参照しません。**Legacy Wander path is not part of the validated integrated patrol demo**。コードは削除せず、欠落を既知制約として保存し、push blockerから外しています。

「3周成功」の正確な当日コマンド/logは固定SHAに保存されていません。supervisorの`patrol_command()`は`--loops 1`固定、Patrol READMEの直接起動例は`--loops 3`です。正規アーキテクチャは特定できますが、supervisorが3周実行したとは断定しません。loop数やruntimeロジックは変更していません。

## 終了方法・安全事項

操作PCの統合supervisorをCtrl+Cで終了し、Patrolの停止、q0 return、arm_sdk weight 0、ownershipの解放を確認します。residentは既存clientの`{"operation":"stop"}`で停止し、relayも終了します。relayが開始したSLAM sessionだけを閉じてください。例外、stale odometry/IMU/LiDAR、ownership不明、return失敗では再開を強制しないこと。実機での終了・fault処理も今回は未検証です。

## 既知の制約

- 原bundle/venvは未保存ですが、sourceと固定依存から再構築するsetupを追加しました。実機とbyte単位の完全一致は未保証、aarch64でのbuild/import確認は今回SKIP。YOLOは元の記録hashとの一致を確認済みです。
- Linux専用`resource`、Unix socket、CycloneDDS依存はWindowsで検証できません。一部MuJoCo距離比較にはOS間の微小数値差があり、閾値を変更していません。
- 歴史資料の旧環境パス、任意のmotion生成ツールに必要なGVHMR/GMR/SMPL等はデモ実行依存ではありません。これらの生成ツールの外部依存は保存しています。
- 過去検証資料へのリンクには、不要な生成物を除外したため開けないものがあります。必要なtrajectory・安全gateのstage_result・テストfixtureは保持しました。
- Push・PR・mergeは今回禁止。公開権利の個別確認は[PUBLIC_LICENSE_REVIEW.md](PUBLIC_LICENSE_REVIEW.md)を参照。Windowsでは元SHAでも再現する距離比較failureを**UPSTREAM KNOWN FAILURE**とし、統合のpush blockerから外しました。実機実行可能という保証は別の検証です。
