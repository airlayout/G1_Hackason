# 統合・検証報告（2026-10-01）

## 作業と保存元

- 作業checkout: `C:\dev\g1-integration-20261001\G1_Hackason`
- Branch: `feature/entame-g1-integrated-demo`
- Base: fetch済み`origin/Dev/Entame`。cleanな新規cloneから作成。main/他メンバーbranchは変更なし。
- g1-bottle-reaction実SHA: `506d6cbd849502139be223ec7def74b3d75feaaa`
- motiondecode-test実SHA: `78bacb68caa620797042703444a8286223663128`
- `git archive`から直接保存。`.git`・履歴・submoduleなし。既存`Entame/real/arm_wave_real.py`は未変更。

配置は`Entame/real/integrated_demo/{g1-bottle-reaction,motiondecode-test}`。g1側はsrc、config、4 WAV、motions、patrol、robot_side、scripts、tools、tests、docsと依存定義を保存。motion側はscripts、tests、config、10 motion CSV、metadata、Unitree SDK、CycloneDDS、G1モデル/mesh、ライセンス、必要なtrajectoryと過去安全gate資料を保存。

## 除外

元snapshotから370ファイルの不要な`output/`生成物・実機raw telemetry・ログ・プレビュー動画/画像等を除外。完全一覧は`SNAPSHOT_MANIFEST.json`の`excluded`。安全gateが読むstage_result、選定trajectory、offlineテストに必要なq0/ownership資料は保持。venv、cache、socket、PID、SSH ControlMaster、`.runtime`は元SHAの追跡内容にありません。新しい検証環境・ログ・補助スクリプトもcheckout外へ保存し、runtime cacheはignoreしました。

## 最小パス修正

- shellの`/home/ubuntu/dev/g1-bottle-reaction`を`BASH_SOURCE`から求めるrootへ変更。
- Pythonの`/home/ubuntu/dev/motiondecode-test`を自身の位置から求める兄弟ディレクトリへ変更。
- desktop Pythonの`/home/ubuntu/.venvs/g1-game-vision/bin/python`を`sys.executable`または`G1_PYTHON`（既定python3）へ変更。
- PatrolとPC2境界のSDKを同梱`motiondecode-test/external`へ変更。PC2 stdin実行対応のため、SDK既定値は作業rootからの`../motiondecode-test/external`（`G1_SDK_PATH`で上書き可）。
- residentのPython SDKはremote rootの`external`を参照。
- legacy Wanderの既定deployment rootを`/tmp/g1-integrated-demo/g1-bottle-reaction`へ、SDKをその兄弟snapshotへ変更。
- 歴史文書の旧2repoパスを`<integrated_demo>`表示へ変更。実行logic、判定、interlock、motion profile、パラメータ、テスト閾値は変更なし。
- `/tmp/motiondecode-current`等のG1 runtime path、Unitree内部IP/ownership識別、安全PID確認は保持。

変更ファイル17件と元/統合後SHA256はmanifestの`modified`に収録。デモのReaction→MotionDecodeソース参照は同梱snapshotで解決し、外部の元2repo cloneは不要です。任意motion生成ツールのGVHMR/GMR等、および既存G1 platform libraryの依存は残ります。legacy Wanderの欠落スクリプトは下記BLOCKEDとして保存しています。

## Third-party資産・公開確認

- YOLO重みは元Git SHAにないため収録なし。Git管理外の`.runtime/models`へのsetup取得手順をREADMEに記載。当日の重みhashは不明で再現一致を保証できない。Ultralytics AGPL-3.0/Enterprise条件を公開前に確認する。
- Unitree SDK/モデルは同梱BSD-3-Clause、CycloneDDSは同梱EPL-2.0/EDL-1.0表示を保持。CycloneDDS `.so`はLinux x86_64、CRCには元snapshotのarchitecture別libraryを保存。実機aarch64依存へx86_64 libraryを流用しない。
- Chingmu motion CSV 10本は固定revision/hash、出典と非商用prototyping等の利用条件を保存。商用配布・転売・競合dataset hostingは禁止。public repoへの再配布範囲は公開前に確認が必要（READMEに記載）。
- 50 MiB超ファイルなし。最大ファイルはdatasetのrevision metadata約13.2 MBで、保存したsearch_motionsの参照先のため保持。
- 全統合ファイルをcredential/API key/token/password assignment/private key/.env/個人email/巨大fileの観点で検索。実credential検出0、sensitive filename検出0。CycloneDDS private-keyヘッダー1件はellipsisを含む文書の断片例。manual_gate_tokenは安全確認文字列でcredentialではない。G1の内部IP・標準hostname・会場接続例は実装/診断資料として保持し、個人hostnameは検出なし。
- 旧2repo絶対パスは実行コードから除去。offline q0 fixtureの`source_ownership_report`文字列1件を履歴証拠として保持（現runtimeはそのパスを開かない）。

## 検証

検証環境はcheckout外の`validation-venv`、Windows Python 3.13、numpy 2.5.3、OpenCV 4.14.0.94、PyYAML 6.0.3、pytest 9.1.1、MuJoCo 3.13.0。検証用audit hookで非loopback送信と本物のssh/scp/rsync子プロセスを禁止。Unitree/CycloneDDSの実機bindingはインストールしていません。

| 状態 | 結果 |
|---|---|
| PASS | 固定SHA一致、archive snapshot、既存arm_wave保持、symlink 2本と実行permission保持 |
| PASS | Python AST構文435ファイル、motion CSV 10本の元SHA256一致 |
| PASS | game_vision/supervisor import、MotionDecode兄弟パス一致、4 WAV validate_sound |
| PASS | Bash scripts構文`bash -n`、旧実行絶対パス検索、secret scan、巨大file確認 |
| PASS | g1 pytest: 595 passed、14 skipped、3 deselected。Linux専用2ファイルを除外 |
| PASS | motion pytest: 104 passed。2件を環境理由でdeselect |
| PASS | `python -m g1_bottle_reaction --simulate --robot mock --speech console`完走 |
| PASS | `patrol/run_patrol.py --mode dry-run`: 1 loop、418 mock commands、NO G1 COMMAND SENT |
| PASS | `scripts/play_g1_arms.py output/selected_arms.csv --dry-run`: SDK import/DDS初期化なし、nominal clearance PASS |
| SKIP | g1: `test_g1_usb_send.py`、`test_g1_ssh_safe_action.py`はLinux resource依存、3 deselectedはresource/AF_UNIX/POSIX absolute path依存。14 skippedは既存テストのoptional環境判定 |
| SKIP | motion: `test_measurement_denies_sdk_publisher_and_direct_dds_datawriter`は未導入CycloneDDS binding、`test_state_machine_unknown_busy_and_reconnect`はWindows AF_UNIX未対応 |
| BLOCKED | motion geometry比較1件: `test_geometry_audit_uses_exact_runtime_pairs_and_refuses_classification`。約5.1056e-9 mm差、許容1e-9 mm。元固定SHAのcheckoutでも同じfailureを再現、統合での新規回帰ではない。閾値変更なし |
| BLOCKED | Git管理外の9/29事前構築bundle・MuJoCo 3.1.6実機deps・当日YOLO重みhashを取得できず、完全実機runtime再現は未確認 |
| BLOCKED | legacy `--with-wander`が参照する`scripts/g1-wander-reactive-mvp.py`は指定SHAに存在しない。Patrol経路は同梱。代替実装の推測追加なし |
| SKIP | 実機接続・DDS command・LocoClient・walk・Arm・real MotionDecode・relay実機接続・SSH/deploymentはユーザー禁止により全て未実施 |

`git diff --cached --check`はPASS。元snapshotのCSV byte/hashとvendor sourceを維持するため、`.gitattributes`でCSV CRLFのblank-at-eolとsnapshot/vendorの既存EOF空行を限定除外しています。Whitespace cleanupによるsnapshot改変は行っていません。

## 停止点

ローカルcommitのみ。commit SHAとcommit後clean状態は最終チャット報告に記載します。push、PR作成、Dev/Entame/mainへのmerge、force pushは未実行です。

READY FOR REVIEW BEFORE PUSH: NO

コード統合はローカルレビュー可能ですが、完全runtime再現の不足、上記Windowsでの未通過確認、Chingmu/YOLO公開条件の確認が残るため、公開/実機動作可能という判定は出していません。
