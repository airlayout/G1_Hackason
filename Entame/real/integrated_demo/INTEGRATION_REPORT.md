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

- YOLO重みはGit管理外。最終精査で元のG1_PERSON_YOLO.mdに公式v8.3.0 URL・サイズ・SHA256を確認し、同一hashの公式重みを再取得した。setup/fetch_yolo.pyで再構築可能。Ultralytics AGPL-3.0/Enterprise条件は保持。
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
| KNOWN UPSTREAM LIMITATION | UPSTREAM KNOWN FAILURE: motion geometry比較1件。約5.1056e-9 mm差、許容1e-9 mm。元固定SHAでも同一failure、閾値変更なし。push blockerから除外 |
| RECONSTRUCTIBLE | setup/rebuild_runtime.py、runtime/build依存pins、source/native build、manifestで再構築。original runtime bundle not archivedは既知制約。YOLOは公式重みと記録hashの一致を実確認 |
| KNOWN UPSTREAM LIMITATION | legacy Wanderスクリプト欠落。正規Patrol/Reaction/MotionDecode経路はwander offで無参照。push blockerから除外 |
| SKIP | 実機接続・DDS command・LocoClient・walk・Arm・real MotionDecode・relay実機接続・SSH/deploymentはユーザー禁止により全て未実施 |

`git diff --cached --check`はPASS。元snapshotのCSV byte/hashとvendor sourceを維持するため、`.gitattributes`でCSV CRLFのblank-at-eolとsnapshot/vendorの既存EOF空行を限定除外しています。Whitespace cleanupによるsnapshot改変は行っていません。

## 停止点

ローカルcommitのみ。commit SHAとcommit後clean状態は最終チャット報告に記載します。push、PR作成、Dev/Entame/mainへのmerge、force pushは未実行です。

## 08b54dfに対する最終blocker精査

### DISTANCE TEST — KNOWN UPSTREAM LIMITATION

**UPSTREAM KNOWN FAILURE**。対象はmotiondecode-testの`tests/test_safety_monitor.py:147`、failure assertionは153行。
`test_geometry_audit_uses_exact_runtime_pairs_and_refuses_classification`がright_wrist_yaw_link/right_hip_pitch_linkの距離を`-2.980470734706089 mm`、絶対許容`1e-9 mm`で比較し、Windows MuJoCo 3.13.0では差`5.10561771e-9 mm`で失敗する。

ユーザー指定の`506d6cbd849502139be223ec7def74b3d75feaaa`はg1側SHAで、このテスト自体は含まれない。正しい統合前の検証対象は、これと対のMotionDecode固定SHA`78bacb68caa620797042703444a8286223663128`。この元checkoutで同じtestを再実行し、同じ数値差を再現した。

元SHAと統合commitのGit blob一致証拠:

| ファイル（motiondecode-test内） | 両方のblob |
|---|---|
| tests/test_safety_monitor.py | 516974934f2ba801e8acb741e56109574334ba7a |
| scripts/audit_geometry.py | ab1e737074fcdce38895817aa810da6a367ef896 |
| scripts/common.py | 6c60cbb2f10d879c00e248a5e997dea90ee6a881 |
| external/g1_official_29dof.xml | 41f514f95564bad128f5c6aeef334812d3bd3939 |
| output/hold_baseline_noise_20260913_165151/fresh_q0.json | 3c2c15aadcb6cda006a9a903e07c0765e5295e58 |

source/asset全体の比較もmanifestに保存。path portabilityが新たに発生させたfailureではなく、9/29固定版に含まれるWindows上の数値比較状態。9/29実機でこのunit testがfailしていたとは記録から証明できないため、そのようには断定しない。実装・閾値の変更なし、push blockerから除外。

### MOTIONDECODE RUNTIME — RECONSTRUCTIBLE

元g1 `.gitignore`が`.runtime/`、venv、cache、vendorを除外。restoreは`.runtime/motiondecode-known-good/{code,deps}`と3 manifestを要求するがGit treeにbundleなし。既存ローカルcloneの候補パスにも利用可能bundleなし。未追跡だった設計上の理由は生成済みコードコピー/依存物の分離とignore設定で説明できるが、当日の作者の判断やfull内容は記録不足。

codeに要求されるresident_worker/client、ownership probe、named reactions、motion hash指定assets等は同梱sourceに存在。setupは同梱source/必要gate資料からcodeを生成する。depsはMuJoCo 3.1.6、numpy、CycloneDDS binding、SDK import依存、MuJoCoのetils/glfw/OpenGL等を固定pinで構築。Unitree SDK/CRCは同梱版を使い、native CycloneDDS 0.10.2を固定commitからaarch64へcompile。CPython 3.8/aarch64用MuJoCo 3.1.6とNumPy 1.24.4 wheelの取得成功を確認した。

`setup/rebuild_runtime.py --output <new-dir>`でsource-only生成・manifest検証PASS。`--install-deps`は別のLinux aarch64/Python 3.8 build hostで実行する。今回Windowsではnative buildを実行していない。生成depsは当日pip freezeではなく、当日bundleのbyte一致・実機測定同等性は未保証。これは正直に残す再構築制約で、古い生成bundleがないことだけではpushを止めない。

今回の追加runtimeコード変更はshellパス修正のみ: native libraryをdeployしたcode内へ解決し、先の統合で誤挿入していたSSH stdin内のBASH_SOURCE root行を削除。後者は今回見つかった統合側のパス不具合として修正し、元repo由来とは分類しない。Pythonの9/29制御logic・gate・motion挙動は変更していない。

### YOLO WEIGHT — RECONSTRUCTIBLE

**YOLO WEIGHT RECONSTRUCTIBLE**。`yolo11n.pt`、期待path `.runtime/models/yolo11n.pt`、公式v8.3.0 URL、5,613,764 bytes、SHA256 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`が元G1_PERSON_YOLO.mdに存在。setup/fetch_yolo.pyで公式取得、size/hash一致を実確認。weightはGit管理外、モデルload/inferenceは未実行。

### LEGACY WANDER — KNOWN UPSTREAM LIMITATION

`start-integrated-demo.sh`は`--with-wander`とremote Wanderを起動するlegacy経路。`run_integrated_demo.py`はReaction/Patrol childを分離し、`--patrol-control-socket`、pause/resumeを使用、`--with-wander is deliberately absent`、`WANDER=OFF`。G1_INTEGRATED_DEMO_20260926.mdにも実機検証済み構成はwander offとある。旧欠落scriptはcanonical経路に不要、削除や代替実装なし、push blockerから除外。

### CANONICAL ENTRYPOINT

保存された検証済みアーキテクチャのentrypointは`scripts/run_integrated_demo.py --real-patrol --operator-approved-real-patrol`。同梱Patrol + detection/audio Reaction + MotionDecode residentの経路。camera既定はssh-jpeg、legacy start scriptはssh-rtp + Wander。

「9/29に3周成功」の正確な実行コマンド/logは固定snapshotにない。supervisorは`--loops 1`固定で、Patrol READMEの直接実行例は`--loops 3`。最終Patrolデモの構成は判定できるが、このsupervisorそのものが3周したと推測で断定しない。元動作を変えるloop数変更は行わない。未保存の運用記録はKNOWN UPSTREAM LIMITATIONであり、統合の新規不具合ではない。

### PUBLIC LICENSE CHECK

詳細はPUBLIC_LICENSE_REVIEW.md。Unitree BSD、CycloneDDS EPL/EDL、Viewer MITのnoticeと由来を保持。Chingmuは非商用prototyping条件と出典を保持、商用/競合dataset用途の許可は主張しない。private repoだったことを理由とするblockerなし。

**TRUE BLOCKER**: `assets/audio/reactions/{person/detected.wav,banana/detected.wav,plushie/detected.wav,plushie/plushie_affectionate.wav}`の作者/声モデル/公開再配布条件を固定snapshotから確認できない。音声は保存し、公開権利の確認をユーザーへ依頼。違反だと断定せず、この4ファイルだけを未確認として分類。

### 最終検証

- full g1 pytest実行: Linux resource非対応で収集停止。対応範囲の全suite: **595 PASS / 14既存SKIP / 3環境依存deselect**、Linux専用2ファイル除外。
- full MotionDecode pytest: **104 PASS / 3 failure**。2件はCycloneDDS未導入/AF_UNIX未対応、1件は上記UPSTREAM KNOWN FAILURE。元checkoutの該当testも同じfailureを再実行確認。
- setup integrity tests **4 PASS**（asset改ざん、manifest path escape、空deps、weight hash拒否）。source-only bundle生成・検証PASS、aarch64 wheel取得PASS、native build/importはWindowsのためSKIP。
- Python syntax **438 PASS**、Bash syntax PASS、import/兄弟path/4 WAV検証PASS、motion CSV 10 hash PASS。
- mock simulation、Patrol dry-run、MotionDecode SDKなしdry-run PASS。非loopback通信/SSH等の禁止audit hookを付けて実行。
- secret scanは実credential0、private-key headerはvendorの省略された文書例1件のみ。license scanは上記個別一覧。weight/build/venv/cacheはGitへ追加しない。
- git diff --checkとcommit後statusは最終チェックで確認。push/PR/merge/G1接続/SSH/DDS/motion/walkingは未実行。

READY FOR PUSH: NO

TRUE BLOCKERS: 上記4 WAVの公開再配布権利の確認のみ。距離比較・原bundle未保存・公式YOLO重み・legacy Wander欠落をpush blockerとしては扱わない。
