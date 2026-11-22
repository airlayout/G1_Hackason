# MotionDecode → Unitree G1：最小リアクション検証

## Resident reaction worker

ゲーム中のcold startを避ける実験経路は、PC2でDDS、LowState、publisher、ownership
watchdog、MuJoCo preflight、trajectory deltaをREADY前に初期化します。reactionごとの
Python起動やCSV解析は行いません。

```bash
PYTHONPATH=/tmp/motiondecode-hold-deps:scripts:/home/unitree/unitree_sdk2_python \
LD_LIBRARY_PATH=/home/unitree/work/unitree_sdk2/thirdparty/lib/aarch64 \
python3 scripts/resident_worker.py --real --confirm-site-ready \
  --network-interface eth0 --expected-arm-pid 2934 \
  --reaction surprise --socket /tmp/motiondecode-reaction.sock
```

`status` はcommandを送らずに確認できます。

```bash
python3 scripts/resident_worker_client.py --socket /tmp/motiondecode-reaction.sock
```

workerは `STARTING → READY → EXECUTING → READY` を明示し、同時requestを拒否します。
各reaction後はweightを0へ戻します。DDS writerの常駐とG1 ownershipは別に扱います。

2026-09-12、Ubuntu共有PCでの調査・作成結果。作業先は `<integrated_demo>/motiondecode-test`。

**2026-09-13の無線receive-only検証：BLOCKED。** 実経路は `wlp128s20f3`（10.42.0.1/24、AP `G1TELEOP`）からG1無線IP `10.42.0.76` で、192.168.123.161/.164への残存routeは未使用の `enp129s0` を向いていました。無線multicastとprocess-local unicast discovery peerの両方で30秒観測しましたが、LowState/DDS matchは0件でした。このためfresh q0と120秒fast-loop測定は開始せず、HOLD RETEST READYはNOです。[無線検証レポート](output/wireless_safety_validation_20260913_201526/report.md)にroute、受信結果、offline geometry監査を保存しました。

`scripts/safety_monitor.py` は同期ownership discoveryをFK/collision loopから分離し、thread-safe cacheとsession単位のlatched faultを提供します。hard safetyとbaseline-relative観測も別の結果に分けました。既存0.001 mm比較は変更せず、pair別候補値は測定時に算出するだけです。`scripts/validate_wireless_safety.py` は明示したinterface/peerで30秒preflight後、成功時だけ120秒receive-only fast loopへ進みます。`scripts/audit_geometry.py` はruntimeと同じMuJoCo pair/FKでsigned distance、closest points、mesh/frame transformとoffline previewを生成します。

**2026-09-13のreceive-only測定：NOISE-A。** 120.017秒でLowState 116,343件を取得し、runtimeと同じFK・全pair距離計算で全件を再評価しました。自然なbaseline悪化は最大0.039021 mmで、現行0.001 mmを9 baseline pairの各65.7〜90.0%のsampleが超えました。application commandはNONE、衝突閾値は未変更、HOLD RETEST READYはNOです。[測定レポート](output/hold_baseline_noise_20260913_165151/report.md)に全pair統計、joint相関、周期と設計案を保存しました。

測定は `scripts/measure_hold_baseline_noise.py --seconds 120 --output-dir output/hold_baseline_noise_<timestamp>` で実行できます。Publisher/DataWriter生成をプロセス内で禁止しています。今回だけPIDが2934から2884へ変わっていたため、`--internal-observation-json` に15秒間idleの内部serviceと同じIP/host/process/topic構成を記録したreportを渡し、正確なGUID/PIDをreceive-only観測内だけで照合しました。実機runnerのownership設定は変更していません。

今回q0のbaseline外になっていた前回停止pairは `scripts/supplement_previous_baseline_pair.py` で同じFKから補足し、`scripts/finalize_hold_noise_report.py` で全sampleを結合しました。次のHOLDへ進む前に、pair別ノイズ帯のレビューと、平均34.37 ms／p99 104.00 msだったlive監視周期の改善・検証が必要です。

**最新の実機試験（21:54）：STAGE 1 HOLDはruntime baseline監視でFAIL。送信は初期化済みweight=0の3通だけで、正のweightには到達していません。STAGE 2とFULLは未実行です。** [実機試験レポート](output/real_robot_test_20260912_214812/report.md)にfresh q0、送信・実測ログ、停止理由と解放確認を記録しました。以下の「command未送信」は21:21までのoffline作業履歴です。

今回の専用runnerは `scripts/run_real_reaction.py`。既存runnerの右腕以外の初期値を避け、公式arm7の上半身17関節を実測角で初期化します。右腕以外はその角度を保持し、脚のgainは0です。各stageは独立コマンドで、毎回15秒のreceive-only preflightとfresh q0から再評価します。既存数値閾値を緩めた再試行はしていません。

**metadata調査、PCプレビュー、安全機構、DDS ownership、最新q0からの全frame自動探索まで実施しました。実機へのモーション送信はしていません。**

以前選定した **`SII_Direction_Pointing_00001` のフレーム180〜270** は、最新q0からのentryで新規collisionが生じるため不採用です。代わりに10 CSV・44,641 frameをq0起点で探索し、**`EESB_Frustration_00001 [5552,5613)`** の右腕区間を厳格安全条件を通過した候補として検出しました。

21:21の15秒ownership調査では、G1内部の既知Arm serviceだけが存在し、`rt/arm_sdk` と `rt/armsdk` のcommand sampleは0、Arm ActionはIDLE、LowStateと静止判定はPASSでした。同じ観測のq0から新候補へ入って戻るoffline FK検査もPASSです。これは**実機候補が見つかった**という段階であり、実機試験の許可や成功を意味しません。次はユーザーが [比較ページ](output/reaction_candidates/index.html) のentry/reaction/returnを確認します。

## 1. 環境と共有PCの扱い

| 項目 | 調査結果・今回の利用 |
|---|---|
| OS | Ubuntu 24.04.4 LTS |
| Python | PATH先頭はMiniforge 3.14.6。`/usr/bin/python3` 3.12.3で本プロジェクトの `.venv` を新規作成 |
| GPU | NVIDIA GeForce RTX 5060 Ti、16311 MiB、Driver 595.84 |
| CUDA | Driver API `cuInit(0)=0`、1 device、API version 13020。`nvcc`はPATH上になし。CUDA環境の追加・変更なし |
| MuJoCo | 調べた既存venvにはなし。専用venvへ3.13.0導入。EGLで画像・MP4出力を確認 |
| 既存Python SDK | `/home/ubuntu/unitree_sdk2_python`、commit `65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5`。既存コピーを `external/unitree_sdk2py` に複製して使用 |
| 既存関連venv | `/home/ubuntu/.venvs/g1-game-vision`、`<integrated_demo>/g1-bottle-reaction/.venv-g1`、Miniforge `tv`。変更なし |
| CycloneDDS | Python binding 0.10.2を専用venvへ。既存 `<integrated_demo>/g1-bottle-reaction/.runtime/cyclonedds` のCライブラリ0.10.2を `external/cyclonedds` に複製 |
| G1モデル | `/home/ubuntu/dev/GMR/assets/unitree_g1` のメッシュをコピー。最終プレビューはUnitree公式MuJoCo `g1_29dof.xml`を使用 |
| 有線NIC | `enp129s0`、`192.168.123.200/24`。LowStateとarm_sdkのParticipantはいずれも `Hostname=Unitree`、unicast IP `.161` を自己申告。arm_sdk発行元はG1本体側が最有力。ただしDDSの自己申告metadataでありpacket headerによる独立確認ではない |
| 実機受信 | `rt/lowstate`、`mode_machine=5`、`mode_pr=0`、更新するtick、腕motorstate=0を受信 |

本作業は既存2リポジトリのファイル、OS、ドライバ、CUDA、ROS、既存SDKを変更していません。Python依存、pip/HF/tmp等のキャッシュはプロジェクト内です。依存の正確な一覧は [requirements.lock.txt](requirements.lock.txt)、環境実測は [output/environment.json](output/environment.json)。現在の総使用量は約0.5 GBです。

調査中、公式SDKの既定「config tracing」でCycloneDDSがnative abortする既知問題を再現しました。公式初期化が既定の `/tmp/cdds.LOG` を出力した点は保存先制約の例外になってしまいました。現行実装は**このプロセス内だけでtracingを無効化**し、プロジェクト内のCライブラリを使用します。その後、receive-only接続が成功しています。SDK原本やシステムの修正はしていません。

既存制御参考：`/home/ubuntu/unitree_sdk2_python/example/g1/high_level/g1_arm7_sdk_dds_example.py`、`<integrated_demo>/g1-bottle-reaction/src/g1_bottle_reaction/adapters/g1_robot.py`、既存のread-only DDS診断と `docs/G1_CAMERA_WIRED.md`。

## 2. MotionDecodeの調査・仕様・相違点

取得元は [CMRobot/MotionDecode](https://huggingface.co/datasets/CMRobot/MotionDecode)。取得時revisionは **`988decc8724ad686ace4b003851506e694000acc`**。サンプルはこのrevisionに固定し、SHA-256を記録しました。

- `metadata/index.csv` は**482分類のtaxonomy**。takeごとのFPS・duration・説明・quality flagはありません。
- HF APIのファイル一覧も取得して分類番号と実パスを対応付けました。約95,000ファイルの**名前一覧のみ**で、データ全体をダウンロードしていません。
- 実軌道は10 CSV、合計約15 MBだけ取得。[downloads.json](data/metadata/downloads.json)が正確なパス・revision・サイズ・hashの一覧です。
- 優先指定のAttention Indication、Attention Switching、Gesture Communication、Emotional Expressions、Calling/Respondingを調査。Thinking/Hesitating（1.7.7.3）とJoy/Excitement（1.7.7.1）はtaxonomyにありますが、このrevisionの対応samplesが0件でした。Confusion、Attentive Listening、Excitementなどを代替調査しました。
- 例：taxonomyの`Attention_Indication_and_Eye_Contact`と実ディレクトリの`Attention_Pointing_And_Gaze`は同じ番号1.7.6.2です。名前だけの検索では取りこぼします。

### CSVの確認結果

各行は1 frame、ヘッダ付き36列です。読み込みでは列名を照合して並べ直すため、単なる列番号への依存はありません。

| 元CSVの0始まり列 | 内容 |
|---|---|
| 0〜2 | `root_pos_x(m), root_pos_y(m), root_pos_z(m)`：m |
| 3〜6 | `root_rot_w, root_rot_x, root_rot_y, root_rot_z`：wxyz quaternion |
| 7〜35 | `dof_<joint_name>_joint(rad)`：29関節、rad |

**元CSVのFPSは未確定です。** [配布README](https://huggingface.co/datasets/CMRobot/MotionDecode/blob/main/README.md)は全体・光学収録・BVHについて120 Hz/FPSとしますが、retargeted CSVごとのsampling rateは示しません。[Beyond-Capture Viewer](https://github.com/Beyond-Capture/MotionDecode-Viewer)の既定値は30 FPSです。CSVにはtimestampやFPS欄がなく、metadata・更新履歴・公式BVHプレイヤーもCSVのFPSを確定する証拠になりませんでした。

このため `inspect_motion.py` は既定でduration/角速度rad/sをUNKNOWNとし、rad/frameと30/120 FPSの場合の参考値を示します。`--fps` は**仮定を明記した比較計算**であり、FPSの認証ではありません。元CSVプレビューの `--display-fps 120` も表示上の時間設定です。

**実機用は元FPSを使いません。** フレーム番号で区間を切り出し、`--duration 2` で新しい時間軸を明示して作り直しています。これは元の時間を再現する再生ではなく、抽出したポーズ列の低速な再利用です。出力CSVの `time_s` は確定値で、0.4倍なら5秒になります。

さらに、READMEのroot位置「mm」と実CSVヘッダ・Viewerの「m」が食い違います。実CSVの名前と値に合わせてmを採用。MuJoCoにはwxyzをそのまま渡します。範囲超過のクリップ・±πのunwrap・不明な単位の自動補正は行いません。

## 3. ゲーム向け候補10本

全て末尾は `_00001`。実ファイル名は以下の通りです。意味づけは今回のゲームへの提案で、配布元のtake説明ではありません。**元のdurationは全て不明**。各候補の関節最小/最大・変化量は `output/*_inspection.json`、12コマ画像は `output/*_sheet.jpg` にあります。

| モーション名 | frame数 | rootの初期位置から最大距離 | ゲームへの用途・見た目と判断 |
|---|---:|---:|---|
| **SII_Direction_Pointing_00001** | 2251 | 0.071 m | 「そこだ／何かいた？」。右腕を上げて示す。初動だけ選定。後半の大きな挙上は使用しない |
| DL_Attention_Pointing_And_Gaze_00001 | 1849 | 0.155 m | 「あれ？」。腕を外へ出すが、全編には大きな腰回転もある |
| DL_Call_Response_Attention_Switch_00001 | 1826 | 0.507 m | 「何かいた？」。向き替え・体重移動が混ざり、腕単独の意味は弱め |
| DL_Gesture_Communication_00001 | 8239 | 2.985 m | 「こちら／そこ」。多様な身振りと大きな位置移動。追加の区間選定が必要 |
| DL_Emotional_Expression_00001 | 5424 | 1.751 m | 「びっくり」。しゃがみ等を含み、最大5.544 rad/frameの腕不連続。初回不採用 |
| EESB_Confusion_00001 | 2793 | 0.434 m | 「困った／気のせい？」。頭へ手を寄せる、腰に手を当てる。接触動作が多く不採用 |
| EESB_Attentive_Listening_00001 | 2700 | 0.371 m | 「何か聞こえた？」。主に全身の傾き・体重移動。腕だけでは伝わりにくい |
| EESB_Excitement_00001 | 3212 | 0.699 m | 「見つけた！」。大きな挙上、姿勢変化、不連続1.797 rad/frame。初回不採用 |
| EESB_Observing_Surroundings_00001 | 10421 | 1.182 m | 「見回す」。手を額へ上げ、移動もある。頭に近いので不採用 |
| EESB_Frustration_00001 | 5926 | 0.573 m | 「困った」。腕の表現はあるが低い姿勢・座り相当を含む。初回不採用 |

この10本の全編に「脚が全く動かない」と判断できるものはありません。全身再生は実装していません。

## 4. 最新q0起点の自動探索

receive-only ownership観測 [arm_control_ownership_20260912_212110.json](output/arm_control_ownership_20260912_212110.json) と同時取得LowStateを使いました。右腕q0は `[0.152152, -0.067363, -0.286902, 1.435914, -0.054852, -0.008197, -0.108985]` radです。q0には15 mm未満のpairが9組あり、そのうち `right_wrist_yaw_link / right_hip_pitch_link` だけが約 **−0.444 mm** のpenetrationです。これはこのq0限定のbaselineで、恒久ignoreにはしていません。

全10 CSV・44,641 frameで右腕q0距離を計算し、開始frameの最大差1.0 rad以内から61/91/121 frame windowと1.0/1.5/2.0/2.5/3.0秒の明示retimeを組み合わせました。生成142,170件、動きの大きさと連続性を通ったpre-score対象84,035件、各CSV最大3件の非重複候補を詳細検査した結果は26件です。`SII_Direction_Pointing_00001` は最新q0から1.0 rad以内の開始frameが0件でした。

厳格PASSは1件です。

| 項目 | `EESB_Frustration_00001 [5552,5613)` |
|---|---:|
| ゲーム用途としてのこちらの分類 | confusion / 「おかしいな」 |
| 明示reaction時間 | 3.0 s |
| q0からclip全体の最大関節差 | 0.700 rad |
| entry / return | 4.378 / 5.176 s |
| 最大速度 / 加速度 | 0.209 rad/s / 0.931 rad/s² |
| 右腕reaction幅 | 0.224 rad（手首だけではなく肩・肘） |
| 判定対象pair最小clearance entry / clip / return | 15.468 / 15.468 / 15.468 mm |
| 新規penetration / baseline悪化 | 0 / 0 |
| 安全判定 | `READY <=0.8 rad` |

raw最小clearanceはentry/returnでq0の既存 −0.444 mm、clipで動かさない左腕の既存8.605 mmです。安全判定は、これらが悪化しないこと、右腕側のbaseline pairが初動で全て改善すること、clip中に右腕側が15 mmを満たすこと、新しいpairが15 mmを下回らないことを別々に確認しています。

ランキングの全詳細は [JSON](output/reaction_search_ranked_20260912_212110.json) と [CSV](output/reaction_search_ranked_20260912_212110.csv)。安全PASS候補のentry/reaction/return動画は [比較ページ](output/reaction_candidates/index.html) と [rank01.mp4](output/reaction_candidates/rank01.mp4) です。

## 5. 以前の固定候補と加工内容

`SII_Direction_Pointing_00001.csv` の **0始まり `[180,271)` = 91 frame**。元軌道全編の脚総変化量18.16 rad、腕42.82 radで、調べた10本ではroot移動が最小でした。ただしこれだけで安全とは判定していません。

区間を切り出し、11 frameの対称移動平均（端はedge padding）をかけ、quintic smoothstepで時間位相を開始・終了時に緩め、**2秒・50 Hz・101 frame**へ再標本化しました。手首の揺れを抑えるための処理で、元ポーズ列を加工した派生軌道です。

右肩pitchの変化は約0.312 rad（17.9°）、右肘約0.272 rad（15.6°）。0.4倍での右腕最大角速度は **0.1663 rad/s**、最大差分加速度は **0.8127 rad/s²**。これらは確定した出力時間軸で算出しています。

使用ファイル：[output/selected_arms.csv](output/selected_arms.csv)、根拠・mapping・hash：[output/selected_arms.json](output/selected_arms.json)。

## 6. PC上のプレビュー結果

[プレビュー一覧（ローカルHTML）](output/preview.html)で選定動画と10候補の画像をまとめて確認できます。

[右腕の初回候補・0.4倍速MP4](output/selected_arms_speed04.mp4) / [12コマ画像](output/selected_arms_sheet.jpg)。脚と腰を固定したFK表示です。左腕も表示のために最初のポーズへ固定します。実機の左腕は指令しません。映像は選定区間のみで、実機の現在姿勢からの出入り・制御権の切り替えは含みません。

元区間のroot最大変位は約21 mm、脚の最大関節幅0.167 rad、腰0.148 rad。完全静止ではないため、それらは抽出・送信対象にしません。腕を小さく前へ持ち上げる見た目で、頭へ手を運ぶ区間や大きな指差しの保持区間を避けています。

公式モデルで選定軌道101 frameを検査した最小距離は約 **62.7 mm**（名目姿勢）。必要距離15 mmを上回りました。実機出入りの検査は50 Hzで補間後の経路全体を検査します。

21:09の実測q0では `right_wrist_yaw_link / right_hip_pitch_link` が約 **−0.456 mm**。このq0だけのbaseline contactとして記録しましたが、開始姿勢への移行中にbaseline pairが悪化し、`right_rubber_hand / right_hip_roll_link` は最小約 **−13.40 mm**になりました。q0ではpenetrationしていなかった5 pairが移行中に新規penetrationし、復帰中も最小約 **−3.89 mm**です。baseline contact aware判定でも開始・復帰ともFAILです。clip本体はPASSし、既存baseline pairを除いた実行形の必要最小距離は約15.47 mmです。

検査対象は前腕・手首・手と、頭・胴体・骨盤・左右の脚上部・反対側の腕。手首同士など同じ腕の連続する部品の筐体重なりは対象外です。convex mesh近似・FKであり、物理追従・転倒・把持物・指先・外部障害物の検証ではありません。

モデルの注意：GMRのモデルと取得した公式MuJoCoモデルは、取得した公式URDFに対して肩originのzが10 mm異なります。CSV→SDKの**関節名・軸・index**照合とは分けて記録し、最終表示は公式MJCFを使いました。実機型式・手・外装との一致は現場で確認が必要です。

## 7. 実機用変換・制御と非常停止

29関節のmappingを、**公式Python arm7サンプルのクラス定義をASTで読み、公式URDFの関節名と照合**して作成します。サンプルを実行してmappingを取得する方式ではありません。[公式DDS index表](https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/g1/g1_joint_index_dds.md)の29 DOF欄とも一致します。23 DOF欄には別の並びがあるため、23 DOF機は拒否します。

元CSV：左脚SDK0〜5、右脚6〜11、腰yaw/roll/pitchが12〜14、左腕15〜21、右腕22〜28。CSVのjoint部分はこの順序でしたが、実装は常に名前で対応付けます。[全29関節とCSV列の対応](output/joint_mapping.json)。

| 関節 | 元CSV列（0始まり） | SDK index | 初回送信 |
|---|---:|---:|---|
| 左shoulder pitch / roll / yaw | 22 / 23 / 24 | 15 / 16 / 17 | しない |
| 左elbow | 25 | 18 | しない |
| 左wrist roll / pitch / yaw | 26 / 27 / 28 | 19 / 20 / 21 | しない |
| 右shoulder pitch / roll / yaw | 29 / 30 / 31 | 22 / 23 / 24 | する |
| 右elbow | 32 | 25 | する |
| 右wrist roll / pitch / yaw | 33 / 34 / 35 | 26 / 27 / 28 | する |

通信は公式high-levelサンプルと同じ **`rt/arm_sdk`**。`rt/lowcmd`、歩行RPC、MotionSwitcher、全身low-level制御は使用しません。motor_cmdの右腕以外は初期値（gain=0）に保ち、index29だけをarm_sdk weightとして使います。SDKメッセージには全motor分の欄がありますが、脚・腰・左腕の軌道は設定しません。

通常は **現在の静止腕姿勢を取得 → 同じ姿勢でweightを5秒で上げる → 開始姿勢へquintic補間 → 5秒のclip → 取得時の姿勢へ戻る → weightを5秒で下げる → writerを閉じる** の順です。取得姿勢をneutralとしてよいことを現場で確認します。固定のゼロ角neutralを勝手に押し付けません。

開始・復帰補間は最低2秒、関節速度0.25 rad/s以下になるよう自動延長します。20:09の姿勢で計算した参考値は開始7.89秒、clip5秒、復帰7.60秒、weight移行を含め約30.49秒。ただしその経路は不合格なので実行しません。

Kp=60、Kd=1.5、dq=0、tau=0は公式arm7 Pythonサンプルの値。独自にgainを増やしていません。これらも全てのG1で安全を保証する値ではありません。

主な実行拒否・停止条件：NaN/Inf、列の不一致、hash不一致、公式角度範囲から30 mrad以内への接近、clip幅0.5 rad超、速度0.25 rad/s超、加速度1 rad/s²超、姿勢への移行量1.2 rad超、新規15 mm未満pair、baseline pairの悪化、新規penetration、古いLowState、tick停止、複数LowState送信元、未知arm_sdk/armsdk writer、既知内部arm_sdkのactive traffic、Arm Action active/unknown、非静止、過大な傾き/角速度、motor error/高温、追従誤差、脚・腰・左腕の変化、100 msを超えるループ遅れ。

`rt/arm_sdk`（underscoreあり）はMotion modeのuser arm command blend入力です。G1内部PID 2934の既知publisherは、存在だけでは競合とせず、sampleが流れた場合にACTIVEとして拒否します。`rt/armsdk`（underscoreなし）はArm Action service入力で、G1内部PID 2934のsubscriberは正常です。未知publisherがあれば拒否します。`rt/arm/action/state` は公式IDLの `std_msgs::msg::dds_::String_` で読み、観測済みの `holding/id/name` JSON形だけをdecodeします。

実行時も現在姿勢と次の目標について衝突距離を毎周期確認します。通常復帰も同じ監視下です。**Ctrl+C/例外では次の軌道を止め、zero gain・weight=0を3回送る解放を試みます**。異常時にneutralへ動かし続けることは避けています。通信断、native crash、電源断、SIGKILLにはPythonのfinallyが効かず、物理停止手段が必要です。解放の受理を確認するACKはありません。

## 8. 実行コマンド

別のターミナルで次を実行してください。全スクリプトの出力先は本プロジェクト内に制限しています。

```bash
cd <integrated_demo>/motiondecode-test
source scripts/activate.sh

# 検索：taxonomyのみ存在する分類も明示
python scripts/search_motions.py --query hesitation
python scripts/search_motions.py --reaction suspicious

# 元CSV検査：元FPSを仮定しない
python scripts/inspect_motion.py data/motions/SII_Direction_Pointing_00001.csv

# 元の全身軌道をPCで見る（120は表示の設定で、元FPS確定値ではない）
python scripts/preview_motion.py data/motions/SII_Direction_Pointing_00001.csv \
  --display-fps 120 --mp4 output/direction_original.mp4

# 選定区間の腕抽出を再生成
python scripts/extract_arm_motion.py data/motions/SII_Direction_Pointing_00001.csv \
  --start-frame 180 --end-frame 271 --duration 2 --smoothing-window 11 \
  --output output/selected_arms.csv

# 初回の右腕だけをPC表示
python scripts/preview_motion.py output/selected_arms.csv --arms --right-only \
  --speed 0.4 --mp4 output/selected_arms_speed04.mp4 --sheet output/selected_arms_sheet.jpg

# 完全オフライン：SDK importもDDS初期化もしない
python scripts/play_g1_arms.py output/selected_arms.csv --speed 0.4 --dry-run

# 実機へ動作commandを送らない、3秒のreceive-only接続確認
python scripts/check_g1_connection.py --network-interface enp129s0 --seconds 3

# 3 topicを15〜30秒receive-only観測し、同じ観測のLowStateも別名保存
python scripts/inspect_arm_sdk_publishers.py \
  --network-interface enp129s0 --seconds 30 \
  --state-json output/connection_ownership_YYYYMMDD_HHMMSS.json

# 上のownership reportと同時取得したLowStateを使うcombined offline preflight
python scripts/evaluate_g1_preflight.py output/selected_arms.csv --speed 0.4 \
  --state-json output/connection_ownership_YYYYMMDD_HHMMSS.json \
  --ownership-json output/arm_control_ownership_YYYYMMDD_HHMMSS.json

# 最新q0から10 CSVの全frameを段階探索し、上位だけを詳細collision検査
python scripts/search_reactions.py \
  --state-json output/connection_search_YYYYMMDD_HHMMSS.json \
  --ownership-json output/arm_control_ownership_YYYYMMDD_HHMMSS.json \
  --stamp YYYYMMDD_HHMMSS

# 厳格PASS上位5件までのentry/reaction/returnをMuJoCo表示
python scripts/render_reaction_previews.py \
  --ranked-json output/reaction_search_ranked_YYYYMMDD_HHMMSS.json \
  --state-json output/connection_search_YYYYMMDD_HHMMSS.json \
  --output-dir output/reaction_candidates

# 受信姿勢での経路検査。これはオフライン、不合格なら終了コード2
python scripts/play_g1_arms.py output/selected_arms.csv --speed 0.4 --dry-run \
  --state-json output/connection.json

# 現場で安定立位・安全なneutral・制御の競合解消を確認した後のみ
# 現状の姿勢/競合では拒否される。安全判定を外して実行しない。
python scripts/play_g1_arms.py output/selected_arms.csv --speed 0.4 \
  --network-interface enp129s0 --enable-real-robot
```

実機フラグを付けてもすぐには動きません。receive-only preflight、経路検査、時間・joint・リスクの表示後に、現場で `ARMS` と入力した場合だけwriterを作ります。入力後も姿勢変更・競合を再確認します。`--state-json` と実機フラグは併用不可です。フラグなしもdry-run、dry-runと実機フラグの同時指定は拒否します。

対話型MuJoCo表示には `--mp4 ...` の代わりに `--interactive` を指定できます。ウィンドウを開けない場合は生成済みMP4/画像を使用してください。

初回の依存導入は既に完了しています。再構築時のみ `/usr/bin/python3 -m venv .venv`、`source scripts/activate.sh` 後に `python -m pip install -r requirements.lock.txt`。CycloneDDS bindingの再ビルドにはCヘッダが必要なので、環境を消してから再構築する前に `external/cyclonedds` のincludeを確認してください。現在のvenvはそのまま使えます。データの再取得・検証は `python scripts/download_candidates.py`（10件上限）。

## 9. 検証結果・残るリスク

- テスト26件通過。列順をシャッフルした入力、NaN/Inf、quaternion、改変ファイル、速度・角度拒否、default/dry-runのネットワーク禁止、右腕以外の有効commandゼロ、補間、共有retimeの明示時間と端点減速、現在姿勢拒否、Arm Actionの厳格decode、stale state、Ctrl+C/異常終了時解放、正常終了時weight解放を確認。
- [output/dry_run.txt](output/dry_run.txt)：候補単体のdry-run通過。
- [output/preflight_saved_state.json](output/preflight_saved_state.json)：20:09の実機姿勢からの経路は不合格。
- [output/connection_discovery.json](output/connection_discovery.json)：20:20にLowState writer=1、arm_sdk writer=1。こちらが作成したcommand publisherは0。
- [output/arm_sdk_publishers_20260912_204642.json](output/arm_sdk_publishers_20260912_204642.json)：30.043秒のreceive-only調査。LowStateは1 sourceでPASS。arm_sdk writerは1、全111 pollで存続、sample 0、終了時にも存在。endpoint `01106c5f-69d1-55de-cc92-bcf900000403`、participant `01106c5f-69d1-55de-cc92-bcf9000001c1`。関連topicから腕API/arm actionを扱うG1側サービスと推定。Ubuntu側の該当ローカルプロセスは見つからなかった。
- [output/arm_control_ownership_20260912_210935.json](output/arm_control_ownership_20260912_210935.json)：15.041秒の3-topic調査。PID 2934をEXPECTED ROBOT INTERNAL PARTICIPANTに分類。内部arm_sdk 0 sample、外部arm_sdk writer 0、armsdk writer 0、Arm Actionは150 sample/9.973 HzですべてIDLE、ownership PASS。
- [output/combined_preflight_20260912_210935.json](output/combined_preflight_20260912_210935.json)：同一観測のq0による総合offline結果。joint/速度/加速度/clipはPASS。移行量1.287 rad、baseline悪化、新規penetration、entry/return collisionによりREAL ROBOT TEST BLOCKED。
- [output/current_pose_search_20260912_212110.json](output/current_pose_search_20260912_212110.json)：21:21のownership PASSと同時取得したq0、左右腕・腰・脚・attitude・q0 baseline pair。
- [output/reaction_search_ranked_20260912_212110.json](output/reaction_search_ranked_20260912_212110.json)：10 motionの段階探索。厳格PASS 1件、0.9/1.0 rad参考帯0件、`REAL ROBOT CANDIDATE FOUND`。command publisher 0。
- PC検証は運動学だけ。実機追従、ゲイン適合、立位balance、指/手型/装着物、床との実距離、周囲、人、ケーブル、ネットワーク断でのファームウェア挙動は未検証。
- 元FPS未確定。今回の出力は明示的にretimeした派生軌道で、元モーションの再生速度に対する0.4倍という意味ではありません。
- 未知writerは停止中でも競合判定します。唯一の例外は、IP・hostname・process・PID・関連topicが全て一致した既知G1内部Arm serviceで、これもsampleが流れればACTIVEとして拒否します。誰が所有しているか不明なプロセスを自動で停止したり、制御を奪ったりしません。
- 配布元の [LICENSE.md](https://huggingface.co/datasets/CMRobot/MotionDecode/blob/main/LICENSE.md) はacademic research/personal study/non-commercial prototypingを許容し、商用配布等は書面の許諾を要求しています。今回のローカル検証から商用ゲームへ進める際は条件を確認してください。データ由来の表記は **Chingmu / CMRobot MotionDecode**。Viewerコード、SDK、ロボットモデルのライセンスはそれぞれ別です。

## 10. 次のステップ

まず [rank01.mp4](output/reaction_candidates/rank01.mp4) を確認し、右腕を外へ上げて戻す表現がゲーム用途に合うか判断します。今回はここで停止し、実機commandは送りません。

実機試験へ進める場合も、保存済みq0でそのまま実行しません。直前にownership/LowStateを新しいtimestampで再取得し、同じ区間をその新q0から再評価します。q0の姿勢が変われば今回のPASSは無効です。現場でG1の安定立位、周囲、手・外装、物理停止手段を確認した後、専用の実行artifactへ昇格させます。

## 構成と根拠ファイル

`.venv/`、`.cache/`、`data/metadata/`、`data/motions/`、`output/`、`external/`、`scripts/`、`tests/`を使用します。各スクリプトは `--help` で引数を確認できます。

- [公式Python arm7サンプル](https://github.com/unitreerobotics/unitree_sdk2_python/blob/65691c8a8bc53b98d3976dba4dbf9d5d20b2e7f5/example/g1/high_level/g1_arm7_sdk_dds_example.py)：joint index、arm_sdk、Kp/Kd、weight。保存先 `external/g1_arm7_sdk_dds_example.py`。
- [公式G1 URDF](https://github.com/unitreerobotics/unitree_ros/blob/master/robots/g1_description/g1_29dof.urdf)：関節名・軸・角度範囲。保存先 `external/g1_official_29dof.urdf`。
- [公式MuJoCoモデル](https://github.com/unitreerobotics/unitree_mujoco/blob/main/unitree_robots/g1/g1_29dof.xml)：最終FKモデル。保存先 `external/g1_official_29dof.xml`。
- [Beyond-Capture Viewerコード](https://github.com/Beyond-Capture/MotionDecode-Viewer/blob/master/csv_to_animation.py)：root wxyz、列名、既定fps。参考として保存、実行依存ではありません。
- `external/MotionDecode_README.md`、`MotionDecode_Log.md`、`MotionDecode_LICENSE.md`、`Viewer_README.md`を保存。コード・資料のhashは `output/source_hashes.json`。

実機動作の確認完了を示すログはまだありません。
