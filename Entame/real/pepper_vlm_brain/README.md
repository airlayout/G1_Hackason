# Pepper VLM Brain — Hackathon snapshot

このdirectoryはPepper向けVLM BrainのHackathon snapshotです。G1の統合デモとは独立した、画像認識・判断・Pepper連携のソースを収録しています。

## Source

- Source repository: [IharaTakumi/robot-vlm-brain](https://github.com/IharaTakumi/robot-vlm-brain)
- Source branch: `main`
- Source commit SHA: `8baedfbff570407faf1ac65b9bd0ac0f5760a172`
- Snapshot作成日: 2026-10-07（JST）
- Commit: `feat(pepper): add live VLM perception and reaction loop`

## 現在の主要構成

|構成|役割|
|---|---|
|`brain/`, `config.py`|Qwen3-VLによる画像判断、JSON/schema検証、fallback。Perception・State・Plannerの分離経路も収録|
|`hybrid/`|YOLOによる高速な人物geometry更新と、独立したQwen workerによる低頻度の意味認識・Planner。最新frameを優先|
|`adapters/pepper/camera.py`|qi / ALVideoDevice経由のtop camera読取りと画像payload検証|
|`adapters/pepper/motion.py`|dry-run、HTTP bridge、ROS 2のhead/speech transportと指令検証|
|`pepper_bridge_android/`|Kotlin / Android QiSDKのmotion bridge。robot focusとHTTP指令を扱う|
|`latency/`, evaluation scripts, `scenarios/`, `reports/`, `tests/`|遅延比較、保存データの評価、シナリオ、過去の検証記録、テストソース|

## Hackathon environment

ハッカソンではUbuntu PCから利用する前提です。この取り込み作業ではUbuntuセットアップ、モデル推論、Pepper実機の動作確認は実施していません。

`requirements.txt` はWindows CUDA wheelの指定を含み、`requirements-lock.txt` にはWindows上のローカルwheelへの参照があります。`setup_4b_env.ps1` / `setup_hybrid.ps1` 等も過去のWindows環境向けです。これらはsourceのまま保持しており、Ubuntu用インストール手順としては扱えません。UbuntuのPython / CUDA / PyTorch、QwenモデルとYOLO重み、qiまたはROS 2の依存・接続環境は別途確認が必要です。モデル重み、元動画、仮想環境、cacheは同梱していません。

ROS 2 runnerは `/opt/ros/jazzy/setup.bash` と `/tmp/pepper_naoqi_ws.BwBAe7/install2/setup.bash` を参照します。後者は開発環境固有のoverlayで、このsnapshotにはありません。コードを変更していないため、Ubuntu上でそのまま実行可能とは保証しません。HTTP Android bridgeの経路とROS 2 head/speechの経路は別の実装です。

## Pepper関連entry points

|ファイル|役割・実装上の条件|
|---|---|
|`run_pepper_camera.py`|Pepper cameraのsnapshot/probe、YOLO検出、hybrid dry-run。記録用robotへ指令を保存|
|`run_pepper_motion.py`|MOVE / TURN / LOOK / STOPのJSON指令を検証。既定はdry-run。実HTTP送信には `--send` と `--bridge-url` が必要|
|`run_vlm_pepper_head.py`|画像1枚をQwenへ入力し、人物・方向・視覚反応のgateを通った判断をLOOKと発話へ対応付け。ROS 2送信は `--send` で有効化|
|`capture_pepper_ros_image.py`|ROS 2から画像・joint stateを取得。live runnerのcapture workerとして使用|
|`run_vlm_pepper_live.py`|最新camera frame → Qwen → head / speechの反応loop。人物・方向・salient factの変化、cooldown、head角度閾値で反応を制御。実LOOK / SPEAKは `--send` で有効化|

## 既存の検証記録と範囲

[reports/PHASE33.md](reports/PHASE33.md) はPC上のSplit Brain主要3画像のPASSと、拡張9画像の方向誤認による8/9 FAILを記録しています。[reports/PHASE34.md](reports/PHASE34.md) は遅延profile比較、[reports/PHASE36.md](reports/PHASE36.md) は動画simulationでのYOLO / Qwen hybrid検証を記録しています。PHASE36の `NOT READY` はその過去のsimulation時点の評価です。

最新commitにはPepper camera / motion adapter、Android bridge、head / live runnerがあります。実装の存在と、実機・Ubuntuでの動作確認は区別してください。このsnapshotには最新live loopの実機成功を証明する新たな検証結果は追加していません。過去reportのPASSは記載された条件の結果であり、現在の全経路の保証ではありません。

## Provenance

source commitのtracked files 395件を `git archive` によりsnapshotとして取り込みました。別repositoryのGit historyはmergeしていません。コード本体・設定・requirements・tests・既存reportsはsource commitから変更していません。このdirectory内の意図的な変更は本 `README.md` のみです。sourceのローカル未コミット変更やuntracked filesは取り込んでいません。

README以外の全394ファイルをsource archiveとbyte単位で比較し、一致を確認しました。pytest / full testはコード未変更のため今回実施しません。

## Historical / Windows development notes

元READMEのWindows PowerShell setupやGPU検証の記録は、[source commitのREADME](https://github.com/IharaTakumi/robot-vlm-brain/blob/8baedfbff570407faf1ac65b9bd0ac0f5760a172/README.md) を参照してください。収録されたPowerShell scriptsとrequirementsは履歴の再現用として保持しています。Windowsのpathをbashへ置き換えたUbuntu手順は作成していません。
