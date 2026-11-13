# Voice（音声・音声対話）

G1に喋らせる・G1のマイクで聞き取る機能全般。`SimpleWalk/`・`Mapping/`等と同じ
「機能ドメインごとにトップレベル1つ」の構成に合わせて新設。

音声はsim側で検証する意味が薄い（マイク・スピーカーの実機挙動そのものが対象のため）
ので、`sim/`・`real/`のような分割はせず、**このフォルダ直下にコードを置く**。

## 「手を振って」で SDK のプリセットを動かす

手振り専用の `configs/tasks.wave.json` と `configs/tools.wave.json` を追加した。
G1 内蔵マイク → 音量による区間抽出 → EmbeddingGemma 2 → `wave_hand`
→ 汎用ツール `sdk_arm_action(preset="high wave")` → SDK の `ExecuteAction(26)` とつながる。
この設定では巡回の開始を登録せず、**Nav2 / ROS を起動せずに**動作確認できる。
既定の巡回設定を使う場合は、引き続き次節の手順に従う。

### G1 上で準備する

モデルの配置と推論 venv は[次節の環境構築](#g1-の環境と登録音声を準備する)と共通。
ツールサーバー用には ROS を含まない専用 venv を用意する:

```bash
cd /home/unitree/G1_Hackason
python3 -m venv /tmp/g1-voice-wave-tools
/tmp/g1-voice-wave-tools/bin/python -m pip install -r Voice/requirements-tools.txt
```

SDK 呼び出しは別プロセスで行い、`VOICE_SDK_PYTHON` には `unitree_sdk2py` が入っている
G1 上の Python を指定する。既定は `/usr/bin/python3`。共有 Python への追加インストールは行わない。
`VOICE_SDK_INTERFACE` は既定で `eth0`。

巡回・歩行・他の腕制御を終了し、停止中の機体を既存の手順で SDK の腕動作を受け付ける状態にする。
呼び出し口は FSM ID が 500 / 501 / 801 の場合だけ要求を送り、SDK の拒否も失敗として返す。
**制御モードの選択・解除、起立、走行許可の変更は行わない。**
FSM の検査は静止状態や他の制御主体の不在を保証するものではないため、これらは試験前に確認する。

### 登録音声を収録し、dry-run で確認する

まず説明文の設定で起動する。この段階ではツールサーバーも SDK も不要:

```bash
/tmp/g1-voice-infer/bin/python Voice/onboard_voice.py \
  --model /home/unitree/models/embeddinggemma-2 --device cpu \
  --config Voice/configs/tasks.wave.json --tools Voice/configs/tools.wave.json \
  --output-dir _local/voice/wave-reference-runs
```

待受表示のあと「手を振って」「手を振ってください」「手を振らないで」「こんにちは」を、各判定ログを待って収録する。
Ctrl+C で終了し、保存された各 `input.wav` を確認する。
`configs/tasks.wave.audio.example.json` を `_local/voice/tasks.wave.json` にコピーし、
`reference_audio` をそれぞれ手振り 2 種類・否定・雑談の WAV の絶対パスに変更する。
説明文だけの照合では日本語の肯定・否定が近いスコアになった記録があるため、
実行前に登録に使っていない録音で判定を確認し、しきい値を調整する。

上記コマンドの `--config` を `_local/voice/tasks.wave.json` に変えて再起動する。
「手を振って」で `wave_hand` / `dry_run`、否定や雑談で `rejected` になることを確認する。
保存済み WAV でも `--replay /path/to/test.wav` を追加して確認できる。

### 音声から実行する

別々の G1 上のターミナルで、ツールサーバーと音声プロセスを起動する。
どちらもリポジトリ直下から実行し、既存の音声・ツールサーバーは先に終了する:

```bash
VOICE_SDK_PYTHON=/usr/bin/python3 VOICE_SDK_INTERFACE=eth0 \
  /tmp/g1-voice-wave-tools/bin/python Voice/tool_server.py \
  --config Voice/configs/tools.wave.json --execute
```

```bash
/tmp/g1-voice-infer/bin/python Voice/onboard_voice.py \
  --model /home/unitree/models/embeddinggemma-2 --device cpu \
  --config _local/voice/tasks.wave.json --tools Voice/configs/tools.wave.json --execute
```

「手を振って」と話すと、登録した `high wave` だけを呼ぶ。
`--execute` はサーバーと音声プロセスの両方に必要。手振り専用設定では
`deploy/run_tools.sh` と巡回用の systemd unit は使わない。

SDK の応答後も 5 秒間は同じ実行グループと腕ツールのプロセス間ロックを保持し、連続実行を抑える。
ツール結果の `process_exited` はプロセス終了、子の JSON の `accepted` は SDK の受理を表し、
物理動作の完了通知ではない。Ctrl+C は待受・プロセスを終了するが、受理済みプリセットの停止は保証しない。
ログは既定の `_local/voice/onboard-runs/` に保存する。

`Entame/real/arm_wave_real.py` の既存の手振り実績をもとにしているが、
この音声経路の G1 実機確認は未実施。テストでは SDK の境界を置き換え、実機へは送信しない。
実モデルでも登録した合成音声から `wave_hand` → dry-run を確認した。
未登録の言い換えを見送るケースがあり、実録音による分類精度は未検証。

## G1 本体で常時受信し、巡回を開始する

`onboard_voice.py` は、**G1 の PC2 上だけで音声入力からツール実行まで完結する**構成。
モデル推論用プロセスと ROS ツール実行用プロセスを、同じ G1 の Unix socket で接続する。
推論環境に ROS を入れる必要はなく、ツールサーバーに torch を入れる必要もない。

```text
G1 内蔵マイク（UDP / eth0）
  → 常時受信 → 音量による区間抽出 → キュー（最大1件）
  → 常駐 EmbeddingGemma 2 → タスクID・固定引数
  → Unix socket → ツールサーバー（既存 ROS 環境）
  → /g1/patrol/start → 応答の success と RUNNING 状態を確認
```

巡回用の既定設定に登録しているツールは `patrol_start`。準備済みの巡回を開始・再開する。
既に巡回中なら既存の巡回ノードが再起動せず応答する。
巡回路の用意・地図との位置合わせ・発進ゲート・走行許可は既存の Navigation の手順で準備する。

### 音声区間の切り出し

`configs/segment.json` で変更できる。開始・終了のどちらでも**マイクの受信は止めない**。

| 設定 | 初期値 | 意味 |
|---|---|---|
| `threshold_dbfs` | -35 | RMS 音量の閾値（0 dBFS がフルスケール） |
| `frame_ms` | 20 | 音量を評価する単位 |
| `start_ms` | 100 | 閾値以上が続いたら区間を開始 |
| `end_ms` | 600 | 閾値未満が続いたら区間を確定し、推論する |
| `pre_roll_ms` | 300 | 語頭を落とさないため開始前も含める |
| `min_voiced_ms` | 300 | 短い物音を除くため必要な発声時間 |
| `max_ms` | 15000 | 長い連続音は破棄し、無音になるまで次を作らない |

終了判定の無音はほぼ除いてベクトル化する。100ms を超える受信途切れでは部分区間を破棄する。
推論中も受信は続け、待ち行列には最新の 1 件だけを残す。`--max-age`（既定 5 秒）を過ぎた区間は
推論前・推論後に見送り、サーバーでも実行期限を確認する。同じツールの連続要求は
`--cooldown`（既定 2 秒）で抑える。音量だけでは音声・環境音を確実に区別できないため、
タスク照合と見送り判定も通す。

### G1 の環境と登録音声を準備する

推論環境は Python 3.11 以上（推奨 3.12）で `requirements.txt` を入れる。
G1 は ARM64 のため、JetPack / CUDA と PyTorch の組み合わせを実機で確認する。
まず `--device cpu` で通し、CUDA は対応する torch を用意した環境で測定する。
共有の Python を変更せず、専用 venv を使う。以下の配置先は例で、G1 の実際のパスに合わせる。

```bash
cd /home/unitree/G1_Hackason
python3.12 -m venv /tmp/g1-voice-infer
/tmp/g1-voice-infer/bin/python -m pip install -r Voice/requirements.txt
```

モデルは準備時に取得して G1 に置く。**常駐処理は既定で local_files_only=True** とし、
運用中に外部 API やモデルのダウンロードを必要としない。

```bash
/tmp/g1-voice-infer/bin/python -c 'from huggingface_hub import snapshot_download; snapshot_download("google/embeddinggemma-2", local_dir="/home/unitree/models/embeddinggemma-2")'
```

登録音声は G1 上で常時受信を dry-run 起動して収録できる。まず説明文だけの設定を使う:

```bash
/tmp/g1-voice-infer/bin/python Voice/onboard_voice.py \
  --model /home/unitree/models/embeddinggemma-2 --device cpu \
  --config Voice/configs/tasks.patrol.json --output-dir _local/voice/reference-runs
```

待受表示のあと「巡回を開始して」と話し、判定ログが出てから「巡回を始めないで／待っていて」と話す。
各区間は判定結果にかかわらず `_local/voice/reference-runs/<実行ID>/input.wav` に保存される。
処理が終わったら Ctrl+C で終了し、録音を確認して開始・見送りの WAV を選ぶ。
`configs/tasks.patrol.audio.example.json` を `_local/voice/tasks.patrol.json` にコピーし、
各 `reference_audio` を選んだ WAV の絶対パスに変更する（相対パスは設定ファイルから解決する）。
登録音声に使っていない録音で、肯定・否定・言い換え・雑談のスコアを検証してしきい値を調整する。
既定の `tasks.patrol.json` は説明文だけの最小設定で、実機での認識精度を保証するものではない。

### ツールサーバーと常時受信を起動する

Navigation 側の変更は `navigation.launch.py` と PC2 起動スクリプトの再配置が必要。
既存の [配置・ビルド手順](../Navigation/nav2_stable/deploy/README.md) に従う。

ROS 用 venv は、**既存の pixi の Python から**作り、rclpy を共有する:

```bash
cd /home/unitree/g1_nav2/pc2_humble
~/.pixi/bin/pixi run python -m venv --system-site-packages /tmp/g1-voice-tools
/tmp/g1-voice-tools/bin/python -m pip install -r /home/unitree/G1_Hackason/Voice/requirements-tools.txt
```

G1 上でツールサーバーを起動する（ROS 環境の source はラッパー内で行う）:

```bash
cd /home/unitree/G1_Hackason
VOICE_TOOLS_PYTHON=/tmp/g1-voice-tools/bin/python bash Voice/deploy/run_tools.sh --execute --onboard-heartbeat
```

別の G1 上のプロセスで常時受信・推論を起動する:

```bash
/tmp/g1-voice-infer/bin/python Voice/onboard_voice.py \
  --model /home/unitree/models/embeddinggemma-2 --device cpu \
  --config /home/unitree/G1_Hackason/_local/voice/tasks.patrol.json --execute
```

`--execute` を外すと、サーバー接続も実行もせず、音声と選択結果の記録だけを行う。
ログは `_local/voice/onboard-runs/` に WAV・音声ベクトル・判定・ツール結果を保存する。
モデルと登録例のベクトルは起動時に一度読み込む。

### 外部 PC から独立した生存監視

Nav2 は既定では従来の外部操作 PC heartbeat を要求する。オンボード構成を選ぶ場合は、
次の明示的な起動オプションを使う:

```bash
/home/unitree/g1_nav2/g1up.sh --patrol /path/to/route.yaml --supervision onboard --operator-timeout 1.0
```

この指定は **heartbeat_required=true を保ち**、受信先を `127.0.0.1:47312` にする。
従来の `operator` モードの既定ポート 47311 と混在させない。
ツールサーバーは、音声プロセスから 1 秒以内の健全な生存報告が届いている間だけ専用 heartbeat を送る。
健全性はマイクパケットの鮮度・推論ワーカーの生存・処理期限（既定 30 秒）で判定する。
マイク受信の停止・推論停止・IPC 切断・どちらかのプロセス終了で heartbeat が途絶し、既存 Nav2 が FAULT に落とす。
これは外部の人による通信監視から、オンボード処理の監視への**運用モードの変更**。
`operator_heartbeat` / `operator_lost` という既存の診断名はそのまま使われる。

発進ゲート・走行許可・センサー / TF / SDK 監視は従来どおり。FAULT を音声側で自動解除しない。
機体準備ができたあと既存の手順で走行を許可し、そこで初めて音声による `patrol_start` を受け付ける。

### SSH セッションに依存せず動かす場合

`deploy/g1-voice-{tools,listener}.service` は systemd **user** service の例。
`deploy/environment.example` を `~/.config/g1-voice/environment` にコピーし、実際のパスへ変更する。
2 つの unit を `~/.config/systemd/user/` に置いて、`systemctl --user daemon-reload` 後に
`systemctl --user start g1-voice-listener` で起動する。
tools の socket が待受状態になってから listener が開始される。
ログは `journalctl --user -u g1-voice-listener -u g1-voice-tools -f`。

unit の配置・有効化や機体の設定変更はこの実装では行わない。ログアウト後の user manager の生存設定は
G1 側で確認する。`/tmp` の venv は再起動で消えるため、自動起動に使う場合は環境の保存場所も整える。
停止は `systemctl --user stop g1-voice-listener g1-voice-tools`。自動再起動・巡回の自動再開はしない。

### ツールを追加する

音声処理は `tool_runtime.py` の特定のツールを知らず、名前とスキーマ検証済みの引数だけを渡す。

1. `configs/tools.json` に名前・説明・`backend`・接続先・`input_schema`・実行期限・排他グループを追加。
2. タスク設定に、そのツール名を `action`、固定引数を `arguments` として登録し、照合例を追加。

既存の backend は `ros_service` と `command`。例えば巡回一時停止を増やすなら、
`ros_service` に `/g1/patrol/pause` を登録でき、音声側のコード変更は不要。
ROS backend は応答の `success_field` と、任意の `confirm` に指定した状態トピックで結果を確認できる。
`command` は管理者が登録した `argv` を shell=False で実行し、`{引数名}` のトークンにだけ値を渡す。
`command` の `config.cwd` に相対パスを指定すると、ツール設定 JSON のあるディレクトリから解決する。
`tools.wave.json` はこの仕組みでリポジトリの配置場所に依存せず SDK 用ラッパーを呼び出す。
引数スキーマは `additionalProperties: false` とし、必要に応じて enum や数値範囲を制限する。
新しい実行方式は `ToolSpec, arguments -> ToolResult` の callable を backend として登録する。

共通基盤が提供するのは登録・JSON Schema 検証・dry-run・期限・排他・構造化結果・Unix IPC。
`exclusive_group` が同じツールは同時実行せず、待ち行列にも積まない。
`command` はタイムアウトや終了時に子の process group も停止する。
実行にはクライアントとサーバー双方の `--execute` が必要。socket は mode 700 の所有ディレクトリ内に mode 600 で作る。

### 実機なしで確認する

```bash
_local/voice/.venv/bin/python Voice/onboard_voice.py --replay /path/to/instruction.wav --device cpu
PYTHON=_local/voice/.venv/bin/python bash Voice/tests/run_tests.sh
```

`--replay` は実時間で区間検出を試す dry-run 専用で、`--execute` と併用できない。
単発入力でも `voice_command.py --audio ... --config ... --tools Voice/configs/tools.json` で共通ツール基盤を使える。
実モデルによる日本語合成音声の区間抽出→巡回開始の選択→dry-run を開発 PC で確認。
G1 の実マイク・Jetson 推論性能・ROS 実機サービス・停止連動は実機での確認が必要。

## 音声からタスクを選ぶ（EmbeddingGemma 2）

音声波形を **`google/embeddinggemma-2` に直接入力**し、768 次元のベクトルを作る。
登録済みタスクの説明文も同じモデルでベクトル化し、コサイン類似度でタスクを選ぶ。
文字起こしや会話用 LLM は挟まない。

```text
WAV / 操作 PC のマイク → 16 kHz・モノラル → EmbeddingGemma 2
                                               ↓ 音声ベクトル
登録済みタスクの説明文 → 同じモデルのベクトル → 比較・見送り判定 → 既存タスク
```

### 環境の準備

Python 3.11 以上（推奨 3.12）。推論側は専用の venv に依存を入れ、既存のロボット制御環境は
`--robot-python` で指定できる。Docker は使わない。以下はリポジトリ直下で実行する。

```bash
python3.12 -m venv _local/voice/.venv
_local/voice/.venv/bin/python -m pip install -r Voice/requirements.txt
```

CPU 専用にしたい場合は、requirements のインストール前に CPU 版の torch / torchvision を入れる。

```bash
_local/voice/.venv/bin/python -m pip install torch torchvision --index-url https://download.pytorch.org/whl/cpu
```

初回は Hugging Face からモデルを取得するためネット接続が必要。以後はローカルキャッシュを使う。
画像エンコーダの重みは読み込まない。計算は CPU で float32、対応 CUDA GPU では bfloat16 を使う。
float16 は使わない。`AutoProcessor` の構築には、音声のみの場合も torchvision が必要。

### 使い方

音声からベクトルを作って保存するだけなら、`--embed-only` を付ける。

```bash
_local/voice/.venv/bin/python Voice/voice_command.py --audio /path/to/instruction.wav --embed-only --device cpu
```

タスクの照合と実行予定の表示（既定の dry-run。ロボットへの接続も行わない）:

```bash
_local/voice/.venv/bin/python Voice/voice_command.py --list-tasks
_local/voice/.venv/bin/python Voice/voice_command.py --audio /path/to/instruction.wav
```

操作 PC のマイクを 5 秒録音して照合する:

```bash
_local/voice/.venv/bin/python Voice/voice_command.py --record-seconds 5
```

録音は固定時間で明示的に区切る。入力先は `--mic-device "デバイス名"` で変えられる。
PortAudio が必要。G1 本体の UDP multicast マイク受信には、上記の `onboard_voice.py` を使う。
音声ファイルは WAV / FLAC 等に対応し、ステレオや 44.1 / 48 kHz は自動で変換する。
長さは 0.2〜30 秒。無音・非有限値・範囲外の長さはモデルに渡す前に断る。

実機で動かすときだけ `--execute` を付ける。制御側の SDK・接続・機材の準備は、
既存の [ボタン押し実機手順](../Button_Press/J1-gen/real/REAL_DAY_PROCEDURE.md) に従う。

```bash
_local/voice/.venv/bin/python Voice/voice_command.py --audio /path/to/instruction.wav \
  --execute --robot-python G1_HuggingFace/venv/bin/python --network-interface enp3s0
```

`--robot-python` の既定は `G1_HuggingFace/venv/bin/python`。
選択済みのタスクを同期的に 1 件実行し、音声指示経由の同時実行はロックで断る。
Ctrl+C は既存タスクにも届き、その停止処理が終わるまで待つ。

### 登録済みタスクと判定

| タスク ID | 実行処理 |
|---|---|
| `press_bottle` | `auto_press.py --target bottle` |
| `press_down` | `auto_press.py --target down --button-set wheelchair --detector yolo` |
| `no_action` | 実行を見送る（待機・否定・未対応の上ボタン指示・雑談の照合例） |

実行先は `Button_Press/J1-gen/real/auto_press.py`。その既存の事前確認・制御処理を呼び出す。
`configs/tasks.json` にタスク ID・説明文・実行処理名・判定しきい値を登録する。
実行処理を増やすときは `task_actions.py` の `TASK_ARGS` にも対応を追加する。
設定ファイルにはコマンドを記載せず、コードで登録した実行処理名を指定する。

各タスクの説明文のうち最も高い類似度を使い、次の順で判定する:

1. 最高スコアが `min_similarity` 未満なら見送る。
2. 別タスクの 2 位との差が `min_margin` 未満なら、曖昧として見送る。
3. 勝ったタスクの `action` が `null` なら見送る。それ以外は実行候補にする。

**既定のしきい値 0.5 / 0.08 は未校正の初期値。** 日本語の実録音・話者の違い・雑音・
「押す / 押さない」「上 / 下」を含むデータで判定を検証して調整する。
`no_action` の照合例だけで否定や未対応指示を確実に識別できる保証はない。
また、`no_action` は実行中のタスクを停止する機能ではない。

### タスクごとに音声の登録例を使う

日本語の合成音声で確認したところ、説明文だけの照合では、肯定の指示が否定の説明文にも
強く一致し、有効な指示を見送るケースがあった。**ベクトル生成の動作確認と、指示分類の精度は別に扱う。**
説明文だけで実機を動かせる精度を確認済みという意味ではない。

タスクに `reference_audio` を登録すると、そのタスクでは説明文の代わりに登録音声を
EmbeddingGemma 2 でベクトル化し、入力音声と音声同士で比較できる。
パスは設定ファイルからの相対パスか絶対パス。言い換えや複数話者の例を配列に追加できる。

まず「ボトルを押して」「エレベーターの下ボタンを押して」「ボトルを押さないで」を
それぞれ録音する例（既存ファイルは上書きしない）:

```bash
_local/voice/.venv/bin/python Voice/voice_command.py --record-seconds 4 --embed-only --save-audio _local/voice/reference-audio/bottle.wav
_local/voice/.venv/bin/python Voice/voice_command.py --record-seconds 4 --embed-only --save-audio _local/voice/reference-audio/down.wav
_local/voice/.venv/bin/python Voice/voice_command.py --record-seconds 4 --embed-only --save-audio _local/voice/reference-audio/cancel.wav
```

この 3 ファイルを使う設定例は `configs/tasks.audio.example.json` にある:

```bash
_local/voice/.venv/bin/python Voice/voice_command.py --audio /path/to/new-instruction.wav --config Voice/configs/tasks.audio.example.json
```

この設定の 0.75 / 0.08 もしきい値の初期値。登録に使っていない音声で、言い換え・話者の違い・
否定・雑談・無関係な音を検証する。同じ音声を再入力して一致するだけでは精度の検証にはならない。

### 出力と終了コード

`_local/voice/runs/<UTC日時とID>/` に次を保存する（`--output-dir` で保存先を変更できる）。

- `audio_embedding.npy`: L2 正規化した float32、形状 `(768,)` の音声ベクトル。
- `result.json`: モデル ID / revision、使用した設定、各タスクのスコア、選択・見送り理由、実行コマンドとプロセス終了コード。
- `input.wav`: マイク録音時の入力音声。

ベクトルは `numpy.load(path, allow_pickle=False)` で読み込める。
モデルの取得・推論に失敗した場合も、記録フォルダ作成後のエラーは `result.json` に残る。

終了コードは `0`: ベクトル生成・dry-run 完了または子プロセスが 0 で終了、`2`: 入力・環境エラー、
`3`: 判定による見送り、`4`: 子プロセスが非 0 で終了、`130`: 中断。
既存タスクは中止でも 0 を返すことがあるため、子プロセスの終了コードだけでは物理タスクの成功を表さない。
動作結果は既存の `_local/button_press/auto/` の記録で確認する。

### ファイルとテスト

- `voice_command.py`: 起動入口、波形・ベクトル・判定記録の保存。
- `audio_input.py`: ファイル / マイクの入力、16 kHz・モノラル化。
- `embedding_gemma.py`: 生波形 / 説明文 → マスク付き平均プーリング → L2 正規化。
- `task_router.py`: タスクごとの類似度集約と見送り判定。
- `task_actions.py`: タスク ID と既存コマンドの対応、dry-run / 実行。

```bash
PYTHON=_local/voice/.venv/bin/python bash Voice/tests/run_tests.sh
```

テストはモデルのダウンロード・マイク・実機接続を行わない。torch 未導入時は
モデル境界のテストのみスキップする。実モデルでの確認には上記 `--embed-only` を使う。

公式仕様: [EmbeddingGemma 2 モデルカード](https://ai.google.dev/gemma/docs/embeddinggemma/model_card_2)、
[Transformers のプーリング・音声入力の参照実装](https://github.com/huggingface/transformers/blob/v5.22.2/tests/models/embedding_gemma2/test_modeling_embedding_gemma2.py)。

## スコープ（3段階、難易度・依存の順）

| 段階 | 内容 | 依存 |
|---|---|---|
| 1. 単発TTS | G1内蔵TTS（`AudioClient.TtsMaker`）で決め打ちの文言を喋らせる | `unitree_sdk2py`のみ |
| 2. 単発マイク文字化 | G1のマイクmulticastを録音し、OpenAI等で文字起こしする | `unitree_sdk2py`不要（標準UDP） |
| 3. 対話パイプライン | STT→LLM→TTSを連結し、G1と会話する | 1・2に加えPC2上の中継サーバ |

段階3は外部の公開実装（下記「参考・移植元」）が既にあるため、まずはそれを
そのまま動かして実機で成功パスを確認し、その後このフォルダへ移植するのが
効率的（ゼロから再実装しない）。

## 構成（予定）

- `tts_speak_real.py` — 単発TTS（段階1）。実装済み
- `mic_transcribe_real.py` — 単発マイク文字化（段階2）。TODO（下記「参考・移植元」参照）
- `dialogue/` — 対話パイプライン（段階3）。TODO、外部実装の移植待ち

既知の注意点（`FAILURES.md`参照）:
- マイクの自動待ち受け(`listen`)方式は空文字を返すことがある。GUI方式（人間が
  録音区間をON/OFFする）を優先する
- マイク文字化と対話パイプライン(`dialogue/`)は同時に起動しない
  （マイクmulticastの競合）
- LLM出力をTTSに渡す前に、空文字・絵文字のみの場合はスキップするサニタイズが必要
  （ElevenLabs等が`input_text_empty`でクラッシュするため）
- APIキーは`.env`（git管理外）に置く。コミットしない

## なぜPC2側に中継サーバが要るのか（段階3）

スピーカー再生(`AudioClient.PlayStream`)はDDS通信＝`unitree_sdk2py`+`cyclonedds`
依存で、Linux前提。操作PCがWindows等の場合、直接叩くのはリスクが高い。
そのため**SDK依存部分だけをPC2（G1内蔵のUbuntu機）に置き**、操作PCとは単純な
TCP/JSON+PCMで通信する構成を取る。マイク入力はSDK不要（素のUDP multicast）
なので操作PCから直接受信できる。詳細は`Common/g1-onboard-pc/README.md`。

## 参考・移植元

- [mayochan32/unitree-g1-physical-ai](https://github.com/mayochan32/unitree-g1-physical-ai)
  （公開リポジトリ、`voice-conversation/`配下）
  - `g1_bridge/bridge_server.py` — PC2上で動かす中継サーバ（TCP版、追加インストール不要）
  - `pc_pipeline/` — 操作PC側のSTT/LLM/TTSパイプライン本体
  - `RUNBOOK.md` — PC2がJetson Orin NX(aarch64)であることや、共有機材としての
    運用ルール（`sudo`禁止・`/tmp`限定・音量復元等）が詳しく書かれている
- 実機で成功した記録（社内カタログ、2026-09-01〜05時点）: 内蔵TTS・マイク文字化・
  対話パイプラインのSTT/LLM部分まで実機確認済み。TTSは主にElevenLabsの空文字
  クラッシュ対策後の状態で稼働。詳細は`FAILURES.md`。

## 環境

`G1_HuggingFace/venv/`（操作PC側、Python 3.12）を使う想定。ネットワーク接続・
疎通確認は`Common/network/`を参照。**Dockerは使わない**
（→ ルート`README.md`の「開発環境の使い分け」）。

## 進め方

1. `tts_speak_real.py`で単発TTSが実機で鳴ることを確認
2. マイク文字化・対話パイプラインは外部実装（上記）をそのまま動かして成功パスを
   確認してから、このフォルダへ移植する
3. 失敗した内容は`FAILURES.md`に記録する

## 状態

段階1（単発TTS）は実機確認済み。段階2・3は外部実装の移植待ち（未着手）。
音声からのタスク選択は上記の EmbeddingGemma 2 パイプラインとして別途実装。
