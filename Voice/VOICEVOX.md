# VOICEVOX で G1 に日本語を喋らせる（2026-11-14 実機確認済み）

G1 内蔵の TTS（`AudioClient.TtsMaker`）は**日本語を読めない**（下記「内蔵 TTS の実機試験」）。
そこで操作PCの VOICEVOX CORE で日本語の音声を作り、PC2 に送って `PlayStream` で流す。

```
操作PC（WSL）: 文 ──voicevox_core──> WAV（24kHz・モノラル）
              ──scp──> PC2: /tmp/g1_voice_tts/ ──ssh──> g1_speaker.py（16kHz に直して PlayStream）
```

| ファイル | 役割 | 動かす場所 |
|---|---|---|
| `voicevox_speak.py` | 文から WAV を作り、PC2 に送って鳴らす | 操作PC（`voicevox_core` を入れた venv） |
| `g1_speaker.py` | WAV を 16kHz に直して `PlayStream` で鳴らす。`Dev/SecurityUI` の `G1_HuggingFace/UI/Speaker/g1_speaker.py` を写したもの（`APP_NAME` だけ変更） | PC2（system python3.8） |
| `tts_speak_real.py` | 内蔵 TTS（英語のみ）。`--check` で鳴らさずに音声サービスへの到達だけ確かめられる | PC2 |

- 合成は操作PCで行う（インターネット不要）。PC から音声の DDS は叩かず、PC2 の中で鳴らす
  （PC でカメラと音声の DDS を同時に使うとエラー 3102/3104 になった記録がある）
- 既定の声は**剣崎雌雄（スタイル 21）、高さ・抑揚は VOICEVOX の標準（`--pitch 0`・`--intonation 1`）**。
  ずんだもん・九州そら・白上虎太郎・玄野武宏などと実機で聞き比べて決めた。初めは少し低く（-0.06）・抑揚控えめ（0.8）に
  していたが、標準のほうが明るく、大きく聞こえた
- 外に出す場面（発表・動画など）では「**VOICEVOX:剣崎雌雄**」のクレジット表記が要る
- 音量は変えない（共用の機体。2026-11-14 時点で 100）。PC2 では `/tmp` の下だけを使う

## 使う

リポジトリの一番上（`G1_Hackason/`）で、操作PCのターミナルから:

```bash
bash Voice/say.sh "こんにちは"                       # いちばん手軽: 既定の声で喋らせる（STYLE=32 などで声を変える）
PY=~/voicevox_core/venv/bin/python
$PY Voice/voicevox_speak.py --play "こんにちは。私はジーワンです"   # 作って送って鳴らす
$PY Voice/voicevox_speak.py "こんにちは"            # 作って PC2 に送るだけ（鳴らすコマンドを出す）
$PY Voice/voicevox_speak.py --no-send "こんにちは"  # 作るだけ（Voice/out/speech.wav）
$PY Voice/voicevox_speak.py --list-speakers         # 使える声とスタイル ID
$PY Voice/voicevox_speak.py --play --style 3 --pitch 0 --intonation 1 "ずんだもんなのだ"  # 声を変える
```

- PC2 の宛先の既定は Wi-Fi 側の `unitree@192.168.0.82`。有線なら `--host unitree@192.168.123.164`
- 鳴らさずに PC2 の音声サービスに届くかだけ見る:
  `ssh unitree@192.168.0.82 'python3 /tmp/g1_voice_tts/g1_speaker.py --check'`

使える声（入れてあるモデル `0.vvm`・`2.vvm`・`4.vvm`・`9.vvm`）:

| 声 | スタイル ID |
|---|---|
| 四国めたん | ノーマル 2、あまあま 0、ツンツン 6、セクシー 4 |
| ずんだもん | ノーマル 3、あまあま 1、ツンツン 7、セクシー 5 |
| 春日部つむぎ | 8 |
| 雨晴はう | 10 |
| 玄野武宏 | 11 |
| 剣崎雌雄 | 21（既定） |
| 九州そら（2.vvm） | ノーマル 16、あまあま 15、ツンツン 18、セクシー 17 |
| 白上虎太郎（9.vvm） | ふつう 12、わーい 32、びくびく 33、おこ 34、びえーん 35 |

他のモデル（青山龍星 `15.vvm` など）の一覧は voicevox_vvm の README.txt にある。青山龍星・No.7 などは
企業が関わる利用に事前確認が要るので、使う前に規約を確かめること。

## エレベーターのボタンを押す流れで喋らせる（2026-11-14 追加。実機ではまだ鳴らしていない）

押す流れの節目で、決まった言葉を喋らせる。言葉は `Button_Press/J1-gen/configs/voice.yaml`。

| 出来事 | 言葉（既定） | いつ |
|---|---|---|
| `press_start` | ボタンを押します | 計画が立ち、腕を伸ばし始めるとき |
| `lit` | 点灯成功 | 点灯を確かめたとき（押している間だけ点いたのを見たときも） |
| `retry` | 点灯していません。もう一度押します | 点灯せず、深く押し直すとき |
| `not_confirmed` | 点灯を確認できませんでした | 押し直しても確かめられなかったとき |
| `gave_up` | ボタンを押すのを中止します | 見つからない・計画が立たないなどで中止したとき |

押している間は合成しない（PC2 には `voicevox_core` が入らない）。ノート PC で前もって WAV にして PC2 の
`/tmp/g1_voice_tts/phrases/` に置き、押す流れでは鳴らすだけ。鳴らすのは別スレッド・別プロセスなので、
腕の制御（50 Hz）は待たない。鳴らせなくても注意を 1 行出すだけで、押す流れは続ける。

```bash
P=G1_HuggingFace/venv/bin/python
$P Button_Press/J1-gen/real/voice_phrases.py make         # WAV を作る（Voice/out/phrases/）
$P Button_Press/J1-gen/real/voice_phrases.py send         # PC2 に送る（PC2 の /tmp は再起動で消える。その日に 1 回）
$P Button_Press/J1-gen/real/voice_phrases.py play --all   # 全部を順に鳴らして確かめる
```

押す流れで喋らせるのは `real/auto_press.py` に `--voice` を付ける（dry-run でも喋る）:
- PC2 の上で動かすとき: `--voice local`（PC2 の中で `g1_speaker.py` を動かす）
- 有線のノート PC から動かすとき: `--voice ssh`（無線の ssh で PC2 に鳴らさせる。PC から音声の DDS は叩かない）

鳴らした記録（言葉・成否・かかった秒）は、エージェントの結果の JSON の `"voice"` に入る。
言葉を変えたら `make` → `send` をやり直す。

## 入れる（操作PC。1 回だけ）

⚠️ **このノートPCは ARM64（aarch64）**。x64 版は動かない（"Exec format error"）。合計 約 250MB。

```bash
mkdir -p ~/voicevox_core && cd ~/voicevox_core
curl -sSL -o download https://github.com/VOICEVOX/voicevox_core/releases/download/0.17.1/download-linux-arm64
chmod +x download
# 利用規約（ONNX Runtime・音声モデル）への同意を聞かれる。本人が読んで y を押す
./download --exclude c-api additional-libraries --models-pattern 0.vvm -o ./voicevox_core
# 剣崎雌雄・玄野武宏のモデル（音声モデルの規約は上で同意済み）
curl -sSL -o voicevox_core/models/vvms/4.vvm https://github.com/VOICEVOX/voicevox_vvm/releases/download/0.17.1/4.vvm
# 聞き比べに使った九州そら（2.vvm）・白上虎太郎（9.vvm）。既定の声には要らない
curl -sSL -o voicevox_core/models/vvms/2.vvm https://github.com/VOICEVOX/voicevox_vvm/releases/download/0.17.1/2.vvm
curl -sSL -o voicevox_core/models/vvms/9.vvm https://github.com/VOICEVOX/voicevox_vvm/releases/download/0.17.1/9.vvm
python3 -m venv venv
venv/bin/pip install https://github.com/VOICEVOX/voicevox_core/releases/download/0.17.1/voicevox_core-0.17.1-cp310-abi3-manylinux_2_34_aarch64.whl
```

## はまった点

- **`voicevox_core` は出力の周波数に 24000 の倍数しか指定できない**（16000 で `InvalidQueryError`）。
  24kHz で作り、PC2 の `g1_speaker.py` が 16kHz に直す。実機で切れ・速さの狂いは無かった
- **VOICEVOX ENGINE（HTTP サーバ版）は使わない。** 2GB あり、G1 用の Wi-Fi 経由では 3 時間かかる見込み
  だった。ライブラリ版の CORE なら 約 250MB
- ダウンロードツールは利用規約への同意を対話で聞くので、Claude などから非対話で走らせると
  `unexpected end of file` で止まる。本人のターミナルで打つ

## 内蔵 TTS（`TtsMaker`）の実機試験（2026-11-14、PC2 の上で実行、音量 100）

| 文 | 話者 | 結果（耳で確認） |
|---|---|---|
| 「こんにちは、私はジーワンです」 | 1 | G1 のスピーカーから、エラーのような音声で「japanese …」のようなものが何回も続いた（ターミナルのログは他と同じで `code=0`） |
| "Hello, I am G1" | 1 | **英語で喋った** |
| 「こんにちは、私はジーワンです」 | 0 | 「ふっ」という音だけで、ほぼ無音 |

- `code=0` は「受理された」という意味で、鳴った証拠ではない
- 以前の `tts_speak_real.py` は `code, _ = TtsMaker(...)` と受けていたが、PC2 の SDK
  （`~/unitree_sdk2_python`）の `TtsMaker` は `code` だけを返すため、呼んだ直後に `TypeError` で落ちていた
  （2026-11-14 に修正）。README の「段階1は実機確認済み」はこのスクリプトでの記録ではない可能性がある
