# Voice（音声・音声対話）

G1に喋らせる・G1のマイクで聞き取る機能全般。`SimpleWalk/`・`Mapping/`等と同じ
「機能ドメインごとにトップレベル1つ」の構成に合わせて新設。

音声はsim側で検証する意味が薄い（マイク・スピーカーの実機挙動そのものが対象のため）
ので、`sim/`は作らず**`real/`のみ**。

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

- `real/tts_speak_real.py` — 単発TTS（段階1）。実装済み
- `real/mic_transcribe_real.py` — 単発マイク文字化（段階2）。TODO（`README`参照）
- `real/dialogue/` — 対話パイプライン（段階3）。TODO、外部実装の移植待ち

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

1. `real/tts_speak_real.py`で単発TTSが実機で鳴ることを確認
2. マイク文字化・対話パイプラインは外部実装（上記）をそのまま動かして成功パスを
   確認してから、このフォルダへ移植する
3. 失敗した内容は`FAILURES.md`に記録する

## 状態

段階1（単発TTS）は実機確認済み。段階2・3は外部実装の移植待ち（未着手）。
