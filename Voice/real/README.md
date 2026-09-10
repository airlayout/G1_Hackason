# Voice / real

実機G1上での音声関連コードをここに置く。`../README.md`の3段階に対応。

## `tts_speak_real.py`（段階1・実機確認済み）

G1内蔵TTS（`AudioClient.TtsMaker`）で指定した文言を喋らせる、最小のワンショット
スクリプト。追加インストール不要（`unitree_sdk2py`のみ）。

```bash
python Voice/real/tts_speak_real.py --network-interface enp3s0 "こんにちは"
```

## `mic_transcribe_real.py`（段階2・TODO）

マイクmulticastを受信し、外部APIで文字起こしするスクリプト。まだこのリポジトリには
無い。移植元・参考実装は`../README.md`の「参考・移植元」を参照。

既知の注意点（`../FAILURES.md`参照）:
- 自動待ち受け(`listen`)方式は空文字を返すことがある。GUI方式（人間が録音区間を
  ON/OFFする）を優先する
- 対話パイプライン（`dialogue/`）と同時に起動しない（マイクmulticastの競合）

## `dialogue/`（段階3・TODO）

STT→LLM→TTSの対話パイプライン。PC2側に音声中継サーバ（TCP、追加インストール
不要）を配置する構成になる。まずは外部実装（`../README.md`参照）をそのまま動かして
成功パスを確認してから、このフォルダへ移植する。

既知の注意点（`../FAILURES.md`参照）:
- LLM出力をTTSに渡す前に、空文字・絵文字のみの場合はスキップするサニタイズが必要
  （ElevenLabs等が`input_text_empty`でクラッシュするため）
- APIキーは`.env`（git管理外）に置く。コミットしない
