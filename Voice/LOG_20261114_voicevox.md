# 2026-11-14 G1 に日本語を喋らせる（作業ログ）

目的: G1 本体のスピーカーで日本語を喋らせる。手順と仕組みは `VOICEVOX.md`。
実機は Wi-Fi の ssh（`unitree@192.168.0.82`）で PC2 に入って試した。鳴らすコマンドは本人が打った。

## できたこと

- **VOICEVOX（剣崎雌雄）の声で、G1 本体のスピーカーから日本語を喋らせた。**
  操作PCで `voicevox_core` が WAV を作り、PC2 の `g1_speaker.py` が `PlayStream` で流す。
  `voicevox_speak.py --play "文"` の 1 コマンドで、作る・送る・鳴らすまで行う
- 声を実機で聞き比べて決めた: ずんだもん → 玄野武宏・剣崎雌雄の標準と低め、の 4 種類 →
  **剣崎雌雄を少し低く（`--pitch -0.06`）・抑揚控えめ（`--intonation 0.8`）**を既定にした
- 24kHz で作った WAV を PC2 で 16kHz に直して流しても、切れや速さの狂いは無かった
- 内蔵 TTS（`TtsMaker`）は話者 1 で**英語なら喋る**ことを確かめた
- `tts_speak_real.py` の `TypeError` を直し、鳴らさずに確かめる `--check` を足した

## できなかったこと・分かった制約

- **内蔵 TTS は日本語を読めない。** 話者 1 は「japanese …」のようなエラー音声が続き、話者 0 はほぼ無音
  （`code=0` は返る）。日本語は外部 TTS の音声を流すしかない
- **VOICEVOX ENGINE は入れられなかった。** G1 用の Wi-Fi 経由では 0.16MB/s で、2GB に 3 時間かかる見込み
  だった。しかも落としていたのは x64 版で、ARM64 のこのノートPCでは動かなかった。ライブラリ版の CORE に切り替えた
- `voicevox_core` は 16kHz で出力できない（24000 の倍数のみ）
- VOICEVOX のダウンロードツールは利用規約の同意を対話で聞くため、Claude からは走らせられない（本人が打った）

## 前からの記録とのつながり

- 他のメンバーの記録: Console 班（10/1）は `TtsMaker` の話者 0 が無音・話者 1 が鳴ると記録。
  SecurityUI 班（11/3）は日本語が未確認として `TtsMaker` を使わず、録音 WAV を `PlayStream` で流していた。
  今回の `g1_speaker.py` はその実装を写したもの
- `Dev/UI` の `Console/docs/REAL_ROBOT_CHECKLIST.md` の C5（話者 ID と日本語の可否）は今回の結果で答えが出た
  （このブランチには無いので未反映）

## 残っていること

- `Voice/README.md` へのリンク追加。`Dev/Voice` で README が大きく書き換えられており、このブランチで
  README を触ると `main` に入れるときに衝突するため、今回は新しいファイル（`VOICEVOX.md` と本ログ）に分けた
- 本体の音声アシスタント（"Hello Robot"）や `Dev/Voice` の音声待受と同時に鳴らしたときの挙動は未確認
