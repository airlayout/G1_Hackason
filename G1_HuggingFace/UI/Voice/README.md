# Voice — 人を検知したときの声かけの録音

警備 UI は、人を検知すると G1 本体のスピーカーでここにある WAV を鳴らす
（`Main/config.yaml` の `voice`）。

⚠️ **このリポジトリは公開されている。** git に入れた録音は誰でも聞け、履歴から消すのも難しい。
入れてよいのは**録った本人が公開に同意したものだけ**。`tachiiri_kinshi.wav` は録った本人が了承して入れた（2026-11-03）。
それ以外の `.wav` は `.gitignore` で除外してある（切り出す前の録音・試し録りを誤って入れないため）。

| ファイル | 台詞 |
|---|---|
| `tachiiri_kinshi.wav` | 「立ち入り禁止です。警備員に連絡します」 |

## 録る（Ubuntu の操作PC）

```bash
cd G1_HuggingFace/UI/Voice
wpctl set-volume @DEFAULT_AUDIO_SOURCE@ 1.0              # 入力の音量を 100% に（低いと無音で録れる）
arecord -f S16_LE -r 16000 -c 1 -d 5 tachiiri_kinshi.wav   # 5 秒録る。言い終えたら待つだけ
aplay tachiiri_kinshi.wav                                   # 聞き直す
```

- **前後の無音は切り落とす。** 無音もそのまま鳴るので、5 秒録ったら声の部分だけ残す
  （無音も繰り返しのたびに鳴り、人が消えてから止まるまでも延びる）
- 形式は PCM の WAV なら何でもよい（44.1kHz・ステレオでも可）。PC2 側で 16kHz・モノラル・16bit に直して送る
- ⚠️ 他人の声や、配布してよいか分からない音声を git に入れないこと。エンタメ班の WAV も、配布してよいか
  確かめていないという理由で git から外されている。録り直して差し替えるときも、入れる前に本人の了承を取る

## 本体で鳴るか確かめる（UI を上げずに）

```bash
ssh g1 'mkdir -p ~/g1_ui_speaker'
scp ../Speaker/g1_speaker.py tachiiri_kinshi.wav g1:~/g1_ui_speaker/
ssh g1 'python3 ~/g1_ui_speaker/g1_speaker.py --check'                                   # 鳴らさずに音量を見る
ssh g1 'python3 ~/g1_ui_speaker/g1_speaker.py --wav ~/g1_ui_speaker/tachiiri_kinshi.wav'  # 1 回鳴らす
```

無線のときは `g1` を `unitree@<PC2 の Wi-Fi 側の IP>` に読み替える。

## 仕組み

```
操作PC: UI サーバ ──ssh（標準入力に "play"）──> PC2: Speaker/g1_speaker.py --serve
                                                     └─ AudioClient.PlayStream（DDS は PC2 の中だけ）
```

- **人が映っている間はずっと繰り返す。** 言い終えたら `repeat_gap_s`（既定 1 秒）空けてまた言う
- 人が消えたら止める（言いかけの台詞は最後まで言う）。消えたと判定するのは、検知ログと同じく 1.5 秒（`absent_grace_s`）映らなかったとき
- ⚠️ **カメラが途切れても止める。** 映像が途絶えると「消失」が出ないので、繰り返す前に毎回カメラが生きているかを確かめている
- 音量は変えない（共用の機体なので、いまの音量のまま鳴らす）
- **無線（見るだけ）でも鳴らす**（2026-11-03 に決めた）。声は検知で自動的に出るもので、画面からの操作ではない。機体は動かさない
- 画面の上の「声かけ」に状態が出る: 待機 / 話し中 / 失敗 / ログのみ（ssh の宛先が無い。`Main/run.sh` 単体のとき）/ 切
- ⚠️ **PC から音声の DDS を叩かない。** エンタメ班が PC でカメラと音声の DDS を同時に使ってエラー 3102/3104 を踏んだため、
  エンタメ班と同じく PC2 の中から鳴らす
- ⚠️ 純正の `TtsMaker`（その場で読み上げ）は使っていない。公式資料では中国語と英語のみで、日本語は未確認

## 実機での確認（2026-11-03・有線）

- `g1_speaker.py --check` が PC2 の system python3.8 で動き、音声サービスに届いた（音量 100。`GetVolume` は `(0, {'volume': 100})` を返す）
- 単発で鳴らして、本体のスピーカーから聞こえた
- `bash run.sh --device <C922 の by-id>` で通し: 人が 2〜3 秒映った回は 1 回、**9 秒映った回は 3 回鳴り、外れたら止まった**

- 無線（`bash run.sh --wifi --device <C922 の by-id>`。PC2 は Physical_AI_5G、操作PC は Physical_AI）でも、
  映っていた 3.5 秒に 2 回鳴り、外れたら止まった。Nav2 は動かしていない（声かけに Nav2 は要らない）

## 未確認のこと（2026-11-03 時点）

- 本体の音声アシスタント（"Hello Robot"）とスピーカーを取り合うか
- Nav2 や巡回と同時に動かしたときに PC2 の負荷が問題にならないか
