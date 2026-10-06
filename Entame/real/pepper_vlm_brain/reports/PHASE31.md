# PHASE 3.1 実測結果

基準SHA: `c5f5c2513d27ed588d34f529b61ec643861606cf`。Qwen3-VL-2B-Instruct BF16 / SDPA / RTX 3060 Ti 8GB。
ObservationとDecisionの分離、Observation由来のMemory、previous selected/currentの2枚入力は実装済み。
**実動画シナリオはsingle/temporalともFAIL。人物不在の誤認とcombined出力の不安定さが残った。**

|項目|結果|根拠|
|---|---|---|
|Observation schema|PASS（実装）|必須型・enum・presence/count・transitionの整合性を検証。不正出力は採用しない|
|Memory from Observation|PASS（実装）|人物状態はObservationだけから更新。Decisionはlast_actionだけ記録|
|Two-frame temporal input|PASS（入力経路）|初回1枚、以降7回は実processorで2枚、84+84 image tokens。一度のgeneration|
|Combined VLM output reliability|FAIL|single 0/8、temporal 4/8のみ有効|
|8→10 disappearance detection|FAIL|10秒はperson_visible=trueとDISAPPEAREDが矛盾。8秒にも同じ矛盾|
|SEARCH previous direction|FAIL|10秒のraw actionはLOOK LEFT。8秒出力が不正のためMemoryのlast directionはUNKNOWNのまま|
|10→12 reappearance detection|PASS（12秒の視覚出力）/ FAIL（全遷移）|12秒はtrue LEFT APPEARED。10秒のabsenceがMemoryに記録されずscenarioの再捕捉はFAIL|
|Existing regressions|PASS|旧72ケースを保持して契約をObservationへ移行。追加41件、計113 PASS。旧静止画実GPUもLOOK RIGHT PERSON MID|

## Resolution ablation（architecture変更前）

同一910×512の10秒source pixel、同じ旧prompt/直前Memory（LEFT,true）を使用。
旧schemaには人物有無がないためpresenceはDecision由来の参考proxyであり、独立したObservationの実測ではない。
解像度以外の設定とprompt SHA256は同一。各callのCUDA peakをreset。reservedには同じprocessのallocator履歴が含まれる。

|long edge|人物proxy|target/action|latency s|image/input tokens|allocated/reserved GiB|
|---|---|---|---:|---|---|
|384|true（誤認）|PERSON / LOOK CENTER|7.532|84 / 615|4.133 / 4.156|
|448|true（誤認）|PERSON / LOOK CENTER|8.383|112 / 643|4.139 / 4.211|
|512|true（誤認）|PERSON / LOOK CENTER|9.964|144 / 675|4.153 / 4.266|

3解像度すべてLOOK CENTER / PERSON。**この画像では384px縮小だけを原因とは説明できず、512pxでも改善しなかった。**
参照: `phase31_resolution_ablation.json`。動画全体や他シーンで解像度の効果がないとまでは結論しない。

## Final temporal timeline

以下はraw Observationとraw Decisionを、採用されたActionと区別して掲載する。
不正なObservationやDecisionをSEARCHへ修正していない。WAITは既存の検証失敗fallbackであり、知覚成功として扱わない。

|秒|visible/count|direction/distance|transition|raw Decision|採用Action|
|---:|---|---|---|---|---|
|2|true / 1|LEFT / NEAR|UNKNOWN|"LOOK"（不正な文字列）|WAIT UNKNOWN（fallback）|
|6|true / 1|LEFT / NEAR|DISAPPEARED|LOOK LEFT / PERSON / NEAR / 0.95|WAIT UNKNOWN（fallback）|
|8|true / 1|LEFT / NEAR|DISAPPEARED|LOOK LEFT / PERSON / NEAR / 0.95|WAIT UNKNOWN（fallback）|
|10|true / 1|LEFT / NEAR|DISAPPEARED|LOOK LEFT / PERSON / NEAR / 0.95|WAIT UNKNOWN（fallback）|
|12|true / 1|LEFT / NEAR|APPEARED|LOOK LEFT / PERSON / NEAR / 0.95|LOOK LEFT|
|16|true / 1|CENTER / NEAR|STILL_VISIBLE|LOOK CENTER / PERSON / NEAR / 0.95|LOOK CENTER|
|20|true / 1|CENTER / NEAR|STILL_VISIBLE|WAIT CENTER / PERSON / NEAR / 0.95|WAIT CENTER|
|24|true / 1|CENTER / NEAR|STILL_VISIBLE|WAIT CENTER / PERSON / NEAR / 0.95|WAIT CENTER|

blocking_obstacleは全8時点NONE。scene_summaryは要求せず、解析にも使用しない。
2秒はDecisionが文字列、6/8/10秒はtrue+DISAPPEAREDの矛盾で拒否。last_seen_person_directionは2/6/8/10秒後UNKNOWN、
12秒後LEFT、16/20/24秒後CENTER。20秒は実際にはLEFTなのにCURRENTをCENTERと判断しており、
前画像の人物位置を現在画像へ混同している可能性がある（この出力だけでは原因を確定できない）。
有効出力に対してrunnerが方向やActionを置換する処理はない。
singleは全8時点でUNKNOWN transitionを返したがDecisionが文字列LOOKで、すべてfallback。
single raw presenceは全true、directionは2/6/8/12/20秒LEFT、10秒RIGHT、16/24秒CENTER。

## Performance（384px, max_new_tokens=224）

|mode|平均latency s|input tokens|image tokens|output tokens|peak allocated/reserved GiB|driver peak MiB|
|---|---:|---|---|---|---|---|
|single|7.699|561–561|84–84|66–66|4.112 / 4.129|5045|
|temporal|13.212|561–682|84–168|66–108|4.154 / 4.188|5106|

temporalの2枚入力7回だけの平均は14.022秒。初回1枚を含む8回平均は上表。
model load時間は除外、前処理と生成を含む。singleのDecision欠落によりoutput tokensが少ないため、
latency差を画像2枚の計算量だけに帰属させない。single/temporalとも独立した新規Memoryから開始。
BF16 2Bのみ使用。VRAM fraction 0.80を維持し、OOMなし。4B/4bitは実施していない。

## Additional evaluation-only samples

full-resolution画像を目視確認して人物あり3/7/13秒、不在9/9.5/10.25秒を追加。
10.5秒は左端に頭の一部、11秒は顔が見えるため、不在サンプルから除外した。
正解ラベルは`scenarios/IMG_7121_phase31_additional.json`だけに置き、Brain/promptには渡さない。
**raw Observation単体のperson_visible accuracyは3/6=50%。不在3枚はすべてtrueと誤認。**
6枚ともDecision文字列でcombined outputは無効。有効なcombined出力だけを認め、fallbackを失敗と数えるaccuracyは0/6=0%。
rawの診断用ObservationをMemoryへ採用する処理はない。同じ動画の近接画像であり一般化性能を保証しない。

## Validation and reproduction

113 tests PASS（旧72ケース保持＋追加41）。既存Memory/runner/scenarioテストは同じ目的を保ちつつ明示Observationを渡す契約へ変更。
新規検証: strict schema、enum、first frame、malformed Observation/Decision、方向保持/更新、Decisionだけでは人物Memory不変、
confidence/summary非依存、previous selected画像の対応、失敗後もprevious画像進行、Action/Observationの改変検出、期待値のモデル入力混入防止。
pip check: No broken requirements found。compileallとgit diff --checkも成功。
旧GPU静止画: LOOK RIGHT / PERSON / MID, fallback=False。
Webcam実機は今回未再検証。Pepper/G1接続、SDK変更、driver/CUDA変更なし。

```powershell
.\.venv\Scripts\python.exe run_resolution_ablation.py
.\.venv\Scripts\python.exe run_phase31_comparison.py
.\.venv\Scripts\python.exe evaluate_scenario.py reports\phase31_temporal_final.json scenarios\IMG_7121_phase31.json --output reports\phase31_scenario_temporal.json
.\.venv\Scripts\python.exe -m pytest -q
```

再実行は同名reportを更新する。今回の初回実験は`phase31_single.json`/`phase31_temporal.json`に別保存。
初回promptの画像1枚での遷移違反を受け、一度だけ一般的な修正（previousがなければUNKNOWNだけ許可）を追加した。
その結果、UNKNOWN遵守は改善した一方Decisionが文字列になった。この負の結果も最終reportに保持し、追加の画像依存prompt調整はしていない。
評価label/timestamp/hashを行動条件にする処理はない。evaluatorの期待値は後処理だけに使う。

## Changed files

- `brain/schemas.py`, `brain/decision.py`, `brain/memory.py`, `brain/prompt.py`, `brain/vlm.py`: Observation/combined parser、Memory分離、ラベル付き2枚入力。
- `config.py`, `run_video.py`: 安全な512px ablation上限、Observation/temporal CLI、previous-frame記録と一度の生成。
- `evaluate_scenario.py`, `scenarios/IMG_7121_phase31*.json`: Observation/Action/Memory/画像枚数の厳密な後処理評価。
- `tests/test_memory.py`, `tests/test_video.py`, `tests/test_scenario.py`, `tests/test_observation.py`: 契約移行と追加41ケース。
- `run_resolution_ablation.py`, `run_phase31_comparison.py`: 再現用benchmark harness（正解をモデルへ渡さない）。
- `README.md`, `reports/phase31_*.json`, `reports/PHASE31.md`: 利用説明と実測の正負両結果。
最終commit SHAとcommit後のgit statusは完了メッセージに記載。
