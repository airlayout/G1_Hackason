# PHASE 3.3: 2B baseline Split Brain GPU validation

2026-10-02 / starting stable commit `979d6570d4520d689b081b4dfdb81442b183bad0`.

**PC上のSplit Brain接続と主要8→10→12検証はPASS。拡張9画像の実動画シナリオは13秒の知覚方向誤りによりFAIL（8/9）。**

## Criteria

|項目|結果|
|---|---|
|Perception → deterministic State → text-only Planner → validated Decision → Mock Robot|PASS|
|保存Observation 8秒 LOOK LEFT / 10秒 SEARCH LEFT / 12秒 LOOK LEFT|3/3 PASS|
|実動画8→10 DISAPPEARED / 10秒 SEARCH LEFT / 10→12 APPEARED|3/3 PASS|
|Planner bare structured JSON|保存3/3・実動画3/3・9/9 PASS|
|同一model / processor再利用、実weight load 1回/プロセス|PASS|
|拡張9画像 human-label scenario|8/9 FAIL（13秒）|
|9画像のPlannerと入力Observation/Stateのpolicy整合性|9/9 PASS|
|Tests|201 PASS = 既存183 + 追加18|

## Conditions and boundaries

RTX 3060 Ti 8GB / Qwen3-VL-2B-Instruct BF16 / pinned revision `89644892e4d85e24eaac8bacfd4f463576704203` / SDPA / 384px / output budget 96 / greedy generation。安定 .venvを使用。Perception prompt/schema、State、Planner promptは基準commitから変更なし。Perception prompt SHA256 `be189c8c054f9ea74821de6370e9653551626039b4511359beca9eaa4b106bf3`。4B・依存環境・Pepperへの変更なし。

保存Observation replayを最初に実GPU検証しPASSを確認後、実動画3画像を実行。3画像の完了後PASSを確認して9画像へ進んだ。各runは別プロセスの1 model / processorを使用し、そのrun内の全stageで同一IDとload_count=1を記録。Perception image_count=1/image_tokens=84、Planner image_count=0/image_tokens=0。Plannerへのuser入力はCurrentObservationとStateのJSONのみ。scenarioの期待値は事後evaluatorにだけ存在する。

Plannerのvalid Actionは変更しない。不正JSONはWAIT fallback、Observation/Stateは保持。未引用enumのquote補完は行わない。MockRobotはvalidated Decision型のみ受け付け、実際にprintした出力をreportへ保存。Brain latencyはMock実行、モデルload、動画decode、frame保存、report I/Oを含まない。

## Actual frame results

|run|秒|person / direction|transition / missed|last direction|VLM Action / direction|P(s)|State(ms)|Planner(s)|Brain(s)|
|---|---:|---|---|---|---|---:|---:|---:|---:|
|saved|8|True / LEFT|UNKNOWN / 0|LEFT|LOOK / LEFT|0.000|0.011|1.974|1.974|
|saved|10|False / UNKNOWN|DISAPPEARED / 1|LEFT|SEARCH / LEFT|0.000|0.015|1.537|1.537|
|saved|12|True / LEFT|APPEARED / 0|LEFT|LOOK / LEFT|0.000|0.013|1.633|1.633|
|three|8|True / LEFT|UNKNOWN / 0|LEFT|LOOK / LEFT|7.528|0.018|1.631|9.159|
|three|10|False / UNKNOWN|DISAPPEARED / 1|LEFT|SEARCH / LEFT|6.488|0.016|1.478|7.966|
|three|12|True / LEFT|APPEARED / 0|LEFT|LOOK / LEFT|6.651|0.013|1.569|8.220|
|nine|3|True / CENTER|UNKNOWN / 0|CENTER|LOOK / CENTER|7.309|0.017|1.013|8.322|
|nine|7|True / LEFT|STILL_VISIBLE / 0|LEFT|LOOK / LEFT|6.520|0.016|1.540|8.060|
|nine|8|True / LEFT|STILL_VISIBLE / 0|LEFT|LOOK / LEFT|6.774|0.013|1.514|8.288|
|nine|9|False / UNKNOWN|DISAPPEARED / 1|LEFT|SEARCH / LEFT|6.671|0.030|1.535|8.206|
|nine|9.5|False / UNKNOWN|STILL_ABSENT / 2|LEFT|SEARCH / LEFT|6.414|0.013|1.477|7.891|
|nine|10|False / UNKNOWN|STILL_ABSENT / 3|LEFT|SEARCH / LEFT|6.496|0.014|1.477|7.973|
|nine|10.25|False / UNKNOWN|STILL_ABSENT / 4|LEFT|WAIT / UNKNOWN|6.577|0.013|0.909|7.485|
|nine|12|True / LEFT|APPEARED / 0|LEFT|LOOK / LEFT|6.631|0.012|1.563|8.194|
|nine|13|True / CENTER|STILL_VISIBLE / 0|CENTER|LOOK / CENTER|6.675|0.013|1.623|8.298|

9画像では9秒がDISAPPEARED/count=1、9.5秒がSTILL_ABSENT/count=2、10秒がSTILL_ABSENT/count=3。直前のlast direction LEFTからSEARCH LEFTが生成された。10.25秒はcount=4のため既存の汎用policy（recent 1..3）でWAIT UNKNOWN。12秒はAPPEARED/LOOK LEFT。13秒はPhase 3.2と同じCENTER誤認でLOOK CENTER。human label LEFTに対してFAILだが、Plannerはcurrent Observationに忠実。StateもCENTERへ更新され、知覚誤りが履歴に影響する限界を示す。Perceptionは調整せず、評価基準もLEFTのまま維持。

## Performance

|run|mean Perception(s)|mean State(ms)|mean Planner(s)|mean Brain(s)|peak allocated/reserved(GiB)|load(s)|
|---|---:|---:|---:|---:|---:|---:|
|saved|0.000|0.013|1.715|1.715|4.054 / 4.082|3.226|
|three|6.889|0.016|1.559|8.448|4.055 / 4.096|3.128|
|nine|6.674|0.016|1.406|8.080|4.055 / 4.096|3.117|

Phase 3.1 temporalは全8frame平均13.212s、2画像7frame平均14.022s。今回9画像Brain平均8.080s。image入力84tokensとtext-only Plannerを分離した結果、Planner平均1.406sで追加できた。異なるframe schedule・prompt・output token数・測定scopeを含むため統制した速度比較ではない。旧temporalはvalid combined 4/8、今回は全9frameでPerception/Planner valid JSONだが方向accuracyは4/5。速度のみで優劣を判断しない。

各frameのinput/output token、stage peak VRAM、全raw、validated Observation/Decision、State before/after、Mock実出力、fallback reasonはJSONに保存。今回fallbackは全run 0。savedのPerception 0sとtoken N/Aは推論未実行を意味する。初回保存replay reportのperception内source GPU情報はPhase 3.2由来で、今回のPlanner計測ではない。

## Evidence and negative results

- `phase33_saved_planner.json` / `.log` と `phase33_saved_evaluation.json`: 最初のGPU Planner replay。
- `phase33_e2e_three.json` / `.log` と `phase33_three_evaluation.json`: 主要3frameのE2E。
- `phase33_e2e_nine.json` / `.log` と `phase33_nine_evaluation.json`: 13秒FAILを含む全9frame。
- `phase33_summary.json`: latency/tokens/VRAM/固定ファイル比較。
- `phase33_tests.log`: 201 PASS。malformed/enum/immutable入力/failure preservation/model reuse/ordering/validated Robot/evaluator false-success rejection。
- `phase33_three_interim_evaluation.json`: 3frame runが進行中で8/10秒のみ書き込まれた時点に誤って評価した診断。missing12/status runningでFAIL。GPU/Planner failureではなく、完了後の評価は全3frame PASS。診断も保持。
- Phase 3.2・4B・Phase 3.1の全結果は既存ファイルとして保持。追加prompt trialなし。

## Git

実装・raw report・評価・testsを一括commit。最終commit SHAとcommit後git statusは完了メッセージに記載（自身のSHAを同じcommitへ埋め込むことはできない）。
