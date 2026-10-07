# PHASE 3.4 LATENCY FINAL BATTLE

2026-10-02 / baseline commit `2cbba110d6fef22f696397803777cf17451f7969`。

## Baseline profiling

JSON baselineのPhase 3.3実測はPerception 6.674s、State 0.016ms、Planner 1.406s、Brain 8.080s。今回の同一video再測定は以下。

|stage|total(s)|image prep(s)|processor(s)|vision(s)|prefill(s)|TTFT(s)|decode(s)|ms/decode token|output tokens|parsing(ms)|
|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|
|perception|6.660|0.004|0.003|4.462|4.563|4.565|2.087|49.035|43.556|0.068|
|planner|1.439|0.000|0.001|N/A|0.071|0.073|1.365|49.354|28.667|0.044|

Baseline Brain mean 8.099s。Vision/prefillの大半はvision patch embeddingのConv3d。短い出力だけではその時間を除去できなかった。

CUDA eventはfirst forwardとVision/patch embeddingを計測。Visionはprefill内なので加算しない。TTFTはgeneration開始から最初のgenerated tokenのCPU到着まで。stage TTFTはprocessor/転送を含む。decodeは初token以降、EOSを含む残りの生成時間。ms/tokenはgenerated_count-1で割り、1 tokenではN/A。validation/parsingは別のCPU wall timer。

JSON instrumentedとuninstrumentedを同じ9画像×2回＋fresh 8/10/12で比較した。profilingの計測誤差とprofile順序・GPU clockの影響を分離する統制実験ではないが、collectorなしの軽いforward hooks/CUDA events/streamerは大きな上乗せを示さなかった。CUDA CUPTI profilerは収集処理が戻らず停止した診断を保存し、SDPA backend確認はATen dispatchとnative backend selectorで代替した。

## Profiles comparison

|profile / dtype|presence|direction (visible)|P/D valid|main|extended /9|P mean(s)|Planner mean(s)|Brain mean/median/p95(s)|peak allocated/reserved(GiB)|gate / tier|
|---|---:|---:|---:|---|---|---:|---:|---|---:|---|
|class_384 / bf16|100.0%|80.0%|100.0% / 0.0%|FAIL|0/9, 0/9|4.454|0.397|4.851 / 4.857 / 4.935|4.027 / 4.064|REJECT / REJECT|
|class_both / bf16|100.0%|80.0%|100.0% / 100.0%|FAIL|2/9, 2/9|4.440|0.090|4.530 / 4.518 / 4.643|4.038 / 4.064|REJECT / REJECT|
|class_both_256_fp16 / fp16|100.0%|60.0%|100.0% / 100.0%|FAIL|2/9, 2/9|1.827|0.077|1.904 / 1.901 / 1.980|4.039 / 4.668|REJECT / REJECT|
|class_both_288_fp16 / fp16|100.0%|80.0%|100.0% / 100.0%|FAIL|2/9, 2/9|2.523|0.092|2.615 / 2.592 / 2.772|4.039 / 4.668|REJECT / REJECT|
|class_both_320_fp16 / fp16|100.0%|80.0%|100.0% / 100.0%|FAIL|2/9, 2/9|3.334|0.091|3.426 / 3.415 / 3.519|4.039 / 4.668|REJECT / REJECT|
|class_both_336_fp16 / fp16|100.0%|80.0%|100.0% / 100.0%|FAIL|2/9, 2/9|3.302|0.090|3.392 / 3.379 / 3.533|4.039 / 4.668|REJECT / REJECT|
|class_both_fp16 / fp16|100.0%|80.0%|100.0% / 100.0%|FAIL|2/9, 2/9|4.600|0.086|4.686 / 4.698 / 4.815|4.039 / 4.668|REJECT / REJECT|
|class_constrained / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|7/9, 7/9|4.211|1.460|5.671 / 5.736 / 5.794|4.055 / 4.096|REJECT / REJECT|
|class_constrained_fp16 / fp16|100.0%|80.0%|100.0% / 100.0%|PASS|6/9, 6/9|4.521|1.234|5.755 / 5.600 / 6.167|4.056 / 4.668|REJECT / REJECT|
|compact / bf16|44.4%|40.0%|66.7% / 0.0%|FAIL|0/9, 0/9|5.104|0.285|5.389 / 5.478 / 5.632|4.041 / 4.064|REJECT / REJECT|
|compact_exact / bf16|44.4%|40.0%|44.4% / 44.4%|FAIL|1/9, 1/9|4.679|0.178|4.858 / 4.775 / 5.158|4.055 / 4.096|REJECT / REJECT|
|json / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|8/9, 8/9|6.660|1.439|8.099 / 8.188 / 8.421|4.055 / 4.096|accuracy PASS / C|
|json_fp16 / fp16|100.0%|80.0%|100.0% / 100.0%|PASS|7/9, 7/9|6.819|1.274|8.093 / 7.885 / 8.602|4.056 / 4.668|REJECT / REJECT|
|json_linear / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|8/9, 8/9|2.227|1.451|3.678 / 3.786 / 4.012|4.055 / 4.096|accuracy PASS / B|
|json_linear_efficient / bf16|0.0%|0.0%|0.0% / 0.0%|FAIL|0/9, 0/9|0.045|0.000|0.045 / 0.043 / 0.055|N/A / N/A|REJECT / REJECT|
|json_linear_efficient_expanded / bf16|0.0%|0.0%|0.0% / 0.0%|FAIL|0/9, 0/9|0.014|0.000|0.014 / 0.014 / 0.016|N/A / N/A|REJECT / REJECT|
|json_linear_efficient_expanded_fixed / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|7/9, 7/9|2.019|1.368|3.386 / 3.422 / 3.543|4.033 / 4.076|REJECT / REJECT|
|json_linear_uninstrumented / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|8/9, 8/9, 8/9|2.167|1.412|3.578 / 3.710 / 3.799|4.055 / 4.096|accuracy PASS / B|
|json_uninstrumented / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|8/9, 8/9|6.731|1.448|8.179 / 8.265 / 8.502|4.055 / 4.096|accuracy PASS / C|
|minified_linear_256 / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|6/9, 6/9|2.254|0.989|3.243 / 3.258 / 3.331|4.060 / 4.096|REJECT / REJECT|
|minified_linear_288 / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|6/9, 6/9|2.259|0.981|3.239 / 3.254 / 3.326|4.060 / 4.096|REJECT / REJECT|
|minified_linear_320 / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|6/9, 6/9|2.214|0.977|3.191 / 3.235 / 3.329|4.060 / 4.096|REJECT / REJECT|
|minified_linear_336 / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|6/9, 6/9|2.212|0.997|3.209 / 3.276 / 3.386|4.060 / 4.096|REJECT / REJECT|
|minified_linear_384 / bf16|100.0%|80.0%|100.0% / 100.0%|PASS|6/9, 6/9|2.284|0.981|3.265 / 3.258 / 3.344|4.060 / 4.096|REJECT / REJECT|
|minimal_256 / bf16|0.0%|0.0%|0.0% / 0.0%|FAIL|0/9, 0/9|2.066|0.000|2.066 / 2.050 / 2.236|3.992 / 4.064|REJECT / REJECT|
|minimal_288 / bf16|0.0%|0.0%|0.0% / 0.0%|FAIL|0/9, 0/9|2.789|0.000|2.789 / 2.809 / 2.911|3.995 / 4.064|REJECT / REJECT|
|minimal_320 / bf16|0.0%|0.0%|0.0% / 0.0%|FAIL|0/9, 0/9|3.640|0.000|3.640 / 3.618 / 3.739|3.998 / 4.064|REJECT / REJECT|
|minimal_336 / bf16|0.0%|0.0%|0.0% / 0.0%|FAIL|0/9, 0/9|3.609|0.000|3.609 / 3.600 / 3.765|3.998 / 4.064|REJECT / REJECT|
|minimal_384 / bf16|0.0%|0.0%|0.0% / 0.0%|FAIL|0/9, 0/9|5.019|0.000|5.019 / 5.062 / 5.226|4.004 / 4.064|REJECT / REJECT|

## Input/TTFT/decode ablation

|profile|P input/image/output tokens (mean)|P TTFT/decode(s)|Planner input/output tokens (mean)|Planner TTFT/decode(s)|
|---|---|---|---|---|
|class_384|161.000 / 84.000 / 1.000|4.446 / 0.000|263.222 / 7.667|0.077 / 0.318|
|class_both|161.000 / 84.000 / 1.000|4.431 / 0.000|310.222 / 1.000|0.087 / 0.000|
|class_both_256_fp16|109.000 / 32.000 / 1.000|1.820 / 0.000|310.222 / 1.000|0.074 / 0.001|
|class_both_288_fp16|122.000 / 45.000 / 1.000|2.515 / 0.000|310.222 / 1.000|0.090 / 0.001|
|class_both_320_fp16|137.000 / 60.000 / 1.000|3.327 / 0.000|310.222 / 1.000|0.088 / 0.001|
|class_both_336_fp16|137.000 / 60.000 / 1.000|3.294 / 0.000|310.222 / 1.000|0.088 / 0.000|
|class_both_fp16|161.000 / 84.000 / 1.000|4.592 / 0.000|310.222 / 1.000|0.084 / 0.001|
|class_constrained|161.000 / 84.000 / 1.000|4.203 / 0.000|382.222 / 29.556|0.094 / 1.364|
|class_constrained_fp16|161.000 / 84.000 / 1.000|4.513 / 0.000|382.222 / 23.778|0.090 / 1.142|
|compact|265.000 / 84.000 / 8.000|4.733 / 0.363|326.667 / 8.000|0.067 / 0.359|
|compact_exact|308.000 / 84.000 / 8.000|4.331 / 0.339|380.500 / 8.000|0.069 / 0.331|
|json|339.000 / 84.000 / 43.556|4.565 / 2.087|382.778 / 28.667|0.073 / 1.365|
|json_fp16|339.000 / 84.000 / 43.556|4.629 / 2.181|382.778 / 24.333|0.072 / 1.200|
|json_linear|339.000 / 84.000 / 43.556|0.098 / 2.121|382.778 / 28.667|0.073 / 1.376|
|json_linear_efficient|N/A / N/A / N/A|N/A / N/A|N/A / N/A|N/A / N/A|
|json_linear_efficient_expanded|N/A / N/A / N/A|N/A / N/A|N/A / N/A|N/A / N/A|
|json_linear_efficient_expanded_fixed|339.000 / 84.000 / 43.556|0.087 / 1.923|382.778 / 30.111|0.060 / 1.306|
|json_linear_uninstrumented|339.000 / 84.000 / 43.556|N/A / N/A|382.778 / 28.667|N/A / N/A|
|json_uninstrumented|339.000 / 84.000 / 43.556|N/A / N/A|382.778 / 28.667|N/A / N/A|
|minified_linear_256|300.000 / 32.000 / 43.556|0.096 / 2.151|395.778 / 18.556|0.089 / 0.898|
|minified_linear_288|313.000 / 45.000 / 43.556|0.098 / 2.154|395.778 / 18.556|0.089 / 0.890|
|minified_linear_320|328.000 / 60.000 / 42.778|0.098 / 2.107|395.778 / 18.556|0.089 / 0.887|
|minified_linear_336|328.000 / 60.000 / 42.000|0.101 / 2.103|395.778 / 18.556|0.088 / 0.907|
|minified_linear_384|352.000 / 84.000 / 43.556|0.101 / 2.174|395.778 / 18.556|0.089 / 0.891|
|minimal_256|98.000 / 32.000 / 8.000|1.718 / 0.341|N/A / N/A|N/A / N/A|
|minimal_288|111.000 / 45.000 / 8.000|2.442 / 0.341|N/A / N/A|N/A / N/A|
|minimal_320|126.000 / 60.000 / 8.000|3.295 / 0.338|N/A / N/A|N/A / N/A|
|minimal_336|126.000 / 60.000 / 8.000|3.262 / 0.340|N/A / N/A|N/A / N/A|
|minimal_384|150.000 / 84.000 / 8.000|4.662 / 0.349|N/A / N/A|N/A / N/A|

## Accuracy and selection

期待label/actionは事後evaluatorだけが読む。raw compact/classを厳密にdecodeし、既存CurrentObservation/PlannerDecisionへ復元した後、schema/scenario evaluatorをcanonical JSON viewで再利用する。元rawは変更しない。型、enum、重複field、presence/count/absence整合性は既存schemaで検証。不正rawはWAIT fallbackで、Stateを捏造・修正しない。有効だが誤ったActionもそのままMockに渡し、FAILとして記録する。

採用gateはpresence 100%、visible direction >=80%、各9-frame runのextended score >=8/9、fresh 8/10/12全PASS、全stage valid、raw/decoded/State/Mock/model reuse integrity PASSを要求する。速度だけで採用しない。Tierはこのgate通過後にmean Brain <=1.5 S、<=2.5 A、<=4 B、それ以上C。主scenarioのみ良いprofileを採用しないため、Bにもbaseline精度維持を要求した。

最初のpipe compactはfield名/説明文、矛盾した不在方向などで失敗。150-token minimal promptはinput目標を満たしたが、出力形式が崩れ不採用。exact文法の追加は全画像に同一のserialization指定を行うprofileで、画像別ruleはない。こちらにも知覚/Actionの誤りが残り不採用。

single-tokenはtokenizer実測で1 tokenのA/B/C/Dをscene classへ対応。finite constrained版は許可codeのlogit argmaxを使い、GT/State machineでcodeを選ばない。Planner finite版も全generic Action+direction code候補のモデルlogitから選ぶ。code valid率が100%でもAction accuracyのFAILはそのまま判定する。距離とblockingをUNKNOWN、countを0/1にするclass modeは情報量を減らした実験であり、full Observationと同等の知覚機能を証明していない。

Best measured eligible pure-VLM profile: **json_linear_uninstrumented**, Brain mean **3.578s**, tier **B**。presence 100.0%、direction 80.0%、main PASS、extended [8, 8, 8]。accuracy gateを満たす候補で最小のmean latencyを選んだ。

Best full Observation profile: **json_linear_uninstrumented**, mean **3.578s**。元の5-field知覚契約を保持する。

Final tier: **B**。HYBRID RECOMMENDED = **NO（今回のlatency gate）**。

Tierはmean latencyで判定する。best uninstrumentedのp95も確認したが、この9画像×3回の測定で将来の全frameのdeadlineを保証するものではない。S/Aは未達。

Tier Cの場合、synchronous pure-VLMを実世界demoのdefault候補にしない。次候補は高頻度fast perceptionでlive geometryを取得し、scene/stateからVLM high-level Plannerを非同期実行する構成。Robot execution時のgeometryは最新fast perceptionから得る。今回Hybrid、Pepper/G1接続は実装していない。

## Attention / dtype / advanced backend

既存torch 2.9.1+cu128 / RTX 3060 Ti 8GB / pinned 2B revision / SDPA。同一GPU loaded weightをPerception/Plannerで共有。FP16はBF16 weightを同じ2B instanceでconvertし、FP32 rotary/numerical buffersを保持した。別processでBF16/FP16を比較し、load/conversionはruntime latencyに含めない。

Windows wheelのFlash Attentionはcompiled available=False。強制FLASH_ATTENTIONはNo available kernelでFAIL。追加package/FlashAttention-2/driver/CUDA変更は行わない。default SDPAの選択backendと全operatorは `phase34_attention_default.json`、強制失敗は `phase34_attention_flash.json`、CUPTI中断は `phase34_attention_cuda_attempt.json` に保存。

native selectorでの実選択は `phase34_attention_selection.json`: Vision EFFICIENT_ATTENTION、text prefill/decode MATH。textのquery 16 heads / KV 8 headsはnative GQAでMATHへfallback。単純なEFFICIENT強制はunsupported GQAでFAIL。`efficient_expanded`は既存repeat_kv経路で同じKVをquery headsへ展開し、既存EFFICIENT kernelを使う別profile。fn差替えはprocess内のgeneration contextだけで、例外時も元へ戻す。最初の実装はmock call historyがCUDA Tensorを保持してOOMになったためraw失敗を残し、参照を保持しないplain predicateへ修正した `_fixed` 試行で再検証。

advanced profileのConv3d→Linearがある場合、non-overlapping patchをflattenし同じweight/biasでF.linearを実行する代数的に等価なbackend。kernel=stride、padding=0、dilation=1、groups=1、入力patchサイズを確認し、profileごとに一時的に差し替え必ず元へ戻す。外部packageもweight二重loadも不要。stable engine/source、venvは変更しない。数学的等価性と実GPUの9画像/fresh mainを検証し、accuracy gateで採否を判断する。Static KV cache、torch.compile、quantization、外部backendは未実施。

`phase34_patch_equivalence.json`: actual BF16 patch max abs difference 0.03125、RMSE 0.000687、exact fraction 76.97%。代数的等価でもBF16 reduction丸めはbit-identicalではない。baseline JSONを保持したEnd-to-Endのpresence/direction/main/extended gateで結果を確認した。Windows環境にはTritonとMSVC clがなく、追加build依存を導入するcompile/外部backend実験は今回行わなかった。

## Reproduction / evidence

各profileでwarmupを保存して除外し、9画像を2回、別のfresh Stateで8/10/12を1回測定。最終uninstrumentedは9画像×3回=27 samples＋fresh main。mean/median/p95は異なるframeを含む測定分布であり、profile順序による温度/clock差を含む。model load、動画decode、Mock実行、report I/OはBrain timer外。誤ったoutput/invalid/fallbackを含む全frameのraw/復元Observation/State before-after/transition/Decision/Mock/各stage tokens・TTFT・decode・peak VRAMをJSON保存。uninstrumentedのTTFT/decode内訳、および戻り値を得られなかった初期GPU例外trialのtoken/peak項目はN/A。例外時のpartial telemetry取得は最終実装に追加し、State保持と共にテストした。

`reports/phase34/*.json` が全raw、`reports/phase34/evaluations/*.json` が評価、`reports/phase34_summary.json` が集計。rootの `phase34_*.log` にstdout/stderrと初期からの全負の結果を保持。画像サイズは同じsource RGBを用い、input pixels SHAはprovenance専用。engineへtimestamp/hash/GTを渡さない。

Tests: **269 PASS**（既存201＋追加68）。`phase34_tests_final.log`、pip check PASS、`phase34_environment.json` と `phase34_frozen_path.json` の9 functional files unchangedを保存。最終git status/commit SHAは完了メッセージに記載。開始時からのユーザー編集 `reports/PHASE33.md` は保持して今回のcommitから除外する。stable functional pathと4Bの環境/実装/過去reportsはfreeze。
