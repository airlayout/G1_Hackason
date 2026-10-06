# PHASE 3.5 FAST VLM FINAL BATTLE ROUND 2

Fast geometryは実GPUで約0.10秒に到達した。しかしfull Observationを入力しても1-token Plannerはpolicyを維持できず、採用を見送る。同一モデルのdual-rate prototypeは状態・resolverの契約を満たすが、semantic処理中の待ち時間が大きい。**PURE/DUAL-RATE VLM VIABLE = NO、HYBRID FAST DETECTOR + VLM RECOMMENDED = YES（CASE 4）**。

基準SHA: `46423378034c69dce2f07aa9b56689f19090c222`。Qwen3-VL-2B-Instruct revision `89644892e4d85e24eaac8bacfd4f463576704203`、BF16、384px、default SDPA、RTX 3060 Ti 8GB。Linear patch実装と既存schema/State/policy/Adapter、評価データ、stable venvを保持した。Pepper/G1接続、4B、YOLO実装、package/driver/CUDA変更は実施していない。

## 測定方法

`IMG_7121.mp4`の既存9点（3,7,8,9,9.5,10,10.25,12,13）を使用。A/B/B2は各warmup 1回、9点×3回、fresh Stateの8→10→12を1回。Dはwarmup 1回、9点×2回、fresh 3点を1回。集計は9点の測定分のみで、load/warmup/fresh主シナリオを除外。映像抽出・ファイル保存・Mock出力はBrain計時外。

AとPlannerの時間はstreamer/CUDA eventsで計測。B/B2のPerceptionはstable referenceと同じuninstrumented JSON経路。p95は標本の線形補間で、長時間のライブ動作を保証する値ではない。1-token出力のdecode_totalは最初のtoken後の終了処理時間であり、autoregressive decode_stepsは0。

runner/engineはGTファイルを読まない。画像hashとtimestampは取得・順序・出力のprovenanceとして保存し、promptやclass選択に渡さない。Planner入力はimmutable Observation、Stateのmemory、transitionのみ。GT評価は別commandで実行。

## Stable reference

`json_linear_uninstrumented`: Perception 2.167s、Planner 1.412s、Brain mean/median/p95 3.5785 / 3.7097 / 3.7986s。presence 100%、visible direction 80%、main PASS、extended [8, 8, 8]、VRAM allocated/reserved 4.055/4.096 GiB。既存記録を再利用し、今回の計測として数えない。

## 全profile比較

| Profile | P mean (s) | Planner mean (s) | E2E mean/median/p95 (s) | 精度・判定 |
|---|---:|---:|---|---|
| Stable full JSON | 2.1669 | 1.4115 | 3.5785 / 3.7097 / 3.7986 | main PASS、extended 8/9×3、維持 |
| A geometry only | 0.0999 | N/A | NOT RUN | geometry gate PASS、full Action判定は対象外 |
| B full Observation + finite Planner | 2.2661 | 0.0931 | 2.3593 / 2.3558 / 2.4190 | main FAIL、extended [3, 3, 3]、不採用 |
| B2 full Observation + finite Planner | 2.2585 | 0.0738 | 2.3324 / 2.3280 / 2.4316 | main FAIL、extended [5, 5, 5]、不採用 |
| C / C2 fast E2E | N/A | N/A | NOT RUN | B/B2 gate FAILにより実行禁止 |
| D serialized dual-rate + JSON Planner | 0.0980 fast + 2.2378 semantic | 1.4490 | 3.7850 / 3.8879 / 4.1046 | prototype PASS、main PASS、extended [8, 8]、速度要件FAIL |

## A. Linear patch + 1-token Fast Perception

Presence [27, 27] = 100%、direction [12, 15] = 80%、valid class 100%。main geometry 8→10→12 PASS、extended geometry [8, 8, 8]。13秒はD=RIGHTと誤認し、GTのLEFTへ補正していない。

Perception mean/median/p95 **0.0999 / 0.0972 / 0.1062s**。VRAM allocated/reserved 4.007/4.012 GiB。

| A stage metric | mean / median / p95 |
|---|---|
| processor_s | 0.0029 / 0.0027 / 0.0041 |
| device_transfer_s | 0.0006 / 0.0006 / 0.0009 |
| image_preprocessing_s | 0.0048 / 0.0045 / 0.0065 |
| patch_embed_cuda_elapsed_s | 0.0002 / 0.0002 / 0.0002 |
| vision_cuda_elapsed_s | 0.0261 / 0.0258 / 0.0293 |
| prefill_cuda_elapsed_s | 0.0849 / 0.0845 / 0.0921 |
| ttft_stage_s | 0.0955 / 0.0952 / 0.1020 |
| ttft_generation_s | 0.0870 / 0.0864 / 0.0941 |
| decode_total_s | 0.0005 / 0.0004 / 0.0006 |
| text_decoding_s | 0.0001 / 0.0001 / 0.0002 |
| input_tokens | 161.0000 / 161.0000 / 161.0000 |
| image_tokens | 84.0000 / 84.0000 / 84.0000 |
| output_tokens | 1.0000 / 1.0000 / 1.0000 |
| validation_parsing_s | 0.0000 / 0.0000 / 0.0000 |
| peak_allocated_gib | 4.0065 / 4.0065 / 4.0065 |
| peak_reserved_gib | 4.0117 / 4.0117 / 4.0117 |
| stage_wall_s | 0.0998 / 0.0972 / 0.1061 |

Fast PerceptionはA/B/C/Dの4つのgeometry codeを実際のmodel logitsから選ぶ。A=NO_PERSON、B=LEFT、C=CENTER、D=RIGHT。token IDs 32/33/34/35、各exactly 1 tokenをload時に検証。出力にはdistance/obstacleの意味情報がなく、UNKNOWN、person_countは0/1。Full Observationの代替採用ではない。

## B. Full Observation + 1-token Planner

Bは元のpolicy文章にgeneric class catalogを付けたprofile。B2は同一policyとcatalogを、短い指示と一般的なserialization例で提示した追加profile。B2の例はvisible RIGHT FAR、last CENTER/missed 2、長時間不在で障害物NONEであり、動画固有の期待Actionは使用していない。

各codeはload時にA〜N = token ID 32〜45の1-tokenであることを確認。AllowedCodesは他tokenをmaskし、未修正のclass logitsのargmaxを選ぶ。action/direction/targetは選ばれたcodeから直接復元し、LOOK/FOUND/APPROACHのdistanceのみ入力Observationの値をserialization時にbindingする。誤ったAction/directionもそのまま残してpolicy evaluatorでFAILとする。このdistance bindingはPhase 3.4の既存codecを変更しない。

| Code | Action / direction / target |
|---|---|
| A | LOOK / LEFT / PERSON |
| B | LOOK / CENTER / PERSON |
| C | LOOK / RIGHT / PERSON |
| D | APPROACH / CENTER / PERSON |
| E | SEARCH / LEFT / PERSON |
| F | SEARCH / CENTER / PERSON |
| G | SEARCH / RIGHT / PERSON |
| H | AVOID_LEFT / LEFT / OBSTACLE |
| I | AVOID_RIGHT / RIGHT / OBSTACLE |
| J | FOUND / LEFT / PERSON |
| K | FOUND / CENTER / PERSON |
| L | FOUND / RIGHT / PERSON |
| M | WAIT / UNKNOWN / NONE |
| N | SURPRISE / UNKNOWN / NONE |

### B

Planner valid code 100%、policy一致 10/30（fresh主シナリオを含む）。main FAIL、extended [3, 3, 3]、presence 100%、direction 80%、integrity True。

Planner mean/median/p95 **0.0931 / 0.0931 / 0.0952s**。Brain mean/median/p95 2.3593 / 2.3558 / 2.4190s。VRAM allocated/reserved 4.065/4.098 GiB。

| Planner metric | mean / median / p95 |
|---|---|
| input_tokens | 416.7778 / 417.0000 / 417.0000 |
| prefill_cuda_elapsed_s | 0.0885 / 0.0884 / 0.0904 |
| ttft_stage_s | 0.0918 / 0.0915 / 0.0937 |
| ttft_generation_s | 0.0905 / 0.0903 / 0.0925 |
| output_tokens | 1.0000 / 1.0000 / 1.0000 |
| decode_total_s | 0.0005 / 0.0005 / 0.0008 |
| validation_parsing_s | 0.0000 / 0.0000 / 0.0000 |

Fresh 8→10→12の実出力:

| 秒 | P direction/visible | State transition / last_seen / missed | Raw code → Decision |
|---:|---|---|---|
| 8 | LEFT / True | UNKNOWN / LEFT / 0 | C → LOOK RIGHT |
| 10 | UNKNOWN / False | DISAPPEARED / LEFT / 1 | M → WAIT UNKNOWN |
| 12 | LEFT / True | APPEARED / LEFT / 0 | J → FOUND LEFT |

10秒ではinput transition=DISAPPEARED、last_seen=LEFT、missed=1。SEARCH LEFTのE logit=27.75に対し、選択Mのlogit=34.0。Runner/StateはSEARCHへ置換していない。full JSON Plannerよりpolicy accuracyが悪化したため不採用。

### B2

Planner valid code 100%、policy一致 20/30（fresh主シナリオを含む）。main FAIL、extended [5, 5, 5]、presence 100%、direction 80%、integrity True。

Planner mean/median/p95 **0.0738 / 0.0736 / 0.0759s**。Brain mean/median/p95 2.3324 / 2.3280 / 2.4316s。VRAM allocated/reserved 4.051/4.084 GiB。

| Planner metric | mean / median / p95 |
|---|---|
| input_tokens | 363.7778 / 364.0000 / 364.0000 |
| prefill_cuda_elapsed_s | 0.0694 / 0.0691 / 0.0712 |
| ttft_stage_s | 0.0725 / 0.0721 / 0.0746 |
| ttft_generation_s | 0.0714 / 0.0710 / 0.0732 |
| output_tokens | 1.0000 / 1.0000 / 1.0000 |
| decode_total_s | 0.0005 / 0.0004 / 0.0007 |
| validation_parsing_s | 0.0000 / 0.0000 / 0.0000 |

Fresh 8→10→12の実出力:

| 秒 | P direction/visible | State transition / last_seen / missed | Raw code → Decision |
|---:|---|---|---|
| 8 | LEFT / True | UNKNOWN / LEFT / 0 | A → LOOK LEFT |
| 10 | UNKNOWN / False | DISAPPEARED / LEFT / 1 | G → SEARCH RIGHT |
| 12 | LEFT / True | APPEARED / LEFT / 0 | J → FOUND LEFT |

10秒ではinput transition=DISAPPEARED、last_seen=LEFT、missed=1。SEARCH LEFTのE logit=32.5に対し、選択Gのlogit=32.75。Runner/StateはSEARCHへ置換していない。full JSON Plannerよりpolicy accuracyが悪化したため不採用。

## C. Fast VLM End-to-End

**NOT RUN**。AはPASSだがB/B2が独立gateを満たさないため、組み合わせを実GPUで実行していない。mean/median/p95、main、extended、presence/direction、VRAMはいずれもCの測定値としてN/A。AとBの独立stage時間の足し算をE2E実測として報告しない。C/C2 runnerはgate FAILでmodel load前に終了する。

**Final tier: REJECT（accuracy gate FAIL、S〜D認定なし）**。S≤0.5/A≤1.0/B≤1.5/C≤2.5/D>2.5の速度区分は精度PASSが前提。Aの0.10秒だけを使ってfast E2EをTier Sへ昇格させない。

## D. Dual-rate prototype

**Prototype contract PASS / latest-geometry resolver PASS**。同一model/processorのinstance ID、load_count=1を全stageで確認。Original JSON Plannerのpolicy [9, 9, 3]、main PASS、extended [8, 8]。Fast geometry presence [18, 18]、direction [8, 10]。

Schedulerはfast→full semantic→元のJSON Plannerの直列アクセス。Fast Geometryのみがdeterministic State/transitionを更新する。semantic結果は別slotに保存し、geometryを書き換えない。取得timestamp/generationが完全一致し、presence/directionも一致する時だけdistance/obstacleを融合する。旧semanticはdiagnosticsへ保持するが新geometryやStateを上書きしない。不一致では意味情報をUNKNOWNにする。

| D measured budget | mean / median / p95 (s) |
|---|---|
| fast_s | 0.0980 / 0.0976 / 0.1072 |
| semantic_s | 2.2378 / 2.2096 / 2.3716 |
| planner_s | 1.4490 / 1.5717 / 1.6443 |
| cycle_s | 3.7850 / 3.8879 / 4.1046 |
| fast_blocked_by_semantic_planner_s | 3.6868 / 3.7832 / 4.0047 |

VRAM allocated/reserved 4.056/4.096 GiB。高速stage自体は約0.10秒だがsemantic＋Plannerによる待ち時間は平均約3.69秒、p95約4.00秒。このprototypeはオフライン選択フレームの測定であり、カメラの連続稼働・10Hz・ライブframe dropを測定していない。本構成の単一GPU・非preemptiveアクセスでは即時反応を保証できない。

### Latest-geometry execution resolver

immutable PlannerDecisionをsemantic intentとして保持し、実行直前のLOOK/FOUND PERSONだけ方向を最新geometryへbindingする。action/target/distanceは元の値を維持する。SEARCHはモデルが選んだmemory方向のままであり、resolverでSEARCHへ変更しない。追跡対象が消失/UNKNOWNならcommandを抑止し、WAIT/SEARCHを捏造しない。APPROACHは最新人物がCENTERでなければ抑止する。Robotへ渡すのは再validationしたPlannerDecisionのみ。

追加dry runは実GPUの8秒LOOK LEFTを保持した後、実GPUの13秒fast code D=RIGHTを受け、8秒semanticを遅れて配送する。geometryはRIGHTのまま、元decisionはLOOK LEFTのまま、resolved decisionはLOOK RIGHT。配送順だけを模擬し、GPU同時実行は主張しない。13秒のRIGHTはGTに対して誤りであり、これはresolver契約のPASSであって視覚精度PASSではない。

Dのpolicy/scenario評価はresolver前の**元のVLM Decision**を使用する。resolverによる方向bindingで失敗したPlannerを救済してPASSにはしない。

## 次Phaseの判断

**CASE 4: finite Planner精度FAIL、および精度を維持するDのcycle mean>2.5s。PURE/DUAL-RATE VLM VIABLE = NO / HYBRID FAST DETECTOR + VLM RECOMMENDED = YES。**Phase 3.4のHYBRID=NOを引き継がず、今回の即時反応要件で再判定した。この結論は今回測定したprofileとschedulerに対するもので、将来の全VLM最適化が不可能という意味ではない。

次の設計候補はCamera→fast person detector（YOLO等）→timestamp付きgeometry/State、低頻度semantic VLM→既存policyのVLM Planner→最新geometry resolver→validated Adapter。検出器はcurrent visibility/directionを担当し、semantic branchはdistance/obstacleを補う。新Actionは追加せず、古いsemanticで最新geometryを上書きしない契約を引き継ぐ。

既存G1経験はローカル[人物YOLO実験記録](C:/dev/g1-bottle-reaction/docs/G1_PERSON_YOLO.md:14)と[person_yolo.py](C:/dev/g1-bottle-reaction/src/g1_bottle_reaction/game_vision/person_yolo.py:98)をread-onlyで確認した。記録ではYOLO11n、imgsz=640、confidence=.25、上限15 FPS、RTX 5060 Ti/Linuxで3050回、平均5.4ms・14.6 FPS。camera/GUIはYOLOを待たず、latest frame slot、1枚だけのin-flight、別process推論、500ms超の結果無効化を使用する。PC取得timestampを使い、推論完了時刻で結果を新しく見せない。これらを再利用設計の出発点とする。stealth側ではraw labelをsemantic PLAYERへ変換し、継続trackingと離散イベントを分離する経験もある。

5.4msは別GPU/OSの過去記録で、今回の3060 Ti/Windowsの見積りや撮影→実行時間としては使用しない。次Phaseでは同じIMG_7121と基準labelsでpresence/direction、13秒の方向、消失/再出現、fresh SEARCHを評価する。bbox中心を既存の40%/60%区分へ変換し、confidence/target選択をGT別ルールにしない。さらにVLMとの同一GPU競合、latest-slot frame age、queue待ち、fast更新間隔p95/最大、VRAM、capture-to-executionを実測する。別processでも同一GPUの待ち時間が自動的に解消するとは仮定しない。detectorを入れても元JSON Plannerの約1.4秒は残る。LOOK trackingのgeometry更新と、SEARCH等の意味的Action変更にかかる時間を別々に測り、failed finite Plannerを採用しない。YOLO実装・package導入・実機接続は今回行わない。

## 保存物・テスト・Git

**356 passed in 5.77s**。既存269件にPhase 3.5の87件を追加。class mapping、invalid/nonfinite拒否、distance serializationとAction保持、logits argmax、GPU1-token/Linear証拠、Planner無画像・無GT入力、StateがSEARCHを強制しないこと、timestamp/generation precedence、stale semantics、resolver/validated Mock出力、model reuse、C gate拒否、評価のraw不変性・Action改ざん拒否を確認。pip check PASS。

[全機械可読summary](phase35/summary.json)、[freeze証拠](phase35/freeze.json)、[環境](phase35/environment.json)、[G1 YOLO過去記録の出典・hash](phase35/g1_yolo_reference.json)、[変更ファイル一覧](phase35/changed_files.json)、[commit前Git status](phase35/git_status_before_commit.txt)。

Raw output / logits / token IDs / Observation / State / Planner input / resolved Decision / 全latency:

- [A raw](phase35/A.json) / [A evaluation](phase35/evaluations/A.json)
- [B raw](phase35/B.json) / [B evaluation](phase35/evaluations/B.json)
- [B2 raw](phase35/B2.json) / [B2 evaluation](phase35/evaluations/B2.json)
- [C NOT RUN](phase35/C.json) / [C2 NOT RUN](phase35/C2.json)
- [D raw + late-delivery dry run](phase35/D.json) / [D evaluation](phase35/evaluations/D.json)
- [GPU A/B log](phase35_gpu_AB.log) / [GPU B2 log](phase35_gpu_B2.log) / [GPU D log](phase35_gpu_D.log)
- [全tests log](phase35_tests_final.log) / [pip check](phase35_pip_check.log) / [C gate guard](phase35_C_gate_guard.log)

既存reports/PHASE33.mdには作業開始前からユーザー変更がある。編集・stage・commit対象に含めていない。既存stableとPhase 3.4 reportsは保持。新規実装はphase35/と専用benchmark/evaluator/report/testのみ。このレポート・全実装・実GPU結果を含むcommit SHAとcommit後Git statusは、commit完了後に追加する[completion record](phase35/completion.json)へ保存する。completion記録を含む最終HEAD SHAは最終応答に記載する。

再実行（GPU commandは直列に実行）:

```powershell
$env:HF_HUB_OFFLINE='1'
.venv\Scripts\python.exe benchmark_phase35.py C:\Users\slowh\Downloads\IMG_7121.mp4 --experiments A,B,B2 --repeats 3
.venv\Scripts\python.exe evaluate_phase35.py reports/phase35/A.json reports/phase35/B.json reports/phase35/B2.json
.venv\Scripts\python.exe benchmark_dual_rate.py C:\Users\slowh\Downloads\IMG_7121.mp4
.venv\Scripts\python.exe evaluate_phase35.py reports/phase35/D.json
.venv\Scripts\python.exe -m pytest -q
.venv\Scripts\python.exe -m pip check
.venv\Scripts\python.exe report_phase35.py
```

## Phase 3.4 profile履歴（元記録を保持）

比較用の全履歴。Phase 3.4のtier判定は今回のtierに流用しない。失敗profileを削除せず、詳細理由は元の[PHASE34](PHASE34.md)へ。

| Prior profile | Brain mean (s) | Main | Extended | Prior eligible |
|---|---:|---|---|---|
| class_384 | 4.8510 | False | [0, 0] | False |
| class_both | 4.5297 | False | [2, 2] | False |
| class_both_256_fp16 | 1.9036 | False | [2, 2] | False |
| class_both_288_fp16 | 2.6148 | False | [2, 2] | False |
| class_both_320_fp16 | 3.4255 | False | [2, 2] | False |
| class_both_336_fp16 | 3.3925 | False | [2, 2] | False |
| class_both_fp16 | 4.6861 | False | [2, 2] | False |
| class_constrained | 5.6710 | True | [7, 7] | False |
| class_constrained_fp16 | 5.7553 | True | [6, 6] | False |
| compact | 5.3893 | False | [0, 0] | False |
| compact_exact | 4.8577 | False | [1, 1] | False |
| json | 8.0993 | True | [8, 8] | True |
| json_fp16 | 8.0927 | True | [7, 7] | False |
| json_linear | 3.6777 | True | [8, 8] | True |
| json_linear_efficient | 0.0447 | False | [0, 0] | False |
| json_linear_efficient_expanded | 0.0138 | False | [0, 0] | False |
| json_linear_efficient_expanded_fixed | 3.3862 | True | [7, 7] | False |
| json_linear_uninstrumented | 3.5785 | True | [8, 8, 8] | True |
| json_uninstrumented | 8.1795 | True | [8, 8] | True |
| minified_linear_256 | 3.2430 | True | [6, 6] | False |
| minified_linear_288 | 3.2394 | True | [6, 6] | False |
| minified_linear_320 | 3.1907 | True | [6, 6] | False |
| minified_linear_336 | 3.2090 | True | [6, 6] | False |
| minified_linear_384 | 3.2650 | True | [6, 6] | False |
| minimal_256 | 2.0660 | False | [0, 0] | False |
| minimal_288 | 2.7891 | False | [0, 0] | False |
| minimal_320 | 3.6397 | False | [0, 0] | False |
| minimal_336 | 3.6088 | False | [0, 0] | False |
| minimal_384 | 5.0194 | False | [0, 0] | False |
