# IMG_7121: Phase 2A / Minimum Memory evaluation

2026-10-01 JST。**Phase 2A PASS / Phase 3 disappearance-to-SEARCH scenario FAIL**。
期待Actionをtimestampやframe indexから選ぶ処理は実装していません。
MemoryはVLMの有効なDecisionを記録し、次のVLM promptへ渡すだけです。

## Input and reproduction

- `C:\Users\slowh\Downloads\IMG_7121.mp4`: 29.5秒、910×512、30fps、885 frames。
- source SHA256: `3ecc220cd52007997078aa70782118e22f1d75c92884e3a8ebe8523f6a73a929`
- requested/actual timestamps: 2,6,8,10,12,16,20,24 seconds、全て一致。
- frame indices: 60,180,240,300,360,480,600,720。
- 各画像を独立したimageとして処理。同じmodel instanceを再利用。
- final profile: Qwen3-VL-2B-Instruct / BF16 / SDPA / legacy prompt / 384px / 96 max tokens。
- repeated runで、入力PNG hash・frame indices・Decision・Memory状態が全て一致。
- 生成token数53〜55。max_new_tokens=96による切り詰めはありません。

```powershell
cd C:\dev\robot-vlm-brain
.\.venv\Scripts\python.exe run_video.py 'C:\Users\slowh\Downloads\IMG_7121.mp4' --timestamps 2,6,8,10,12,16,20,24 --memory --save-frames --monitor-gpu
```

## Timeline: actual VLM results, no override

| seconds | frame | action | direction | target | distance | confidence |
|---:|---:|---|---|---|---|---:|
| 2 | 60 | LOOK | LEFT | PERSON | NEAR | 0.95 |
| 6 | 180 | LOOK | LEFT | PERSON | NEAR | 0.95 |
| 8 | 240 | LOOK | LEFT | PERSON | NEAR | 0.95 |
| 10 | 300 | LOOK | CENTER | PERSON | MID | 0.95 |
| 12 | 360 | LOOK | LEFT | PERSON | NEAR | 0.95 |
| 16 | 480 | LOOK | CENTER | PERSON | NEAR | 0.95 |
| 20 | 600 | LOOK | LEFT | PERSON | NEAR | 0.95 |
| 24 | 720 | LOOK | CENTER | PERSON | NEAR | 0.95 |

10秒の人手確認では人物がいません。しかしraw VLM responseは
`LOOK CENTER / PERSON / MID`、scene_summary=`person visible in center`でした。
memory_beforeは last_seen_person_direction=LEFT、person_visible_last_frame=true、
frames_since_person_seen=0。VLMが人物を誤認したため、memory_afterの最後の方向がCENTERへ
更新されました。SEARCH LEFTは選択されず、消失検出もFAILです。
12秒の人物位置/actionは適切でも、その直前のabsenceが検出されていないため、
完全なreappearance state transitionをPASSにはしません。

raw JSONとparsed Decisionの一致、全frame間のMemory continuityは検査PASSです。
これは正しい画像認識を保証しません。confidence=0.95でも誤認が起きました。

## Performance

各値は指定8frameの実測。latencyはloadを除く、前処理+生成+decode。

| setting | 448px + memory | 384px + memory | 384px repeat |
|---|---:|---:|---:|
| PIL input | 448×252 | 384×216 | 384×216 |
| Processor patch alignment | 448×256 | 384×224 | 384×224 |
| image tokens | 112 | 84 | 84 |
| input tokens (image+text+special) | 643 | 615 | 615 |
| prompt characters | ~2241 | ~2241 | ~2241 |
| generated tokens | 53–55 | 53–55 | 53–55 |
| mean preprocessing seconds | 0.014 | 0.012 | 0.011 |
| mean generation seconds | 8.512 | 7.140 | 7.002 |
| mean latency seconds | 8.527 | 7.152 | 7.013 |
| peak allocated GiB | 4.139 | 4.133 | 4.133 |
| peak reserved GiB | 4.164 | 4.156 | 4.156 |
| sampled driver memory peak MiB | 5088 | 5076 | 5057 |
| sampled system GPU utilization mean | 35.4% | 36.3% | 36.4% |

384pxで平均latencyが約16.1%減少。6/8/12秒のLEFT判定を維持し、20秒のLEFT判定も適切でした。
10秒の人物誤認は解消していません。videoの標準を384pxにしました。
standalone image/Webcamは従来の448px。Driver/CUDA/dependenciesの変更はありません。

短いprompt、観測説明を先に生成するprompt、user側へpolicyを移す試行も行いましたが、
必須field欠落やFOUND CENTER/WAITへの偏り、方向判断悪化が起き、採用しませんでした。
途中の試行は `video_IMG_7121_memory448_{compact,grounded,final,userpolicy}.json` に残しています。
これらはhistorical experimentsで、名前のfinalはその試行の出力形式を示し、採用profileではありません。
今後compactを実行すると、現在の実験用promptを使用します。標準のlegacy promptは維持しました。

## Verification

```text
PHASE 2A PRERECORDED VIDEO
Local MP4 open: PASS
Timestamp seek/frame extraction: PASS
Existing VLM Brain reuse: PASS
Multiple frames inference: PASS
Structured Action: PASS
Report JSON output: PASS
Saved input frames: PASS
Tests: PASS (72)
Existing Phase 1 regression: PASS

PHASE 3 MINIMUM MEMORY
Memory implementation/unit tests: PASS
Actual disappearance detection: FAIL
Actual SEARCH previous direction: FAIL
Complete reappearance transition: NOT VERIFIED (previous absence was missed)
```

- `phase2a3-tests.json`: 72 passed、schema/parser/camera/video/Memory/scenario tests。
- `phase1-regression-video.json`: 実GPUでLOOK RIGHT / PERSON / MID / 0.95、fallback=false。
- `video_IMG_7121_memory384_legacy.json`: 最終採用profileの8frame、raw/metrics/Memory。
- `video_IMG_7121_memory384_repeat.json`: 同条件の再実行。
- `scenario_IMG_7121_final.json`: 明示的なFAILと各checkの理由。
- `video_IMG_7121_summary.json`: 計算した性能値、timeline、再現性の照合結果。
- 保存PNG: `reports/frames/video_IMG_7121_memory384_legacy/`（Git除外）。

## Changed files

- `.gitignore`: 元videoとprivate input framesを除外。
- `adapters/video_source.py`: local video ownership、metadata、timestamp seek、RGB image extraction。
- `run_video.py`: interval/timestamps、incremental JSON reports、frame保存、任意Memory/telemetry。
- `brain/images.py`: source非依存の共通PIL前処理。
- `brain/memory.py`: 4つの状態、fallback時の観測保持、SEARCHとsightingの区別。
- `brain/vlm.py`, `brain/telemetry.py`: input/prompt/token/latency/VRAM/GPU計測。
- `brain/prompt.py`, `config.py`, `run_image.py`: 既存promptの保持、実験profile、token budget。
- `evaluate_scenario.py`, `scenarios/IMG_7121_hide_and_seek.json`: 推論後だけの汎用評価と人手annotation。
- `tests/test_video.py`, `tests/test_memory.py`, `tests/test_scenario.py`: meaningful regression tests。
- `README.md`: video/Memory、FAILの制約、実行手順、optional OBSの手動経路。
- `reports/`: 実測と成功/失敗の証拠。model weights、元MP4、画像はcommitしません。

次の検証候補は、Actionから独立したstructured person-presence/direction観測、または
VLMによる観測と判断の2段階化です。SEARCHをコードで強制せず、空の椅子の誤認を
解消できるかを別のシーンでも評価する必要があります。今回は4Bへ進んでいません。
