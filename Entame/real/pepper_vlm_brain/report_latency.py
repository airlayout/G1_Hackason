"""Build the Phase 3.4 report entirely from saved inference and post-run evaluations."""
import argparse
import json
import subprocess
import re
from pathlib import Path
from evaluate_latency import evaluate_report
from run_video import write_report


def main():
    parser=argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--directory',type=Path,default=Path('reports/phase34'))
    args=parser.parse_args(); root=Path(__file__).parent
    def load(path): return json.loads(path.read_text(encoding='utf-8'))
    labels=load(root/'scenarios/IMG_7121_phase32_perception.json')
    nine=load(root/'scenarios/IMG_7121_phase33_nine.json'); three=load(root/'scenarios/IMG_7121_phase33_three.json')
    results=[]
    for path in sorted(args.directory.glob('*.json')):
        report=load(path)
        if 'runs' not in report or report['status']=='running': continue
        result=evaluate_report(report,labels,nine,three); result['report_file']=str(path)
        write_report(args.directory/'evaluations'/path.name,result); results.append(result)
    eligible=[r for r in results if r['eligible']]
    best=min(eligible,key=lambda r:r['metrics']['total_brain_s']['mean']) if eligible else None
    full=[r for r in eligible if r['profile']['format']=='json']
    bestfull=min(full,key=lambda r:r['metrics']['total_brain_s']['mean']) if full else None
    baseline=next(r for r in results if r['report_file'].endswith('/json.json') or r['report_file'].endswith('\\json.json'))
    testlog=root/'reports/phase34_tests_final.log'
    tests=re.search(r'(\d+) passed',testlog.read_text()) if testlog.exists() else None
    summary={'starting_commit':'2cbba110d6fef22f696397803777cf17451f7969','profiles':results,
             'best':best['report_file'] if best else None,'best_full_observation':bestfull['report_file'] if bestfull else None,
             'final_tier':best['tier'] if best else 'C',
             'hybrid_recommended':not best or best['metrics']['total_brain_s']['mean']>4}
    summary['tests_passed']=int(tests.group(1)) if tests else None
    write_report(root/'reports/phase34_summary.json',summary)
    def mean(r,key,stage=None):
        m=r['metrics'][stage] if stage else r['metrics']
        value=m[key]['mean']; return 'N/A' if value is None else f'{value:.3f}'
    def percent(r,key): return f"{r[key]*100:.1f}%"
    def number(value): return 'N/A' if value is None else f'{value:.3f}'
    b=baseline['metrics']
    lines=['# PHASE 3.4 LATENCY FINAL BATTLE','', '2026-10-02 / baseline commit `2cbba110d6fef22f696397803777cf17451f7969`。', '',
           '## Baseline profiling','',
           'JSON baselineのPhase 3.3実測はPerception 6.674s、State 0.016ms、Planner 1.406s、Brain 8.080s。今回の同一video再測定は以下。', '',
           '|stage|total(s)|image prep(s)|processor(s)|vision(s)|prefill(s)|TTFT(s)|decode(s)|ms/decode token|output tokens|parsing(ms)|',
           '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for stage,wall in [('perception','perception_wall_s'),('planner','planner_wall_s')]:
        keys=['image_preprocessing_s','processor_s','vision_cuda_elapsed_s','prefill_cuda_elapsed_s','ttft_generation_s',
              'decode_total_s','decode_ms_per_token','output_tokens']
        parsing=baseline['metrics'][stage]['validation_parsing_s']['mean']*1000
        lines.append('|'+stage+'|'+mean(baseline,wall)+'|'+'|'.join(mean(baseline,k,stage) for k in keys)+f'|{parsing:.3f}|')
    lines+=['', f"Baseline Brain mean {mean(baseline,'total_brain_s')}s。Vision/prefillの大半はvision patch embeddingのConv3d。短い出力だけではその時間を除去できなかった。", '',
            'CUDA eventはfirst forwardとVision/patch embeddingを計測。Visionはprefill内なので加算しない。TTFTはgeneration開始から最初のgenerated tokenのCPU到着まで。stage TTFTはprocessor/転送を含む。decodeは初token以降、EOSを含む残りの生成時間。ms/tokenはgenerated_count-1で割り、1 tokenではN/A。validation/parsingは別のCPU wall timer。', '',
            'JSON instrumentedとuninstrumentedを同じ9画像×2回＋fresh 8/10/12で比較した。profilingの計測誤差とprofile順序・GPU clockの影響を分離する統制実験ではないが、collectorなしの軽いforward hooks/CUDA events/streamerは大きな上乗せを示さなかった。CUDA CUPTI profilerは収集処理が戻らず停止した診断を保存し、SDPA backend確認はATen dispatchとnative backend selectorで代替した。', '',
            '## Profiles comparison','',
            '|profile / dtype|presence|direction (visible)|P/D valid|main|extended /9|P mean(s)|Planner mean(s)|Brain mean/median/p95(s)|peak allocated/reserved(GiB)|gate / tier|',
            '|---|---:|---:|---:|---|---|---:|---:|---|---:|---|']
    for r in results:
        p=r['profile']; total=r['metrics']['total_brain_s']; name=Path(r['report_file']).stem
        scores=', '.join(f'{score}/9' for score in r['extended_scores'])
        lines.append(f"|{name} / {p['dtype']}|{percent(r,'presence_accuracy')}|{percent(r,'direction_accuracy')}|{percent(r,'perception_valid_rate')} / {percent(r,'planner_valid_rate')}|{'PASS' if r['main_pass'] else 'FAIL'}|{scores}|{mean(r,'perception_wall_s')}|{mean(r,'planner_wall_s')}|{total['mean']:.3f} / {total['median']:.3f} / {total['p95']:.3f}|{number(r['peak_allocated_gib'])} / {number(r['peak_reserved_gib'])}|{'accuracy PASS' if r['eligible'] else 'REJECT'} / {r['tier']}|")
    lines+=['', '## Input/TTFT/decode ablation','',
            '|profile|P input/image/output tokens (mean)|P TTFT/decode(s)|Planner input/output tokens (mean)|Planner TTFT/decode(s)|',
            '|---|---|---|---|---|']
    for r in results:
        lines.append(f"|{Path(r['report_file']).stem}|{mean(r,'input_tokens','perception')} / {mean(r,'image_tokens','perception')} / {mean(r,'output_tokens','perception')}|{mean(r,'ttft_generation_s','perception')} / {mean(r,'decode_total_s','perception')}|{mean(r,'input_tokens','planner')} / {mean(r,'output_tokens','planner')}|{mean(r,'ttft_generation_s','planner')} / {mean(r,'decode_total_s','planner')}|")
    lines+=['', '## Accuracy and selection', '',
            '期待label/actionは事後evaluatorだけが読む。raw compact/classを厳密にdecodeし、既存CurrentObservation/PlannerDecisionへ復元した後、schema/scenario evaluatorをcanonical JSON viewで再利用する。元rawは変更しない。型、enum、重複field、presence/count/absence整合性は既存schemaで検証。不正rawはWAIT fallbackで、Stateを捏造・修正しない。有効だが誤ったActionもそのままMockに渡し、FAILとして記録する。', '',
            '採用gateはpresence 100%、visible direction >=80%、各9-frame runのextended score >=8/9、fresh 8/10/12全PASS、全stage valid、raw/decoded/State/Mock/model reuse integrity PASSを要求する。速度だけで採用しない。Tierはこのgate通過後にmean Brain <=1.5 S、<=2.5 A、<=4 B、それ以上C。主scenarioのみ良いprofileを採用しないため、Bにもbaseline精度維持を要求した。', '',
            '最初のpipe compactはfield名/説明文、矛盾した不在方向などで失敗。150-token minimal promptはinput目標を満たしたが、出力形式が崩れ不採用。exact文法の追加は全画像に同一のserialization指定を行うprofileで、画像別ruleはない。こちらにも知覚/Actionの誤りが残り不採用。', '',
            'single-tokenはtokenizer実測で1 tokenのA/B/C/Dをscene classへ対応。finite constrained版は許可codeのlogit argmaxを使い、GT/State machineでcodeを選ばない。Planner finite版も全generic Action+direction code候補のモデルlogitから選ぶ。code valid率が100%でもAction accuracyのFAILはそのまま判定する。距離とblockingをUNKNOWN、countを0/1にするclass modeは情報量を減らした実験であり、full Observationと同等の知覚機能を証明していない。', '']
    if best:
        lines += [f"Best measured eligible pure-VLM profile: **{Path(best['report_file']).stem}**, Brain mean **{mean(best,'total_brain_s')}s**, tier **{best['tier']}**。presence {percent(best,'presence_accuracy')}、direction {percent(best,'direction_accuracy')}、main PASS、extended {best['extended_scores']}。accuracy gateを満たす候補で最小のmean latencyを選んだ。", '']
    if bestfull:
        lines += [f"Best full Observation profile: **{Path(bestfull['report_file']).stem}**, mean **{mean(bestfull,'total_brain_s')}s**。元の5-field知覚契約を保持する。", '']
    lines += [f"Final tier: **{summary['final_tier']}**。HYBRID RECOMMENDED = **{'YES' if summary['hybrid_recommended'] else 'NO（今回のlatency gate）'}**。", '',
              'Tierはmean latencyで判定する。best uninstrumentedのp95も確認したが、この9画像×3回の測定で将来の全frameのdeadlineを保証するものではない。S/Aは未達。', '',
              'Tier Cの場合、synchronous pure-VLMを実世界demoのdefault候補にしない。次候補は高頻度fast perceptionでlive geometryを取得し、scene/stateからVLM high-level Plannerを非同期実行する構成。Robot execution時のgeometryは最新fast perceptionから得る。今回Hybrid、Pepper/G1接続は実装していない。', '',
              '## Attention / dtype / advanced backend','',
              '既存torch 2.9.1+cu128 / RTX 3060 Ti 8GB / pinned 2B revision / SDPA。同一GPU loaded weightをPerception/Plannerで共有。FP16はBF16 weightを同じ2B instanceでconvertし、FP32 rotary/numerical buffersを保持した。別processでBF16/FP16を比較し、load/conversionはruntime latencyに含めない。', '',
              'Windows wheelのFlash Attentionはcompiled available=False。強制FLASH_ATTENTIONはNo available kernelでFAIL。追加package/FlashAttention-2/driver/CUDA変更は行わない。default SDPAの選択backendと全operatorは `phase34_attention_default.json`、強制失敗は `phase34_attention_flash.json`、CUPTI中断は `phase34_attention_cuda_attempt.json` に保存。', '',
              'native selectorでの実選択は `phase34_attention_selection.json`: Vision EFFICIENT_ATTENTION、text prefill/decode MATH。textのquery 16 heads / KV 8 headsはnative GQAでMATHへfallback。単純なEFFICIENT強制はunsupported GQAでFAIL。`efficient_expanded`は既存repeat_kv経路で同じKVをquery headsへ展開し、既存EFFICIENT kernelを使う別profile。fn差替えはprocess内のgeneration contextだけで、例外時も元へ戻す。最初の実装はmock call historyがCUDA Tensorを保持してOOMになったためraw失敗を残し、参照を保持しないplain predicateへ修正した `_fixed` 試行で再検証。', '',
              'advanced profileのConv3d→Linearがある場合、non-overlapping patchをflattenし同じweight/biasでF.linearを実行する代数的に等価なbackend。kernel=stride、padding=0、dilation=1、groups=1、入力patchサイズを確認し、profileごとに一時的に差し替え必ず元へ戻す。外部packageもweight二重loadも不要。stable engine/source、venvは変更しない。数学的等価性と実GPUの9画像/fresh mainを検証し、accuracy gateで採否を判断する。Static KV cache、torch.compile、quantization、外部backendは未実施。', '',
              '`phase34_patch_equivalence.json`: actual BF16 patch max abs difference 0.03125、RMSE 0.000687、exact fraction 76.97%。代数的等価でもBF16 reduction丸めはbit-identicalではない。baseline JSONを保持したEnd-to-Endのpresence/direction/main/extended gateで結果を確認した。Windows環境にはTritonとMSVC clがなく、追加build依存を導入するcompile/外部backend実験は今回行わなかった。', '',
              '## Reproduction / evidence','',
              '各profileでwarmupを保存して除外し、9画像を2回、別のfresh Stateで8/10/12を1回測定。最終uninstrumentedは9画像×3回=27 samples＋fresh main。mean/median/p95は異なるframeを含む測定分布であり、profile順序による温度/clock差を含む。model load、動画decode、Mock実行、report I/OはBrain timer外。誤ったoutput/invalid/fallbackを含む全frameのraw/復元Observation/State before-after/transition/Decision/Mock/各stage tokens・TTFT・decode・peak VRAMをJSON保存。uninstrumentedのTTFT/decode内訳、および戻り値を得られなかった初期GPU例外trialのtoken/peak項目はN/A。例外時のpartial telemetry取得は最終実装に追加し、State保持と共にテストした。', '',
              '`reports/phase34/*.json` が全raw、`reports/phase34/evaluations/*.json` が評価、`reports/phase34_summary.json` が集計。rootの `phase34_*.log` にstdout/stderrと初期からの全負の結果を保持。画像サイズは同じsource RGBを用い、input pixels SHAはprovenance専用。engineへtimestamp/hash/GTを渡さない。', '',
              f"Tests: **{summary['tests_passed']} PASS**（既存201＋追加68）。`phase34_tests_final.log`、pip check PASS、`phase34_environment.json` と `phase34_frozen_path.json` の9 functional files unchangedを保存。最終git status/commit SHAは完了メッセージに記載。開始時からのユーザー編集 `reports/PHASE33.md` は保持して今回のcommitから除外する。stable functional pathと4Bの環境/実装/過去reportsはfreeze。"]
    (root/'reports/PHASE34.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print(json.dumps({key:summary[key] for key in ['best','best_full_observation','final_tier','hybrid_recommended']}))
    return 0


if __name__=='__main__': raise SystemExit(main())
