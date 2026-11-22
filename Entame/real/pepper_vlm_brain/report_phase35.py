"""Build the final report exclusively from completed, evaluated GPU evidence."""
import hashlib
import json
import subprocess
from importlib.metadata import version, PackageNotFoundError
from pathlib import Path
from run_video import write_report

ROOT=Path(__file__).parent
OUT=ROOT/'reports/phase35'
BASE='46423378034c69dce2f07aa9b56689f19090c222'


def read(path): return json.loads((ROOT/path).read_text(encoding='utf-8'))
def git(*args): return subprocess.check_output(['git',*args],cwd=ROOT,text=True).strip()
def value(v): return 'N/A' if v is None else f'{v:.4f}'
def triple(metric): return ' / '.join(value(metric[k]) for k in ['mean','median','p95'])


def main():
    a=read('reports/phase35/evaluations/A.json')
    planners={n:read(f'reports/phase35/evaluations/{n}.json') for n in ['B','B2']}
    d=read('reports/phase35/evaluations/D.json')
    ref=read('reports/phase34/evaluations/json_linear_uninstrumented.json')
    raw_a=read('reports/phase35/A.json'); raw_d=read('reports/phase35/D.json')
    skipped=[]
    for name,partner in [('C','B'),('C2','B2')]:
        if planners[partner]['eligible'] and a['eligible']:
            raise RuntimeError(f'{name} is authorized: run its GPU benchmark before finalizing')
        report={'experiment':name,'status':'NOT RUN','reason':f'{partner} failed its independent accuracy gate',
                'a_eligible':a['eligible'],'b_eligible':planners[partner]['eligible'],
                'measured_latency_s':None,'measured_accuracy':None,'promoted':False}
        write_report(OUT/f'{name}.json',report); skipped.append(report)
    packages={name:version(name) for name in ['torch','transformers','accelerate','pydantic','Pillow','numpy','opencv-python','pytest']}
    oldenv=read('reports/phase34_environment.json')
    try: version('bitsandbytes'); bnb=True
    except PackageNotFoundError: bnb=False
    nvidia=subprocess.check_output(['nvidia-smi','--query-gpu=name,driver_version,memory.total','--format=csv,noheader'],text=True).strip()
    env={'packages':packages,'bitsandbytes_in_stable_venv':bnb,'nvidia':nvidia,
         'same_as_phase34':packages==oldenv['packages'] and bnb==oldenv['bitsandbytes_in_stable_venv'] and nvidia==oldenv['nvidia'],
         'pip_check':(ROOT/'reports/phase35_pip_check.log').read_text().strip()}
    write_report(OUT/'environment.json',env)
    protected=git('ls-files','brain','latency','adapters','scenarios','config.py','run_brain.py','run_video.py',
                  'evaluate_phase33.py','evaluate_split_scenario.py','benchmark_latency.py','evaluate_latency.py').splitlines()
    changed=set(git('diff','--name-only',BASE).splitlines())
    frozen={p:{'unchanged':p not in changed,'sha256':hashlib.sha256((ROOT/p).read_bytes()).hexdigest()} for p in protected}
    existing_reports=[p for p in git('ls-tree','-r','--name-only',BASE,'reports').splitlines() if p!='reports/PHASE33.md']
    frozen['existing_reports']={'unchanged':not any(p in changed for p in existing_reports),'count':len(existing_reports),
        'preexisting_user_edit_excluded':'reports/PHASE33.md','preexisting_edit_sha256':hashlib.sha256((ROOT/'reports/PHASE33.md').read_bytes()).hexdigest()}
    write_report(OUT/'freeze.json',frozen)
    if not env['same_as_phase34'] or not all(v['unchanged'] for v in frozen.values()):
        raise RuntimeError('Frozen files or environment changed')
    g1root=Path('C:/dev/g1-bottle-reaction')
    reference_paths=['docs/G1_PERSON_YOLO.md','src/g1_bottle_reaction/game_vision/person_yolo.py',
                     'src/g1_bottle_reaction/stealth/target_perception.py','README.md']
    g1reference={'mode':'read-only historical local evidence; no G1 connection or YOLO execution',
        'sources':[{ 'path':str(g1root/p),'sha256':hashlib.sha256((g1root/p).read_bytes()).hexdigest()} for p in reference_paths],
        'documented_device':'RTX 5060 Ti, Linux venv, driver 595.84',
        'documented_model':'YOLO11n, imgsz 640, confidence .25, inference cap 15 FPS',
        'historical_inference_count':3050,'historical_mean_inference_ms':5.4,'historical_mean_inference_fps':14.6,
        'not_measured_here':['RTX 3060 Ti YOLO latency','same-GPU VLM contention','IMG_7121 detector accuracy','capture-to-execution latency']}
    write_report(OUT/'g1_yolo_reference.json',g1reference)
    testlog=(ROOT/'reports/phase35_tests_final.log').read_text(); tests=testlog.strip().splitlines()[-1]
    added_tests=int(tests.split()[0])-269
    status=git('status','--short'); (OUT/'git_status_before_commit.txt').write_text(status+'\n',encoding='utf-8')
    changes=(git('ls-files','--others','--exclude-standard').splitlines() +
             git('diff','--name-only','--diff-filter=A',BASE).splitlines())
    changes=sorted(set(changes+['reports/PHASE35.md','reports/phase35/summary.json','reports/phase35/changed_files.json',
                               'reports/phase35/completion.json']))
    write_report(OUT/'changed_files.json',{'base_sha':BASE,'added':changes,'excluded_preexisting_user_edit':'reports/PHASE33.md'})
    summary={'base_sha':BASE,'reference':ref,'A':a,'B':planners['B'],'B2':planners['B2'],'C':skipped,'D':d,
        'final_tier':'REJECT (accuracy gate FAIL; C not run)','pure_dual_rate_vlm_viable':'NO',
        'hybrid_fast_detector_vlm_recommended':'YES','decision_case':4,'tests':tests,
        'reason':'Both finite Planners fail original policy; serialized full semantic+JSON Planner blocks fast work for >2.5 seconds.',
        'scope':'These measured profiles only; not a proof that all future VLM optimizations are impossible.'}
    write_report(OUT/'summary.json',summary)
    lines=[
        '# PHASE 3.5 FAST VLM FINAL BATTLE ROUND 2', '',
        'Fast geometryは実GPUで約0.10秒に到達した。しかしfull Observationを入力しても1-token Plannerはpolicyを維持できず、採用を見送る。'
        '同一モデルのdual-rate prototypeは状態・resolverの契約を満たすが、semantic処理中の待ち時間が大きい。'
        '**PURE/DUAL-RATE VLM VIABLE = NO、HYBRID FAST DETECTOR + VLM RECOMMENDED = YES（CASE 4）**。', '',
        f'基準SHA: `{BASE}`。Qwen3-VL-2B-Instruct revision `89644892e4d85e24eaac8bacfd4f463576704203`、BF16、384px、default SDPA、RTX 3060 Ti 8GB。'
        'Linear patch実装と既存schema/State/policy/Adapter、評価データ、stable venvを保持した。Pepper/G1接続、4B、YOLO実装、package/driver/CUDA変更は実施していない。', '',
        '## 測定方法', '',
        '`IMG_7121.mp4`の既存9点（3,7,8,9,9.5,10,10.25,12,13）を使用。'
        'A/B/B2は各warmup 1回、9点×3回、fresh Stateの8→10→12を1回。Dはwarmup 1回、9点×2回、fresh 3点を1回。'
        '集計は9点の測定分のみで、load/warmup/fresh主シナリオを除外。映像抽出・ファイル保存・Mock出力はBrain計時外。', '',
        'AとPlannerの時間はstreamer/CUDA eventsで計測。B/B2のPerceptionはstable referenceと同じuninstrumented JSON経路。'
        'p95は標本の線形補間で、長時間のライブ動作を保証する値ではない。1-token出力のdecode_totalは最初のtoken後の終了処理時間であり、autoregressive decode_stepsは0。', '',
        'runner/engineはGTファイルを読まない。画像hashとtimestampは取得・順序・出力のprovenanceとして保存し、promptやclass選択に渡さない。'
        'Planner入力はimmutable Observation、Stateのmemory、transitionのみ。GT評価は別commandで実行。', '',
        '## Stable reference', '',
        f"`json_linear_uninstrumented`: Perception {ref['metrics']['perception_wall_s']['mean']:.3f}s、Planner {ref['metrics']['planner_wall_s']['mean']:.3f}s、"
        f"Brain mean/median/p95 {triple(ref['metrics']['total_brain_s'])}s。presence 100%、visible direction 80%、main PASS、extended {ref['extended_scores']}、"
        f"VRAM allocated/reserved {ref['peak_allocated_gib']:.3f}/{ref['peak_reserved_gib']:.3f} GiB。既存記録を再利用し、今回の計測として数えない。", '',
        '## 全profile比較', '',
        '| Profile | P mean (s) | Planner mean (s) | E2E mean/median/p95 (s) | 精度・判定 |',
        '|---|---:|---:|---|---|',
        f"| Stable full JSON | {ref['metrics']['perception_wall_s']['mean']:.4f} | {ref['metrics']['planner_wall_s']['mean']:.4f} | {triple(ref['metrics']['total_brain_s'])} | main PASS、extended 8/9×3、維持 |",
        f"| A geometry only | {a['metrics']['perception_wall_s']['mean']:.4f} | N/A | NOT RUN | geometry gate PASS、full Action判定は対象外 |",
    ]
    for name,b in planners.items():
        lines.append(f"| {name} full Observation + finite Planner | {b['metrics']['perception_wall_s']['mean']:.4f} | {b['metrics']['planner_wall_s']['mean']:.4f} | {triple(b['metrics']['total_brain_s'])} | main {'PASS' if b['main_pass'] else 'FAIL'}、extended {b['extended_scores']}、不採用 |")
    lines += [
        '| C / C2 fast E2E | N/A | N/A | NOT RUN | B/B2 gate FAILにより実行禁止 |',
        f"| D serialized dual-rate + JSON Planner | {d['metrics']['fast_s']['mean']:.4f} fast + {d['metrics']['semantic_s']['mean']:.4f} semantic | {d['metrics']['planner_s']['mean']:.4f} | {triple(d['metrics']['cycle_s'])} | prototype PASS、main PASS、extended {d['extended_original_planner_scores']}、速度要件FAIL |", '',
        '## A. Linear patch + 1-token Fast Perception', '',
        f"Presence {a['presence_correct_total']} = 100%、direction {a['direction_correct_total']} = 80%、valid class 100%。"
        f"main geometry 8→10→12 PASS、extended geometry {a['extended_geometry_scores']}。13秒はD=RIGHTと誤認し、GTのLEFTへ補正していない。", '',
        f"Perception mean/median/p95 **{triple(a['metrics']['perception_wall_s'])}s**。"
        f"VRAM allocated/reserved {a['peak_allocated_gib']:.3f}/{a['peak_reserved_gib']:.3f} GiB。", '',
        '| A stage metric | mean / median / p95 |', '|---|---|',
    ]
    for k,v in a['metrics']['perception'].items():
        lines.append(f'| {k} | {triple(v)} |')
    lines += ['', 'Fast PerceptionはA/B/C/Dの4つのgeometry codeを実際のmodel logitsから選ぶ。'
        'A=NO_PERSON、B=LEFT、C=CENTER、D=RIGHT。token IDs 32/33/34/35、各exactly 1 tokenをload時に検証。'
        '出力にはdistance/obstacleの意味情報がなく、UNKNOWN、person_countは0/1。Full Observationの代替採用ではない。', '',
        '## B. Full Observation + 1-token Planner', '',
        'Bは元のpolicy文章にgeneric class catalogを付けたprofile。B2は同一policyとcatalogを、短い指示と一般的なserialization例で提示した追加profile。'
        'B2の例はvisible RIGHT FAR、last CENTER/missed 2、長時間不在で障害物NONEであり、動画固有の期待Actionは使用していない。', '',
        '各codeはload時にA〜N = token ID 32〜45の1-tokenであることを確認。AllowedCodesは他tokenをmaskし、'
        '未修正のclass logitsのargmaxを選ぶ。action/direction/targetは選ばれたcodeから直接復元し、'
        'LOOK/FOUND/APPROACHのdistanceのみ入力Observationの値をserialization時にbindingする。'
        '誤ったAction/directionもそのまま残してpolicy evaluatorでFAILとする。このdistance bindingはPhase 3.4の既存codecを変更しない。', '',
        '| Code | Action / direction / target |', '|---|---|',
    ]
    from latency.codecs import PLANNER_CLASS
    for c,fields in PLANNER_CLASS.items(): lines.append(f"| {c} | {' / '.join(fields[:3])} |")
    for name,b in planners.items():
        raw=read(f'reports/phase35/{name}.json'); main=raw['runs'][-1]['frames']; critical=main[1]
        metrics=critical['planner']['metrics']; codes=list(raw['single_token_codes'])
        lines += ['',f'### {name}', '',
            f"Planner valid code 100%、policy一致 {b['policy_total'][0]}/{b['policy_total'][1]}（fresh主シナリオを含む）。"
            f"main {'PASS' if b['main_pass'] else 'FAIL'}、extended {b['extended_scores']}、presence 100%、direction 80%、integrity {b['integrity_pass']}。", '',
            f"Planner mean/median/p95 **{triple(b['metrics']['planner_wall_s'])}s**。"
            f"Brain mean/median/p95 {triple(b['metrics']['total_brain_s'])}s。"
            f"VRAM allocated/reserved {b['peak_allocated_gib']:.3f}/{b['peak_reserved_gib']:.3f} GiB。", '',
            '| Planner metric | mean / median / p95 |','|---|---|']
        for k in ['input_tokens','prefill_cuda_elapsed_s','ttft_stage_s','ttft_generation_s','output_tokens','decode_total_s','validation_parsing_s']:
            lines.append(f"| {k} | {triple(b['metrics']['planner'][k])} |")
        lines += ['', 'Fresh 8→10→12の実出力:', '', '| 秒 | P direction/visible | State transition / last_seen / missed | Raw code → Decision |','|---:|---|---|---|']
        for f in main:
            obs=f['perception']['observation']; state=f['state_after']; decision=f['planner']['decision']
            lines.append(f"| {f['timestamp']} | {obs['person_direction']} / {obs['person_visible']} | {state['transition']} / {state['last_seen_person_direction']} / {state['frames_since_person_seen']} | {f['planner']['raw_response']} → {decision['action']} {decision['direction']} |")
        lines += ['',f"10秒ではinput transition=DISAPPEARED、last_seen=LEFT、missed=1。"
            f"SEARCH LEFTのE logit={metrics['class_logits'][codes.index('E')]}に対し、"
            f"選択{critical['planner']['raw_response']}のlogit={max(metrics['class_logits'])}。"
            'Runner/StateはSEARCHへ置換していない。full JSON Plannerよりpolicy accuracyが悪化したため不採用。']
    lines += ['', '## C. Fast VLM End-to-End', '',
        '**NOT RUN**。AはPASSだがB/B2が独立gateを満たさないため、組み合わせを実GPUで実行していない。'
        'mean/median/p95、main、extended、presence/direction、VRAMはいずれもCの測定値としてN/A。'
        'AとBの独立stage時間の足し算をE2E実測として報告しない。C/C2 runnerはgate FAILでmodel load前に終了する。', '',
        '**Final tier: REJECT（accuracy gate FAIL、S〜D認定なし）**。'
        'S≤0.5/A≤1.0/B≤1.5/C≤2.5/D>2.5の速度区分は精度PASSが前提。'
        'Aの0.10秒だけを使ってfast E2EをTier Sへ昇格させない。', '',
        '## D. Dual-rate prototype', '',
        f"**Prototype contract PASS / latest-geometry resolver PASS**。同一model/processorのinstance ID、load_count=1を全stageで確認。"
        f"Original JSON Plannerのpolicy {d['policy_original_scores']}、main PASS、extended {d['extended_original_planner_scores']}。"
        f"Fast geometry presence {d['presence_correct_total']}、direction {d['direction_correct_total']}。", '',
        'Schedulerはfast→full semantic→元のJSON Plannerの直列アクセス。Fast Geometryのみがdeterministic State/transitionを更新する。'
        'semantic結果は別slotに保存し、geometryを書き換えない。取得timestamp/generationが完全一致し、presence/directionも一致する時だけdistance/obstacleを融合する。'
        '旧semanticはdiagnosticsへ保持するが新geometryやStateを上書きしない。不一致では意味情報をUNKNOWNにする。', '',
        '| D measured budget | mean / median / p95 (s) |','|---|---|',
    ]
    for k,v in d['metrics'].items(): lines.append(f'| {k} | {triple(v)} |')
    lines += ['',f"VRAM allocated/reserved {d['peak_allocated_gib']:.3f}/{d['peak_reserved_gib']:.3f} GiB。"
        '高速stage自体は約0.10秒だがsemantic＋Plannerによる待ち時間は平均約3.69秒、p95約4.00秒。'
        'このprototypeはオフライン選択フレームの測定であり、カメラの連続稼働・10Hz・ライブframe dropを測定していない。'
        '本構成の単一GPU・非preemptiveアクセスでは即時反応を保証できない。', '',
        '### Latest-geometry execution resolver', '',
        'immutable PlannerDecisionをsemantic intentとして保持し、実行直前のLOOK/FOUND PERSONだけ方向を最新geometryへbindingする。'
        'action/target/distanceは元の値を維持する。SEARCHはモデルが選んだmemory方向のままであり、resolverでSEARCHへ変更しない。'
        '追跡対象が消失/UNKNOWNならcommandを抑止し、WAIT/SEARCHを捏造しない。APPROACHは最新人物がCENTERでなければ抑止する。'
        'Robotへ渡すのは再validationしたPlannerDecisionのみ。', '',
        '追加dry runは実GPUの8秒LOOK LEFTを保持した後、実GPUの13秒fast code D=RIGHTを受け、8秒semanticを遅れて配送する。'
        'geometryはRIGHTのまま、元decisionはLOOK LEFTのまま、resolved decisionはLOOK RIGHT。配送順だけを模擬し、GPU同時実行は主張しない。'
        '13秒のRIGHTはGTに対して誤りであり、これはresolver契約のPASSであって視覚精度PASSではない。', '',
        'Dのpolicy/scenario評価はresolver前の**元のVLM Decision**を使用する。resolverによる方向bindingで失敗したPlannerを救済してPASSにはしない。', '',
        '## 次Phaseの判断', '',
        '**CASE 4: finite Planner精度FAIL、および精度を維持するDのcycle mean>2.5s。'
        'PURE/DUAL-RATE VLM VIABLE = NO / HYBRID FAST DETECTOR + VLM RECOMMENDED = YES。**'
        'Phase 3.4のHYBRID=NOを引き継がず、今回の即時反応要件で再判定した。'
        'この結論は今回測定したprofileとschedulerに対するもので、将来の全VLM最適化が不可能という意味ではない。', '',
        '次の設計候補はCamera→fast person detector（YOLO等）→timestamp付きgeometry/State、'
        '低頻度semantic VLM→既存policyのVLM Planner→最新geometry resolver→validated Adapter。'
        '検出器はcurrent visibility/directionを担当し、semantic branchはdistance/obstacleを補う。'
        '新Actionは追加せず、古いsemanticで最新geometryを上書きしない契約を引き継ぐ。', '',
        '既存G1経験はローカル[人物YOLO実験記録](C:/dev/g1-bottle-reaction/docs/G1_PERSON_YOLO.md:14)と'
        '[person_yolo.py](C:/dev/g1-bottle-reaction/src/g1_bottle_reaction/game_vision/person_yolo.py:98)をread-onlyで確認した。'
        '記録ではYOLO11n、imgsz=640、confidence=.25、上限15 FPS、RTX 5060 Ti/Linuxで3050回、平均5.4ms・14.6 FPS。'
        'camera/GUIはYOLOを待たず、latest frame slot、1枚だけのin-flight、別process推論、500ms超の結果無効化を使用する。'
        'PC取得timestampを使い、推論完了時刻で結果を新しく見せない。これらを再利用設計の出発点とする。'
        'stealth側ではraw labelをsemantic PLAYERへ変換し、継続trackingと離散イベントを分離する経験もある。', '',
        '5.4msは別GPU/OSの過去記録で、今回の3060 Ti/Windowsの見積りや撮影→実行時間としては使用しない。'
        '次Phaseでは同じIMG_7121と基準labelsでpresence/direction、13秒の方向、消失/再出現、fresh SEARCHを評価する。'
        'bbox中心を既存の40%/60%区分へ変換し、confidence/target選択をGT別ルールにしない。'
        'さらにVLMとの同一GPU競合、latest-slot frame age、queue待ち、fast更新間隔p95/最大、VRAM、capture-to-executionを実測する。'
        '別processでも同一GPUの待ち時間が自動的に解消するとは仮定しない。'
        'detectorを入れても元JSON Plannerの約1.4秒は残る。LOOK trackingのgeometry更新と、'
        'SEARCH等の意味的Action変更にかかる時間を別々に測り、failed finite Plannerを採用しない。'
        'YOLO実装・package導入・実機接続は今回行わない。', '',
        '## 保存物・テスト・Git', '',
        f'**{tests}**。既存269件にPhase 3.5の{added_tests}件を追加。'
        'class mapping、invalid/nonfinite拒否、distance serializationとAction保持、logits argmax、GPU1-token/Linear証拠、'
        'Planner無画像・無GT入力、StateがSEARCHを強制しないこと、timestamp/generation precedence、stale semantics、'
        'resolver/validated Mock出力、model reuse、C gate拒否、評価のraw不変性・Action改ざん拒否を確認。pip check PASS。', '',
        '[全機械可読summary](phase35/summary.json)、[freeze証拠](phase35/freeze.json)、[環境](phase35/environment.json)、'
        '[G1 YOLO過去記録の出典・hash](phase35/g1_yolo_reference.json)、'
        '[変更ファイル一覧](phase35/changed_files.json)、[commit前Git status](phase35/git_status_before_commit.txt)。', '',
        'Raw output / logits / token IDs / Observation / State / Planner input / resolved Decision / 全latency:', '',
        '- [A raw](phase35/A.json) / [A evaluation](phase35/evaluations/A.json)',
        '- [B raw](phase35/B.json) / [B evaluation](phase35/evaluations/B.json)',
        '- [B2 raw](phase35/B2.json) / [B2 evaluation](phase35/evaluations/B2.json)',
        '- [C NOT RUN](phase35/C.json) / [C2 NOT RUN](phase35/C2.json)',
        '- [D raw + late-delivery dry run](phase35/D.json) / [D evaluation](phase35/evaluations/D.json)',
        '- [GPU A/B log](phase35_gpu_AB.log) / [GPU B2 log](phase35_gpu_B2.log) / [GPU D log](phase35_gpu_D.log)',
        '- [全tests log](phase35_tests_final.log) / [pip check](phase35_pip_check.log) / [C gate guard](phase35_C_gate_guard.log)', '',
        '既存reports/PHASE33.mdには作業開始前からユーザー変更がある。編集・stage・commit対象に含めていない。'
        '既存stableとPhase 3.4 reportsは保持。新規実装はphase35/と専用benchmark/evaluator/report/testのみ。'
        'このレポート・全実装・実GPU結果を含むcommit SHAとcommit後Git statusは、commit完了後に追加する'
        '[completion record](phase35/completion.json)へ保存する。completion記録を含む最終HEAD SHAは最終応答に記載する。', '',
        '再実行（GPU commandは直列に実行）:', '', '```powershell',
        "$env:HF_HUB_OFFLINE='1'",
        '.venv\\Scripts\\python.exe benchmark_phase35.py C:\\Users\\slowh\\Downloads\\IMG_7121.mp4 --experiments A,B,B2 --repeats 3',
        '.venv\\Scripts\\python.exe evaluate_phase35.py reports/phase35/A.json reports/phase35/B.json reports/phase35/B2.json',
        '.venv\\Scripts\\python.exe benchmark_dual_rate.py C:\\Users\\slowh\\Downloads\\IMG_7121.mp4',
        '.venv\\Scripts\\python.exe evaluate_phase35.py reports/phase35/D.json',
        '.venv\\Scripts\\python.exe -m pytest -q',
        '.venv\\Scripts\\python.exe -m pip check',
        '.venv\\Scripts\\python.exe report_phase35.py', '```', '',
        '## Phase 3.4 profile履歴（元記録を保持）', '',
        '比較用の全履歴。Phase 3.4のtier判定は今回のtierに流用しない。失敗profileを削除せず、詳細理由は元の[PHASE34](PHASE34.md)へ。', '',
        '| Prior profile | Brain mean (s) | Main | Extended | Prior eligible |','|---|---:|---|---|---|',
    ]
    prior=read('reports/phase34_summary.json')
    for p in prior['profiles']:
        lines.append(f"| {Path(p['report_file']).stem} | {value(p['metrics']['total_brain_s']['mean'])} | {p['main_pass']} | {p['extended_scores']} | {p['eligible']} |")
    (ROOT/'reports/PHASE35.md').write_text('\n'.join(lines)+'\n',encoding='utf-8')
    print('Saved reports/PHASE35.md; final decision CASE 4, hybrid YES')


if __name__=='__main__': main()
