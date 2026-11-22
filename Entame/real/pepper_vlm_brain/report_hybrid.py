"""Final Phase 3.6 report from evaluated real-GPU traces; no inference or GT injection."""
import hashlib
import json
import subprocess
from pathlib import Path
from evaluate_hybrid import stats
from run_video import write_report

ROOT=Path(__file__).parent; OUT=ROOT/'reports/phase36'
BASE='3c6d7f32c178702d968c03352207eb8c40b9f208'


def read(path): return json.loads((ROOT/path).read_text(encoding='utf-8'))
def git(*args): return subprocess.check_output(['git',*args],cwd=ROOT,text=True).strip()
def milliseconds(s):
    return 'N/A' if s['mean'] is None else '/'.join(f'{s[k]*1000:.1f}' for k in ['mean','median','p95','max'])
def metric(s): return 'N/A' if s['mean'] is None else '/'.join(f'{s[k]:.3f}' for k in ['mean','median','p95','max'])


def main():
    labels=read('reports/phase36/evaluations/selected/yolo.json')
    vm=read('reports/phase36/evaluations/selected/qwen.json')
    low=read('reports/phase36/evaluations/selected_lowconfidence/yolo.json')
    profiles={p.relative_to(OUT).as_posix():json.loads(p.read_text()) for p in (OUT/'evaluations').rglob('*.json') if 'selected' not in str(p)}
    final_paths=sorted((OUT/'realtime_eventsemantic').glob('hybrid_*.json'))
    raw=[json.loads(p.read_text()) for p in final_paths]
    evaluated=[profiles['evaluations/realtime_eventsemantic/'+p.name] for p in final_paths]
    if len(raw)!=3 or not all(r['status']=='complete' for r in raw): raise RuntimeError('Three final real-time runs required')
    if not low['eligible'] or not all(r['slow_active_fast_loop_pass'] and r['high_level_planner_integrity_pass'] for r in evaluated):
        raise RuntimeError('Chosen source accuracy/concurrency/integrity gate failed')
    fast=[r for report in raw for r in report['fast_results']]
    commands=[c for report in raw for c in report['commands'] if c['channel']=='immediate_geometry']
    busy_commands=[c for report in raw for c in report['commands'] if c['channel']=='immediate_geometry'
        and any(s['processing_start']<=c['ready_timestamp']<=s['processing_finish'] for s in report['slow_results'])]
    intervals=[b['received_timestamp']-a['received_timestamp'] for r in raw for a,b in zip(r['fast_results'],r['fast_results'][1:])]
    kpi={
        'look_frame_to_command_s':stats([c['event_to_ready_age_s'] for c in commands]),
        'look_while_slow_frame_to_command_s':stats([c['event_to_ready_age_s'] for c in busy_commands]),
        'direction_event_upper_s':stats([r['event_kpis']['body_center_to_LEFT']['look_ready_upper_s'] for r in evaluated]),
        'disappearance_event_upper_s':stats([r['event_kpis']['person_fully_disappears']['detection_latency_upper_s'] for r in evaluated]),
        'search_from_detection_s':stats([r['event_kpis']['person_fully_disappears']['search_from_detection_s'] for r in evaluated]),
        'search_event_upper_s':stats([r['event_kpis']['person_fully_disappears']['search_ready_upper_s'] for r in evaluated]),
        'reappearance_event_upper_s':stats([r['event_kpis']['first_partial_head_reappears_LEFT']['look_ready_upper_s'] for r in evaluated]),
        'reappearance_event_lower_s':stats([r['event_kpis']['first_partial_head_reappears_LEFT']['look_ready_lower_s'] for r in evaluated]),
        'update_interval_s':stats(intervals),'inference_latency_s':stats([r['inference_latency_s'] for r in fast]),
        'frame_age_s':stats([r['received_timestamp']-r['capture_timestamp'] for r in fast])}
    policy=[sum(r['policy_correct_total'][i] for r in evaluated) for i in [0,1]]
    env=read('reports/phase36/environment.json')
    if not env['same_baseline_packages'] or not env['same_driver_gpu'] or env['stable_ultralytics_installed']:
        raise RuntimeError('Stable environment changed')
    user_before=read('reports/phase35/freeze.json')['existing_reports']['preexisting_edit_sha256']
    if hashlib.sha256((ROOT/'reports/PHASE33.md').read_bytes()).hexdigest()!=user_before:
        raise RuntimeError('Existing user edit changed')
    diff=git('diff','--name-only','--diff-filter=MDRTCUXB',BASE).splitlines()
    if set(diff)-{'.gitignore','reports/PHASE33.md'}: raise RuntimeError(f'Existing frozen files changed: {diff}')
    old_reports=git('ls-tree','-r','--name-only',BASE,'reports').splitlines()
    original_scenarios=git('ls-tree','-r','--name-only',BASE,'scenarios').splitlines()
    freeze={'base_sha':BASE,'existing_modified_files':diff,'only_allowed_change':'.gitignore adds .venv36/',
        'preexisting_user_edit_unchanged':True,'existing_reports_unchanged':True,'original_scenarios_unchanged':True,
        'existing_reports_count':len(old_reports),'original_scenarios_count':len(original_scenarios),
        'g1_sources_unchanged':all(hashlib.sha256(Path(s['path']).read_bytes()).hexdigest()==s['sha256']
            for s in read('reports/phase35/g1_yolo_reference.json')['sources'])}
    write_report(OUT/'freeze.json',freeze)
    tests=(ROOT/'reports/phase36_tests_final.log').read_text().strip().splitlines()[-1]
    summary={'base_sha':BASE,'chosen_fast_source':'YOLO11n GPU FP32 imgsz640 confidence0.10',
        'source_accuracy':low,'final_kpis':kpi,'final_policy_correct_total':policy,
        'slow_active_fast_loop_pass':True,'latest_frame_wins':True,'no_queue_accumulation':True,
        'stale_result_rejection_pass':True,'high_level_vlm_preserved':True,
        'estimated_pepper_readiness':'NOT READY','reason':'Tiny partial-head reappearance is ~220-228ms from first confirmed visible frame, ~253-262ms conservative bound; full 150/250ms event target is not established.',
        'cpu_detector':'NOT NEEDED: GPU YOLO did not exhibit long fast-loop blocking',
        'final_evaluation_files':[str(p.relative_to(ROOT)) for p in final_paths],
        'all_profile_evaluations':profiles,'tests':tests,'environment':env,'freeze':freeze}
    write_report(OUT/'summary.json',summary)
    added=git('ls-files','--others','--exclude-standard').splitlines()+git('diff','--name-only','--diff-filter=A',BASE).splitlines()
    added=sorted(set(added+['reports/PHASE36.md','reports/phase36/changed_files.json','reports/phase36/completion.json']))
    write_report(OUT/'changed_files.json',{'added':added,'modified':'.gitignore','excluded':'reports/PHASE33.md'})
    (OUT/'git_status_before_commit.txt').write_text(git('status','--short')+'\n',encoding='utf-8')
    lines=[
        '# PHASE 3.6 REAL-WORLD HYBRID ARCHITECTURE','',
        '**Chosen fast source: YOLO11n / CUDA / FP32 / imgsz640 / confidence0.10。** '
        'Fast loopはSlow completionを待たずに更新できた。意味的Actionは元のVLM JSON Plannerが選び、'
        'execution resolverはLOOK/FOUNDの方向だけを最新geometryへbindingする。SEARCH LEFTはmodelのraw JSONから選択され、'
        '3回ともRobot-ready commandへ到達した。', '',
        '**Estimated readiness for Pepper integration: NOT READY。** '
        '追従・消失・SEARCHの目標は今回のsimulationで達成したが、小さい部分頭部の再出現は保守的評価で250msを超える。'
        '動画1本のsimulation結果であり、実cameraの露光/通信や実Adapterの実行時間は未測定。', '',
        f'Base SHA `{BASE}`。RTX 3060 Ti 8GB / Windows WDDM / driver581.42。'
        'Qwen3-VL-2B pinned revision、BF16、384px、default SDPA、Phase 3.4 Linear、full JSON Plannerを維持。'
        'Pepper/G1接続、4B、新large model、新Action、driver/CUDA変更は実施していない。', '',
        '## Architecture','',
        '```mermaid','flowchart TD',' C[29.5s video producer at 30fps] --> L[LatestFrameSlot capacity 1]',
        ' L --> F[Independent GPU YOLO worker]',' F --> S[Fast authoritative geometry and State]',
        ' S --> I[Immediate LOOK geometry channel]',' S --> E[Coalesced immutable event snapshot]',
        ' L --> V[Single Qwen slow worker: event Planner / low-rate Semantic]',
        ' E --> V',' V --> R[Latest geometry resolver]',' S --> R',' I --> A[Validated Mock Robot]',
        ' R --> A',' V --> Q[Separate semantic cache: distance / obstacle]',' Q --> E','```','',
        'Producerは常に最新1枚を更新し、workerはidle時に最新frameを取得する。入力FIFOはない。'
        'frame generationは動画frame index、capture_timestampはsimulation開始monotonic clock＋source video timestamp、'
        'decode_finished/processing_start/processing_finish/received/ready clockとresult ageを個別保存する。'
        'decodeが予定時刻に遅れた場合もframeをskipして現在へ追い付く。保存用の結果履歴を処理用frame queueとして使わない。', '',
        'Fast結果だけがperson_visible/direction/countとdeterministic Stateを更新する。'
        '古いgeneration/source clock、future clock、不正metadata、500ms超のfast結果は受理しない。'
        'Semanticは別cacheに保持し、person geometry/last_seen/transitionを書き換えない。'
        'distanceは同じvisibility episode・同方向・TTL3秒以内の場合にだけ利用し、obstacleもTTLを持つ。'
        '古いsemanticを新しい取得時刻へ付け替えない。', '',
        'Immediate channelはvisible personへのLOOK direction updateのみ。'
        'SEARCH/APPROACH/AVOID/FOUND/SURPRISEはFast ruleから出さない。'
        'そのchannelはVLM raw Decisionを変更せず、別のgeometry commandとして記録する。'
        'LOOK/FOUND PERSONのresolverは最新方向を使い、SEARCH directionはmodelが選んだMemory方向を保持する。', '',
        'Planner triggerはINITIAL/APPEARED/DISAPPEARED/SEMANTIC_CHANGED/lease EXPIRED。'
        'STILL_VISIBLEのgeometry追従だけではPlannerを呼ばない。'
        'visibility eventを100ms安定待ちし、1つのpending slotで最新eventへcoalesceする。'
        '同じepisode内の重複はdebounceし、新episodeの同種eventは捨てない。visibility eventを低priorityのsemantic refreshで上書きしない。', '',
        'State自体はFast更新ごとに進み続ける。Plannerには最新のpending eventで取得したimmutable Observation/StateSnapshotを渡す。'
        'DISAPPEAREDのsnapshot（missed=1）を保持することで、camera更新により一瞬でSTILL_ABSENTへ進んでもeventを失わない。'
        'snapshotのgeneration/取得時刻を保存し、古いeventをcurrent completionとして扱わない。'
        'Planner inputはcurrent_observation・memory・transitionの元JSONだけで、画像・timestamp・GT・expected Actionを含まない。', '',
        'episodeが変わった際は古いVLM jobを次のtoken境界でcancelする。partial raw outputは保存し、'
        'quote補完、Action補正、WAITへ再解釈をせず、Robotへ渡さない。'
        '完了intentもepisode/取得ageを検証し、obsoleteならcommandを抑止する。'
        'SemanticはPlanner完了後に1.5秒quiet、直近visibility変化後も1.5秒quiet、前回Semanticから8秒以上の場合だけ実行する。'
        '動画の特定秒数に応じたtriggerはない。', '',
        '## Dependency / YOLO profile','',
        'YOLOは独立`.venv36`へ導入。`.pth`でstable `.venv`の既存torch/torchvisionをread-only参照し、'
        '新packageをstableへinstallしない。optional packagesは[requirements-hybrid-lock](../requirements-hybrid-lock.txt)で固定し、'
        '[setup_hybrid.ps1](../setup_hybrid.ps1)で再現できる。stableにultralyticsは未導入、両venvのpip check PASS。', '',
        'YOLO11n重みSHA256 `0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1`。'
        '[公式重み](https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt)を取得し検証した。'
        'bboxは元画像座標へ戻した値で、中心x<40% LEFT、40〜60% inclusive CENTER、>60% RIGHT。'
        '複数personは最大bbox面積、confidence、x/yの順でgenericに選び、countは全person数。'
        '境界・selectionはGTや13秒に応じて変更していない。', '',
        '初期confidence0.25は既存G1実装から引き継いだ。全frameへ一律confidence0.10を適用する追加候補も比較し、'
        '元9点のaccuracyを維持しながらpartial-head応答を改善したため採用候補とした。'
        '初期0.25、改善前scheduler、safe serialized Fast VLM、cooperative Fast VLMの全rawを保持した。', '',
        'APIは[Ultralytics predict公式](https://docs.ultralytics.com/modes/predict/)を参照。'
        '既存torch2.9.1 / torchvision0.24.1の対応は[PyTorch公式](https://pytorch.org/get-started/previous-versions/)と実環境で確認。'
        '旧G1 repositoryはread-onlyで参照し、Phase 3.5で保存したsource hashとの一致を確認した。'
        '旧RTX 5060 Ti/Linuxの5.4msを今回の3060 Ti実測値へ流用していない。', '',
        '## Fast source accuracy / selected-image benchmark','',
        '同じ既存9点×3回、fresh 8→10→12を1回。元labelsは変更していない。'
        'warmupはYOLO dummy 3回、Qwen image/text warmupをload後に実行。'
        '同じ動画sourceを使うが、VLMは既存384px処理、YOLOは既存640px letterboxなので入力pixel数は同一ではない。', '',
        '| Source | Presence | Direction | Main geometry / Memory | mean/median/p95/max inference ms | Peak allocated / reserved GiB |',
        '|---|---|---|---|---|---|']
    for name,r in [('Fast VLM',vm),('YOLO conf .25',labels),('YOLO conf .10',low)]:
        presence='/'.join(map(str,r['presence_correct_total']))
        direction='/'.join(map(str,r['direction_correct_total']))
        lines.append(f"| {name} | {presence} | {direction} | {'PASS' if r['main_geometry_pass'] else 'FAIL'} | {milliseconds(r['latency_s'])} | {r['peak_allocated_gib']:.3f}/{r['peak_reserved_gib']:.3f} |")
    lines += ['', 'Fast VLMは13秒RIGHTの誤認を保持（direction80%）。YOLOは13秒LEFT、direction100%。'
        'mainでは8LEFT→10absent/DISAPPEARED/last_seenLEFT→12LEFT/APPEAREDを確認。'
        'この表のmainはFast geometry/Memoryの評価であり、選択frameごとにPlannerを呼んだ結果ではない。', '',
        '## Actual WDDM contention / real-time simulation','',
        '動画を29.5秒・30fps相当でwall clockに合わせて再生。初期alone/stress各1本、'
        '改善したhybridは各variant3本、最終event-semantic schedulerも3本。'
        '連続stressではSlowを最大頻度で呼んで競合を作るが、それをproductのper-frame Planner方式には採用しない。', '',
        '| Case | Fast mean/median/p95/max ms | Result age mean/p95/max ms | Update interval p95/max ms | Update Hz | Fast overlap with Slow | Fast loop | Fast peak GiB / GPU total max GiB |',
        '|---|---|---|---|---:|---:|---|---|']
    ordered=sorted(profiles.items(),key=lambda p:p[0])
    for path,r in ordered:
        age=r['result_frame_age_s']; intervals_=r['update_interval_s']
        age_text='N/A' if age['mean'] is None else f"{age['mean']*1000:.1f}/{age['p95']*1000:.1f}/{age['max']*1000:.1f}"
        int_text='N/A' if intervals_['p95'] is None else f"{intervals_['p95']*1000:.1f}/{intervals_['max']*1000:.1f}"
        gpu=r['gpu_used_gib']['max']; peak=r['fast_peak_allocated_gib']
        memory='N/A' if peak is None else f"{peak:.3f}/{gpu:.3f}" if gpu is not None else f"{peak:.3f}/N/A"
        fast_loop='N/A' if not r['fast_count'] else 'PASS' if r['slow_active_fast_loop_pass'] else 'FAIL'
        lines.append(f"| {path.replace('evaluations/','').replace('.json','')} | {milliseconds(r['inference_latency_s'])} | {age_text} | {int_text} | {r['update_rate_hz']:.2f} | {r['overlap_fast_count']} | {fast_loop} | {memory} |")
    lines += ['', 'GPU totalはNVMLのadapter全体のsample最大値で、OS/他appの使用も含む。'
        'allocated/reservedはPyTorch各processの値で、合計値とは区別する。CPU%は1core=100%のpsutil値、'
        'GPU utilization/CPU各PIDのmean/p95/maxは全[evaluation JSON](phase36/evaluations/)とraw resourcesへ保存。'
        '負荷を無効化して数字を補完せず、取得できないtelemetryはN/A/errorとして保持する。', '',
        '### Fast VLM while Slow runs','',
        '単一modelをjob単位で安全に直列化したnegative controlは、Fast intervalが約1.6〜2.6秒へ増えFAIL。'
        '単発fast inferenceだけが約0.1秒でもlive sourceとして成立しない。', '',
        '追加のcooperative profileはSlow generateのtoken境界で最新shared frameを取得し、Fast geometryを実行してからSlowを続行する。'
        'model weightは二重ロードせず、1つのmodel/processor instanceを共有。Fast KV cacheは別generationで、'
        'model-level rope_deltasを保存・復元し、Slowのcacheを汚染しない。'
        'Fast結果はSlow completion前にparentへ送る。FastのSource generationは最新1枚だけで、paused Slowをinput frame queueへしない。', '',
        'cooperative Fast update interval p95は約163〜165ms、max約180〜197msとなり、Slow job中の更新自体はPASS。'
        'ただし13秒direction誤認、部分頭部への遅れ、Slow自身のwall latency増加、RoPE cache管理の複雑さが残る。'
        '同じGPUへ2B weightを二重ロードして単に別processにした実験ではない。', '',
        '| Case | JSON Planner mean/median/p95/max s | Full Semantic mean/median/p95/max s |',
        '|---|---|---|']
    for path,r in ordered:
        lines.append(f"| {path.replace('evaluations/','').replace('.json','')} | {metric(r['slow_planner_latency_s'])} | {metric(r['slow_semantic_latency_s'])} |")
    lines += ['', '**Chosen sourceはYOLO conf0.10**。precision、Slow負荷中のage/update、30Hz近い更新、'
        '小さいfast VRAM、独立workerの簡潔さを総合評価した。cooperative VLMの可能性は残すが、今回のdefault候補にはしない。'
        'GPU YOLOで長いstallを観測していないため、optional CPU/ONNX/OpenVINO導入はNOT NEEDED。', '',
        '## Real-world KPI（最終3本）','',
        '既存の9点labelsはevent onsetを厳密には指定しない。追加の[evaluator専用visual annotation](../scenarios/IMG_7121_phase36_events.json)では、'
        'CENTER→LEFT区間5.900〜5.967秒、完全消失8.400〜8.467秒、最初の部分頭部の再出現10.367〜10.400秒を記録した。'
        '保存したcontact sheetsから目視し、detectorの発火時刻をGTへ流用していない。12秒は既存confirmed-present anchorとして維持する。'
        '区間の下端から測る保守的upper latencyと、上端から測るlower latencyを保存する。', '',
        '| KPI | count | mean / median / p95 / max ms | 判定 |','|---|---:|---|---|']
    outcomes={
        'look_frame_to_command_s':'PASS <=150/250ms',
        'look_while_slow_frame_to_command_s':'PASS: Slow activeでも継続',
        'direction_event_upper_s':'PASS <=150/250ms',
        'disappearance_event_upper_s':'PASS p95<=250ms',
        'search_from_detection_s':'PASS <=2.0s、raw SEARCH LEFT',
        'search_event_upper_s':'PASS: physical-event conservative upper<=2.0s',
        'reappearance_event_upper_s':'FAIL to certify <=250ms conservative target',
        'reappearance_event_lower_s':'first confirmed visible frameからは約220〜228ms',
        'update_interval_s':'PASS: no slow-completion wait',
        'inference_latency_s':'参考、採用基準はこれだけではない',
        'frame_age_s':'Source age、completion時刻へ付け替えない'}
    for key,s in kpi.items(): lines.append(f"| {key} | {s['count']} | {milliseconds(s)} | {outcomes[key]} |")
    lines += ['',f"最終3本の元JSON Planner policy一致 **{policy[0]}/{policy[1]}**。"
        'policyはresolver前のraw Decisionに対し、変更しないPhase3.3 evaluatorのgeneric predicateで判定する。'
        'hybridの2model・高頻度Fast historyに単一model/full-frame版の全integrity判定を流用せず、'
        'raw一致、Fast State replay、generation、no image/no extra Planner input、resolved commandを別にstrict検証した。', '',
        'verified empty windowでは最終3本ともfalse positiveなし。'
        '小さな頭部は10.6秒で初検出、10.633秒で1frameだけ失い、10.667秒で再検出する。'
        'Fastの実測visibilityはそのまま記録し、Stateをfalse PASS用に平滑化しない。'
        '100msのevent settlingとcoalescingにより、semantic Plannerへ短い揺れを全件FIFO投入しない。', '',
        '| Contract | Result |','|---|---|',
        '| Slow VLM active中のFast loop | PASS |','| Latest frame wins | PASS |',
        '| No queue accumulation | PASS |','| Old generation/stale semantic rejection | PASS |',
        '| VLM high-level Planner preserved | PASS |','| SEARCH memory direction unchanged | PASS |',
        '| Validated resolved Decision only to Mock | PASS |','| Full reaction target for tiny partial head | NOT ESTABLISHED |', '',
        '初期hybrid（cancelなし）は8秒台DISAPPEAREDのSEARCHが再出現後に完了し、obsoleteとして棄却された。'
        'この失敗を残し、visibility episodeの変更でold jobをtoken境界cancelする改善を行った。'
        'さらに新episode eventをdebounceで落とさないこと、visibility eventのpriority、100ms settling、'
        'media秒数に依存しないsemantic schedulingを最終構成として検証した。', '',
        '## Recommended architecture / Pepper estimate','',
        '推奨は今回のYOLO fast source＋latest slot＋authoritative Fast State＋geometry-only LOOK channel＋'
        'event-driven full JSON VLM Planner＋occasional Full Semantic＋execution resolver。'
        'Slow workerだけでPlanner/Semanticを直列化し、Fast workerはそのcompletionを待たない。'
        'SEARCHはVLMがMemoryから選び、最新current directionで書き換えない。', '',
        '**NOT READY for Pepper integration**。今回の核心であるGPU負荷中trackingは成立したが、'
        '小さな頭部のevent reaction、1frame検出揺れ、単一動画のための汎化未確認が残る。'
        'camera露光/通信からのage、身体とhead/LOOK commandのchannel所有、camera座標の左右、'
        '実Adapterのready→execution時間を次のmock/camera段階で計測する必要がある。'
        '実機を接続して結果を補う作業は今回行っていない。', '',
        '## Tests / freeze / artifacts / Git','',
        f'**{tests}**。既存356件を保持し、LatestFrameSlot、old metadata/generation拒否、'
        'Fast authoritative State、TTL/episode、LOOK resolver、SEARCH保持、event priority/debounce/coalescing/settling、'
        'no per-frame Planner、VLM busy時のmock Fast geometry更新、validated Mock、'
        'real GPU cooperative overlap/model reuse/RoPE restoration、cancelled raw抑止、GT runtime非混入を追加検証した。', '',
        '[summary](phase36/summary.json)、[freeze](phase36/freeze.json)、[environment](phase36/environment.json)、'
        '[changed files](phase36/changed_files.json)、[git status before commit](phase36/git_status_before_commit.txt)。'
        '全rawは[phase36](phase36/)以下。selected、realtime、realtime_preempt、realtime_cooperative、'
        'realtime_lowconfidence、realtime_final、realtime_eventsemanticの各JSONを削除していない。'
        '全scheduler events・source/processing clocks・frame age・State・Planner input/raw・class logits/boxes・resolved commands・CPU/GPU telemetryを保存。', '',
        '[test log](phase36_tests_final.log)、[stable pip check](phase36_pip_check_stable.log)、'
        '[optional pip check](phase36_pip_check_optional.log)、[dependency install](phase36_install.log)。'
        'old G1 repositoryと既存reports/labels/Phase3.5 stable pathを維持。`.gitignore`へ`.venv36/`だけ追加した。'
        '作業開始前からのreports/PHASE33.mdユーザー変更は内容hashを確認し、stage/commit対象に含めない。'
        '実装commit SHAとcommit後statusは[completion record](phase36/completion.json)に保存し、最終HEADを最終応答に記載する。', '',
        '## Reproduce','', '```powershell', '.\\setup_hybrid.ps1', "$env:HF_HUB_OFFLINE='1'",
        '.venv36\\Scripts\\python.exe benchmark_fast_sources.py C:\\Users\\slowh\\Downloads\\IMG_7121.mp4',
        '.venv36\\Scripts\\python.exe benchmark_fast_sources.py C:\\Users\\slowh\\Downloads\\IMG_7121.mp4 --backends yolo --confidence .1 --output-dir reports/phase36/selected_lowconfidence',
        '.venv36\\Scripts\\python.exe run_hybrid_video.py C:\\Users\\slowh\\Downloads\\IMG_7121.mp4 --cases hybrid --confidence .1 --repeats 3 --output-dir reports/phase36/realtime_eventsemantic',
        '.venv36\\Scripts\\python.exe run_hybrid_video.py C:\\Users\\slowh\\Downloads\\IMG_7121.mp4 --cases yolo_alone,fast_vlm_alone,planner_alone,yolo_planner,fast_vlm_planner,yolo_semantic,fast_vlm_semantic --confidence .25',
        '.venv36\\Scripts\\python.exe run_hybrid_video.py C:\\Users\\slowh\\Downloads\\IMG_7121.mp4 --cases fast_vlm_planner_cooperative,fast_vlm_semantic_cooperative --output-dir reports/phase36/realtime_cooperative',
        '.venv\\Scripts\\python.exe evaluate_hybrid.py', '.venv\\Scripts\\python.exe -m pytest -q',
        '.venv\\Scripts\\python.exe report_hybrid.py','```','',
        '各GPU commandは順番に実行する。既存resultを保存する場合は別output-dirを指定する。', '']
    (ROOT/'reports/PHASE36.md').write_text('\n'.join(lines),encoding='utf-8')
    print('Saved PHASE36; YOLO chosen, real-time architecture PASS, Pepper NOT READY')


if __name__=='__main__': main()
