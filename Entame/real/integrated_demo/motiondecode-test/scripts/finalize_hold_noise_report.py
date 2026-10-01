#!/usr/bin/env python3
"""Join the previous-pair supplement and write the Japanese measurement report."""
import argparse
import csv
import json
from pathlib import Path
import numpy as np
from common import write_json


def main():
    parser=argparse.ArgumentParser(description=__doc__);parser.add_argument('directory',type=Path)
    folder=parser.parse_args().directory
    r=json.loads((folder/'baseline_noise_stats.json').read_text())
    t=json.loads((folder/'monitor_timing.json').read_text())
    s=json.loads((folder/'fresh_q0.json').read_text())
    extra=json.loads((folder/'previous_hand_noise_stats.json').read_text())
    if not r['measurement_valid']:raise ValueError('Do not finalize an incomplete observation')
    r['previous_failing_pairs']=[x for x in r['pair_statistics'] if x['previous_failing_pair']]+[extra]
    r['additional_tracked_pairs']=[extra]
    r['identity_binding']=json.loads((folder/'receive_only_identity_binding.json').read_text())
    # Append the explicitly tracked non-baseline previous hand pair to the
    # clearance table, preserving sample-id correspondence and all 9 baselines.
    target=folder/'clearance_samples.csv';temporary=folder/'clearance_samples.joined.csv'
    with target.open() as f,(folder/'previous_hand_samples.csv').open() as sf,temporary.open('w',newline='') as out:
        a=csv.reader(f);b=csv.reader(sf);ah=next(a);bh=next(b)
        if bh[-1] in ah:raise ValueError('Supplement already joined')
        w=csv.writer(out);w.writerow(ah+[bh[-1]])
        count=0
        for row,additional in zip(a,b,strict=True):
            if row[0]!=additional[0]:raise ValueError('Mismatched sample IDs')
            w.writerow(row+[additional[-1]]);count+=1
    assert count==r['observation']['lowstate_samples']
    temporary.replace(target)
    write_json(folder/'baseline_noise_stats.json',r)
    required=('mean','median','min','max','p95','p99')
    def table_stats(keys):
        rows=['| 指標（ms） | mean | median | min | max | p95 | p99 |','|---|---:|---:|---:|---:|---:|---:|']
        for label,key in keys:
            rows.append('| '+label+' | '+' | '.join(f'{t[key][v]*1000:.4f}' for v in required)+' |')
        return rows
    worst=next(x for x in r['pair_statistics'] if x['pair']==r['worst_pair'])
    lines=['# G1 HOLD baseline noise characterization','', '## Safety','',
        '**Commands sent to G1 (application commands): NONE**。Publisher / DataWriter生成を実行時に禁止したreceive-only測定です。weight・LowCmd・joint/motion command・ownership取得・service操作は行っていません。閾値も変更していません。',
        '初回はLowStateが受信できずBLOCKされました。LAN疎通診断のICMP echo 3回には応答があり、その後DDS受信を確認しました。ICMPとDDS discoveryはロボットのapplication commandに含めていません。',
        'G1内部serviceは昨日のPID 2934から2884へ変わっていました。IP・host・process・関連topic構成の一致と15秒間のcommand sample 0を確認し、記録したGUID/PIDだけをこのreceive-onlyプロセスに限定して照合しました。実機runnerのownership設定は変更していません。DDSの自己申告metadataによる識別という限界は従来と同じです。',
        '測定中：external arm_sdk writer = 0、armsdk writer = 0、Arm Action = IDLE、LowState正常、静止判定PASS。',
        '', '## Observation','',f"fresh q0: {s['timestamp']}",
        f"右腕 q0 [rad]: `{s['arm_q'][7:]}`",f"duration: **{t['duration_s']:.3f} s** / LowState samples: **{t['lowstate_samples']:,}** / LowState callback reception rate: **{t['lowstate_reception_rate_hz']:.3f} Hz**",
        f"live FK / collision evaluations: **{t['live_monitor_samples']:,}** / evaluation rate: **{t['live_collision_evaluation_rate_hz']:.3f} Hz**。",
        'LowState全受信q/dq/deltaを保存しました。live監視は20 msを目標に実行し、さらに取得後の全sampleを同一の29関節FK・全pair計算で再評価しました。前回停止した左手pairが今回はbaseline外だったため、同じmesh距離計算で補足し、clearance_samples.csvには9 baseline pairとその1 pairを保存しています。',
        'FKはruntimeの処理を共通関数へ移したもので、旧処理と結果が完全一致するテストを通しています。新しい近似形状・距離式は使っていません。',
        'LowState rateはPython callbackで実際に取得したunique-tick sampleのレートです。未受信sampleがないことやwire上の配信レートは保証しません。receipt時刻、tick、live評価時age、replay時ageを区別して保存しています。','']
    lines+=table_stats([('LowState受信間隔','lowstate_reception_interval_s'),('live monitor間隔','monitor_sample_interval_s'),
        ('FK処理時間','live_FK_processing_s'),('collision処理時間','live_collision_processing_s'),
        ('ownership照合時間','live_ownership_processing_s'),('live全処理時間','live_total_processing_s'),
        ('FK開始時LowState age','live_lowstate_age_at_FK_s')])
    lines+=['',f"live処理時間が20 msを超えた割合: **{100*t['live_processing_over_20ms_fraction']:.2f}%**。主因はownership照合です。受信停止と、計算が受信より遅いことは別の問題です。",
        '', '## Joint noise','',f"max q deviation: **{r['max_joint_deviation_rad']:.9f} rad**。p99 q deviation（sampleごとの29関節最大絶対偏差）: **{r['p99_of_per_sample_max_joint_deviation_rad']:.9f} rad**。",'',
        '| Joint | q peak-to-peak rad | std rad | p99 absolute q−q0 rad |','|---|---:|---:|---:|']
    for j in r['joint_statistics']:
        lines.append(f"| {j['joint']} | {j['peak_to_peak_rad']:.9f} | {j['std_rad']:.9f} | {j['p99_deviation_from_q0_rad']:.9f} |")
    lines+=['','## Clearance noise','',f"Worst baseline pair: `{r['worst_pair']}`。最大自然baseline悪化量: **{r['maximum_natural_baseline_worsening_mm']:.6f} mm**。",
        f"全baseline pairのsampleごとの最大絶対偏差：p95 **{r['p95_per_sample_worst_absolute_clearance_deviation_mm']:.6f} mm**、p99 **{r['p99_per_sample_worst_absolute_clearance_deviation_mm']:.6f} mm**。",
        'Current tolerance: **0.001 mm**。以下はmm。悪化率は `initial − clearance > 0.001 mm` の割合です。', '',
        '| Pair | initial | mean | median | min | max | p-p | std | MAD | 最大悪化 | p95 abs | p99 abs | 悪化率 |',
        '|---|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|---:|']
    for x in r['pair_statistics']+[extra]:
        label=x['pair']+('（前回停止）' if x['previous_failing_pair'] else '')
        if x is extra:label+='（今回baseline外）'
        numbers=[x[k] for k in ['initial','mean','median','minimum','maximum','peak_to_peak','std','MAD',
                 'maximum_baseline_worsening','p95_absolute_deviation_from_q0','p99_absolute_deviation_from_q0']]
        lines.append('| '+label+' | '+' | '.join(f'{v:.6f}' for v in numbers)+f" | {x['fraction_worsening_over_current_tolerance']*100:.2f}% |")
    lines+=['','## Classification','',f"**{r['classification']}**。自然な静止状態だけで0.001 mmを頻繁に超えるため、現在の値はruntime monitor用として過敏です。記述上の「頻繁」は1 pair以上で全sampleの5%以上と定義してあり、安全閾値の変更ではありません。",'',
        '## Interpretation','',f"Was the previous STAGE 1 stop likely a monitoring false positive? **{r['previous_stop_likely_monitoring_false_positive']}**。",
        '前回はpositive-weight HOLD前の停止で、positive weightは0通でした。今回のreceive-onlyでも同じpairで前回相当以上の変動が確認できています。ただし姿勢が前回と異なり、関節の実際の微小な動きとencoderの量子化・測定ノイズは独立計測なしには分離できません。前回原因の断定ではありません。','']
    for x in r['previous_failing_pairs']:
        top=x['joint_correlations'][0]
        lines+=[f"- `{x['pair']}`：最大sample間変化 {x['sample_to_sample_abs_step_mm']['max']:.6f} mm、受信間隔と変化幅の相関 {x['step_vs_reception_interval_correlation']:.6f}、initial−median {x['initial_minus_median_mm']:.6f} mm。",
                f"  最大相関jointは {top['joint']}（r={top['pearson_r']:.6f}）。同じFKの局所感度から予測したclearance変化の残差RMSEは {x['same_FK_local_sensitivity_explanation']['rmse_mm']:.9f} mm。"]
    lines+=['','関節qの変化だけでclearance変動をほぼ説明でき、sample間の受信周期が飛んだ時だけ出る現象ではありません。初期q0の単発sampleがmedianから数µmずれることも、0.001 mm判定を超える頻度に影響します。',
        f"新規penetration pair（全受信sampleを再評価）: {r['new_penetration_pairs']}。新規15 mm未満pair: {r['new_clearance_deficit_pairs']}。",
        '今回のq0は既存の右手首／右股関節pitchが約−2.980 mm、右手／右股関節pitchが約−4.270 mmです。これは変動幅とは別の絶対値であり、ノイズ帯を増やして安全扱いするものではありません。',
        '', '## Recommended baseline-worsening tolerance design','',
        '**Do NOT modify it yet.** 以下は測定分布から算出した設計比較値で、適用していません。',
        '1. **Hard safety**：新規penetration、新規collision pair、joint limit違反、既存の絶対clearance基準を大きく外れる変化は、noise帯の判定と分離する。noise toleranceで新規penetrationを許可しない。',
        '2. **Baseline worsening**：姿勢ごと・pairごとの安静分布を取り、medianとfresh q0の距離差を記録する。候補式 `T99 = |d(q0)−median| + p99(|d−median|)` を、MAD方式と観測最大値と比較する。基準はmotion中に更新して追従させない。',
        '3. **Hysteresis**：停止帯を超えた後、より狭い `|d(q0)−median| + p95(|d−median|)` 帯へ戻るまで解除しない方式を検討する。持続時間や連続回数は今回勝手に決めず、時系列の連続超過と独立再測定で検証する。',
        'p99方式は校正分布にも尾部が残ります。MAD方式は外れ値に強い一方、非正規な尾部を保証しません。観測最大値やpeak-to-peakも有限120秒の範囲にすぎません。以下の比較値をそのままreal runnerのstop閾値へ代入してはいけません。','',
        '| Baseline pair | bias+p99 mm | bias+3×1.4826MAD mm | 観測最大悪化 mm | 観測p-p mm | bias+p95解除帯 mm |',
        '|---|---:|---:|---:|---:|---:|']
    for x in r['pair_statistics']:
        opts=x['design_options_NOT_APPLIED_mm'];vals=list(opts.values());vals[-1]+=abs(x['initial_minus_median_mm'])
        lines.append('| '+x['pair']+' | '+' | '.join(f'{v:.6f}' for v in vals)+' |')
    lines+=['','## HOLD RETEST READY','', '**NO**。',
        '- 現行0.001 mmのままでは自然変動でも停止する。pair別判定設計のレビュー・独立データ検証・適用は未実施。',
        '- 現行監視は20 ms周期を満たしていない。ownership照合の処理を制御周期から分離する設計と、実際にFKへ使うsampleのage・処理期限の検証が必要。',
        '- 今回q0の既存penetrationが前回と異なり、positive-weight HOLDそのものは未検証。',
        '次回案：fresh q0 → receive-only baseline取得 → 全controlled jointsをfresh実測角に初期化 → 現場で承認した低いweightから段階的ramp → q0 HOLD → release。今回はHOLDを送らず、25% reactionにも進めません。',
        '', '## Commands sent','', 'arm_sdk: **NONE**\n\nMotion: **NONE**\n\nNavigation: **NONE**\n\nSLAM: **NONE**',
        '', '## Files','',
        '- fresh_q0.json / lowstate_samples.csv / clearance_samples.csv',
        '- baseline_noise_stats.json / monitor_timing.json / preflight.json',
        '- live_monitor_samples.json / ownership_samples.json',
        '- previous_hand_noise_stats.json / previous_hand_samples.csv',
        '- receive_only_identity_binding.json / tests.txt']
    (folder/'report.md').write_text('\n'.join(lines)+'\n')
    print('Report finalized:',folder/'report.md')


if __name__=='__main__':main()
