"""腕の指令の積み上げ・送る目標・立つ HOMIE の指令の扱い（G1_Hackason の直し B・2・8、2026-10-26）。

学習（VIRAL6、torch）と実機（Door/real、numpy・PC2 の Python 3.8）に、1 字も違わない同じファイルを置く。
  学習: gr00t/rl/envs/base_task/target_guard.py（パッチ 0069）
  実機: G1_Hackason/Door/real/target_guard.py（こちらが元。試験 tests/test_target_guard.py が 2 つが同じことを確かめる）
numpy の配列と torch の tensor のどちらを渡しても動くよう、両方にある書き方（.clip・*・reshape）だけを使う。
torch は import しない（PC2 には無い）。

B: 腕の目標は「前の積み上げ D に増分を足す」だけなので、腕が胴や扉に止められると D がたまり続け、
   その後ずっと力の上限で押し付ける（10-23 の腕だけの空振りで 28.8 N·m）。積み上げ D そのものを
   「今の角度 ±delta」と「可動域の端から margin 内側」に収まる値へ書き戻す（送る目標だけ切っても D はたまる）。
2: 送る目標を可動域の内側に切り、1 周期の変化を max_step までにする（実機の door_run_real.py と同じ）。
8: 立つ HOMIE には歩く指令（各フレームの先頭 3 つ）を 0 にして渡す（学習ではそうしていた）。
"""


def guard_arm_accumulator(D_arm, q_arm, q_default_arm, q_lower_arm, q_upper_arm, action_scale, delta, margin):
    """腕の積み上げ D を、目標の角度（action_scale·D＋既定の角度）が次の範囲に入る値へ切る。

    範囲は [今の角度 − delta, 今の角度 + delta] と [可動域の下端 + margin, 上端 − margin] の重なり。
    重ならない（今の角度が可動域の端の margin の外にある）ときは、可動域の側が勝つ。
    切られない要素は入力の値をそのまま返す（割り算の丸めで D が動かない）。入力は書き換えない。

    D_arm・q_arm: [..., 14]、q_default_arm・q_lower_arm・q_upper_arm: [14]（後ろの次元で揃う形なら何でも）
    """
    lim_lo = q_lower_arm + margin
    lim_hi = q_upper_arm - margin
    lo = (q_arm - delta).clip(lim_lo, None).clip(None, lim_hi)  # min(max(q − δ, 下端), 上端)
    hi = (q_arm + delta).clip(None, lim_hi).clip(lim_lo, None)  # max(min(q + δ, 上端), 下端)
    return D_arm.clip((lo - q_default_arm) / action_scale, (hi - q_default_arm) / action_scale)


def limit_sent_target(target, last_target, q_lower=None, q_upper=None, margin=None, max_step=None):
    """送る目標の角度を切る。margin があれば可動域の端から内側へ、max_step があれば前の目標から ±max_step まで。

    last_target が None（最初の周期）なら変化の上限は掛けない。入力は書き換えない。
    """
    out = target
    if margin is not None:
        out = out.clip(q_lower + margin, q_upper - margin)
    if max_step is not None and last_target is not None:
        out = last_target + (out - last_target).clip(-max_step, max_step)
    return out


def zero_stand_command(homie_obs, n_frames=6, n_cmd=3):
    """HOMIE の観測（履歴 5＋今の 1 の n_frames フレームを後ろの次元につないだもの）の、各フレームの先頭 n_cmd を 0 にした写しを返す"""
    frame = homie_obs.shape[-1] // n_frames
    frames = (homie_obs * 1).reshape(tuple(homie_obs.shape[:-1]) + (n_frames, frame))  # * 1 で写しを作る
    frames[..., :n_cmd] = 0
    return frames.reshape(homie_obs.shape)
