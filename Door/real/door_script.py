"""11-12 台本の計画（docs/plan/2026-11-12-g1-door-scripted-stage123.md・S2・S3・S5）: 段 2（レバーを下げる）と
段 3（扉を押し開ける）を、学習した方策を使わず手で書いた状態機械で進める。numpy だけ。

走らせ役（door_run_staged.StagedDoor）が毎周期（50 Hz）step を呼ぶ。ここは扉の座標の上の「手のひらの行き先（goal）」と
脚への指令だけを決める。goal を関節の角度にするのは呼ぶ側（g1_arm_ik.ArmIK）。機体・逆運動学から切り離してあるので、
偽の手のひらで状態の移りを試験できる。

  小段  2a 待つ → 2b 構える（経由点 → レバーの上）→ 2c 下ろして触れる → 2d 押し下げる
        → 3a 掛け金を外す（下げたまま前へ・扉 3°）→ 3c 押しながら歩く（扉 30°）→ done
        3c は push_mode で 2 通り: "walk"（既定・11-12 夜）＝腕は体に対して固め、前へ歩き続けて体で押す／
        "pulse"（11-12 実機）＝腕で扉の円に沿って押し、手が前へ出すぎたら短く歩く。11-12 の機体では pulse だと
        押した反力で純正が後ろへ 2〜3 歩踏み出して 10〜12 cm 下がった（作業ログ 4 章）
        体が回ったら retreat（手を上げて 2a からやり直し・1 回だけ）。だめな時は failed（呼ぶ側が立ったまま止める）
  座標  door_frame.py と同じ（原点＝開口の中心の床・+x＝押して入る向き・把手が左ならレバーは +y）
  定数  script_params.json（load_params）。回と回の間はこのファイルだけ直す

⚠️ 既定値は 11-12 の機体で扉 10° まで通った script_params.json の値（2a〜3a）。3c の walk はまだ機体で試していない
"""
from __future__ import annotations

import json
import math
from dataclasses import dataclass, fields, replace
from typing import List, Optional, Sequence, Tuple

import numpy as np

import door_frame as dfr
import door_staged as ds
import hand_door_angle as hda

Walk = Tuple[bool, float, float, float]
STAND: Walk = (False, 0.0, 0.0, 0.0)
SUBS = ("2a", "2b", "2c", "2d", "3a", "3c", "done", "retreat", "failed")
STOP_AFTER = ("", "2a", "2b", "2c", "2d", "3a")
PUSH_MODES = ("walk", "pulse")
WALK_HANDS = ("body", "arc")
DT = 0.02
MISS_SHIFT_MAX = 0.10   # 押し直しの横のずらしの上限 [m]（棒の長さの内）
BACK_TOL_M = 0.02       # 押し直しの前に下がる: 2a の終わりの位置からこれより前に居る間は下がる
ROW_LEN = 12


@dataclass(frozen=True)
class ScriptParams:
    # 構えの位置（pelvis・扉の座標）。y はレバーの当てる点からの差（把手が左の扉の向き。右の扉は符号を返す）
    stance_x: float = -0.40
    stance_y_off: float = -0.22    # 手が肩より 8 cm 外に来る（内だと肘が胸の前へ回り込む・11-12 机の上の確かめ）
    stance_yaw_deg: float = 0.0
    # 当てる点を棒に沿って付け根の側へずらす [m]（11-13: 寸法の当てる点は棒の先から 2.5 cm・手は先から 5.5 cm しか内側に無く、
    # 3c で蝶番の側へ 6.6 cm ずれて棒から滑った）。構えの位置・経由点・押し直しのずらしは皆この点から測る
    ref_dy: float = 0.0
    # 半身で押す（11-13 案 A）: 段 2 の始めから、腰 yaw で上体を把手と反対の側へ twist_deg 回す（把手が左なら右へ）。
    # 把手の側の肩が前・中央寄りに出て、手が骨盤のほぼ正面に来る（扉の押し返しが体を回す腕が短くなる）。骨盤は扉にまっすぐのまま
    twist_deg: float = 0.0
    twist_rate: float = 0.6        # その回す速さ [rad/s]
    # 上体を前へ傾ける（11-13 案 B）: lean_from の小段から腰 pitch を前へ lean_deg。手の高さ（約 0.9 m）で押し返されると
    # 体は後ろへ倒される向きになり、純正が後ろへ踏む（半身の 1 回目: 向きは ±5° のまま 14 cm 下がった）→ 重心を先に前へ置く
    lean_deg: float = 0.0
    lean_from: str = "2a"
    lean_rate: float = 0.3         # 傾ける速さ [rad/s]
    stance_tol: Tuple[float, float, float] = (0.04, 0.04, 6.0)  # 前後 m・左右 m・向き °
    # 逆運動学の基準の姿勢（左腕 7 軸）。肩 roll を外へ開いた形（11-11 の s2f の角度 −0.8,0.17,… だと肘が胸の前へ回り込み、
    # 肩 roll が体の側へ −0.5〜−0.8 rad 押す＝11-03 に熱を持った型。11-12 机の上の通り道の確かめで決めた）
    posture: Tuple[float, ...] = (-0.6, 0.5, 0.3, -0.3, 1.5, 0.4, -0.5)
    # 手の先の向き（水平のまま蝶番の側へ振る角度。0＝扉へまっすぐ・90＝棒に沿って真横）。負の値で向きを縛らない。
    # 11-12 実機: 60° は棒の脇を通って空振り、80° は肩 yaw が可動域の端で手が止まった → 0°（まっすぐ前）で手の先で押す形が通った
    hand_dir_deg: float = 0.0
    # 2a 待つ
    settle_s: float = 1.5
    yaw_still_deg: float = 1.5     # 直近 0.5 秒の向きの幅がこれ未満なら止まっている
    wait_max_s: float = 5.0
    # 2b 構える（レバーの当てる点からの差 [前, 横, 上]。横は把手の側が正）
    # 11-12 実機 1 本目: レバーの手前 12 cm をまっすぐ上げたら、手の先（手のひらの原点の約 10 cm 先）が棒の下に掛かり、
    # 肩 pitch 17 N·m で押し上げ続けて 5 秒で 53→76 °C・機体が後ろへ 23 cm 押された。棒の無い把手の側（付け根の外）で上げてから横へ入る
    # 道筋: 把手の側・板から遠い所でレバーより 12 cm 高く上げる → その高さのまま横へ → 前へ出てレバーの 10 cm 上 → 2c で上から下ろす
    # 11-12 夜: 最初の経由点の高さを −0.25（実機で通った値）→ −0.18。−0.25 だと構えの許容の隅（後ろ・右・左回り）で
    # 手が届かず 2b で止まる（script_gate.py）。横 +0.18 は保つ（+0.12 にすると上げる間の棒とのすき間が 5.6→1.4 cm）
    via: Tuple[Tuple[float, float, float], ...] = ((-0.16, 0.18, -0.18), (-0.16, 0.18, 0.12), (-0.16, 0.03, 0.12))
    # 11-12 実機 2 本目: 前 0 だと手の先（手のひらの原点の 10〜12 cm 先）が板に当たり、機体が後ろへ 10 cm 押された。
    # 10 回目に通った形: 手のひらを当てる点の 10 cm 手前・3 cm 付け根寄りに置き、手の先で棒を上から押す
    above: Tuple[float, float, float] = (-0.10, 0.03, 0.10)
    raise_speed: float = 0.15
    raise_max_s: float = 10.0
    via_tol: float = 0.04
    reach_tol: float = 0.02
    base_s: float = 0.3            # レバーの上で止まって空中の値（遅れ・力）を取る長さ
    raise_stall_s: float = 1.2     # 2b: 行き先が先行の上限に張り付いたまま、手がこの間 1 cm も進まなければ止まる（何かに当たっている）
    pelvis_move_max: float = 0.08  # 2b〜3a（脚は止めている）: pelvis が 2a の終わりからこれより動いたら止まる（押されて後ずさりしている）
    # 2c 下ろして触れる
    touch_speed: float = 0.03
    touch_floor: float = -0.09     # 当てる点よりこれだけ下まで触れなければ空振り（手は先が上がっているので、手のひらは棒より下まで下がる）
    # 11-12 実機 3 本目: 「空中の遅れからの差」は、着いた直後の垂れ（−1.2 cm）が下ろす間に消えるだけで立った（偽の触れた）。
    # 「行き先が手のひらよりこれだけ下にあり、かつ手のひらが直近 touch_still_s に touch_still_m も下がっていない」で見る
    lag_line: float = 0.015
    touch_still_s: float = 0.3
    touch_still_m: float = 0.003
    tau_line: float = 6.0          # 肩 pitch の力の見積もりの変化 [N·m]（空中の値から。姿勢で 3 N·m は動く）
    touch_hold: int = 3
    # 11-12 実機 4 本目: このレバーのばねは軽く、手は止まらずにレバーごと下がった（肩 roll の力が +4 N·m 動いただけ）＝止まりでは触れたと分からない。
    # 手のひらが当てる点の高さ＋touch_at まで下りたら触れたとみなす（止まり・力で先に分かればそちらで）
    touch_by_height: bool = True
    touch_at: float = 0.0
    # 2d 押し下げる
    press_speed: float = 0.04
    press_depth: float = 0.07      # 触れた高さからの下がり（11-12 実機: 7 cm で掛け金が外れるのに足りた・下がり切ってはいない）
    press_min: float = 0.03        # 下がりが止まった時、これ以上下がっていれば下げたとみなす
    press_max_s: float = 3.0
    stall_s: float = 0.6
    hold_extra: float = 0.01       # 3c（掛け金が外れた後）は行き先を下げた時の高さよりこれだけ下に置く
    lead_max: float = 0.05         # 行き先が手のひらより先へ行ける上限（押し付けの力の頭打ち）
    # 3a 掛け金を外す
    unlatch_speed: float = 0.05
    unlatch_deg: float = 3.0
    unlatch_stall_s: float = 3.0
    nudge_vx: float = 0.2
    nudge_s: float = 0.5
    # 11-12 夕 空振りの押し直し: 3a で扉の角度が miss_stall_s 伸びない（手の先が板で止まる＝棒に乗っていない）か、
    # 歩いて押す 3a で pelvis が miss_adv 進んでも掛け金が外れない時、手を上げ（歩いた分は下がり）、レバーの上の点を横へ
    # miss_shifts の順にずらして押し下げからやり直す（横は把手の付け根の側が正・当てる点の初めの位置からの差）。使い切ったら元の止まり方
    # 11-12 実機 2 回目: 立ち位置の横の読みが 9.8 cm 違い、手は棒の先の外（蝶番の側）へ下りて空を切った → 付け根の側から試す。
    # 棒は 13 cm・初めの手の先は付け根から 7.4 cm（門番）なので、横の外れ −6.4〜+5.6 cm は初めから乗る。+5・−5・+9 cm で −11〜+15 cm まで拾う
    # 当たった 4 回は 3a に入って 1.0〜1.9 秒で 3° に届いた（0.5° ずつの伸びは 1 秒 空かない）
    miss_shifts: Tuple[float, ...] = (0.05, -0.05, 0.09)
    miss_stall_s: float = 1.5
    miss_adv: float = 0.12         # 手は 0.05 m/s でしか前へ出ないので、当たった回でも 3° までに体は 5〜10 cm 寄る（それより大きく）
    miss_back_s: float = 4.0       # 歩いた分を下がる時間の上限
    # 3c 押しながら歩く
    push_mode: str = "walk"        # "walk"＝腕は体に対して固めて前へ歩き続ける／"pulse"＝腕で押し、手が前へ出すぎたら短く歩く
    walk_in_3a: bool = False       # walk: 3a（掛け金を外す）から前へ歩き出す（案 ②。脚を止めて押すと後ろへ踏み出す時に試す）
    push_press: float = 0.02       # walk: 腕の行き先を、体に対して固めた位置よりこれだけ前に置く（扉に押し当て続ける分）
    walk_min_fwd: float = 0.15     # walk: 手が pelvis の前へこれより近くまで押し込まれたら止まる（扉が押し返して体が手に追いついた）
    angle_med_s: float = 0.5       # 扉の角度の終わりの判定は、直近この長さの中央値で（歩く間の読みの跳びに引きずられない）
    push_lead_deg: float = 4.0
    push_dz: float = 0.0           # 3c の行き先の高さの足し（0＝下げた時の高さ − hold_extra）
    push_dr: float = 0.0           # 行き先を蝶番から遠ざける足し [m]（負で蝶番寄り＝板の上へ移す時）
    push_dn: float = 0.0           # 3c の行き先を板へ向けてずらす [m]（11-13: 手の先が棒でなく板の表に当たるまで。約 0.06）
    push_z: float = 0.0            # 3c の手のひらの高さ [m]（0＝下げた時の高さ − hold_extra。11-13: レバーの下の板 0.80）
    goal_deg: float = 30.0
    push_max_s: float = 20.0
    push_stall_s: float = 3.0
    reach_hi: float = 0.37         # 手のひらが pelvis の前へこれだけ出たら脚を前へ（腕の届きは肩の高さで約 0.41 m）
    reach_lo: float = 0.30         # ここまで戻ったら脚を止める
    walk_vx: float = 0.2
    walk_vy: float = 0.0
    walk_wz: float = 0.0
    walk_yaw_kp: float = 1.5       # walk: 3c に入った時の向きへ戻す倍率 [1/s]（0 で切る。11-13 実機で 3c に体が +38° 回った）
    walk_wz_max: float = 0.5       # walk: その回りの指令の上限 [rad/s]（段 1 の WZ_MAX と同じ）
    walk_lat_kp: float = 1.0       # walk: 3c に入った時の横の位置へ戻す倍率 [1/s]（0 で切る）
    walk_yaw_ff: float = 0.0       # walk: 前の速さ 1 m/s あたりの先回りの回り [rad/s]（把手の側へ回される分を先に打ち消す。11-13）
    walk_hand: str = "body"        # walk: "body"＝手を体に固める／"arc"＝手を扉の角度＋push_lead_deg の板の上に置く（11-13）
    walk_fwd_hold: float = 0.24    # walk・arc: 手の前への出がこれ未満なら、前の指令を walk_vx_hold へ落とす [m]
    walk_vx_hold: float = 0.2      # walk・arc: その時の前の指令 [m/s]（0 にすると止まって後ろへ踏み出す）
    # 11-13: 3c の前の指令を walk_vx_start から walk_vx へ walk_ramp_s 秒かけて上げる（0 は一気に。当たりの衝撃を小さく）
    walk_ramp_s: float = 0.0
    walk_vx_start: float = 0.15    # 純正が歩き出す線（0.1 は 0〜10%・0.15 で 59% 応える。11-09 の掃引）
    walk_3a_vx: float = 0.0        # walk_in_3a: 3a の前の指令 [m/s]（0 は walk_vx。11-13: 0.2＝足踏みで掛け金の山を受け止める）
    walk_vy_max: float = 0.15      # walk: その横の指令の上限 [m/s]
    pulse_max_s: float = 1.0
    pulse_gap_s: float = 0.6
    # 守り
    tau_max: float = 20.0          # 肩 pitch の力の見積もりの上限 [N·m]
    tau_max_s: float = 2.0
    yaw_abort_deg: float = 8.0     # 2c〜3a で体がこれだけ回ったら手を上げてやり直す
    retries: int = 1
    waist_comp_from: str = "2c"    # この小段から腰 yaw の打ち消しを頼む（走らせ役の --waist-comp-gain が 0 なら効かない）。"" で頼まない
    done_hold_s: float = 1.0
    stop_after: str = ""           # この小段が終わったら立ったまま止まる（"" は最後まで）
    pose_tau_s: float = 0.2        # 腕の行き先に使う pelvis の姿勢のなまし
    # 止まった時（走らせ役の POST）に手を扉から離す: 手のひらを扉の座標で後ろへ release_back・上へ release_up、release_s 秒で
    # （11-12 実機: 手が扉に当たったまま保つと、2 回目は 20 秒で 56→72 °C・10 回目は 94 秒で 56→66 °C）
    release_back: float = 0.04
    release_up: float = 0.02
    release_s: float = 1.5
    # 把手と反対の手（把手が左なら右手）を板に添えて両手で押す（11-13 ユーザー案。左手 1 本だと扉の押し返しで体が
    # 左へ回され、回りの指令を上限にしても戻せなかった＝8 回目は 1 秒で +36°）。動かすのは走らせ役の側（door_run_staged.ScriptArm）
    # ⚠️ 門番（script_gate.py・door_clearance.py）はこの手の当たりを見ていない
    helper: bool = False
    helper_from: str = "2c"        # この小段から手を板の前に構える（2b は左腕が大きく動くので、その後）
    helper_dy: float = 0.32        # 手のひらの横: 当てる点から蝶番の側へ [m]（レバーの先から約 30 cm 離れる）
    helper_z: float = 0.95         # 手のひらの高さ [m]（右肩は床から約 1.02 m）
    helper_x_ready: float = -0.20  # 構えの手のひらの原点の x（手の先は約 10 cm 先＝板の表 −0.02 の 8 cm 手前）
    helper_x_press: float = -0.10  # 3a から寄せる x（手の先が板の表を 2 cm 越えて押し当てる）
    helper_speed: float = 0.25     # 構えへ寄せる速さ [m/s]
    helper_press_speed: float = 0.10  # 3a で板へ寄せる速さ [m/s]
    helper_back: float = 0.10      # 止まった時に手を引く量 [m]（release_s 秒で）
    note: str = ""


def load_params(path: Optional[str] = None, **override) -> ScriptParams:
    """定数ファイル（JSON）を読む。知らない名前・型の違う値は起動で落とす（当日の打ち間違いを機体の前で見つける）"""
    raw = {}
    if path:
        with open(path, encoding="utf-8") as f:
            raw = json.load(f)
    raw = {**raw, **{k: v for k, v in override.items() if v is not None}}
    known = {f.name: f for f in fields(ScriptParams)}
    bad = sorted(set(raw) - set(known))
    if bad:
        raise ValueError(f"台本の定数に知らない名前: {bad}")
    out = {}
    for k, v in raw.items():
        d = getattr(ScriptParams(), k)
        if isinstance(d, tuple):
            v = tuple(tuple(float(x) for x in e) if isinstance(e, (list, tuple)) else float(e) for e in v)
        elif isinstance(d, bool) or isinstance(d, str):
            if not isinstance(v, type(d)):
                raise ValueError(f"台本の定数 {k} の型が違う: {v!r}")
        elif isinstance(d, int):
            v = int(v)
        else:
            v = float(v)
        out[k] = v
    p = replace(ScriptParams(), **out)
    check_params(p)
    return p


def check_params(p: ScriptParams) -> None:
    if len(p.posture) != 7 or len(p.above) != 3 or any(len(v) != 3 for v in p.via) or len(p.stance_tol) != 3:
        raise ValueError("台本の定数: posture は 7 つ・above と via の各点は 3 つ・stance_tol は 3 つ")
    if p.push_mode not in PUSH_MODES:
        raise ValueError(f"push_mode は {PUSH_MODES} のどれか: {p.push_mode!r}")
    if p.stop_after not in STOP_AFTER:
        raise ValueError(f"stop_after は {STOP_AFTER} のどれか: {p.stop_after!r}")
    if p.waist_comp_from not in ("",) + SUBS[:6]:
        raise ValueError(f"waist_comp_from は小段の名前か空: {p.waist_comp_from!r}")
    nums = [getattr(p, f.name) for f in fields(p) if isinstance(getattr(p, f.name), (int, float))]
    if not all(math.isfinite(float(v)) for v in nums):
        raise ValueError("台本の定数に有限でない値がある")
    pos = ("settle_s", "raise_speed", "touch_speed", "press_speed", "press_depth", "lead_max", "unlatch_speed", "goal_deg",
           "reach_tol", "via_tol", "tau_max", "pulse_max_s")
    for k in pos:
        if not getattr(p, k) > 0:
            raise ValueError(f"台本の定数 {k} は正: {getattr(p, k)}")
    if not (0 < p.press_min <= p.press_depth and p.reach_lo < p.reach_hi and p.unlatch_deg < p.goal_deg):
        raise ValueError("台本の定数: press_min ≤ press_depth・reach_lo < reach_hi・unlatch_deg < goal_deg")
    if p.push_mode == "walk" and not p.walk_vx:
        raise ValueError("push_mode walk には walk_vx が要る（0 では押せない）")
    if not (0.0 <= p.release_back <= 0.10 and 0.0 <= p.release_up <= 0.10 and p.release_s > 0):
        raise ValueError("release_back・release_up は 0〜0.10 m、release_s は正")
    if p.walk_vx and abs(p.walk_vx) < ds.V_MIN:
        raise ValueError(f"walk_vx は 0 か {ds.V_MIN} m/s 以上（純正は 0.1〜0.15 m/s では動かない）")
    if not (all(abs(v) <= MISS_SHIFT_MAX for v in p.miss_shifts) and p.miss_stall_s > 0 and p.miss_adv > 0 and p.miss_back_s > 0):
        raise ValueError(f"miss_shifts は ±{MISS_SHIFT_MAX} m の内、miss_stall_s・miss_adv・miss_back_s は正")
    if not (-0.60 <= p.stance_x <= ds.NEAR_STOP_X - 0.02):
        raise ValueError(f"stance_x は −0.60〜{ds.NEAR_STOP_X - 0.02:.2f} m（近すぎると段 1 が止まる）: {p.stance_x}")
    if not (0.0 <= p.push_dn <= 0.12 and (p.push_z == 0.0 or 0.60 <= p.push_z <= 1.10)):
        raise ValueError("push_dn は 0〜0.12 m・push_z は 0 か 0.60〜1.10 m")
    if not -0.05 <= p.ref_dy <= 0.06:
        raise ValueError(f"ref_dy は −0.05〜0.06 m（付け根は寸法の当てる点から 10.5 cm）: {p.ref_dy}")
    if not (0.0 <= p.twist_deg <= 40.0 and 0.1 <= p.twist_rate <= 1.0):
        raise ValueError("twist_deg は 0〜40°・twist_rate は 0.1〜1.0 rad/s")
    if p.twist_deg > 0 and p.waist_comp_from:
        raise ValueError("twist_deg を使う時は waist_comp_from を空にする（腰 yaw の打ち消しと一緒には使わない）")
    if not (0.0 <= p.lean_deg <= 20.0 and 0.05 <= p.lean_rate <= 0.6 and p.lean_from in SUBS[:6]):
        raise ValueError(f"lean_deg は 0〜20°・lean_rate は 0.05〜0.6 rad/s・lean_from は {SUBS[:6]} のどれか")
    if not (0.0 <= p.walk_ramp_s <= 10.0 and 0.12 <= p.walk_vx_start <= 0.3):
        raise ValueError("walk_ramp_s は 0〜10 秒・walk_vx_start は 0.12〜0.3 m/s")
    if p.walk_hand not in WALK_HANDS:
        raise ValueError(f"walk_hand は {WALK_HANDS} のどれか: {p.walk_hand!r}")
    if not (0.0 <= p.walk_yaw_ff <= 2.0 and p.walk_min_fwd < p.walk_fwd_hold and 0.0 <= p.walk_vx_hold):
        raise ValueError("walk_yaw_ff は 0〜2・walk_min_fwd < walk_fwd_hold・walk_vx_hold ≥ 0")
    if p.helper_from not in SUBS[:5]:
        raise ValueError(f"helper_from は {SUBS[:5]} のどれか: {p.helper_from!r}")
    if not (p.helper_speed > 0 and p.helper_press_speed > 0 and 0.0 <= p.helper_back <= 0.20
            and 0.10 <= p.helper_dy <= 0.60 and 0.70 <= p.helper_z <= 1.20 and p.helper_x_ready < p.helper_x_press <= 0.0):
        raise ValueError("helper の定数: 速さは正・helper_back 0〜0.20・helper_dy 0.10〜0.60・helper_z 0.70〜1.20・"
                         "helper_x_ready < helper_x_press ≤ 0")


def script_ref_point(p: ScriptParams, width: float, backset: float, lever_height: float, lever_left: bool = True) -> np.ndarray:
    """台本の当てる点（扉の座標）＝ 寸法から出した当てる点を、棒に沿って付け根の側へ ref_dy ずらした点"""
    lr = 1.0 if lever_left else -1.0
    return ds.lever_ref_point(width, backset, lever_height, lever_left) + np.array([0.0, lr * p.ref_dy, 0.0])


def script_stance(p: ScriptParams, width: float, backset: float, lever_height: float, lever_left: bool = True) -> ds.Stance:
    """台本の構えの位置（段 1 の寄りの行き先）"""
    lr = 1.0 if lever_left else -1.0
    ref = script_ref_point(p, width, backset, lever_height, lever_left)
    return ds.Stance(dfr.PlanarPose(p.stance_x, float(ref[1]) + lr * p.stance_y_off, lr * math.radians(p.stance_yaw_deg)),
                     p.stance_tol[0], p.stance_tol[1], math.radians(p.stance_tol[2]))


class PoseFilter:
    """pelvis の姿勢を 1 次でなます（FAST-LIO の 10 Hz の段と歩みの揺れを腕の行き先へそのまま流さない）"""

    def __init__(self, tau_s: float) -> None:
        self.tau, self.pose = tau_s, None

    def step(self, pose: dfr.PlanarPose, dt: float = DT) -> dfr.PlanarPose:
        if self.pose is None or self.tau <= 0:
            self.pose = pose
            return pose
        a = min(1.0, dt / self.tau)
        o = self.pose
        self.pose = dfr.PlanarPose(o.x + a * (pose.x - o.x), o.y + a * (pose.y - o.y), o.yaw + a * ds.wrap(pose.yaw - o.yaw))
        return self.pose


@dataclass(frozen=True)
class ScriptOut:
    sub: str
    goal: Optional[np.ndarray]     # 扉の座標での手のひらの行き先（None は腕を下ろしたまま）
    walk: Walk
    direction: Optional[np.ndarray]  # 扉の座標での手の先の向き（単位ベクトル。None は縛らない）
    angle_obs: np.ndarray          # door_angle_lidar の形 [角度 rad, 測れたか, 古さ/1.0 s]
    waist_comp: bool
    stop: str = ""                 # 空でなければ立ったまま止まる理由
    success: bool = False
    info: str = ""
    waist_twist: float = 0.0       # 腰 yaw の目標に足す角度 [rad]（借りた時の角度から。半身で押す）
    waist_lean: float = 0.0        # 腰 pitch の目標に足す角度 [rad]（前へが正）


def _toward(g: np.ndarray, wp: np.ndarray, step: float) -> np.ndarray:
    d = wp - g
    n = float(np.linalg.norm(d))
    return wp.copy() if n <= step else g + d * (step / n)


class ScriptDoor:
    """段 2・3 の台本。step には、時刻・なました pelvis の姿勢・扉の座標の手のひら・肩 pitch の力の見積もり・
    pelvis から見た手のひらの前への出（m）を渡す"""

    def __init__(self, p: ScriptParams, width: float, backset: float, lever_height: float, lever_left: bool = True) -> None:
        check_params(p)
        self.p, self.lr = p, (1.0 if lever_left else -1.0)
        self.ref = script_ref_point(p, width, backset, lever_height, lever_left)
        self.line = hda.door_line(width, lever_left)
        self.sub = "2a"
        self.t_sub: Optional[float] = None
        self.t_prev: Optional[float] = None
        self.goal: Optional[np.ndarray] = None
        self.wp_i = 0
        self.yaws: List[Tuple[float, float]] = []
        self.yaw0: Optional[float] = None
        self.base: List[Tuple[float, float]] = []   # レバーの上で止まっている間の（遅れ, 力）
        self.lag0 = self.tau0 = 0.0
        self.touch_n = 0
        self.z_hist: List[Tuple[float, float]] = []      # 2c: 直近の（時刻, 手のひらの高さ）
        self.z_contact: Optional[float] = None
        self.z_min = math.inf
        self.t_progress = 0.0
        self.palm_ref: Optional[np.ndarray] = None
        self.angle = 0.0
        self.angle_best = 0.0
        self.nudged = False
        self.nudge_until: Optional[float] = None
        self.walking_until: Optional[float] = None
        self.walk_rest_until = 0.0
        self.tau_over_t: Optional[float] = None
        self.retries_left = p.retries
        self.miss_i = 0                                  # 空振りの押し直しを使った回数
        self.shift = 0.0                                 # レバーの上の点の横のずらし [m]（把手の付け根の側が正）
        self.x_home: Optional[float] = None              # 押し直しの前に下がって戻る pelvis の x（2a の終わりの位置）
        self.t_back: Optional[float] = None
        self.xy0: Optional[Tuple[float, float]] = None   # 2a の終わりの pelvis の位置
        self.raise_best = math.inf                       # 2b: 今の経由点までの残りの最小
        self.raise_t = 0.0
        self.palm_body: Optional[np.ndarray] = None      # walk: 3c に入った時の手のひら（pelvis の向きの座標・前・左）
        self.yaw_3c0: Optional[float] = None             # walk: 3c に入った時の pelvis の向き（保つ向き）
        self.y_3c0: Optional[float] = None               # walk: 3c に入った時の pelvis の横の位置（保つ位置）
        self.yaw_3a0: Optional[float] = None             # walk_in_3a: 3a に入った時の pelvis の向き（保つ向き）
        self.angles: List[Tuple[float, float]] = []      # 直近の（時刻, 扉の角度 rad）
        self.reason = ""
        self.success = False
        self.lag = self.tau = self.fwd = math.nan
        self.palm = np.full(3, math.nan)

    def twist_rad(self) -> float:
        """腰 yaw に足す角度 [rad]（把手が左なら負＝上体を右へ）"""
        return -self.lr * math.radians(self.p.twist_deg)

    def lean_rad(self) -> float:
        """腰 pitch に足す角度 [rad]（前へが正）。lean_from の小段から（やり直しの retreat・done も保つ）"""
        if self.p.lean_deg <= 0.0:
            return 0.0
        on = self.sub in ("retreat", "done") or (self.sub in SUBS[:6] and SUBS.index(self.sub) >= SUBS.index(self.p.lean_from))
        return math.radians(self.p.lean_deg) if on else 0.0

    # --- 3c の walk ---
    def _enter_3c(self, t: float, pose: dfr.PlanarPose, palm: np.ndarray) -> None:
        c, s = math.cos(pose.yaw), math.sin(pose.yaw)
        d = np.asarray(palm, dtype=float)[:2] - np.array([pose.x, pose.y])
        self.palm_body = np.array([c * d[0] + s * d[1], -s * d[0] + c * d[1]])
        self.yaw_3c0, self.y_3c0 = pose.yaw, pose.y
        self._go("3c", t)

    def _hold_yaw_wz(self, pose: dfr.PlanarPose, ref: Optional[float] = None, ff: bool = True) -> float:
        """walk: 3c に入った時の向きへ戻す回りの指令 [rad/s]。
        11-13 実機: 扉が左の手を押し返し、体が左へ回った（3c の 8.8 秒で +38°・扉 8° で止まる）"""
        p = self.p
        ref = self.yaw_3c0 if ref is None else ref
        if ref is None or not math.isfinite(pose.yaw) or p.walk_yaw_kp <= 0.0:
            return p.walk_wz
        # 先回り: 把手の側の手が押し返されて体は把手の側へ回る（把手が左なら左回り）→ 前の速さに比べた右回りを先に入れる
        ff = -self.lr * p.walk_yaw_ff * p.walk_vx if ff else 0.0
        wz = p.walk_wz + ff + p.walk_yaw_kp * ds.wrap(ref - pose.yaw)
        return float(np.clip(wz, -p.walk_wz_max, p.walk_wz_max))

    def _hold_lat_vy(self, pose: dfr.PlanarPose) -> float:
        """walk: 3c に入った時の横の位置（扉の座標の y）へ戻す横の指令 [m/s・体の座標]。
        11-13 実機: 向きを保った 2 回とも体が蝶番の側へ 11〜12 cm 流れ、手が体の左へ離れて回す力の腕が伸びた"""
        p = self.p
        if self.y_3c0 is None or not (math.isfinite(pose.y) and math.isfinite(pose.yaw)) or p.walk_lat_kp <= 0.0:
            return p.walk_vy
        vy = p.walk_vy + p.walk_lat_kp * math.cos(pose.yaw) * (self.y_3c0 - pose.y)
        return float(np.clip(vy, -p.walk_vy_max, p.walk_vy_max))

    def _arc_goal(self, deg: float, palm: np.ndarray) -> np.ndarray:
        """3c の手の行き先: 扉の角度 deg＋push_lead_deg だけ開いた板の上（当てる点を蝶番の回りに回す）"""
        p = self.p
        tgt = math.radians(min(deg + p.push_lead_deg, p.goal_deg + p.push_lead_deg))
        # push_dn: 閉じた扉の座標で板へ向けて（+x）ずらしてから回す（11-13: レバーでなく、その下の板を押す）
        v = self.palm_ref[:2] + np.array([p.push_dn, 0.0]) - self.line.hinge_xy
        r = float(np.linalg.norm(v))
        v = v * ((r + p.push_dr) / r)
        a = self.line.open_sign * tgt
        c, s = math.cos(a), math.sin(a)
        xy = self.line.hinge_xy + np.array([c * v[0] - s * v[1], s * v[0] + c * v[1]])
        # 掛け金が外れた後は強く押さえない: 高さは下げた時の手のひら − hold_extra（＋ push_dz）。push_z を渡すとその高さ
        z = p.push_z if p.push_z > 0 else self.palm_ref[2] - p.hold_extra + p.push_dz
        return self._lead_cap(np.array([xy[0], xy[1], z]), palm)

    def _push_walk(self, t: float, pose: dfr.PlanarPose, palm: np.ndarray, palm_fwd: float) -> Tuple[Walk, str]:
        """腕は体に対して固め（3c に入った時の手のひらの位置＋push_press 前）、前へ歩き続けて体で押す"""
        p = self.p
        if self.palm_body is None:
            self._enter_3c(t, pose, palm)
        deg = math.degrees(self.angle)
        if p.walk_hand == "arc":  # 11-13: 手は扉の角度＋push_lead_deg の板の上へ（一歩ごとの体の揺れは腕で吸う）
            self.goal = self._arc_goal(deg, palm)
        else:
            c, s = math.cos(pose.yaw), math.sin(pose.yaw)
            b = self.palm_body + np.array([p.push_press, 0.0])
            xy = np.array([pose.x, pose.y]) + np.array([c * b[0] - s * b[1], s * b[0] + c * b[1]])
            self.goal = self._lead_cap(np.array([xy[0], xy[1], self.palm_ref[2] - p.hold_extra + p.push_dz]), palm)
        if deg > self.angle_best + 1.0:
            self.angle_best, self.t_progress = deg, t
        self.walking_until = t + DT  # 記録の「歩きの指令を出しているか」
        if self.angle_median_deg() >= p.goal_deg:
            self.walking_until = None
            self.success = True
            self.goal = np.asarray(palm, dtype=float).copy()
            self._go("done", t)
            return STAND, ""
        if math.isfinite(palm_fwd) and palm_fwd < p.walk_min_fwd:
            self.walking_until = None
            self._fail(f"手が体へ押し込まれた（手の前への出 {palm_fwd:.2f} m・線 {p.walk_min_fwd} m。扉が押し返している）", t)
            return STAND, self.reason
        if t - self.t_progress > p.push_stall_s:
            self.walking_until = None
            self._fail(f"歩いても扉が {p.push_stall_s:.0f} 秒 開いていかない（{deg:.1f}°・手の前への出 {palm_fwd:.2f} m）", t)
            return STAND, self.reason
        if t - self.t_sub > p.push_max_s:
            self.walking_until = None
            self._fail(f"{p.push_max_s:.0f} 秒で {p.goal_deg:.0f}° に届かない（{deg:.1f}°）", t)
            return STAND, self.reason
        vx = p.walk_vx
        if p.walk_ramp_s > 0.0 and p.walk_vx > p.walk_vx_start:
            vx = p.walk_vx_start + (p.walk_vx - p.walk_vx_start) * min(1.0, max(0.0, (t - self.t_sub) / p.walk_ramp_s))
        if p.walk_hand == "arc" and math.isfinite(palm_fwd) and palm_fwd < p.walk_fwd_hold:
            vx = min(p.walk_vx, p.walk_vx_hold)  # 扉より体が先に出ている: 足踏みで踏ん張るだけ（止めると後ろへ踏み出す）
        return (True, vx, self._hold_lat_vy(pose), self._hold_yaw_wz(pose)), ""

    # --- 部品 ---
    def _off(self, d: Sequence[float]) -> np.ndarray:
        return self.ref + np.array([d[0], self.lr * d[1], d[2]])

    def _above(self) -> np.ndarray:
        a = self.p.above
        return self._off((a[0], a[1] + self.shift, a[2]))

    def _miss(self, why: str, t: float) -> bool:
        """空振りとみて押し直す（手を上げ、横へずらす）。使い切っていたら False"""
        p = self.p
        if self.miss_i >= len(p.miss_shifts):
            return False
        self.shift = p.miss_shifts[self.miss_i]
        self.miss_i += 1
        print(f"[script] 空振りとみる（{why}）: 手を上げ、レバーの上の点を横へ {100 * self.shift:+.0f} cm ずらして押し直す"
              f"（{self.miss_i}/{len(p.miss_shifts)} 回目）", flush=True)
        self.x_home = self.xy0[0] if self.xy0 is not None else None
        self.palm_ref, self.angle, self.z_contact, self.angles = None, 0.0, None, []
        self.nudge_until, self.t_back = None, None
        self._go("retreat", t)
        return True

    def _go(self, sub: str, t: float) -> None:
        print(f"[script] {self.sub} → {sub}（{self._where()}）", flush=True)
        self.sub, self.t_sub = sub, t

    def _where(self) -> str:
        d = self.palm - self.ref
        return (f"手のひら−当てる点 前 {100 * d[0]:+.1f} 横 {100 * d[1]:+.1f} 上 {100 * d[2]:+.1f} cm・"
                f"扉 {math.degrees(self.angle):.1f}°")

    def _fail(self, why: str, t: float) -> None:
        self.reason = f"台本 {self.sub}: {why}"
        self._go("failed", t)

    def _angle_obs(self) -> np.ndarray:
        if self.palm_ref is None:
            return dfr.door_closed_angle_obs()
        return np.array([self.angle, 1.0, hda.LATENCY_S / hda.MAX_AGE_S])

    def _lead_cap(self, g: np.ndarray, palm: np.ndarray) -> np.ndarray:
        """行き先を手のひらから lead_max の内へ（上下と水平を別々に。下へ押さえたままでも前へ進められるように）"""
        out = np.array(g, dtype=float, copy=True)
        out[2] = palm[2] + max(-self.p.lead_max, min(self.p.lead_max, out[2] - palm[2]))
        d = out[:2] - palm[:2]
        n = float(np.linalg.norm(d))
        if n > self.p.lead_max:
            out[:2] = palm[:2] + d * (self.p.lead_max / n)
        return out

    def _stop_after(self, done_sub: str) -> bool:
        if self.p.stop_after == done_sub:
            self.reason = f"台本: {done_sub} が終わった（止める所・{self._where()}）"
            return True
        return False

    def row(self) -> np.ndarray:
        """記録の 1 行: [小段の番号, 行き先 xyz, 手のひら xyz, 扉の角度 °, 遅れ m, 肩 pitch の力, 手の前への出 m, 歩きの指令を出しているか]"""
        g = self.goal if self.goal is not None else np.full(3, math.nan)
        return np.array([SUBS.index(self.sub), *g, *self.palm, math.degrees(self.angle), self.lag, self.tau, self.fwd,
                         float(self.walking_until is not None or self.nudge_until is not None)])

    # --- 毎周期 ---
    def angle_median_deg(self) -> float:
        """直近 angle_med_s の扉の角度の中央値 [°]"""
        return math.degrees(float(np.median([a for _, a in self.angles]))) if self.angles else 0.0

    def release_goal(self, palm: np.ndarray) -> np.ndarray:
        """止まった時に手を扉から離す行き先（扉の座標）"""
        return np.asarray(palm, dtype=float) + np.array([-self.p.release_back, 0.0, self.p.release_up])

    def touching(self) -> bool:
        """手が扉かレバーに触れている（触れていそうな）小段か。止まった時に手を離すかの判定"""
        return self.z_contact is not None or self.sub in ("2c", "2d", "3a", "3c", "done")

    def step(self, t: float, pose: dfr.PlanarPose, palm: np.ndarray, tau_pitch: float, palm_fwd: float,  # noqa: C901
             palm_angle: Optional[np.ndarray] = None) -> ScriptOut:
        """palm は行き先の基準（なました姿勢の手のひら）。palm_angle を渡すと扉の角度はそちらで読む
        （なましていない姿勢の手のひら。11-12 実機: 歩く間はなましの遅れで角度の読みが 1 秒に 6.5→1.9→7.2° と跳んだ）"""
        p = self.p
        if self.t_sub is None:
            self.t_sub = t
        dt = DT if self.t_prev is None else min(0.1, max(0.0, t - self.t_prev))
        self.t_prev = t
        palm = np.asarray(palm, dtype=float)
        self.palm, self.tau, self.fwd = palm, float(tau_pitch), float(palm_fwd)
        if self.sub not in ("failed", "done") and not (np.isfinite(palm).all() and math.isfinite(pose.yaw)):
            self._fail("手のひらか姿勢が有限でない", t)
        if self.sub in ("2c", "2d", "3a", "3c") and self.goal is not None:
            self.lag = float(palm[2] - self.goal[2])
        pa = np.asarray(palm_angle, dtype=float) if palm_angle is not None else palm
        if self.palm_ref is not None and np.isfinite(pa).all():
            self.angle = hda.door_angle(self.line, self.palm_ref, pa, "hinge")
            self.angles.append((t, self.angle))
            self.angles = [e for e in self.angles if t - e[0] <= p.angle_med_s]
        self.yaws.append((t, pose.yaw))
        self.yaws = [e for e in self.yaws if t - e[0] <= 0.5]
        walk: Walk = STAND
        stop = ""

        # 守り: 肩 pitch の力・体の回り
        legs_still = not (self.sub == "3a" and p.push_mode == "walk" and p.walk_in_3a)
        if (self.sub in ("2b", "2c", "2d", "3a") and self.xy0 is not None and self.nudge_until is None and not self.nudged
                and legs_still):
            moved = math.hypot(pose.x - self.xy0[0], pose.y - self.xy0[1])
            if moved > p.pelvis_move_max:
                self._fail(f"脚を止めているのに pelvis が {100 * moved:.0f} cm 動いた（押されて後ずさりしている）", t)
        if self.sub in ("2b", "2c", "2d", "3a", "3c"):
            if math.isfinite(tau_pitch) and abs(tau_pitch) > p.tau_max:
                self.tau_over_t = t if self.tau_over_t is None else self.tau_over_t
                if t - self.tau_over_t > p.tau_max_s:
                    self._fail(f"肩 pitch の力 {tau_pitch:+.1f} N·m が {p.tau_max_s:.0f} 秒 上限 {p.tau_max:.0f} を越えた", t)
            else:
                self.tau_over_t = None
        if self.sub in ("2c", "2d", "3a") and self.yaw0 is not None:
            turned = math.degrees(ds.wrap(pose.yaw - self.yaw0))
            if abs(turned) > p.yaw_abort_deg:
                if self.retries_left > 0:
                    self.retries_left -= 1
                    print(f"[script] 体が {turned:+.1f}° 回った: 手を上げてやり直す（残り {self.retries_left} 回）", flush=True)
                    self.palm_ref, self.angle, self.z_contact = None, 0.0, None
                    self._go("retreat", t)
                else:
                    self._fail(f"体が {turned:+.1f}° 回った（やり直しは使い切った）", t)

        if self.sub == "2a":
            still = len(self.yaws) > 5 and math.degrees(max(y for _, y in self.yaws) - min(y for _, y in self.yaws)) < p.yaw_still_deg
            if t - self.t_sub >= p.settle_s and still:
                self.yaw0, self.xy0 = pose.yaw, (pose.x, pose.y)
                self.raise_best, self.raise_t = math.inf, t
                if self.goal is None:  # 初めの 2a。やり直しの 2a は手を上げたまま待っているので行き先と経由点の番号を保つ
                    self.goal, self.wp_i = palm.copy(), 0
                if self._stop_after("2a"):
                    stop = self.reason
                else:
                    self._go("2b", t)
            elif t - self.t_sub > p.wait_max_s:
                self._fail(f"{p.wait_max_s:.0f} 秒待っても体が止まらない", t)
        elif self.sub == "2b":
            wps = [self._off(v) for v in p.via] + [self._above()]
            wp = wps[self.wp_i]
            self.goal = self._lead_cap(_toward(self.goal, wp, p.raise_speed * dt), palm)  # 当たったら行き先は 5 cm 先で止まる
            at_goal = float(np.linalg.norm(self.goal - wp)) < 1e-9
            last = self.wp_i == len(wps) - 1
            left = float(np.linalg.norm(palm - wp))
            if left < self.raise_best - 0.01:
                self.raise_best, self.raise_t = left, t
            if at_goal and not last and float(np.linalg.norm(palm - wp)) < p.via_tol:
                self.wp_i += 1
                self.base = []
                self.raise_best, self.raise_t = math.inf, t
            elif at_goal and last and float(np.linalg.norm(palm - wp)) < p.reach_tol:
                self.base.append((float(palm[2] - self.goal[2]), float(tau_pitch)))
                if len(self.base) * DT >= p.base_s:
                    self.lag0 = float(np.mean([b[0] for b in self.base]))
                    self.tau0 = float(np.nanmean([b[1] for b in self.base])) if np.isfinite([b[1] for b in self.base]).any() else 0.0
                    self.touch_n, self.z_hist = 0, []
                    if self._stop_after("2b"):
                        stop = self.reason
                    else:
                        self._go("2c", t)
            else:
                self.base = []
            if self.sub == "2b" and not stop and left > p.via_tol and t - self.raise_t > p.raise_stall_s:
                self._fail(f"手が {p.raise_stall_s} 秒 進まない（何かに当たっている。経由点 {self.wp_i}・残り {100 * left:.1f} cm・"
                           f"肩 pitch の力 {tau_pitch:+.1f} N·m）", t)
            if self.sub == "2b" and not stop and t - self.t_sub > p.raise_max_s:
                self._fail(f"{p.raise_max_s:.0f} 秒で手がレバーの上に着かない（経由点 {self.wp_i}・残り "
                           f"{100 * float(np.linalg.norm(palm - wp)):.1f} cm）", t)
        elif self.sub == "2c":
            g = self.goal.copy()
            g[2] = max(g[2] - p.touch_speed * dt, self.ref[2] + p.touch_floor - p.lead_max)
            self.goal = self._lead_cap(g, palm)
            lag = float(palm[2] - self.goal[2])
            self.z_hist.append((t, float(palm[2])))
            self.z_hist = [e for e in self.z_hist if t - e[0] <= p.touch_still_s]
            still = (t - self.z_hist[0][0] >= 0.8 * p.touch_still_s
                     and self.z_hist[0][1] - float(palm[2]) < p.touch_still_m)
            dtau = abs(tau_pitch - self.tau0) if math.isfinite(tau_pitch) else 0.0
            self.touch_n = self.touch_n + 1 if ((lag > p.lag_line and still) or dtau > p.tau_line) else 0
            by_height = p.touch_by_height and palm[2] <= self.ref[2] + p.touch_at
            if self.touch_n >= p.touch_hold or by_height:
                self.z_contact, self.z_min, self.t_progress = float(palm[2]), float(palm[2]), t
                print(f"[script] 触れた（{'高さで' if by_height and self.touch_n < p.touch_hold else '止まり・力で'}）: 高さ {palm[2]:.3f} m（当てる点 {self.ref[2]:.3f}）・遅れ {100 * lag:.1f} cm・力の変化 {dtau:.1f} N·m", flush=True)
                if self._stop_after("2c"):
                    stop = self.reason
                else:
                    self._go("2d", t)
            elif palm[2] <= self.ref[2] + p.touch_floor:
                self._fail(f"当てる点の {-100 * p.touch_floor:.0f} cm 下まで下ろしても触れない（空振り）", t)
        elif self.sub == "2d":
            g = self.goal.copy()  # 行き先は手のひらの lead_max 先まで下げ続ける（ばねのあるレバーを押し切る。力は lead_max で頭打ち）
            g[2] = g[2] - p.press_speed * dt
            self.goal = self._lead_cap(g, palm)
            if palm[2] < self.z_min - 0.001:
                self.z_min, self.t_progress = float(palm[2]), t
            depth = self.z_contact - float(palm[2])
            stalled = t - self.t_progress > p.stall_s or t - self.t_sub > p.press_max_s
            if depth >= p.press_depth or (stalled and depth >= p.press_min):
                self.palm_ref, self.angle, self.angle_best, self.t_progress = pa.copy(), 0.0, 0.0, t
                self.angles = []
                self.goal = np.array([palm[0], palm[1], self.goal[2]])  # 押さえる深さはそのまま（掛け金が外れるまで）
                print(f"[script] 下げた: 下がり {100 * depth:.1f} cm・肩 pitch の力 {tau_pitch:+.1f} N·m", flush=True)
                if self._stop_after("2d"):
                    stop = self.reason
                else:
                    self._go("3a", t)
            elif stalled:
                self._fail(f"レバーが下がらない（下がり {100 * depth:.1f} cm・線 {100 * p.press_min:.0f} cm・力 {tau_pitch:+.1f} N·m）", t)
        elif self.sub == "3a":
            g = self.goal.copy()
            g[0] += p.unlatch_speed * dt
            self.goal = self._lead_cap(g, palm)
            deg = math.degrees(self.angle)
            if deg > self.angle_best + 0.5:
                self.angle_best, self.t_progress = deg, t
            walk_3a = p.push_mode == "walk" and p.walk_in_3a
            if walk_3a:  # 11-13: 前は walk_3a_vx（足踏みで踏ん張る）・向きは 3a に入った時の向きを保つ（先回りは入れない）
                if self.yaw_3a0 is None:
                    self.yaw_3a0 = pose.yaw
                walk = (True, p.walk_3a_vx if p.walk_3a_vx > 0 else p.walk_vx, p.walk_vy,
                        self._hold_yaw_wz(pose, self.yaw_3a0, ff=False))
            adv = pose.x - self.xy0[0] if (walk_3a and self.xy0 is not None) else 0.0
            if self.nudge_until is not None:
                if t < self.nudge_until:
                    walk = (True, p.nudge_vx, 0.0, 0.0)
                else:
                    self.nudge_until, self.t_progress = None, t
            if deg >= p.unlatch_deg:
                self.nudge_until = None
                self.t_progress, self.angle_best = t, deg
                if self._stop_after("3a"):
                    stop = self.reason
                    walk = STAND
                else:
                    self._enter_3c(t, pose, palm)
            elif adv > p.miss_adv:
                why = f"pelvis が {100 * adv:.0f} cm 進んでも掛け金が外れない（扉 {deg:.1f}°）"
                if self._miss(why, t):
                    walk = STAND
                else:
                    self._fail(why + "・押し直しは使い切った", t)
            elif (self.nudge_until is None and not self.nudged and t - self.t_progress > p.miss_stall_s
                  and self._miss(f"扉が {p.miss_stall_s} 秒 伸びない（{deg:.1f}°）", t)):
                walk = STAND
            elif self.nudge_until is None and t - self.t_progress > p.unlatch_stall_s:
                if not self.nudged:
                    self.nudged, self.nudge_until = True, t + p.nudge_s
                    print(f"[script] 扉が {p.unlatch_stall_s:.0f} 秒動かない: 脚を前へ {p.nudge_vx} m/s で {p.nudge_s} 秒", flush=True)
                else:
                    self._fail(f"掛け金が外れない（扉 {deg:.1f}°・脚で 1 回押した後）", t)
        elif self.sub == "3c" and p.push_mode == "walk":
            walk, stop = self._push_walk(t, pose, palm, palm_fwd)
        elif self.sub == "3c":
            deg = math.degrees(self.angle)
            self.goal = self._arc_goal(deg, palm)
            if deg > self.angle_best + 1.0:
                self.angle_best, self.t_progress = deg, t
            # 脚: 手が前へ出すぎたら短く前へ（出し続けない。止まって腕で押し、また出す）
            if self.walking_until is not None:
                if t >= self.walking_until or palm_fwd < p.reach_lo:
                    self.walking_until, self.walk_rest_until = None, t + p.pulse_gap_s
                else:
                    walk = (True, p.walk_vx, p.walk_vy, p.walk_wz)
            elif palm_fwd > p.reach_hi and t >= self.walk_rest_until and p.walk_vx:
                self.walking_until = t + p.pulse_max_s
                walk = (True, p.walk_vx, p.walk_vy, p.walk_wz)
            if self.angle_median_deg() >= p.goal_deg:
                self.walking_until = None
                walk = STAND
                self.success = True
                self.goal = palm.copy()
                self._go("done", t)
            elif t - self.t_progress > p.push_stall_s:
                self._fail(f"扉が {p.push_stall_s:.0f} 秒 開いていかない（{deg:.1f}°・手の前への出 {palm_fwd:.2f} m）", t)
            elif t - self.t_sub > p.push_max_s:
                self._fail(f"{p.push_max_s:.0f} 秒で {p.goal_deg:.0f}° に届かない（{deg:.1f}°）", t)
        elif self.sub == "retreat":
            up = np.array([palm[0], palm[1], self.ref[2] + p.above[2]]) if self.goal is None else np.array(
                [self.goal[0], self.goal[1], self.ref[2] + p.above[2]])
            self.goal = _toward(self.goal if self.goal is not None else palm.copy(), up, p.raise_speed * dt)
            hand_up = abs(palm[2] - up[2]) < p.via_tol or t - self.t_sub > 3.0
            # 押し直し: 歩いて押した分は、手が上がってから後ろへ下がって戻る（構えの位置から離れたままだと手が届かない）
            back = hand_up and self.x_home is not None and pose.x > self.x_home + BACK_TOL_M
            if back:
                self.t_back = t if self.t_back is None else self.t_back
                back = t - self.t_back < p.miss_back_s
            if back:
                walk = (True, -abs(p.walk_vx), 0.0, 0.0)
            elif hand_up:
                self.x_home, self.t_back = None, None
                self.yaws = []
                self.xy0 = None
                self.wp_i = len(p.via)  # 経由点は飛ばしてレバーの上へ戻る
                self._go("2a", t)
        elif self.sub == "done":
            if t - self.t_sub >= p.done_hold_s:
                stop = f"台本: 扉 {self.angle_median_deg():.0f}° まで開けた（成功）"
        if self.sub == "failed":
            stop = self.reason
        wc = bool(p.waist_comp_from) and self.sub in SUBS[SUBS.index(p.waist_comp_from):5]  # 脚を止めている 3a まで
        goal = self.goal.copy() if self.goal is not None else None
        direction = None
        if p.hand_dir_deg >= 0.0:
            a = math.radians(p.hand_dir_deg)
            direction = np.array([math.cos(a), -self.lr * math.sin(a), 0.0])
        return ScriptOut(self.sub, goal, walk, direction, self._angle_obs(), wc, stop, self.success and self.sub == "done", self._where(),
                         waist_twist=self.twist_rad(), waist_lean=self.lean_rad())
