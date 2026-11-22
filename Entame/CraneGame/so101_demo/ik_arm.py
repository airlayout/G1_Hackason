"""腕の先端の位置（前 x・高さ z）と道具の向きから、肩・肘・手首の関節値を逆算する（平面。台座 pan は別）。

人の腕を「前へ伸ばす」「上げる」に合わせるには、関節を1つずつ手の入力に結ぶのではなく、腕全体の形（先端の位置と
道具の向き）を決めて、そこから各関節を求める必要がある。肩を手の高さ、肘を手の大きさで別々に動かすと、肩だけが
端（腕を後ろへ振り上げた姿勢）に張り付き、腕が上を向く（2026-11-22 録画ログ。CRANE_PLAN.md §0）。

形は SO-101 公式モデル（sim_assets/SO101/so101_new_calib.xml）から取った。関節の角度が 0 のとき、上腕は真上、前腕は
前へ水平、道具もほぼ水平（先端 x=0.39 m）で、教示した H4「腕を前へ水平に伸ばす」（肩 -3・肘 20・手首 -6）に近い。
MuJoCo の FK と先端が 1e-9 m で一致することは test_ik_arm.py で確かめる（mujoco が入っているときだけ）。
座標: 台座の回転軸の位置が原点、x=前、z=上（m）。角度の正は、腕が前下へ倒れる向き。
"""
import math

SHOULDER = (0.0692345, 0.1166)        # 肩の軸の位置
UPPER = (0.0280000, 0.1125700)        # 肩 → 肘（角度 0 のとき）
FORE = (0.1349000, 0.0052000)         # 肘 → 手首
TOOL = (0.1592274, -0.0079013)        # 手首 → 先端
LIFT_SIGN = ELBOW_SIGN = WRIST_SIGN = 1


def rotate(v, a):
    """ベクトル v を角度 a（rad）だけ、前下へ倒す向きに回す。"""
    return (v[0] * math.cos(a) + v[1] * math.sin(a), -v[0] * math.sin(a) + v[1] * math.cos(a))


def direction(v):
    return math.atan2(v[1], v[0])


def forward(lift, elbow, wrist):
    """関節角（rad）から、先端の位置 (x, z) と道具の向き（度。水平 0、上向き＋）。"""
    a1, a2, a3 = lift, lift + elbow, lift + elbow + wrist
    e = rotate(UPPER, a1)
    w = rotate(FORE, a2)
    t = rotate(TOOL, a3)
    tip = (SHOULDER[0] + e[0] + w[0] + t[0], SHOULDER[1] + e[1] + w[1] + t[1])
    return tip, math.degrees(direction(t))


def inverse(x, z, pitch_deg):
    """先端 (x, z)・道具の向き pitch_deg（度）に当たる関節角（rad）を (lift, elbow, wrist, 届いたか) で返す。
    届かないときは、手首の位置を肩から届く距離に引き寄せた解を返す（届いたか = False）。肘は「上」側の解（角度 0 の形に近い側）。"""
    a3 = direction(TOOL) - math.radians(pitch_deg)          # 道具の向きを決める 手首までの角の合計
    tool = rotate(TOOL, a3)
    wx, wz = x - tool[0], z - tool[1]
    sx, sz = wx - SHOULDER[0], wz - SHOULDER[1]
    l1, l2 = math.hypot(*UPPER), math.hypot(*FORE)
    dist = math.hypot(sx, sz)
    lo, hi = abs(l1 - l2) + 1e-6, l1 + l2 - 1e-6
    reached = lo <= dist <= hi
    if not reached:
        scale = min(hi, max(lo, dist)) / (dist or 1.0)
        sx, sz, dist = sx * scale, sz * scale, min(hi, max(lo, dist))
    cos_gamma = (dist * dist - l1 * l1 - l2 * l2) / (2 * l1 * l2)
    gamma = -math.acos(max(-1.0, min(1.0, cos_gamma)))      # 肘が上側になる解
    theta_u = math.atan2(sz, sx) - math.atan2(l2 * math.sin(gamma), l1 + l2 * math.cos(gamma))
    lift = direction(UPPER) - theta_u
    elbow = direction(FORE) - lift - (theta_u + gamma)
    wrist = a3 - lift - elbow
    return lift, elbow, wrist, reached
