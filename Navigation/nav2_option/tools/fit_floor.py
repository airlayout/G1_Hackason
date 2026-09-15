"""点群から床平面を当てて、センサーの傾きと高さを出す(2026-09-09 の項目6 と同じ土俵)。

基準値(立位、2026-09-09): 法線 (+0.0657,+0.0100,+0.9978) / 傾き 3.81° / 高さ 1.213 m
"""
import sys
import numpy as np

pts = np.load(sys.argv[1])
acc = np.load(sys.argv[2]) if len(sys.argv) > 2 else None
print(f"点数 {pts.shape[0]}")

# 床は必ず「センサーZ軸の下側」にある。逆さ取付(roll≈180°)なので cloud 系では +Z 側。
# 符号を決め打ちせず、|z| が大きい側の点をまとめて候補にする。
r = np.linalg.norm(pts[:, :2], axis=1)
cand = pts[(r > 1.0) & (r < 8.0)]
rng = np.random.default_rng(0)
sub = cand[rng.choice(len(cand), size=min(200000, len(cand)), replace=False)]

best = (0, None)
for _ in range(400):
    s = sub[rng.choice(len(sub), size=3, replace=False)]
    n = np.cross(s[1] - s[0], s[2] - s[0])
    nn = np.linalg.norm(n)
    if nn < 1e-6:
        continue
    n = n / nn
    # 床はセンサーZ軸にほぼ垂直な面。壁を拾わないように絞る
    if abs(n[2]) < 0.80:
        continue
    d = -n @ s[0]
    inl = int((np.abs(sub @ n + d) < 0.03).sum())
    if inl > best[0]:
        best = (inl, (n, d))

n, d = best[1]
if n[2] < 0:            # 法線の向きを +Z 側へ揃える(09-09 の表記に合わせる)
    n, d = -n, -d
# inlier で最小二乗に掛け直す
m = np.abs(sub @ n + d) < 0.05
X = sub[m]
c = X.mean(axis=0)
n = np.linalg.svd(X - c, full_matrices=False)[2][2]
if n[2] < 0:
    n = -n
d = -n @ c
tilt = np.degrees(np.arccos(min(1.0, abs(n[2]))))
az = np.degrees(np.arctan2(n[1], n[0]))
print(f"床平面 inlier {int(m.sum())} 点 / 候補 {len(sub)} 点")
print(f"法線 = ({n[0]:+.4f}, {n[1]:+.4f}, {n[2]:+.4f})")
print(f"**傾き = {tilt:.2f}°**   (基準: 立位 3.81°)   傾きの方位 = {az:+.1f}°")
print(f"**センサー高さ = {abs(d):.3f} m**   (基準: 立位 1.213 m)")
if acc is not None:
    g = acc.mean(axis=0); g = g / np.linalg.norm(g)
    print(f"参考 IMU重力 = ({g[0]:+.4f}, {g[1]:+.4f}, {g[2]:+.4f})  "
          f"法線との角度 = {np.degrees(np.arccos(min(1.0, abs(g @ n)))):.2f}°")
