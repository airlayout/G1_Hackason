"""重力整列済みの live 点群と地図を (x,y,yaw) の3自由度で大域照合する。

両方とも水平化済みなので、roll/pitch を探索する必要は無い。6自由度で探索すると
上下反転(roll=180°)の偽解に落ちるため、3自由度に限定する。
FFT 相関で全平行移動を一括評価し、yaw を走査する。
"""
import sys
import numpy as np
import open3d as o3d
from scipy.signal import fftconvolve

RES = 0.20  # m/cell

def rasterize(pts_xy, x0, y0, w, h, res=RES):
    g = np.zeros((h, w), dtype=np.float32)
    c = np.floor((pts_xy[:,0]-x0)/res).astype(np.int64)
    r = np.floor((pts_xy[:,1]-y0)/res).astype(np.int64)
    m = (c>=0)&(c<w)&(r>=0)&(r<h)
    g[r[m], c[m]] = 1.0
    return g

live = np.asarray(o3d.io.read_point_cloud(sys.argv[1]).points)[:, :2]
mp   = np.asarray(o3d.io.read_point_cloud(sys.argv[2]).points)[:, :2]

# 地図の格子(外れ値を1%タイルで切る)
x0,x1 = np.percentile(mp[:,0],[0.5,99.5]); y0,y1 = np.percentile(mp[:,1],[0.5,99.5])
pad = 5.0
x0-=pad; x1+=pad; y0-=pad; y1+=pad
w = int(np.ceil((x1-x0)/RES)); h = int(np.ceil((y1-y0)/RES))
map_g = rasterize(mp, x0,y0,w,h)
print(f"地図格子 {w}x{h} ({RES}m/cell) 占有セル {int(map_g.sum())}")

# live は原点中心。地図と同じ格子サイズに描く(中心を合わせる)
best = None
scores = []
for yaw_deg in np.arange(0, 360, 2.0):
    a = np.radians(yaw_deg)
    R = np.array([[np.cos(a),-np.sin(a)],[np.sin(a),np.cos(a)]])
    lp = live @ R.T
    # live を格子中心に配置
    cx, cy = (x0+x1)/2, (y0+y1)/2
    live_g = rasterize(lp + [cx,cy], x0,y0,w,h)
    n_live = live_g.sum()
    if n_live < 50: continue
    corr = fftconvolve(map_g, live_g[::-1,::-1], mode='same')
    i = int(np.argmax(corr)); r,c = np.unravel_index(i, corr.shape)
    peak = float(corr[r,c])/float(n_live)
    scores.append((peak, yaw_deg, r, c, n_live, corr))
    if best is None or peak > best[0]:
        best = (peak, yaw_deg, r, c, n_live, corr)

peak, yaw, r, c, n_live, corr = best
# セル -> world 変換。相関の 'same' モードでは中心が無シフトに対応
dx = (c - w//2)*RES; dy = (r - h//2)*RES
cx, cy = (x0+x1)/2, (y0+y1)/2
print(f"\n=== 最良解 ===")
print(f"  yaw = {yaw:.1f}°   ロボット位置 (map座標) = ({cx+dx:+.2f}, {cy+dy:+.2f})")
print(f"  一致率 = {peak:.3f}  (live の占有セル {int(n_live)} 個のうち地図と重なった割合)")

allpeaks = np.array(sorted([s[0] for s in scores])[::-1])
print(f"\n=== 曖昧さの評価 ===")
print(f"  yaw ごとの一致率: 最良 {allpeaks[0]:.3f} / 2位 {allpeaks[1]:.3f} / 中央 {np.median(allpeaks):.3f} / 最低 {allpeaks[-1]:.3f}")
print(f"  最良/中央 = {allpeaks[0]/np.median(allpeaks):.2f} 倍")
flat = corr.ravel()/n_live
print(f"  最良yawでの平行移動方向の鋭さ: 最良 {flat.max():.3f} / 99%tile {np.percentile(flat,99):.3f} / 中央 {np.median(flat):.3f}")
np.save(sys.argv[3] if len(sys.argv)>3 else '/tmp/corr.npy', corr/n_live)
