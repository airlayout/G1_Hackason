#!/usr/bin/env python3
"""保存地図に対する **初期の `map→odom`** を、実スキャンから大域探索で求める。

## 何に使うのか

[map_localizer.py](map_localizer.py) は**局所探索**なので、出発点が地図上のどこかを
おおよそ知っている必要がある。その初期値をここで求める。
`map_localizer.py --initial X Y YAW` に渡す。

`g1_slam_odom_tf.py --map-to-odom` に渡せば、連続 localization を使わない
（起動時に1回だけ合わせる）運用にもできる。

## ⚠️ 先に `--lidar-yaw` を正しくすること

`g1_slam_odom_tf.py` の自動校正は**重力で roll/pitch しか決められず、
鉛直軸まわりの yaw は未拘束のまま残る**。MID-360 は逆さ取付（U-09）なので
**`--lidar-yaw 180` が必要**。

実測（2026-09-13、記録済み bag）:

| `--lidar-yaw` | 残差 yaw | 距離の中央値 | 20cm以内 |
|---|---|---|---|
| 0（未補正） | 168° | 1.23 m | 23% |
| **180** | **+2°** | **0.000 m** | **97%** |

yaw がずれたまま局所マッチャを回しても、窓（±数度）の外なので**永久に合わない**。

## 使い方

    # Nav2 の map_server と g1_slam_odom_tf.py が動いている状態で
    python3 find_map_offset.py
    python3 find_map_offset.py --yaw-range 180 --yaw-step 3   # 全周を探す

⚠️ 地図が広いと 180° 全周探索は数秒かかる。既定は ±20°（yaw を直した後の微調整用）。
"""

import argparse
import math, sys, time, numpy as np, rclpy
sys.path.insert(0, str(__import__("pathlib").Path(__file__).resolve().parent))
from map_localizer import cloud_xyz, distance_field
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSHistoryPolicy, QoSProfile, QoSReliabilityPolicy
from tf2_ros import Buffer, TransformListener

class G(Node):
    def __init__(self, args):
        super().__init__("find_map_offset")
        self.a=args
        self.f=None; self.done=False
        self.buf=Buffer(); self.lis=TransformListener(self.buf,self)
        mq=QoSProfile(depth=1,history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.RELIABLE,
                      durability=QoSDurabilityPolicy.TRANSIENT_LOCAL)
        self.create_subscription(OccupancyGrid,args.map_topic,self.on_map,mq)
        cq=QoSProfile(depth=1,history=QoSHistoryPolicy.KEEP_LAST,
                      reliability=QoSReliabilityPolicy.BEST_EFFORT,
                      durability=QoSDurabilityPolicy.VOLATILE)
        self.create_subscription(PointCloud2,args.cloud_topic,self.on_cloud,cq)
    def on_map(self,m):
        if self.f is not None: return
        g=np.array(m.data,dtype=np.int16).reshape(m.info.height,m.info.width)
        self.occ=(g>=65).astype(np.float32); self.res=m.info.resolution
        self.org=(m.info.origin.position.x,m.info.origin.position.y)
        self.f=distance_field(self.occ>0.5,self.res)
        print(f"地図 {m.info.width}x{m.info.height} 占有={int(self.occ.sum())}",flush=True)
    def on_cloud(self,msg):
        if self.f is None or self.done: return
        try: tf=self.buf.lookup_transform("odom",msg.header.frame_id,rclpy.time.Time())
        except Exception: return
        a=cloud_xyz(msg)
        if a is None: return
        t=tf.transform.translation; q=tf.transform.rotation
        x,y,z,w=q.x,q.y,q.z,q.w
        R=np.array([[1-2*(y*y+z*z),2*(x*y-z*w),2*(x*z+y*w)],
                    [2*(x*y+z*w),1-2*(x*x+z*z),2*(y*z-x*w)],
                    [2*(x*z-y*w),2*(y*z+x*w),1-2*(x*x+y*y)]])
        p=a@R.T+np.array([t.x,t.y,t.z])
        m2=(p[:,2]>=0.3)&(p[:,2]<=1.8); pts=p[m2][:,:2].astype(np.float64)
        if len(pts)>4000:
            pts=pts[np.random.choice(len(pts),4000,replace=False)]
        self.done=True
        print(f"スキャン {len(pts)}点で大域探索する",flush=True)
        h,w2=self.occ.shape
        self.OCCF=np.fft.rfft2(self.occ)
        best=(1e9,None)
        t0=time.time()
        for deg in np.arange(-self.a.yaw_range, self.a.yaw_range + 1e-9, self.a.yaw_step):
            th=math.radians(float(deg)); c,s=math.cos(th),math.sin(th)
            xs=c*pts[:,0]-s*pts[:,1]; ys=s*pts[:,0]+c*pts[:,1]
            # スキャンを地図と同じ格子にラスタ化(原点は地図の原点基準、平行移動0の状態)
            cc=np.floor((xs-self.org[0])/self.res).astype(np.int64)
            rr=np.floor((ys-self.org[1])/self.res).astype(np.int64)
            ok=(cc>=0)&(cc<w2)&(rr>=0)&(rr<h)
            if ok.sum()<200: continue
            sg=np.zeros((h,w2),dtype=np.float32); sg[rr[ok],cc[ok]]=1.0
            # 相関(numpy の FFT で。scipy はコンテナに入っていない)
            corr=np.fft.irfft2(self.OCCF*np.conj(np.fft.rfft2(sg)),s=(h,w2))
            idx=np.unravel_index(np.argmax(corr),corr.shape)
            du,dv=int(idx[0]),int(idx[1])
            if du>h//2: du-=h
            if dv>w2//2: dv-=w2
            score=-corr[idx]
            if score<best[0]:
                best=(score,(dv*self.res,du*self.res,float(deg),corr[idx]))
        print(f"探索 {time.time()-t0:.1f}秒",flush=True)
        if best[1] is None: print("見つからず",flush=True); return
        dx,dy,deg,ov=best[1]
        print(f"\n★ 最良: dx={dx:.3f} dy={dy:.3f} yaw={deg:.1f}°  重なりセル数={ov:.0f}",flush=True)
        # その位置での距離場スコアを出す
        th=math.radians(float(deg)); c,s=math.cos(th),math.sin(th)
        xs=c*pts[:,0]-s*pts[:,1]+dx; ys=s*pts[:,0]+c*pts[:,1]+dy
        cc=np.floor((xs-self.org[0])/self.res).astype(np.int64)
        rr=np.floor((ys-self.org[1])/self.res).astype(np.int64)
        ok=(cc>=0)&(cc<w2)&(rr>=0)&(rr<h); d=self.f[rr[ok],cc[ok]]
        print(f"   距離: 平均={d.mean():.3f} 中央={np.median(d):.3f} "
              f"20cm以内={100*(d<0.2).mean():.0f}% 50cm以内={100*(d<0.5).mean():.0f}%",flush=True)
        print(f"   参考(恒等 dx=dy=yaw=0 のとき):",flush=True)
        cc=np.floor((pts[:,0]-self.org[0])/self.res).astype(np.int64)
        rr=np.floor((pts[:,1]-self.org[1])/self.res).astype(np.int64)
        ok=(cc>=0)&(cc<w2)&(rr>=0)&(rr<h); d0=self.f[rr[ok],cc[ok]]
        print(f"   距離: 平均={d0.mean():.3f} 中央={np.median(d0):.3f} "
              f"20cm以内={100*(d0<0.2).mean():.0f}%",flush=True)
def build_parser():
    p=argparse.ArgumentParser(description=__doc__,
                              formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--map-topic",default="/map")
    p.add_argument("--cloud-topic",default="/utlidar/cloud_livox_mid360")
    p.add_argument("--yaw-range",type=float,default=20.0,
                   help="探索する yaw の範囲[度]。全周を探すなら 180")
    p.add_argument("--yaw-step",type=float,default=1.0,help="yaw の刻み[度]")
    p.add_argument("--timeout",type=float,default=120.0)
    return p

_args=build_parser().parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:]
                                if "--ros-args" in sys.argv else sys.argv[1:])
rclpy.init(); n=G(_args)
try:
    t0=time.time()
    while time.time()-t0<_args.timeout and not n.done: rclpy.spin_once(n,timeout_sec=0.5)
    t0=time.time()
    while time.time()-t0<5: rclpy.spin_once(n,timeout_sec=0.2)
finally:
    n.destroy_node(); rclpy.shutdown()
