#!/usr/bin/env python3
"""RPP の追従点（base_link 系）と指令 vyaw の符号を突き合わせる。

`/lookahead_point` は RPP が **base_link 系**で出す「carrot」で、
RPP はその y の符号どおりに旋回する（curvature = 2y/(x^2+y^2)）。
だから y>0 なら vyaw>0 でなければならない。ここが食い違うなら
base_link の向きか TF の合成が疑わしい。

## これで何を見つけたか（2026-09-09）

**`carrot_z` と `pitch` の 2 列が本題である。**

```
carrot_x carrot_y carrot_z |   cmd_vx cmd_vyaw |   roll°  pitch°    yaw°
   0.400   -0.010    0.010 |    0.284   -0.036 |    1.55   12.22   26.07   ← 壊れている
   0.400   -0.004    0.010 |    0.000    0.000 |    1.63    0.20   25.01   ← 直った後
```

`base_link` が pitch 12° / 高さ 1.27m（＝ LiDAR の位置）になっていた。経路の点は
map の z=0 に在るので、RPP が 3D で `base_link` 系へ変換すると
**全点が一様に約 +0.27m 前・約 -0.033m 右にずれる**（RPP は変換の**後**に z を 0 に
するので歪みが残る）。carrot は「0.400m 先」を名乗るが実際は 0.13m 先で、
右への系統的な偏りが乗る。**歩かせると右へ逸れ続ける。**

⚠️ **落ちない。経路も出る。コストマップも埋まる。**だから
`preflight.sh` が `map -> base_link` の z と pitch を見るようにしてある。

⚠️ **符号の食い違いを 2D で判定してはいけない。** 「機体の yaw とゴールへの方位」を
平面で比べると 48/49 で不一致に見えたが、**RPP は 3D で変換している**ので
それは誤った比較だった。このスクリプトが `/lookahead_point` を直に読むのはそのため。

使い方（Nav2 と同じ DDS 設定で。ゴールを投げている間に走らせる）:
    python3 diag_turn.py 20        # 20 秒ぶん集めて表と符号一致数を出す
"""
import math, sys
import rclpy
from rclpy.node import Node
from rclpy.parameter import Parameter
from geometry_msgs.msg import PointStamped, Twist
from tf2_ros import Buffer, TransformListener


class Diag(Node):
    def __init__(self):
        super().__init__("diag_turn", parameter_overrides=[
            Parameter("use_sim_time", Parameter.Type.BOOL, False)])
        self.buf = Buffer(); self.lis = TransformListener(self.buf, self)
        self.rows = []
        self.cmd = (0.0, 0.0, 0.0)
        self.create_subscription(Twist, "/cmd_vel_nav", self._cmd, 20)
        self.create_subscription(PointStamped, "/lookahead_point", self._look, 20)

    def _cmd(self, m): self.cmd = (m.linear.x, m.linear.y, m.angular.z)

    def _look(self, m):
        try:
            tr = self.buf.lookup_transform("map", "base_link", rclpy.time.Time())
            q = tr.transform.rotation
            # ZYX の roll/pitch/yaw
            sr = 2*(q.w*q.x + q.y*q.z); cr = 1 - 2*(q.x*q.x + q.y*q.y)
            sp = max(-1.0, min(1.0, 2*(q.w*q.y - q.z*q.x)))
            sy = 2*(q.w*q.z + q.x*q.y); cy = 1 - 2*(q.y*q.y + q.z*q.z)
            rpy = (math.atan2(sr, cr), math.asin(sp), math.atan2(sy, cy))
        except Exception:
            rpy = None
        self.rows.append((m.header.frame_id, m.point.x, m.point.y, m.point.z,
                          self.cmd, rpy))


def main():
    rclpy.init()
    n = Diag()
    secs = float(sys.argv[1]) if len(sys.argv) > 1 else 20.0
    import time
    end = time.monotonic() + secs
    while rclpy.ok() and time.monotonic() < end:
        rclpy.spin_once(n, timeout_sec=0.2)
    if not n.rows:
        print("追従点が 1 度も来なかった（ゴールを投げていないか、controller が動いていない）")
        return 1
    agree = sum(1 for _, x, y, z, c, r in n.rows
                if (y >= 0) == (c[2] >= 0) or abs(y) < 1e-3)
    f = n.rows[0][0]
    print("追従点の frame_id: %s   （base_link ならこの比較が成立する）" % f)
    print(" %8s %8s %8s | %8s %8s | %7s %7s %7s" %
          ("carrot_x","carrot_y","carrot_z","cmd_vx","cmd_vyaw","roll°","pitch°","yaw°"))
    for fr, x, y, z, c, r in n.rows[:12]:
        d = ("%7.2f %7.2f %7.2f" % tuple(math.degrees(v) for v in r)) if r else "   -"
        print(" %8.3f %8.3f %8.3f | %8.3f %8.3f | %s" % (x, y, z, c[0], c[2], d))
    print("\n 符号一致 %d / %d" % (agree, len(n.rows)))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
