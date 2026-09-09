"""NavigateToPose にゴールを送り、/cmd_vel が出るかを見る(足は繋がない)。"""
import sys, time
import rclpy
from geometry_msgs.msg import PoseStamped, Twist
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node

gx, gy = float(sys.argv[1]), float(sys.argv[2])
rclpy.init(); n = Node("send_goal_watch")
cmds, smoothed = [], []
n.create_subscription(Twist, "/cmd_vel", lambda m: cmds.append((m.linear.x, m.linear.y, m.angular.z)), 10)
n.create_subscription(Twist, "/cmd_vel_smoothed", lambda m: smoothed.append((m.linear.x, m.linear.y, m.angular.z)), 10)

ac = ActionClient(n, NavigateToPose, "navigate_to_pose")
if not ac.wait_for_server(timeout_sec=30.0):
    print("[NG] navigate_to_pose が現れない"); raise SystemExit(1)

g = NavigateToPose.Goal()
g.pose.header.frame_id = "map"
g.pose.header.stamp = n.get_clock().now().to_msg()
g.pose.pose.position.x = gx
g.pose.pose.position.y = gy
g.pose.pose.orientation.w = 1.0
print(f"[send] Goal (map) = ({gx}, {gy})")

fut = ac.send_goal_async(g)
rclpy.spin_until_future_complete(n, fut, timeout_sec=20.0)
h = fut.result()
if h is None or not h.accepted:
    print("[NG] ゴールが受理されなかった"); raise SystemExit(1)
print("[ok] ゴールが受理された。40秒観察する")

res = h.get_result_async()
t0 = time.time()
while rclpy.ok() and time.time() - t0 < 40 and not res.done():
    rclpy.spin_once(n, timeout_sec=0.5)

print(f"\n=== /cmd_vel: {len(cmds)} 件 ===")
if cmds:
    nz = [c for c in cmds if abs(c[0]) > 1e-4 or abs(c[2]) > 1e-4]
    print(f"  非ゼロ {len(nz)} 件 / 最初 {cmds[0]} / 最後 {cmds[-1]}")
    print(f"  vx 最大 {max(abs(c[0]) for c in cmds):.3f}  wz 最大 {max(abs(c[2]) for c in cmds):.3f}")
print(f"=== /cmd_vel_smoothed: {len(smoothed)} 件 ===")
if smoothed:
    print(f"  最後 {smoothed[-1]}")
if res.done() and res.result() is not None:
    print(f"=== アクション結果 status={res.result().status} (4=SUCCEEDED, 6=ABORTED) ===")
else:
    print("=== 40秒でまだ実行中(タイムアウトせず継続) ===")
