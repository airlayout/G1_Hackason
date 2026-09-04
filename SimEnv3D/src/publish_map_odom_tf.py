"""map -> odom を恒等変換として配信し、AMCL の代わりをする。

Isaac Sim は真値の姿勢を持っているため、自己位置推定は本来不要。
odom が真値なら map と odom は一致するので、恒等変換を流せばよい。

AMCL を使わない理由（実測）:
    歩行中に AMCL の推定が真値からどんどんずれた。
        真値 (-2.83, 3.95) -> AMCL (-2.14, 3.72)  ずれ 0.7 m
        真値 (-5.39, 5.47) -> AMCL (-2.39, 7.57)  ずれ 3.6 m
    真値は X が減る方向へ進んでいるのに AMCL は X が増えると推定し、
    進行方向すら逆に取っていた。Warehouse は同じ形の棚が並び 2D スキャン
    では特徴が乏しいため、パーティクルが誤った位置に収束したとみられる。
    その結果 Nav2 が誤った自己位置で経路を引き直し、G1 は少し歩いては
    旋回を繰り返して目標に到達できなかった。

実機では真値が無いので AMCL が必要になる。その場合はこのスクリプトを
使わず、nav2.yaml の amcl 設定を使うこと。

実行方法:
    source env.sh && python3 src/publish_map_odom_tf.py
"""

from __future__ import annotations

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import TransformBroadcaster

# 配信周期 [Hz]。TF は多少速くても負荷にならない。
PUBLISH_HZ: float = 20.0


class MapOdomIdentity(Node):
    """map -> odom を恒等変換として配信するノード。"""

    def __init__(self) -> None:
        super().__init__("map_odom_identity")
        # Isaac Sim が /clock を配信しているので、それに合わせる
        self.set_parameters(
            [rclpy.parameter.Parameter("use_sim_time", value=True)]
        )
        self._broadcaster = TransformBroadcaster(self)
        self.create_timer(1.0 / PUBLISH_HZ, self._publish)
        self._count = 0
        print("[TF] map -> odom を恒等変換で配信します（AMCL の代わり）")

    def _publish(self) -> None:
        """恒等変換を配信する。"""
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = "map"
        tf.child_frame_id = "odom"
        # 並進・回転ともゼロ（odom がそのまま map 座標になる）
        tf.transform.rotation.w = 1.0
        self._broadcaster.sendTransform(tf)

        self._count += 1
        if self._count % (int(PUBLISH_HZ) * 30) == 0:
            print(f"[TF] 配信中（{self._count} 回）")


def main() -> None:
    """恒等変換を配信し続ける。"""
    rclpy.init()
    node = MapOdomIdentity()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
