"""tf2 の練習（送る側）: ロボットが円を描いて動き、その上にカメラが付いている、という位置関係を送る。

- world → robot: 半径 1m の円を 10 秒で 1 周。進む向き（円の接線）を向く
- robot → camera: ロボットの前 0.2m、上 0.1m に固定（静的な変換）
"""
import math

import rclpy
from geometry_msgs.msg import TransformStamped
from rclpy.node import Node
from tf2_ros import StaticTransformBroadcaster, TransformBroadcaster


def yaw_to_quaternion(yaw):
    """z 軸まわりの回転（yaw）だけのクォータニオン (x, y, z, w)。"""
    return 0.0, 0.0, math.sin(yaw / 2), math.cos(yaw / 2)


class FrameBroadcaster(Node):
    def __init__(self):
        super().__init__("frame_broadcaster")
        self.declare_parameter("radius", 1.0)
        self.declare_parameter("period", 10.0)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.static_broadcaster = StaticTransformBroadcaster(self)
        self.publish_camera_mount()
        self.timer = self.create_timer(0.05, self.publish_robot_pose)

    def publish_camera_mount(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id = "robot"
        t.child_frame_id = "camera"
        t.transform.translation.x = 0.2
        t.transform.translation.z = 0.1
        t.transform.rotation.w = 1.0
        self.static_broadcaster.sendTransform(t)

    def publish_robot_pose(self):
        r = self.get_parameter("radius").value
        period = self.get_parameter("period").value
        now = self.get_clock().now()
        angle = 2 * math.pi * (now.nanoseconds / 1e9) / period
        t = TransformStamped()
        t.header.stamp = now.to_msg()
        t.header.frame_id = "world"
        t.child_frame_id = "robot"
        t.transform.translation.x = r * math.cos(angle)
        t.transform.translation.y = r * math.sin(angle)
        # 反時計回りに進むので、向きは位置の角度 + 90 度
        qx, qy, qz, qw = yaw_to_quaternion(angle + math.pi / 2)
        t.transform.rotation.x, t.transform.rotation.y = qx, qy
        t.transform.rotation.z, t.transform.rotation.w = qz, qw
        self.tf_broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = FrameBroadcaster()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
