"""turtlesim の亀に、決めた点を順にたどらせて線を描く（トピック・サービス・パラメータのまとめ）。

- サービス: 亀を始点に瞬間移動させ、画面を消す（teleport_absolute、clear）
- トピック: 亀の位置（pose）を受け取り、速度（cmd_vel）を送る
- 制御: 目標の点との距離と向きのずれに比例して、進む速さと回る速さを決める（P 制御）

    ros2 run py_pubsub turtle_draw       # 「F」を描く
    ros2 run py_pubsub turtle_draw --ros-args -p points:="[2.0, 2.0, 9.0, 2.0, 5.5, 8.0, 2.0, 2.0]"
"""
import math

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_srvs.srv import Empty
from turtlesim.msg import Pose
from turtlesim.srv import TeleportAbsolute

# 「F」の字。x, y の順に並べる（turtlesim の画面は 0〜11 の正方形）
LETTER_F = [3.0, 2.0, 3.0, 9.0, 7.5, 9.0, 3.0, 9.0, 3.0, 5.5, 6.5, 5.5]


class TurtleDraw(Node):
    def __init__(self):
        super().__init__("turtle_draw")
        flat = self.declare_parameter("points", LETTER_F).value
        self.points = list(zip(flat[0::2], flat[1::2]))
        self.tolerance = self.declare_parameter("tolerance", 0.05).value
        self.pose = None
        self.at_start = False
        self.target = 1  # 0 番目は始点（瞬間移動する）
        self.done = False
        self.create_subscription(Pose, "turtle1/pose", self.on_pose, 10)
        self.cmd = self.create_publisher(Twist, "turtle1/cmd_vel", 10)

    def call(self, srv_type, name, request):
        client = self.create_client(srv_type, name)
        if not client.wait_for_service(timeout_sec=10.0):
            raise RuntimeError(f"service {name} not available")
        future = client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def start(self):
        x, y = self.points[0]
        self.call(TeleportAbsolute, "turtle1/teleport_absolute", TeleportAbsolute.Request(x=x, y=y, theta=0.0))
        self.call(Empty, "clear", Empty.Request())
        self.timer = self.create_timer(0.02, self.step)

    def on_pose(self, msg):
        self.pose = msg

    def step(self):
        if self.pose is None or self.done:
            return
        if not self.at_start:
            # 瞬間移動する前の位置が届くことがあるので、始点に着いたのを確かめてから動かす
            sx, sy = self.points[0]
            self.at_start = math.hypot(self.pose.x - sx, self.pose.y - sy) < 0.1
            return
        tx, ty = self.points[self.target]
        dx, dy = tx - self.pose.x, ty - self.pose.y
        dist = math.hypot(dx, dy)
        if dist < self.tolerance:
            self.get_logger().info(f"point {self.target}: ({tx:.1f}, {ty:.1f}) error={dist:.3f}")
            self.target += 1
            if self.target == len(self.points):
                self.cmd.publish(Twist())  # 止まる
                self.done = True
            return
        err = math.atan2(dy, dx) - self.pose.theta
        err = math.atan2(math.sin(err), math.cos(err))  # -π〜π に直す
        twist = Twist()
        twist.angular.z = 6.0 * err
        # 向きが合うまでは進まない（曲がりながら進むと線がふくらむ）
        twist.linear.x = min(2.0 * dist, 3.0) if abs(err) < 0.1 else 0.0
        self.cmd.publish(twist)


def main(args=None):
    rclpy.init(args=args)
    node = TurtleDraw()
    try:
        node.start()
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
