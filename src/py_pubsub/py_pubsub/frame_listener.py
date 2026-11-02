"""tf2 の練習（受け取る側）: カメラが世界のどこにあるかを、world → robot → camera をたどって求める。

ロボットが半径 1m の円の接線を向いて進み、カメラが前 0.2m にあるなら、
カメラの原点からの水平距離は常に sqrt(1^2 + 0.2^2) ≒ 1.020m になる。
"""
import math

import rclpy
from rclpy.node import Node
from rclpy.time import Time
from tf2_ros import Buffer, TransformException, TransformListener


class FrameListener(Node):
    def __init__(self):
        super().__init__("frame_listener")
        self.buffer = Buffer()
        self.listener = TransformListener(self.buffer, self)
        self.timer = self.create_timer(0.5, self.on_timer)

    def on_timer(self):
        try:
            t = self.buffer.lookup_transform("world", "camera", Time())
        except TransformException as e:
            self.get_logger().info(f"waiting for transform: {e}")
            return
        p = t.transform.translation
        dist = math.hypot(p.x, p.y)
        self.get_logger().info(f"camera in world: x={p.x:+.3f} y={p.y:+.3f} z={p.z:+.3f} dist={dist:.3f}")


def main(args=None):
    rclpy.init(args=args)
    node = FrameListener()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
