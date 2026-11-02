"""パラメータの練習: 送る言葉と間隔を、外から変えられる talker。

例:
    ros2 run py_pubsub param_talker --ros-args -p greeting:=こんにちは -p period:=1.0
    ros2 param set /param_talker greeting "Hi"   # 動いている途中でも変えられる
"""
import rclpy
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from std_msgs.msg import String


class ParamTalker(Node):
    def __init__(self):
        super().__init__("param_talker")
        self.declare_parameter("greeting", "Hello")
        self.declare_parameter("period", 0.5)
        self.publisher_ = self.create_publisher(String, "topic", 10)
        self.timer = self.create_timer(self.get_parameter("period").value, self.timer_callback)
        self.add_on_set_parameters_callback(self.on_params)
        self.i = 0

    def on_params(self, params):
        for p in params:
            if p.name == "period":
                if p.value <= 0:
                    return SetParametersResult(successful=False, reason="period must be greater than 0")
                self.timer.cancel()
                self.timer = self.create_timer(p.value, self.timer_callback)
        return SetParametersResult(successful=True)

    def timer_callback(self):
        msg = String()
        msg.data = f"{self.get_parameter('greeting').value}: {self.i}"
        self.publisher_.publish(msg)
        self.i += 1


def main(args=None):
    rclpy.init(args=args)
    node = ParamTalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
