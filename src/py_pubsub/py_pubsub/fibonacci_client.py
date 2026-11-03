"""アクションの練習（頼む側）: フィボナッチ数列を頼み、途中経過を表示し、結果を受け取る。

    ros2 run py_pubsub fibonacci_client --ros-args -p order:=10
    ros2 run py_pubsub fibonacci_client --ros-args -p order:=10 -p cancel_after:=1.2   # 途中で取り消す
"""
import rclpy
from action_msgs.msg import GoalStatus
from example_interfaces.action import Fibonacci
from rclpy.action import ActionClient
from rclpy.node import Node

STATUS = {
    GoalStatus.STATUS_SUCCEEDED: "succeeded",
    GoalStatus.STATUS_CANCELED: "canceled",
    GoalStatus.STATUS_ABORTED: "aborted",
}


class FibonacciClient(Node):
    def __init__(self):
        super().__init__("fibonacci_client")
        self.order = self.declare_parameter("order", 10).value
        self.cancel_after = self.declare_parameter("cancel_after", 0.0).value
        self.client = ActionClient(self, Fibonacci, "fibonacci")
        self.goal_handle = None

    def run(self):
        if not self.client.wait_for_server(timeout_sec=10.0):
            raise RuntimeError("fibonacci action server not available")
        self.get_logger().info(f"goal: order={self.order}")
        future = self.client.send_goal_async(Fibonacci.Goal(order=self.order), feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, future)
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            self.get_logger().info("goal rejected")
            return
        if self.cancel_after > 0:
            self.cancel_timer = self.create_timer(self.cancel_after, self.cancel)
        result_future = self.goal_handle.get_result_async()
        rclpy.spin_until_future_complete(self, result_future)
        res = result_future.result()
        status = STATUS.get(res.status, str(res.status))
        self.get_logger().info(f"result: {status} {list(res.result.sequence)}")

    def on_feedback(self, msg):
        self.get_logger().info(f"feedback: {list(msg.feedback.sequence)}")

    def cancel(self):
        self.cancel_timer.cancel()  # 1回だけ
        self.get_logger().info("cancel requested")
        self.goal_handle.cancel_goal_async()


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciClient()
    try:
        node.run()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
