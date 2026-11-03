"""アクションの練習（引き受ける側）: フィボナッチ数列を1つずつ計算し、途中経過を返す。

アクションは「時間のかかる仕事を頼み、途中経過（フィードバック）を受け取り、最後に結果をもらう」仕組み。
途中で取り消すこともできる。1回だけ答えをもらうサービスとの違いはそこ。
ロボットでは「この場所まで移動して」（Nav2 の NavigateToPose）のような仕事に使う。

    ros2 run py_pubsub fibonacci_server
"""
import time

import rclpy
from example_interfaces.action import Fibonacci
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node


class FibonacciServer(Node):
    def __init__(self):
        super().__init__("fibonacci_server")
        self.period = self.declare_parameter("period", 0.5).value
        # 計算の途中でも取り消しの依頼を受け取れるよう、コールバックを並行して動かせるグループにする
        self.server = ActionServer(
            self,
            Fibonacci,
            "fibonacci",
            self.execute,
            goal_callback=self.on_goal,
            cancel_callback=self.on_cancel,
            callback_group=ReentrantCallbackGroup(),
        )

    def on_goal(self, goal):
        if not 0 < goal.order <= 40:  # int32 に収まる範囲だけ引き受ける
            self.get_logger().info(f"reject goal: order={goal.order}")
            return GoalResponse.REJECT
        self.get_logger().info(f"accept goal: order={goal.order}")
        return GoalResponse.ACCEPT

    def on_cancel(self, goal_handle):
        self.get_logger().info("cancel requested")
        return CancelResponse.ACCEPT

    def execute(self, goal_handle):
        seq = [0, 1]
        for _ in range(1, goal_handle.request.order):
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                self.get_logger().info(f"canceled: {seq}")
                return Fibonacci.Result(sequence=seq)
            seq.append(seq[-1] + seq[-2])
            goal_handle.publish_feedback(Fibonacci.Feedback(sequence=seq))
            time.sleep(self.period)
        goal_handle.succeed()
        self.get_logger().info(f"succeeded: {seq}")
        return Fibonacci.Result(sequence=seq)


def main(args=None):
    rclpy.init(args=args)
    node = FibonacciServer()
    try:
        rclpy.spin(node, executor=MultiThreadedExecutor())
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
