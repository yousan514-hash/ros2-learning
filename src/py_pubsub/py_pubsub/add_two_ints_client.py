"""サービスの頼み手: 引数の2つの整数を送って、答えを表示する。

例: ros2 run py_pubsub add_client 2 3
"""
import sys

import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class MinimalClientAsync(Node):
    def __init__(self):
        super().__init__("minimal_client_async")
        self.cli = self.create_client(AddTwoInts, "add_two_ints")
        for _ in range(10):
            if self.cli.wait_for_service(timeout_sec=1.0):
                return
            self.get_logger().info("service not available, waiting again...")
        raise RuntimeError("add_two_ints サービスが10秒待っても見つかりません。add_server を起動してください。")

    def send_request(self, a, b):
        req = AddTwoInts.Request()
        req.a, req.b = a, b
        return self.cli.call_async(req)


def main():
    rclpy.init()
    node = MinimalClientAsync()
    future = node.send_request(int(sys.argv[1]), int(sys.argv[2]))
    rclpy.spin_until_future_complete(node, future)
    node.get_logger().info(f"Result of add_two_ints: {sys.argv[1]} + {sys.argv[2]} = {future.result().sum}")
    node.destroy_node()
    rclpy.try_shutdown()


if __name__ == "__main__":
    main()
