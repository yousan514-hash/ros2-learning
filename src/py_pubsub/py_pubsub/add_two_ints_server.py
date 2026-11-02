"""サービスの受け手: 2つの整数を受け取って足し算の答えを返す（公式チュートリアル）。"""
import rclpy
from example_interfaces.srv import AddTwoInts
from rclpy.node import Node


class MinimalService(Node):
    def __init__(self):
        super().__init__("minimal_service")
        self.srv = self.create_service(AddTwoInts, "add_two_ints", self.add_two_ints_callback)

    def add_two_ints_callback(self, request, response):
        response.sum = request.a + request.b
        self.get_logger().info(f"Incoming request: a={request.a} b={request.b}")
        return response


def main(args=None):
    rclpy.init(args=args)
    node = MinimalService()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
