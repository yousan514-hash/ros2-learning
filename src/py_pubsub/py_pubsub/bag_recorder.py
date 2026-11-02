"""rosbag2 の練習: /topic の文字列を、決まった数だけ記録して、きちんと閉じて終わる。

例: ros2 run py_pubsub bag_recorder --ros-args -p uri:=bags/talk -p count:=10
あとで ros2 bag info bags/talk / ros2 bag play bags/talk で確かめる。
"""
import rclpy
import rosbag2_py
from rclpy.node import Node
from rclpy.serialization import serialize_message
from std_msgs.msg import String


class BagRecorder(Node):
    def __init__(self):
        super().__init__("bag_recorder")
        self.declare_parameter("uri", "bags/talk")
        self.declare_parameter("count", 10)
        self.limit = self.get_parameter("count").value
        self.written = 0

        self.writer = rosbag2_py.SequentialWriter()
        self.writer.open(
            rosbag2_py.StorageOptions(uri=self.get_parameter("uri").value, storage_id="mcap"),
            rosbag2_py.ConverterOptions(input_serialization_format="cdr", output_serialization_format="cdr"),
        )
        self.writer.create_topic(rosbag2_py.TopicMetadata(
            id=0, name="topic", type="std_msgs/msg/String", serialization_format="cdr"))
        self.subscription = self.create_subscription(String, "topic", self.on_message, 10)

    def on_message(self, msg):
        if self.written >= self.limit:
            return
        self.writer.write("topic", serialize_message(msg), self.get_clock().now().nanoseconds)
        self.written += 1
        self.get_logger().info(f"recorded {self.written}/{self.limit}: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = BagRecorder()
    try:
        while rclpy.ok() and node.written < node.limit:
            rclpy.spin_once(node, timeout_sec=0.5)
    except KeyboardInterrupt:
        pass
    finally:
        del node.writer  # ここでファイルが閉じられ、metadata.yaml が書かれる
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
