"""疑似カメラのノード（右手首のカメラの代わり）。

/inspection/capture（撮る合図）を受けるたびに、その部品の面の画像を imaging.py で作り、
  /wrist_camera/image_raw  sensor_msgs/Image（rgb8、frame_id = r_camera_optical_frame）
  /inspection/shot         std_msgs/String（JSON: 部品・面・時刻、評価用の正解）
を同じ時刻（header.stamp）で出す。正解は判定には使わず、当たり外れを数えるためだけに使う。

パラメータ:
  seed      乱数の種（同じ種なら同じ画像の並びになる）
  p_defect  欠陥を入れる確率（既定 0.3）
  p_ood     範囲外の照明にする確率（既定 0.05）
"""
from __future__ import annotations

import json

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from . import cell as C
from . import imaging
from . import ros_utils as RU


class CameraNode(Node):
    def __init__(self):
        super().__init__("wrist_camera_sim")
        self.declare_parameter("seed", 1)
        self.declare_parameter("p_defect", 0.3)
        self.declare_parameter("p_ood", 0.05)
        self.declare_parameter("parts_file", "")
        self.rng = np.random.default_rng(int(self.get_parameter("seed").value))
        self.p_defect = float(self.get_parameter("p_defect").value)
        self.p_ood = float(self.get_parameter("p_ood").value)
        parts, _ = C.load_parts(str(self.get_parameter("parts_file").value) or None)
        self.parts = {p.name: p for p in parts}
        self.pub_img = self.create_publisher(Image, "/wrist_camera/image_raw", 5)
        self.pub_shot = self.create_publisher(String, "/inspection/shot", 10)
        self.create_subscription(String, "/inspection/capture", self.on_capture, 10)
        self.get_logger().info(f"ready: {imaging.W}x{imaging.H} rgb8, p_defect={self.p_defect}, p_ood={self.p_ood}")

    def on_capture(self, msg: String):
        info = json.loads(msg.data)
        part = self.parts[info["part"]]
        img, truth = imaging.random_shot(self.rng, part, info["face_type"], self.p_defect, self.p_ood)
        stamp = self.get_clock().now().to_msg()
        shot = {**info, "stamp": [stamp.sec, stamp.nanosec], "truth": truth}
        self.pub_shot.publish(String(data=json.dumps(shot)))
        self.pub_img.publish(RU.image_to_msg(img, stamp, "r_camera_optical_frame"))
        kinds = ",".join(d["type"] for d in truth["defects"]) or "-"
        self.get_logger().info(f"shot #{info['seq']}: {part.name}/{info['face']} (truth: defects={kinds}, "
                               f"out_of_range={truth['ood']})")


def main(args=None):
    rclpy.init(args=args)
    node = CameraNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
