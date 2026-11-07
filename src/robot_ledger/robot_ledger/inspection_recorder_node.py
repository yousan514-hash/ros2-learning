"""検査の結果を、検査セルの鍵で署名して台帳に書くノード（検査セルのメーター）。

双腕の検査セル（dual_arm_inspection）の /inspection/result を受けて、1件ずつ台帳に書き足す。
/inspection/shot と /wrist_camera/image_raw を時刻で組にして、画像の SHA-256（sensor_msgs/Image の data、
生の画素）を一緒に入れる。正解（truth）はシミュレーションだけの情報なので台帳に入れない。
台帳の先頭の行には、どの使用範囲書で働くかの指紋を書く（使用範囲書は <台帳>.scope.json に保存）。

    ros2 run robot_ledger inspection_recorder --ros-args -p ledger:=ledger/cell.jsonl -p key:=ledger/cell.key

請求は、工場が受け取りの署名をしてから（tools/inspection_ledger_demo.py と同じ make_inspection_bill）。
"""
import json
from pathlib import Path

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String

from robot_ledger.billing import LedgerInvalid
from robot_ledger.inspection_ledger import (
    EV_SCOPE, UsageScope, inspection_record, record_inspection, start_cell_ledger,
)
from robot_ledger.ledger import Ledger, RobotIdentity
from robot_ledger.shot_matcher import ShotMatcher


class InspectionRecorder(Node):
    def __init__(self):
        super().__init__("inspection_recorder")
        self.path = Path(self.declare_parameter("ledger", "ledger/inspection_cell.jsonl").value)
        key = self.declare_parameter("key", "ledger/inspection_cell.key").value
        # 疑似カメラの検査ノードは、決まった乱数の種で良品から学ぶ。同じ種なら同じしきい値になるので、種を版にする
        thresholds = self.declare_parameter("thresholds", "calib_seed:20261106").value
        self.identity = RobotIdentity.load_or_create(key)
        self.scope = UsageScope(
            self.identity.robot_id, thresholds,
            price_per_inspection=int(self.declare_parameter("price", 100).value),
            refund_per_miss=int(self.declare_parameter("refund_miss", 1000).value),
            refund_per_overkill=int(self.declare_parameter("refund_overkill", 100).value),
            name="双腕の外観検査セル / ROS 2")
        self.ledger = Ledger.load(self.path)
        if self.ledger.entries:  # 前の記録の続きから書く。壊れていたり、使用範囲書が違ったりしたら止める
            problems = self.ledger.verify(trusted_public_key=self.identity.public_key_hex)
            first = self.ledger.entries[0]
            if not problems and (first.event != EV_SCOPE or first.data.get("scope") != self.scope.fingerprint()):
                problems = ["the first line has another usage scope (start a new ledger file)"]
            if problems:
                raise LedgerInvalid(problems)
        else:
            self.ledger = start_cell_ledger(self.identity, self.scope)
            Ledger.append_line(self.path, self.ledger.entries[0])
        scope_file = self.path.with_suffix(".scope.json")
        scope_file.write_text(json.dumps(self.scope.to_dict(), ensure_ascii=False, indent=1), encoding="utf-8")
        self.matcher = ShotMatcher()
        self.create_subscription(String, "/inspection/shot", self.on_shot, 10)
        self.create_subscription(Image, "/wrist_camera/image_raw", self.on_image, 5)
        self.create_subscription(String, "/inspection/result", self.on_result, 10)
        self.get_logger().info(f"cell {self.identity.robot_id}, ledger {self.path} ({len(self.ledger.entries)} lines), "
                               f"scope {self.scope.fingerprint()[:12]}")

    def on_shot(self, msg: String):
        self.write(self.matcher.add_shot(json.loads(msg.data)))

    def on_image(self, msg: Image):
        self.write(self.matcher.add_image((msg.header.stamp.sec, msg.header.stamp.nanosec), msg.data))

    def on_result(self, msg: String):
        self.write(self.matcher.add_result(json.loads(msg.data)))

    def write(self, ready):
        for res, sha in ready:
            rec = inspection_record(res["part"], res["face"], res["verdict"], res["score"], res["reason"],
                                    self.scope.thresholds, sha, defect_type=res.get("defect_type"), shot=res["seq"])
            e = record_inspection(self.ledger, self.identity, rec)
            Ledger.append_line(self.path, e)
            self.get_logger().info(f"seq {e.seq} shot #{res['seq']} {res['verdict']} "
                                   f"({'in range' if rec['in_range'] else 'OUT OF RANGE'}) hash {e.hash[:12]}")


def main(args=None):
    rclpy.init(args=args)
    node = InspectionRecorder()
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
