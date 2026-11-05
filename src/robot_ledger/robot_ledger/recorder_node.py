"""ロボットの稼働を台帳に記録するノード（turtlesim の亀を「働くロボット」に見立てる）。

亀が決めた点（仕事の場所）に着いて止まるたびに、「仕事が終わった」ことと誤差を、署名つきで台帳に書き足す。
亀を動かす turtle_draw とは別のノードにして、メーターのように外から測る。

    ros2 run robot_ledger ledger_recorder --ros-args -p ledger:=ledger/turtle1.jsonl -p key:=ledger/turtle1.key
"""
import math

import rclpy
from rclpy.node import Node
from turtlesim.msg import Pose

from robot_ledger.ledger import Ledger, RobotIdentity

# turtle_draw の「F」と同じ点。0 番目は始点なので、仕事は 1 番目から
LETTER_F = [3.0, 2.0, 3.0, 9.0, 7.5, 9.0, 3.0, 9.0, 3.0, 5.5, 6.5, 5.5]


class LedgerRecorder(Node):
    def __init__(self):
        super().__init__("ledger_recorder")
        flat = self.declare_parameter("points", LETTER_F).value
        self.points = list(zip(flat[0::2], flat[1::2]))
        self.near = self.declare_parameter("near", 0.2).value  # この距離の中で止まったら「着いた」とみなす
        self.path = self.declare_parameter("ledger", "ledger/turtle1.jsonl").value
        key = self.declare_parameter("key", "ledger/turtle1.key").value
        self.identity = RobotIdentity.load_or_create(key)
        self.ledger = Ledger.load(self.path)  # 前の記録があれば続きから書く
        problems = self.ledger.verify(trusted_public_key=self.identity.public_key_hex)
        if problems:
            raise RuntimeError(f"existing ledger is broken: {problems[0]}")
        self.target = 1
        self.distance = 0.0
        self.last = None
        self.done = False
        self.record("session_start", {"points": len(self.points) - 1})
        self.create_subscription(Pose, "turtle1/pose", self.on_pose, 10)
        self.get_logger().info(f"robot id {self.identity.robot_id}, ledger {self.path}")

    def record(self, event, data):
        e = self.ledger.append(self.identity, event, data)
        Ledger.append_line(self.path, e)
        self.get_logger().info(f"seq {e.seq} {event} {data} hash {e.hash[:12]}")

    def on_pose(self, p):
        if self.done:
            return
        if self.last is not None:
            step = math.hypot(p.x - self.last.x, p.y - self.last.y)
            if step < 1.0:  # 瞬間移動（teleport）は走った距離に数えない
                self.distance += step
        self.last = p
        tx, ty = self.points[self.target]
        err = math.hypot(tx - p.x, ty - p.y)
        # 点の近くで、前に進むのをやめた（向きを変え始めた・止まった）＝その点の仕事が終わった
        if err < self.near and p.linear_velocity == 0.0:
            self.record("task_done", {"point": self.target, "x": round(p.x, 3), "y": round(p.y, 3), "error": round(err, 3)})
            self.target += 1
            if self.target == len(self.points):
                self.record("session_end", {"tasks": self.target - 1, "distance": round(self.distance, 2)})
                self.done = True


def main(args=None):
    rclpy.init(args=args)
    node = LedgerRecorder()
    try:
        while rclpy.ok() and not node.done:
            rclpy.spin_once(node, timeout_sec=0.1)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == "__main__":
    main()
