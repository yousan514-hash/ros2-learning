"""セルの動きのノード。

計画（planner.py）どおりに、30Hz で /joint_states（両腕 12 軸 + 指 2 本）と部品の TF を出す。
部品の TF は、トレイにあるときは world、左手が持っているときは l_tcp を親にする（左手の先に付いて動く）。
カメラが面の正面に着いたら /inspection/capture（std_msgs/String、JSON）を出す。
RViz 用に、部品の形・いまの段取り・カメラの見える範囲を /cell/markers に出す。

パラメータ:
  speed   動きの速さの倍率（既定 1.0）
  loop    最後まで行ったら最初から繰り返す（既定 true）
  parts   検査する部品の名前（カンマ区切り。空なら全部）
"""
from __future__ import annotations

import json
import time

import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from tf2_ros import TransformBroadcaster
from visualization_msgs.msg import Marker, MarkerArray

from . import cell as C
from . import part_shapes
from . import planner as P
from . import ros_utils as RU

FINGER_JOINTS = ("l_finger_a_joint", "l_finger_b_joint")
ZINC = (0.62, 0.63, 0.65)


class CellNode(Node):
    def __init__(self):
        super().__init__("dual_arm_cell")
        self.declare_parameter("rate", 30.0)
        self.declare_parameter("speed", 1.0)
        self.declare_parameter("loop", True)
        self.declare_parameter("parts", "")
        self.declare_parameter("parts_file", "")
        self.speed = float(self.get_parameter("speed").value)
        self.loop = bool(self.get_parameter("loop").value)
        names = [n.strip() for n in str(self.get_parameter("parts").value).split(",") if n.strip()] or None
        parts, _ = C.load_parts(str(self.get_parameter("parts_file").value) or None)
        self.parts = {p.name: p for p in parts}

        t0 = time.monotonic()
        self.planner = P.plan_cycle(parts, names)
        self.timeline = P.Timeline(self.planner.segments)
        caps = sum(1 for s in self.planner.segments for e in s.events if e[0] == "capture")
        self.get_logger().info(
            f"plan: {len(self.planner.segments)} segments, {self.timeline.total:.1f} s per cycle, "
            f"{caps} shots, IK {self.planner.ik_found}/{self.planner.ik_tries} candidates solved "
            f"({time.monotonic() - t0:.1f} s to plan)")
        arms = self.planner.arms
        self.joint_names = arms["l"].geo.joint_names + arms["r"].geo.joint_names + list(FINGER_JOINTS)
        self.part_order = [p.name for p in self.planner.parts.values() if not names or p.name in names]
        self.meshes = {p.name: part_shapes.part_triangles(p.kind, p.dims) for p in parts}

        self.pub_js = self.create_publisher(JointState, "/joint_states", 10)
        self.pub_capture = self.create_publisher(String, "/inspection/capture", 10)
        self.pub_markers = self.create_publisher(MarkerArray, "/cell/markers", 10)
        self.tf = TransformBroadcaster(self)
        self.t_start = time.monotonic()
        self.prev_t = 0.0
        self.seq = 0
        self.label = ""
        self.create_timer(1.0 / float(self.get_parameter("rate").value), self.tick)
        self.create_timer(0.5, self.publish_markers)

    def tick(self):
        elapsed = (time.monotonic() - self.t_start) * self.speed
        total = self.timeline.total
        if self.loop:
            cycle, t = divmod(elapsed, total)
        else:
            cycle, t = 0, min(elapsed, total)
        state, seg = self.timeline.sample(t)
        events = self.timeline.events_between(self.prev_t, t) if (t != self.prev_t) else []
        self.prev_t = t
        self.label = seg.label
        stamp = self.get_clock().now().to_msg()

        js = JointState()
        js.header.stamp = stamp
        js.name = self.joint_names
        js.position = [float(v) for v in state.q["l"]] + [float(v) for v in state.q["r"]] + [state.grip] * 2
        self.pub_js.publish(js)

        tfs = []
        for name, part in self.parts.items():
            if state.held == name:
                tfs.append(RU.transform(part.tool_to_part(state.variant), "l_tcp", f"part_{name}", stamp))
            else:
                tfs.append(RU.transform(part.tray_pose(), "world", f"part_{name}", stamp))
        self.tf.sendTransform(tfs)

        for _, kind, data in events:
            if kind != "capture":
                continue
            self.seq += 1
            msg = {"seq": self.seq, "cycle": int(cycle), **data}
            self.pub_capture.publish(String(data=json.dumps(msg)))
            self.get_logger().info(f"capture #{self.seq}: {data['part']}/{data['face']} ({data['where']})")

    def publish_markers(self):
        stamp = self.get_clock().now().to_msg()
        arr = MarkerArray()
        for i, (name, tris) in enumerate(self.meshes.items()):
            arr.markers.append(RU.mesh_marker("parts", i, tris, f"part_{name}", stamp, ZINC))
        current = next((n for n in self.part_order if n in self.label), "")  # 段取りの名前に部品名が入っている
        idx = self.part_order.index(current) + 1 if current else 0
        text = f"part{idx}/{len(self.part_order)}:{current}\n{self.label}" if current else self.label
        arr.markers.append(RU.text_marker("status", 0, text, "world", stamp, (0.0, -0.50, 1.62), 0.03,
                                          (0.95, 0.95, 0.95)))
        # カメラの見える範囲（面から 0.15m の所で 5cm x 3.75cm）
        z, hx, hy = C.STANDOFF, 0.025, 0.01875
        corners = [(hx, hy, z), (-hx, hy, z), (-hx, -hy, z), (hx, -hy, z)]
        segs = [((0.0, 0.0, 0.0), c) for c in corners] + [(corners[k], corners[(k + 1) % 4]) for k in range(4)]
        arr.markers.append(RU.lines_marker("camera", 0, segs, "r_camera_optical_frame", stamp,
                                           (0.35, 0.65, 1.0), 0.0015, 0.9))
        # 見せる点（左手の TCP がここに来る）
        m = RU.marker("inspect_point", 0, Marker.SPHERE, "world", stamp, (1.0, 1.0, 1.0), 0.25, (0.012,) * 3)
        m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in C.INSPECT_POINT)
        arr.markers.append(m)
        self.pub_markers.publish(arr)


def main(args=None):
    rclpy.init(args=args)
    node = CellNode()
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
