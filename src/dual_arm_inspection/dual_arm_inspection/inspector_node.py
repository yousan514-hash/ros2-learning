"""検査のノード。

起動するときに、良品の画像だけで「ふつうの見え方」としきい値を学ぶ（inspection.Inspector.calibrate）。
そのあと /wrist_camera/image_raw と /inspection/shot を時刻で組にして判定し、3 段で出す:
  OK（自動で通す）/ NG（自動で弾く）/ HOLD = 保留（自信がないので人に回す。範囲外のときも）

出すもの:
  /inspection/result           std_msgs/String（JSON）
  /inspection/image_annotated  sensor_msgs/Image（判定の帯と欠陥の候補の四角を描いた画像）
  /inspection/markers          MarkerArray（RViz に色つきの文字: OK 緑・NG 赤・保留 黄、集計）
"""
from __future__ import annotations

import json
import time

from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import String
from visualization_msgs.msg import Marker, MarkerArray

from . import cell as C
from . import imaging
from . import inspection as I
from . import ros_utils as RU

TEXT_AT = (0.25, -0.50, 1.35)  # 腕と重ならないよう、右腕のさらに右の上に出す


class InspectorNode(Node):
    def __init__(self):
        super().__init__("inspector")
        self.declare_parameter("calib_seed", 20261106)
        self.declare_parameter("parts_file", "")
        parts, _ = C.load_parts(str(self.get_parameter("parts_file").value) or None)
        self.inspector = I.Inspector(parts)
        t0 = time.monotonic()
        self.get_logger().info("learning from good images only ...")
        cal = self.inspector.calibrate(seed=int(self.get_parameter("calib_seed").value))
        self.get_logger().info(
            f"learned {cal['templates']} templates from {cal['good_images']} held-out good images in "
            f"{time.monotonic() - t0:.1f} s: t_ok={cal['t_ok']:.1f}, t_ng={cal['t_ng']:.1f}")
        self.pending: dict[tuple[int, int], dict] = {}
        self.table = {t: {v: 0 for v in ("OK", "NG", "HOLD")} for t in ("good", "defect", "out_of_range")}
        self.pub_result = self.create_publisher(String, "/inspection/result", 10)
        self.pub_img = self.create_publisher(Image, "/inspection/image_annotated", 5)
        self.pub_markers = self.create_publisher(MarkerArray, "/inspection/markers", 10)
        self.create_subscription(String, "/inspection/shot", self.on_shot, 10)
        self.create_subscription(Image, "/wrist_camera/image_raw", self.on_image, 5)

    # 画像と情報は別のトピックで来るので、時刻で組にする（どちらが先に来てもよい）
    def on_shot(self, msg: String):
        shot = json.loads(msg.data)
        self._add(tuple(shot["stamp"]), "shot", shot)

    def on_image(self, msg: Image):
        self._add((msg.header.stamp.sec, msg.header.stamp.nanosec), "image", msg)

    def _add(self, key, kind, value):
        entry = self.pending.setdefault(key, {})
        entry[kind] = value
        if "shot" in entry and "image" in entry:
            del self.pending[key]
            self.judge(entry["shot"], entry["image"])
        while len(self.pending) > 20:
            self.pending.pop(next(iter(self.pending)))

    def judge(self, shot: dict, img_msg: Image):
        img = RU.msg_to_image(img_msg)
        res = self.inspector.inspect(img, shot["part"], shot["face_type"])
        truth = imaging.truth_class(shot["truth"])
        self.table[truth][res.verdict] += 1
        out = {"seq": shot["seq"], "part": shot["part"], "face": shot["face"], "verdict": res.verdict,
               "score": round(res.score, 2), "reason": res.reason, "defect_type": res.defect_type,
               "boxes": res.boxes, "truth": truth}
        self.pub_result.publish(String(data=json.dumps(out)))
        title = f"#{shot['seq']} {shot['part']}/{shot['face']}"
        self.pub_img.publish(RU.image_to_msg(I.annotate(img, res, title), img_msg.header.stamp, img_msg.header.frame_id))
        self.publish_markers(shot, res)
        self.get_logger().info(f"#{shot['seq']} {shot['part']}/{shot['face']}: {res.verdict} "
                               f"({res.defect_type or res.reason}, score {res.score:.1f}) [truth: {truth}]")

    def publish_markers(self, shot: dict, res: I.Result):
        stamp = self.get_clock().now().to_msg()
        rgb = RU.rgb255(I.COLORS[res.verdict])
        label = {"OK": "OK", "NG": "NG", "HOLD": "HOLD->human"}[res.verdict]
        detail = res.defect_type or ("" if res.verdict == "OK" else res.reason)
        head = f"{label}:{detail}" if detail else label
        arr = MarkerArray()
        arr.markers.append(RU.text_marker(
            "result", 0, f"{head}\n{shot['part']}/{shot['face']}\nscore={res.score:.0f}",
            "world", stamp, TEXT_AT, 0.035, rgb))
        t = self.table
        n = sum(sum(v.values()) for v in t.values())
        ok, ng, hold = (sum(t[k][v] for k in t) for v in ("OK", "NG", "HOLD"))
        tally = f"n={n}|OK={ok}|NG={ng}|HOLD={hold}\nvs_truth:miss={t['defect']['OK']}|overkill={t['good']['NG']}"
        arr.markers.append(RU.text_marker("tally", 0, tally, "world", stamp,
                                          (TEXT_AT[0], TEXT_AT[1], TEXT_AT[2] + 0.12), 0.025, (0.95, 0.95, 0.95)))
        m = RU.marker("highlight", 0, Marker.SPHERE, f"part_{shot['part']}", stamp, rgb, 0.45, (0.05,) * 3,
                      lifetime=3.0, frame_locked=True)
        arr.markers.append(m)
        self.pub_markers.publish(arr)


def main(args=None):
    RU.spin(InspectorNode, args)


if __name__ == "__main__":
    main()
