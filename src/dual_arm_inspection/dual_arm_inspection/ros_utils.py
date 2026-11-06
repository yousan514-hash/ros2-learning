"""ROS のメッセージを作る小さな道具（画像の詰め方・Marker）。

cv_bridge は入っていないので、sensor_msgs/Image の rgb8 を自分で詰める:
  height, width, encoding="rgb8", is_bigendian=0, step=width*3, data=行ごとに R,G,B の並び
"""
from __future__ import annotations

import array

import numpy as np
from builtin_interfaces.msg import Duration
from geometry_msgs.msg import Point, Pose, TransformStamped
from sensor_msgs.msg import Image
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker

from .geometry import quat_xyzw


def image_to_msg(img: np.ndarray, stamp, frame_id: str) -> Image:
    if img.dtype != np.uint8 or img.ndim != 3 or img.shape[2] != 3:
        raise ValueError("RGB の uint8 画像（高さ x 幅 x 3）を渡す")
    msg = Image()
    msg.header.stamp = stamp
    msg.header.frame_id = frame_id
    msg.height, msg.width = int(img.shape[0]), int(img.shape[1])
    msg.encoding = "rgb8"
    msg.is_bigendian = 0
    msg.step = msg.width * 3
    # bytes をそのまま入れると 1 要素ずつ確かめられて遅いので、array('B') にして渡す
    msg.data = array.array("B", np.ascontiguousarray(img).tobytes())
    return msg


def msg_to_image(msg: Image) -> np.ndarray:
    if msg.encoding not in ("rgb8", "bgr8"):
        raise ValueError(f"対応していない encoding: {msg.encoding}")
    buf = np.frombuffer(msg.data, dtype=np.uint8).reshape(msg.height, msg.step)
    img = buf[:, : msg.width * 3].reshape(msg.height, msg.width, 3)
    return img[:, :, ::-1].copy() if msg.encoding == "bgr8" else img.copy()


def color(rgb, a: float = 1.0) -> ColorRGBA:
    """rgb は 0〜1 の 3 つの数。"""
    r, g, b = (float(v) for v in rgb)
    return ColorRGBA(r=r, g=g, b=b, a=float(a))


def rgb255(rgb) -> tuple[float, float, float]:
    return tuple(v / 255.0 for v in rgb)


def pose_from_T(T: np.ndarray) -> Pose:
    p = Pose()
    p.position.x, p.position.y, p.position.z = (float(v) for v in T[:3, 3])
    qx, qy, qz, qw = quat_xyzw(T[:3, :3])
    p.orientation.x, p.orientation.y, p.orientation.z, p.orientation.w = qx, qy, qz, qw
    return p


def transform(T: np.ndarray, parent: str, child: str, stamp) -> TransformStamped:
    t = TransformStamped()
    t.header.stamp = stamp
    t.header.frame_id = parent
    t.child_frame_id = child
    t.transform.translation.x, t.transform.translation.y, t.transform.translation.z = (float(v) for v in T[:3, 3])
    qx, qy, qz, qw = quat_xyzw(T[:3, :3])
    t.transform.rotation.x, t.transform.rotation.y, t.transform.rotation.z, t.transform.rotation.w = qx, qy, qz, qw
    return t


def marker(ns: str, mid: int, mtype: int, frame_id: str, stamp, rgb=(1.0, 1.0, 1.0), alpha: float = 1.0,
           scale=(1.0, 1.0, 1.0), lifetime: float = 0.0, frame_locked: bool = False) -> Marker:
    m = Marker()
    m.header.frame_id = frame_id
    m.header.stamp = stamp
    m.ns, m.id, m.type, m.action = ns, mid, mtype, Marker.ADD
    m.pose.orientation.w = 1.0
    m.scale.x, m.scale.y, m.scale.z = (float(v) for v in scale)
    m.color = color(rgb, alpha)
    m.frame_locked = frame_locked
    sec = int(lifetime)
    m.lifetime = Duration(sec=sec, nanosec=int((lifetime - sec) * 1e9))
    return m


def text_marker(ns: str, mid: int, text: str, frame_id: str, stamp, xyz, height: float, rgb,
                lifetime: float = 0.0) -> Marker:
    """視点に向く文字。RViz の字は ASCII だけなので、日本語は入れない。

    Windows の RViz では半角の空白がとても広く描かれ、語がばらばらに見えたので、空白は "_" に替える。
    行を分けたいときは改行（\n）を使う。
    """
    m = marker(ns, mid, Marker.TEXT_VIEW_FACING, frame_id, stamp, rgb, 1.0, (0.0, 0.0, height), lifetime)
    m.pose.position.x, m.pose.position.y, m.pose.position.z = (float(v) for v in xyz)
    m.text = "\n".join("_".join(line.split()) for line in text.split("\n"))
    return m


def mesh_marker(ns: str, mid: int, tris: np.ndarray, frame_id: str, stamp, rgb, alpha: float = 1.0) -> Marker:
    m = marker(ns, mid, Marker.TRIANGLE_LIST, frame_id, stamp, rgb, alpha, frame_locked=True)
    m.points = [Point(x=float(x), y=float(y), z=float(z)) for x, y, z in tris.reshape(-1, 3)]
    return m


def lines_marker(ns: str, mid: int, segments, frame_id: str, stamp, rgb, width: float = 0.002,
                 alpha: float = 1.0) -> Marker:
    m = marker(ns, mid, Marker.LINE_LIST, frame_id, stamp, rgb, alpha, (width, 0.0, 0.0), frame_locked=True)
    m.points = [Point(x=float(p[0]), y=float(p[1]), z=float(p[2])) for seg in segments for p in seg]
    return m
