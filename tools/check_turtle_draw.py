"""turtle_draw の制御を、turtlesim と同じ動きの式で確かめる（turtlesim を起動しなくてよい）。

turtlesim（turtle.cpp）は 16ms ごとに「向き += 角速度×dt → 位置 += (cos, sin)×速さ×dt」で亀を動かし、
最後に届いた速度の指令を使い続ける。ノードのタイマーは 20ms ごと。
各点に着いたときの誤差と、線分からのずれの最大値を出す。

    pixi run check_turtle_draw
"""
import math
import sys
import types
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "src" / "py_pubsub"))
from py_pubsub.turtle_draw import LETTER_F, TurtleDraw  # noqa: E402

DT_SIM, DT_NODE = 0.016, 0.02


def seg_dist(p, a, b):
    """点 p と線分 ab の距離。"""
    (px, py), (ax, ay), (bx, by) = p, a, b
    vx, vy = bx - ax, by - ay
    t = max(0.0, min(1.0, ((px - ax) * vx + (py - ay) * vy) / (vx * vx + vy * vy)))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


class _Pub:
    last = None

    def publish(self, msg):
        self.last = msg


class _Logger:
    def info(self, msg):
        print("  " + msg)


def run(flat, label):
    pts = list(zip(flat[0::2], flat[1::2]))
    # ROS につながず、ノードの step() だけを使う
    node = types.SimpleNamespace(points=pts, tolerance=0.05, pose=None, at_start=False, target=1, done=False, cmd=_Pub())
    node.get_logger = _Logger
    x, y, th = pts[0][0], pts[0][1], 0.0  # 始点に瞬間移動した直後
    v = w = 0.0
    t = next_node = 0.0
    max_dev = 0.0
    print(label)
    while not node.done and t < 120:
        node.pose = types.SimpleNamespace(x=x, y=y, theta=th)
        while next_node <= t:
            TurtleDraw.step(node)
            if node.cmd.last is not None:
                v, w = node.cmd.last.linear.x, node.cmd.last.angular.z
            next_node += DT_NODE
        th = math.atan2(math.sin(th + w * DT_SIM), math.cos(th + w * DT_SIM))
        x += math.cos(th) * v * DT_SIM
        y += math.sin(th) * v * DT_SIM
        t += DT_SIM
        if node.target < len(pts):
            max_dev = max(max_dev, seg_dist((x, y), pts[node.target - 1], pts[node.target]))
    print(f"  done={node.done} time={t:.1f}s max deviation from segments={max_dev:.3f}")
    return node.done and max_dev < 0.05


def main():
    ok = run(LETTER_F, "letter F (default)")
    ok &= run([2.0, 2.0, 9.0, 2.0, 5.5, 8.0, 2.0, 2.0], "triangle")
    print("OK" if ok else "NG")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
