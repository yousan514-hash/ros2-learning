"""検査の3つのトピックを組にする（ROS に依らないので、ノードを起動しなくてもテストできる）。

  /inspection/shot         （JSON: seq・部品・面・stamp）
  /wrist_camera/image_raw  （画像。header.stamp が shot の stamp と同じ）
  /inspection/result       （JSON: seq・判定・点数・reason）

shot の seq → stamp → 画像の SHA-256 とたどって、結果に画像の指紋を付ける。どの順番で届いてもよい。
"""
from __future__ import annotations

import hashlib
from collections import OrderedDict


class ShotMatcher:
    def __init__(self, keep: int = 50):
        self.keep = keep
        self.stamp_of_seq: OrderedDict[int, tuple[int, int]] = OrderedDict()
        self.sha_of_stamp: OrderedDict[tuple[int, int], str] = OrderedDict()
        self.waiting: OrderedDict[int, dict] = OrderedDict()

    def _trim(self, d: OrderedDict) -> None:
        while len(d) > self.keep:
            d.popitem(last=False)

    def add_shot(self, shot: dict) -> list[tuple[dict, str]]:
        self.stamp_of_seq[int(shot["seq"])] = (int(shot["stamp"][0]), int(shot["stamp"][1]))
        self._trim(self.stamp_of_seq)
        return self._flush()

    def add_image(self, stamp: tuple[int, int], data: bytes) -> list[tuple[dict, str]]:
        self.sha_of_stamp[(int(stamp[0]), int(stamp[1]))] = hashlib.sha256(bytes(data)).hexdigest()
        self._trim(self.sha_of_stamp)
        return self._flush()

    def add_result(self, result: dict) -> list[tuple[dict, str]]:
        self.waiting[int(result["seq"])] = result
        self._trim(self.waiting)
        return self._flush()

    def _flush(self) -> list[tuple[dict, str]]:
        ready = []
        for seq in list(self.waiting):
            sha = self.sha_of_stamp.get(self.stamp_of_seq.get(seq, (-1, -1)))
            if sha:
                ready.append((self.waiting.pop(seq), sha))
        return ready
