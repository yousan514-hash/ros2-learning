"""ソースのパッケージを、ビルドしなくても import できるようにする（リポジトリの一番上から pytest を動かすため）。"""
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
