"""ROS 2 の C++ のプログラムの隣に、新しい Visual C++ ランタイムを置く（Windows、管理者権限なし）。

Windows は DLL を「exe と同じフォルダ → System32 → PATH」の順に探す。System32 の
Visual C++ ランタイムが古いと、新しいコンパイラで作られた ROS 2 の C++ のプログラムが
起動してすぐ落ちる（0xC0000005。LOG.md の 2026-11-02 を参照）。

本来の直し方は、管理者の許可を得て Visual C++ 再頒布可能パッケージを最新にすること。
それができないときは、pixi の環境に入っている新しいランタイムを exe の隣に置く
（Microsoft が認めている「ローカル配置」）。System32 のほうが新しければ何もしない。

    pixi run vcrt_local          # 必要なら置く（turtlesim などのタスクは先にこれを実行する）
    pixi run vcrt_local --undo   # 置いたものを消す
"""
from __future__ import annotations

import argparse
import ctypes
import filecmp
import os
import shutil
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]

# conda-forge の vc14_runtime に入っている DLL
RUNTIME = [
    "msvcp140.dll",
    "msvcp140_1.dll",
    "msvcp140_2.dll",
    "msvcp140_atomic_wait.dll",
    "msvcp140_codecvt_ids.dll",
    "vcruntime140.dll",
    "vcruntime140_1.dll",
    "vcruntime140_threads.dll",
    "concrt140.dll",
    "vccorlib140.dll",
    "vcomp140.dll",
]


class _FixedFileInfo(ctypes.Structure):
    _fields_ = [
        (name, ctypes.c_uint32)
        for name in (
            "signature", "struc_version", "file_version_ms", "file_version_ls",
            "product_version_ms", "product_version_ls", "file_flags_mask", "file_flags",
            "file_os", "file_type", "file_subtype", "file_date_ms", "file_date_ls",
        )
    ]


def file_version(path: Path) -> tuple[int, int, int, int] | None:
    """DLL のバージョン（例: (14, 51, 36247, 0)）。読めなければ None。"""
    api = ctypes.windll.version
    size = api.GetFileVersionInfoSizeW(str(path), None)
    if not size:
        return None
    buf = ctypes.create_string_buffer(size)
    if not api.GetFileVersionInfoW(str(path), 0, size, buf):
        return None
    ptr, length = ctypes.c_void_p(), ctypes.c_uint()
    if not api.VerQueryValueW(buf, "\\", ctypes.byref(ptr), ctypes.byref(length)):
        return None
    info = ctypes.cast(ptr, ctypes.POINTER(_FixedFileInfo)).contents
    ms, ls = info.file_version_ms, info.file_version_ls
    return ms >> 16, ms & 0xFFFF, ls >> 16, ls & 0xFFFF


def _fmt(v: tuple[int, ...] | None) -> str:
    return "なし" if v is None else ".".join(map(str, v))


def exe_dirs(roots: list[Path]) -> list[Path]:
    """C++ のプログラム（exe）が入っているフォルダ。"""
    return sorted({p.parent for root in roots if root.is_dir() for p in root.rglob("*.exe")})


def place(src_dir: Path, dirs: list[Path]) -> int:
    n = 0
    for d in dirs:
        for name in RUNTIME:
            src, dst = src_dir / name, d / name
            if not src.exists() or dst.exists():
                continue
            try:
                os.link(src, dst)  # 同じドライブならハードリンク（容量を使わない）
            except OSError:
                shutil.copy2(src, dst)
            n += 1
    return n


def remove(src_dir: Path, dirs: list[Path]) -> int:
    n = 0
    for d in dirs:
        for name in RUNTIME:
            src, dst = src_dir / name, d / name
            # 自分で置いたもの（環境のランタイムと中身が同じもの）だけを消す
            if dst.exists() and src.exists() and filecmp.cmp(src, dst, shallow=False):
                dst.unlink()
                n += 1
    return n


def main(argv: list[str] | None = None) -> int:
    ap = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    ap.add_argument("--undo", action="store_true", help="置いた DLL を消す")
    args = ap.parse_args(argv)

    prefix = Path(os.environ["CONDA_PREFIX"])
    dirs = exe_dirs([prefix / "Library" / "lib", ROOT / "install" / "lib"])
    if args.undo:
        print(f"{remove(prefix, dirs)} 個の DLL を消しました")
        return 0

    system = file_version(Path(os.environ.get("SystemRoot", r"C:\Windows")) / "System32" / "msvcp140.dll")
    env = file_version(prefix / "msvcp140.dll")
    print(f"Visual C++ ランタイム: System32 {_fmt(system)} / 環境 {_fmt(env)}")
    if env is None:
        print("環境にランタイムが見つかりません（pixi install を確認）", file=sys.stderr)
        return 1
    if system is not None and system >= env:
        print("System32 のほうが新しいか同じなので、何もしません")
        return 0
    n = place(prefix, dirs)
    print(f"System32 が古いので、{len(dirs)} 個のフォルダの exe の隣に置きました（新しく {n} 個）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
