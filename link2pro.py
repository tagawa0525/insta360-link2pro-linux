#!/usr/bin/env python3
"""Insta360 Link 2 Pro のジンバル（パン/チルト/ズーム）を制御する。

UVC の PTZ コントロールを V4L2 の ioctl 経由で直接叩く。

角度の単位は度。デバイス内部では 1/3600 度（秒角）で保持されるが、
step が 3600 のため実際の分解能は 1 度刻みに丸められる。

重要: このカメラはビデオストリームが流れていない間はスタンバイ状態で、
PTZ 指令を受理して値を保持するものの物理的には駆動しない（読み返しは
指令値と一致するため、値だけでは駆動を確認できない）。そのため本スクリプトは
移動中だけダミーのストリームを開いてカメラを起こす。Zoom や OBS などが
すでにカメラを使用中であれば --no-wake で省略できる。
"""

from __future__ import annotations

import argparse
import ctypes
import fcntl
import glob
import os
import shutil
import struct
import subprocess
import sys
import time
from typing import Self

# V4L2 コントロール ID (linux/v4l2-controls.h)
V4L2_CID_PAN_ABSOLUTE = 0x009A0908
V4L2_CID_TILT_ABSOLUTE = 0x009A0909
V4L2_CID_ZOOM_ABSOLUTE = 0x009A090D

# ioctl 番号 (linux/videodev2.h)。_IOWR('V', nr, struct)
_IOC_WRITE_READ = 3


def _iowr(nr: int, size: int) -> int:
    return (_IOC_WRITE_READ << 30) | (size << 16) | (ord("V") << 8) | nr


VIDIOC_QUERYCTRL = _iowr(36, 68)
VIDIOC_G_CTRL = _iowr(27, 8)
VIDIOC_S_CTRL = _iowr(28, 8)

DEVICE_NAME = "Insta360 Link 2 Pro"
UNITS_PER_DEGREE = 3600


class Gimbal:
    """PTZ コントロールを持つ V4L2 デバイスへのハンドル。"""

    def __init__(self, path: str) -> None:
        self.path = path
        self.fd = os.open(path, os.O_RDWR)
        self.limits = {
            name: self._query(cid)
            for name, cid in (
                ("pan", V4L2_CID_PAN_ABSOLUTE),
                ("tilt", V4L2_CID_TILT_ABSOLUTE),
                ("zoom", V4L2_CID_ZOOM_ABSOLUTE),
            )
        }

    def close(self) -> None:
        os.close(self.fd)

    def __enter__(self) -> Self:
        return self

    def __exit__(self, *exc: object) -> None:
        self.close()

    def _query(self, cid: int) -> tuple[int, int, int]:
        """コントロールの (min, max, step) を返す。"""
        buf = ctypes.create_string_buffer(
            struct.pack("II32siiiiI8x", cid, 0, b"", 0, 0, 0, 0, 0), 68
        )
        fcntl.ioctl(self.fd, VIDIOC_QUERYCTRL, buf)
        _, _, _, lo, hi, step, _, _ = struct.unpack("II32siiiiI8x", buf.raw)
        return lo, hi, step

    def _get(self, cid: int) -> int:
        buf = ctypes.create_string_buffer(struct.pack("Ii", cid, 0), 8)
        fcntl.ioctl(self.fd, VIDIOC_G_CTRL, buf)
        return struct.unpack("Ii", buf.raw)[1]

    def _set(self, cid: int, value: int) -> None:
        fcntl.ioctl(self.fd, VIDIOC_S_CTRL, struct.pack("Ii", cid, value))

    def _clamp(self, axis: str, raw: int) -> int:
        lo, hi, step = self.limits[axis]
        raw = max(lo, min(hi, raw))
        # step の倍数に丸める。ドライバ側でも丸められるが、返り値を予測可能にする
        return round(raw / step) * step

    @property
    def pan(self) -> float:
        return self._get(V4L2_CID_PAN_ABSOLUTE) / UNITS_PER_DEGREE

    @property
    def tilt(self) -> float:
        return self._get(V4L2_CID_TILT_ABSOLUTE) / UNITS_PER_DEGREE

    @property
    def zoom(self) -> float:
        return self._get(V4L2_CID_ZOOM_ABSOLUTE) / 100

    def range_deg(self, axis: str) -> tuple[float, float]:
        lo, hi, _ = self.limits[axis]
        return lo / UNITS_PER_DEGREE, hi / UNITS_PER_DEGREE

    def set_pan_tilt(self, pan: float | None = None, tilt: float | None = None) -> None:
        if pan is not None:
            self._set(
                V4L2_CID_PAN_ABSOLUTE, self._clamp("pan", round(pan * UNITS_PER_DEGREE))
            )
        if tilt is not None:
            self._set(
                V4L2_CID_TILT_ABSOLUTE,
                self._clamp("tilt", round(tilt * UNITS_PER_DEGREE)),
            )

    def set_zoom(self, factor: float) -> None:
        self._set(V4L2_CID_ZOOM_ABSOLUTE, self._clamp("zoom", round(factor * 100)))

    def glide(self, pan: float, tilt: float, duration: float, fps: int = 30) -> None:
        """現在位置から (pan, tilt) まで、duration 秒かけて滑らかに動かす。

        ジンバルは目標角へ最高速で追従するため、絶対位置を細かく送り続けることで
        速度を制御する。
        """
        start_pan, start_tilt = self.pan, self.tilt
        steps = max(1, round(duration * fps))
        for i in range(1, steps + 1):
            # ease-in-out（開始と終了を滑らかにする）
            t = i / steps
            eased = t * t * (3 - 2 * t)
            self.set_pan_tilt(
                start_pan + (pan - start_pan) * eased,
                start_tilt + (tilt - start_tilt) * eased,
            )
            time.sleep(duration / steps)


class Wake:
    """移動中だけダミーのストリームを流し、カメラをスタンバイから起こす。

    UVC のこのデバイスは mmap ストリーミングしか対応しない（read() 不可）ため、
    標準ライブラリだけでストリームを開くには ioctl を多数実装する必要がある。
    ここでは既にシステムにある v4l2-ctl に任せ、PTZ 制御本体は標準ライブラリのみに保つ。
    """

    SETTLE = 1.5  # ストリーム開始からジンバルが駆動可能になるまでの待ち

    def __init__(self, device: str, enabled: bool = True) -> None:
        self.device = device
        self.enabled = enabled
        self.proc: subprocess.Popen | None = None

    def __enter__(self) -> Self:
        if not self.enabled:
            return self
        if shutil.which("v4l2-ctl") is None:
            print(
                "警告: v4l2-ctl が見つかりません。カメラがスタンバイのままだと"
                "指令は受理されても物理的に動きません。\n"
                "      別途カメラを使用するアプリを起動しておくか、v4l2-utils を導入してください。",
                file=sys.stderr,
            )
            return self
        self.proc = subprocess.Popen(
            [
                "v4l2-ctl",
                "-d",
                self.device,
                "--set-fmt-video=width=640,height=480,pixelformat=MJPG",
                "--stream-mmap",
                "--stream-to=/dev/null",
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        time.sleep(self.SETTLE)
        if self.proc.poll() is not None:
            # 他のアプリが排他的に掴んでいる場合など。その場合は既に起きているので続行
            self.proc = None
        return self

    def __exit__(self, *exc: object) -> None:
        if self.proc is None:
            return
        # 移動指令の直後に切ると停止途中で固まるため、駆動完了を待ってから止める
        time.sleep(self.SETTLE)
        self.proc.terminate()
        self.proc.wait()


def find_device(name: str = DEVICE_NAME) -> str:
    """名前が一致し、かつ PTZ コントロールを持つ video デバイスを探す。"""
    candidates = []
    for path in sorted(glob.glob("/dev/video*")):
        sysfs = f"/sys/class/video4linux/{os.path.basename(path)}/name"
        try:
            with open(sysfs) as f:
                if name.lower() not in f.read().strip().lower():
                    continue
        except OSError:
            continue
        candidates.append(path)

    for path in candidates:
        try:
            gimbal = Gimbal(path)
        except OSError:
            continue
        gimbal.close()
        return path

    if candidates:
        raise SystemExit(f"{name} は見つかりましたが PTZ 制御できません: {candidates}")
    raise SystemExit(f"{name} が見つかりません。USB 接続を確認してください。")


def cmd_status(g: Gimbal, args: argparse.Namespace) -> None:
    print(f"デバイス: {g.path}")
    for axis, value, unit in (("pan", g.pan, "°"), ("tilt", g.tilt, "°")):
        lo, hi = g.range_deg(axis)
        print(
            f"  {axis:5s}: {value:+7.1f}{unit}  (範囲 {lo:+.0f}{unit} 〜 {hi:+.0f}{unit})"
        )
    zlo, zhi, _ = g.limits["zoom"]
    print(f"  zoom : {g.zoom:6.2f}x  (範囲 {zlo / 100:.2f}x 〜 {zhi / 100:.2f}x)")


def cmd_moveto(g: Gimbal, args: argparse.Namespace) -> None:
    pan = g.pan if args.pan is None else args.pan
    tilt = g.tilt if args.tilt is None else args.tilt
    if args.duration > 0:
        g.glide(pan, tilt, args.duration)
    else:
        g.set_pan_tilt(pan, tilt)
    if args.zoom is not None:
        g.set_zoom(args.zoom)
    _report(g)


def cmd_move(g: Gimbal, args: argparse.Namespace) -> None:
    args.pan = g.pan + args.pan
    args.tilt = g.tilt + args.tilt
    cmd_moveto(g, args)


def cmd_center(g: Gimbal, args: argparse.Namespace) -> None:
    if args.duration > 0:
        g.glide(0, 0, args.duration)
    else:
        g.set_pan_tilt(0, 0)
    g.set_zoom(1.0)
    _report(g)


def cmd_zoom(g: Gimbal, args: argparse.Namespace) -> None:
    g.set_zoom(args.factor)
    _report(g)


def cmd_demo(g: Gimbal, args: argparse.Namespace) -> None:
    """パン・チルトの可動域を一巡するデモ。"""
    waypoints = [
        ("右へパン", 60, 0),
        ("左へパン", -60, 0),
        ("中央へ", 0, 0),
        ("上を向く", 0, 30),
        ("下を向く", 0, -30),
        ("右上", 45, 25),
        ("左下", -45, -25),
        ("原点復帰", 0, 0),
    ]
    for label, pan, tilt in waypoints:
        print(f"  {label} → pan {pan:+.0f}° / tilt {tilt:+.0f}°")
        g.glide(pan, tilt, args.duration)
    _report(g)


def _report(g: Gimbal) -> None:
    print(f"pan {g.pan:+.1f}° / tilt {g.tilt:+.1f}° / zoom {g.zoom:.2f}x")


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-d", "--device", help="V4L2 デバイスパス（省略時は自動検出）")
    parser.add_argument(
        "--no-wake",
        dest="wake",
        action="store_false",
        help="起こすためのダミーストリームを開かない（他アプリがカメラ使用中の場合に指定）",
    )
    sub = parser.add_subparsers(dest="command", required=True)

    def add_duration(p: argparse.ArgumentParser, default: float) -> None:
        p.add_argument(
            "-t",
            "--duration",
            type=float,
            default=default,
            help=f"移動にかける秒数。0 で最高速（既定 {default}）",
        )

    p = sub.add_parser("status", help="現在の姿勢と可動範囲を表示")
    p.set_defaults(func=cmd_status, needs_wake=False)

    p = sub.add_parser("moveto", help="絶対角度で移動")
    p.add_argument("-p", "--pan", type=float, help="パン角（度、右が正）")
    p.add_argument("-l", "--tilt", type=float, help="チルト角（度、上が正）")
    p.add_argument("-z", "--zoom", type=float, help="ズーム倍率")
    add_duration(p, 1.0)
    p.set_defaults(func=cmd_moveto, needs_wake=True)

    p = sub.add_parser("move", help="現在位置からの相対移動")
    p.add_argument("-p", "--pan", type=float, default=0.0, help="パン変化量（度）")
    p.add_argument("-l", "--tilt", type=float, default=0.0, help="チルト変化量（度）")
    p.add_argument("-z", "--zoom", type=float, help="ズーム倍率（絶対値）")
    add_duration(p, 1.0)
    p.set_defaults(func=cmd_move, needs_wake=True)

    p = sub.add_parser("center", help="原点（正面・等倍）へ戻す")
    add_duration(p, 1.0)
    p.set_defaults(func=cmd_center, needs_wake=True)

    p = sub.add_parser("zoom", help="ズーム倍率を設定")
    p.add_argument("factor", type=float, help="倍率（1.0 〜 4.0）")
    p.set_defaults(func=cmd_zoom, needs_wake=True)

    p = sub.add_parser("demo", help="可動域を一巡するデモ動作")
    add_duration(p, 1.5)
    p.set_defaults(func=cmd_demo, needs_wake=True)

    return parser


def main(argv: list[str] | None = None) -> int:
    args = build_parser().parse_args(argv)
    path = args.device or find_device()
    try:
        with (
            Wake(path, enabled=args.wake and args.needs_wake),
            Gimbal(path) as gimbal,
        ):
            args.func(gimbal, args)
    except PermissionError:
        raise SystemExit(
            f"{path} を開けません。video グループに所属しているか確認してください。"
        )
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
