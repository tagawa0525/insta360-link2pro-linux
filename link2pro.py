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
VIDIOC_G_FMT = _iowr(4, 208)  # struct v4l2_format は 64-bit で 208 バイト

# UVC Extension Unit (linux/uvcvideo.h)
# struct uvc_xu_control_query { u8 unit; u8 selector; u8 query; u16 size; u8 *data; }
XU_QUERY_FMT = "<BBBxH2xQ"
UVCIOC_CTRL_QUERY = (_IOC_WRITE_READ << 30) | (16 << 16) | (ord("u") << 8) | 0x21
UVC_SET_CUR, UVC_GET_CUR, UVC_GET_LEN = 0x01, 0x81, 0x85

# ベンダ独自コントロールの所在。詳細は references/README.md を参照
XU_MODE = (9, 2)  # モード制御
XU_RESET = (11, 5)  # ジンバルリセット（1 バイト、SET 専用）

DEVICE_NAME = "Insta360 Link 2 Pro"
UNITS_PER_DEGREE = 3600

# desk サブコマンドが机へ向けるチルト角。モニタ上端に載せた実機で、映像を
# 見ながら 1 度ずつ追い込んだ値。設置環境で変わるため --tilt で上書きできる
DESK_TILT = -53.0

# 追跡へ入る前に正面・水平へ戻すのにかける秒数。向け直す経路では glide が
# この秒数だけ待ちながら目標を送り、resync 経由の経路では同じ秒数だけ待つ。
# いずれもジンバルが物理的に到達するまでの猶予として使う
FACE_FRONT_SECONDS = 1.5

# モード名 -> (モード ID, 書き込む byte[1], 完了とみなす byte[1])
#
# byte[1] に 0x00 を書いて入り口の状態にすると、カメラが自分で次の状態へ進む。
# 原典（Link / Link 2 向け）の最終値を直接書いても機能は動作しないため、
# ホワイトボードと DeskView は必ず 0x00 を経由する。
MODES: dict[str, tuple[int, int, int | None]] = {
    "normal": (0x00, 0x00, None),
    "tracking": (0x01, 0x00, None),
    "overhead": (0x05, 0x03, None),
    "whiteboard": (0x04, 0x00, 0x02),
    # DeskView の byte[1] は 0x10 の約 0.2 秒後に 0x11 になるが、0x10 のまま
    # 変わらないこともある（実機で 12 秒観測）。byte[0] が入れ替わった時点で
    # 有効なので、来ないかもしれない値は待たない
    "deskview": (0x06, 0x00, None),
}
MODE_BY_ID = {v[0]: k for k, v in MODES.items()}
# ジンバルが自律的に動くモード。この間とその直後は制御値が実位置を反映しない
AUTONOMOUS_MODES = frozenset({"tracking", "overhead"})
MODE_TRANSITION = 0xFF  # 遷移中に byte[0] が返す値。この間の書き込みは無視される
# ホワイトボードの自動検出が失敗したときに byte[1] が返す値。カメラは
# これを返した約 1 秒後に byte[0] を自分で normal へ戻す
WHITEBOARD_DETECT_FAILED = 0x03


class Gimbal:
    """PTZ コントロールを持つ V4L2 デバイスへのハンドル。"""

    def __init__(self, path: str) -> None:
        self.path = path
        self.fd = os.open(path, os.O_RDWR)
        self.xu = Xu(self.fd)
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
        # V4L2 の有効値は lo + n*step (≤ hi)。lo 基準で丸め、n を範囲内に収める
        # ことで、範囲端と step が整合しないデバイスでも hi を超えない
        n = round((raw - lo) / step)
        n = max(0, min(n, (hi - lo) // step))
        return lo + n * step

    @property
    def pan(self) -> float:
        return self._get(V4L2_CID_PAN_ABSOLUTE) / UNITS_PER_DEGREE

    @property
    def tilt(self) -> float:
        return self._get(V4L2_CID_TILT_ABSOLUTE) / UNITS_PER_DEGREE

    @property
    def zoom(self) -> float:
        return self._get(V4L2_CID_ZOOM_ABSOLUTE) / 100

    def format_size(self) -> tuple[int, int]:
        """現在ネゴシエートされているキャプチャ解像度 (width, height) を返す。"""
        buf = ctypes.create_string_buffer(struct.pack("<I4x", 1), 208)
        fcntl.ioctl(self.fd, VIDIOC_G_FMT, buf)
        width, height = struct.unpack_from("<II", buf.raw, 8)
        return width, height

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

    # --- ベンダ独自モード ---

    def mode_state(self) -> tuple[int, int]:
        """現在のモードを (byte[0], byte[1]) で返す。"""
        raw = self.xu.get(*XU_MODE)
        return raw[0], raw[1]

    @property
    def mode(self) -> str:
        """現在のモード名。遷移中や未知の値は生の値を返す。"""
        mode_id, _ = self.mode_state()
        return MODE_BY_ID.get(mode_id, f"0x{mode_id:02x}")

    def set_mode(self, name: str, timeout: float = 15.0) -> tuple[int, int]:
        """モードを切り替え、遷移が落ち着くまで待つ。

        書き込み直後は byte[0] が 0xFF（遷移中）を返し、その間の書き込みは
        無視される。目的の値が読めるまでリトライする必要がある。
        """
        normal_id = MODES["normal"][0]
        current = self.mode_state()[0]
        if name != "normal" and current not in (normal_id, MODES[name][0]):
            # モード間を直接切り替えると 0xFF のまま固まる。必ず通常を経由する
            self._enter(*MODES["normal"], timeout, "normal")
        return self._enter(*MODES[name], timeout, name)

    def _enter(
        self,
        mode_id: int,
        entry_flag: int,
        settled_flag: int | None,
        timeout: float,
        name: str,
    ) -> tuple[int, int]:
        deadline = time.time() + timeout
        last_write = 0.0
        entered = False  # 今回の遷移で目的のモードに入ったことを観測したか
        while True:
            state = self.mode_state()
            entered = entered or state[0] == mode_id
            if (
                entered
                and mode_id == MODES["whiteboard"][0]
                and state[1] == WHITEBOARD_DETECT_FAILED
            ):
                # 検出失敗はカメラ側の終了状態。ここで入り口を書き直すと
                # 0xff（遷移中）に戻り、失敗の理由が読み取れなくなる。
                # byte[1] は次のモードに入るまで前の値が残るため、モードに
                # 入ったことを確認するまでは前回の失敗の残骸と区別できない
                raise WhiteboardNotDetected(
                    f"ホワイトボードを自動検出できませんでした（byte[0]=0x{state[0]:02x}"
                    f" byte[1]=0x{state[1]:02x}）。ボードが画角に入っているか確認するか、"
                    "--corners で四隅を指定してください。"
                )
            if state[0] == mode_id:
                if settled_flag is None or state[1] == settled_flag:
                    return state
                # モードには入った。以降はカメラ側の検出を待つだけ。
                # ここで入り口の値を書き直すと検出がやり直しになる
            elif time.time() - last_write >= 3.0:
                # 遷移中(0xFF)の書き込みは無視されるため、間隔を空けて promote し直す
                self.xu.patch(*XU_MODE, {0: mode_id, 1: entry_flag})
                last_write = time.time()
            if time.time() >= deadline:
                if state[0] == mode_id:
                    # モードは有効。検出が終わらなかっただけ（ホワイトボード等）
                    return state
                raise ModeTimeout(
                    f"{name} へ切り替わりません（byte[0]=0x{state[0]:02x}"
                    f" byte[1]=0x{state[1]:02x}）。ストリームが流れているか確認してください。"
                )
            time.sleep(1.0)

    def set_whiteboard_region(self, corners: list[tuple[float, float]]) -> None:
        """ホワイトボード補正の四辺形を明示指定する。

        corners は左上・左下・右下・右上の順で、画面に対する 0〜1 の正規化座標。
        通常モードから、モード ID・フラグ・座標を含む完全なペイロードを一括で書く。
        受理されると 1 秒以内に byte[1]=0x02 で即ロックし、自動検出は走らない。

        bytes 34-37 のアスペクト値はカメラ側で検証され、範囲外だとペイロード
        全体が黙って破棄されて自動検出に落ちる。受理窓は現在ネゴシエート
        されているビデオフォーマットのアスペクトを中心とした約 ±6% の固定窓で、
        四隅の形状には依存しない（16:9 と 4:3 の両ストリームで実測）。
        そのため現在のフォーマットから計算した値を書く。
        """
        if len(corners) != 4:
            raise ValueError("四隅は 4 点で指定してください")
        if any(not 0.0 <= v <= 1.0 for pt in corners for v in pt):
            raise ValueError("四隅の座標は 0.0〜1.0 の正規化座標で指定してください")
        width, height = self.format_size()
        payload = b"".join(struct.pack("<f", v) for pt in corners for v in pt)
        payload += struct.pack("<f", width / height)
        changes = {0: MODES["whiteboard"][0], 1: 0x02, 50: 0xAA}
        changes.update({2 + i: b for i, b in enumerate(payload)})
        self.xu.patch(*XU_MODE, changes)

    def whiteboard_region(self) -> list[tuple[float, float]] | None:
        """検出済みの四隅を返す。未検出なら None。"""
        raw = self.xu.get(*XU_MODE)
        vals = [struct.unpack("<f", raw[2 + 4 * i : 6 + 4 * i])[0] for i in range(8)]
        if not any(vals):
            return None
        return [(vals[i], vals[i + 1]) for i in (0, 2, 4, 6)]

    def reset_gimbal(self) -> None:
        """ジンバルを中央・水平へ物理的に戻す。

        原典の Unit 9 Selector 14 は Link 2 Pro では効かない。
        物理位置は pan/tilt の制御値に反映されないため、
        呼び出し後に制御値と実位置がずれる点に注意。
        """
        self.xu.write(*XU_RESET, bytes([0x01]))

    def resync(self) -> None:
        """制御値を実位置に合わせ直す。

        自律動作のあとは制御値と物理位置がずれる。同じ値を書いても無操作に
        なって動かないため、いったん別の値を経由して原点へ戻す。
        """
        self.set_pan_tilt(1, 1)
        time.sleep(1)
        self.set_pan_tilt(0, 0)
        self.set_zoom(1.0)

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


class ModeError(RuntimeError):
    """モードの切り替えに失敗した。"""


class ModeTimeout(ModeError):
    """モードが所定時間内に目的の状態へ遷移しなかった。"""


class WhiteboardNotDetected(ModeError):
    """ホワイトボードの自動検出がカメラ側で失敗した。"""


class Xu:
    """UVC Extension Unit へのアクセス。Gimbal と同じ fd を共有する。"""

    def __init__(self, fd: int) -> None:
        self.fd = fd

    def _query(
        self, unit: int, sel: int, query: int, size: int, payload: bytes | None = None
    ):
        buf = ctypes.create_string_buffer(payload if payload else size)
        fcntl.ioctl(
            self.fd,
            UVCIOC_CTRL_QUERY,
            struct.pack(XU_QUERY_FMT, unit, sel, query, size, ctypes.addressof(buf)),
        )
        return buf.raw[:size]

    def length(self, unit: int, sel: int) -> int:
        return int.from_bytes(self._query(unit, sel, UVC_GET_LEN, 2), "little")

    def get(self, unit: int, sel: int) -> bytes:
        return self._query(unit, sel, UVC_GET_CUR, self.length(unit, sel))

    def patch(self, unit: int, sel: int, changes: dict[int, int]) -> bytes:
        """現在値を読み、指定バイトだけ差し替えて書き戻す。

        バッファ長が機種で異なるうえ、モード以外の設定も同じバッファに同居する。
        ゼロ埋めして書くと他の設定を壊すため、必ず read-modify-write する。
        """
        ln = self.length(unit, sel)
        data = bytearray(self._query(unit, sel, UVC_GET_CUR, ln))
        for offset, value in changes.items():
            data[offset] = value
        self._query(unit, sel, UVC_SET_CUR, ln, bytes(data))
        return bytes(data)

    def write(self, unit: int, sel: int, payload: bytes) -> None:
        """GET 不可のセレクタ向けに生の値を書く。"""
        self._query(unit, sel, UVC_SET_CUR, len(payload), payload)


class Wake:
    """移動中だけダミーのストリームを流し、カメラをスタンバイから起こす。

    UVC のこのデバイスは mmap ストリーミングしか対応しない（read() 不可）ため、
    標準ライブラリだけでストリームを開くには ioctl を多数実装する必要がある。
    ここでは既にシステムにある v4l2-ctl に任せ、PTZ 制御本体は標準ライブラリのみに保つ。
    """

    # ストリーム開始からカメラが指令を受け付けるまでの待ち。
    # 1.5 秒では XU のモード変更が取りこぼされることを実機で確認した
    SETTLE = 3.0

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
                "      別途カメラを使用するアプリを起動しておくか、v4l-utils を導入してください。",
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
            # 即終了の正当な理由は「他アプリがカメラを使用中」（この場合ストリームは
            # 既に流れており続行してよい）だが、権限やフォーマットの問題でも同様に
            # 終了する。原因を切り分けられるよう終了コードを添えて警告する
            print(
                f"警告: 起こすためのストリーム (v4l2-ctl) が終了コード "
                f"{self.proc.returncode} で即終了しました。\n"
                "      他アプリがカメラ使用中なら問題ありません。そうでない場合は"
                "スタンバイのままとなり、指令を受理しても物理的に動きません。",
                file=sys.stderr,
            )
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
    mode_id, flag = g.mode_state()
    print(f"  mode : {g.mode} (byte[0]=0x{mode_id:02x} byte[1]=0x{flag:02x})")
    if mode_id == MODES["whiteboard"][0]:
        region = g.whiteboard_region()
        if region:
            pts = " ".join(f"({x:.3f},{y:.3f})" for x, y in region)
            print(f"         四隅 {pts}")
        else:
            print("         四隅は未検出")
    if mode_id == MODES["tracking"][0]:
        print("  注意 : 追跡中は pan/tilt の値が実位置を反映しません")


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


def _release_mode(g: Gimbal) -> str | None:
    """自律的に動くモード（AUTONOMOUS_MODES）だけを解除し、制御値を合わせ直す。

    それ以外のモードでは何もしない。自律動作のあとは制御値が実位置とずれて
    おり、目的の角度を書いても「現在値と同じ」と見なされて駆動しないことが
    あるため、解除に続けて resync する。解除したモード名を返す（対象外なら
    None。呼び出し側で通常モードへ戻す必要があるかは、この戻り値では判断
    できないので g.mode を見ること）。
    """
    current = g.mode
    if current not in AUTONOMOUS_MODES:
        return None
    print(f"{current} を解除します")
    g.set_mode("normal")
    g.resync()
    return current


def _face_front(g: Gimbal) -> None:
    """追跡に入れる状態を作る。

    追跡は人物が画角に入っていないと起動せず、byte[0] が 0xff のまま
    タイムアウトする（机へ向けた状態で再現。正面へ戻すと 1 秒で入る）。
    追跡はどのみちカメラを自分で向けるため、先に正面・水平へ戻しても
    副作用にならない。

    指令を出すだけでは足りず、ジンバルが物理的に向き終わるのを待つ必要が
    ある。待たずに続けると、まだ元の方向を向いている間に判定が走って失敗する。
    向け直す経路では glide が duration 分だけ待ちながら目標を送るのでそれが
    待ちを兼ねるが、resync 経由では原点を指令するだけで到達を待たないため、
    別途待つ必要がある。
    """
    released = _release_mode(g)
    if released is None and g.mode != "normal":
        # DeskView などが残っていると画も向きも別物になる。素の状態から入る
        g.set_mode("normal")
    if (g.pan, g.tilt) != (0.0, 0.0):
        g.glide(0.0, 0.0, FACE_FRONT_SECONDS)
    elif released is not None:
        # resync は原点を指令するが到達は待たない。真下を向いた overhead から
        # 入ると、戻り切る前に追跡の判定が走ってしまう
        time.sleep(FACE_FRONT_SECONDS)


def cmd_desk(g: Gimbal, args: argparse.Namespace) -> None:
    """机上を書画カメラとして写す。

    DeskView は画を 180 度回すだけでカメラは正面を向いたままなので、単体では
    机が写らない。逆に真下を向けるだけでは画が上下逆になる。「下を向ける」の
    がチルト、「上下を戻す」のが DeskView という分担のため、両方が要る。

    モードに入るときにジンバルが動くので、チルトは入ったあとに指定する。
    """
    # 追跡が有効なままだと机へ向けても被写体へ向き直される
    _release_mode(g)

    state = g.set_mode("deskview")
    print(f"mode: {g.mode} (byte[0]=0x{state[0]:02x} byte[1]=0x{state[1]:02x})")
    # プリセットとして画角を確定させる。パンやズームが残っていると机の端や
    # その一部しか写らない
    args.pan = 0.0
    args.zoom = 1.0
    cmd_moveto(g, args)


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


def _parse_corners(text: str) -> list[tuple[float, float]]:
    nums = [float(v) for v in text.replace(" ", "").split(",")]
    if len(nums) != 8:
        raise argparse.ArgumentTypeError(
            "四隅は x1,y1,x2,y2,x3,y3,x4,y4 の 8 個で指定します"
        )
    if any(not 0.0 <= v <= 1.0 for v in nums):
        raise argparse.ArgumentTypeError("座標は 0.0〜1.0 の正規化座標で指定します")
    return [(nums[i], nums[i + 1]) for i in range(0, 8, 2)]


def cmd_mode(g: Gimbal, args: argparse.Namespace) -> None:
    if args.name is None:
        cmd_status(g, args)
        return
    if args.corners and args.name != "whiteboard":
        raise SystemExit("--corners は whiteboard でのみ指定できます")

    if args.name == "tracking":
        _face_front(g)

    if args.corners:
        # 手動指定は通常モードから完全なペイロードを一括で書く。
        # 受理されれば通常 1 秒程度で即ロックするが、カメラ起動直後は
        # 遷移(0xFF)が長引くことがあるためポーリングで待つ
        g.set_mode("normal")
        g.set_whiteboard_region(args.corners)
        deadline = time.time() + 10
        while time.time() < deadline:
            state = g.mode_state()
            if state == (MODES["whiteboard"][0], 0x02):
                break
            time.sleep(0.5)
        state = g.mode_state()
    else:
        state = g.set_mode(args.name)
    print(f"mode: {g.mode} (byte[0]=0x{state[0]:02x} byte[1]=0x{state[1]:02x})")

    if args.name == "whiteboard":
        region = g.whiteboard_region()
        if region:
            print("  四隅 " + " ".join(f"({x:.3f},{y:.3f})" for x, y in region))
            if args.corners and any(
                abs(av - ev) > 0.01
                for actual, expected in zip(region, args.corners)
                for av, ev in zip(actual, expected)
            ):
                print("  警告: 指定と異なる四隅が返りました（自動検出に落ちた可能性）")
        elif args.corners:
            print(
                "  指定した四隅が破棄されました。縦長など不正な形状の可能性があります"
            )
        else:
            print("  四隅が未検出です。ボードが画角に入っているか確認してください")
    elif args.name == "tracking":
        print("  追跡中は pan/tilt での位置指定が打ち消されます")


def cmd_reset(g: Gimbal, args: argparse.Namespace) -> None:
    # 追跡などが有効なままだと、リセット直後にカメラが再び被写体へ向いてしまい
    # リセットが無意味になる。先にモードを解除する
    current = g.mode
    if current != "normal":
        if args.keep_mode:
            print(f"警告: {current} が有効です。リセットが打ち消される可能性があります")
        else:
            print(f"{current} を解除します（維持するには --keep-mode）")
            g.set_mode("normal")

    g.reset_gimbal()
    print("ジンバルを中央へリセットしました")
    time.sleep(5)
    if args.resync:
        g.resync()
        _report(g)
    else:
        print(
            "  制御値は実位置とずれています。合わせるには --resync を指定してください"
        )


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

    p = sub.add_parser("desk", help="机上を書画カメラとして写す（DeskView + チルト）")
    p.add_argument(
        "-l",
        "--tilt",
        type=float,
        default=DESK_TILT,
        help=f"机へ向けるチルト角（度、既定 {DESK_TILT:+.0f}）。"
        "最適値はカメラの高さと机までの距離で変わる",
    )
    add_duration(p, 1.0)
    p.set_defaults(func=cmd_desk, needs_wake=True)

    p = sub.add_parser("zoom", help="ズーム倍率を設定")
    p.add_argument("factor", type=float, help="倍率（1.0 〜 4.0）")
    p.set_defaults(func=cmd_zoom, needs_wake=True)

    p = sub.add_parser("demo", help="可動域を一巡するデモ動作")
    add_duration(p, 1.5)
    p.set_defaults(func=cmd_demo, needs_wake=True)

    p = sub.add_parser("mode", help="撮影モードを切り替え（省略時は現状を表示）")
    p.add_argument("name", nargs="?", choices=sorted(MODES), help="切り替え先のモード")
    p.add_argument(
        "--corners",
        type=_parse_corners,
        help="whiteboard の四隅を x1,y1,...,x4,y4 で指定"
        "（左上・左下・右下・右上、0〜1 の正規化座標、横長のみ）。"
        "省略時はカメラの自動検出に任せる",
    )
    p.set_defaults(func=cmd_mode, needs_wake=True)

    p = sub.add_parser("reset", help="ジンバルを中央・水平へ物理的に戻す")
    p.add_argument(
        "--resync",
        action="store_true",
        help="リセット後に pan/tilt/zoom の制御値を実位置へ合わせる",
    )
    p.add_argument(
        "--keep-mode",
        action="store_true",
        help="有効なモードを解除せずリセットする（追跡中は打ち消される）",
    )
    p.set_defaults(func=cmd_reset, needs_wake=True)

    return parser


def main(argv: list[str] | None = None) -> int:
    # ioctl 番号と構造体レイアウト (XU_QUERY_FMT のポインタ幅、v4l2_format の
    # 208 バイト等) は 64-bit ABI 前提でハードコードしている。32-bit では
    # 別の値になり誤動作するため、実行前に明示的に弾く
    if struct.calcsize("P") != 8:
        raise SystemExit(
            "このスクリプトは 64-bit Python 専用です"
            "（ioctl 構造体レイアウトを 64-bit ABI 前提で組み立てています）。"
        )
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
    except ModeError as e:
        raise SystemExit(str(e))
    except KeyboardInterrupt:
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
