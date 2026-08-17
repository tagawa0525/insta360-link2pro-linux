#!/usr/bin/env python3
"""Insta360 Link 2 Pro をブラウザから操作する Web UI サーバー。

link2pro.py の Gimbal を HTTP 経由で操作できるようにする。Tailscale 等の
閉じたネットワーク内で使う前提で、認証は持たない（既定では Tailscale の
IP にのみバインドし、tailnet の外へは露出しない）。

映像は /dev/video* の MJPEG をそのまま multipart/x-mixed-replace で配る。
JPEG から JPEG への変換がないため、依存は既存の v4l2-ctl のみで足りる。
"""

from __future__ import annotations

import argparse
import contextlib
import json
import shutil
import subprocess
import sys
import threading
import time
from collections.abc import Iterator
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path

import link2pro
from link2pro import Gimbal, ModeError, Wake

# プレビューの解像度とフレームレート。実機の MJPG が出せる離散値から、
# tailnet 越しの帯域と見やすさの折り合いで選んだ（既定の 60fps は約 7MB/s
# 流れることを実測。30fps へ落として半減させる）
STREAM_WIDTH, STREAM_HEIGHT = 1280, 720
STREAM_FPS = 30

HTML_PATH = Path(__file__).with_name("webui.html")
BOUNDARY = b"link2pro-frame"


class FrameSplitter:
    """連結された JPEG バイト列をフレーム単位に切り出す。

    MJPEG のスキャンデータ内の 0xFF はバイトスタッフィングされるため、
    SOI (FFD8) と EOI (FFD9) の素朴な探索でフレーム境界を判定できる。
    """

    SOI = b"\xff\xd8"
    EOI = b"\xff\xd9"

    def __init__(self) -> None:
        self._buf = bytearray()

    def feed(self, chunk: bytes) -> list[bytes]:
        self._buf += chunk
        frames = []
        while True:
            start = self._buf.find(self.SOI)
            if start < 0:
                # SOI が現れるまでのごみ（v4l2-ctl の前置き等）は捨てる。
                # 末尾 1 バイトは SOI の前半かもしれないため残す
                del self._buf[:-1]
                break
            end = self._buf.find(self.EOI, start + len(self.SOI))
            if end < 0:
                del self._buf[:start]
                break
            frames.append(bytes(self._buf[start : end + len(self.EOI)]))
            del self._buf[: end + len(self.EOI)]
        return frames


class Streamer:
    """v4l2-ctl の MJPEG 出力を読み、最新フレームを購読者へ配る。

    購読者がいる間だけキャプチャを走らせる。カメラはストリームが流れて
    いる間だけ指令を受け付ける（スタンバイ解除）ため、プレビューを開いて
    いる間は別途 Wake を立てる必要がない。
    """

    def __init__(self, device: str) -> None:
        self.device = device
        self._cond = threading.Condition()
        self._frame: bytes | None = None
        self._seq = 0
        self._clients = 0
        self._proc: subprocess.Popen | None = None
        self._started_at = 0.0

    @property
    def active(self) -> bool:
        with self._cond:
            return self._proc is not None and self._proc.poll() is None

    def wait_settled(self) -> None:
        """ストリーム開始直後の、指令が取りこぼされる期間を待つ。"""
        with self._cond:
            started = self._started_at
        remaining = started + Wake.SETTLE - time.monotonic()
        if remaining > 0:
            time.sleep(remaining)

    def _start(self) -> None:
        fmt = (
            f"--set-fmt-video=width={STREAM_WIDTH},height={STREAM_HEIGHT},"
            "pixelformat=MJPG"
        )
        self._proc = subprocess.Popen(
            [
                "v4l2-ctl",
                "-d",
                self.device,
                fmt,
                f"--set-parm={STREAM_FPS}",
                "--stream-mmap",
                "--stream-to=-",
            ],
            stdout=subprocess.PIPE,
            stderr=subprocess.DEVNULL,
        )
        self._started_at = time.monotonic()
        threading.Thread(target=self._pump, args=(self._proc,), daemon=True).start()

    def _stop(self) -> None:
        if self._proc is not None:
            self._proc.terminate()
            self._proc = None

    def _pump(self, proc: subprocess.Popen) -> None:
        splitter = FrameSplitter()
        assert proc.stdout is not None
        while chunk := proc.stdout.read(65536):
            frames = splitter.feed(chunk)
            if not frames:
                continue
            with self._cond:
                self._frame = frames[-1]
                self._seq += 1
                self._cond.notify_all()
        proc.wait()
        with self._cond:
            # 購読者を待ちぼうけにしないため、終了も通知で知らせる
            if self._proc is proc:
                self._proc = None
            self._cond.notify_all()

    def frames(self) -> Iterator[bytes]:
        """購読者ごとに最新フレームを順に返す。切断・停止で終わる。"""
        with self._cond:
            self._clients += 1
            if self._proc is None:
                self._start()
        try:
            seq = 0
            while True:
                with self._cond:
                    self._cond.wait_for(
                        lambda last=seq: self._seq > last or self._proc is None,
                        timeout=5,
                    )
                    if self._proc is None or self._seq == seq:
                        return  # キャプチャ停止またはフレームが途絶えた
                    seq = self._seq
                    frame = self._frame
                assert frame is not None
                yield frame
        finally:
            with self._cond:
                self._clients -= 1
                if self._clients == 0:
                    self._stop()


def _number(params: dict, key: str) -> float | None:
    """params[key] を数値として返す。無ければ None、数値以外は ValueError。

    JSON の true/false は Python では int の派生で数値チェックをすり抜ける
    が、角度として意味を成さないため弾く。
    """
    value = params.get(key)
    if value is None:
        return None
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise TypeError(f"{key} は数値で指定します: {value!r}")
    return value


def _status_body(g: Gimbal) -> dict:
    zlo, zhi, _ = g.limits["zoom"]
    return {
        "pan": g.pan,
        "tilt": g.tilt,
        "zoom": g.zoom,
        "mode": g.mode,
        "ranges": {
            "pan": list(g.range_deg("pan")),
            "tilt": list(g.range_deg("tilt")),
            "zoom": [zlo / 100, zhi / 100],
        },
    }


def handle_api(g: Gimbal, command: str, params: dict) -> tuple[int, dict]:
    """API コマンドを Gimbal 操作へ振り分け、(HTTP ステータス, 応答) を返す。"""
    try:
        if command == "status":
            pass
        elif command == "moveto":
            pan = _number(params, "pan")
            tilt = _number(params, "tilt")
            zoom = _number(params, "zoom")
            if pan is not None or tilt is not None:
                g.set_pan_tilt(pan, tilt)
            if zoom is not None:
                g.set_zoom(zoom)
        elif command == "zoom":
            factor = _number(params, "factor")
            if factor is None:
                raise ValueError("factor を指定します")
            g.set_zoom(factor)
        elif command == "center":
            g.set_pan_tilt(0.0, 0.0)
            g.set_zoom(1.0)
        elif command == "mode":
            name = params.get("name")
            if name not in link2pro.MODES:
                raise ValueError(f"モードは {sorted(link2pro.MODES)} から指定します")
            if name == "tracking":
                link2pro._face_front(g)
            g.set_mode(name)
        elif command == "desk":
            tilt = _number(params, "tilt")
            if tilt is None:
                tilt = link2pro.DESK_TILT
            # cmd_desk と同じ手順: 追跡の解除 → DeskView → 画角の確定
            link2pro._release_mode(g)
            g.set_mode("deskview")
            g.glide(0.0, tilt, 1.0)
            g.set_zoom(1.0)
        elif command == "reset":
            if g.mode != "normal":
                g.set_mode("normal")
            g.reset_gimbal()
            time.sleep(5)  # 物理リセットの完了を待つ（cmd_reset と同じ）
            g.resync()
        else:
            return 404, {"error": f"未知のコマンドです: {command}"}
    except (ValueError, TypeError) as e:
        return 400, {"error": str(e)}
    except ModeError as e:
        return 409, {"error": str(e)}
    return 200, _status_body(g)


class Handler(BaseHTTPRequestHandler):
    """HTTP の入出力。gimbal / streamer / lock はサーバー側が持つ（main 参照）。"""

    def do_GET(self) -> None:
        if self.path == "/":
            body = HTML_PATH.read_bytes()
            self._respond(200, "text/html; charset=utf-8", body)
        elif self.path == "/stream.mjpg":
            self._stream()
        elif self.path == "/api/status":
            self._api("status", {})
        else:
            self._respond(404, "text/plain; charset=utf-8", b"not found")

    def do_POST(self) -> None:
        if not self.path.startswith("/api/"):
            self._respond(404, "text/plain; charset=utf-8", b"not found")
            return
        length = int(self.headers.get("Content-Length") or 0)
        try:
            params = json.loads(self.rfile.read(length) or b"{}")
            if not isinstance(params, dict):
                raise TypeError
        except (ValueError, TypeError):
            self._respond(400, "application/json", b'{"error": "JSON body required"}')
            return
        self._api(self.path.removeprefix("/api/"), params)

    def _api(self, command: str, params: dict) -> None:
        server = self.server
        with server.lock, self._awake(command):
            status, body = handle_api(server.gimbal, command, params)
        self._respond(
            status,
            "application/json",
            json.dumps(body, ensure_ascii=False).encode(),
        )

    def _awake(self, command: str):
        """指令の間カメラを起こしておく。status は読むだけなので不要。"""
        server = self.server
        if command == "status":
            return contextlib.nullcontext()
        if server.streamer.active:
            # プレビュー配信がストリームを流しているので Wake は不要。
            # ただし開始直後は指令が取りこぼされるため落ち着くまで待つ
            server.streamer.wait_settled()
            return contextlib.nullcontext()
        return Wake(server.gimbal.path)

    def _stream(self) -> None:
        self.send_response(200)
        self.send_header(
            "Content-Type",
            f"multipart/x-mixed-replace; boundary={BOUNDARY.decode()}",
        )
        self.end_headers()
        try:
            for frame in self.server.streamer.frames():
                self.wfile.write(
                    b"--%s\r\nContent-Type: image/jpeg\r\n"
                    b"Content-Length: %d\r\n\r\n%s\r\n" % (BOUNDARY, len(frame), frame)
                )
        except (BrokenPipeError, ConnectionResetError):
            pass  # ブラウザ側の切断は正常系

    def _respond(self, status: int, content_type: str, body: bytes) -> None:
        self.send_response(status)
        self.send_header("Content-Type", content_type)
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, format: str, *args: object) -> None:
        # 状態のポーリングとフレーム配信でログが埋まるため、既定では出さない
        pass


def _default_host() -> str:
    """Tailscale の IPv4 にバインドし、tailnet の外へ露出しない既定を作る。"""
    if shutil.which("tailscale"):
        result = subprocess.run(
            ["tailscale", "ip", "-4"], capture_output=True, text=True, check=False
        )
        if result.returncode == 0 and result.stdout.strip():
            return result.stdout.strip().splitlines()[0]
    print(
        "警告: Tailscale の IP を取得できないため 127.0.0.1 にバインドします。\n"
        "      他ホストへ公開するには --host を明示してください。",
        file=sys.stderr,
    )
    return "127.0.0.1"


def main(argv: list[str] | None = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__.splitlines()[0])
    parser.add_argument("-d", "--device", help="V4L2 デバイスパス（省略時は自動検出）")
    parser.add_argument(
        "--host",
        help="バインド先アドレス（省略時は Tailscale の IPv4、なければ 127.0.0.1）",
    )
    parser.add_argument("--port", type=int, default=8600, help="ポート（既定 8600）")
    args = parser.parse_args(argv)

    device = args.device or link2pro.find_device()
    host = args.host or _default_host()

    with Gimbal(device) as gimbal:
        server = ThreadingHTTPServer((host, args.port), Handler)
        server.daemon_threads = True
        server.gimbal = gimbal
        server.streamer = Streamer(device)
        server.lock = threading.Lock()
        print(f"カメラ: {device}")
        print(f"http://{host}:{args.port}/ で待ち受けます（Ctrl-C で終了）")
        try:
            server.serve_forever()
        except KeyboardInterrupt:
            return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())
