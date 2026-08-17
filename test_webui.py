"""Web UI の API 分配と MJPEG フレーム分割に対するテスト。

実機なしで検証するため、Gimbal は必要なメソッドだけ持つ代役に差し替える。
HTTP 層（ソケット・ストリーミング配信）は対象外で、純粋ロジックのみを見る。
"""

from __future__ import annotations

import io

import pytest

import link2pro
import webui

# 検証用の最小 JPEG 断片。実際の中身は問わず SOI/EOI の対だけを模す
JPEG_A = b"\xff\xd8\xff\xe0AAAA\xff\xd9"
JPEG_B = b"\xff\xd8\xff\xe0BBBB\xff\xd9"


class TestFrameSplitter:
    def test_単一フレームを切り出す(self) -> None:
        s = webui.FrameSplitter()
        assert s.feed(JPEG_A) == [JPEG_A]

    def test_複数フレームが一括で来ても分割する(self) -> None:
        s = webui.FrameSplitter()
        assert s.feed(JPEG_A + JPEG_B) == [JPEG_A, JPEG_B]

    def test_フレーム途中で分かれても結合する(self) -> None:
        s = webui.FrameSplitter()
        assert s.feed(JPEG_A[:5]) == []
        assert s.feed(JPEG_A[5:] + JPEG_B[:3]) == [JPEG_A]
        assert s.feed(JPEG_B[3:]) == [JPEG_B]

    def test_SOI以前のごみは捨てる(self) -> None:
        s = webui.FrameSplitter()
        assert s.feed(b"\x00\xffgarbage" + JPEG_A) == [JPEG_A]

    def test_EOI待ちの断片は保持し続ける(self) -> None:
        s = webui.FrameSplitter()
        assert s.feed(JPEG_A[:-1]) == []
        assert s.feed(JPEG_A[-1:]) == [JPEG_A]


class FakeGimbal:
    """XU・ioctl を伴わない Gimbal の代役。呼ばれた操作を記録する。"""

    def __init__(self) -> None:
        self._pan = 0.0
        self._tilt = 0.0
        self._zoom = 1.0
        self._mode = "normal"
        self.limits = {"zoom": (100, 400, 1)}
        self.calls: list[tuple] = []

    @property
    def pan(self) -> float:
        return self._pan

    @property
    def tilt(self) -> float:
        return self._tilt

    @property
    def zoom(self) -> float:
        return self._zoom

    @property
    def mode(self) -> str:
        return self._mode

    def range_deg(self, axis: str) -> tuple[float, float]:
        return (-150.0, 150.0) if axis == "pan" else (-70.0, 70.0)

    def set_pan_tilt(self, pan=None, tilt=None) -> None:
        self.calls.append(("set_pan_tilt", pan, tilt))
        if pan is not None:
            self._pan = pan
        if tilt is not None:
            self._tilt = tilt

    def set_zoom(self, factor: float) -> None:
        self.calls.append(("set_zoom", factor))
        self._zoom = factor

    def glide(self, pan: float, tilt: float, duration: float, fps: int = 30) -> None:
        self.calls.append(("glide", pan, tilt, duration))
        self._pan, self._tilt = pan, tilt

    def set_mode(self, name: str, timeout: float = 15.0) -> tuple[int, int]:
        self.calls.append(("set_mode", name))
        self._mode = name
        return (link2pro.MODES[name][0], 0)

    def resync(self) -> None:
        self.calls.append(("resync",))

    def reset_gimbal(self) -> None:
        self.calls.append(("reset_gimbal",))


@pytest.fixture(autouse=True)
def sleeps(monkeypatch: pytest.MonkeyPatch) -> list[float]:
    """待ちを実際には行わず、待った秒数を記録する。"""
    calls: list[float] = []
    monkeypatch.setattr(link2pro.time, "sleep", calls.append)
    monkeypatch.setattr(webui.time, "sleep", calls.append)
    return calls


class TestReadParams:
    """POST ボディの検証。

    Content-Type を application/json に限定することで、ブラウザの
    クロスオリジン simple request（text/plain 等）による CSRF 的な
    カメラ操作を preflight で遮断する。
    """

    def test_JSONオブジェクトを読む(self) -> None:
        rfile = io.BytesIO(b'{"pan": 1}')
        assert webui.read_params("application/json", 10, rfile) == {"pan": 1}

    def test_charset付きのContentTypeも受け付ける(self) -> None:
        rfile = io.BytesIO(b"{}")
        assert webui.read_params("application/json; charset=utf-8", 2, rfile) == {}

    def test_JSON以外のContentTypeは拒む(self) -> None:
        with pytest.raises(ValueError, match="application/json"):
            webui.read_params("text/plain", 10, io.BytesIO(b'{"pan": 1}'))

    def test_ContentTypeなしは拒む(self) -> None:
        with pytest.raises(ValueError, match="application/json"):
            webui.read_params(None, 10, io.BytesIO(b'{"pan": 1}'))

    def test_空ボディは空パラメータとして扱う(self) -> None:
        assert webui.read_params("application/json", 0, io.BytesIO(b"")) == {}

    def test_巨大ボディは読む前に拒む(self) -> None:
        class Untouchable:
            def read(self, n: int) -> bytes:
                raise AssertionError("上限超過のボディを読んではいけない")

        with pytest.raises(ValueError, match="大きすぎ"):
            webui.read_params("application/json", webui.MAX_BODY + 1, Untouchable())

    def test_オブジェクト以外のJSONは拒む(self) -> None:
        with pytest.raises(TypeError):
            webui.read_params("application/json", 3, io.BytesIO(b"[1]"))


class TestHandleApi:
    def test_statusは姿勢と可動範囲を返す(self) -> None:
        g = FakeGimbal()
        status, body = webui.handle_api(g, "status", {})
        assert status == 200
        assert body["pan"] == 0.0
        assert body["tilt"] == 0.0
        assert body["zoom"] == 1.0
        assert body["mode"] == "normal"
        assert body["ranges"]["pan"] == [-150.0, 150.0]
        assert body["ranges"]["zoom"] == [1.0, 4.0]

    def test_movetoで角度とズームを設定する(self) -> None:
        g = FakeGimbal()
        status, _ = webui.handle_api(g, "moveto", {"pan": 30, "tilt": -10, "zoom": 2.0})
        assert status == 200
        assert ("set_pan_tilt", 30, -10) in g.calls
        assert ("set_zoom", 2.0) in g.calls

    def test_movetoは省略した軸を保つ(self) -> None:
        g = FakeGimbal()
        g._tilt = -20.0
        webui.handle_api(g, "moveto", {"pan": 15})
        assert ("set_pan_tilt", 15, None) in g.calls
        assert g.tilt == -20.0

    def test_movetoは数値以外を拒む(self) -> None:
        g = FakeGimbal()
        status, _body = webui.handle_api(g, "moveto", {"pan": "abc"})
        assert status == 400
        assert g.calls == []

    def test_movetoは真偽値も拒む(self) -> None:
        # JSON の true/false は Python では int の派生なので、数値チェックを
        # すり抜けやすい。角度として渡っても意味を成さないため弾く
        g = FakeGimbal()
        status, _ = webui.handle_api(g, "moveto", {"pan": True})
        assert status == 400
        assert g.calls == []

    def test_zoomを設定する(self) -> None:
        g = FakeGimbal()
        status, _ = webui.handle_api(g, "zoom", {"factor": 3.0})
        assert status == 200
        assert ("set_zoom", 3.0) in g.calls

    def test_centerで原点と等倍へ戻す(self) -> None:
        g = FakeGimbal()
        g._pan = 40.0
        status, _ = webui.handle_api(g, "center", {})
        assert status == 200
        assert ("set_pan_tilt", 0.0, 0.0) in g.calls
        assert ("set_zoom", 1.0) in g.calls

    def test_モードを切り替える(self) -> None:
        g = FakeGimbal()
        status, body = webui.handle_api(g, "mode", {"name": "deskview"})
        assert status == 200
        assert g.mode == "deskview"
        assert body["mode"] == "deskview"

    def test_不明なモード名は拒む(self) -> None:
        g = FakeGimbal()
        status, _ = webui.handle_api(g, "mode", {"name": "party"})
        assert status == 400
        assert g.calls == []

    def test_trackingは正面へ戻してから入る(self) -> None:
        g = FakeGimbal()
        g._pan = 40.0
        webui.handle_api(g, "mode", {"name": "tracking"})
        # _face_front 相当: 正面へ glide してから追跡へ
        assert ("glide", 0.0, 0.0, link2pro.FACE_FRONT_SECONDS) in g.calls
        assert g.calls[-1] == ("set_mode", "tracking")

    def test_モード遷移の失敗は409で返す(self) -> None:
        class TimeoutGimbal(FakeGimbal):
            def set_mode(self, name, timeout=15.0):
                raise link2pro.ModeTimeout("モード遷移がタイムアウトしました")

        status, body = webui.handle_api(TimeoutGimbal(), "mode", {"name": "whiteboard"})
        assert status == 409
        assert "タイムアウト" in body["error"]

    def test_deskは既定チルトで机へ向ける(self) -> None:
        g = FakeGimbal()
        status, _ = webui.handle_api(g, "desk", {})
        assert status == 200
        assert g.mode == "deskview"
        assert ("glide", 0.0, link2pro.DESK_TILT, 1.0) in g.calls
        assert ("set_zoom", 1.0) in g.calls

    def test_deskはチルトを上書きできる(self) -> None:
        g = FakeGimbal()
        webui.handle_api(g, "desk", {"tilt": -60})
        assert ("glide", 0.0, -60, 1.0) in g.calls

    def test_resetは解除とresyncを伴う(self) -> None:
        g = FakeGimbal()
        g._mode = "tracking"
        status, _ = webui.handle_api(g, "reset", {})
        assert status == 200
        assert ("set_mode", "normal") in g.calls
        assert ("reset_gimbal",) in g.calls
        assert ("resync",) in g.calls

    def test_未知のコマンドは404(self) -> None:
        status, _ = webui.handle_api(FakeGimbal(), "party", {})
        assert status == 404
