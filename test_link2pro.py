"""モード遷移の状態機械に対するテスト。

実機なしで検証するため、XU の読み書きだけを差し替えた Gimbal を組み立てる。
"""

from __future__ import annotations

import pytest

import link2pro


class FakeXu:
    """あらかじめ与えた状態列を順に返す XU の代役。

    末尾の状態は以降ずっと返し続ける（実機のポーリングと同じく、
    読み出し回数に依存せず最終状態が観測される）。
    """

    def __init__(self, states: list[tuple[int, int]]) -> None:
        self.states = list(states)
        self.writes: list[dict[int, int]] = []

    def get(self, unit: int, selector: int) -> bytes:
        state = self.states.pop(0) if len(self.states) > 1 else self.states[0]
        return bytes(state) + bytes(62)

    def patch(self, unit: int, selector: int, changes: dict[int, int]) -> None:
        self.writes.append(changes)


@pytest.fixture(autouse=True)
def sleeps(monkeypatch: pytest.MonkeyPatch) -> list[float]:
    """ポーリング間隔で待たず、待った回数を記録する。"""
    calls: list[float] = []
    monkeypatch.setattr(link2pro.time, "sleep", calls.append)
    return calls


def gimbal(states: list[tuple[int, int]]) -> link2pro.Gimbal:
    g = link2pro.Gimbal.__new__(link2pro.Gimbal)
    g.xu = FakeXu(states)
    return g


def enter_whiteboard(g: link2pro.Gimbal) -> tuple[int, int]:
    return g._enter(*link2pro.MODES["whiteboard"], 15.0, "whiteboard")


def test_whiteboard_detect_failure_is_reported() -> None:
    """自動検出の失敗 (byte[1]=0x03) は専用の例外で報告する。

    実機では 0x04,0x00 -> 0x04,0x01 と進んだあと、ボードが見つからないと
    約 13 秒で byte[1]=0x03 を返し、カメラ自身が normal へ戻す。
    """
    g = gimbal([(0x00, 0x00), (0xFF, 0x00), (0x04, 0x00), (0x04, 0x01), (0x04, 0x03)])

    with pytest.raises(link2pro.WhiteboardNotDetected) as excinfo:
        enter_whiteboard(g)

    assert "--corners" in str(excinfo.value)
    # 失敗後に入り口を書き直すと 0xff に戻り、誤った診断になる
    assert g.xu.writes == [{0: 0x04, 1: 0x00}]


def test_whiteboard_detect_success() -> None:
    """検出が完了したら byte[1]=0x02 の状態を返す。"""
    g = gimbal([(0x00, 0x00), (0xFF, 0x00), (0x04, 0x01), (0x04, 0x02)])

    assert enter_whiteboard(g) == (0x04, 0x02)


def test_stale_failure_flag_does_not_abort() -> None:
    """入り口を書く前に残っていた 0x03 は前回の失敗の残骸で、今回の結果ではない。"""
    g = gimbal([(0x00, 0x03), (0x04, 0x00), (0x04, 0x02)])

    assert enter_whiteboard(g) == (0x04, 0x02)


def test_stale_failure_flag_during_transition_does_not_abort() -> None:
    """byte[1] は次のモードに入るまで前の値が残る。遷移中の 0x03 も残骸。

    失敗直後（カメラが byte[1] を消すまでの約 1 秒）に再実行すると、
    入り口を書いたあとも 0x03 を読みうる。
    """
    g = gimbal([(0x00, 0x03), (0xFF, 0x03), (0x04, 0x00), (0x04, 0x02)])

    assert enter_whiteboard(g) == (0x04, 0x02)


class FakeGimbal:
    """コマンド関数から呼ばれた操作を順に記録する Gimbal の代役。"""

    def __init__(self, mode: str = "normal") -> None:
        self.calls: list[tuple] = []
        # 自律動作のあとを模して、制御値は原点から離れた値にしておく
        self.pan = 40.0
        self.tilt = 0.0
        self.zoom = 1.0
        self.mode = mode

    def set_mode(self, name: str) -> tuple[int, int]:
        self.calls.append(("mode", name))
        self.mode = name
        return (0x06, 0x11)

    def resync(self) -> None:
        self.calls.append(("resync",))
        self.pan, self.tilt = 0.0, 0.0

    def glide(self, pan: float, tilt: float, duration: float) -> None:
        self.calls.append(("glide", pan, tilt, duration))
        self.pan, self.tilt = pan, tilt

    def set_pan_tilt(self, pan: float | None = None, tilt: float | None = None) -> None:
        self.calls.append(("set_pan_tilt", pan, tilt))
        # 本物と同じく、None はその軸を変更しない
        if pan is not None:
            self.pan = pan
        if tilt is not None:
            self.tilt = tilt

    def set_zoom(self, factor: float) -> None:
        self.calls.append(("zoom", factor))
        self.zoom = factor


def run_desk(argv: list[str], mode: str = "normal") -> FakeGimbal:
    return run_cli(argv, mode)


def run_cli(argv: list[str], mode: str = "normal", **attrs: object) -> FakeGimbal:
    args = link2pro.build_parser().parse_args(argv)
    g = FakeGimbal(mode)
    for k, v in attrs.items():
        setattr(g, k, v)
    args.func(g, args)
    return g


def test_tracking_faces_front_first() -> None:
    """追跡は人物が画角に入っていないと起動しない（実機で確認）。

    机へ向けた状態（tilt -53）では 15 秒待っても入れず、正面・水平へ戻すと
    1 秒で入る。追跡はどのみちカメラを自分で向けるので、先に正面へ戻す。
    """
    g = run_cli(["mode", "tracking"], mode="normal", pan=40.0, tilt=-53.0)

    assert g.calls == [
        ("set_pan_tilt", 0.0, 0.0),
        ("mode", "tracking"),
    ]


def test_tracking_from_another_mode_releases_and_resyncs() -> None:
    """自律動作していたモードから入るときは制御値も合わせ直す。"""
    g = run_cli(["mode", "tracking"], mode="overhead")

    assert g.calls == [
        ("mode", "normal"),
        ("resync",),
        ("mode", "tracking"),
    ]


def test_tracking_already_front_does_not_move() -> None:
    """既に正面・水平なら動かさない。無駄な駆動を避ける。"""
    g = run_cli(["mode", "tracking"], mode="normal", pan=0.0, tilt=0.0)

    assert g.calls == [("mode", "tracking")]


def test_other_modes_are_not_recentered() -> None:
    """正面化は追跡のためだけ。他のモードは向きを変えない。"""
    g = run_cli(["mode", "overhead"], mode="normal", pan=40.0, tilt=-53.0)

    assert g.calls == [("mode", "overhead")]


def test_desk_faces_front() -> None:
    """机は正面にある。パンが残っていると机の端しか写らない。

    順序も重要で、チルトはモードに入ったあとに指定する（モードに入るときに
    ジンバルが動くため、先に指定すると打ち消される）。
    """
    g = run_desk(["desk", "-t", "0"])

    assert g.calls == [
        ("mode", "deskview"),
        ("set_pan_tilt", 0.0, link2pro.DESK_TILT),
        ("zoom", 1.0),
    ]


def test_desk_tilt_can_be_overridden() -> None:
    """既定角は設置環境（カメラ高さ・机までの距離）依存なので上書きできる。"""
    g = run_desk(["desk", "--tilt", "-60", "-t", "0"])

    assert g.calls == [
        ("mode", "deskview"),
        ("set_pan_tilt", 0.0, -60.0),
        ("zoom", 1.0),
    ]


def test_desk_glides_by_default() -> None:
    """既定では移動に時間をかける（急な駆動を避ける）。"""
    g = run_desk(["desk"])

    assert g.calls == [
        ("mode", "deskview"),
        ("glide", 0.0, link2pro.DESK_TILT, 1.0),
        ("zoom", 1.0),
    ]


@pytest.mark.parametrize("mode", ["tracking", "overhead"])
def test_desk_releases_autonomous_modes_and_resyncs(mode: str) -> None:
    """ジンバルが自律的に動くモードでは、制御値が実位置とずれている。

    追跡は解除しないと机へ向けても被写体へ向き直される。またずれたままだと、
    正面を指す 0 を書いても「現在値と同じ」と見なされて駆動しない。
    """
    g = run_desk(["desk", "-t", "0"], mode=mode)

    assert g.calls == [
        ("mode", "normal"),
        ("resync",),
        ("mode", "deskview"),
        ("set_pan_tilt", 0.0, link2pro.DESK_TILT),
        ("zoom", 1.0),
    ]


@pytest.mark.parametrize("mode", ["normal", "deskview", "whiteboard"])
def test_desk_does_not_resync_without_autonomous_motion(mode: str) -> None:
    """ジンバルが動いていないモードでは制御値を信用できる。

    resync は 1 秒待って原点を経由するため、desk を続けて実行するたびに
    走ると無駄な動きになる。
    """
    g = run_desk(["desk", "-t", "0"], mode=mode)

    assert ("resync",) not in g.calls
    assert ("mode", "normal") not in g.calls


def test_deskview_does_not_wait_beyond_the_mode_id(sleeps: list[float]) -> None:
    """DeskView は byte[0] が入れ替わった時点で有効。byte[1] は待たない。

    実機では 0x06,0x10 のあと約 0.2 秒で 0x11 になるが、12 秒観測しても
    0x10 のままのことがある。来ないかもしれない値を待つと最大 15 秒止まる。
    """
    g = gimbal([(0x00, 0x10), (0xFF, 0x10), (0x06, 0x10)])

    assert g._enter(*link2pro.MODES["deskview"], 15.0, "deskview") == (0x06, 0x10)
    assert g.xu.writes == [{0: 0x06, 1: 0x00}]
    # 3 回目の読みで返る。0x11 を待つとタイムアウトまで 15 回待つことになる
    assert len(sleeps) == 2


def test_mode_timeout_mentions_stream() -> None:
    """モードにすら入れないときはストリームを疑う（従来どおり）。"""
    g = gimbal([(0xFF, 0x00)])

    with pytest.raises(link2pro.ModeTimeout, match="ストリーム"):
        g._enter(*link2pro.MODES["whiteboard"], 0.0, "whiteboard")
