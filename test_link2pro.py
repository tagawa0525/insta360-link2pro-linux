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
def no_sleep(monkeypatch: pytest.MonkeyPatch) -> None:
    """ポーリング間隔で待たない。"""
    monkeypatch.setattr(link2pro.time, "sleep", lambda _: None)


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


def test_mode_timeout_mentions_stream() -> None:
    """モードにすら入れないときはストリームを疑う（従来どおり）。"""
    g = gimbal([(0xFF, 0x00)])

    with pytest.raises(link2pro.ModeTimeout, match="ストリーム"):
        g._enter(*link2pro.MODES["whiteboard"], 0.0, "whiteboard")
