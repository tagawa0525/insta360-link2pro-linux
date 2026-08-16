# 参照プロジェクト

Insta360 Link 系の UVC Extension Unit (XU) プロトコルを調査する際に参照した外部リポジトリの記録。

クローンした実体は `.gitignore` で除外している。再取得するには以下を実行する。

```bash
cd references
git clone https://github.com/vrwallace/Insta360-Link-1-and-2-Controller-for-Linux
git clone https://github.com/jfwoods/insta360link-controller
git clone https://github.com/EdenCoder/insta360-linux
```

## 取得したリポジトリ

| リポジトリ | 取得コミット | 日付 | 内容 |
| --- | --- | --- | --- |
| [vrwallace/Insta360-Link-1-and-2-Controller-for-Linux](https://github.com/vrwallace/Insta360-Link-1-and-2-Controller-for-Linux) | `216124b96ec5b6d7f64d4c85d4e3e405f679e1ec` | 2026-07-12 | Free Pascal 製の Linux 向けコントローラ。**XU セレクタマップの一次情報**。`uv4l2.pas` にセレクタ定数、`uinsta360link.pas` 冒頭にモード表がある |
| [jfwoods/insta360link-controller](https://github.com/jfwoods/insta360link-controller) | `0500fe7586996fcbb56178809cd68358b3fd9761` | 2026-04-04 | ESP32 ジョイスティック + デーモン。WebSocket(9000)/TCP(9001) 経由の PTZ 制御が主で、対応は macOS |
| [EdenCoder/insta360-linux](https://github.com/EdenCoder/insta360-linux) | `98ab785db53d203579f97d430372f39a85fcfdee` | 2026-03-03 | Linux 向けユーティリティ |

## 参考記事

- [Reverse-engineering Insta360 Link Controller WebSockets protocol](https://dt.in.th/Insta360LinkControllerWebSocketProtocol) — 純正 Link Controller の WebSocket プロトコル解析（XU とは別レイヤ）

## 対応表（vrwallace 版のソースから抽出）

Windows の KS プロパティ監視によって特定されたもの、と原典に記載がある。

### Unit 9 / Selector 2 — モード制御

`uinsta360link.pas` の冒頭コメントより。原典は 52 バイトバッファ。

| モード | byte[0] | byte[1] |
| --- | --- | --- |
| 通常（オフ） | `0x00` | `0x00` |
| AI 追跡 | `0x01` | `0x00` |
| ホワイトボード | `0x04` | `0x01` |
| オーバーヘッド（書画カメラ） | `0x05` | `0x03` |
| DeskView（机 + 顔の分割表示） | `0x06` | `0x10` |

### その他のセレクタ

`uv4l2.pas` より。

| 定数 | Unit | Selector | 内容 |
| --- | --- | --- | --- |
| `XU_MODE_CONTROL` | 9 | 2 | モード制御（上表） |
| `XU_GIMBAL_RESET_CONTROL` | 9 | 14 | ジンバルを中央へリセット（1 バイト、SET 専用） |
| `XU_TRACKING_FRAME_CONTROL` | 9 | 19 | 追跡フレーミング: `0x01` 頭部 / `0x02` 上半身 / `0x03` 全身 |
| `XU_TRACKING_TARGET_CONTROL` | 10 | 1 | 追跡対象: buf[4] に `0x00` 単一 / `0x01` グループ |

## 手元の Link 2 Pro での検証状況

原典は Insta360 Link (PID `0x4C01`) と Link 2 (PID `0x4C04`) が対象で、**Link 2 Pro (PID `0x4C06`) は対象外**。実機で確認した差異は以下。

| 項目 | 原典 | Link 2 Pro 実機 | 判定 |
| --- | --- | --- | --- |
| XU Unit ID | 9 | 9, 10, 11 が存在 | 一致 |
| Unit 9 / Sel 2 の長さ | 52 バイト | **61 バイト** | 相違。長さのみ異なりバイト位置の意味は一致 |
| Unit 9 / Sel 2 の byte[0]=`0x01` | AI 追跡 | 書き込み・読み返し成功 | 動作 |
| Unit 9 / Sel 14 | 1 バイト SET 専用 | 1 バイト SET 専用 | 一致 |
| Unit 9 / Sel 19 | 1 バイト GET/SET | 1 バイト GET/SET、初期値 `0x02`（上半身） | 一致 |
| Unit 10 / Sel 1 | 追跡対象を buf[4] に SET | **GET 専用**（`info=0x01`）で byte[4] は増加し続けるカウンタ | **相違。未検証** |

バッファ長が異なるため、原典の「ゼロ埋めしたバッファを書く」方式ではなく、
**現在値を読んで該当バイトだけ差し替えて書き戻す**方が既存設定を壊さず安全。
