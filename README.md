# insta360-link2pro-linux

Insta360 Link 2 Pro を Linux から制御する CLI。純正アプリ（Windows/macOS のみ）が
提供する機能のうち、以下を UVC/V4L2 経由で操作できる。

- パン / チルト / ズーム（V4L2 標準コントロール）
- AI 人物追跡・オーバーヘッド・ホワイトボード・DeskView の各モード切り替え
  （ベンダ独自の UVC Extension Unit をリバースエンジニアリング）
- ホワイトボード補正の四隅の手動指定と自動検出
- ジンバルの中央リセット

PTZ 制御本体は Python 標準ライブラリのみで動作する。カメラをスタンバイから
起こすためのダミーストリームにのみ `v4l2-ctl`（v4l-utils）を使う。

## 必要なもの

- Linux（V4L2 / uvcvideo）
- Python 3.11+
- v4l-utils（`v4l2-ctl`。Zoom や OBS など他アプリがカメラ使用中なら `--no-wake` で不要）
- `/dev/video*` への読み書き権限（通常は `video` グループ）

## 使い方

```bash
./link2pro.py status                     # 姿勢・可動範囲・現在モード
./link2pro.py moveto --pan 45 --tilt 20  # 絶対角度で移動（-t で移動秒数）
./link2pro.py move --pan -30             # 相対移動
./link2pro.py zoom 2.5                   # ズーム 1.0〜4.0x
./link2pro.py center                     # 正面・等倍へ
./link2pro.py reset --resync             # ジンバルを物理的に中央へ

./link2pro.py mode tracking              # AI 人物追跡
./link2pro.py mode whiteboard            # ホワイトボード補正（自動検出）
./link2pro.py mode whiteboard \
  --corners 0.12,0.14,0.09,0.62,0.62,0.63,0.63,0.15   # 四隅を手動指定
./link2pro.py mode deskview              # 机側を映す（180 度回転）
./link2pro.py mode overhead              # 書画カメラ（真下）
./link2pro.py mode normal                # 解除
```

## 仕組みと調査記録

ベンダ独自機能は UVC Extension Unit（Unit 9 / 10 / 11）への読み書きで制御する。
セレクタ対応表・モード遷移の状態機械・ホワイトボード四隅ペイロードの構造・
アスペクト受理窓の測定結果など、実機でのリバースエンジニアリングの詳細は
[references/README.md](references/README.md) にまとめてある。

要点:

- モード切り替えは `byte[1]=0x00` の入り口状態を経由する必要がある。
  既知の最終値を直接書いても受理されるが動作しない
- カメラはストリームが流れていない間、指令を受理しても物理的に駆動しない
- 追跡やリセットなどの自律動作中は `pan_absolute` 等の制御値が実位置を
  反映しない。値の読み返しだけでは動作を確認できない

## 注意

- 非公式のリバースエンジニアリング成果であり、Insta360 とは無関係
- 実機確認は Insta360 Link 2 Pro（USB ID `2e1a:4c06`、FW v0.3.0.8）のみ。
  Link / Link 2 / Link 2C Pro では XU の配置・挙動が異なる（一部は
  references/README.md に記載）
- 未知のセレクタへの書き込みはカメラのファームウェアをクラッシュさせる
  ことがある（USB 再列挙で復帰）

## ライセンス

[MIT](LICENSE)
