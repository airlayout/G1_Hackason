# 失敗ログ

このフォルダでの開発中に実際に起きた失敗と、その原因・反省・再発防止策を記録する。
新しい失敗が起きたら、下のテンプレートに沿って追記する。

## テンプレート

```
## YYYY-MM-DD: 失敗の一言タイトル

**何が起きたか**
（観測された事実。憶測ではなく実際に見えたことを書く）

**原因**
（コード上・手順上の根本原因）

**反省**
（なぜ事前に気づけなかったか。見落としていた前提は何か）

**再発防止策**
（実際にコード/手順をどう変えたか）
```

---

## 2026-08-30: シムが落ちた後、残ったサブプロセスがポート5555を占有し続け、以後すべての実行が失敗した

**何が起きたか**
カメラ画像を取得しようとしてシミュレーション(`SimpleWalk/sim/release_band_and_walk_forward.py`)
を起動したところ、`Image publishing subprocess started` の直後に
`Segmentation fault (コアダンプ)` で落ちた。

その後、再実行しても同じ場所で落ち続けた。2回目の実行でようやく、segfault の直前に
以下が出ていることに気づいた:

```
zmq.error.ZMQError: Address already in use (addr='tcp://*:5555')
```

あわせて `resource_tracker: There appear to be 10 leaked semaphore objects` /
`1 leaked shared_memory objects` の警告も出ていた。

**原因**
画像配信のサブプロセス(`MP_START_METHOD: "spawn"` で起動される multiprocessing の子)が、
親プロセスの異常終了後も生き残り、**ポート5555と共有メモリを保持し続けていた**。

そのため新しく起動したシムの配信サブプロセスが `socket.bind("tcp://*:5555")` に失敗して
例外死し、メインプロセスもそれに巻き込まれて segfault する、という連鎖になっていた。
一度落ちると、後片付けをしない限り**必ず**再発する状態だった。

**反省**
「昨日は動いたのに今日は動かない」という状況で、**直前に自分が変更したスクリプトを疑った**。
実際の原因は前回の実行が残した状態であり、コードは何も悪くなかった。
プロセス・ポート・共有メモリといった「プログラムの外に残る状態」を疑うという発想が
そもそも無かった。

また、1回目の segfault のログだけでは原因が見えず、`Address already in use` は
2回目の実行で初めて表示された。**エラーが出たら最後まで読み、複数回の実行のログを
見比べる**べきだった。

なお、そもそもの1回目の segfault の原因は未特定(WSL2 + GPU無しの環境で、描画まわりの
衝突と推測しているが確認できていない)。

**再発防止策**
シムが落ちたら、再実行の前に必ず以下で後片付けする。手順は
`Perception/sim/README.md` の「つまずいた点」にも記載した。

```bash
ss -lntp | grep 5555        # 誰がポートを掴んでいるか確認
pkill -f spawn_main         # 残った配信サブプロセスを止める
rm -f /dev/shm/psm_*        # 漏れた共有メモリを削除
```

→ 教訓として一般化すると: **同じコマンドが突然失敗するようになったときは、
コードの変更よりも先に「前回の実行が残した状態」を疑う。**

---

## 2026-09-04: unitree_sdk2_pythonにG1専用のカメラ/videoモジュールが存在しなかった

**何が起きたか**

プロトタイプ（`~/g1-hackason-local`）でカメラ取得を実装する際、`unitree_sdk2py`から
G1のカメラ映像を取得しようとしたが、`unitree_sdk2py`には2025年時点でG1専用の
video/cameraモジュールが存在しないと判明した（go2 / b2 / b2wのみ）。GitHub issue
[unitreerobotics/unitree_sdk2_python#106](https://github.com/unitreerobotics/unitree_sdk2_python/issues/106)
では、go2用の`VideoClient`(`unitree_sdk2py.go2.video.video_client`)がG1でも
そのまま動作する場合があるという未検証の報告があるのみだった。

**原因**

Unitree SDKのカメラ対応がロボット機種ごとに個別実装されており、G1向けの
公式カメラAPIがそもそも用意されていなかった（SDK側の制約であり、実装ミスではない）。

**反省**

`unitree_sdk2py`が全機種で同じAPI体系を持つと思い込んでおり、事前にG1固有の
対応状況を確認していなかった。

**再発防止策**

今回のPerception実装では、そもそも`unitree_sdk2py`のカメラAPI（DDS経由）には
依存しない設計に変更した。`run_g1_server.py --camera`（sim/real共通）が配信する
ZMQストリームに直接接続する方式（`common/camera/zmq_camera.py`の`ZmqFrameSource`）
を採用し、この問題自体を回避している。ワイヤプロトコルは`lerobot`本体の
`ZMQCamera`実装を調査して合わせたもの（詳細は`README.md`の「ZMQカメラストリーム
について」を参照）。**ただし、lerobot公式のZMQプロトコル（JSON+base64 JPEGの
メッセージ形式）に準拠しているだけなので、lerobot側が将来このプロトコルを
変更した場合は`zmq_camera.py`側の追従が必要**であることに注意。
