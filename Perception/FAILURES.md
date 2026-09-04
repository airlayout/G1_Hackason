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
