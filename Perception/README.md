# Perception（画像取得・認識）

G1のカメラ映像を取得し、YOLOによる物体検出を行う機能。

## 構成

- `common/` — sim/real共通のロジック
  - `camera/` — 画像取得元。`ZmqFrameSource`（ZMQストリーム、sim/real共通）・
    `VideoFileSource`（動画ファイル、テスト/フォールバック用）・
    `WebcamSource`（Webカメラ、手元確認用）
  - `detector/` — `YoloDetector`（ultralytics YOLO26による物体検出）
  - `output/` — `ResultWriter`（検出結果をconsole/JSON/CSVへ出力）
  - `pipeline.py` — `FrameSource -> YoloDetector -> ResultWriter` を繋いでループ実行する
- `sim/` — MuJoCoシムの`head_camera`が配信するZMQストリームに接続して動作確認
  （`run_sim.py` + `configs/config.yaml`）
- `real/` — 実機G1のカメラ（`run_g1_server.py --camera`が配信するZMQストリーム、
  デフォルトポート5555）に接続（`run_real.py` + `configs/config.yaml`）
- `tests/` — `run_tests.sh`（CIが自動検出）。サンプル画像・動画によるYOLO検出の動作確認

sim・realのどちらも**同じZMQストリームの仕組み**（`common/camera/zmq_camera.py`の
`ZmqFrameSource`）を使う。接続先(`server_address`)がシムなら`localhost`、実機なら
G1本体のIPになるだけの違い。

## 環境

`G1_HuggingFace/venv/`（操作PC側、Python 3.12の素のvenv）を共通で使う想定。
Perceptionは**Dockerを使わずvenv環境で動かす**方針（Dockerを使うのは`Mapping/`のみ）。

### セットアップ

```bash
cd G1_HuggingFace
python3.12 -m venv venv          # 未構築の場合。詳細はSETUP.mdを参照
source venv/bin/activate

cd ../Perception
pip install -r requirements.txt
```

- `pip install`後の初回実行時、`ultralytics`がYOLOの重み(`yolo26n.pt`)を自動ダウンロード
  する。ネットワーク接続が必要（リポジトリ直下の`.gitignore`が`*.pt`を除外しているため、
  重みファイル自体はコミットしない）
- CUDA推論を有効にする場合、pip既定の`torch`/`torchvision`はCPU版wheelが入ることがある。
  GPUドライバのCUDAバージョンに対応した`torch`を`--index-url`指定で入れ直すこと
  （`G1_HuggingFace/README.md`の既知の注意点を参照）

### 実行

```bash
# sim: MuJoCoシム側でZMQカメラ配信を有効にした状態(head_camera, port 5555)で
python Perception/sim/run_sim.py

# real: 実機G1側で `run_g1_server.py --camera` を起動した状態で
python Perception/real/run_real.py --server-address 192.168.123.164
```

## ZMQカメラストリームについて

`common/camera/zmq_camera.py`の`ZmqFrameSource`は、`lerobot`本体
（`src/lerobot/cameras/zmq/camera_zmq.py`の`ZMQCamera`）が使っているワイヤプロトコルを
調査した上で、`pyzmq`+`cv2`+`numpy`だけで薄く再実装したもの。`lerobot`パッケージ自体
（cyclonedds/unitree_sdk2py/pinocchio込みの重い依存）には依存しない。

- サーバ（`run_g1_server.py --camera`、MuJoCoシムも同じ）はZMQ PUBソケットで
  JSON文字列 `{"images": {"<camera_name>": "<base64エンコードされたJPEG>"}}` を配信する
- クライアントはSUBソケット（`SUBSCRIBE ""`・`CONFLATE=True`で常に最新フレームのみ保持・
  `RCVTIMEO`でタイムアウト）で接続し、受信したJSONをデコードして1フレームを得る

⚠️ **lerobot公式のZMQプロトコルに準拠しているが、lerobot側でこのプロトコル
（JSONのキー名や base64+JPEG という形式）が将来変わった場合は、
`common/camera/zmq_camera.py`側も追従が必要。** 変わっていないか確認したい場合は
`G1_HuggingFace/lerobot/src/lerobot/cameras/zmq/camera_zmq.py`を参照すること
（このリポジトリには含まれない外部クローン。`.gitignore`で除外済み）。

## 進め方

`G1_HuggingFace/`で歩行機能を実装した際の流れ（シムで検証 → 実機で検証）を踏襲する:
1. `sim/`でロジックを作り、MuJoCo環境の`head_camera`で動作確認
2. `real/`で実機カメラに接続し、同じロジックが実機でも動くか確認

失敗した内容は`FAILURES.md`に記録する。

## 状態

`common/`・`sim/`・`real/`の骨格を実装済み。ZMQストリーム経由の実接続確認（sim/real
双方）はこれから。
