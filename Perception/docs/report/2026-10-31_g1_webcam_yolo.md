# 2026-10-31 G1 の Web カメラで YOLO（人の検出）を PC2 の CPU と GPU で動かした

G1 の PC2 に挿した Web カメラの画像に、PC2 の中で YOLO（`yolo26n`）をかけた。CPU（onnxruntime）と GPU（TensorRT）の両方で
動くことを確かめ、速さを測った。**GPU は CPU の約 6 倍速く（1 枚 17 ms）、CPU 使用率は約 7 分の 1、検出結果はほぼ同じ。**

## まとめ

| | 1 枚の検出時間（平均 / 90%） | 受け取りを含めた毎秒の枚数 | CPU 使用率（1 コア = 100%） |
|---|---|---|---|
| CPU（onnxruntime、2 スレッド） | 169.0 / 173.2 ms | 5.6 | 194% |
| CPU（onnxruntime、4 スレッド） | 100.9 / 104.3 ms | 9.1 | 376% |
| CPU（onnxruntime、6 スレッド） | 89.2 / 108.0 ms | 10.0 | 524% |
| **GPU（TensorRT 8.5、FP16）** | **17.1 / 18.7 ms** | 18.7（※） | **55%** |

- 入力はどれも Web カメラ（640×480）を時計回りに 90° 回した 480×640、`--frames 100`、検出は `person` だけ
- ※ GPU の「毎秒の枚数」には 1 枚目の準備の時間（1.28 秒）が入っている。それを除けばおよそ 24 枚/秒（概算。直接は測っていない）
- CPU で使うなら **4 スレッド**がよい（6 スレッドは平均が 1 割速いだけで、90% は悪く、CPU は 1.4 倍使う）
- 推論だけの GPU の時間は 5.6 ms（`trtexec`。下記）。残りの約 11 ms は前処理（縮小・色の変換）など CPU 側

## 環境

| | |
|---|---|
| G1 PC2 | Jetson Orin NX、JetPack 5.1.1（L4T R35.3.1）、CPU 8 コア、メモリ 15 GB、電源モード MAXN |
| GPU まわり | CUDA 11.4、cuDNN 8.6、TensorRT 8.5.2（`/usr/src/tensorrt/bin/trtexec`） |
| CPU 版の Python | conda の `lerobot` 環境（Python 3.12、onnxruntime 1.29.0（CPU のみ）、cv2 4.13、numpy 2.2.6） |
| GPU 版の Python | システムの `/usr/bin/python3`（Python 3.8.10、tensorrt 8.5.2.2、cv2 5.0.0、numpy 1.24.4） |
| Web カメラ | **前方**の Innomaker U20CAM-1080p（USB `0c45:6366`）。測ったときは `/dev/video6`（video7 は同じカメラの付属ノード。video0〜5 は頭の RealSense）。**番号は再起動で変わる**（下の追記） |
| モデル | `yolo26n.pt` → ONNX（end-to-end、出力 (1, 300, 6)、入力 640×640） |
| つなぎ方 | ラボPC から Wi-Fi で `ssh unitree@192.168.0.82`（有線の 192.168.123.164 ではない） |

**PC2 には何もインストールしていない。** PC2 に置いたのは `~/perception/` の下のファイル（`common/`、`real/check_yolo.py`、
`yolo26n.onnx`、`yolo26n.engine`、`yolo26n.names.json`）だけ。

## 分かったこと

### Web カメラは横向きに付いている
撮ったままだと 90° 倒れて写る（天井が左、床が右）。**時計回りに 90° 回す**と正しい向きになる
（`check_yolo.py --rotate cw`）。色の順は OpenCV のまま（BGR）で正しい。

### PC2 の GPU は Python 3.12 の torch では使えない
`lerobot` 環境の torch 2.11 は CUDA 13 向けで、`torch.cuda.is_available()` は `False`
（「The NVIDIA driver on your system is too old (found version 11040)」）。onnxruntime も CPU 版だけ。
**JetPack に入っている TensorRT 8.5 を、システムの Python 3.8 から使う**と、何も入れずに GPU で動く。

### GPU と CPU の結果は同じ
同じ 20 枚（そのときの Web カメラの画像）を両方にかけると、人の数は 20 枚中 19 枚で同じ、確からしさの差は 0.03 以下
（FP16 による差）。残りの 1 枚は、しきい値 0.25 すれすれの枠が GPU 側だけ 1 つ多かった。
保存した画像 1 枚（人が 4 人写っている）でも、CPU 0.91 / 0.65 / 0.50 / 0.47、GPU 0.91 / 0.65 / 0.51 / 0.47。

### 検出の数は部屋の様子で大きく変わる
GPU で測った 100 枚のうち、人を検出できたのは 2 枚だけだった。不具合ではない。そのときは人がノートPC に半分隠れて座っていて、
確からしさが 0.4〜0.55 だったため、しきい値 0.5 を超えたり超えなかったりしていた（CPU でも同じ時間帯は 30 枚中 8 枚）。
CPU で測ったとき（94〜100 / 100）は、手前と奥に人がはっきり写っていた。**速さ以外の数字は、同じ画像で比べること。**

## 手順

### ラボPC: ONNX とクラス名を書き出す

```bash
G1_HuggingFace/venv/bin/python Perception/real/export_onnx.py
```

→ `_local/perception/models/yolo26n.onnx`、`yolo26n.names.json`（TensorRT のエンジンにはクラス名が入らないので別のファイルにする）。

### PC2 へ写す

```bash
ssh unitree@192.168.0.82 'mkdir -p ~/perception/real'
scp -r Perception/common _local/perception/models/yolo26n.onnx _local/perception/models/yolo26n.names.json unitree@192.168.0.82:~/perception/
scp Perception/real/check_yolo.py unitree@192.168.0.82:~/perception/real/
```

### PC2: TensorRT のエンジンを作る（PC2 の上でしか作れない。約 12 分）

```bash
cd ~/perception && /usr/src/tensorrt/bin/trtexec --onnx=yolo26n.onnx --saveEngine=yolo26n.engine --fp16
```

今回の結果（`--warmUp=500 --iterations=200`、エンジン作成 714.6 秒）:

```
Throughput: 193.153 qps
Latency: mean = 5.5726 ms, percentile(90%) = 5.63477 ms
GPU Compute Time: mean = 5.16748 ms
H2D Latency: mean = 0.399201 ms
```

### PC2: 動かす

```bash
# CPU（conda の lerobot 環境の Python）
cd ~/perception && ~/miniforge3/envs/lerobot/bin/python real/check_yolo.py --webcam 6 --rotate cw --model yolo26n.onnx --frames 100 --threads 4
# GPU（システムの Python 3.8）
cd ~/perception && /usr/bin/python3 real/check_yolo.py --webcam 6 --rotate cw --backend tensorrt --model yolo26n.engine --frames 100
```

結果は `~/perception/_local/perception/check_yolo/<日時>/`（`summary.json` と、人が写った画像）。

## つまずいたこと

| 起きたこと | 原因 | どうしたか |
|---|---|---|
| ラボPC から 192.168.123.164 に SSH できない | その日は Wi-Fi（192.168.0.82）でつないでいた | `ps` で動いている `ssh` の宛先を見て分かった |
| ラボPC で `export_onnx.py` が `No module named 'onnx'` | `G1_HuggingFace/venv` に onnx が無く、ラボPC がインターネットにつながらなかった | PC2（インターネットにつながる）で `pip download` して `scp` で持ってきて、`pip install --no-index --find-links` |
| 持ってきた onnx が入らない（`from versions: none`） | **ラボPC（WSL）も aarch64**（Snapdragon）だった。x86_64 用を落としていた | PC2 で普通に `pip download`（PC2 も aarch64・Python 3.12 なので、そのまま合う） |
| `--webcam 6` で「カメラを開けない」 | 別の人のプロセス（`python3 -`）が `/dev/video6` を開いていた | 持ち主が止めてから測った。カメラは同時に 1 つのプロセスしか開けない |
| Python 3.8 で `common/` を読めない | `np.ndarray \| None` など 3.10 以降の型の書き方 | `common/` の 6 ファイルに `from __future__ import annotations` を足した |
| `trt.nptype()` が `module 'numpy' has no attribute 'bool'` | TensorRT 8.5 が、numpy 1.24 で無くなった `np.bool` を使っている | `trt_yolo.py` で型の対応を自分で持つ |

## 変えたコード

- `common/detector/trt_yolo.py`（新規）: TensorRT のエンジンで動かす検出器。前処理・後処理は `onnx_yolo.py` のものを使う。
  GPU のメモリの確保・転送は pycuda などを足さずに `ctypes` で `libcudart.so` を直接呼ぶ。TensorRT 8.5 の書き方（`execute_v2`）なので、
  JetPack を上げて TensorRT 10 になったら見直す
- `common/detector/__init__.py`: `build_detector` で `backend: tensorrt` を選べる
- `real/check_yolo.py`: `--backend tensorrt`、`--rotate cw|ccw|180` を追加
- `real/export_onnx.py`: `yolo26n.names.json`（クラス名）も書き出す
- `common/camera/*.py`（4 つ）、`common/detector/onnx_yolo.py`、`common/detector/types.py`: Python 3.8 で読めるよう
  `from __future__ import annotations` を追加（3.12 での動きは変わらない）

## 残っていること

- 人までの距離・姿勢を変えて、どこまで検出できるか（今回は、部屋にいた人が写っていたのを見ただけ）
- 頭カメラの配信・歩行と同時に動かしたときの速さ（今回は `rgbd_server.py` だけが同時に動いていた）
- GPU 版の前処理（約 11 ms）を減らす（縮小を GPU でする、など）
- `trt_yolo.py` のテストは無い（TensorRT のある PC2 でしか動かないため）。既存の `Perception/tests` の 3 ファイルは、
  変更後にラボPC で通ることを確かめた（`test_onnx_detector` 6 件、`test_zmq_camera` 3 件、`test_pipeline`）

## 追記: 人を検出したら知らせる・認識した映像を配信する（person_alert.py）

同じ日に、上の GPU 版を使って `real/person_alert.py` を作った。**前方のカメラで人を認識し、その映像をブラウザで見られることを
確認した**（ブラウザでの表示は、班のメンバーが目で見て確かめた）。使い方は `Perception/real/README.md` の person_alert の節。

| 機能 | 確かめたこと |
|---|---|
| 人を検出したら写真を残す | 人が 3 フレーム続いたら、枠を描いた写真と記録を G1 に保存。10 秒は出さない。写真は 1000 枚まで（`--max-photos 3` で古いものから消え、記録は残った） |
| PC へ送る | ZMQ 5560 番。60 秒で出来事 4 件、PC（`person_alert_receiver.py`）に 4 件とも届いた。PC が受け取っていなくても G1 には残った |
| 映像の配信 | `--stream 8080` → ブラウザで `http://192.168.0.82:8080/`。Wi-Fi で 10.0 枚/秒、1 枚約 52 KB、約 0.5 MB/s、2 人同時でも同じ。配信しながらでも検出は約 28 fps |
| **前方のカメラでの認識** | Mapping の配信（5555 番）から受け取り、40 秒で約 4.7 枚/秒、出来事 3 件。座っている 2 人を person 0.61・0.64 で検出。向き・色とも正しい |

前方のカメラは、Mapping の `~/mapping_tools/innomaker_camera_server.py` が開いている。そのため、カメラを直接は開かず、その配信を受け取る
（Mapping を止めずに使える。速さは Mapping の設定の 5 枚/秒）:

```bash
cd ~/perception && /usr/bin/python3 real/person_alert.py --zmq localhost:5555 --zmq-bgr --rotate 180 --backend tensorrt --model yolo26n.engine --stream 8080
```

### つまずいたこと（追記）

| 起きたこと | 原因 | どうしたか |
|---|---|---|
| ブラウザの映像が、前方ではなく腕のリストカメラだった | G1 の再起動（15:35）で `/dev/video` の番号が入れ替わった。腕のリストカメラ（SunplusIT の Full HD webcam）が video6 に、前方の Innomaker が video8 になり、`--webcam 6` のままだった | `--webcam` に `/dev/v4l/by-id/...` のパスを渡せるようにした |
| Mapping の配信をそのまま使うと、上下が逆さで色も逆になる | Mapping の配信は反時計回りに回してあり（正しい向きは時計回り）、色は OpenCV のままの BGR（`ZmqFrameSource` は RGB の前提） | `--rotate 180` と `--zmq-bgr`（BGR のまま使う）を足した |
| G1 に Wi-Fi で SSH できなくなった | 再起動のあと、G1 が先に見つかった `Fujitsu_free_Wi-Fi` に自動でつながった（Physical_AI_5G は優先度 100 だが、起動直後にまだ見えていなかったとみられる。一度つながると乗り換えない）。その間も Tailscale ではつながっていた | `Fujitsu_free_Wi-Fi` の自動接続を切った（`nmcli connection modify Fujitsu_free_Wi-Fi connection.autoconnect no`）。再起動での確認はまだ |

## 画像の置き場所

人が写っているので git には入れていない。ラボPC の次の場所にある:

- `_local/perception/webcam/20261031_152244_raw.jpg`（撮ったまま）・`_rotated.jpg`（回したもの）・`_yolo.jpg`（ラボPC の ultralytics で検出）
- `_local/perception/pc2_check_yolo/<日時>/`（PC2 の `check_yolo.py` の結果。日時は PC2 の時計で、ラボPC より約 1 時間遅れている）

## 追記（同日夜）: ONNX の書き出し方を直し、エンジンを作り直した

上で使った ONNX（end-to-end の出力、`nms=False`）は、元の `.pt` と確からしさが少しずれていた（bus.jpg の人が `.pt` 0.855 → 0.92）。
ここでの「GPU と CPU の結果は同じ」は同じ ONNX どうしの比べで、`.pt` とのずれには気づけなかった。
`real/export_onnx.py` を従来型の出力（`end2end=False`、NMS は `onnx_yolo.py`）に直して書き出し直し、PC2 のエンジンも作り直した。
直したあとは、TensorRT・ONNX とも `.pt` との差が 0.005 以下。GPU の 1 枚は約 20 ms（NMS を Python で行う分、17 ms から増えた）。
前のファイルは PC2 の `~/perception/old_end2end_20261031/` にある。詳しくは
`Button_Press/J1-gen/docs/reports/onboard_pc2_20261031.md` の 7 章。
