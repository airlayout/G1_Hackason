# PC2 で動かすときの注意（YOLO の ONNX、lowstate の受け取り）

G1 の PC2（Jetson Orin NX）で押す処理・認識を動かす準備（2026-10-31）で分かった注意点。経緯と数値は
[../reports/onboard_pc2_20261031.md](../../reports/onboard_pc2_20261031.md)。

## YOLO26 を ONNX にするときは `end2end=False`。確かめる相手は元の `.pt`

- **日付**: 2026-10-31
- **プロジェクト**: G1 ボタン押し（Button_Press/J1-gen）、Perception
- **前提バージョン**: ultralytics 8.4.164、onnxruntime 1.31（ノート PC）・1.29（PC2）、TensorRT 8.5（PC2）
- **状況**: `export(format="onnx", opset=17, simplify=False, nms=False)` で書き出したボタンの YOLO が、45 枚すべてで `.pt` と違う結果を出した
  （確からしさが 1.0 に張り付き、4 個が 2 個になる）。人の検出（yolo26n）も同じ書き方で、確からしさが 0.855 → 0.92 にずれていた
- **原因**: YOLO26 には end-to-end の出口（出力 (1, 300, 6)）と従来型の出口（(1, 4 + クラス数, 8400)）があり、`nms=False` だと end-to-end で書き出される。
  `.pt` の `predict` は従来型を使っている。opset・simplify は関係なかった。前の確認は「同じ ONNX を ultralytics で動かした結果」と比べていたので、気づけなかった
- **対処**: `export(format="onnx", imgsz=640, opset=17, end2end=False)`（NMS は `Perception/common/detector/onnx_yolo.py` が行う）。
  ボタンは `real/export_button_onnx.py`、人は `Perception/real/export_onnx.py`。どちらも `.pt` を `predict(..., rect=False)` で正方形に入れて比べる
- **教訓**: 形式を変えたモデルは、元のモデルと同じ入力で比べる。ultralytics を上げたら（`end2end=False` は「将来なくなる」と注意が出る）、書き出しと確認をやり直す

## lowstate をコールバックで全部受けると、それだけで 1 コアの 7 割を使う

- **日付**: 2026-10-31
- **プロジェクト**: G1 ボタン押し（Button_Press/J1-gen）
- **前提バージョン**: PC2（Jetson Orin NX）、Python 3.12、cyclonedds 0.10.2、unitree_sdk2py 1.0.1
- **状況**: PC2 で lowstate を受け取るだけのプロセスが 1 コアの約 73% を使った（lowstate は約 1,040 通 / 秒）
- **原因**: 変換（`parse_lowstate`）ではなく、SDK のコールバックが 1 通ごとに Python の関数を呼び、受信データを変換すること。
  何もしないコールバックでも 72% だった
- **対処**: `common/dds.py` の `LatestLowState`（最新 1 件だけ持つ DataReader。読むときに `take(N=1)`）を使う。50 Hz で読んで 10%。
  届いた数を数える `check_connection.py` だけは `LowStateReader(..., every_message=True)` で今までどおり
- **教訓**: 重いところは「何もしない版」を作って測り分ける。Yada さんの `real_g1.py`（エレベーター）は自分のコールバックで受けているので、PC2 で動かすときは同じ問題が出る
