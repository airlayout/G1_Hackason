from __future__ import annotations

import ast
from pathlib import Path

import cv2
import numpy as np

from .types import Detection

# NOTE(なぜONNXか): G1のPC2(Jetson Orin NX、Ubuntu 20.04、aarch64)では、Python 3.12用のtorchはCPU版しか無く大きい。
# ultralyticsで書き出したONNXをonnxruntime(CPU)で動かせば、torch/ultralyticsを入れずにYOLOを動かせる
# (onnxruntimeのaarch64用の配布物はPC2のglibc 2.31で動くことを、2026-10-22にラボPCで配布物の中を調べて確かめた)。
#
# 書き出し(ラボPCで、ultralyticsとonnxが入った環境で。Perception/real/export_onnx.py):
#   YOLO("yolo26n.pt").export(format="onnx", imgsz=640, simplify=False, opset=17, nms=False)
# 出力は2つの形がある。どちらも読める:
#   - end-to-end(既定、メタデータ end2end=True): (1, 300, 6) = [x1, y1, x2, y2, 確からしさ, クラス](入力640x640の画素)。
#     YOLO26 はNMSを使わない作りなので、そのまま使う
#   - 従来型(end2end=False): (1, 4 + クラス数, 候補数) = (1, 84, 8400)。先頭4つが中心x・中心y・幅・高さ、残りがクラスごとの
#     確からしさ。重なった枠をまとめる処理(NMS)をここで行う
# 2026-10-22 に bus.jpg・zidane.jpg で、同じONNXをultralyticsで動かした結果と比べ、どちらの形も枠・確からしさが一致した。
# .pt をultralyticsで動かした結果とは少し違う(人の数は同じ、IoU 0.91以上、確からしさの差 最大0.13)。.pt は縦横比に合わせた
# 大きさ(例: 640x480)で推論し、ONNX は640x640に固定(余りを灰色で埋める)なので、入力が違うため


def letterbox(image: np.ndarray, size: int) -> tuple[np.ndarray, float, tuple[float, float]]:
    """縦横比を保ったまま size x size に縮め、余りを灰色(114)で埋める(ultralyticsの前処理と同じ)。

    返すのは (埋めた画像, 縮めた倍率, 左・上に足した余白)。
    """
    h, w = image.shape[:2]
    r = min(size / h, size / w)
    new_w, new_h = int(round(w * r)), int(round(h * r))
    pad_x, pad_y = (size - new_w) / 2, (size - new_h) / 2
    if (new_w, new_h) != (w, h):
        image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_LINEAR)
    top, bottom = int(round(pad_y - 0.1)), int(round(pad_y + 0.1))
    left, right = int(round(pad_x - 0.1)), int(round(pad_x + 0.1))
    out = cv2.copyMakeBorder(image, top, bottom, left, right, cv2.BORDER_CONSTANT, value=(114, 114, 114))
    return out, r, (left, top)


def nms(boxes: np.ndarray, scores: np.ndarray, iou_threshold: float) -> list[int]:
    """重なった枠をまとめる(確からしさの高い順に残し、IoUがiou_threshold以上の枠を消す)。boxesは (N, 4) のxyxy。"""
    order = np.argsort(-scores)
    keep: list[int] = []
    areas = np.maximum(boxes[:, 2] - boxes[:, 0], 0) * np.maximum(boxes[:, 3] - boxes[:, 1], 0)
    while order.size:
        i = int(order[0])
        keep.append(i)
        rest = order[1:]
        xx1 = np.maximum(boxes[i, 0], boxes[rest, 0])
        yy1 = np.maximum(boxes[i, 1], boxes[rest, 1])
        xx2 = np.minimum(boxes[i, 2], boxes[rest, 2])
        yy2 = np.minimum(boxes[i, 3], boxes[rest, 3])
        inter = np.maximum(xx2 - xx1, 0) * np.maximum(yy2 - yy1, 0)
        iou = inter / np.maximum(areas[i] + areas[rest] - inter, 1e-9)
        order = rest[iou < iou_threshold]
    return keep


def postprocess_end2end(
    output: np.ndarray,
    names: dict[int, str],
    class_ids: list[int] | None,
    confidence_threshold: float,
    scale: float,
    pad: tuple[float, float],
    image_shape: tuple[int, int],
) -> list[Detection]:
    """end-to-endの出力 (1, 候補数, 6) = [x1, y1, x2, y2, 確からしさ, クラス] を、元の画像の画素のDetectionにする(NMSは不要)。"""
    pred = output[0]
    sel = pred[:, 4] >= confidence_threshold
    if class_ids is not None:
        sel &= np.isin(pred[:, 5].astype(int), class_ids)
    h, w = image_shape
    out: list[Detection] = []
    for x1, y1, x2, y2, conf, cls in pred[sel]:
        x1, x2 = (np.clip((v - pad[0]) / scale, 0, w) for v in (x1, x2))
        y1, y2 = (np.clip((v - pad[1]) / scale, 0, h) for v in (y1, y2))
        out.append(Detection(class_name=names[int(cls)], confidence=float(conf),
                             bbox=(float(x1), float(y1), float(x2), float(y2))))
    return sorted(out, key=lambda d: -d.confidence)


def postprocess(
    output: np.ndarray,
    names: dict[int, str],
    class_ids: list[int] | None,
    confidence_threshold: float,
    iou_threshold: float,
    scale: float,
    pad: tuple[float, float],
    image_shape: tuple[int, int],
    max_det: int = 300,
) -> list[Detection]:
    """従来型のONNXの出力 (1, 4 + クラス数, 候補数) を、元の画像の画素のDetectionにする(NMSを行う)。"""
    pred = output[0].T  # (候補数, 4 + クラス数)
    cls_scores = pred[:, 4:]
    if class_ids is not None:
        mask = np.zeros(cls_scores.shape[1], dtype=bool)
        mask[class_ids] = True
        cls_scores = np.where(mask[None, :], cls_scores, 0.0)
    cls = np.argmax(cls_scores, axis=1)
    conf = cls_scores[np.arange(len(cls)), cls]
    sel = conf >= confidence_threshold
    if not np.any(sel):
        return []
    xywh, cls, conf = pred[sel, :4], cls[sel], conf[sel]
    xyxy = np.stack([xywh[:, 0] - xywh[:, 2] / 2, xywh[:, 1] - xywh[:, 3] / 2,
                     xywh[:, 0] + xywh[:, 2] / 2, xywh[:, 1] + xywh[:, 3] / 2], axis=1)
    # クラスごとにNMSをかける(クラスごとに座標をずらして、違うクラスの枠どうしは重ならないようにする)
    offset = cls[:, None].astype(np.float32) * 7680.0
    keep = nms(xyxy + offset, conf, iou_threshold)[:max_det]
    h, w = image_shape
    out: list[Detection] = []
    for i in keep:
        x1, y1, x2, y2 = xyxy[i]
        x1, x2 = (np.clip((v - pad[0]) / scale, 0, w) for v in (x1, x2))
        y1, y2 = (np.clip((v - pad[1]) / scale, 0, h) for v in (y1, y2))
        out.append(Detection(class_name=names[int(cls[i])], confidence=float(conf[i]),
                             bbox=(float(x1), float(y1), float(x2), float(y2))))
    return out


class OnnxYoloDetector:
    """ultralyticsで書き出したYOLOのONNXを、onnxruntime(CPU)で動かす検出器。YoloDetectorと同じdetect()を持つ。"""

    def __init__(
        self,
        model_path: str = "yolo26n.onnx",
        classes: list[str] | None = None,
        confidence_threshold: float = 0.5,
        iou_threshold: float = 0.7,
        num_threads: int | None = None,
    ) -> None:
        try:
            import onnxruntime as ort
        except ImportError as e:
            raise RuntimeError("onnxruntime がインストールされていません(`pip install onnxruntime`)") from e
        if not Path(model_path).exists():
            raise FileNotFoundError(f"ONNXのモデルが無い: {model_path}(ラボPCで yolo26n.pt から書き出して置く)")
        opts = ort.SessionOptions()
        if num_threads:
            opts.intra_op_num_threads = num_threads
        self._session = ort.InferenceSession(str(model_path), sess_options=opts, providers=["CPUExecutionProvider"])
        self._input_name = self._session.get_inputs()[0].name
        self._imgsz = int(self._session.get_inputs()[0].shape[2])
        meta = self._session.get_modelmeta().custom_metadata_map
        self.names: dict[int, str] = ast.literal_eval(meta["names"]) if "names" in meta else {}
        out_shape = self._session.get_outputs()[0].shape
        self.end2end = meta.get("end2end", "False") == "True" or (len(out_shape) == 3 and out_shape[-1] == 6)
        self._confidence_threshold = confidence_threshold
        self._iou_threshold = iou_threshold
        self._class_ids: list[int] | None = None
        if classes:
            name_to_id = {name: idx for idx, name in self.names.items()}
            missing = [c for c in classes if c not in name_to_id]
            if missing:
                raise ValueError(f"モデルに存在しないクラス名が指定されました: {missing}")
            self._class_ids = [name_to_id[c] for c in classes]

    @property
    def device(self) -> str:
        return f"cpu(onnxruntime{', end-to-end' if self.end2end else ', NMS'})"

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """1フレーム(BGR)に対して推論を行い、Detectionのリストを返す"""
        image, scale, pad = letterbox(frame, self._imgsz)
        blob = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        output = self._session.run(None, {self._input_name: blob})[0]
        if self.end2end:
            return postprocess_end2end(output, self.names, self._class_ids, self._confidence_threshold,
                                       scale, pad, frame.shape[:2])
        return postprocess(output, self.names, self._class_ids, self._confidence_threshold, self._iou_threshold,
                           scale, pad, frame.shape[:2])
