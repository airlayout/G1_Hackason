from __future__ import annotations

import ctypes
import json
from pathlib import Path

import cv2
import numpy as np

from .onnx_yolo import letterbox, postprocess, postprocess_end2end
from .types import Detection

# NOTE(なぜTensorRTか): G1のPC2(Jetson Orin NX、JetPack 5.1.1 = L4T R35.3.1)のGPUは、Python 3.12用のtorch(CUDA 13向け)では
# ドライバが古くて使えない(2026-10-31 確認)。JetPackに入っている TensorRT 8.5.2 なら GPU で動かせる。TensorRT の Python
# バインディングはシステムの Python 3.8(/usr/bin/python3)にだけ入っているので、これはそこで動かす(common/ は3.8でも読めるようにしてある)。
#
# エンジンはPC2の上で、ONNXから trtexec で作る(PC2のGPU・TensorRTの版に合わせたものになるので、ラボPCでは作れない):
#   /usr/src/tensorrt/bin/trtexec --onnx=yolo26n.onnx --saveEngine=yolo26n.engine --fp16
# エンジンにはクラス名が入らないので、Perception/real/export_onnx.py が書き出す yolo26n.names.json を隣に置く。
#
# GPUのメモリの確保・転送は、pycuda などを足さずに済むよう ctypes で CUDA ランタイム(libcudart)を直接呼ぶ。
# 入出力の扱いは TensorRT 8.5 の bindings の書き方(execute_v2)。TensorRT 10 では使えなくなる書き方なので、JetPack を上げたら見直す。

_H2D, _D2H = 1, 2  # cudaMemcpyKind


def _load_cudart() -> ctypes.CDLL:
    for name in ("libcudart.so", "/usr/local/cuda/lib64/libcudart.so"):
        try:
            return ctypes.CDLL(name)
        except OSError:
            continue
    raise RuntimeError("CUDA ランタイム(libcudart.so)が見つからない(JetPack の CUDA が入っているか、/usr/local/cuda/lib64 を確かめる)")


class TrtYoloDetector:
    """ultralyticsで書き出したYOLOのONNXから作ったTensorRTのエンジンを、GPUで動かす検出器。OnnxYoloDetectorと同じdetect()を持つ。"""

    def __init__(
        self,
        engine_path: str = "yolo26n.engine",
        classes: list[str] | None = None,
        confidence_threshold: float = 0.5,
        iou_threshold: float = 0.7,
        names_path: str | None = None,
    ) -> None:
        try:
            import tensorrt as trt
        except ImportError as e:
            raise RuntimeError("tensorrt を import できない(PC2ではシステムの /usr/bin/python3 で動かす)") from e
        engine_path = Path(engine_path)
        if not engine_path.exists():
            raise FileNotFoundError(f"TensorRTのエンジンが無い: {engine_path}(PC2の上で trtexec で ONNX から作る)")
        names_path = Path(names_path) if names_path else engine_path.with_suffix(".names.json")
        if not names_path.exists():
            raise FileNotFoundError(f"クラス名のファイルが無い: {names_path}(export_onnx.py が ONNX の隣に書き出す)")
        self.names: dict[int, str] = {int(k): v for k, v in json.loads(names_path.read_text()).items()}

        self._cudart = _load_cudart()
        self._cudart.cudaMalloc.argtypes = [ctypes.POINTER(ctypes.c_void_p), ctypes.c_size_t]
        self._cudart.cudaMemcpy.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_size_t, ctypes.c_int]
        self._cudart.cudaFree.argtypes = [ctypes.c_void_p]

        self._logger = trt.Logger(trt.Logger.WARNING)
        self._runtime = trt.Runtime(self._logger)
        self._engine = self._runtime.deserialize_cuda_engine(engine_path.read_bytes())
        if self._engine is None:
            raise RuntimeError(f"エンジンを読めない: {engine_path}(別の TensorRT の版・別のGPUで作ったものは使えない)")
        self._context = self._engine.create_execution_context()

        # 入出力ごとに、ホスト側の配列とGPU側のメモリを1つずつ用意しておく(毎フレーム確保し直さない)
        self._host: list[np.ndarray] = []
        self._device: list[ctypes.c_void_p] = []
        self._input_index = self._output_index = -1
        for i in range(self._engine.num_bindings):
            shape = tuple(self._engine.get_binding_shape(i))
            # trt.nptype() は TensorRT 8.5 では np.bool(numpy 1.24 で無くなった)を使っていて落ちるので、自分で対応させる
            dtype = {trt.float32: np.float32, trt.float16: np.float16, trt.int32: np.int32,
                     trt.int8: np.int8, trt.bool: np.bool_}[self._engine.get_binding_dtype(i)]
            host = np.empty(shape, dtype=dtype)
            ptr = ctypes.c_void_p()
            self._check(self._cudart.cudaMalloc(ctypes.byref(ptr), host.nbytes), "cudaMalloc")
            self._host.append(host)
            self._device.append(ptr)
            if self._engine.binding_is_input(i):
                self._input_index = i
            elif self._output_index < 0:
                self._output_index = i
        self._bindings = [int(p.value) for p in self._device]
        self._imgsz = int(self._host[self._input_index].shape[2])
        out_shape = self._host[self._output_index].shape
        self.end2end = len(out_shape) == 3 and out_shape[-1] == 6

        self._confidence_threshold = confidence_threshold
        self._iou_threshold = iou_threshold
        self._class_ids: list[int] | None = None
        if classes:
            name_to_id = {name: idx for idx, name in self.names.items()}
            missing = [c for c in classes if c not in name_to_id]
            if missing:
                raise ValueError(f"モデルに存在しないクラス名が指定されました: {missing}")
            self._class_ids = [name_to_id[c] for c in classes]

    @staticmethod
    def _check(err: int, what: str) -> None:
        if err != 0:
            raise RuntimeError(f"{what} が失敗した(cudaError {err})")

    @property
    def device(self) -> str:
        return f"cuda(tensorrt{', end-to-end' if self.end2end else ', NMS'})"

    def detect(self, frame: np.ndarray) -> list[Detection]:
        """1フレーム(BGR)に対して推論を行い、Detectionのリストを返す"""
        image, scale, pad = letterbox(frame, self._imgsz)
        blob = cv2.cvtColor(image, cv2.COLOR_BGR2RGB).transpose(2, 0, 1)[None].astype(np.float32) / 255.0
        host_in, host_out = self._host[self._input_index], self._host[self._output_index]
        np.copyto(host_in, blob, casting="same_kind")
        self._check(self._cudart.cudaMemcpy(self._device[self._input_index], host_in.ctypes.data, host_in.nbytes, _H2D),
                    "cudaMemcpy(入力)")
        if not self._context.execute_v2(self._bindings):
            raise RuntimeError("TensorRT の推論(execute_v2)が失敗した")
        self._check(self._cudart.cudaMemcpy(host_out.ctypes.data, self._device[self._output_index], host_out.nbytes, _D2H),
                    "cudaMemcpy(出力)")
        output = host_out.astype(np.float32)
        if self.end2end:
            return postprocess_end2end(output, self.names, self._class_ids, self._confidence_threshold,
                                       scale, pad, frame.shape[:2])
        return postprocess(output, self.names, self._class_ids, self._confidence_threshold, self._iou_threshold,
                           scale, pad, frame.shape[:2])

    def close(self) -> None:
        for ptr in self._device:
            self._cudart.cudaFree(ptr)
        self._device = []
