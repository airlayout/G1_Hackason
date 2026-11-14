"""実機・模擬環境で共用するエレベーターボタンYOLO（BGR入力）。"""
from pathlib import Path

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_BUTTON_WEIGHTS = REPO_ROOT / "Button_Press/J1-gen/weights/button_final8.pt"
DEFAULT_CONFIDENCE = 0.4
DEFAULT_IMAGE_SIZE = 640


class ButtonDetector:
    def __init__(self, detector, *, direction=None, class_name=None):
        names = set(detector.class_names)
        allowed = ({class_name} if class_name else names & {"up", "down", "button"})
        if not allowed or not allowed <= names:
            raise ValueError(f"ボタンクラスがありません: 指定={class_name}, モデル={sorted(names)}")
        if direction is not None:
            if direction not in ("up", "down") or direction not in allowed:
                raise ValueError("--button-directionにはup/downを識別する重みが必要です。"
                                 "button単一クラスの重みでは--target-pixelで選択してください")
            allowed = {direction}
        self.detector = detector
        self.class_names = tuple(sorted(allowed))

    @property
    def device(self):
        return self.detector.device

    def detect(self, bgr):
        return [d for d in self.detector.detect(bgr) if d.class_name in self.class_names]


def make_button_detector(weights=None, confidence=DEFAULT_CONFIDENCE, device="auto", *,
                         runtime=None, imgsz=DEFAULT_IMAGE_SIZE, direction=None, class_name=None):
    """モデルを一度読み込み、最初の推論を制御開始前に済ませる。"""
    path = Path(weights).expanduser() if weights else DEFAULT_BUTTON_WEIGHTS
    if not path.is_file():
        raise FileNotFoundError(f"ボタンYOLOの重みがありません: {path}")
    if not np.isfinite(confidence) or not 0 < confidence < 1:
        raise ValueError("confidenceは0～1の間が必要です")
    if runtime is None:
        from real.j1gen_camera import J1CameraRuntime
        runtime = J1CameraRuntime()
    bridge = runtime.module('perception_bridge')
    if path.suffix.lower() == '.onnx':
        cls = bridge.perception('detector.onnx_yolo').OnnxYoloDetector
        backend = cls(model_path=str(path.resolve()),classes=None,confidence_threshold=confidence,num_threads=2)
        backend.class_names = tuple(backend.names.values())
    else:
        cls = bridge.perception('detector.yolo_detector').YoloDetector
        backend = cls(model_name=str(path.resolve()),classes=None,confidence_threshold=confidence,device=device,imgsz=imgsz)
    detector = ButtonDetector(backend,direction=direction,class_name=class_name)
    detector.detect(np.zeros((480, 640, 3), dtype=np.uint8))
    return detector


def direction_detections(detections, direction):
    """シムの旧buttonモデルは既存の幾何選択へ渡す。上下モデルは方向も照合する。"""
    return [d for d in detections if d.class_name in (direction, "button")]


def add_sim_detector_arguments(parser, *, fallback="oracle"):
    parser.add_argument("--weights", help="ボタン用YOLO .pt。--detector yoloのみなら同梱のfinal8")
    parser.add_argument("--detector", choices=(fallback, "yolo"),
                        help=f"省略時は--weightsがあればyolo、なければ{fallback}")
    parser.add_argument("--confidence", type=float, default=DEFAULT_CONFIDENCE)
    parser.add_argument("--device", default="cpu")
    parser.add_argument("--detector-imgsz", type=int, default=DEFAULT_IMAGE_SIZE)


def resolve_sim_weights(args, *, fallback="oracle"):
    """YOLOは明示選択されたときだけ有効にする。重みの指定ミスで試験方式を変えない。"""
    mode = getattr(args, "detector", None) or ("yolo" if args.weights else fallback)
    if mode != "yolo":
        if args.weights:
            raise ValueError(f"--detector {mode} と --weights は同時に指定できません")
        return None
    return str(Path(args.weights).expanduser().resolve() if args.weights else DEFAULT_BUTTON_WEIGHTS)
