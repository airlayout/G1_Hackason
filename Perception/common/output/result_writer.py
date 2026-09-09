import csv
import json
from pathlib import Path
from typing import Any

from ..detector.yolo_detector import Detection


def build_detection_record(
    frame_index: int,
    timestamp: float,
    detections: list[Detection],
    processing_time_ms: float,
) -> dict[str, Any]:
    """1フレーム分の検出結果を、出力先を問わない共通の辞書表現に変換する。

    将来ROS2トピック送信やUnitree SDK経由でNav2/VLA側に渡す拡張をする際は、
    この関数の戻り値をそのまま使えるようにしてある。
    """
    return {
        "frame_index": frame_index,
        "timestamp": timestamp,
        "processing_time_ms": round(processing_time_ms, 2),
        "detections": [
            {
                "class_name": d.class_name,
                "confidence": round(d.confidence, 4),
                "bbox": [round(v, 1) for v in d.bbox],
            }
            for d in detections
        ],
    }


def print_console(record: dict[str, Any]) -> None:
    """検出結果をコンソールに表示する"""
    n = len(record["detections"])
    header = (
        f"[frame {record['frame_index']:06d}] "
        f"{record['processing_time_ms']:.1f}ms  detections={n}"
    )
    print(header)
    for d in record["detections"]:
        x1, y1, x2, y2 = d["bbox"]
        print(
            f"    {d['class_name']:<10} conf={d['confidence']:.2f} "
            f"bbox=({x1:.0f}, {y1:.0f}, {x2:.0f}, {y2:.0f})"
        )


def append_jsonl(path: Path, record: dict[str, Any]) -> None:
    """検出結果を1行1レコードのJSON Lines形式で追記する"""
    with path.open("a", encoding="utf-8") as f:
        f.write(json.dumps(record, ensure_ascii=False) + "\n")


def append_csv(path: Path, record: dict[str, Any]) -> None:
    """検出結果をCSVに追記する(1検出=1行、フレーム内に検出が0件の行も出力する)"""
    is_new_file = not path.exists()
    fieldnames = [
        "frame_index",
        "timestamp",
        "processing_time_ms",
        "class_name",
        "confidence",
        "x1",
        "y1",
        "x2",
        "y2",
    ]

    with path.open("a", encoding="utf-8", newline="") as f:
        writer = csv.DictWriter(f, fieldnames=fieldnames)
        if is_new_file:
            writer.writeheader()

        detections = record["detections"]
        if not detections:
            writer.writerow(
                {
                    "frame_index": record["frame_index"],
                    "timestamp": record["timestamp"],
                    "processing_time_ms": record["processing_time_ms"],
                    "class_name": "",
                    "confidence": "",
                    "x1": "",
                    "y1": "",
                    "x2": "",
                    "y2": "",
                }
            )
            return

        for d in detections:
            x1, y1, x2, y2 = d["bbox"]
            writer.writerow(
                {
                    "frame_index": record["frame_index"],
                    "timestamp": record["timestamp"],
                    "processing_time_ms": record["processing_time_ms"],
                    "class_name": d["class_name"],
                    "confidence": d["confidence"],
                    "x1": x1,
                    "y1": y1,
                    "x2": x2,
                    "y2": y2,
                }
            )


class ResultWriter:
    """設定(config["output"])に応じて、検出結果をconsole/JSON/CSVへ出力する"""

    def __init__(
        self,
        console: bool = True,
        save_json: bool = False,
        save_csv: bool = False,
        output_dir: str = "outputs",
        run_name: str = "run",
    ) -> None:
        self._console = console
        self._save_json = save_json
        self._save_csv = save_csv

        out_dir = Path(output_dir)
        out_dir.mkdir(parents=True, exist_ok=True)
        self._json_path = out_dir / f"{run_name}.jsonl"
        self._csv_path = out_dir / f"{run_name}.csv"

    def write(
        self,
        frame_index: int,
        timestamp: float,
        detections: list[Detection],
        processing_time_ms: float,
    ) -> dict[str, Any]:
        record = build_detection_record(frame_index, timestamp, detections, processing_time_ms)

        if self._console:
            print_console(record)
        if self._save_json:
            append_jsonl(self._json_path, record)
        if self._save_csv:
            append_csv(self._csv_path, record)

        return record
