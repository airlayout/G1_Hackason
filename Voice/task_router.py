"""説明文と音声のコサイン類似度から、実行するタスクか見送りかを決める。"""

from __future__ import annotations

import json
import math
import re
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np


@dataclass(frozen=True)
class Task:
    task_id: str
    descriptions: tuple[str, ...]
    action: str | None
    reference_audio: tuple[str, ...] = ()
    arguments: dict = field(default_factory=dict)

    @property
    def reference_count(self) -> int:
        return len(self.reference_audio) if self.reference_audio else len(self.descriptions)


@dataclass(frozen=True)
class TaskCatalog:
    tasks: tuple[Task, ...]
    min_similarity: float
    min_margin: float

    @property
    def descriptions(self) -> tuple[str, ...]:
        return tuple(text for task in self.tasks for text in task.descriptions)

    @property
    def reference_count(self) -> int:
        return sum(task.reference_count for task in self.tasks)


@dataclass(frozen=True)
class Candidate:
    task_id: str
    score: float


@dataclass(frozen=True)
class Decision:
    task_id: str | None
    reason: str
    candidates: tuple[Candidate, ...]


def load_catalog(path: Path) -> TaskCatalog:
    data = json.loads(path.expanduser().read_text(encoding="utf-8"))
    if not isinstance(data, dict):
        raise ValueError("タスク設定は JSON オブジェクトで指定してください")
    thresholds = []
    for key, low, high in (("min_similarity", -1.0, 1.0), ("min_margin", 0.0, 2.0)):
        value = data.get(key)
        if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value) or not low <= value <= high:
            raise ValueError(f"{key} は {low}〜{high} の数値にしてください")
        thresholds.append(float(value))
    rows = data.get("tasks")
    if not isinstance(rows, list) or len(rows) < 2:
        raise ValueError("比較用に 2 件以上のタスク（見送り用も含む）を登録してください")
    tasks: list[Task] = []
    seen: set[str] = set()
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("タスクは JSON オブジェクトで指定してください")
        task_id, descriptions, action = row.get("id"), row.get("descriptions"), row.get("action")
        if not isinstance(task_id, str) or not re.fullmatch(r"[a-z][a-z0-9_]*", task_id) or task_id in seen:
            raise ValueError(f"タスク ID が不正、または重複しています: {task_id}")
        if not isinstance(descriptions, list) or not descriptions or not all(isinstance(s, str) and s.strip() for s in descriptions):
            raise ValueError(f"{task_id}: descriptions に空でない説明文を登録してください")
        if "action" not in row or (action is not None and (not isinstance(action, str) or not action)):
            raise ValueError(f"{task_id}: action は実行処理の名前か null にしてください")
        references = row.get("reference_audio", [])
        if not isinstance(references, list) or not all(isinstance(s, str) and s.strip() for s in references):
            raise ValueError(f"{task_id}: reference_audio は音声ファイルのパスの配列にしてください")
        audio_paths: list[str] = []
        for reference in references:
            audio_path = Path(reference).expanduser()
            if not audio_path.is_absolute():
                audio_path = path.expanduser().resolve().parent / audio_path
            if not audio_path.is_file():
                raise ValueError(f"{task_id}: 登録音声がありません: {audio_path}")
            audio_paths.append(str(audio_path.resolve()))
        arguments = row.get("arguments", {})
        if not isinstance(arguments, dict):
            raise ValueError(f"{task_id}: arguments はオブジェクトにしてください")
        json.dumps(arguments, allow_nan=False)
        tasks.append(Task(task_id, tuple(descriptions), action, tuple(audio_paths), arguments))
        seen.add(task_id)
    return TaskCatalog(tuple(tasks), *thresholds)


class TaskRouter:
    def __init__(self, catalog: TaskCatalog, document_embeddings: np.ndarray) -> None:
        self.catalog = catalog
        vectors = np.asarray(document_embeddings, dtype=np.float64)
        if vectors.ndim != 2 or vectors.shape[0] != catalog.reference_count or vectors.shape[1] == 0:
            raise ValueError("登録例とベクトルの件数・形状が一致しません")
        norms = np.linalg.norm(vectors, axis=1, keepdims=True)
        if not np.isfinite(vectors).all() or not np.isfinite(norms).all() or (norms <= 1e-9).any():
            raise ValueError("説明文のベクトルが不正です")
        self.vectors = vectors / norms

    def route(self, audio_embedding: np.ndarray) -> Decision:
        vector = np.asarray(audio_embedding, dtype=np.float64)
        if vector.shape != (self.vectors.shape[1],) or not np.isfinite(vector).all() or np.linalg.norm(vector) <= 1e-9:
            raise ValueError("音声ベクトルの形状または値が不正です")
        similarities = np.clip(self.vectors @ (vector / np.linalg.norm(vector)), -1.0, 1.0)
        candidates: list[Candidate] = []
        offset = 0
        for task in self.catalog.tasks:
            end = offset + task.reference_count
            candidates.append(Candidate(task.task_id, float(similarities[offset:end].max())))
            offset = end
        candidates.sort(key=lambda item: item.score, reverse=True)
        first, second = candidates[:2]
        if first.score < self.catalog.min_similarity:
            reason = "low_similarity"
        elif self.catalog.min_margin > 0 and first.score - second.score < self.catalog.min_margin:
            reason = "ambiguous"
        elif next(task for task in self.catalog.tasks if task.task_id == first.task_id).action is None:
            reason = "no_action"
        else:
            return Decision(first.task_id, "accepted", tuple(candidates))
        return Decision(None, reason, tuple(candidates))
