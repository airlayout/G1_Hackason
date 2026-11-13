"""単発入力と常駐入力で共有する、登録例のベクトル化。"""

from __future__ import annotations

from pathlib import Path

import numpy as np

from Voice.audio_input import load_audio
from Voice.embedding_gemma import EmbeddingGemma
from Voice.task_router import TaskCatalog


def encode_references(encoder: EmbeddingGemma, catalog: TaskCatalog) -> np.ndarray:
    texts = [text for task in catalog.tasks if not task.reference_audio for text in task.descriptions]
    text_vectors = encoder.encode_documents(texts) if texts else None
    vectors = []
    offset = 0
    for task in catalog.tasks:
        if task.reference_audio:
            vectors.extend(encoder.encode_audio(load_audio(Path(path))) for path in task.reference_audio)
        else:
            end = offset + len(task.descriptions)
            vectors.extend(text_vectors[offset:end])
            offset = end
    return np.stack(vectors)
