import time
from dataclasses import dataclass


@dataclass
class Timer:
    """1フレームの処理時間(ミリ秒)を計測するための単純なストップウォッチ

    使い方:
        with Timer() as t:
            ...何か処理...
        print(t.elapsed_ms)
    """

    _start: float = 0.0
    elapsed_ms: float = 0.0

    def __enter__(self) -> "Timer":
        self._start = time.perf_counter()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.elapsed_ms = (time.perf_counter() - self._start) * 1000.0
