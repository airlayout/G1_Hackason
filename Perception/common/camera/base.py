from abc import ABC, abstractmethod

import numpy as np


class FrameSource(ABC):
    """画像取得元を差し替え可能にするための共通インターフェース。

    ZMQカメラストリーム(sim/real共通)・ローカル動画ファイル・Webカメラは、
    すべてこのクラスを継承して実装する。パイプライン側はこのインターフェースのみに依存する。
    """

    @abstractmethod
    def open(self) -> None:
        """接続/デバイスオープンなどの初期化を行う"""
        raise NotImplementedError

    @abstractmethod
    def read(self) -> np.ndarray | None:
        """1フレーム分のBGR画像(numpy.ndarray)を返す。取得できない場合はNone"""
        raise NotImplementedError

    @abstractmethod
    def close(self) -> None:
        """リソースの解放を行う"""
        raise NotImplementedError

    def __enter__(self) -> "FrameSource":
        self.open()
        return self

    def __exit__(self, exc_type, exc_val, exc_tb) -> None:
        self.close()
