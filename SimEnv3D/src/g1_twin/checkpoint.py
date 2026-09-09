"""学習済み checkpoint の取得。

Isaac Sim の起動に依存しないため、単体でも import できる。
（エントリポイント側に置くと import 時に AppLauncher が二重起動するため分離した）
"""

from __future__ import annotations

import urllib.request
from pathlib import Path

from .policy import CHECKPOINT_URL

# 既定のキャッシュ先（SimEnvTest/checkpoints/）
DEFAULT_CACHE_DIR: Path = Path(__file__).resolve().parents[2] / "checkpoints"
DEFAULT_CACHE_NAME: str = "g1_flat_checkpoint.pt"


def resolve_checkpoint(explicit_path: str = "", cache_dir: Path | None = None) -> str:
    """checkpoint のパスを決める。無ければダウンロードしてキャッシュする。

    Args:
        explicit_path: 明示指定されたパス（空なら自動取得）
        cache_dir: キャッシュ先ディレクトリ（既定は SimEnvTest/checkpoints/）

    Returns:
        ローカルの checkpoint.pt のパス

    Raises:
        FileNotFoundError: 明示指定されたパスが存在しない場合
    """
    if explicit_path:
        path = Path(explicit_path)
        if not path.is_file():
            raise FileNotFoundError(f"[G1] checkpoint が見つかりません: {path}")
        return str(path)

    directory = cache_dir or DEFAULT_CACHE_DIR
    directory.mkdir(parents=True, exist_ok=True)
    cached = directory / DEFAULT_CACHE_NAME

    if cached.is_file():
        print(f"[OK] キャッシュ済みの checkpoint を使用します: {cached}")
        return str(cached)

    print(f"[INFO] checkpoint をダウンロードします: {CHECKPOINT_URL}")
    urllib.request.urlretrieve(CHECKPOINT_URL, cached)
    print(f"[OK] ダウンロード完了: {cached} ({cached.stat().st_size} bytes)")
    return str(cached)
