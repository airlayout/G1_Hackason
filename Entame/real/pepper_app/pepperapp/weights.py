"""Pinned YOLO11n weights (Ultralytics assets v8.3.0), verified by size and SHA256."""
import hashlib
import logging
import urllib.request
from pathlib import Path

MODEL_DIR = Path(__file__).resolve().parent.parent / ".cache" / "models"
RELEASE = "https://github.com/ultralytics/assets/releases/download/v8.3.0"
WEIGHTS = {
    "yolo11n.pt": (5_613_764, "0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1"),
    "yolo11n-pose.pt": (6_255_593, "869e83fcdffdc7371fa4e34cd8e51c838cc729571d1635e5141e3075e9319dc0"),
}

log = logging.getLogger(__name__)


def _verified(path: Path, size: int, sha256: str) -> bool:
    return (path.is_file() and path.stat().st_size == size
            and hashlib.sha256(path.read_bytes()).hexdigest() == sha256)


def ensure_weights(directory: Path = MODEL_DIR) -> dict[str, Path]:
    """Download missing weights. A file that fails verification is never kept."""
    directory.mkdir(parents=True, exist_ok=True)
    paths = {}
    for name, (size, sha256) in WEIGHTS.items():
        target = directory / name
        if not _verified(target, size, sha256):
            partial = target.with_suffix(".partial")
            urllib.request.urlretrieve(f"{RELEASE}/{name}", partial)
            if not _verified(partial, size, sha256):
                partial.unlink(missing_ok=True)
                raise RuntimeError(f"{name}: size or SHA256 mismatch")
            partial.replace(target)
        log.info("verified %s", target)
        paths[name] = target
    return paths


if __name__ == "__main__":
    logging.basicConfig(level=logging.INFO, format="[weights] %(message)s")
    ensure_weights()
