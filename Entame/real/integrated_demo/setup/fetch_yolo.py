"""Fetch the recorded official YOLO11n release; hash bytes, never load the model."""
import hashlib
from pathlib import Path
import urllib.request

URL = 'https://github.com/ultralytics/assets/releases/download/v8.3.0/yolo11n.pt'
SHA256 = '0ebbc80d4a7680d14987a577cd21342b65ecfd94632bd9a8da63ae6417644ee1'
SIZE = 5613764
DEST = Path(__file__).resolve().parents[1] / 'g1-bottle-reaction/.runtime/models/yolo11n.pt'


def main():
    data = DEST.read_bytes() if DEST.exists() else urllib.request.urlopen(URL, timeout=60).read(SIZE + 1)
    if len(data) != SIZE or hashlib.sha256(data).hexdigest() != SHA256:
        raise SystemExit('YOLO size/hash mismatch; no file written')
    DEST.parent.mkdir(parents=True, exist_ok=True)
    if not DEST.exists():
        DEST.write_bytes(data)
    print('YOLO WEIGHT RECONSTRUCTIBLE: recorded size/SHA256 PASS')


if __name__ == '__main__':
    main()
