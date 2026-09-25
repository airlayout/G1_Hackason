#!/usr/bin/env python3
"""記録の地図が **どの座標系** に乗っているかを出す（タグの登録と照合する名前）。

- 別の地図の座標系へ移した記録（`apply_transform_to_session.py`、案 A）は、
  `manifest.json` の `aligned_to`（なければ `diagnostic.aligned_to`）がその名前
- それ以外は記録自身のセッション名

`--write` で `<session>/map/map_frame.txt` に書く。PC2 へ地図を配るときに
`~/g1_cfg/map/map_frame.txt` として一緒に送り、`live/up.sh` が
`localize_from_tags.py --map-frame` に渡す。registry の `map_frame` と違えば起動が止まる。

    python3 quickstart/map_frame.py runs/20260911T201836_wireless_dualcam_map01 --write
"""
from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path


def map_frame(session: Path) -> str:
    manifest = session / "manifest.json"
    if manifest.exists():
        meta = json.loads(manifest.read_text(encoding="utf-8"))
        aligned = meta.get("aligned_to") or meta.get("diagnostic", {}).get("aligned_to")
        if aligned:
            return str(aligned)
    return session.resolve().name


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session_dir", type=Path)
    ap.add_argument("--write", action="store_true", help="map/map_frame.txt に書く")
    a = ap.parse_args()
    if not a.session_dir.is_dir():
        print(f"[NG] 記録がありません: {a.session_dir}", file=sys.stderr)
        return 1
    name = map_frame(a.session_dir)
    print(name)
    if a.write:
        out = a.session_dir / "map" / "map_frame.txt"
        out.write_text(name + "\n", encoding="utf-8")
        print(f"[OK] {out}", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
