"""占有格子を「自由 / 占有 / 未知」の3値にそろえ、PNG と照合用の指紋を作る。

⚠️ **標準ライブラリだけで書いてある。** このファイルは PC2 にも配って `ros_adapter.py`
から使う。PC2 の pixi 環境に OpenCV / numpy がある保証は無いので、依存を持ち込まない
（`map_web.py` が同じ理由で依存を避けている）。

3値の符号:
    FREE=0  OCCUPIED=1  UNKNOWN=2
セルの並びは **OccupancyGrid と同じく 0 行目が下端（原点側）**。PNG は 0 行目が上端なので
`png_from_cells` の中で上下を反転する。

指紋（`cells_digest`）は、UI が表示している地図と Nav2 が使っている地図が同じかを
確かめるためのもの。⚠️ **大きさと原点が同じでも中身が違う地図はありうる**
（同じ範囲で取り直した地図など）。それを見分けるために中身まで比べる。
"""
from __future__ import annotations

import hashlib
import struct
import zlib
from typing import Iterable

FREE, OCCUPIED, UNKNOWN = 0, 1, 2

# 描画色。mapimage.py（yaml から読む側）と同じ3色にそろえる
_COLORS: dict[int, bytes] = {
    FREE: bytes((0xE8, 0xE8, 0xE4)),
    OCCUPIED: bytes((0x1A, 0x1A, 0x1A)),
    UNKNOWN: bytes((0x44, 0x44, 0x44)),
}

# OccupancyGrid の値（-1 / 0..100）を3値に落とす境目。ros_adapter.py の従来の描画と同じ
FREE_BELOW = 20        # 0 <= v < 20 なら自由
OCCUPIED_ABOVE = 65    # v > 65 なら占有


def cells_from_occupancy(data: Iterable[int]) -> bytes:
    """OccupancyGrid.data（int8 の並び）を3値の bytes にする。"""
    out = bytearray()
    for v in data:
        if 0 <= v < FREE_BELOW:
            out.append(FREE)
        elif v > OCCUPIED_ABOVE:
            out.append(OCCUPIED)
        else:
            out.append(UNKNOWN)
    return bytes(out)


def cells_digest(cells: bytes, width: int, height: int) -> str:
    """3値の格子の指紋。大きさも含めて混ぜる（同じ中身で縦横が違う地図を区別する）。"""
    h = hashlib.sha1()
    h.update(struct.pack("<II", width, height))
    h.update(cells)
    return h.hexdigest()[:16]


def png_from_cells(cells: bytes, width: int, height: int) -> bytes:
    """3値の格子を RGB の PNG にする。

    ⚠️ **パレット形式にしない。** `Adapter/mock/check.py` が PNG を自前で RGB として
    読んで上下の向きを確かめているため、形式を変えるとそちらが黙って壊れる。
    """
    if len(cells) != width * height:
        raise ValueError(f"セル数が合わない: {len(cells)} != {width}x{height}")
    rows = []
    # OccupancyGrid の 0 行目は下端。PNG は上端から書くので逆順に積む
    for r in range(height - 1, -1, -1):
        row = cells[r * width:(r + 1) * width]
        rows.append(b"\x00" + b"".join(_COLORS[c] for c in row))   # 行頭はフィルタ無し(0)
    raw = b"".join(rows)

    def chunk(kind: bytes, body: bytes) -> bytes:
        return (struct.pack(">I", len(body)) + kind + body
                + struct.pack(">I", zlib.crc32(kind + body) & 0xFFFFFFFF))

    ihdr = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)   # 8bit / RGB
    return (b"\x89PNG\r\n\x1a\n" + chunk(b"IHDR", ihdr)
            + chunk(b"IDAT", zlib.compress(raw, 6)) + chunk(b"IEND", b""))
