#!/usr/bin/env python3
"""rosbag2(sqlite3) の `/rosout` を **ROS 無しで**読む。記録を普通のログに戻す。

    python3 bag_read_rosout.py <bag>_0.db3 > run.log

## なぜ要るのか（2026-09-25）

実機の launch ログは `/tmp/nav_launch.log` に出るが、**PC2 を再起動すると消える**
（実際に失った）。一方 `runs/*/` の記録には `/rosout` が入っているので、
**そこからログ本文を丸ごと復元できる**。この道具でその日の原因究明ができた。

⚠️ **`metadata.yaml` が無くても読める。** 記録が電源断で切られると
`metadata.yaml` が書かれないが、`.db3` さえあればここは動く
（`ros2 bag` で開きたいときだけ `ros2 bag reindex -s sqlite3` が要る）。

📌 操作PC に ROS は入っていないので、**手元で解析できる**のが利点。
   同じ手口で `/odom` `/tf` `/cmd_vel_smoothed` も読める（`_local/logs_20260925/` の
   スクリプト群が実例）。

## CDR の剥がし方（はまりどころ）

- 先頭 **4 バイト**が CDR ヘッダ（representation id + options）。以降リトルエンディアン
- 文字列は「長さ(uint32、**NUL 込み**)＋本体」
- ⚠️ **各フィールドは自分の大きさに整列する。** 整列の基準は**ヘッダの直後**なので
  `(o - 4) % n` で測る。ここを間違えると静かに値がずれる
"""
import sqlite3
import struct
import sys
from datetime import datetime

LEVELS = {10: "DEBUG", 20: "INFO", 30: "WARN", 40: "ERROR", 50: "FATAL"}


class R:
    def __init__(self, b: bytes):
        self.b, self.o = b, 4          # 先頭4バイトは CDR ヘッダ

    def align(self, n: int) -> None:
        self.o += (-(self.o - 4)) % n

    def u8(self) -> int:
        v = self.b[self.o]; self.o += 1; return v

    def u32(self) -> int:
        self.align(4)
        v = struct.unpack_from("<I", self.b, self.o)[0]; self.o += 4; return v

    def i32(self) -> int:
        self.align(4)
        v = struct.unpack_from("<i", self.b, self.o)[0]; self.o += 4; return v

    def s(self) -> str:
        n = self.u32()
        v = self.b[self.o:self.o + n - 1].decode("utf-8", "replace"); self.o += n
        return v


def main() -> None:
    con = sqlite3.connect(sys.argv[1] if len(sys.argv) > 1 else "run2.db3")
    tid = con.execute("SELECT id FROM topics WHERE name='/rosout'").fetchone()[0]
    for ts, data in con.execute(
            "SELECT timestamp,data FROM messages WHERE topic_id=? ORDER BY timestamp", (tid,)):
        r = R(data)
        r.i32(); r.u32()            # stamp.sec, stamp.nanosec
        lvl = r.u8()
        name = r.s(); msg = r.s()
        t = datetime.fromtimestamp(ts / 1e9).strftime("%H:%M:%S")
        print(f"{t} [{LEVELS.get(lvl, lvl)}] [{name}] {msg}")


main()
