"""頭のライダーの記録（record_dds_to_bag.py の db3）から前方だけを切り出し、door_lidar_angle.py・door_lidar_frame.py が読む npz にする。

10-16・17 は PC2 で切り出していたが、その道具は残っていなかった（2026-10-21 に Mac 側で書き直した）。
db3 は SQLite、点群は CDR なので、g1_mapping/rebuild.py の parse_pointcloud2 で配置を読み、numpy でまとめて取り出す。

出力（ライダーの座標のまま。上下逆さまの取付は door_lidar_angle.py が戻す）:
  xyz_mm   (N, 3) int16  前方の箱に入った点 [mm]
  counts   (回の数,) int32  回ごとの点の数
  stamp_ns (回の数,) int64  点群のヘッダの時刻（ライダーの時計）
  recv_ns  (回の数,) int64  PC2 が受け取った時刻（db3 の timestamp。/dog_odom の記録と突き合わせる時計）

使い方: Navigation/.venv/bin/python Door/real/crop_lidar_bag.py <セッションの場所> <出力.npz>
"""
from __future__ import annotations

import argparse
import sqlite3
import struct
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[2] / "Mapping" / "real" / "python"))
from g1_mapping.rebuild import find_bag_files, parse_pointcloud2  # noqa: E402

LIDAR_TOPIC = "/utlidar/cloud_livox_mid360"
# 前方の箱（ライダーの座標。取付は上下逆さまだが x は前のまま）
X_MIN, X_MAX = 0.0, 4.0  # [m]
Y_ABS_MAX = 2.5  # [m]


def scan_xyz(payload: bytes) -> np.ndarray:
    """1 回分の点群の有限な点を (n, 3) で全部返す（ライダーの座標）"""
    layout = parse_pointcloud2(payload)
    n = min(layout.point_count, (len(payload) - layout.data_start) // layout.point_step)
    raw = np.frombuffer(payload, dtype=np.uint8, count=n * layout.point_step, offset=layout.data_start)
    rec = raw.reshape(n, layout.point_step)
    dtype = np.dtype("<f4") if layout.format_code == "f" else np.dtype("<f8")
    width = dtype.itemsize
    cols = [rec[:, o:o + width].copy().view(dtype)[:, 0] for o in (layout.x_offset, layout.y_offset, layout.z_offset)]
    xyz = np.stack(cols, axis=1).astype(np.float64)
    return xyz[np.isfinite(xyz).all(1)]


def crop_scan(payload: bytes) -> np.ndarray:
    """1 回分の点群から、前方の箱に入る有限な点を (n, 3) で返す"""
    xyz = scan_xyz(payload)
    keep = (xyz[:, 0] > X_MIN) & (xyz[:, 0] < X_MAX) & (np.abs(xyz[:, 1]) < Y_ABS_MAX)
    return xyz[keep]


def iter_lidar(session: Path):
    """(受け取った時刻 [ns], ヘッダの時刻 [ns], CDR) を記録順に"""
    for bag in find_bag_files(session):
        con = sqlite3.connect(f"file:{bag}?mode=ro", uri=True)
        try:
            row = con.execute("SELECT id FROM topics WHERE name = ?", (LIDAR_TOPIC,)).fetchone()
            if row is None:
                raise SystemExit(f"[crop] {bag} に {LIDAR_TOPIC} が無い")
            for recv_ns, payload in con.execute(
                    "SELECT timestamp, data FROM messages WHERE topic_id = ? ORDER BY timestamp", (row[0],)):
                sec, nsec = struct.unpack_from("<iI", payload, 4)  # 先頭 4 バイトは CDR のカプセル化ヘッダ
                yield recv_ns, sec * 1_000_000_000 + nsec, payload
        finally:
            con.close()


def crop_session(session: Path, out: Path) -> dict:
    scans, stamps, recvs = [], [], []
    for recv_ns, stamp_ns, payload in iter_lidar(session):
        scans.append(crop_scan(payload))
        stamps.append(stamp_ns)
        recvs.append(recv_ns)
    if not scans:
        raise SystemExit(f"[crop] {session} にライダーの回が 1 つも無い")
    xyz = np.concatenate(scans)
    out.parent.mkdir(parents=True, exist_ok=True)
    np.savez_compressed(out, xyz_mm=np.round(xyz * 1000.0).astype(np.int16),
                        counts=np.array([len(s) for s in scans], np.int32),
                        stamp_ns=np.array(stamps, np.int64), recv_ns=np.array(recvs, np.int64))
    dur = (recvs[-1] - recvs[0]) / 1e9
    return {"回": len(scans), "秒": round(dur, 2), "Hz": round((len(scans) - 1) / dur, 2) if dur > 0 else None,
            "1回の点の中央値": int(np.median([len(s) for s in scans])), "出力": str(out)}


def main() -> None:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("session", type=Path)
    ap.add_argument("out", type=Path)
    a = ap.parse_args()
    print("[crop]", crop_session(a.session, a.out))


if __name__ == "__main__":
    main()
