"""PC2 の /dev/video* が、それぞれ何の映像（カラー・深度・IR・補助データ）を出すかを表示する（見るだけ）。

v4l2-ctl が PC2 に無いため、Python の標準ライブラリだけで V4L2 に問い合わせる。
形式の一覧を聞くだけで映像は開かない（VIDIOC_QUERYCAP / VIDIOC_ENUM_FMT のみ。VIDIOC_S_FMT や
STREAMON はしない）ので、ほかのプロセス（videohub など）が使っている最中でも答えが返り、その邪魔もしない。

    python3 ~/button_press/real/depth_server/v4l2_list.py

PC2 のシステムの Python 3.8 で動くように書く（型ヒントは from __future__ で文字列扱い）。
"""

from __future__ import annotations

import fcntl
import glob
import os
import struct

# ioctl の番号（linux/videodev2.h。_IOR('V', 0, 104 バイト) と _IOWR('V', 2, 64 バイト)）
VIDIOC_QUERYCAP = 0x80685600
VIDIOC_ENUM_FMT = 0xC0405602

BUF_TYPE_VIDEO_CAPTURE = 1
BUF_TYPE_META_CAPTURE = 13
CAP_VIDEO_CAPTURE = 0x00000001
CAP_META_CAPTURE = 0x00800000

# 形式の名前（fourcc）から、何の映像かの目安
KIND = {
    "Z16 ": "深度",
    "YUYV": "カラー", "RGB3": "カラー", "BGR3": "カラー", "UYVY": "カラー", "MJPG": "カラー",
    "GREY": "IR", "Y8I ": "IR", "Y12I": "IR", "Y16 ": "IR", "Y8  ": "IR",
}


def query_cap(fd: int) -> tuple[str, str, int]:
    """カード名・USB の場所・device_caps を返す。"""
    buf = bytearray(104)
    fcntl.ioctl(fd, VIDIOC_QUERYCAP, buf)
    card = bytes(buf[16:48]).split(b"\0")[0].decode(errors="replace")
    bus = bytes(buf[48:80]).split(b"\0")[0].decode(errors="replace")
    caps = struct.unpack_from("I", buf, 84)[0]
    device_caps = struct.unpack_from("I", buf, 88)[0]
    return card, bus, device_caps or caps


def enum_formats(fd: int, buf_type: int) -> list[tuple[str, str]]:
    """(fourcc, 説明) の一覧を返す。"""
    out: list[tuple[str, str]] = []
    for index in range(64):
        buf = bytearray(64)
        struct.pack_into("II", buf, 0, index, buf_type)
        try:
            fcntl.ioctl(fd, VIDIOC_ENUM_FMT, buf)
        except OSError:
            break
        desc = bytes(buf[12:44]).split(b"\0")[0].decode(errors="replace")
        fourcc = bytes(buf[44:48]).decode(errors="replace")
        out.append((fourcc, desc))
    return out


def usb_interface(dev: str) -> str:
    """sysfs から USB のインターフェース（例: 1-2.3:1.3）を返す。"""
    link = f"/sys/class/video4linux/{os.path.basename(dev)}/device"
    try:
        return os.path.basename(os.path.realpath(link))
    except OSError:
        return "?"


def main() -> None:
    devs = sorted(glob.glob("/dev/video*"), key=lambda p: int("".join(c for c in p if c.isdigit()) or 0))
    if not devs:
        print("[v4l2_list] /dev/video* が無い")
        return
    for dev in devs:
        try:
            fd = os.open(dev, os.O_RDONLY | os.O_NONBLOCK)
        except OSError as e:
            print(f"[v4l2_list] {dev}: 開けない（{e}）")
            continue
        try:
            card, bus, caps = query_cap(fd)
            kinds = []
            if caps & CAP_VIDEO_CAPTURE:
                kinds.append(("映像", enum_formats(fd, BUF_TYPE_VIDEO_CAPTURE)))
            if caps & CAP_META_CAPTURE:
                kinds.append(("補助データ", enum_formats(fd, BUF_TYPE_META_CAPTURE)))
            print(f"[v4l2_list] {dev}: {card}（USB {usb_interface(dev)}、{bus}）")
            if not kinds:
                print(f"[v4l2_list]     映像も補助データも出さない（caps=0x{caps:08x}）")
            for label, fmts in kinds:
                if not fmts:
                    print(f"[v4l2_list]     {label}: 形式の一覧なし")
                for fourcc, desc in fmts:
                    guess = KIND.get(fourcc, "補助データ" if label == "補助データ" else "?")
                    print(f"[v4l2_list]     {label}: '{fourcc}' {desc} → {guess}")
        except OSError as e:
            print(f"[v4l2_list] {dev}: 問い合わせに失敗（{e}）")
        finally:
            os.close(fd)
    by_id = sorted(glob.glob("/dev/v4l/by-id/*"))
    if by_id:
        print("[v4l2_list] 安定名（by-id）:")
        for p in by_id:
            print(f"[v4l2_list]   {os.path.basename(p)} → {os.path.basename(os.path.realpath(p))}")


if __name__ == "__main__":
    main()
