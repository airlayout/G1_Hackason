"""2 台のスマホ（cam1・cam2）の映像が OMEN に届く時刻のずれを測る。

    python measure_sync.py [--mode motion|flash] [--secs 20]

motion（既定）: 固定した 2 台を景色に向けたまま、止める・振る・止める…を繰り返す（20 秒）。
  画の動きの大きさ（前のこまとの明るさの差）の時間変化を 2 台で比べ、相互相関の山からずれを出す。
  道具はいらない。
flash: Mac の画面に publish/flash.html を全画面で出し、2 台を向ける。白黒の切り替わりの時刻を比べる。

結果は <out>/sync_<日時>.json（offset_s は「右が左より何秒遅れて届くか」）。ステレオの組を作るときに使う。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time

import cv2
import numpy as np

HERE = pathlib.Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))

from stereo_core import estimate_offset, estimate_offset_xcorr  # noqa: E402

SYNC_LONG_SIDE = 320  # 明るさしか見ないので小さく解く
POLL_S = 0.003


def roi_brightness(bgr: np.ndarray) -> float:
    h, w = bgr.shape[:2]
    roi = bgr[int(h * 0.3) : int(h * 0.7), int(w * 0.3) : int(w * 0.7)]
    return float(cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY).mean())


def record(grabbers: dict, secs: float, mode: str) -> dict[str, tuple[np.ndarray, np.ndarray]]:
    """こまごとの（届いた時刻, 値）。値は flash なら真ん中の明るさ、motion なら前のこまとの差"""
    seen = {k: -1 for k in grabbers}
    prev: dict[str, np.ndarray | None] = {k: None for k in grabbers}
    data = {k: ([], []) for k in grabbers}
    t_end = time.time() + secs
    while time.time() < t_end:
        for k, g in grabbers.items():
            item = g.latest()
            if item is None or item[1] == seen[k]:
                continue
            seen[k] = item[1]
            if mode == "flash":
                data[k][0].append(item[0])
                data[k][1].append(roi_brightness(item[2]))
                continue
            gray = cv2.cvtColor(item[2], cv2.COLOR_BGR2GRAY).astype(np.float32)
            if prev[k] is not None:
                data[k][0].append(item[0])
                data[k][1].append(float(np.abs(gray - prev[k]).mean()))
            prev[k] = gray
        time.sleep(POLL_S)
    return {k: (np.asarray(t), np.asarray(v)) for k, (t, v) in data.items()}


def main() -> None:
    from rtsp_grabber import LatestFrameGrabber

    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--left", default="rtsp://127.0.0.1:8554/cam1")
    p.add_argument("--right", default="rtsp://127.0.0.1:8554/cam2")
    p.add_argument("--mode", choices=["motion", "flash"], default="motion")
    p.add_argument("--secs", type=float, default=20.0)
    p.add_argument("--out", default=str(pathlib.Path.home() / "work_inoue/phone_slam/calib"))
    args = p.parse_args()

    grabbers = {
        "left": LatestFrameGrabber(args.left, long_side=SYNC_LONG_SIDE, history_len=4),
        "right": LatestFrameGrabber(args.right, long_side=SYNC_LONG_SIDE, history_len=4),
    }
    print("[時刻] 2 台の映像を待つ", flush=True)
    while any(g.latest() is None for g in grabbers.values()):
        time.sleep(0.2)
    how = "2 台を一緒に、止める・振る・止める…" if args.mode == "motion" else "点滅を写す"
    print(f"[時刻] {args.secs:.0f} 秒測る（{how}）", flush=True)
    data = record(grabbers, args.secs, args.mode)
    for g in grabbers.values():
        g.close()

    (tl, vl), (tr, vr) = data["left"], data["right"]
    res = estimate_offset(tl, vl, tr, vr) if args.mode == "flash" else estimate_offset_xcorr(tl, vl, tr, vr)
    if args.mode == "flash":
        res["ok"] = res.get("n_events", 0) > 0
    res.update(
        mode=args.mode,
        left=args.left,
        right=args.right,
        secs=args.secs,
        left_fps=round(len(tl) / args.secs, 1),
        right_fps=round(len(tr) / args.secs, 1),
        created=time.strftime("%Y-%m-%d %H:%M:%S"),
    )
    out = pathlib.Path(args.out).expanduser() / f"sync_{time.strftime('%m%d_%H%M')}.json"
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(json.dumps(res, ensure_ascii=False, indent=2) + "\n")
    if not res["ok"]:
        print(f"[時刻] 測れなかった（{res}）。2 台を固定したまま、もっと大きく止める・振るを繰り返す", flush=True)
    elif args.mode == "motion":
        print(
            f"[時刻] 右は左より {res['offset_s'] * 1000:+.0f} ms 遅れて届く（相関の山 {res['peak_corr']:.2f}・"
            f"{res['secs']:.0f} 秒分）。fps 左 {res['left_fps']}・右 {res['right_fps']}",
            flush=True,
        )
    else:
        print(
            f"[時刻] 右は左より {res['offset_s'] * 1000:+.0f} ms 遅れて届く"
            f"（ばらつき 標準偏差 {res['jitter_std_s'] * 1000:.0f} ms・{res['n_events']} 回）。"
            f"fps 左 {res['left_fps']}・右 {res['right_fps']}",
            flush=True,
        )
    print(f"[時刻] 保存した: {out}", flush=True)


if __name__ == "__main__":
    main()
