"""深度付きストリームが届くかを確かめる（ラボ PC で実行）。タスク3。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/probe_rgbd.py
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/probe_rgbd.py --address localhost --save

受信したフレーム数・fps・1 フレームの大きさ・内部パラメータ・画像中央の深度を表示する。
--save を付けると、最後のフレームのカラー（PNG）と深度（16bit PNG）と内部パラメータ（JSON）を
_local/button_press/probe/ に保存する（16bit PNG は深度の値をそのまま保存できる）。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from dataclasses import asdict
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.camera_rgbd import RgbdZmqSource  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--config", default="camera.yaml")
    p.add_argument("--address", help="接続先（既定は設定ファイル）")
    p.add_argument("--port", type=int, help="ポート（既定は設定ファイル）")
    p.add_argument("--seconds", type=float, default=5.0, help="受信する秒数")
    p.add_argument("--save", action="store_true", help="最後のフレームを保存する")
    args = p.parse_args()

    cfg = load_config(args.config)["rgbd"]
    address = args.address or cfg["server_address"]
    port = args.port or int(cfg["port"])
    print(f"[probe_rgbd] 接続: tcp://{address}:{port}")
    n = 0
    last = None
    t0 = time.monotonic()
    with RgbdZmqSource(address, port, int(cfg["timeout_ms"])) as src:
        while time.monotonic() - t0 < args.seconds:
            f = src.read_rgbd()
            if f is None:
                print("[probe_rgbd] ❌ タイムアウト。サーバが動いているか、アドレスとポートを確認すること")
                return 1
            n += 1
            last = f
    assert last is not None
    dt = time.monotonic() - t0
    d = last.depth
    h, w = d.shape
    center = d[h // 2 - 5:h // 2 + 5, w // 2 - 5:w // 2 + 5]
    valid = center[center > 0]
    print(f"[probe_rgbd] {n} フレーム / {dt:.1f} 秒（{n / dt:.1f} fps）、最後のフレーム番号 {last.frame_id}")
    print(f"[probe_rgbd] カラー {last.color_bgr.shape}、深度 {d.shape} {d.dtype}、深度の単位 {last.depth_scale} m")
    print(f"[probe_rgbd] 内部パラメータ: {asdict(last.intrinsics)}")
    print(f"[probe_rgbd] 送信からの遅れ {(time.time() - last.timestamp) * 1000:.0f} ms"
          "（PC2 とラボ PC の時計がずれていると正しくない）")
    if valid.size:
        print(f"[probe_rgbd] 画像中央の深度（中央値）: {np.median(valid) * last.depth_scale:.3f} m")
    else:
        print("[probe_rgbd] 画像中央の深度: 測れていない（0）")
    print(f"[probe_rgbd] 深度が 0（測れなかった）の画素: {np.mean(d == 0) * 100:.1f}%")

    if args.save:
        import cv2

        out = REPO_ROOT / "_local" / "button_press" / "probe"
        out.mkdir(parents=True, exist_ok=True)
        stem = time.strftime("%Y%m%d_%H%M%S")
        cv2.imwrite(str(out / f"{stem}_color.png"), last.color_bgr)
        cv2.imwrite(str(out / f"{stem}_depth.png"), last.depth)
        (out / f"{stem}_intrinsics.json").write_text(
            json.dumps({"intrinsics": asdict(last.intrinsics), "depth_scale": last.depth_scale}, indent=2)
        )
        print(f"[probe_rgbd] 保存: {out}/{stem}_*")
    return 0


if __name__ == "__main__":
    sys.exit(main())
