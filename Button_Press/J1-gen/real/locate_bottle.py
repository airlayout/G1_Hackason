"""頭カメラの画像と深度から、ボトル（対象）の位置を pelvis 座標で求める（タスク4）。ラボ PC で実行する。

    # ライブ（深度付きカメラサーバから受信。腰の角度は DDS の lowstate から読む）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --live
    # 無線でノート PC から（lowstate は読まない。腰の角度は 0° と仮定）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --live --no-lowstate --camera-config camera_wifi.yaml
    # 保存したファイル（probe_rgbd.py --save の出力。<stem> は _color.png などの前の部分）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --files _local/button_press/probe/20260930_101500
    # 収録したデータ（record.py の出力フォルダ）を再生して、10 フレームおきに求める
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --recording _local/button_press/recordings/<フォルダ> --every 10

腰の角度（pelvis 座標に直すのに使う）:
- --waist-deg YAW ROLL PITCH で直接与える
- --no-lowstate で lowstate を読まない（DDS が届かない無線でノート PC から試すとき）。腰の角度は 0° と仮定し、
  「腰の角度は仮定」と警告して、結果の JSON にも waist_assumed: true を残す。カメラ座標の値は腰によらない
- どれも付けなければ、--live では DDS の lowstate から読む（NIC は configs/arm.yaml の network_interface
  か --network-interface）。ファイルでは、メタデータに waist_q があればそれ、無ければ 0°（警告を出す）。
  収録では、そのフレームを保存したときの腰の角度（無ければ近い時刻の lowstate、それも無ければ 0°）

結果（枠・基準点・座標）を描いた画像と、結果の JSON（カメラ座標・pelvis 座標・腰の角度が仮定か）を
_local/button_press/locate/ に保存する。
"""

from __future__ import annotations

import argparse
import json
import pathlib
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.camera_cfg import add_camera_args, load_camera_config  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.localize import Located, Locator, make_detector  # noqa: E402
from common.rgbd_io import load_rgbd  # noqa: E402
from common.rgbd_protocol import RgbdFrame  # noqa: E402
from common.robot_model import WAIST_IDX  # noqa: E402


def draw(frame: RgbdFrame, found: list[Located]) -> np.ndarray:
    img = frame.color_bgr.copy()
    for x in found:
        x1, y1, x2, y2 = (int(round(t)) for t in x.bbox)
        color = (0, 255, 0) if x.ok else (0, 0, 255)
        cv2.rectangle(img, (x1, y1), (x2, y2), color, 2)
        cv2.circle(img, (int(x.pixel[0]), int(x.pixel[1])), 5, (0, 255, 255), -1)
        text = f"{x.class_name} {x.confidence:.2f}"
        if x.ok and x.p_pelvis is not None:
            text += " (" + ", ".join(f"{v:+.3f}" for v in x.p_pelvis) + ")"
        cv2.putText(img, text, (x1, max(15, y1 - 5)), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 1)
    return img


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", action="store_true", help="深度付きカメラサーバから受信する")
    src.add_argument("--files", type=Path, help="保存したファイルの <stem>")
    src.add_argument("--recording", type=Path, help="収録したフォルダ（record.py の出力）")
    p.add_argument("--every", type=int, default=1, help="--recording で、何フレームおきに処理するか")
    p.add_argument("--waist-deg", type=float, nargs=3, metavar=("YAW", "ROLL", "PITCH"))
    p.add_argument("--network-interface", help="lowstate を読む NIC（既定は configs/arm.yaml）")
    p.add_argument("--no-lowstate", action="store_true",
                   help="lowstate を読まない（DDS が届かない無線など）。腰の角度は 0° と仮定し、その旨を警告・記録する")
    p.add_argument("--frames", type=int, default=1, help="--live で処理するフレーム数")
    add_camera_args(p)
    p.add_argument("--detector", choices=["yolo", "color"], help="検出器（既定は configs/localize.yaml）")
    p.add_argument("--out-dir", type=Path, help="画像などの保存先（既定は _local/button_press/ の下）")
    args = p.parse_args()

    robot_cfg = load_config("robot.yaml")
    loc_cfg = load_config("localize.yaml")
    if args.detector:
        loc_cfg["detector"]["type"] = args.detector
    locator = Locator(robot_cfg, loc_cfg, make_detector(loc_cfg))
    out = args.out_dir or REPO_ROOT / "_local" / "button_press" / "locate"
    out.mkdir(parents=True, exist_ok=True)

    # (フレーム, 腰の角度, 腰の角度が仮定か)
    frames: list[tuple[RgbdFrame, np.ndarray, bool]] = []
    assumed_msg = ("⚠️ 腰の角度は仮定（0°）。lowstate を読んでいないので、pelvis 座標は腰が 0° のときの値。"
                   "カメラ座標の値は腰の角度によらない")
    if args.recording:
        from common.recording import Recording

        rec = Recording(args.recording)
        print(f"[locate] 収録 {args.recording}: {len(rec)} フレーム、lowstate {rec.lowstate_count} 行")
        for k in range(0, len(rec), max(1, args.every)):
            fr = rec.load(k)
            if fr.rgbd is None:
                print(f"[locate] フレーム {fr.index}: 深度が無い（RGB だけの収録）ので飛ばす")
                continue
            if args.waist_deg is not None:
                qw = np.radians(args.waist_deg)
            elif fr.waist_q is not None:
                qw = fr.waist_q
            else:
                print(f"[locate] ⚠️ フレーム {fr.index}: 腰の角度が分からないので 0° とする")
                qw = None
            # 表示と保存するファイル名には、収録の中での番号を使う（サーバが付けた番号ではなく）
            fr.rgbd.frame_id = fr.index
            frames.append((fr.rgbd, np.zeros(3) if qw is None else qw, qw is None))
    elif args.files:
        frame, meta = load_rgbd(args.files)
        if args.waist_deg is not None:
            qw = np.radians(args.waist_deg)
        elif "waist_q" in meta:
            qw = np.asarray(meta["waist_q"], dtype=float)
        else:
            print("[locate] ⚠️ 腰の角度が分からないので 0° とする（--waist-deg で与えられる）")
            qw = None
        frames.append((frame, np.zeros(3) if qw is None else qw, qw is None))
    else:
        from common.camera_rgbd import RgbdZmqSource

        cam = load_camera_config(args)["rgbd"]
        reader = None
        if args.no_lowstate:
            print(f"[locate] {assumed_msg}")
        elif args.waist_deg is None:
            from common.dds import LowStateReader

            iface = args.network_interface or load_config("arm.yaml")["network_interface"]
            reader = LowStateReader(0, iface)
            reader.open()
            reader.wait(10.0)
        with RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"])) as s:
            for _ in range(args.frames):
                f = s.read_rgbd()
                if f is None:
                    print(f"[locate] ❌ 深度付きストリームが届かない（接続先 {cam['server_address']}:{cam['port']}。"
                          f"{args.camera_config} の pc2_host か --host、PC2 のサーバが動いているかを確認）")
                    return 1
                if reader is not None:
                    st = reader.latest()
                    assert st is not None
                    frames.append((f, st.q[list(WAIST_IDX)].copy(), False))
                elif args.waist_deg is not None:
                    frames.append((f, np.radians(args.waist_deg), False))
                else:
                    frames.append((f, np.zeros(3), True))

    ok = False
    fmt = lambda v: "[" + ", ".join(f"{x:+.3f}" for x in v) + "]"  # noqa: E731
    for frame, qw, assumed in frames:
        found = locator.locate(frame, qw)
        waist = ", ".join(f"{v:+.1f}" for v in np.degrees(qw))
        note = "（仮定）" if assumed else ""
        print(f"[locate] フレーム {frame.frame_id}（腰 yaw/roll/pitch = {waist}°{note}）: 検出 {len(found)} 個")
        for x in found:
            print(f"[locate]   {x.summary()}")
        best = locator.best(found)
        if best is not None and best.p_optical is not None and best.p_pelvis is not None:
            ok = True
            print(f"[locate] → 対象: カメラ座標（x 右、y 下、z 前）{fmt(best.p_optical)} m、"
                  f"pelvis 座標 {fmt(best.p_pelvis)} m{note}")
            if assumed:
                print(f"[locate]   {assumed_msg}")
        stem = out / f"{time.strftime('%Y%m%d_%H%M%S')}_{frame.frame_id}"
        cv2.imwrite(f"{stem}.png", draw(frame, found))
        result = {
            "frame_id": frame.frame_id, "timestamp": frame.timestamp, "waist_q": qw.tolist(),
            "waist_assumed": assumed, "warning": assumed_msg if assumed else "",
            "calibration_offset_pelvis_m": loc_cfg["calibration"]["offset_pelvis_m"],
            "detections": [{"class": x.class_name, "confidence": x.confidence, "bbox": list(x.bbox),
                            "pixel": list(x.pixel), "depth_m": x.depth_m,
                            "p_optical_m": None if x.p_optical is None else x.p_optical.tolist(),
                            "p_pelvis_m": None if x.p_pelvis is None else x.p_pelvis.tolist(),
                            "reason": x.reason} for x in found],
        }
        pathlib.Path(f"{stem}.json").write_text(json.dumps(result, indent=2, ensure_ascii=False))
        print(f"[locate] 保存: {stem}.png / .json")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
