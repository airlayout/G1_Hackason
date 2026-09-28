"""頭カメラの画像と深度から、ボトル（対象）の位置を pelvis 座標で求める（タスク4）。ラボ PC で実行する。

    # ライブ（深度付きカメラサーバから受信。腰の角度は DDS の lowstate から読む）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --live
    # 保存したファイル（probe_rgbd.py --save の出力。<stem> は _color.png などの前の部分）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --files _local/button_press/probe/20260930_101500
    # 収録したデータ（record.py の出力フォルダ）を再生して、10 フレームおきに求める
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/locate_bottle.py --recording _local/button_press/recordings/<フォルダ> --every 10

腰の角度（pelvis 座標に直すのに使う）:
- --waist-deg YAW ROLL PITCH で直接与える
- 与えなければ、--live では DDS の lowstate から読む（NIC は configs/arm.yaml の network_interface
  か --network-interface）。ファイルでは、メタデータに waist_q があればそれ、無ければ 0°（警告を出す）。
  収録では、そのフレームを保存したときの腰の角度（無ければ近い時刻の lowstate、それも無ければ 0°）

結果（枠・基準点・座標）を描いた画像を _local/button_press/locate/ に保存する。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

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
    p.add_argument("--frames", type=int, default=1, help="--live で処理するフレーム数")
    args = p.parse_args()

    robot_cfg = load_config("robot.yaml")
    loc_cfg = load_config("localize.yaml")
    locator = Locator(robot_cfg, loc_cfg, make_detector(loc_cfg))
    out = REPO_ROOT / "_local" / "button_press" / "locate"
    out.mkdir(parents=True, exist_ok=True)

    frames: list[tuple[RgbdFrame, np.ndarray]] = []
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
                qw = np.zeros(3)
            # 表示と保存するファイル名には、収録の中での番号を使う（サーバが付けた番号ではなく）
            fr.rgbd.frame_id = fr.index
            frames.append((fr.rgbd, qw))
    elif args.files:
        frame, meta = load_rgbd(args.files)
        if args.waist_deg is not None:
            qw = np.radians(args.waist_deg)
        elif "waist_q" in meta:
            qw = np.asarray(meta["waist_q"], dtype=float)
        else:
            print("[locate] ⚠️ 腰の角度が分からないので 0° とする（--waist-deg で与えられる）")
            qw = np.zeros(3)
        frames.append((frame, qw))
    else:
        from common.camera_rgbd import RgbdZmqSource

        cam = load_config("camera.yaml")["rgbd"]
        reader = None
        if args.waist_deg is None:
            from common.dds import LowStateReader

            iface = args.network_interface or load_config("arm.yaml")["network_interface"]
            reader = LowStateReader(0, iface)
            reader.open()
            reader.wait(10.0)
        with RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"])) as s:
            for _ in range(args.frames):
                f = s.read_rgbd()
                if f is None:
                    print("[locate] ❌ 深度付きストリームが届かない（configs/camera.yaml の接続先とポートを確認）")
                    return 1
                if reader is not None:
                    st = reader.latest()
                    assert st is not None
                    qw = st.q[list(WAIST_IDX)].copy()
                else:
                    qw = np.radians(args.waist_deg)
                frames.append((f, qw))

    ok = False
    for frame, qw in frames:
        found = locator.locate(frame, qw)
        waist = ", ".join(f"{v:+.1f}" for v in np.degrees(qw))
        print(f"[locate] フレーム {frame.frame_id}（腰 yaw/roll/pitch = {waist}°）: 検出 {len(found)} 個")
        for x in found:
            print(f"[locate]   {x.summary()}")
        best = locator.best(found)
        if best is not None:
            ok = True
            print(f"[locate] → 対象: pelvis 座標 {np.round(best.p_pelvis, 3)} m")
        path = out / f"{time.strftime('%Y%m%d_%H%M%S')}_{frame.frame_id}.png"
        cv2.imwrite(str(path), draw(frame, found))
        print(f"[locate] 保存: {path}")
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
