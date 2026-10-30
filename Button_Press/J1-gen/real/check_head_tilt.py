"""頭カメラの向き（pitch / roll）を、深度から床や机の面を見つけて確かめる。実機日の最初に実行する（首の傾きの確認）。

    # ライブ（有線のラボ PC。腰の角度と IMU は DDS の lowstate から読む。立った状態で、床か机の面が広く写る向きで）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_head_tilt.py --live --frames 5
    # 無線から（lowstate を読まない。腰 0°・骨盤は水平と仮定して警告する）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_head_tilt.py --live --no-lowstate --camera-config camera_wifi.yaml
    # 保存したファイル / 収録（record.py の出力。--every フレームおき）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_head_tilt.py --files _local/button_press/probe/<stem>
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_head_tilt.py --recording _local/button_press/recordings/<フォルダ> --every 30

出力: 見つけた平面ごとの pitch / roll、中央値、URDF（robot.yaml）との差、判定の文。結果は _local/button_press/head_tilt/ に JSON で残す。
読み方は common/head_tilt.py。判定は「床と机など複数の面で同じ pitch になるか」を見る。骨盤が傾いている（IMU で 2° 超）
ときは、立ち直してから測る。**カメラの設定（camera.yaml）は、このスクリプトは書き換えない**。
"""

from __future__ import annotations

import argparse
import json
import sys
import time
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.camera_cfg import add_camera_args, load_camera_config  # noqa: E402
from common.camera_geometry import HeadCameraTransform  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.head_tilt import estimate_head_tilt, judge, pelvis_tilt_deg  # noqa: E402
from common.rgbd_io import load_rgbd  # noqa: E402
from common.rgbd_protocol import RgbdFrame  # noqa: E402
from common.robot_model import WAIST_IDX  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", action="store_true")
    src.add_argument("--files", type=Path)
    src.add_argument("--recording", type=Path)
    p.add_argument("--every", type=int, default=30)
    p.add_argument("--frames", type=int, default=5, help="--live で使うフレーム数")
    p.add_argument("--network-interface")
    p.add_argument("--no-lowstate", action="store_true")
    add_camera_args(p)
    args = p.parse_args()

    robot_cfg = load_config("robot.yaml")  # URDF の値（camera.yaml の上書きは使わない。URDF との差を見るため）
    urdf_pitch = float(np.degrees(robot_cfg["head_camera"]["rpy"][1]))
    transform = HeadCameraTransform(robot_cfg)

    # (フレーム, 腰の角度, IMU の四元数 or None)
    items: list[tuple[RgbdFrame, np.ndarray, np.ndarray | None]] = []
    if args.live:
        from common.camera_rgbd import RgbdZmqSource

        cam = load_camera_config(args)["rgbd"]
        reader = None
        if not args.no_lowstate:
            from common.dds import LowStateReader

            reader = LowStateReader(0, args.network_interface or load_config("arm.yaml")["network_interface"])
            reader.open()
            reader.wait(10.0)
        else:
            print("[head_tilt] ⚠️ lowstate を読まない: 腰は 0°、骨盤は水平と仮定する（IMU の傾きは確かめられない）")
        with RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"])) as s:
            for _ in range(args.frames):
                f = s.read_rgbd()
                if f is None:
                    print("[head_tilt] ❌ 深度付きストリームが届かない（接続先・PC2 のサーバを確認）")
                    return 1
                st = reader.latest() if reader is not None else None
                items.append((f, st.q[list(WAIST_IDX)].copy() if st is not None else np.zeros(3),
                              st.imu_quat if st is not None else None))
                time.sleep(0.3)
    elif args.files:
        frame, meta = load_rgbd(args.files)
        items.append((frame, np.asarray(meta.get("waist_q", np.zeros(3)), dtype=float), None))
    else:
        from common.recording import Recording

        rec = Recording(args.recording)
        for k in range(0, len(rec), max(1, args.every)):
            fr = rec.load(k)
            if fr.rgbd is not None:
                items.append((fr.rgbd, fr.waist_q if fr.waist_q is not None else np.zeros(3), None))
        print("[head_tilt] ⚠️ 収録・ファイルには IMU が無いので、骨盤は水平と仮定する")

    all_planes, rows = [], []
    for frame, qw, imu in items:
        tilt = pelvis_tilt_deg(imu) if imu is not None else None
        if tilt is not None and tilt > 2.0:
            print(f"[head_tilt] ⚠️ 骨盤が {tilt:.1f}° 傾いている。立ち直してから測る（IMU の向きで補正はしている）")
        planes = estimate_head_tilt(frame, transform, robot_cfg, qw, imu)
        all_planes += planes
        for pl in planes:
            print(f"[head_tilt] フレーム {frame.frame_id}: 面（{pl.inlier_fraction * 100:.0f}% の点、カメラから {pl.height_m:.2f} m）"
                  f" pitch {pl.pitch_deg:.1f}° roll {pl.roll_deg:+.1f}°（残り {pl.residual_deg:.1f}°"
                  f"{'' if pl.residual_deg <= 2.0 else ' → 水平でない面。使わない'}）")
        rows.append({"frame_id": frame.frame_id, "pelvis_tilt_deg": tilt, "planes": [vars(x) for x in planes]})
    res = judge(all_planes, urdf_pitch)
    print(f"[head_tilt] 判定: {res['verdict']}")
    out = REPO_ROOT / "_local" / "button_press" / "head_tilt"
    out.mkdir(parents=True, exist_ok=True)
    path = out / f"{time.strftime('%Y%m%d_%H%M%S')}.json"
    path.write_text(json.dumps({"urdf_pitch_deg": urdf_pitch, "result": res, "frames": rows}, indent=2, ensure_ascii=False))
    print(f"[head_tilt] 保存: {path}")
    return 0 if res["pitch_deg"] is not None else 1


if __name__ == "__main__":
    sys.exit(main())
