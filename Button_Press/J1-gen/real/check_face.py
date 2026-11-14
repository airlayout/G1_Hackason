"""柱の手前（約 0.5〜1.5 m）から、頭カメラの深度で柱の面の距離・向き・端が取れるかを確かめる。腕も足も動かさない。

    # ライブ（有線のラボ PC。腰の角度と IMU は DDS の lowstate から読む）。メジャーで測った pelvis から柱までの距離を添える
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_face.py --live --frames 5 --measured-cm 100
    # 無線から（lowstate を読まない。腰 0°・骨盤は水平と仮定する）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_face.py --live --no-lowstate --camera-config camera_wifi.yaml
    # 保存したファイル（このスクリプトが残したもの、probe の出力）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/check_face.py --files _local/button_press/face_check/<時刻>/frame_00

出力: フレームごとの距離・向き・端・床で直した首の向き、続けて測ったまとめ（configs/approach.yaml の face.tracking）。
ライブでは各フレームを _local/button_press/face_check/<時刻>/ に保存する（遠くからの乗り場の深度はまだ手元に無いので、
あとで設定を変えて読み直せるように）。読み方は common/column_face.py。
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
from common.column_face import FaceFinder, FaceNotFound, FaceTracker  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.head_tilt import pelvis_tilt_deg  # noqa: E402
from common.rgbd_io import load_rgbd, save_rgbd  # noqa: E402
from common.rgbd_protocol import RgbdFrame  # noqa: E402
from common.robot_model import WAIST_IDX  # noqa: E402


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", action="store_true")
    src.add_argument("--files", type=Path, nargs="+")
    p.add_argument("--frames", type=int, default=5, help="--live で使うフレーム数")
    p.add_argument("--measured-cm", type=float, help="メジャーで測った pelvis から柱の面までの距離 [cm]（比べるだけ）")
    p.add_argument("--network-interface")
    p.add_argument("--no-lowstate", action="store_true")
    add_camera_args(p)
    args = p.parse_args()

    # エレベーターのエージェントと同じく robot.yaml の向き（camera.yaml の head_camera_mount は使わない）。首のずれは床で直す
    cfg = load_config("approach.yaml")["face"]
    finder = FaceFinder.from_config(load_config("robot.yaml"), cfg)
    tracker = FaceTracker(cfg["tracking"])
    out = REPO_ROOT / "_local" / "button_press" / "face_check" / time.strftime("%Y%m%d_%H%M%S")

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
            print("[face] ⚠️ lowstate を読まない: 腰は 0°、骨盤は水平と仮定する")
        out.mkdir(parents=True, exist_ok=True)
        with RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"])) as s:
            for i in range(args.frames):
                f = s.read_rgbd()
                if f is None:
                    print("[face] ❌ 深度付きストリームが届かない（接続先・PC2 のサーバを確認）")
                    return 1
                st = reader.latest() if reader is not None else None
                qw = st.q[list(WAIST_IDX)].copy() if st is not None else np.zeros(3)
                imu = st.imu_quat if st is not None else None
                save_rgbd(f, out / f"frame_{i:02d}", {"waist_q": qw.tolist(),
                                                     "imu_quat": None if imu is None else list(map(float, imu))})
                items.append((f, qw, imu))
                time.sleep(0.3)
    else:
        for stem in args.files:
            frame, meta = load_rgbd(stem)
            imu = meta.get("imu_quat")
            items.append((frame, np.asarray(meta.get("waist_q", np.zeros(3)), dtype=float),
                          None if imu is None else np.asarray(imu, dtype=float)))

    rows, merged = [], None
    for frame, qw, imu in items:
        tilt = pelvis_tilt_deg(imu) if imu is not None else None
        if tilt is not None and tilt > 2.0:
            print(f"[face] ⚠️ 骨盤が {tilt:.1f}° 傾いている（IMU の向きで補正はしている）")
        i = frame.intrinsics
        K = np.array([[i.fx, 0, i.cx], [0, i.fy, i.cy], [0, 0, 1.0]])
        try:
            face = finder.find(frame.depth_m(), K, qw, imu)
        except FaceNotFound as e:
            print(f"[face] フレーム {frame.frame_id}: ❌ {e}")
            rows.append({"frame_id": frame.frame_id, "error": str(e)})
            continue
        print(f"[face] フレーム {frame.frame_id}: {face.summary()}")
        rows.append({"frame_id": frame.frame_id, "distance_m": face.distance_m, "yaw_deg": float(np.degrees(face.yaw_rad)),
                     "left_edge_m": face.left_edge_m, "right_edge_m": face.right_edge_m, "n_points": face.n_points,
                     "floor_found": face.floor_found, "pitch_correction_deg": face.pitch_correction_deg,
                     "correction_from": face.correction_from, "floor_z_m": face.floor_z_m, "others_m": face.others_m})
        merged = tracker.add(face) or merged
    if merged is None:
        print(f"[face] まとめ: {tracker.frames} フレーム続けてそろわなかった（見つからない、またはばらつきが大きい）")
    else:
        print(f"[face] まとめ: {merged.summary()}")
        if args.measured_cm is not None:
            print(f"[face] メジャー {args.measured_cm:.1f} cm との差: {merged.distance_m * 100 - args.measured_cm:+.1f} cm")
    out.mkdir(parents=True, exist_ok=True)
    path = out / "result.json"
    summary = None if merged is None else {
        "distance_m": merged.distance_m, "yaw_deg": float(np.degrees(merged.yaw_rad)),
        "left_edge_m": merged.left_edge_m, "right_edge_m": merged.right_edge_m}
    path.write_text(json.dumps({"measured_cm": args.measured_cm, "frames": rows, "merged": summary},
                               indent=2, ensure_ascii=False))
    print(f"[face] 保存: {path}")
    return 0 if merged is not None else 1


if __name__ == "__main__":
    sys.exit(main())
