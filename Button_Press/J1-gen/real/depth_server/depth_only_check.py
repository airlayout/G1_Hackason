"""カラーのカメラ（/dev/video4）をほかのプロセスが使っていても、深度だけなら取れるかを確かめる。

PC2 の RealSense（D435i）は USB の中で 2 つに分かれている（v4l2_list.py で確認、2026-09-29）:
- 1.0: 深度（/dev/video0、Z16）と IR（/dev/video2）
- 1.3: カラー（/dev/video4、YUYV）… Unitree の videohub_pc4 が開いている

rgbd_server はカラーと深度を 1 つの pipeline で開くので、カラーが使用中だと深度も取れない。
ここでは深度（と、望めば左の IR）だけを開き、数フレーム受け取って中央の距離を表示する。
カラーのカメラは開かないので videohub の邪魔はしない。

あわせて、カラーの内部パラメータと、深度 → カラーの位置関係（外部パラメータ）を、映像を開かずに
カメラの設定から読んで表示する（カラーを別の経路で受け取ったときに、深度を重ねるのに使う）。

    python3 ~/button_press/real/depth_server/depth_only_check.py
    python3 ~/button_press/real/depth_server/depth_only_check.py --ir   # 左の IR も一緒に開く

PC2 のシステムの Python 3.8 で動くように書く。
"""

from __future__ import annotations

import argparse
import sys
import time


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__.split("\n")[0])
    p.add_argument("--width", type=int, default=640)
    p.add_argument("--height", type=int, default=480)
    p.add_argument("--fps", type=int, default=30)
    p.add_argument("--frames", type=int, default=30, help="受け取るフレーム数")
    p.add_argument("--ir", action="store_true", help="左の IR（Y8）も一緒に開く")
    args = p.parse_args()

    import numpy as np
    import pyrealsense2 as rs

    devs = rs.context().query_devices()
    if len(devs) == 0:
        print("[depth_check] ❌ RealSense が見つからない")
        return 1
    dev = devs[0]
    print(f"[depth_check] {dev.get_info(rs.camera_info.name)}（シリアル {dev.get_info(rs.camera_info.serial_number)}）")

    # 映像を開かずに、カメラの設定からカラーと深度の内部パラメータ・位置関係を読む
    color_prof = depth_prof = None
    for sensor in dev.query_sensors():
        for sp in sensor.get_stream_profiles():
            if not sp.is_video_stream_profile():
                continue
            vp = sp.as_video_stream_profile()
            if (vp.width(), vp.height(), vp.fps()) != (args.width, args.height, args.fps):
                continue
            if sp.stream_type() == rs.stream.color and color_prof is None:
                color_prof = vp
            if sp.stream_type() == rs.stream.depth and depth_prof is None:
                depth_prof = vp
    if color_prof is not None:
        i = color_prof.get_intrinsics()
        print(f"[depth_check] カラーの内部パラメータ（{i.width}x{i.height}）: "
              f"fx={i.fx:.1f} fy={i.fy:.1f} cx={i.ppx:.1f} cy={i.ppy:.1f} model={i.model}")
    else:
        print(f"[depth_check] カラーに {args.width}x{args.height}@{args.fps} の設定が無い")
    if color_prof is not None and depth_prof is not None:
        e = depth_prof.get_extrinsics_to(color_prof)
        print(f"[depth_check] 深度 → カラー: 平行移動 {[round(t, 4) for t in e.translation]} m")

    config = rs.config()
    config.enable_device(dev.get_info(rs.camera_info.serial_number))
    config.enable_stream(rs.stream.depth, args.width, args.height, rs.format.z16, args.fps)
    if args.ir:
        config.enable_stream(rs.stream.infrared, 1, args.width, args.height, rs.format.y8, args.fps)
    pipeline = rs.pipeline()
    try:
        profile = pipeline.start(config)
    except RuntimeError as e:
        print(f"[depth_check] ❌ 深度を開けない: {e}")
        return 1
    try:
        scale = float(profile.get_device().first_depth_sensor().get_depth_scale())
        di = profile.get_stream(rs.stream.depth).as_video_stream_profile().get_intrinsics()
        print(f"[depth_check] ✅ 深度を開けた。単位 {scale} m、"
              f"深度の内部パラメータ fx={di.fx:.1f} fy={di.fy:.1f} cx={di.ppx:.1f} cy={di.ppy:.1f}")
        t0 = time.time()
        got = 0
        for _ in range(args.frames):
            frames = pipeline.wait_for_frames(5000)
            depth = frames.get_depth_frame()
            if not depth:
                continue
            got += 1
            d = np.asanyarray(depth.get_data())
            h, w = d.shape
            center = d[h // 2 - 5:h // 2 + 5, w // 2 - 5:w // 2 + 5]
            valid = center[center > 0]
            if got == 1 or got == args.frames:
                dist = f"{float(np.median(valid)) * scale:.3f} m" if valid.size else "測れない（0）"
                ir_note = ""
                if args.ir:
                    ir = frames.get_infrared_frame(1)
                    ir_note = f"、IR 平均の明るさ {float(np.asanyarray(ir.get_data()).mean()):.0f}" if ir else "、IR なし"
                print(f"[depth_check] {got} 枚目: 中央の距離 {dist}、"
                      f"有効な画素 {100.0 * float((d > 0).mean()):.0f}%{ir_note}")
        dt = time.time() - t0
        print(f"[depth_check] {got}/{args.frames} 枚を {dt:.1f} 秒で受け取った（{got / dt:.1f} fps）")
    finally:
        pipeline.stop()
    return 0


if __name__ == "__main__":
    sys.exit(main())
