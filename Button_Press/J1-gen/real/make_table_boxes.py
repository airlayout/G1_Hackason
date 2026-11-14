"""立った状態の深度から、机の障害物の箱（左右 9 cm ごと）を作る。ラボ PC で実行する。

    # ライブ（腰の角度は DDS の lowstate から読む）。まず確認だけ（何も保存しない）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/make_table_boxes.py --live
    # 確認してよければ保存する（configs/obstacles.yaml の table_1…table_n を置き換える）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/make_table_boxes.py --live --save
    # 保存したファイル / 収録から（腰の角度はメタデータ。--recording は --index のフレーム）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/make_table_boxes.py --files _local/button_press/probe/<stem>

なぜ深度から作るか: 吊り下げ・ダンピングで触って作ると、立った状態と机の見え方が変わる（2026-10-21、約 4 cm 手前・約 5 cm 低く
出た）。押すのは立った状態なので、立った状態の深度で作る。斜めの机は、左右 9 cm ごとの箱に分けて、縁を追う。
前提: 頭カメラの向きが合っていること（先に real/check_head_tilt.py。ずれていると机の高さ・手前の縁がずれる）。
腕を下ろした手や体が写っていると、箱が体の前を覆うので、確認の出力（各 strip の手前の縁）と画像（上から見た図）を見てから --save する。
"""

from __future__ import annotations

import argparse
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.camera_cfg import add_camera_args, apply_head_camera_mount, load_camera_config  # noqa: E402
from common.camera_geometry import HeadCameraTransform  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.realday import OBSTACLES, replace_obstacle_group, table_boxes_from_points  # noqa: E402
from common.rgbd_io import load_rgbd  # noqa: E402
from common.rgbd_protocol import RgbdFrame  # noqa: E402
from common.robot_model import WAIST_IDX  # noqa: E402


def pelvis_points(frame: RgbdFrame, tf: HeadCameraTransform, q_waist: np.ndarray, step: int = 3) -> np.ndarray:
    d = frame.depth_m()[::step, ::step]
    h, w = frame.depth.shape
    v, u = np.mgrid[0:h:step, 0:w:step]
    ok = (d > 0.2) & (d < 3.0)
    z = d[ok].astype(float)
    i = frame.intrinsics
    opt = np.stack([(u[ok] - i.cx) * z / i.fx, (v[ok] - i.cy) * z / i.fy, z], axis=1)
    r, p = tf.pelvis_from_optical(np.asarray(q_waist, dtype=float))
    return opt @ r.T + p + tf.offset


def draw_top_view(points: np.ndarray, boxes: list[dict], info: dict, path: Path) -> None:
    """上から見た図（x 前方が上、y 左が左）。点は薄く、箱の手前の縁を赤で。"""
    s, ox, oy = 600, 0.0, 0.0  # 1 m = 600 px
    img = np.full((540, 720, 3), 255, np.uint8)

    def px(x: float, y: float) -> tuple[int, int]:
        return int(360 - y * s), int(540 - 20 - (x - ox) * s)

    top = points[np.abs(points[:, 2] - info["z_top"]) <= 0.015]
    for x, y, _ in top[::5]:
        a, b = px(x, y)
        if 0 <= a < 720 and 0 <= b < 540:
            img[b, a] = (160, 160, 160)
    for b in boxes:
        c, h = np.array(b["center"]), np.array(b["half_size"])
        p1, p2 = px(c[0] - h[0], c[1] + h[1]), px(c[0] - h[0], c[1] - h[1])
        cv2.line(img, p1, p2, (0, 0, 255), 2)
    cv2.circle(img, px(0.0, 0.0), 5, (255, 0, 0), -1)
    cv2.putText(img, f"top view: z_top {info['z_top']:+.3f} m (red = front edge of boxes)", (8, 18),
                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 1)
    cv2.imwrite(str(path), img)


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    src = p.add_mutually_exclusive_group(required=True)
    src.add_argument("--live", action="store_true")
    src.add_argument("--files", type=Path)
    src.add_argument("--recording", type=Path)
    p.add_argument("--index", type=int, default=0, help="--recording で使うフレーム番号")
    p.add_argument("--prefix", default="table", help="箱の名前の前につける語（table_1…）")
    p.add_argument("--save", action="store_true", help="configs/obstacles.yaml に保存する（既定は確認だけ）")
    p.add_argument("--obstacles-file", type=Path, default=OBSTACLES)
    # 2026-10-31 に 0.03 → 0.01（計画の余裕 press.yaml の collision.clearance_m 0.01 は別にある）。実機でぶつかりそうなら戻す
    p.add_argument("--margin-m", type=float, default=0.01, help="箱のまわりの余裕 [m]")
    p.add_argument("--depth-m", type=float, default=0.6, help="手前の縁から奥への長さ")
    p.add_argument("--below-m", type=float, default=0.8, help="天板から下への長さ")
    p.add_argument("--strip-m", type=float, default=0.09, help="箱の左右の幅")
    p.add_argument("--note", default="")
    p.add_argument("--network-interface")
    add_camera_args(p)
    args = p.parse_args()

    robot_cfg = load_config("robot.yaml")
    cam_cfg = load_camera_config(args, quiet=True)
    apply_head_camera_mount(robot_cfg, cam_cfg)
    tf = HeadCameraTransform(robot_cfg, np.asarray(load_config("localize.yaml")["calibration"]["offset_pelvis_m"], dtype=float))

    if args.live:
        from common.camera_rgbd import RgbdZmqSource
        from common.dds import LowStateReader

        reader = LowStateReader(0, args.network_interface or load_config("arm.yaml")["network_interface"])
        reader.open()
        reader.wait(10.0)
        cam = cam_cfg["rgbd"]
        with RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"])) as s:
            frame = s.read_rgbd()
        if frame is None:
            print("[table] ❌ 深度付きストリームが届かない")
            return 1
        st = reader.latest()
        q_waist = st.q[list(WAIST_IDX)].copy()
    elif args.files:
        frame, meta = load_rgbd(args.files)
        q_waist = np.asarray(meta.get("waist_q", np.zeros(3)), dtype=float)
    else:
        from common.recording import Recording

        fr = Recording(args.recording).load(args.index)
        if fr.rgbd is None:
            print("[table] ❌ そのフレームに深度が無い")
            return 1
        frame, q_waist = fr.rgbd, (fr.waist_q if fr.waist_q is not None else np.zeros(3))

    pts = pelvis_points(frame, tf, q_waist)
    try:
        boxes, info = table_boxes_from_points(pts, args.margin_m, args.depth_m, args.below_m, args.strip_m)
    except ValueError as e:
        print(f"[table] ❌ {e}")
        return 1
    print(f"[table] 天板の高さ z {info['z_top']:+.3f} m（pelvis 座標。{info['n_top_points']} 点）、箱 {len(boxes)} 個")
    for k, st_ in enumerate(info["strips"], start=1):
        print(f"[table]   {args.prefix}_{k}: y {st_['y_lo']:+.2f}〜{st_['y_hi']:+.2f} m、手前の縁 x {st_['x_front']:+.3f} m"
              f"（余裕 {args.margin_m * 100:.0f} cm 込みで {st_['x_front'] - args.margin_m:+.3f}）、{st_['n']} 点")
    out = REPO_ROOT / "_local" / "button_press" / "table_boxes"
    out.mkdir(parents=True, exist_ok=True)
    png = out / f"{time.strftime('%Y%m%d_%H%M%S')}.png"
    draw_top_view(pts, boxes, info, png)
    print(f"[table] 上から見た図: {png}")
    near = [s for s in info["strips"] if abs(s["y_lo"] + args.strip_m / 2) < 0.1 and s["x_front"] < 0.2]
    if near:
        print("[table] ⚠️ 体の正面（y ≈ 0）に、手前の縁が x 0.2 m 以内の箱がある。手や体を机と取り違えていないか、図で確かめる")
    if not args.save:
        print("[table] 保存していない（--save を付けると configs/obstacles.yaml に書く）")
        return 0
    names = replace_obstacle_group(args.prefix, boxes, args.note or f"{time.strftime('%Y-%m-%d')} 立った状態の深度から作った箱", args.obstacles_file)
    print(f"[table] 保存した: {args.obstacles_file}（{', '.join(names)}）。実機日が終わったらコミットして残す")
    return 0


if __name__ == "__main__":
    sys.exit(main())
