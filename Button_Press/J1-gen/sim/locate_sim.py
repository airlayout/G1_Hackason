"""MuJoCo で、頭カメラの画像と深度からボトルの位置（pelvis 座標）を求め、正解と比べる（タスク4）。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/locate_sim.py
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/sim/locate_sim.py --waist-deg 10 0 5 --save

検出は、YOLO の代わりに MuJoCo のセグメンテーションで枠を作る（YOLO は作り物のボトルを検出しないため）。
--yolo を付けると YOLO も試す。正解は、ボトルの胴の前面（ロボット側）の、基準点と同じ高さの点。
"""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import REPO_ROOT, load_config  # noqa: E402
from common.localize import Locator, make_detector  # noqa: E402
from common.sim_camera import SegmentationDetector, SimHeadCamera  # noqa: E402
from common.sim_scene import BOTTLE, build_scene_model, detection_pose  # noqa: E402


def locate_in_sim(waist_deg: tuple[float, float, float] = (0.0, 0.0, 0.0), use_yolo: bool = False,
                  save: bool = False) -> dict[str, object]:
    """シーンを描画してボトルの位置を求める。結果（求めた点、正解の点、誤差 [m] など）を返す。"""
    import mujoco

    robot_cfg = load_config("robot.yaml")
    scene_cfg = load_config("sim_scene.yaml")
    loc_cfg = load_config("localize.yaml")
    m = build_scene_model(robot_cfg, scene_cfg)
    d = mujoco.MjData(m)
    q = detection_pose(scene_cfg, np.zeros(29))
    q_waist = np.radians(np.asarray(waist_deg, dtype=float))
    q[12:15] = q_waist
    d.qpos[:29] = q
    mujoco.mj_forward(m, d)

    cam = SimHeadCamera(m, d, robot_cfg)
    frame = cam.render()
    detector = make_detector(loc_cfg) if use_yolo else SegmentationDetector(cam, BOTTLE)
    loc = Locator(robot_cfg, loc_cfg, detector)
    found = loc.locate(frame, q_waist)
    best = loc.best(found)
    for x in found:
        print(f"[locate_sim] {x.summary()}")
    result: dict[str, object] = {"found": found, "best": best}
    if best is not None:
        # 正解: 基準点の画素へ飛ばした光線が当たる点（深度画像を使わずに求めた、同じ画素の本当の位置）と、
        # ボトルの胴の前面の点
        u, v = best.pixel
        truth_ray = cam.ray_hit_pelvis(u, v)
        b = scene_cfg["bottle"]
        assert best.p_pelvis is not None
        front_x = b["xy"][0] - b["radius"]
        result.update(truth_ray=truth_ray, err_ray=float(np.linalg.norm(best.p_pelvis - truth_ray)),
                      err_front_x=float(best.p_pelvis[0] - front_x),
                      err_center_y=float(best.p_pelvis[1] - b["xy"][1]))
        print(f"[locate_sim] 光線が当たる点との差 {result['err_ray'] * 1000:.1f} mm、"
              f"胴の前面との x の差 {result['err_front_x'] * 1000:+.1f} mm、"
              f"ボトルの中心との y の差 {result['err_center_y'] * 1000:+.1f} mm")
    else:
        print("[locate_sim] ボトルの位置が求まらなかった")
    if save:
        import cv2

        out = REPO_ROOT / "_local" / "button_press" / "sim"
        out.mkdir(parents=True, exist_ok=True)
        img = frame.color_bgr.copy()
        for x in found:
            x1, y1, x2, y2 = (int(round(t)) for t in x.bbox)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(img, (int(x.pixel[0]), int(x.pixel[1])), 5, (0, 255, 255), -1)
        cv2.imwrite(str(out / "locate_sim.png"), img)
        print(f"[locate_sim] 保存: {out / 'locate_sim.png'}")
    cam.close()
    return result


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--waist-deg", type=float, nargs=3, default=[0.0, 0.0, 0.0], metavar=("YAW", "ROLL", "PITCH"))
    p.add_argument("--yolo", action="store_true", help="セグメンテーションの代わりに YOLO で検出する")
    p.add_argument("--save", action="store_true", help="枠と基準点を描いた画像を _local/button_press/sim/ に保存")
    args = p.parse_args()
    r = locate_in_sim(tuple(args.waist_deg), args.yolo, args.save)
    return 0 if r.get("best") is not None else 1


if __name__ == "__main__":
    sys.exit(main())
