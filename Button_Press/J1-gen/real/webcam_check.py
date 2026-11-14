"""前方の Web カメラ（Innomaker）で ▲▼ が見えるか・どこにあると推定するかを確かめ、焦点距離と取り付けを較正する。腕も足も動かさない。

    # 1. 見るだけ: Web カメラの画像の ▲▼ と、大きさからの推定（較正前でも、--fx を仮に渡せば距離の目安が出る）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/webcam_check.py --frames 10 --fx 500
    # 2. 較正の組を集める: 頭カメラで ▲▼ が見える近さ（柱まで 35〜50 cm）で、両方のカメラが同じ ▲▼ を見た組を保存する。
    #    リモコンで立つ位置・向きを変えながら 5〜8 回（左右・距離をばらつかせる）。巻き尺で測ったカメラの位置を --xyz に
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/webcam_check.py --collect --xyz 0.10 0.0 0.30 --detector yolo
    # 3. 較正: 集めた組から焦点距離と向きを求め、configs/webcam.yaml に書く値を表示する（書き換えはしない）
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/webcam_check.py --fit _local/button_press/webcam/pairs.json
    # 4. 較正のあと: 柱の手前（0.6〜1.2 m）で、3 つの推定（柱の面との交点・大きさ・視差）を比べる
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/webcam_check.py --compare --detector yolo

映像は Mapping 班の配信（PC2 の 5555 番。configs/webcam.yaml の stream）。頭カメラは camera.yaml（rgbd_server は --no-legacy-rgb）。
画像は _local/button_press/webcam/<時刻>/ に残す（あとで設定を変えて読み直せるように）。
⚠️ YOLO の重みは頭カメラの画像で学習したもの。Web カメラの画像で ▲▼ が見つかるかは、まず 1 で確かめる。
"""

from __future__ import annotations

import argparse
import copy
import json
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.camera_cfg import add_camera_args, load_camera_config  # noqa: E402
from common.config import REPO_ROOT, load_config  # noqa: E402
from common.elevator_buttons import make_candidate_detector  # noqa: E402
from common.webcam_buttons import (  # noqa: E402
    WebcamModel,
    WebcamNotReady,
    WebcamStream,
    find_pair,
    fit_camera,
    from_size,
)

OUT = REPO_ROOT / "_local" / "button_press" / "webcam"


def fit(path: Path, cfg: dict, fx_guess: float, fit_xyz: bool) -> int:
    data = json.loads(path.read_text(encoding="utf-8"))
    pairs = [(np.asarray(r["p_pelvis"], dtype=float), np.asarray(r["uv"], dtype=float)) for r in data["pairs"]]
    if len(pairs) < 6:
        print(f"[webcam] 組が {len(pairs)} 個しかない（▲▼ で 2 個ずつ。3 回以上集めてから）")
        return 1
    c = cfg["camera"]
    w, h = (int(v) for v in c["size"])
    xyz = np.asarray(data.get("xyz") or c.get("xyz") or [0.1, 0.0, 0.3], dtype=float)
    cam0 = WebcamModel(fx_guess, fx_guess, (w - 1) / 2.0, (h - 1) / 2.0, xyz, np.radians(np.asarray(c["rpy_deg"], float)))
    cam, rms = fit_camera(pairs, cam0, fit_xyz=fit_xyz)
    print(f"[webcam] {len(pairs)} 個の組から求めた: 焦点距離 {cam.fx:.1f} px、向き rpy {np.round(np.degrees(cam.rpy), 2).tolist()}°"
          + (f"、位置 {np.round(cam.xyz, 3).tolist()} m" if fit_xyz else f"（位置は {np.round(cam.xyz, 3).tolist()} m のまま）")
          + f"、画素のずれ RMS {rms:.2f} px")
    if rms > 3.0:
        print("[webcam] ⚠️ 画素のずれが大きい（3 px 超）。組の取り違え・頭カメラの位置のずれ・位置の測り間違いを疑う")
    print("[webcam] configs/webcam.yaml の camera に書く値:")
    print(f"  fx: {cam.fx:.1f}\n  xyz: {np.round(cam.xyz, 3).tolist()}\n  rpy_deg: {np.round(np.degrees(cam.rpy), 2).tolist()}")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    mode = ap.add_mutually_exclusive_group()
    mode.add_argument("--collect", action="store_true", help="較正の組を集める（頭カメラでも ▲▼ を見る）")
    mode.add_argument("--fit", type=Path, metavar="PAIRS_JSON", help="集めた組から較正する")
    mode.add_argument("--compare", action="store_true", help="較正のあと、3 つの推定を比べる（頭カメラの柱の面も見る）")
    ap.add_argument("--frames", type=int, default=10)
    ap.add_argument("--fx", type=float, help="焦点距離 [px] の仮の値（較正前に、大きさからの距離の目安を出すため・--fit の初期値）")
    ap.add_argument("--xyz", type=float, nargs=3, help="巻き尺で測った Web カメラの位置（pelvis 座標 [m]）")
    ap.add_argument("--fit-xyz", action="store_true", help="--fit で位置も求める（組が多く、距離をばらつかせて集めたときだけ）")
    ap.add_argument("--detector", choices=["classic", "yolo"], help="▲▼ の見つけ方（既定は elevator.yaml）")
    ap.add_argument("--button-set", choices=["general", "wheelchair"], help="押す組（既定は webcam.yaml）")
    ap.add_argument("--network-interface")
    add_camera_args(ap)
    args = ap.parse_args()

    cfg = load_config("webcam.yaml")
    if args.button_set:
        cfg["buttons"]["button_set"] = args.button_set
    if args.fit:
        return fit(args.fit, cfg, args.fx or 500.0, args.fit_xyz)

    elevator = load_config("elevator.yaml")
    if args.detector:
        elevator["detector"] = args.detector
    elevator["geometry"]["button_set"] = cfg["buttons"]["button_set"]
    detector = make_candidate_detector(elevator)
    cam_cfg = load_camera_config(args)
    host = cfg["stream"].get("host") or cam_cfg.get("pc2_host") or cam_cfg["rgbd"]["server_address"]
    stream = WebcamStream(host, cfg)
    stream.open()
    try:
        cam = WebcamModel.from_config(cfg)
    except WebcamNotReady as e:
        cam = None
        if args.fx and args.xyz:
            c = copy.deepcopy(cfg)
            c["camera"].update(fx=args.fx, xyz=list(args.xyz))
            cam = WebcamModel.from_config(c)
            print(f"[webcam] ⚠️ 未較正。仮の焦点距離 {args.fx:g} px・位置 {list(args.xyz)} で計算する")
        elif args.compare:
            print(f"[webcam] ❌ {e}")
            return 1
    out = OUT / time.strftime("%Y%m%d_%H%M%S")
    out.mkdir(parents=True, exist_ok=True)

    head = assist = None
    if args.collect or args.compare:
        from common.approach import WebcamAssist
        from common.camera_rgbd import RgbdZmqSource
        from common.column_face import FaceFinder
        from common.dds import LowStateReader, init_dds
        from common.elevator_buttons import ButtonFinder
        from common.robot_model import WAIST_IDX

        iface = args.network_interface or load_config("arm.yaml")["network_interface"]
        init_dds(0, iface)
        reader = LowStateReader(0, iface)
        reader.open()
        reader.wait(10.0)
        rgbd = RgbdZmqSource(cam_cfg["rgbd"]["server_address"], int(cam_cfg["rgbd"]["port"]), int(cam_cfg["rgbd"]["timeout_ms"]))
        rgbd.open()
        robot_cfg = load_config("robot.yaml")
        head = {"rgbd": rgbd, "reader": reader, "buttons": ButtonFinder.from_config(robot_cfg, elevator),
                "face": FaceFinder.from_config(robot_cfg, load_config("approach.yaml")["face"]), "waist": list(WAIST_IDX)}
        if args.compare:
            assist = WebcamAssist(lambda: None, detector, cam, cfg)  # 画像は 1 枚ずつ渡す（下で get_frame を差し替える）

    pairs: list[dict] = []
    rows = []
    for k in range(args.frames):
        got = stream.read()
        if got is None:
            print("[webcam] ❌ Web カメラの映像が届かない（Mapping の配信 innomaker_camera_server.py が動いているか、5555 番）")
            return 1
        rgb, _ = got
        cv2.imwrite(str(out / f"webcam_{k:02d}.png"), rgb[:, :, ::-1])
        pair = find_pair(detector.detect(rgb), cfg)
        row: dict = {"frame": k, "found": pair is not None}
        if pair is None:
            print(f"[webcam] {k}: ▲▼ の組が見つからない")
            rows.append(row)
            continue
        row.update(uv=pair.mid_px.tolist(), spacing_px=pair.spacing_px, diam_px=[pair.up_diam_px, pair.down_diam_px])
        msg = f"[webcam] {k}: ▲▼ の中点 ({pair.mid_px[0]:.0f}, {pair.mid_px[1]:.0f}) px、間隔 {pair.spacing_px:.1f} px"
        if cam is not None:
            sz = from_size(cam, pair, cfg)
            if "by_spacing" in sz:
                p = sz["by_spacing"]
                msg += f"、間隔からの位置: 前 {p[0] * 100:.0f} cm・左右 {p[1] * 100:+.0f} cm"
                row["by_spacing"] = p.tolist()
        if head is not None:
            f = head["rgbd"].read_rgbd()
            st = head["reader"].latest()
            if f is not None and st is not None:
                i = f.intrinsics
                K = np.array([[i.fx, 0, i.cx], [0, i.fy, i.cy], [0, 0, 1.0]])
                qw = st.q[head["waist"]]
                if args.collect:
                    from common.elevator_buttons import ButtonNotFound

                    try:
                        call = head["buttons"].find(f.color_bgr[:, :, ::-1].copy(), f.depth_m(), K, qw)
                        for b, uv in ((call.up, pair.up_px), (call.down, pair.down_px)):
                            pairs.append({"p_pelvis": b.center.tolist(), "uv": uv.tolist()})
                        msg += f"、頭カメラ: {call.summary()} → 組を足した（{len(pairs)} 個）"
                    except ButtonNotFound as e:
                        msg += f"、頭カメラ: ▲▼ が見つからない（{e}）。もっと近づく"
                else:
                    from common.column_face import FaceNotFound

                    try:
                        face = head["face"].find(f.depth_m(), K, qw, st.imu_quat)
                        assist.get_frame = lambda: (rgb, 0.0)
                        est = assist.estimate(face)
                        row["face"] = face.summary()
                        row["estimates"] = est
                        if est is not None:
                            def fmt(key: str) -> str:
                                v = est.get(key)
                                return "—" if v is None else f"前 {v[0] * 100:.0f}・左右 {v[1] * 100:+.0f}"
                            msg += (f"\n          柱の面: {face.summary()}\n          交点 {fmt('plane')} / 間隔 {fmt('by_spacing')}"
                                    f" / 直径 {fmt('by_diameter')} / 視差 {fmt('parallax')} cm")
                    except FaceNotFound as e:
                        msg += f"、柱の面: {e}"
        print(msg)
        rows.append(row)
        time.sleep(0.3)
    stream.close()
    (out / "result.json").write_text(json.dumps({"rows": rows}, indent=2, ensure_ascii=False, default=float))
    if args.collect:
        path = OUT / "pairs.json"
        old = json.loads(path.read_text(encoding="utf-8"))["pairs"] if path.exists() else []
        path.write_text(json.dumps({"xyz": list(args.xyz) if args.xyz else None, "pairs": old + pairs}, indent=2))
        print(f"[webcam] 較正の組: 今回 {len(pairs)} 個、合わせて {len(old) + len(pairs)} 個 → {path}")
    print(f"[webcam] 保存: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
