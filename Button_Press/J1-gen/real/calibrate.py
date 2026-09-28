"""較正（タスク7、段階4）: カメラで求めた対象の位置と、指先で触れた位置（FK）を比べ、一定のずれを補正値にする。
lowstate と頭カメラを読むだけで、何も送信しない。

    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/calibrate.py --arm right
    G1_HuggingFace/venv/bin/python Button_Press/J1-gen/real/calibrate.py --arm right --write   # 補正値を書き込む

⚠️ 指先で触れるときは腕を手で動かすので、リモコンでダンピング（L2+B）状態にする。全身が脱力するので、
   必ず座った状態か、吊り下げた状態で行う（開始時に確認して Enter を待つ）。

1 か所の手順。**押す位置の近くで、ボトルの置き場所を変えて 3〜5 か所**繰り返す:
1. **腕をカメラの視野から外した状態で**、Enter → カメラでボトルの位置を求める（10 フレームの中央値。
   補正は入れない生の値）。基準点（深度を取った点）を黄色い点で描いた画像を _local/button_press/calibration/ に保存する
2. **そのあとで**、人が腕を持って、中指の先を基準点の位置（ボトルの胴の、黄色い点のところ）に触れさせ、
   Enter → FK で指先の位置を求める。このときカメラは使わない（手が写り込むと深度が狂うため、順番を逆にしない）
3. 差（指先 − カメラ）を記録する

最後に、場所ごとの差（補正値の候補）と、その平均からのずれを表で表示する。ばらつきが小さい（数 mm）なら
一定のずれなので、平均を
configs/localize.yaml の calibration.offset_pelvis_m にする（--write で書き込む）。
ばらつきが大きいなら、補正では直らない（カメラの向き、手先の点、関節の対応などを疑う）。

定規で pelvis の座標を直接測るのは難しいので、FK の指先を物差しの代わりに使う
（FK が正しいことは、先に fk_check.py で確かめておく）。
"""

from __future__ import annotations

import argparse
import copy
import json
import re
import sys
import time
from pathlib import Path

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))

from common.config import CONFIG_DIR, REPO_ROOT, load_config  # noqa: E402
from common.dds import LowStateReader  # noqa: E402
from common.localize import Locator, make_detector  # noqa: E402
from common.realday import FingertipFK, confirm_support  # noqa: E402
from common.robot_model import WAIST_IDX  # noqa: E402

LOCALIZE_YAML = CONFIG_DIR / "localize.yaml"


def write_offset(offset: np.ndarray, path: Path = LOCALIZE_YAML) -> None:
    """localize.yaml の offset_pelvis_m の行だけを書き換える（コメントは残す）。"""
    text = path.read_text(encoding="utf-8")
    new = "  offset_pelvis_m: [" + ", ".join(f"{v:.4f}" for v in offset) + "]"
    text2, n = re.subn(r"^  offset_pelvis_m: \[.*\]$", new, text, flags=re.MULTILINE)
    if n != 1:
        raise ValueError(f"{path} に offset_pelvis_m の行が 1 つだけ無い（{n} 個）")
    path.write_text(text2, encoding="utf-8")


def summarize(diffs: list[np.ndarray]) -> tuple[np.ndarray, np.ndarray]:
    d = np.array(diffs)
    return d.mean(axis=0), d.std(axis=0)


def place_table(pairs: list[dict]) -> list[str]:
    """場所ごとの位置・差（補正値の候補）・平均からのずれを、表の行にする（mm）。"""
    diffs = np.array([p["diff"] for p in pairs])
    mean = diffs.mean(axis=0)
    rows = ["  場所 | カメラの位置 x, y, z [m]      | 差（指先 − カメラ）x, y, z [mm] | 平均からのずれ [mm]"]
    for i, (p, d) in enumerate(zip(pairs, diffs), start=1):
        cam = ", ".join(f"{v:+.3f}" for v in p["camera_raw"])
        dd = ", ".join(f"{v * 1000:+6.1f}" for v in d)
        dev = np.linalg.norm(d - mean) * 1000
        rows.append(f"  {i:4d} | {cam} | {dd} | {dev:5.1f}")
    return rows


def main() -> int:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--arm", choices=["left", "right"], default="right", help="触れる指先の腕")
    p.add_argument("--frames", type=int, default=10, help="カメラで位置を求めるフレーム数（中央値を取る）")
    p.add_argument("--write", action="store_true", help="最後に、差の平均を localize.yaml に書き込む")
    p.add_argument("--network-interface", help="G1 につないでいる NIC（既定は configs/arm.yaml）")
    args = p.parse_args()

    if not confirm_support():
        print("[calib] 中止した")
        return 1
    from common.camera_rgbd import RgbdZmqSource

    robot_cfg = load_config("robot.yaml")
    arm_cfg = load_config("arm.yaml")
    loc_cfg = load_config("localize.yaml")
    old_offset = np.asarray(loc_cfg["calibration"]["offset_pelvis_m"], dtype=float)
    raw_cfg = copy.deepcopy(loc_cfg)
    raw_cfg["calibration"]["offset_pelvis_m"] = [0.0, 0.0, 0.0]  # 補正を入れない生の値で比べる
    locator = Locator(robot_cfg, raw_cfg, make_detector(loc_cfg))
    fk = FingertipFK(robot_cfg, load_config("press.yaml")["ik"])
    reader = LowStateReader(int(arm_cfg.get("domain_id", 0)), args.network_interface or arm_cfg["network_interface"])
    reader.open()
    reader.wait(10.0)
    cam = load_config("camera.yaml")["rgbd"]
    out = REPO_ROOT / "_local" / "button_press" / "calibration"
    out.mkdir(parents=True, exist_ok=True)
    stamp = time.strftime("%Y%m%d_%H%M%S")
    pairs: list[dict] = []

    with RgbdZmqSource(cam["server_address"], int(cam["port"]), int(cam["timeout_ms"])) as src:
        while True:
            ans = input(f"[calib] {len(pairs) + 1} か所目: ボトルを押す位置の近くに置き、腕をカメラの視野から外して"
                        " Enter（q で終わる）: ").strip().lower()
            if ans == "q":
                break
            pts = []
            last = None
            for _ in range(args.frames):
                f = src.read_rgbd()
                st = reader.latest()
                if f is None or st is None:
                    continue
                best = locator.best(locator.locate(f, st.q[list(WAIST_IDX)]))
                if best is not None and best.p_pelvis is not None:
                    pts.append(best.p_pelvis)
                    last = (f, best)
            if len(pts) < max(3, args.frames // 2) or last is None:
                print(f"[calib] ❌ ボトルの位置が求まらない（{len(pts)}/{args.frames} フレーム）。置き方・照明を変えてやり直す")
                continue
            p_cam = np.median(np.array(pts), axis=0)
            spread = np.ptp(np.array(pts), axis=0)
            f, best = last
            img = f.color_bgr.copy()
            x1, y1, x2, y2 = (int(round(t)) for t in best.bbox)
            cv2.rectangle(img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            cv2.circle(img, (int(best.pixel[0]), int(best.pixel[1])), 6, (0, 255, 255), -1)
            img_path = out / f"{stamp}_{len(pairs) + 1}.png"
            cv2.imwrite(str(img_path), img)
            print(f"[calib] カメラ（生の値）: {np.round(p_cam, 4)} m（{len(pts)} フレームのばらつき "
                  f"{np.round(spread * 1000, 1)} mm）。基準点の画像: {img_path}")
            ans = input("[calib] 中指の先を黄色い点の位置に触れさせて Enter（s でこの場所を飛ばす）: ").strip().lower()
            if ans == "s":
                continue
            st = reader.latest()
            assert st is not None
            p_fk = fk.positions(st.q)[args.arm]
            diff = p_fk - p_cam
            print(f"[calib] 指先（FK）: {np.round(p_fk, 4)} m → 差（指先 − カメラ）: {np.round(diff * 1000, 1)} mm")
            pairs.append({"camera_raw": p_cam.tolist(), "fingertip_fk": p_fk.tolist(), "diff": diff.tolist(),
                          "waist_q": st.q[list(WAIST_IDX)].tolist(), "image": str(img_path)})
            (out / f"{stamp}.json").write_text(json.dumps({"arm": args.arm, "pairs": pairs}, indent=2))

    if not pairs:
        print("[calib] 記録なし")
        return 1
    mean, std = summarize([np.array(x["diff"]) for x in pairs])
    print("[calib] 場所ごとの補正値の候補:")
    for row in place_table(pairs):
        print(f"[calib] {row}")
    if len(pairs) < 3:
        print("[calib] ⚠️ 3 か所未満。ばらつきが分からないので、できれば場所を増やす")
    print(f"[calib] {len(pairs)} か所の差の平均 {np.round(mean * 1000, 1)} mm、ばらつき（標準偏差）{np.round(std * 1000, 1)} mm")
    print(f"[calib] 今の補正値 {np.round(old_offset * 1000, 1)} mm → 新しい補正値の候補 {np.round(mean * 1000, 1)} mm")
    if np.any(std > 0.01):
        print("[calib] ⚠️ ばらつきが 1 cm を超える。一定のずれではないので、補正では直らない可能性がある")
    if np.linalg.norm(mean) > 0.05:
        print("[calib] ⚠️ ずれが 5 cm を超える。カメラの取り付け位置・手先の点・関節の対応を先に疑う")
    if args.write:
        write_offset(mean)
        print(f"[calib] {LOCALIZE_YAML} の offset_pelvis_m を書き換えた")
    else:
        print("[calib] 書き込むには --write を付けてやり直すか、localize.yaml の offset_pelvis_m を手で書き換える")
    print(f"[calib] 記録: {out / (stamp + '.json')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
