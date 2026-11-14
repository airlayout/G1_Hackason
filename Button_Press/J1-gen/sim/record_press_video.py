"""ボトルを押す動作（MuJoCo、ボトル・机あり）を、実際の速さで再生される動画（25 fps、mp4）にする。

    P=G1_HuggingFace/venv/bin/python
    $P Button_Press/J1-gen/sim/record_press_video.py --case 0.32,-0.17,0.0                       # 今の設定
    $P Button_Press/J1-gen/sim/record_press_video.py --case 0.22,0.0,0.0 --overlay reach_wide.yaml --name near_center

配置は sim/reach_verify.py と同じ（机の縁 x・ボトルの左右 y・縁からボトルの手前の面まで gap [m]、天板は pelvis の高さ、
押す点は天板から 10 cm 上、計画の机は実機と同じ深い箱、開始の腕は 10/21 の実機の立った姿勢）。
シミュレーションの時間で 1 コマずつ撮るので、G1 の本当の速さが見える。計画が立たない配置は、腕を動かさずに終わる（その旨を画に出す）。
左は斜め前から見た G1、右は頭のカメラ。左上に経過時間と、腕の段階（[arm] の表示を英語にしたもの）を出す。
保存先: _local/button_press/videos/<名前>.mp4（git の対象外）。
"""

from __future__ import annotations

import argparse
import io
import sys
import tempfile
from pathlib import Path
from typing import Any

import cv2
import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
sys.path.insert(0, str(Path(__file__).resolve().parent))

from common.config import load_config  # noqa: E402
from common.realday import replace_obstacle_group  # noqa: E402
from press_bottle_sim import run_sim  # noqa: E402
from reach_map import desk_boxes  # noqa: E402
from reach_verify import PRESS_H, TOP_Z, make_args, scene_for  # noqa: E402

REPO = Path(__file__).resolve().parents[3]
OUT_DIR = REPO / "_local" / "button_press" / "videos"
FPS = 25
W, H = 640, 480
# [arm] の表示 → 画に出す英語（cv2 は日本語を描けない）
STEPS = [("経由の姿勢へ移動", "to via pose"), ("手前の姿勢へ移動", "to approach pose"), ("押し込み", "pressing"),
         ("経由の姿勢へ戻る", "back to via pose"), ("開始姿勢へ戻る", "back to start"), ("weight 0→1", "arm control on"),
         ("安全終了", "arm control off"), ("手前の姿勢へ戻る", "back to approach pose")]


class StepTee(io.TextIOBase):
    """標準出力をそのまま流しつつ、最後の [arm] の段階を覚える。"""

    def __init__(self, inner: Any) -> None:
        self.inner, self.step = inner, ""

    def write(self, s: str) -> int:
        for line in s.splitlines():
            if line.startswith("[arm]"):
                for ja, en in STEPS:
                    if ja in line:
                        self.step = en
        return self.inner.write(s)

    def flush(self) -> None:
        self.inner.flush()


def _text(img: np.ndarray, s: str, xy: tuple[int, int], scale: float = 0.7, color=(255, 255, 255)) -> None:
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 4, cv2.LINE_AA)
    cv2.putText(img, s, xy, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 2, cv2.LINE_AA)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--case", required=True, help="edge,y,gap [m]（例: 0.32,-0.17,0.0）")
    ap.add_argument("--overlay", help="設定の上書き（configs/ の YAML か、そのパス）。無ければ今の設定")
    ap.add_argument("--title", default="", help="画の上に出す説明（英語）")
    ap.add_argument("--name", default="", help="ファイル名（既定は case と設定から作る）")
    ap.add_argument("--azimuth", type=float, default=-130.0, help="斜め前から見るカメラの向き [度]")
    ap.add_argument("--elevation", type=float, default=-20.0)
    ap.add_argument("--distance", type=float, default=1.3)
    args = ap.parse_args()

    import mujoco

    edge, y, gap = (float(v) for v in args.case.split(","))
    setting = Path(args.overlay).stem if args.overlay else "default"
    out = OUT_DIR / f"{args.name or f'{setting}_{edge:.2f}_{y:+.2f}_{gap:.2f}'}.mp4"
    out.parent.mkdir(parents=True, exist_ok=True)
    obs_file = tempfile.NamedTemporaryFile(suffix=".yaml", delete=False).name
    replace_obstacle_group("table", desk_boxes(edge, TOP_Z, 0.0), "", Path(obs_file))
    run_args = make_args(args.overlay, [edge + gap, y, TOP_Z + PRESS_H], f"video_{setting}", obs_file)
    head_name = load_config("robot.yaml")["head_camera"]["name"]

    tee = StepTee(sys.stdout)
    st: dict[str, Any] = {"t0": None, "next": 0.0, "frames": 0}
    caption = f"{setting}  desk edge {edge * 100:.0f} cm, bottle y {y * 100:+.0f} cm, gap {gap * 100:.0f} cm"

    def hook(backend: Any) -> None:
        m, d = backend.model, backend.data
        side = mujoco.Renderer(m, height=H, width=W)
        head = mujoco.Renderer(m, height=H, width=W)
        cam = mujoco.MjvCamera()
        pelvis = mujoco.mj_name2id(m, mujoco.mjtObj.mjOBJ_BODY, "pelvis")
        mujoco.mj_forward(m, d)
        cam.lookat[:] = d.xpos[pelvis] + np.array([0.22, -0.12, 0.05])   # 体の右前（右腕とボトルの間）
        cam.azimuth, cam.elevation, cam.distance = args.azimuth, args.elevation, args.distance
        writer = cv2.VideoWriter(str(out), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (2 * W, H))
        st.update(side=side, head=head, writer=writer, d=d)

        def frame() -> np.ndarray:
            side.update_scene(d, camera=cam)
            head.update_scene(d, camera=head_name)
            img = np.hstack([side.render(), head.render()])[:, :, ::-1].copy()
            t0 = d.time if st["t0"] is None else st["t0"]   # 計画が断られると一度も進めずに終わる
            _text(img, f"t = {d.time - t0:5.2f} s   {tee.step}", (12, 30))
            _text(img, args.title or caption, (12, H - 16), 0.6)
            _text(img, "head camera", (W + 12, 30))
            return img

        st["frame"] = frame
        orig = backend.tick

        def tick(dt: float) -> None:
            orig(dt)
            if st["t0"] is None:
                st["t0"] = d.time
            while d.time - st["t0"] >= st["next"] - 1e-9:
                writer.write(frame())
                st["frames"] += 1
                st["next"] += 1.0 / FPS

        backend.tick = tick

    old = sys.stdout
    sys.stdout = tee
    try:
        res, _backend, _logger = run_sim(run_args, scene_name="sim_scene.yaml", scene_overrides=scene_for(edge, y, gap),
                                         on_backend=hook)
    finally:
        sys.stdout = old
    if "frame" in st:
        last = st["frame"]()
        ok = res.code == 0
        _text(last, "SUCCESS" if ok else "NOT PRESSED (plan rejected or stopped)", (12, 70), 1.0,
              (0, 255, 0) if ok else (0, 0, 255))
        hold = 2.0 if st["frames"] else 5.0   # 最後の画を保つ秒数（動かなかったときは、読めるように長め）
        for _ in range(int(hold * FPS)):
            st["writer"].write(last)
        st["writer"].release()
        st["side"].close()
        st["head"].close()
    print(f"[video] {out}（{st['frames'] / FPS:.1f} 秒分、終了 {res.code}: {res.message[:80]}）")
    return 0


if __name__ == "__main__":
    sys.exit(main())
