"""ステレオ SLAM を電話なしで通すための、合成の部屋を 2 台のカメラで歩く映像。

    python tests/make_synthetic_room.py <TUM の rgb フォルダ> <出す所> [--secs 30]

縦 2.5 m・横 4 m・奥行 3 m の箱の部屋の 6 面に TUM の写真を貼り、K_TRUE のカメラ 2 台
（右は左の +x に BASELINE_M）で歩く。出す物:
  room_left.mp4・room_right.mp4  左右の映像（1280x720・30 fps）
  room_gt.txt                    正解の左のカメラの軌跡（TUM の形: 秒 x y z qx qy qz qw。メートル）
世界の座標は最初のカメラ（x 右・y 下・z 前）。make_synthetic_stereo.py の較正の映像と同じ K・基線。
"""

from __future__ import annotations

import argparse
import pathlib

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

SIZE = (1280, 720)
FPS = 30
K_TRUE = np.array([[820.0, 0, 640.0], [0, 815.0, 362.0], [0, 0, 1]])
BASELINE_M = 0.12
HALF = np.array([2.0, 1.25, 1.5])  # 部屋の半分の大きさ（x 横・y 縦・z 奥行）
TEX_PX_PER_M = 400.0


def load_textures(tum_rgb: pathlib.Path) -> list[np.ndarray]:
    files = sorted(tum_rgb.glob("*.png"))
    picks = [files[int(i)] for i in np.linspace(0, len(files) - 1, 6)]
    return [cv2.imread(str(f)) for f in picks]


def render(c2w_R: np.ndarray, c2w_t: np.ndarray, textures: list[np.ndarray]) -> np.ndarray:
    """箱の部屋の中のカメラから見た画（光線と 6 面の交わりを画素ごとに求める）"""
    w, h = SIZE
    u, v = np.meshgrid(np.arange(w) + 0.5, np.arange(h) + 0.5)
    rays_c = np.stack([(u - K_TRUE[0, 2]) / K_TRUE[0, 0], (v - K_TRUE[1, 2]) / K_TRUE[1, 1], np.ones_like(u)], -1)
    rays_w = rays_c @ c2w_R.T
    best_t = np.full((h, w), np.inf)
    plane_id = np.full((h, w), -1)
    for axis in range(3):
        for sign in (-1, 1):
            with np.errstate(divide="ignore", invalid="ignore"):
                t = (sign * HALF[axis] - c2w_t[axis]) / rays_w[..., axis]
            ok = (t > 1e-6) & (t < best_t)
            best_t[ok] = t[ok]
            plane_id[ok] = axis * 2 + (sign > 0)
    hit = c2w_t + rays_w * best_t[..., None]
    out = np.zeros((h, w, 3), np.uint8)
    for pid in range(6):
        m = plane_id == pid
        if not m.any():
            continue
        axis = pid // 2
        a, b = [k for k in range(3) if k != axis]
        tex = textures[pid]
        mx = ((hit[..., a] + HALF[a]) * TEX_PX_PER_M).astype(np.float32)
        my = ((hit[..., b] + HALF[b]) * TEX_PX_PER_M).astype(np.float32)
        sampled = cv2.remap(tex, mx, my, cv2.INTER_LINEAR, borderMode=cv2.BORDER_REFLECT_101)
        out[m] = sampled[m]
    return out


def trajectory(secs: float):
    """左のカメラ→世界の姿勢。部屋の中で大きな楕円を歩き、首を左右に振る"""
    for k in range(int(secs * FPS)):
        s = k / FPS
        a = 2 * np.pi * s / secs
        pos = np.array([0.9 * np.sin(a), 0.1 * np.sin(3 * a), 0.6 * (1 - np.cos(a))]) - np.array([0, 0, 0.3])
        yaw = 0.6 * np.sin(a) + 0.25 * np.sin(2.3 * a)
        pitch = 0.08 * np.sin(1.7 * a)
        R = Rotation.from_euler("yx", [yaw, pitch]).as_matrix()
        yield s, R, pos


def main() -> None:
    p = argparse.ArgumentParser()
    p.add_argument("tum_rgb")
    p.add_argument("out")
    p.add_argument("--secs", type=float, default=30.0)
    args = p.parse_args()
    out = pathlib.Path(args.out)
    out.mkdir(parents=True, exist_ok=True)
    textures = load_textures(pathlib.Path(args.tum_rgb))
    four = cv2.VideoWriter_fourcc(*"mp4v")
    wl = cv2.VideoWriter(str(out / "room_left.mp4"), four, FPS, SIZE)
    wr = cv2.VideoWriter(str(out / "room_right.mp4"), four, FPS, SIZE)
    first = None
    with open(out / "room_gt.txt", "w") as gt:
        for s, R, pos in trajectory(args.secs):
            wl.write(render(R, pos, textures))
            wr.write(render(R, pos + R @ np.array([BASELINE_M, 0, 0]), textures))
            if first is None:
                first = (R, pos)
            R0, p0 = first  # 最初のカメラを世界にする
            Rr, pr = R0.T @ R, R0.T @ (pos - p0)
            q = Rotation.from_matrix(Rr).as_quat()
            gt.write(f"{s:.6f} " + " ".join(f"{x:.6f}" for x in [*pr, *q]) + "\n")
    wl.release()
    wr.release()
    print(f"[合成] 部屋 {args.secs:.0f} 秒 → {out}")


if __name__ == "__main__":
    main()
