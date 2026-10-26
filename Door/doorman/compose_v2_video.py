"""V2 の動画を 1 回ずつ作る（Mac）: 左＝シミュレータを外から見た動画、右＝そのとき G1 の頭のカメラに写っていた画像と、推定・正解の値。

入力（exp_v2_video_1019.sh の出力を Mac に持ってきたもの）:
  <記録>/video/env###_*.mp4   外から（eval_from_start.sh の VIDEO=1。1 ステップ 1 枚、最初の 1 枚は 1 歩目の前）
  <記録>/summary.json         count_success.py の台ごとの成否
  <画像>/shard_*.npz          頭のカメラ（vision_dataset、毎ステップ。img・pose・angle・est・stage・env・episode）
出力: <出力先>/env###_<成否>.mp4（H.264・25 fps）

使い方: python compose_v2_video.py <記録> <画像> <出力先> [--envs 0 3 5]
"""
from __future__ import annotations

import argparse
import glob
import json
import math
import subprocess
from pathlib import Path

import cv2
import numpy as np
from PIL import Image, ImageDraw, ImageFont

FONT_PATH = "/System/Library/Fonts/ヒラギノ角ゴシック W4.ttc"
FONT = ImageFont.truetype(FONT_PATH, 17)
FONT_S = ImageFont.truetype(FONT_PATH, 14)
FONT_B = ImageFont.truetype("/System/Library/Fonts/ヒラギノ角ゴシック W6.ttc", 20)
STAGES = ["0 扉へ歩く", "1 構える", "2 レバーに手", "3 レバーを下げる", "4 押して開ける", "5 通り抜ける"]
LEFT_W, H = 640, 480
RIGHT_W = 520
STEP = 2  # 50 Hz を 25 fps に


def load_vision(d: str) -> dict:
    parts = {}
    for f in sorted(glob.glob(f"{d}/shard_*.npz")):
        z = np.load(f)
        for k in ("img", "pose", "angle", "est", "stage", "env", "episode", "step"):
            parts.setdefault(k, []).append(z[k])
    return {k: np.concatenate(v) for k, v in parts.items()}


def outcome(e: dict, max_deg: float) -> tuple[str, str]:
    if e["success"]:
        return "ok", "成功（扉の先 1.5 m まで通り抜けた）"
    if max_deg >= 88:
        return "fail_overpush", "失敗（扉を 90° まで押し切った）"
    if max_deg < 10:
        return "fail_noopen", f"失敗（扉が開かない・最大 {max_deg:.0f}°）"
    return "fail_stuck", f"失敗（扉 {max_deg:.0f}° で止まった・区間 {e['max_stage']}）"


def right_panel(gray: np.ndarray, t: float, st: int, pose, ang, est) -> Image.Image:
    panel = Image.new("RGB", (RIGHT_W, H), (24, 27, 33))
    d = ImageDraw.Draw(panel)
    d.text((14, 8), "G1 の頭のカメラ（推定のモデルが見る画像）", fill=(230, 232, 236), font=FONT_S)
    im = Image.fromarray(gray).convert("RGB").resize((480, 270), Image.BILINEAR)
    panel.paste(im, (20, 32))
    y0 = 314
    d.text((20, y0), f"{t:5.1f} 秒　区間 {STAGES[int(st)] if 0 <= st < 6 else st}", fill=(230, 232, 236), font=FONT)
    rows = [
        ("", "正解", "推定"),
        ("扉の角度", f"{math.degrees(ang):5.1f}°", f"{math.degrees(est[3]):5.1f}°"),
        ("前後 x", f"{pose[0]:+.2f} m", f"{est[0]:+.2f} m"),
        ("左右 y", f"{pose[1]:+.2f} m", f"{est[1]:+.2f} m"),
        ("向き", f"{math.degrees(pose[2]):+5.0f}°", f"{math.degrees(est[2]):+5.0f}°"),
    ]
    for i, (a, b, c) in enumerate(rows):
        yy = y0 + 30 + 26 * i
        col = (150, 156, 166) if i == 0 else (230, 232, 236)
        d.text((20, yy), a, fill=col, font=FONT)
        d.text((150, yy), b, fill=col, font=FONT)
        d.text((290, yy), c, fill=(120, 185, 245) if i else col, font=FONT)
    err = abs(math.degrees(est[3] - ang))
    d.text((400, y0 + 56), f"差 {err:4.1f}°", fill=(240, 170, 90) if err > 5 else (150, 156, 166), font=FONT_S)
    pe = math.hypot(est[0] - pose[0], est[1] - pose[1]) * 100
    d.text((400, y0 + 82), f"差 {pe:4.1f} cm", fill=(240, 170, 90) if pe > 5 else (150, 156, 166), font=FONT_S)
    return panel


def compose(env: int, video: Path, vis: dict, e: dict, out_dir: Path) -> Path:
    m = (vis["env"] == env) & (vis["episode"] == 0)
    idx = np.nonzero(m)[0]
    idx = idx[np.argsort(vis["step"][idx])]
    max_deg = float(np.degrees(vis["angle"][idx].max()))
    tag, title = outcome(e, max_deg)
    cap = cv2.VideoCapture(str(video))
    frames = []
    while True:
        ok, fr = cap.read()
        if not ok:
            break
        frames.append(fr)
    cap.release()
    n = min(len(idx), len(frames) - 1)
    tmp = out_dir / f"_tmp_env{env:03d}.mp4"
    W = LEFT_W + RIGHT_W
    writer = cv2.VideoWriter(str(tmp), cv2.VideoWriter_fourcc(*"mp4v"), 50 // STEP, (W, H + 36))
    for k in range(0, n, STEP):
        i = idx[k]
        left = Image.fromarray(cv2.cvtColor(cv2.resize(frames[k + 1], (LEFT_W, H)), cv2.COLOR_BGR2RGB))
        dl = ImageDraw.Draw(left)
        dl.text((12, 8), "シミュレータ（外から・固定のカメラ）", fill=(20, 20, 20), font=FONT_S)
        canvas = Image.new("RGB", (W, H + 36), (255, 255, 255))
        dc = ImageDraw.Draw(canvas)
        color = (20, 113, 74) if tag == "ok" else (168, 32, 44)
        dc.text((12, 7), f"env {env}・{title}", fill=color, font=FONT_B)
        canvas.paste(left, (0, 36))
        canvas.paste(right_panel(vis["img"][i], k / 50.0, int(vis["stage"][i]), vis["pose"][i], float(vis["angle"][i]), vis["est"][i]),
                     (LEFT_W, 36))
        writer.write(cv2.cvtColor(np.asarray(canvas), cv2.COLOR_RGB2BGR))
    writer.release()
    out = out_dir / f"env{env:03d}_{tag}.mp4"
    subprocess.run(["ffmpeg", "-y", "-loglevel", "error", "-i", str(tmp), "-c:v", "libx264", "-pix_fmt", "yuv420p",
                    "-crf", "30", "-preset", "slow", "-movflags", "+faststart", str(out)], check=True)
    tmp.unlink()
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("vision")
    ap.add_argument("out")
    ap.add_argument("--envs", nargs="*", type=int)
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    vis = load_vision(a.vision)
    per_env = {e["env"]: e for e in json.load(open(Path(a.record) / "summary.json"))["per_env"]}
    for env in a.envs if a.envs else sorted(per_env):
        vids = sorted(Path(a.record, "video").glob(f"env{env:03d}_*.mp4"))
        if not vids:
            print(f"env {env}: 外からの動画が無い")
            continue
        p = compose(env, vids[0], vis, per_env[env], out)
        print(f"書いた {p}（{p.stat().st_size / 1e6:.1f} MB）")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
