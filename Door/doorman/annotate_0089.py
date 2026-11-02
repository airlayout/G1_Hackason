"""Isaac Sim の描画（台ごとの mp4）を 1 回目の終わりで切り、各コマに 経過秒・段階・扉の角度・上半身の関節の速さの最大と打ち切りの上限を
書き込み、最後に終わった理由（パッチ 0089 の term_reason）を 1.5 秒出す。OMEN で回す（字体は Noto Sans CJK、ffmpeg は imageio_ffmpeg のもの）。
上限（termination_level × 20 rad/s）は、評価が写した重み（ckpt/<記録の名前>/last.pt）の env_state_dict から読む（設定より重みの値が効く）。

使い方: python annotate_0089.py <記録（term_analysis.py の per_env.json がある）> <出力先> <名前の頭>
"""
from __future__ import annotations

import glob
import json
import subprocess
import sys
from pathlib import Path

import imageio_ffmpeg
import numpy as np
from PIL import Image, ImageDraw, ImageFont

STAGE_NAMES = ("近づく", "構える", "つかむ", "レバー", "押し開け", "通り抜け")
FONT_B = "/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc"
FONT_R = "/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc"
OK_RGB, NG_RGB = (40, 160, 70), (200, 50, 50)
FPS = 50
FFMPEG = imageio_ffmpeg.get_ffmpeg_exe()
PART = {
    "shoulder_pitch": "肩の前後", "shoulder_roll": "肩の開き", "shoulder_yaw": "肩のひねり", "elbow": "肘",
    "wrist_roll": "手首のひねり", "wrist_pitch": "手首の上下", "wrist_yaw": "手首の左右",
    "waist_yaw": "腰のひねり", "waist_roll": "腰の左右", "waist_pitch": "腰の前後",
}


def speed_limit(rec: Path) -> float | None:
    """評価が写した重みの打ち切りの上限 [rad/s]。重みが無ければ None"""
    ckpt = rec.parent.parent / "ckpt" / rec.name / "last.pt"
    if not ckpt.exists():
        return None
    import torch

    state = torch.load(ckpt, map_location="cpu", weights_only=False).get("env_state_dict", {})
    return 20.0 * float(state["termination_level"]) if "termination_level" in state else None


def joint_ja(name: str) -> str:
    side = "左" if name.startswith("left_") else "右" if name.startswith("right_") else ""
    core = name.removeprefix("left_").removeprefix("right_").removesuffix("_joint")
    return side + PART.get(core, core)


def result_text(row: dict) -> str:
    if row["success"]:
        return "成功（通り抜けた）"
    stage = STAGE_NAMES[row["end_stage"]]
    if row["kind"] == "関節の速さ":
        return f"失敗：{stage}で {joint_ja(row['term_joint'])} {row['term_vel']:.0f} rad/s → 打ち切り"
    return f"失敗：{stage}で {row['kind']}"


def read_frames(path: Path, n: int) -> np.ndarray:
    w, h = imageio_ffmpeg.read_frames(str(path)).__next__()["size"]  # 動画の大きさ（評価の RENDER_W・RENDER_H）
    raw = subprocess.run([FFMPEG, "-v", "error", "-i", str(path), "-frames:v", str(n), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                         capture_output=True, check=True).stdout
    return np.frombuffer(raw, np.uint8).reshape(-1, h, w, 3)


def draw(frame: np.ndarray, lines: list[str], result: str | None, ok: bool) -> bytes:
    img = Image.fromarray(frame)
    W, H = img.size
    d = ImageDraw.Draw(img, "RGBA")
    f_title, f_body = ImageFont.truetype(FONT_B, 20), ImageFont.truetype(FONT_R, 16)
    d.rectangle([0, 0, W, 58], fill=(255, 255, 255, 215))
    d.text((12, 4), lines[0], font=f_title, fill=(20, 20, 20))
    d.text((12, 32), lines[1], font=f_body, fill=(40, 40, 40))
    if result:
        f_res = ImageFont.truetype(FONT_B, 20)
        w = d.textlength(result, font=f_res)
        x0, y0 = max(4, (W - w) / 2 - 14), H - 60
        d.rectangle([x0, y0, x0 + w + 28, y0 + 40], fill=(*(OK_RGB if ok else NG_RGB), 235))
        d.text((x0 + 14, y0 + 6), result, font=f_res, fill=(255, 255, 255))
    return np.asarray(img).tobytes()


def main() -> None:
    rec, out_dir, prefix = Path(sys.argv[1]), Path(sys.argv[2]), sys.argv[3]
    files = sorted(glob.glob(f"{rec}/states_*.npz"))
    keys = ("stage", "episode_length", "door_dof_pos", "upper_vel_max")
    st = {k: np.concatenate([np.load(f)[k] for f in files]) for k in keys}
    rows = {r["env"]: r for r in json.loads((rec / "per_env.json").read_text(encoding="utf-8"))}
    limit = speed_limit(rec)
    limit_text = f"（上限 {limit:.1f}）" if limit is not None else ""
    print(f"[annotate] 打ち切りの上限 {limit_text or '（重みが無い）'}")
    out_dir.mkdir(parents=True, exist_ok=True)
    for src in sorted((rec / "video").glob("env*.mp4")):
        e = int(src.stem[3:6])
        row = rows[e]
        ep = st["episode_length"][:, e]
        cut = np.where(ep[1:] < ep[:-1])[0]
        b = int(cut[0]) + 1 if len(cut) else len(ep)
        frames = read_frames(src, b)
        n = min(b, len(frames))  # 描画のコマと記録の歩は 1 対 1（どちらも 50 コマ/秒）
        H, W = frames.shape[1:3]
        result = result_text(row)
        tag = "ok" if row["success"] else {"関節の速さ": "speed", "扉が 90°": "open90", "扉から 4 m": "far",
                                           "倒れ": "fall", "持ち時間切れ": "overtime"}.get(row["kind"], "other")
        dst = out_dir / f"{prefix}_env{e:03d}_{tag}.mp4"
        enc = subprocess.Popen([FFMPEG, "-v", "error", "-y", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{W}x{H}", "-r", str(FPS),
                                "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "28", "-movflags", "+faststart", str(dst)],
                               stdin=subprocess.PIPE)

        def lines(t: int) -> list[str]:
            s = int(st["stage"][t, e])
            deg = abs(float(np.degrees(st["door_dof_pos"][t, e, 0])))
            vel = float(st["upper_vel_max"][t, e])
            return [f"{t / FPS:5.1f} 秒  段階 {s} {STAGE_NAMES[s]}", f"扉 {deg:.0f}°・上半身の関節の最大 {vel:.1f} rad/s{limit_text}"]

        for t in range(n):
            enc.stdin.write(draw(frames[t], lines(t), result if t == n - 1 else None, row["success"]))
        last = draw(frames[n - 1], lines(n - 1), result, row["success"])
        for _ in range(int(1.5 * FPS)):
            enc.stdin.write(last)
        enc.stdin.close()
        enc.wait()
        print(f"[annotate] {dst.name} {n} コマ（{n / FPS:.1f} 秒）{result}")


if __name__ == "__main__":
    main()
