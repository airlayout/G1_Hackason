"""段 5（10-20）: レバーへの当たり方の動画を 1 回ずつ作る（Mac）。

左＝シミュレータを外から見た動画、右上＝レバーの周りの拡大、右下＝手がレバーから受ける力（物理 1 回ごとの最大）と
手のひらの速さのグラフ。初めて触れた瞬間の前後は 1/4 の速さで流す。

入力（exp_touch_video_1020.sh の出力を Mac に持ってきたもの）:
  <記録>/video/env###_*.mp4   外から（eval_from_start.sh の VIDEO=1。1 ステップ 1 枚、最初の 1 枚は 1 歩目の前）
  <記録>/states_*.npz         毎ステップの状態（pregrasp_diag・lever_force_substeps）
  <記録>/summary.json         count_success.py の台ごとの成否
出力: <出力先>/<名前>_<ok|fail>_env###.mp4（H.264・25 fps）

使い方: python compose_touch_video.py <記録> <出力先> --name <名前> --label <条件の名前> [--envs 0 3]
"""
from __future__ import annotations

import argparse
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
from PIL import Image, ImageDraw, ImageFont

sys.path.insert(0, str(Path(__file__).parent))
from touch_metrics import DT, TOUCH_N, episodes, load  # noqa: E402


def read_frames(video: Path) -> list[np.ndarray]:
    """mp4 を RGB の配列の並びにする（ffmpeg で読む。OpenCV は使わない）"""
    probe = subprocess.run(["ffprobe", "-v", "error", "-select_streams", "v:0", "-show_entries", "stream=width,height",
                            "-of", "csv=p=0", str(video)], capture_output=True, text=True, check=True,
                           stdin=subprocess.DEVNULL)
    w, h = (int(x) for x in probe.stdout.strip().split(","))
    raw = subprocess.run(["ffmpeg", "-v", "error", "-i", str(video), "-f", "rawvideo", "-pix_fmt", "rgb24", "-"],
                         capture_output=True, check=True, stdin=subprocess.DEVNULL).stdout  # 呼び出し元の標準入力を読まない
    return list(np.frombuffer(raw, np.uint8).reshape(-1, h, w, 3))


def open_writer(out: Path, w: int, h: int) -> subprocess.Popen:
    """RGB の配列を受け取って H.264 の mp4 に書く ffmpeg"""
    return subprocess.Popen(["ffmpeg", "-y", "-loglevel", "error", "-f", "rawvideo", "-pix_fmt", "rgb24", "-s", f"{w}x{h}",
                             "-r", str(50 // STEP), "-i", "-", "-c:v", "libx264", "-pix_fmt", "yuv420p", "-crf", "30",
                             "-preset", "slow", "-movflags", "+faststart", str(out)], stdin=subprocess.PIPE)

FONT_PATH = "/System/Library/Fonts/ヒラギノ角ゴシック W4.ttc"
FONT = ImageFont.truetype(FONT_PATH, 17)
FONT_S = ImageFont.truetype(FONT_PATH, 13)
FONT_B = ImageFont.truetype("/System/Library/Fonts/ヒラギノ角ゴシック W6.ttc", 20)
STAGES = ["0 扉へ歩く", "1 構える", "2 レバーに手", "3 レバーを下げる", "4 押して開ける", "5 通り抜ける"]
LEFT_W, LEFT_H = 800, 600
RIGHT_W = 480
TOP = 40
STEP = 2  # 50 Hz を 25 fps に
SLOW = 4  # 触れた前後は 1/4 の速さ
SLOW_BEFORE_S, SLOW_AFTER_S = 0.3, 0.5
WINDOW_S = 4.0  # グラフに出す長さ
FORCE_MAX, SPEED_MAX = 100.0, 1.6
INK, MUTED, GRID = (230, 232, 236), (150, 156, 166), (58, 64, 74)
FORCE_C, SPEED_C, LIMIT_C = (235, 120, 70), (90, 160, 235), (80, 190, 130)


# 外から見るカメラ（eval_from_start.sh の RENDER_EYE・RENDER_TARGET の既定、env の原点から）。焦点距離は 10-20 に
# 2 台の触れた瞬間の手の写りから合わせた（960×720 で 872 画素＝水平の画角 58°、差は 10 画素以内）
CAM_EYE = np.array([-2.6, -1.4, 1.7])
CAM_TARGET = np.array([-0.2, -0.1, 0.9])
FOCAL_PX_960 = 872.0
ZOOM_W, ZOOM_H = 400, 191  # 右上の枠（452×216）と同じ縦横比
ZOOM_UP_PX = 30  # 構える点はレバーの 12 cm 上で、手は上から来るので、枠を少し上へ寄せる


def project(p: np.ndarray, width: int, height: int) -> tuple[float, float]:
    """env の原点から見た位置 p [m] を、外から見る動画の画素 (u, v) にする"""
    f = (CAM_TARGET - CAM_EYE) / np.linalg.norm(CAM_TARGET - CAM_EYE)
    right = np.cross(f, [0.0, 0.0, 1.0])
    right /= np.linalg.norm(right)
    up = np.cross(right, f)
    d = p - CAM_EYE
    focal = FOCAL_PX_960 * width / 960
    return width / 2 + focal * (d @ right) / (d @ f), height / 2 - focal * (d @ up) / (d @ f)


def zoom_box_for(lever: np.ndarray, width: int, height: int) -> tuple[int, int, int, int]:
    """レバーの当てる点を中心にした拡大の枠（画面の外へはみ出さないように寄せる）"""
    u, v = project(lever, width, height)
    w, h = ZOOM_W * width // 960, ZOOM_H * width // 960
    x = int(min(max(u - w / 2, 0), width - w))
    y = int(min(max(v - h / 2 - ZOOM_UP_PX * width / 960, 0), height - h))
    return x, y, w, h


def chart(d: ImageDraw.ImageDraw, box: tuple, t: np.ndarray, y: np.ndarray, now: float, ymax: float, limit: float,
          color: tuple, title: str, unit: str, touch_t: float | None) -> None:
    """1 本の量を、今の時刻までの WINDOW_S 秒ぶん描く（縦は 0〜ymax、目安の線つき）"""
    x0, y0, w, h = box
    d.text((x0, y0 - 20), title, fill=INK, font=FONT_S)
    for frac in (0.0, 0.5, 1.0):
        yy = y0 + h - h * frac
        d.line([(x0, yy), (x0 + w, yy)], fill=GRID)
        d.text((x0 + w + 4, yy - 8), f"{ymax * frac:g}", fill=MUTED, font=FONT_S)
    ly = y0 + h - h * min(limit / ymax, 1.0)
    d.line([(x0, ly), (x0 + w, ly)], fill=LIMIT_C, width=2)
    d.text((x0 + 4, ly - 17), f"目安 {limit:g} {unit}", fill=LIMIT_C, font=FONT_S)
    t0 = now - WINDOW_S
    if touch_t is not None and t0 <= touch_t <= now:
        tx = x0 + w * (touch_t - t0) / WINDOW_S
        d.line([(tx, y0), (tx, y0 + h)], fill=MUTED)
        d.text((tx + 3, y0 + 2), "触れた", fill=MUTED, font=FONT_S)
    m = (t >= t0) & (t <= now)
    if m.sum() >= 2:
        pts = [(x0 + w * (tt - t0) / WINDOW_S, y0 + h - h * min(v / ymax, 1.0)) for tt, v in zip(t[m], y[m])]
        d.line(pts, fill=color, width=2)
    d.text((x0, y0 + h + 4), f"{WINDOW_S:g} 秒前", fill=MUTED, font=FONT_S)
    d.text((x0 + w - 24, y0 + h + 4), "今", fill=MUTED, font=FONT_S)


def right_panel(zoom: Image.Image, now: float, stage: int, ts: np.ndarray, force: np.ndarray, speed: np.ndarray,
                ep: dict, touch_t: float | None, slow: bool, zoom_title: str) -> Image.Image:
    panel = Image.new("RGB", (RIGHT_W, LEFT_H), (24, 27, 33))
    d = ImageDraw.Draw(panel)
    d.text((14, 6), zoom_title, fill=INK, font=FONT_S)
    panel.paste(zoom.resize((RIGHT_W - 28, 216), Image.BILINEAR), (14, 26))
    d.text((14, 250), f"{now:5.2f} 秒　区間 {STAGES[stage] if 0 <= stage < 6 else stage}", fill=INK, font=FONT)
    if slow:
        d.text((RIGHT_W - 120, 250), "スロー 1/4", fill=(250, 200, 90), font=FONT)
    chart(d, (14, 300, RIGHT_W - 60, 100), ts, force, now, FORCE_MAX, 15.0, FORCE_C, "手がレバーから受ける力 [N]（物理 1 回ごとの最大）", "N", touch_t)
    chart(d, (14, 450, RIGHT_W - 60, 80), ts, speed, now, SPEED_MAX, 0.2, SPEED_C, "手のひらの速さ [m/s]", "m/s", touch_t)
    if touch_t is not None and now >= touch_t and ep.get("touched"):
        col = (80, 190, 130) if ep["first_peak_n"] <= 15 else (240, 120, 100)
        d.text((14, 560), f"当たりの山 {ep['first_peak_n']:.1f} N　触れる直前の速さ {ep['speed_before']:.2f} m/s", fill=col, font=FONT)
    return panel


def outcome(e: dict, max_deg: float) -> tuple[str, str]:
    if e["success"]:
        return "ok", "成功（扉を通り抜けた）"
    return "fail", f"失敗（区間 {STAGES[e['max_stage']]} で止まった・扉の最大 {max_deg:.0f}°）"


def frame_plan(n: int, touch: int | None) -> list[tuple[int, bool]]:
    """(記録の行, スローか) の並び。ふだんは 2 ステップに 1 枚、触れた前後は毎ステップを SLOW/STEP 回ずつ"""
    plan = []
    k = 0
    lo = touch - int(SLOW_BEFORE_S / DT) if touch is not None else n
    hi = touch + int(SLOW_AFTER_S / DT) if touch is not None else n
    while k < n:
        if lo <= k < hi:
            plan += [(k, True)] * (SLOW // STEP)  # ふだんは 1 ステップ 0.5 枚、スローは 2 枚＝1/4 の速さ
            k += 1
        else:
            plan.append((k, False))
            k += STEP
    return plan


CLOSE_CENTER = (0.55, 0.62)  # 横のカメラでレバーが写る所（画面の幅・高さに対する割合。10-20 に元の方策の 8 台で見た）
CLOSE_FRAC = 0.75  # 横のカメラの幅のうち切り出す割合（少し拡大する）


def close_crop(fr: np.ndarray) -> Image.Image:
    """横のカメラの 1 枚から、レバーの周りを右上の枠と同じ縦横比で切り出す"""
    h, w = fr.shape[:2]
    cw = int(w * CLOSE_FRAC)
    ch = int(cw * 216 / (RIGHT_W - 28))
    x = max(0, min(w - cw, int(w * CLOSE_CENTER[0] - cw / 2)))
    y = max(0, min(h - ch, int(h * CLOSE_CENTER[1] - ch / 2)))
    return Image.fromarray(fr[y:y + ch, x:x + cw])


def compose(env: int, video: Path, r: dict, ep: dict, summ: dict, name: str, label: str, out_dir: Path,
            close_video: Path | None = None) -> Path:
    stage, epl = r["stage"][:, env], r["episode_length"][:, env]
    ends = np.nonzero(epl[1:] < epl[:-1])[0]
    n = int(ends[0]) + 1 if len(ends) else len(stage)
    force = r["hand_lever_force_peak"][:n, env]
    palm = r["palm_used"][:n, env]
    speed = np.zeros(n)
    speed[1:] = np.linalg.norm(np.diff(palm, axis=0), axis=-1) / DT
    ts = np.arange(n) * DT
    hinge = np.degrees(r["door_dof_pos"][:n, env, 0])
    touch_idx = np.nonzero(force > TOUCH_N)[0]
    touch = int(touch_idx[0]) if len(touch_idx) else None
    touch_t = touch * DT if touch is not None else None
    tag, title = outcome(summ, float(hinge.max()))
    frames = read_frames(video)
    close = read_frames(close_video) if close_video is not None else None
    n = min(n, len(frames) - 1, len(close) - 1 if close is not None else n)
    W, H = LEFT_W + RIGHT_W, LEFT_H + TOP
    out = out_dir / f"{name}_{tag}_env{env:03d}.mp4"
    writer = open_writer(out, W, H)
    h0, w0 = frames[0].shape[:2]
    lever = r["grasp_target"][touch if touch is not None else n // 2, env]
    zx, zy, zw, zh = zoom_box_for(lever, w0, h0)
    for k, slow in frame_plan(n, touch):
        full = Image.fromarray(frames[k + 1])  # 最初の 1 枚は 1 歩目の前
        left = full.resize((LEFT_W, LEFT_H), Image.BILINEAR)
        dl = ImageDraw.Draw(left)
        sx, sy = LEFT_W / full.width, LEFT_H / full.height
        if close is None:
            dl.rectangle([zx * sx, zy * sy, (zx + zw) * sx, (zy + zh) * sy], outline=(250, 200, 90), width=2)
        canvas = Image.new("RGB", (W, H), (255, 255, 255))
        dc = ImageDraw.Draw(canvas)
        color = (20, 113, 74) if tag == "ok" else (168, 32, 44)
        dc.text((12, 9), f"{label}・env {env}・{title}", fill=color, font=FONT_B)
        canvas.paste(left, (0, TOP))
        zoom = close_crop(close[k + 1]) if close is not None else full.crop((zx, zy, zx + zw, zy + zh))
        canvas.paste(right_panel(zoom, k * DT, int(stage[k]), ts[:n], force[:n], speed[:n], ep, touch_t, slow,
                                 "レバーの横のカメラ" if close is not None else "レバーの周り（拡大）"), (LEFT_W, TOP))
        writer.stdin.write(np.asarray(canvas, dtype=np.uint8).tobytes())
    writer.stdin.close()
    if writer.wait() != 0:
        raise RuntimeError(f"ffmpeg が失敗した: {out}")
    return out


def main() -> int:
    ap = argparse.ArgumentParser()
    ap.add_argument("record")
    ap.add_argument("out")
    ap.add_argument("--name", required=True)
    ap.add_argument("--label", required=True)
    ap.add_argument("--envs", nargs="*", type=int)
    ap.add_argument("--close", help="同じ種で横のカメラから撮った記録（成否と触れたステップが同じことを確かめてから使う）")
    a = ap.parse_args()
    out = Path(a.out)
    out.mkdir(parents=True, exist_ok=True)
    r = load(a.record)
    eps = {e["env"]: e for e in episodes(r, "peak")}
    per_env = {e["env"]: e for e in json.load(open(Path(a.record) / "summary.json"))["per_env"]}
    close_r = load(a.close) if a.close else None
    for env in a.envs if a.envs else sorted(per_env):
        vids = sorted(Path(a.record, "video").glob(f"env{env:03d}_*.mp4"))
        if not vids or env not in eps:
            print(f"env {env}: 動画か記録が無い")
            continue
        close_video = None
        if close_r is not None:
            cv = sorted(Path(a.close, "video").glob(f"env{env:03d}_*.mp4"))
            fa, fb = close_r["hand_lever_force_peak"][:, env], r["hand_lever_force_peak"][:, env]
            m = min(len(fa), len(fb))
            same = bool(cv) and np.allclose(fa[:m], fb[:m], atol=1e-3)
            if same:
                close_video = cv[0]
            else:
                print(f"env {env}: 横のカメラの回が同じでない（力の記録が食い違う）。拡大の枠を使う")
        p = compose(env, vids[0], r, eps[env], per_env[env], a.name, a.label, out, close_video)
        e = eps[env]
        extra = f"当たりの山 {e['first_peak_n']:.1f} N・直前 {e['speed_before']:.2f} m/s" if e.get("touched") else "触れていない"
        print(f"書いた {p}（{p.stat().st_size / 1e6:.1f} MB）{extra}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
