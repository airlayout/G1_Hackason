"""追従の検証用に、手の動きを1回だけ動画に録る（約50秒）。

使い方: .venv/bin/python record_motion.py [カメラ番号]
macOS の `say` が、次にやる動きを声で案内する。録った動画は debug/motion-日時.mp4（鏡像にはしない生の映像）。
この動画を何度でも再生して、手の特徴量→ロボット姿勢の計算を、人が動かずに確かめる。
"""
import subprocess
import sys
import time
from pathlib import Path

import cv2

CAMERA = int(sys.argv[1]) if len(sys.argv) > 1 else 0
WIDTH, HEIGHT, FPS = 1280, 960, 30                      # 画面の画角 4:3 に合わせる
START_COUNT = 5
FOLDER = Path(__file__).parent / "debug"

# （開始秒, 声で案内する文）。手のひらは、いつもカメラへ向ける
SCRIPT = [
    (0, "手のひらをカメラへ向けて、肩の高さで、止まってください"),
    (6, "腕を、まっすぐ上へ上げて、下ろします。2回"),
    (16, "腕を、カメラへ近づけて、遠ざけます。2回"),
    (26, "手を、左右へ動かします。2回"),
    (36, "腕は動かさず、親指と人差し指を、開いて、くっつけます。3回"),
    (48, "指をくっつけたまま、腕を上げて、下ろします"),
    (58, "終わりです。手を下ろしてください"),
]
TOTAL_S = 62


def say(text):
    print(f"[案内] {text}")
    subprocess.Popen(["say", "-v", "Kyoko", text], stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def main():
    cap = cv2.VideoCapture(CAMERA)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, WIDTH)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, HEIGHT)
    if not cap.isOpened():
        sys.exit("[録画] カメラを開けません（カメラ番号を変えるか、権限を確認）")
    ok, frame = cap.read()
    if not ok:
        sys.exit("[録画] 映像を読めません")
    h, w = frame.shape[:2]
    FOLDER.mkdir(exist_ok=True)
    path = FOLDER / f"motion-{time.strftime('%Y%m%d-%H%M%S')}.mp4"
    out = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"), FPS, (w, h))
    print(f"[録画] {w}x{h} → {path}")
    say(f"{START_COUNT}秒後に始めます。カメラの前に立ってください")
    began = time.time() + START_COUNT
    pending = list(SCRIPT)
    while True:
        ok, frame = cap.read()
        now = time.time()
        if not ok:
            break
        if now < began:
            cv2.putText(frame, f"start in {began - now:.0f}", (30, 60), cv2.FONT_HERSHEY_SIMPLEX, 1.5, (0, 200, 255), 3)
            cv2.imshow("record (q: quit)", frame)
        else:
            t = now - began
            if pending and t >= pending[0][0]:
                say(pending.pop(0)[1])
            out.write(frame)
            cv2.imshow("record (q: quit)", frame)
            if t >= TOTAL_S:
                break
        if cv2.waitKey(1) & 0xFF == ord("q"):
            break
    cap.release()
    out.release()
    cv2.destroyAllWindows()
    print(f"[録画] 保存しました: {path}")


if __name__ == "__main__":
    main()
