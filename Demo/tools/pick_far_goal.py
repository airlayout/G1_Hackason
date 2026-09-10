#!/usr/bin/env python3
"""地図の上を実際に辿って、指定した道のりだけ離れた到達可能な地点を選ぶ。

直線距離で選ぶと壁の向こう側を指してしまう。BFS で自由空間を辿るので、
**返る地点は必ず経路がある**（Nav2 が同じ経路を引くとは限らないが、
「そもそも行けない場所を指した」という失敗は無くなる）。

  python3 pick_far_goal.py <地図.yaml> <x> <y> [道のり m] [余裕 m]

## 実測（2026-09-10・uis_room_v3_clean）

出発 (5.0, -0.6) から自由空間を辿って届く最遠点は、要求する余裕で急に変わる。

    余裕 0.30 m … 146.0 m
    余裕 0.35 m … 146.0 m
    余裕 0.40 m … 107.8 m
    余裕 0.45 m … 107.8 m
    余裕 0.50 m …  33.8 m   ← デモの長距離はここで選んでいる
    余裕 0.60 m …   6.7 m   ← 通路が閉じる

Nav2 側は robot_radius 0.25 ／ inflation_radius 0.35。0.5 m を要求すれば余裕を持って通る。
この設定で選んだ (14.804, -5.128)（道のり 15.0 m ／ 直線 10.7 m）は、
Isaac Sim で 66 秒・error_code 0・到達誤差 0.06 m で走破した。
"""
import math, pathlib, sys
from collections import deque


def load_map(yaml_path: pathlib.Path):
    info = {}
    for line in yaml_path.read_text().splitlines():
        if ":" in line and not line.strip().startswith("#"):
            k, v = line.split(":", 1)
            info[k.strip()] = v.strip()
    res = float(info["resolution"])
    origin = eval(info["origin"])
    negate = int(info.get("negate", 0))
    occ_th = float(info.get("occupied_thresh", 0.65))
    free_th = float(info.get("free_thresh", 0.196))
    pgm = yaml_path.parent / pathlib.Path(info["image"]).name

    with open(pgm, "rb") as f:
        assert f.readline().strip() == b"P5"
        line = f.readline()
        while line.startswith(b"#"):
            line = f.readline()
        w, h = map(int, line.split())
        maxv = int(f.readline())
        data = f.read()
    return res, origin, negate, occ_th, free_th, w, h, maxv, data


def main() -> int:
    yaml_path = pathlib.Path(sys.argv[1])
    rx, ry = float(sys.argv[2]), float(sys.argv[3])
    want_m = float(sys.argv[4]) if len(sys.argv) > 4 else 14.0
    clear_m = float(sys.argv[5]) if len(sys.argv) > 5 else 0.6

    res, origin, negate, occ_th, free_th, w, h, maxv, data = load_map(yaml_path)

    # 自由セル。ROS の慣習どおり占有確率 p = (maxv - 画素) / maxv（negate なら反転）
    free = bytearray(w * h)
    for py in range(h):
        row = (h - 1 - py) * w          # pgm は上から、地図は下から
        for px in range(w):
            v = data[row + px]
            p = v / maxv if negate else (maxv - v) / maxv
            free[py * w + px] = 1 if p < free_th else 0

    # ロボットの半径ぶん縮める（壁に貼り付いた点を選ばない）
    r = max(1, round(clear_m / res))
    safe = bytearray(w * h)
    for y in range(r, h - r):
        for x in range(r, w - r):
            if not free[y * w + x]:
                continue
            ok = True
            for dy in range(-r, r + 1):
                base = (y + dy) * w
                if not all(free[base + x + dx] for dx in range(-r, r + 1)):
                    ok = False
                    break
            if ok:
                safe[y * w + x] = 1

    cx = int((rx - origin[0]) / res)
    cy = int((ry - origin[1]) / res)

    # 出発点が縮めた自由空間の外にいることがある（漂流で壁際に寄る）。近い安全セルへ寄せる
    if not safe[cy * w + cx]:
        best = None
        for y in range(max(0, cy - 60), min(h, cy + 60)):
            for x in range(max(0, cx - 60), min(w, cx + 60)):
                if safe[y * w + x]:
                    d = (x - cx) ** 2 + (y - cy) ** 2
                    if best is None or d < best[0]:
                        best = (d, x, y)
        if best is None:
            print("GOAL none  （出発点の周りに安全な自由空間が無い）")
            return 1
        _, cx, cy = best
        print(f"[INFO] 出発点を安全側へ {math.hypot(cx-(rx-origin[0])/res, cy-(ry-origin[1])/res)*res:.2f} m 寄せた")

    # BFS（4 近傍。道のりはセル数 × 解像度）
    INF = -1
    dist = [INF] * (w * h)
    dist[cy * w + cx] = 0
    q = deque([(cx, cy)])
    far = (0, cx, cy)
    while q:
        x, y = q.popleft()
        d = dist[y * w + x]
        if d > far[0]:
            far = (d, x, y)
        for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1)):
            nx, ny = x + dx, y + dy
            if 0 <= nx < w and 0 <= ny < h and safe[ny * w + nx] and dist[ny * w + nx] == INF:
                dist[ny * w + nx] = d + 1
                q.append((nx, ny))

    want_cells = want_m / res
    best = None
    for y in range(h):
        base = y * w
        for x in range(w):
            d = dist[base + x]
            if d < 0:
                continue
            err = abs(d - want_cells)
            if best is None or err < best[0]:
                best = (err, d, x, y)

    _, d, gx, gy = best
    wx = gx * res + origin[0]
    wy = gy * res + origin[1]
    sx, sy = cx * res + origin[0], cy * res + origin[1]
    print(f"[INFO] 到達できる最遠点は 道のり {far[0]*res:.1f} m")
    print(f"[INFO] 出発 ({sx:+.2f}, {sy:+.2f})  →  ゴール ({wx:+.2f}, {wy:+.2f})")
    print(f"[INFO] 道のり {d*res:.1f} m ／ 直線 {math.hypot(wx-sx, wy-sy):.1f} m")
    print(f"GOAL {wx:.3f} {wy:.3f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
