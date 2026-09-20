"""アダプタの口を叩いて、UI が期待する形で返ってくるかを確かめる。

⚠️ **「動いた」ではなく「何がどう返ったか」を出す。** 通ったことだけ表示すると、
配線が半分しか効いていなくても気づけない。
"""
from __future__ import annotations

import json
import sys
import time
import urllib.error
import urllib.request

B = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8081"
ok_all = True


def check(label: str, cond: bool, detail: str = "") -> None:
    global ok_all
    ok_all = ok_all and cond
    print(f"  {'✅' if cond else '❌'} {label}{(' — ' + detail) if detail else ''}")


def get(path: str):
    with urllib.request.urlopen(B + path, timeout=5) as r:
        return r.status, r.read()


def post(path: str):
    req = urllib.request.Request(B + path, method="POST", data=b"")
    try:
        with urllib.request.urlopen(req, timeout=10) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


# --- 起動待ち ---------------------------------------------------------------
for _ in range(120):
    try:
        get("/healthz")
        break
    except Exception:
        time.sleep(0.5)
else:
    print("❌ アダプタが起動しなかった")
    sys.exit(1)

# ROS の latched や 1Hz の状態が揃うのを待つ
time.sleep(4.0)

print("### 1. 状態が UI の契約どおりか")
_, raw = get("/api/state")
s = json.loads(raw)
print("   " + json.dumps(s, ensure_ascii=False)[:300])
check("鍵がそろっている",
      set(s) >= {"connected", "pose", "goal", "plan", "route",
                 "bridge_state", "patrol", "message"},
      f"実際: {sorted(s)}")
check("bridge_status が届いている", s["connected"] and s["bridge_state"] != "DISCONNECTED",
      f"bridge_state={s['bridge_state']}")
check("TF から現在地が取れている", s["pose"] is not None,
      f"pose={s['pose']} message={s['message']!r}")
check("巡回路が届いている（latched の QoS が合っている）", len(s["route"]) == 4,
      f"{len(s['route'])} 点")
check("巡回の総数が取れている", s["patrol"]["total"] == 4, f"{s['patrol']}")
# ⚠️ 現在地が地図の外にあると UI の表示範囲の計算が壊れる（一度踏んだ）
px, py = s["pose"]["x"], s["pose"]["y"]
check("現在地が地図の範囲内にある", -5.2 <= px <= -3.2 and -19.6 <= py <= -16.6,
      f"pose=({px:.2f}, {py:.2f}) 地図は x -5.2..-3.2 / y -19.6..-16.6")

print("\n### 2. 地図（OccupancyGrid -> PNG）")
st, raw = get("/api/map.json")
info = json.loads(raw)
print("   " + json.dumps(info, ensure_ascii=False))
check("地図のメタ情報が来る", st == 200 and info.get("width") == 40 and info.get("height") == 60)
check("resolution と origin が一致（float32 の丸め誤差が出ていない）",
      info.get("resolution") == 0.05
      and [round(v, 2) for v in info.get("origin", [])] == [-5.2, -19.6],
      f"resolution={info.get('resolution')!r}")
st, png = get("/api/map.png")
check("PNG が返る", st == 200 and png.startswith(b"\x89PNG"), f"{len(png)} bytes")

print("\n### 3. 上下の向き（ここを間違えると地図が裏返る）")
# fake_ros は下端寄り（row 3、原点側）にだけ目印の壁を置いている。
# PNG は 0 行目が上端なので、目印は**下半分**に出るはず
import zlib, struct   # noqa: E402  PNG を自前で読む（PIL が無い）
def png_rows(data: bytes):
    pos, idat, w, h = 8, b"", 0, 0
    while pos < len(data):
        ln = struct.unpack(">I", data[pos:pos + 4])[0]
        typ = data[pos + 4:pos + 8]
        body = data[pos + 8:pos + 8 + ln]
        if typ == b"IHDR":
            w, h = struct.unpack(">II", body[:8])
        elif typ == b"IDAT":
            idat += body
        pos += 12 + ln
    raw = zlib.decompress(idat)
    stride = w * 3
    out, prev = [], bytearray(stride)
    p = 0
    for _ in range(h):
        f = raw[p]; p += 1
        line = bytearray(raw[p:p + stride]); p += stride
        for i in range(stride):
            a = line[i - 3] if i >= 3 else 0
            b = prev[i]
            c = prev[i - 3] if i >= 3 else 0
            if f == 1: line[i] = (line[i] + a) & 255
            elif f == 2: line[i] = (line[i] + b) & 255
            elif f == 3: line[i] = (line[i] + (a + b) // 2) & 255
            elif f == 4:
                pa, pb, pc = abs(b - c), abs(a - c), abs(a + b - 2 * c)
                pr = a if (pa <= pb and pa <= pc) else (b if pb <= pc else c)
                line[i] = (line[i] + pr) & 255
        out.append(bytes(line)); prev = line
    return w, h, out

w, h, rows = png_rows(png)
def dark_count(row: bytes) -> int:
    return sum(1 for i in range(0, len(row), 3) if row[i] < 0x40)
top_half = sum(dark_count(rows[y]) for y in range(h // 2))
bottom_half = sum(dark_count(rows[y]) for y in range(h // 2, h))
check("目印(原点側の壁)が PNG の下半分にある", bottom_half > top_half,
      f"上半分 {top_half} px / 下半分 {bottom_half} px")

print("\n### 4. 操作（断り文がそのまま通るか）")
r = post("/api/command/patrol_start")
print(f"   READY 前の patrol_start -> {r}")
check("NAVIGATING でないと断られる", not r["ok"] and "bridge" in r["message"])
check("断り文が言い換えられずに届く", "enable_navigation" in r["message"])

r = post("/api/command/enable_navigation")
check("走行許可が通る", r["ok"], r["message"])
r = post("/api/command/patrol_start")
check("巡回開始が通る", r["ok"], r["message"])
time.sleep(2.0)
s = json.loads(get("/api/state")[1])
check("巡回状態が RUNNING になる", s["patrol"]["state"] == "RUNNING", f"{s['patrol']}")
check("経路(/plan)が届く", len(s["plan"]) >= 2, f"{len(s['plan'])} 点")
check("目的地が取れる（/plan の終点から。/goal_pose には流れない）",
      s["goal"] is not None, f"{s['goal']}")

r = post("/api/command/patrol_pause")
time.sleep(1.5)
s = json.loads(get("/api/state")[1])
check("一時停止で HOLD になる（PAUSED ではない）", s["patrol"]["state"] == "HOLD",
      f"{s['patrol']['state']}")

r = post("/api/command/launch_missile")
check("知らない操作は断られる", not r["ok"], r["message"])

print("\n### 5. 現在地が実際に動くか")
a = json.loads(get("/api/state")[1])
post("/api/command/patrol_start")
time.sleep(3.0)
b = json.loads(get("/api/state")[1])
moved = ((b["pose"]["x"] - a["pose"]["x"]) ** 2 + (b["pose"]["y"] - a["pose"]["y"]) ** 2) ** 0.5
check("TF を追えている", moved > 0.05, f"3秒で {moved:.3f} m 動いた")

print()
print("すべて通った" if ok_all else "❌ 失敗あり")
sys.exit(0 if ok_all else 1)
