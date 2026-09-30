"""**本物の Nav2 スタック**に対してアダプタを確かめる。

`mock/check.py` との違い: 相手が偽の ROS ではなく、実際の
`ros2 launch g1_navigation navigation.launch.py backend:=mock` で上がった
Nav2 / g1_cmd_router / g1_state_bridge / patrol_node。**偽物は SDK だけ。**

⚠️ **これでも実機の保証にはならない。** 物理・実 LiDAR・実際の歩容が無い。
出せるのは「本物のトピック・サービス・QoS・状態機械に対して配線が合っている」まで。
"""
from __future__ import annotations

import json
import sys
import time
import urllib.error
import urllib.request

B = sys.argv[1] if len(sys.argv) > 1 else "http://127.0.0.1:8098"
ok_all = True

# 期待値は Navigation トラックの設定そのもの
MAP_ORIGIN = [-6.0, -6.0]          # maps/synthetic_room.yaml
WAYPOINTS = [(-3.0, 3.0), (3.0, 3.0), (3.0, -3.5), (-3.0, -3.5)]   # patrol_synthetic.yaml


def check(label: str, cond: bool, detail: str = "") -> None:
    global ok_all
    ok_all = ok_all and cond
    print(f"  {'✅' if cond else '❌'} {label}{(' — ' + detail) if detail else ''}", flush=True)


def get(path: str):
    with urllib.request.urlopen(B + path, timeout=5) as r:
        return r.status, r.read()


def state() -> dict:
    return json.loads(get("/api/state")[1])


def post(path: str) -> dict:
    req = urllib.request.Request(B + path, method="POST", data=b"")
    try:
        with urllib.request.urlopen(req, timeout=20) as r:
            return json.loads(r.read())
    except urllib.error.HTTPError as e:
        return json.loads(e.read())


def wait_for(cond, secs: float, label: str):
    """`cond(state)` が真になるまで待つ。何を待っていたかを必ず出す。"""
    end = time.time() + secs
    s = {}
    while time.time() < end:
        try:
            s = state()
            if cond(s):
                return s
        except Exception:
            pass
        time.sleep(0.5)
    print(f"     （{label} を {secs:.0f} 秒待ったが変わらず: "
          f"bridge={s.get('bridge_state')} patrol={s.get('patrol')}）", flush=True)
    return s


for _ in range(180):
    try:
        get("/healthz")
        break
    except Exception:
        time.sleep(0.5)
else:
    print("❌ アダプタが起動しなかった")
    sys.exit(1)

print("### 1. 本物の g1_cmd_router / patrol_node が見えているか", flush=True)
s = wait_for(lambda s: s["connected"], 60, "bridge_status の到着")
print("   " + json.dumps({k: s.get(k) for k in
                          ("connected", "bridge_state", "patrol", "pose")},
                         ensure_ascii=False), flush=True)
check("本物の /g1/bridge_status が届いている",
      s.get("connected") and s.get("bridge_state") not in (None, "", "DISCONNECTED"),
      f"bridge_state={s.get('bridge_state')}")
check("本物の巡回路(patrol_synthetic)が届いている", len(s.get("route", [])) == 4,
      f"{len(s.get('route', []))} 点")
if len(s.get("route", [])) == 4:
    got = [(round(x, 1), round(y, 1)) for x, y in s["route"]]
    check("巡回路の座標が設定ファイルと一致", got == WAYPOINTS, f"{got}")
check("巡回の総数が取れている", s["patrol"]["total"] == 4, f"{s['patrol']}")

print("\n### 2. 本物の map_server の地図", flush=True)
st, raw = get("/api/map.json")
info = json.loads(raw) if st == 200 else {}
print("   " + json.dumps(info, ensure_ascii=False), flush=True)
check("地図が届いている", st == 200 and info.get("width", 0) > 0,
      f"HTTP {st}")
check("origin が synthetic_room.yaml と一致",
      [round(v, 2) for v in info.get("origin", [])] == MAP_ORIGIN, f"{info.get('origin')}")
check("resolution が 0.05（float32 の誤差が出ていない）", info.get("resolution") == 0.05,
      f"{info.get('resolution')!r}")
st, png = get("/api/map.png")
check("PNG が返る", st == 200 and png.startswith(b"\x89PNG"), f"{len(png)} bytes")

print("\n### 3. 本物の TF 連鎖から現在地が取れるか", flush=True)
s = wait_for(lambda s: s.get("pose") is not None, 40, "TF map->base_link")
check("map->base_link が解けている", s.get("pose") is not None,
      f"pose={s.get('pose')} message={s.get('message')!r}")

print("\n### 4. 本物のサービスに対する操作", flush=True)
r = post("/api/command/patrol_start")
print(f"   許可前の patrol_start -> {r}", flush=True)
check("走行許可の前は断られる", not r["ok"], r["message"])
check("本物の patrol_node の断り文がそのまま届く",
      "enable_navigation" in r["message"] or "走行できる状態ではない" in r["message"],
      r["message"])

s = wait_for(lambda s: s["bridge_state"] in ("READY", "NAVIGATING"), 60,
             "bridge が READY になるの")
check("bridge が READY まで上がった", s["bridge_state"] in ("READY", "NAVIGATING"),
      f"bridge_state={s['bridge_state']}")

r = post("/api/command/enable_navigation")
check("enable_navigation（本物の g1_cmd_router）が通る", r["ok"], r["message"])
s = wait_for(lambda s: s["bridge_state"] == "NAVIGATING", 20, "NAVIGATING になるの")
check("bridge が NAVIGATING になった", s["bridge_state"] == "NAVIGATING",
      f"bridge_state={s['bridge_state']}")

r = post("/api/command/patrol_start")
check("patrol_start（本物の patrol_node）が通る", r["ok"], r["message"])
s = wait_for(lambda s: s["patrol"]["state"] == "RUNNING", 20, "RUNNING になるの")
check("巡回が RUNNING になった", s["patrol"]["state"] == "RUNNING", f"{s['patrol']}")

print("\n### 5. 本物の Nav2 が立てた経路と目的地", flush=True)
s = wait_for(lambda s: len(s.get("plan", [])) > 2, 60, "Nav2 の /plan")
check("Nav2 の経路(/plan)が届く", len(s.get("plan", [])) > 2,
      f"{len(s.get('plan', []))} 点")
check("目的地が取れる（/plan の終点から。/goal_pose には流れない）",
      s.get("goal") is not None, f"{s.get('goal')}")

print("\n### 6. 一時停止", flush=True)
r = post("/api/command/patrol_pause")
check("patrol_pause が通る", r["ok"], r["message"])
s = wait_for(lambda s: s["patrol"]["state"] == "HOLD", 20, "HOLD になるの")
check("一時停止で HOLD になる（本物も PAUSED ではない）",
      s["patrol"]["state"] == "HOLD", f"{s['patrol']['state']}")

# ⚠️ Nav2 は Goal を畳んでも空の /plan を出し直さない。賞味期限を付けていないと
#    **止まっているのに経路と目的地が出たまま**になる（2026-09-20 に実際そうだった）
print("     経路が消えるのを待つ（賞味期限）...", flush=True)
s = wait_for(lambda s: not s.get("plan"), 15, "経路が消えるの")
check("止まったら経路が消える", not s.get("plan"), f"{len(s.get('plan', []))} 点")
check("止まったら目的地も消える", s.get("goal") is None, f"{s.get('goal')}")

print()
print("すべて通った" if ok_all else "❌ 失敗あり")
sys.exit(0 if ok_all else 1)
