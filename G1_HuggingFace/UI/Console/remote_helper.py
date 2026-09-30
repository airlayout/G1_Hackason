# Jetson (Python 3.8) 上で動くヘルパー。server.py が ssh 経由で起動する。
# stdin に JSON 1行 {"op":"status"} / {"op":"set","id":N} を受け、stdout に JSON 1行を返す。
# __ALLOWED__ は server.py が許可 ID のリストに置換する。
import json
import sys

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.comm.motion_switcher.motion_switcher_client import MotionSwitcherClient
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

ALLOWED = set(__ALLOWED__)

ChannelFactoryInitialize(0, "eth0")
msc = MotionSwitcherClient()
msc.SetTimeout(2.0)
msc.Init()
loco = LocoClient()
loco.SetTimeout(2.0)
loco.Init()
print(json.dumps({"ready": True}), flush=True)


def status():
    code, mode = msc.CheckMode()
    name = (mode or {}).get("name", "") if code == 0 else None
    fsm_code, fsm_id = loco.GetFsmId()
    return {"ok": True, "checkmode_code": code, "service": name,
            "fsm_code": fsm_code, "fsm_id": fsm_id if fsm_code == 0 else None}


for line in sys.stdin:
    try:
        req = json.loads(line)
        if req["op"] == "status":
            res = status()
        elif req["op"] == "set":
            fsm_id = int(req["id"])
            if fsm_id not in ALLOWED:
                res = {"ok": False, "error": "許可されていない FSM ID: %d" % fsm_id}
            else:
                res = {"ok": True, "set_code": loco.SetFsmId(fsm_id)}
        else:
            res = {"ok": False, "error": "unknown op"}
    except Exception as exc:  # 1リクエストの失敗でヘルパー全体を落とさない
        res = {"ok": False, "error": "%s: %s" % (type(exc).__name__, exc)}
    print(json.dumps(res), flush=True)
