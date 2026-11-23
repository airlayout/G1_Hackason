# Jetson (Python 3.8) 上で動く運動専用ヘルパー。g1console/motion.py の HelperSender が子プロセスとして起動する。
# 許す操作は move / stop / task だけ。モード切替や姿勢変更の指令は持たない（モード切替は remote_helper 側）。
# stdin に JSON 1 行を受け、stdout に JSON 1 行を返す。EOF になったら停止指令を出して終わる。
# __LIMITS__ と __TASKS__ は HelperSender が置換する（ハブ側と同じ上限で二重にクランプする）。
import json
import math
import sys

# SDK は失敗時に stdout へ print する。応答が壊れないよう、応答専用に元の stdout を確保する。
_reply = sys.stdout
sys.stdout = sys.stderr

from unitree_sdk2py.core.channel import ChannelFactoryInitialize
from unitree_sdk2py.g1.loco.g1_loco_client import LocoClient

LIMITS = __LIMITS__
TASKS = set(__TASKS__)
MAX_DURATION_S = 0.5  # ハブが送る値は 0.2。これより長い指令は受けない（暴走時に歩き続けないように）
RPC_TIMEOUT_S = 0.8


def reply(obj):
    print(json.dumps(obj), file=_reply, flush=True)


def code_of(result):
    """SDK の戻りは版により code 単体または (code, data)。"""
    return result[0] if isinstance(result, tuple) else result


def clamp(key, value):
    lo, hi = LIMITS[key]
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError(key)
    return min(max(float(value), lo), hi)


ChannelFactoryInitialize(0, "eth0")
loco = LocoClient()
loco.SetTimeout(RPC_TIMEOUT_S)
loco.Init()
reply({"ready": True})


def handle(req):
    op = req.get("op")
    if op == "move":
        duration = min(max(float(req["duration"]), 0.05), MAX_DURATION_S)
        return {"ok": True, "code": code_of(loco.SetVelocity(
            clamp("vx", req["vx"]), clamp("vy", req["vy"]), clamp("wz", req["wz"]), duration))}
    if op == "stop":
        return {"ok": True, "code": code_of(loco.StopMove())}
    if op == "task":
        if req.get("id") not in TASKS:
            raise ValueError("task")
        return {"ok": True, "code": code_of(loco.SetTaskId(int(req["id"])))}
    raise ValueError("op")


for line in sys.stdin:
    try:
        reply(handle(json.loads(line)))
    except Exception as exc:  # 1 件の不正で止まらない。理由だけ返す
        reply({"ok": False, "error": "%s: %s" % (type(exc).__name__, exc)})

try:  # 親が消えた（EOF）。最後に必ず止める
    loco.StopMove()
except Exception:
    pass
