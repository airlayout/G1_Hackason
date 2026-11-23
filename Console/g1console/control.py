"""「腕の制御が有効か」「脚は自律か」を、観測したモードとテレメトリから決める（読み取りだけ・G1 には触れない）。

腕: rt/lowstate の motor_state[i].mode（1=有効 / 0=ゼロトルク）。指令側 rt/lowcmd の mode ではない
    （lowcmd は最後に流れた指令が残るだけで、今の有効状態を表さない）。
脚: 標準モーションサービス（モーションコントローラ）が稼働して脚を制御しているか。
    Zero Torque・Damping はサービスが動いていても自律ではない。デバッグモードは自律ではない。

label は画面に大きく出す短い判定、detail はその理由（小さく添える）。
"""

from g1console.modes import FSM_LABELS

LEG_JOINTS = range(0, 12)
ARM_JOINTS = range(15, 29)
NOT_AUTONOMOUS_FSM = frozenset((0, 1))  # Zero Torque・Damping
JOINTS_STALE_S = 3.0  # これより古い lowstate は判定に使わない
CMD_LIVE_S = 1.0  # この秒数以内に lowcmd が来ていれば「指令が流れている」


def _modes(joints, indexes):
    """指定関節の motor_state.mode のリスト。取れなければ None。"""
    if not joints:
        return None
    age = joints.get("age_s")
    if age is None or age > JOINTS_STALE_S:
        return None
    items = joints.get("items") or []
    if len(items) <= max(indexes):
        return None
    values = [items[i].get("mode") for i in indexes]
    return None if any(v is None for v in values) else values


def _arm(telemetry):
    total = len(ARM_JOINTS)
    modes = _modes((telemetry or {}).get("joints"), ARM_JOINTS)
    if modes is None:
        return {"state": "unknown", "label": "不明", "detail": "関節の状態を受信できていません",
                "enabled_count": None, "total": total, "commanded": None}
    enabled = sum(1 for m in modes if m == 1)
    cmd_age = ((telemetry or {}).get("ages") or {}).get("rt/lowcmd")
    commanded = cmd_age is not None and cmd_age <= CMD_LIVE_S
    if enabled == total:
        state, label = "enabled", "有効"
        detail = "モータ %d/%d · %s" % (enabled, total, "指令を受信中" if commanded else "指令なし")
    elif enabled == 0:
        state, label = "disabled", "無効"
        detail = "Zero Torque。リモコンで Damping に入れると有効になります"
    else:
        state, label, detail = "partial", "一部のみ", "モータ %d/%d が有効" % (enabled, total)
    return {"state": state, "label": label, "detail": detail,
            "enabled_count": enabled, "total": total, "commanded": commanded}


def _legs(mode, telemetry):
    modes = _modes((telemetry or {}).get("joints"), LEG_JOINTS)
    enabled = None if modes is None else sum(1 for m in modes if m == 1)
    state = (mode or {}).get("state")
    fsm_id = (mode or {}).get("fsm_id")
    if state == "ok" and fsm_id is not None:
        name = FSM_LABELS.get(fsm_id, "不明")
        if fsm_id in NOT_AUTONOMOUS_FSM:
            result, label, detail = "not_autonomous", "自律ではない", "%s · FSM %s" % (name, fsm_id)
        else:
            result, label, detail = "autonomous", "自律", "モーションコントローラが制御中 · %s · FSM %s" % (name, fsm_id)
    elif state == "debug":
        result, label, detail = "not_autonomous", "自律ではない", "デバッグモード（モーションコントローラ停止）"
    else:
        result, label, detail = "unknown", "不明", "モードを取得できていません"
    return {"state": result, "label": label, "detail": detail, "fsm_id": fsm_id if state == "ok" else None,
            "enabled_count": enabled, "total": len(LEG_JOINTS)}


def control_summary(mode, telemetry):
    """{"arm": ..., "legs": ...}。モードかテレメトリが無ければ各項目は state="unknown"。"""
    return {"arm": _arm(telemetry), "legs": _legs(mode, telemetry)}
