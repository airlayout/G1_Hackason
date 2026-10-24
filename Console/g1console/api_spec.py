"""開発コンソール API の定義（唯一の出典）。OpenAPI 3.0.3 を組み立て、次の 3 つを同じ内容から出す。

  GET /api            OpenAPI（JSON）        GET /openapi.yaml  OpenAPI（YAML）
  docs/openapi.yaml   同上（ファイル）        docs/API.md        人と AI が読む用の要約

`python3 Console/g1console/api_spec.py` で docs/ を再生成する。tests が「スキーマ = 実際のレスポンス」を検証するので、
API を変えたらここを直す（食い違うとテストが落ちる）。手書きの YAML ではなく Python で組む理由は、
標準ライブラリだけで、スキーマの部品を共有し、テストから同じ定義を使えるから。
"""
import json

try:
    from g1console.modes import ACTIONS
except ImportError:  # `python3 Console/g1console/api_spec.py` で直接動かしたとき
    from modes import ACTIONS

ERRORS = "HTTP 400=入力不正 / 404=未定義 / 500=保存失敗 / 502=G1 または Jetson に届かない・G1 が拒否。本文はすべて Error。"

# ---- スキーマの部品 -------------------------------------------------------------------------------------------
def S(desc=None, **kw): return {"type": "string", **({"description": desc} if desc else {}), **kw}
def I(desc=None, **kw): return {"type": "integer", **({"description": desc} if desc else {}), **kw}
def N(desc=None, **kw): return {"type": "number", **({"description": desc} if desc else {}), **kw}
def B(desc=None): return {"type": "boolean", **({"description": desc} if desc else {})}
def A(items, desc=None): return {"type": "array", "items": items, **({"description": desc} if desc else {})}
def null(schema): return {**schema, "nullable": True}
def ref(name): return {"$ref": "#/components/schemas/" + name}
def free(desc=None): return null({"type": "object", **({"description": desc} if desc else {}), "additionalProperties": True})
def obj(props, required=(), desc=None, extra=False):
    return {"type": "object", **({"description": desc} if desc else {}), "properties": props,
            **({"required": list(required)} if required else {}), "additionalProperties": extra}


HEAD = {"jetson": ref("Link"), "g1": ref("Link"), "mode": ref("Mode"), "control": ref("Control"),
        "sampled_at": null(N("最後に G1 を確認した時刻（epoch 秒）。null=まだ確認していない")),
        "age_s": null(N("最後の確認から今までの秒数。大きければ古い値"))}
HEAD_REQ = list(HEAD)

MOTION_PROPS = {
    "state": S(enum=["idle", "active"], desc="idle=操作者なし / active=操作者あり"), "operator": B("操作者がいるか"),
    "moving": B("ハブが速度指令を送っている最中か"), "target": obj({"vx": N("m/s"), "vy": N("m/s"), "wz": N("rad/s")}, ["vx", "vy", "wz"], "いまの目標速度（上限で丸め済み）"),
    "blocked": null(S("動かさない理由（歩行モードでない・状態が古い・Nav2 稼働中など）。null=動かせる")), "disable_reason": null(S("手動操作を禁止している理由（Nav2 稼働中など）")),
    "last_rx_age_s": null(N("操作者の最後の ctl からの秒数。0.5 を超えると自動停止")),
    "last_event": free("直近の出来事 {kind: claim/lost/estop/release/takeover/disabled/sender_error/shutdown, detail, at}。接続断で解除されたかはここで分かる"),
    "sender": obj({"backend": S("local / mock / unavailable"), "error": null(S("直近の送信失敗"))}, ["backend", "error"]),
    "limits": free("vx / vy / wz の [下限, 上限]。実機未検証の初期値"),
    "timing": obj({"send_period_s": N(), "duration_s": N("SetVelocity の有効時間"), "liveness_s": N("この秒数 ctl が途絶えたら停止")}),
}
MOTION_REQ = ["state", "operator", "moving", "target", "blocked", "sender"]

TASK_PROPS = {
    "name": S("タスク名（Console/tasks/<name>.json）"), "title": S(), "description": S(),
    "argv": A(S(), "実行されるコマンド（配列。シェルは経由しない）"), "group": null(S("排他グループ。同じグループは同時に走らせない")),
    "timeout_s": N("この秒数で SIGTERM"),
    "verified": B("false=実機（Jetson・G1）で未確認。画面は実行・停止の前に注意ポップアップを出す。結果を断定しない"),
    "caution": A(S(), "実行・停止の前に画面が見せる注意書き（server 側の定義）"),
    "state": S(enum=["idle", "running", "stopping"]),
    "result": null(S("直近の実行の結果。ok / failed / stopped / timeout")), "exit_code": null(I("終了コード。負はシグナル")),
    "started_at": null(N("epoch 秒")), "ended_at": null(N("epoch 秒。null=実行中または未実行")),
}
TASK_REQ = ["name", "title", "description", "argv", "group", "timeout_s", "verified", "caution", "state", "result", "exit_code", "started_at", "ended_at"]

SCHEMAS = {
    "Error": obj({"error": S("平易な日本語の理由")}, ["error"], "エラー本文（全エンドポイント共通）。G1・Jetson に届かない 502 では ok=false・offline・ssh_exit などの生の情報も付く", extra=True),
    "Link": obj({"state": S(enum=["ok", "down", "unknown", "checking"], desc="ok=届いている / down=届かない / unknown=上流が切れていて見えていない"),
                 "detail": S("理由（画面の診断文と同じ）")}, ["state", "detail"], "接続 1 段の状態。jetson が down なら g1 は unknown"),
    "Mode": obj({"state": S(enum=["ok", "debug", "unknown", "none"], desc="ok=通常モードで FSM ID が取れた / debug=デバッグモード / unknown=取得不可 / none=接続なし"),
                 "label": S("表示名"), "fsm_id": I("FSM ID（state=ok のときだけ）"), "service": S("モーションサービス名"), "detail": S()},
                ["state", "label"], "現在のモード"),
    "Control": obj({"arm": obj({"state": S(enum=["enabled", "disabled", "partial", "unknown"],
                                           desc="enabled=腕 14 軸すべてモータ有効 / disabled=すべて無効（ゼロトルク） / partial=一部のみ / unknown=lowstate が古いか mode が無い"),
                                "label": S("画面に大きく出す短い判定（有効 / 無効 / 一部のみ / 不明）"), "detail": S("判定の理由（画面に小さく添える）"),
                                "enabled_count": null(I("rt/lowstate の motor_state.mode が 1 の腕関節数")),
                                "total": I("腕関節の数（14）"), "commanded": null(B("rt/lowcmd が 1 秒以内に来ているか"))},
                               ["state", "label", "detail", "total"], "腕の制御が有効か。指令側 lowcmd の mode ではなく状態側 lowstate の mode で判定"),
                    "legs": obj({"state": S(enum=["autonomous", "not_autonomous", "unknown"],
                                            desc="autonomous=モーションコントローラが脚を制御中 / not_autonomous=ゼロトルク・ダンピング・デバッグモード"),
                                 "label": S("画面に大きく出す短い判定（自律 / 自律ではない / 不明）"), "detail": S("判定の理由（画面に小さく添える）"),
                                 "fsm_id": null(I("state=ok のモードのときだけ")),
                                 "enabled_count": null(I("脚関節 12 軸のうち motor_state.mode が 1 の数")), "total": I("脚関節の数（12）")},
                                ["state", "label", "detail", "total"], "脚が自律（標準モーションサービスの制御下）か")},
                   ["arm", "legs"], "腕の制御が有効か・脚は自律か（読み取りだけで算出）"),
    "Status": obj({**HEAD, "latency_ms": null(I("G1 への往復 ms")), "last_g1_ok_at": null(N()),
                   "last_mode": free("最後に確認できたモード（切断中の参考。現在値ではない）"),
                   "paused": B("true=自動確認・再接続を停止中（値は更新されない）"), "poll_interval_s": N(),
                   "server": obj({"backend": S("接続先の表示"), "mock": B()}, ["backend", "mock"]),
                   "telemetry": free("ヘルパーの生テレメトリ（形は /api/state・/api/joints が整えたもの）")},
                  HEAD_REQ + ["paused", "server"], "接続・モード・鮮度。まず最初に読む"),
    "State": obj({**HEAD, "battery": free("soc[%]・voltage・current・temperature（単位は推定。notes 参照）"), "imu": free("rpy[rad]・gyro・accel・temperature"),
                  "imu2": free(), "odom": free("起動位置からの推定。ドリフトする"), "mainboard": free("value の意味は未確認（生の値）"),
                  "system": free("Jetson の温度・負荷など"), "remote": free("リモコン入力。受信なし=null"), "estop": free("非常停止。受信なし=null"),
                  "strings": free(), "ages": free("トピックごとの最終受信からの秒数"),
                  "notes": obj({"units_assumed": A(S()), "unverified": A(S())}, desc="単位が推定・意味が未確認の項目")},
                 HEAD_REQ + ["notes"], "状態タブ。値が null のものは未受信または接続なし"),
    "Joints": obj({**HEAD, "mode_machine": null(I()), "age_s_joints": null(N()),
                   "items": A(obj({"index": I(), "name": S("SDK の並び順による名前（実機で未確認）"), "q": N("角度 rad"), "dq": N("速度 rad/s"),
                                   "tau": N("トルク推定"), "temperature": A(N()), "mode": I("rt/lowstate の motor_state.mode（1=有効 / 0=ゼロトルク）。古いヘルパーでは無い"),
                                   "cmd": null(free("lowcmd の指令値。最後に流れた指令が残るだけで、今の有効状態ではない"))},
                                  ["index", "name"], extra=True), "関節 29 軸（未接続なら空）"),
                   "notes": obj({"unverified": A(S())})}, HEAD_REQ + ["items"], "関節タブ"),
    "Snapshot": obj({**HEAD, "tabs": A(S(), "タブ名の一覧（= URL ハッシュ = API 名）"), "state": ref("State"), "joints": ref("Joints")},
                    HEAD_REQ + ["tabs", "state", "joints"], "state と joints をまとめたもの"),
    "Volume": obj({"ok": B(), "code": I("0=G1 が受理"), "volume": I("0〜100", minimum=0, maximum=100)}, ["ok", "code", "volume"], extra=True),
    "VolumeSet": obj({"ok": B(), "set_code": I(), "read_code": I(), "volume_after": I("読み戻した値。指定値と一致すれば反映済み")},
                     ["ok", "set_code", "read_code", "volume_after"], extra=True),
    "Ack": obj({"ok": B(), "code": I("0=G1 が受理（効果は目視・聴取で確認）")}, ["ok", "code"], extra=True),
    "ModeAck": obj({"ok": B(), "set_code": I("0=G1 が受理。到達ではない")}, ["ok", "set_code"], extra=True),
    "Cameras": obj({"enabled": B("配信先（Jetson の camera_stream.py）が設定済みか"), "cameras": {"type": "object", "additionalProperties": S(), "description": "名前 → 表示名"}},
                   ["enabled", "cameras"]),
    "Dds": obj({"topics": A(obj({"topic": S(), "type": S(), "status": S(enum=["displayed", "todo", "excluded"], desc="表示済み / 未購読（未実装） / 対象外"), "note": S()},
                                ["topic", "type", "status", "note"])),
                "services": A(obj({"name": S(), "status": S(), "note": S()}, ["name", "status", "note"]))}, ["topics", "services"]),
    "Buttons": obj({"buttons": A(obj({"id": I("POST /api/mode の id"), "label": S(), "danger": B()}, ["id", "label", "danger"])),
                    "transitions": free("現在の FSM ID（文字列）→ 画面で押せるボタンの id。公式の状態遷移図に沿う"),
                    "fallback": A(I(), "transitions に無い現在値のとき押せるボタンの id"),
                    "speed": A(obj({"value": I("POST /api/speed の value"), "label": S()}, ["value", "label"])),
                    "speed_fsm_ids": A(I(), "速度モードを送れる現在の FSM ID（歩行中のみ）")},
                   ["buttons", "transitions", "fallback", "speed", "speed_fsm_ids"]),
    "SpeedPatch": obj({"value": I("GET /api/buttons の speed に載っている value のみ")}, ["value"]),
    "Features": obj({"actions": A(obj({"id": S(), "label": S(), "task_id": I()}, ["id", "label", "task_id"]), "実行できるアクション"),
                     "features": A(obj({"id": S(), "label": S(), "implemented": B(), "verified": B("false=実機で未確認。結果を断定しない"), "note": S()},
                                       ["id", "label", "implemented", "verified"])),
                     "planned_modes": A({"type": "object", "additionalProperties": True}), "planned_actions": A({"type": "object", "additionalProperties": True})},
                    ["features"]),
    "SettingsValues": obj({"dev_pc": S("開発用 PC の IP（表示・記録のみ）"), "jetson_host": S("Jetson の IP かホスト名（ssh とカメラの接続先）"),
                           "jetson_user": S("空なら ~/.ssh/config に任せる"), "jetson_key": S("ssh 秘密鍵のパス。空なら既定"),
                           "g1_ip": S("G1 本体の IP（表示・記録のみ）"), "camera_port": I(minimum=1, maximum=65535),
                           "lidar_port": I(minimum=1, maximum=65535)},
                          ["dev_pc", "jetson_host", "jetson_user", "jetson_key", "g1_ip", "camera_port", "lidar_port"]),
    "SettingsPatch": obj({"dev_pc": S("IP アドレスかホスト名。空文字で未設定"), "jetson_host": S(), "jetson_user": S(), "jetson_key": S(),
                          "g1_ip": S(), "camera_port": I(minimum=1, maximum=65535), "lidar_port": I(minimum=1, maximum=65535)}, desc="指定した項目だけ変わる（全て任意）"),
    "Settings": obj({"settings": ref("SettingsValues"), "labels": {"type": "object", "additionalProperties": S()}}, ["settings", "labels"]),
    "MonitorPatch": obj({"paused": B("true=停止 / false=再開"), "check": B("true=1 回だけ確認")}, desc="どちらか、または両方"),
    "Monitor": obj({"paused": B()}, ["paused"]),
    "HubStopPatch": obj({"confirm": B("true のときだけ停止する。無ければ 400")}, ["confirm"]),
    "HubStopAck": obj({"ok": B(), "message": S()}, ["ok", "message"]),
    "Scenarios": obj({"scenarios": {"type": "object", "additionalProperties": S()}, "current": S()}, ["scenarios", "current"]),
    "ScenarioPatch": obj({"scenario": S()}, ["scenario"]),
    "Scenario": obj({"scenario": S()}, ["scenario"]),
    "MotionStatus": obj(MOTION_PROPS, MOTION_REQ, "運動操作の状態", extra=True),
    "MotionClaim": obj({**MOTION_PROPS, "session": S("以後の ctl / release / action に付ける操作権の ID")}, MOTION_REQ + ["session"], "操作権を取った結果", extra=True),
    "MotionClaimPatch": obj({"takeover": B("true=他の人の操作権を奪う（相手は停止する）")}, desc="全て任意"),
    "MotionCtlPatch": obj({"session": S(), "seq": I("増える番号。小さい・同じものは捨てる"), "vx": N("前後 m/s"), "vy": N("左右 m/s"), "wz": N("旋回 rad/s")},
                          ["session", "seq", "vx", "vy", "wz"], "上限を超えた値はサーバーが丸める"),
    "MotionSessionPatch": obj({"session": S()}, ["session"]),
    "MotionActionPatch": obj({"session": S(), "name": S("GET /api/features の actions の id", enum=[a["id"] for a in ACTIONS])}, ["session", "name"]),
    "Task": obj(TASK_PROPS, TASK_REQ, "タスクの定義と直近の実行状態"),
    "Tasks": obj({"tasks": A(ref("Task"))}, ["tasks"]),
    "TaskLogs": obj({"name": S(), "next": I("最後の行の seq。次回の since に使う"), "truncated": B("since 以降の一部がリングバッファから消えた"),
                     "lines": A(obj({"seq": I(), "text": S()}, ["seq", "text"]))}, ["name", "next", "truncated", "lines"]),
    "TaskNamePatch": obj({"name": S("GET /api/tasks の name。これ以外（argv・パス）は受け付けない")}, ["name"]),
    "Pose": obj({"x": N("map フレームの x [m]"), "y": N("map フレームの y [m]"), "yaw_deg": N("向き [度]")}, ["x", "y", "yaw_deg"]),
    "NavState": obj({
        "available": B("false=アダプタから読めていない（reason に理由）"), "reason": S("読めない理由"),
        "connected": B("Nav2 の bridge_status が生きている＝Nav2 稼働中とみなす"), "pose": null(ref("Pose")), "goal": null(ref("Pose")),
        "plan": A(A(N(), "[x, y]"), "経路（/plan）"), "route": A(A(N(), "[x, y]"), "巡回ルート"),
        "bridge_state": S(enum=["DISCONNECTED", "STANDBY", "READY", "NAVIGATING", "FAULT", "E_STOP", "UNKNOWN"]),
        "patrol": obj({"state": S(), "index": I(), "total": I()}, ["state", "index", "total"]), "message": S(),
        "age_s": null(N("最後にアダプタを読んでからの秒数")), "exclusive_reason": null(S("手動操作を禁止している理由。null=禁止していない")),
        "heartbeat": obj({"enabled": B("--heartbeat-to が設定済みか"), "target": S(), "present": B("画面が見ている（生存信号を送っている）か"),
                          "sent": I("送った累計パケット数")}, ["enabled"]),
    }, ["available", "heartbeat"], "Nav2 の現在地・目的地・経路。値は検証済みのものだけ"),
    "NavMapInfo": obj({"resolution": N("m/pixel"), "origin": A(N(), "[x, y]。画像左下の map 座標 [m]"), "width": I("画素"), "height": I("画素")},
                      ["resolution", "origin", "width", "height"], "地図 PNG の座標系。x = origin[0] + px*resolution、y = origin[1] + (height - py)*resolution"),
    "Location": obj({"name": S("英数字・日本語・_ - の 1〜40 文字"), "x": N("m"), "y": N("m"), "yaw_deg": N("度（±180）"),
                     "map_id": S("地図の識別子（任意）")}, ["name", "x", "y", "yaw_deg", "map_id"], "名前つき座標"),
    "Locations": obj({"locations": A(ref("Location"))}, ["locations"]),
    "LocationPatch": obj({"name": S(), "x": N("|x| ≤ 10000"), "y": N("|y| ≤ 10000"), "yaw_deg": N("±180"), "map_id": S("省略可")},
                         ["name", "x", "y", "yaw_deg"], "同じ name があれば上書き"),
    "LocationNamePatch": obj({"name": S()}, ["name"]),
    "LocationDeleted": obj({"deleted": S()}, ["deleted"]),
    "PresencePatch": obj({}, desc="本文は空でよい"),
    "Topics": obj({"enabled": B(), "backend": S("local / mock"), "deny": A(S(), "常に送信を拒否するトピック（fnmatch）。許可リストより強い"),
                   "supported_types": A(S(), "購読で読める型"), "max_subscriptions": I(),
                   "publish": A(obj({"topic": S(), "type": S(), "max_bytes": I(), "min_interval_s": N(), "note": S()},
                                    ["topic", "type", "max_bytes", "min_interval_s", "note"]), "送信の許可リスト（サーバー側ファイルだけ。画面から増やせない）"),
                   "subscribed": A(S())}, ["enabled", "backend", "deny", "supported_types", "max_subscriptions", "publish", "subscribed"]),
    "TopicsReceived": obj({"items": A(obj({"topic": S(), "type": S(), "count": I("受信した件数"), "age_s": null(N("最後の受信からの秒数。null=まだ受信なし")),
                                           "message": free("最新メッセージ（配列は 16 要素、文字列は 512 文字まで）")}, ["topic", "type", "count", "age_s", "message"]))}, ["items"]),
    "TopicSubscribePatch": obj({"topic": S("rt/ で始まる名前"), "type": S("GET /api/topics の supported_types のどれか")}, ["topic", "type"]),
    "TopicNamePatch": obj({"topic": S()}, ["topic"]),
    "TopicPublishPatch": obj({"topic": S("許可リストにあるトピック"), "data": S("送る文字列（max_bytes まで）")}, ["topic", "data"]),
    "TopicSubscribed": obj({"topic": S(), "type": S()}, ["topic", "type"]),
    "TopicUnsubscribed": obj({"unsubscribed": S()}, ["unsubscribed"]),
    "TopicPublished": obj({"published": S(), "bytes": I()}, ["published", "bytes"]),
    "PresenceAck": obj({"ok": B(), "heartbeat": B("生存信号の送出が設定されているか")}, ["ok", "heartbeat"]),
    "ModePatch": obj({"id": I("GET /api/buttons に載っている id のみ")}, ["id"]),
    "VolumePatch": obj({"volume": I(minimum=0, maximum=100)}, ["volume"]),
    "LedPatch": obj({"r": I(minimum=0, maximum=255), "g": I(minimum=0, maximum=255), "b": I(minimum=0, maximum=255)}, ["r", "g", "b"]),
    "TtsPatch": obj({"text": S("読み上げる文", minLength=1, maxLength=100), "speaker_id": I("話者 ID（意味は未確認）", enum=[0, 1])}, ["text", "speaker_id"]),
}

TAGS = {"ops": "操作タブ（接続・モード）", "motion": "運動タブ（物理リモコンの代替）", "tasks": "タスクタブ（定義済みコマンドの実行）", "nav": "マップタブ（Nav2 の表示・地点）", "state": "状態タブ", "joints": "関節タブ", "audio": "音声タブ",
        "camera": "カメラタブ", "dds": "DDS タブ", "settings": "設定タブ", "mock": "--mock のときだけ有効"}

READ, SETTINGS, WRITE, MOTION = "read", "settings", "robot_write", "robot_motion"
E400, E502 = "入力不正", "G1 または Jetson に届かない・G1 が拒否"


def ep(method, path, op, tag, summary, effect, response, *, notes=None, request=None, example=None, errors=(),
       unverified=False, mock_only=False, ok_status="成功", content=None):
    return {"method": method, "path": path, "operationId": op, "tag": tag, "summary": summary, "effect": effect,
            "response": response, "notes": notes, "request": request, "example": example, "errors": list(errors),
            "unverified": unverified, "mock_only": mock_only, "ok_status": ok_status, "content": content}


ENDPOINTS = [
    ep("GET", "/api", "getApi", "ops", "この API 定義（OpenAPI, JSON）", READ, None),
    ep("GET", "/api/status", "getStatus", "ops", "まず最初に読む。接続（jetson/g1）・現在のモード・確認の古さ・自動確認の停止中か", READ, "Status",
       notes="jetson/g1 が ok でなければ、その先の値は信用しない。ok でも age_s が数秒以上なら古い値。paused=true の間は更新されない。"),
    ep("GET", "/api/state", "getState", "state", "バッテリー・IMU・オドメトリ・メインボード・Jetson・リモコン・非常停止", READ, "State",
       notes="notes.units_assumed / notes.unverified に、単位が推定・意味が未確認の項目を列挙している。"),
    ep("GET", "/api/joints", "getJoints", "joints", "29 関節の角度・速度・トルク推定・温度・指令値", READ, "Joints"),
    ep("GET", "/api/snapshot", "getSnapshot", "ops", "state と joints を 1 回で取得（状況把握はこれ 1 本で足りる）", READ, "Snapshot"),
    ep("GET", "/api/audio/volume", "getVolume", "audio", "スピーカー音量を G1 から読む（Jetson 経由で実際に問い合わせる）", READ, "Volume", errors=[(502, E502)]),
    ep("GET", "/api/motion/status", "getMotionStatus", "motion", "運動操作の状態（操作者・目標速度・動かせない理由・直近の出来事）", READ, "MotionStatus",
       notes="last_event.kind が lost なら接続断で停止・解除された。blocked が null でなければ、その理由で動かさない。"),
    ep("GET", "/api/tasks", "getTasks", "tasks", "実行できるタスクの一覧と、それぞれの直近の実行状態", READ, "Tasks",
       notes="定義は Console/tasks/*.json だけ。クライアントから増やせない。"),
    ep("GET", "/api/tasks/logs", "getTaskLogs", "tasks", "タスクのログ（行単位）。クエリ name と since（最後に見た seq）", READ, "TaskLogs",
       errors=[(400, E400 + "（since）"), (404, "定義にないタスク")],
       notes="next を次回の since に渡すと、欠落・重複なく続きだけ読める。リングバッファは 2000 行で、溢れたら truncated=true。"),
    ep("GET", "/api/nav/state", "getNavState", "nav", "Nav2 の現在地・目的地・経路と、手動操作を禁止している理由", READ, "NavState",
       notes="読み取り専用（アダプタの GET だけ）。Nav2 へ送る口は無い。available=false は『アダプタが読めない』で、Nav2 の停止を意味しない。"
             "exclusive_reason が null でなければ手動操作は無効。一度 Nav2 の稼働を見たら、アダプタが黙っても解除しない（安全側）。"),
    ep("GET", "/api/nav/map.json", "getNavMapInfo", "nav", "地図の解像度・原点・大きさ", READ, "NavMapInfo",
       errors=[(404, "--nav-adapter が未設定"), (502, "アダプタに繋がらない・応答が不正")]),
    ep("GET", "/api/nav/map.png", "getNavMapPng", "nav", "地図の PNG（アダプタの画像を検証して中継）", READ, None, content="image/png",
       ok_status="PNG 画像（バイナリ）", errors=[(404, "--nav-adapter が未設定"), (502, "アダプタに繋がらない・PNG でない")],
       notes="座標の対応は GET /api/nav/map.json。RViz との位置合わせは実機で未確認。"),
    ep("GET", "/api/locations", "getLocations", "nav", "登録済みの地点（名前つき座標）", READ, "Locations",
       notes="タスク定義の argv から {loc:NAME.x} / {loc:NAME.y} / {loc:NAME.yaw_deg} で参照できる。"),
    ep("GET", "/api/topics", "getTopics", "dds", "トピック通信の方針（常に拒否・送信の許可リスト・購読中）", READ, "Topics",
       notes="--local / --mock のときだけ。受信は自由（型が読めるものだけ）、送信は許可リストにある String_ だけ。運動・関節・腕・RPC は常に拒否。"),
    ep("GET", "/api/topics/received", "getTopicsReceived", "dds", "購読中トピックの最新メッセージと受信状況", READ, "TopicsReceived", errors=[(502, E502)],
       notes="購読が 0 件ならヘルパーを起動せず空を返す。age_s が null なら一度も受信していない（異常とは限らない）。"),
    ep("GET", "/api/cameras","getCameras", "camera", "カメラ一覧と、配信が設定済みか（映像は GET /camera/{name}）", READ, "Cameras"),
    ep("GET", "/api/camera/status", "getCameraStatus", "camera", "camera_stream の起動状態と、映像デバイスを掴んでいるプロセス（foreign＝自前以外）", READ, None,
       errors=[(502, E502)]),
    ep("GET", "/api/lidar/status", "getLidarStatus", "camera", "lidar_stream（Jetson の点群配信）の起動状態", READ, None, errors=[(502, E502)]),
    ep("GET", "/lidar", "getLidar", "camera", "lidar 点群の最新 1 フレーム（既定は間引き済み）。float32 little-endian の (x,y,z,intensity) の繰り返し。点数は X-Lidar-Points", READ, None,
       errors=[(404, "lidar 未設定（設定タブで Jetson の IP を入力）"), (502, "配信元に届かない。lidar_stream が起動しているか確認")],
       content="application/octet-stream", ok_status="点群バイナリ（16 バイト × 点数）。X-Lidar-Seq / X-Lidar-Points / X-Lidar-Age-Ms ヘッダ付き",
       notes="座標は livox_frame のまま（単位 m）。G1 の向きに対して傾いている可能性がある。"
             "クエリ voxel（ボクセル幅 m、0〜1、0 で間引かない・既定 0.1）と max（上限点数 100〜30000・既定 5000）で間引きの強さを選べる。範囲外は無視して既定値になる。"),
    ep("GET", "/api/dds", "getDds", "dds", "DDS トピックと RPC サービスの台帳（表示済み／未実装／対象外）", READ, "Dds"),
    ep("GET", "/api/buttons", "getButtons", "ops", "切り替えできるモード。POST /api/mode の id はここから選ぶ", READ, "Buttons"),
    ep("GET", "/api/features", "getFeatures", "dds", "機能ごとの implemented / verified（verified=false は実機で未確認）", READ, "Features"),
    ep("GET", "/api/settings", "getSettings", "settings", "接続先の設定（開発用 PC / Jetson / G1 の IP ほか）", READ, "Settings"),
    ep("GET", "/api/scenarios", "getScenarios", "mock", "模擬シナリオの一覧と現在値", READ, "Scenarios", mock_only=True),
    ep("GET", "/camera/{name}", "getCamera", "camera", "カメラ映像（MJPEG のストリーム）。name は GET /api/cameras のキー", READ, None,
       errors=[(404, "カメラ未設定または name が不正"), (502, "配信元に届かない")],
       content="multipart/x-mixed-replace", ok_status="MJPEG ストリーム（終わらない）"),
    ep("GET", "/openapi.yaml", "getOpenapiYaml", "ops", "この API 定義（OpenAPI, YAML）", READ, None, content="application/yaml"),

    ep("POST", "/api/settings", "postSettings", "settings", "接続先を保存して反映（Jetson を変えると ssh を張り直す）", SETTINGS, "Settings",
       request="SettingsPatch", example={"jetson_host": "192.168.123.164", "g1_ip": "192.168.123.161"}, errors=[(400, E400 + "（IP かホスト名以外、範囲外など。理由が error に入る）"), (500, "保存できない")]),
    ep("POST", "/api/nav/presence", "postNavPresence", "nav", "『人が画面を見ている』を知らせる（0.5 秒ごと）。見ている間だけ Nav2 へ生存信号を出す", SETTINGS, "PresenceAck",
       request="PresencePatch", example={}, errors=[(400, E400)], unverified=True,
       notes="--heartbeat-to が設定されているときだけ意味を持つ。0.6 秒途絶えると生存信号は止まる。受信側が loopback を受け付けるかは実機で未確認。"),
    ep("POST", "/api/locations/save", "postLocationSave", "nav", "地点を登録・上書きする", SETTINGS, "Location",
       request="LocationPatch", example={"name": "example-spot", "x": 1.0, "y": 2.0, "yaw_deg": 90.0, "map_id": ""},
       errors=[(400, E400), (409, "地点が上限（200）"), (500, "保存できない")]),
    ep("POST", "/api/locations/delete", "postLocationDelete", "nav", "地点を削除する", SETTINGS, "LocationDeleted",
       request="LocationNamePatch", example={"name": "example-spot"}, errors=[(400, E400), (404, "その名前の地点がない"), (500, "保存できない")],
       notes="タスクがその地点を参照していると、次の起動時に 409 になる。"),
    ep("POST", "/api/topics/subscribe", "postTopicSubscribe", "dds", "トピックを購読する（初回はヘルパーを起動する）", SETTINGS, "TopicSubscribed",
       request="TopicSubscribePatch", example={"topic": "rt/rtc/state", "type": "String_"},
       errors=[(400, E400), (409, "購読の上限"), (422, "型が読めない"), (502, E502)], unverified=True,
       notes="型は名前から推測しない。呼び出し側が GET /api/dds の type を見て、supported_types にあるものだけ指定する。"),
    ep("POST", "/api/topics/unsubscribe", "postTopicUnsubscribe", "dds", "購読をやめる", SETTINGS, "TopicUnsubscribed",
       request="TopicNamePatch", example={"topic": "rt/rtc/state"}, errors=[(400, E400), (404, "購読していない"), (502, E502)], unverified=True),
    ep("POST", "/api/topics/publish", "postTopicPublish", "dds", "許可リストにあるトピックへ String_ を送る。**G1 の DDS に出る**", WRITE, "TopicPublished",
       request="TopicPublishPatch", example={"topic": "rt/console_demo/say", "data": "hello"},
       errors=[(400, E400), (403, "常に拒否するトピック、または許可リストにない"), (413, "max_bytes を超える"), (429, "最小間隔より速い"), (502, E502)], unverified=True,
       notes="拒否リスト（運動・関節・腕・RPC など）は許可リストに書いても効かない。許可リストはサーバー側の --topics-file だけで、画面・API から増やせない。"),
    ep("POST", "/api/camera/start", "postCameraStart", "camera", "Jetson で camera_stream.py を起動（再起動はしない。他が掴んでいれば失敗しうる）", SETTINGS, None, errors=[(502, E502)]),
    ep("POST", "/api/camera/stop", "postCameraStop", "camera", "camera_stream.py だけを停止（videohub など他のプロセスは止めない）", SETTINGS, None, errors=[(502, E502)]),
    ep("POST", "/api/lidar/start", "postLidarStart", "camera", "Jetson で lidar_stream.py を起動（DDS を購読するだけ。再起動はしない）", SETTINGS, None, errors=[(502, E502)]),
    ep("POST", "/api/lidar/stop", "postLidarStop", "camera", "lidar_stream.py だけを停止", SETTINGS, None, errors=[(502, E502)]),
    ep("POST", "/api/monitor", "postMonitor", "ops", "自動確認・自動再接続の停止／再開、または 1 回だけ確認", SETTINGS, "Monitor",
       request="MonitorPatch", example={"paused": True}, errors=[(400, E400)],
       notes="停止中はサーバーが ssh も DDS も叩かない。停止中もモード切替などの明示操作は実行できる。"),
    ep("POST", "/api/hub/stop", "postHubStop", "settings", "ハブ（このコンソール）自体を止める。画面からは再起動できない", SETTINGS, "HubStopAck",
       request="HubStopPatch", example={"confirm": True}, errors=[(400, E400 + "（confirm が true でない）"), (409, "タスク実行中、または運動の操作権が取られている")],
       unverified=True,
       notes="応答の後に終了する（終了コード 0。systemd の Restart=on-failure では再起動しない）。止まったあと、再起動は Jetson で行う。カメラ・lidar の配信は別プロセスで、止まらない見込み（未確認）。"),
    ep("POST", "/api/mode", "postMode", "ops", "G1 のモード（FSM）を切り替える。**ロボットが動く**", MOTION, "ModeAck",
       request="ModePatch", example={"id": 3}, errors=[(400, E400 + "（許可されていない id）"), (502, E502)], unverified=True,
       notes="受理（set_code=0）は到達ではない。実行後は GET /api/status を繰り返し読み、mode.fsm_id が目標になるか確認する（15 秒を目安）。"
             "歩行中（500/501）に切り替えると転倒の恐れがある。実行前に必ず人へ確認する。実機では未検証。"),
    ep("POST", "/api/speed", "postSpeed", "ops", "速度モード（SetSpeedMode）を切り替える。**ロボットの歩行速度が変わる**", MOTION, "ModeAck",
       request="SpeedPatch", example={"value": 0}, errors=[(400, E400 + "（許可されていない value）"), (502, E502)], unverified=True,
       notes="value の意味（どちらが低速か）は資料が無く未確認。読み戻しの手段が無い。歩行中（500/501）のときだけ送る。"),
    ep("POST", "/api/tasks/start", "postTaskStart", "tasks", "定義済みタスクを起動する（名前だけ渡す）。タスクの中身によってはロボットが動きうる", SETTINGS, "Task",
       request="TaskNamePatch", example={"name": "sample-countdown"},
       errors=[(400, E400), (404, "定義にないタスク"), (409, "同じタスク、または同じ排他グループが実行中"), (500, "起動できない")], unverified=True,
       notes="シェルを経由しない。実行後は GET /api/tasks/logs と GET /api/tasks の state / result / exit_code で結果を確認する。"
             "タスクの verified=false は実機で未確認で、caution が注意書き（画面は実行前にポップアップで見せる）。"
             "nav2-up / nav2-down は警備班の g1up.sh と撤収手順を呼ぶ。発進ゲート（G1_ARM）と enable_navigation は行わない。"),
    ep("POST", "/api/tasks/stop", "postTaskStop", "tasks", "実行中のタスクを止める（プロセスグループへ SIGTERM、猶予後に SIGKILL）", SETTINGS, "Task",
       request="TaskNamePatch", example={"name": "sample-countdown"}, errors=[(400, E400), (404, "定義にないタスク"), (409, "実行中ではない")]),
    ep("POST", "/api/motion/claim","postMotionClaim", "motion", "操作権を取る（運動ヘルパーを起動する。初回は数秒かかる）", WRITE, "MotionClaim",
       request="MotionClaimPatch", example={}, errors=[(400, E400), (409, "他の人が操作中・手動操作が禁止されている"), (502, "運動ヘルパーが起動しない"), (503, "--local でないため使えない")], unverified=True,
       notes="操作者は 1 人。接続断・緊急停止・解放のあとは自動で再開しないので、もう一度 claim する。"),
    ep("POST", "/api/motion/ctl", "postMotionCtl", "motion", "目標速度を送る。**ロボットが動く**。10 Hz で送り続けること（0.5 秒途絶すると停止して操作権を解除）", MOTION, "MotionStatus",
       request="MotionCtlPatch", example={"session": "example", "seq": 1, "vx": 0.0, "vy": 0.0, "wz": 0.0}, errors=[(400, E400 + "（数値でない・有限でない）"), (409, "操作権がない")], unverified=True,
       notes="FSM 500/501 以外では動かさない（blocked に理由）。速度は上限で丸める（実機未検証の初期値）。人が見ていないときに送らない。物理リモコンを手元に置く。"),
    ep("POST", "/api/motion/release", "postMotionRelease", "motion", "操作権を手放す（ゼロ速度にする）", MOTION, "MotionStatus",
       request="MotionSessionPatch", example={"session": "example"}, errors=[(400, E400), (409, "操作権がない")], unverified=True),
    ep("POST", "/api/motion/estop", "postMotionEstop", "motion", "緊急停止。ゼロ速度＋StopMove を送り、操作権を解除する（操作権が無くても誰でも押せる）", MOTION, "MotionStatus", unverified=True,
       notes="ダンピングやゼロトルクにはしない（自律直立を保つ）。**ソフトの停止は安全規格を満たさない。最後の砦は物理リモコン。**"),
    ep("POST", "/api/motion/action", "postMotionAction", "motion", "アクション（手を振る・握手）。**ロボットが動く**", MOTION, "MotionStatus",
       request="MotionActionPatch", example={"session": "example", "name": "wave"}, errors=[(400, E400 + "（許可されていないアクション）"), (409, "操作権がない・歩行モードでない"), (502, E502)], unverified=True,
       notes="SetTaskId の番号と効果は実機未検証。握手は開始のあと shake_end で終える。"),
    ep("POST", "/api/audio/volume", "postVolume", "audio", "音量を設定し、読み戻して一致を確認する", WRITE, "VolumeSet",
       request="VolumePatch", example={"volume": 40}, errors=[(400, E400), (502, E502)], unverified=True,
       notes="volume_after が指定値と一致していれば反映済み。"),
    ep("POST", "/api/audio/led", "postLed", "audio", "頭部 LED の色を設定", WRITE, "Ack",
       request="LedPatch", example={"r": 0, "g": 255, "b": 0}, errors=[(400, E400), (502, E502)], unverified=True,
       notes="code=0 は受理のみ。色が変わったかは目視（読み戻せない）。"),
    ep("POST", "/api/audio/tts", "postTts", "audio", "テキストを読み上げる", WRITE, "Ack",
       request="TtsPatch", example={"text": "こんにちは", "speaker_id": 0}, errors=[(400, E400), (502, E502)], unverified=True,
       notes="code=0 は受理のみ。音が出たかは耳で確認。"),
    ep("POST", "/api/scenario", "postScenario", "mock", "模擬シナリオを切り替える", SETTINGS, "Scenario",
       request="ScenarioPatch", example={"scenario": "g1_off"}, errors=[(400, E400)], mock_only=True),
]


# ---- OpenAPI の組み立て --------------------------------------------------------------------------------------
def _operation(e):
    resp_200 = {"description": e["ok_status"]}
    if e["response"]:
        resp_200["content"] = {"application/json": {"schema": ref(e["response"])}}
    elif e["content"]:
        resp_200["content"] = {e["content"]: {"schema": {"type": "string", "format": "binary"} if "stream" in e["ok_status"] or "MJPEG" in e["ok_status"] or e["content"].startswith("image/") else {"type": "string"}}}
    else:
        resp_200["content"] = {"application/json": {"schema": {"type": "object", "additionalProperties": True}}}
    responses = {"200": resp_200}
    for code, why in e["errors"]:
        responses[str(code)] = {"description": why, "content": {"application/json": {"schema": ref("Error")}}}
    desc = e["notes"] or ""
    op = {"operationId": e["operationId"], "tags": [e["tag"]], "summary": e["summary"], "x-effect": e["effect"]}
    if desc:
        op["description"] = desc
    if e["unverified"]:
        op["x-real-robot-verified"] = False
    if e["mock_only"]:
        op["x-mock-only"] = True
    if e["path"] == "/camera/{name}":
        op["parameters"] = [{"name": "name", "in": "path", "required": True, "schema": S(enum=["std", "d435i"])}]
    if e["request"]:
        op["requestBody"] = {"required": True, "content": {"application/json": {
            "schema": ref(e["request"]), "example": e["example"]}}}
    if e["path"] == "/lidar":
        op["parameters"] = [{"name": "voxel", "in": "query", "required": False, "schema": {"type": "number", "minimum": 0, "maximum": 1}},
                            {"name": "max", "in": "query", "required": False, "schema": I(minimum=100, maximum=30000)}]
    op["responses"] = responses
    return op


def openapi() -> dict:
    paths = {}
    for e in ENDPOINTS:
        paths.setdefault(e["path"], {})[e["method"].lower()] = _operation(e)
    return {
        "openapi": "3.0.3",
        "info": {"title": "G1 開発コンソール API", "version": "0.1.0",
                 "description": "人は画面で状態を把握し、Claude Code / Codex は同じ内容をこの API で読み、操作する。\n"
                                "x-effect: read=読むだけ / settings=コンソールの設定だけ変える / robot_write=G1 に書く（音・LED） / robot_motion=G1 が動く。\n"
                                "x-real-robot-verified=false は実機で未確認。エラー: " + ERRORS},
        "servers": [{"url": "http://127.0.0.1:{port}", "variables": {"port": {"default": "18790"}}}],
        "tags": [{"name": k, "description": v} for k, v in TAGS.items()],
        "paths": paths,
        "components": {"schemas": SCHEMAS},
    }


def render_markdown() -> str:
    doc = openapi()
    lines = [
        "# 開発コンソール API 定義", "",
        "<!-- api_spec.py から生成。手で編集せず `python3 Console/g1console/api_spec.py` で再生成する -->", "",
        "正本は OpenAPI: **`docs/openapi.yaml`**（実行中は `GET /openapi.yaml`、JSON なら `GET /api`）。"
        "この文書は人と AI が最初に読む要約。ベース URL は `http://127.0.0.1:18790`。", "",
        "## 使い方の型", "",
        "- **「〜を確認して」**: まず `GET /api/status`（接続・モード・古さ）。詳細は `GET /api/snapshot` 1 回で足りる。",
        "  `jetson.state` / `g1.state` が ok でなければ、その先の値は信用しない。`age_s` が数秒以上、または `paused=true` なら古い値。",
        "- **「〜を実行して」**: `x-effect: robot_motion` は**ロボットが動く**。実行前に人へ確認し、実行後は `GET /api/status` で**到達を確認**する（受理 ≠ 到達）。",
        "  `robot_write`（音・LED）は受理のみで、効果は目視・聴取で確認する。",
        "- `x-real-robot-verified: false`・`GET /api/features` の `verified=false` は実機で未確認。結果を断定しない。",
        "- 本文は JSON。エラーはすべて `Error`（`{\"error\": \"理由\"}`）。" + ERRORS,
        "- 画面のタブ名 = URL ハッシュ = タグ名（`#state` ↔ `/api/state`）。", "",
        "## エンドポイント", "", "| メソッド | パス | operationId | x-effect | 概要 |", "|---|---|---|---|---|",
    ]
    for path, methods in doc["paths"].items():
        for m, op in methods.items():
            flags = (" ⚠実機未検証" if op.get("x-real-robot-verified") is False else "") + (" （mock のみ）" if op.get("x-mock-only") else "")
            lines.append("| %s | `%s` | %s | %s | %s%s |" % (m.upper(), path, op["operationId"], op["x-effect"], op["summary"], flags))
    lines += ["", "## リクエストの例", ""]
    for path, methods in doc["paths"].items():
        for m, op in methods.items():
            if "requestBody" in op:
                body = op["requestBody"]["content"]["application/json"]
                lines += ["### %s %s" % (m.upper(), path), "", op["summary"], "",
                          "```json", json.dumps(body["example"], ensure_ascii=False), "```",
                          "スキーマ: `%s`" % body["schema"]["$ref"].rsplit("/", 1)[1] + ("。" + op["description"] if op.get("description") else ""), ""]
    return "\n".join(lines)


# ---- YAML 出力（標準ライブラリのみ。文字列は JSON 記法で引用＝YAML として有効） ------------------------------
def _scalar(v):
    return json.dumps(v, ensure_ascii=False)


def _key(k):
    k = str(k)
    return k if k.replace("_", "a").replace("-", "a").isalnum() and not k[0].isdigit() else _scalar(k)


def _yaml(v, indent=0):
    pad = "  " * indent
    if isinstance(v, dict):
        if not v:
            return "{}"
        out = []
        for k, x in v.items():
            if isinstance(x, (dict, list)) and x:
                out.append("%s%s:\n%s" % (pad, _key(k), _yaml(x, indent + 1)))
            else:
                out.append("%s%s: %s" % (pad, _key(k), _yaml(x, indent + 1)))
        return "\n".join(out)
    if isinstance(v, list):
        if not v:
            return "[]"
        return "\n".join("%s- %s" % (pad, _yaml(x, indent + 1).lstrip()) for x in v)
    return _scalar(v)


def render_yaml() -> str:
    return "# api_spec.py から生成（手で編集しない）\n" + _yaml(openapi()) + "\n"


if __name__ == "__main__":
    from pathlib import Path
    docs = Path(__file__).resolve().parent.parent / "docs"
    (docs / "API.md").write_text(render_markdown(), encoding="utf-8")
    (docs / "openapi.yaml").write_text(render_yaml(), encoding="utf-8")
    print("[api] wrote", docs / "API.md", docs / "openapi.yaml")
