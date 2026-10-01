"""開発コンソール API の定義（唯一の出典）。OpenAPI 3.0.3 を組み立て、次の 3 つを同じ内容から出す。

  GET /api            OpenAPI（JSON）        GET /openapi.yaml  OpenAPI（YAML）
  docs/openapi.yaml   同上（ファイル）        docs/API.md        人と AI が読む用の要約

`python3 Console/g1console/api_spec.py` で docs/ を再生成する。tests が「スキーマ = 実際のレスポンス」を検証するので、
API を変えたらここを直す（食い違うとテストが落ちる）。手書きの YAML ではなく Python で組む理由は、
標準ライブラリだけで、スキーマの部品を共有し、テストから同じ定義を使えるから。
"""
import json

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


HEAD = {"jetson": ref("Link"), "g1": ref("Link"), "mode": ref("Mode"),
        "sampled_at": null(N("最後に G1 を確認した時刻（epoch 秒）。null=まだ確認していない")),
        "age_s": null(N("最後の確認から今までの秒数。大きければ古い値"))}
HEAD_REQ = list(HEAD)

SCHEMAS = {
    "Error": obj({"error": S("平易な日本語の理由")}, ["error"], "エラー本文（全エンドポイント共通）。G1・Jetson に届かない 502 では ok=false・offline・ssh_exit などの生の情報も付く", extra=True),
    "Link": obj({"state": S(enum=["ok", "down", "unknown", "checking"], desc="ok=届いている / down=届かない / unknown=上流が切れていて見えていない"),
                 "detail": S("理由（画面の診断文と同じ）")}, ["state", "detail"], "接続 1 段の状態。jetson が down なら g1 は unknown"),
    "Mode": obj({"state": S(enum=["ok", "debug", "unknown", "none"], desc="ok=通常モードで FSM ID が取れた / debug=デバッグモード / unknown=取得不可 / none=接続なし"),
                 "label": S("表示名"), "fsm_id": I("FSM ID（state=ok のときだけ）"), "service": S("モーションサービス名"), "detail": S()},
                ["state", "label"], "現在のモード"),
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
                                   "tau": N("トルク推定"), "temperature": A(N()), "cmd": null(free("lowcmd の指令値"))},
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
    "Buttons": obj({"buttons": A(obj({"id": I("POST /api/mode の id"), "label": S(), "danger": B()}, ["id", "label", "danger"]))}, ["buttons"]),
    "Features": obj({"features": A(obj({"id": S(), "label": S(), "implemented": B(), "verified": B("false=実機で未確認。結果を断定しない"), "note": S()},
                                       ["id", "label", "implemented", "verified"])),
                     "planned_modes": A({"type": "object", "additionalProperties": True}), "planned_actions": A({"type": "object", "additionalProperties": True})},
                    ["features"]),
    "SettingsValues": obj({"dev_pc": S("開発用 PC の IP（表示・記録のみ）"), "jetson_host": S("Jetson の IP かホスト名（ssh とカメラの接続先）"),
                           "jetson_user": S("空なら ~/.ssh/config に任せる"), "jetson_key": S("ssh 秘密鍵のパス。空なら既定"),
                           "g1_ip": S("G1 本体の IP（表示・記録のみ）"), "camera_port": I(minimum=1, maximum=65535)},
                          ["dev_pc", "jetson_host", "jetson_user", "jetson_key", "g1_ip", "camera_port"]),
    "SettingsPatch": obj({"dev_pc": S("IP アドレスかホスト名。空文字で未設定"), "jetson_host": S(), "jetson_user": S(), "jetson_key": S(),
                          "g1_ip": S(), "camera_port": I(minimum=1, maximum=65535)}, desc="指定した項目だけ変わる（全て任意）"),
    "Settings": obj({"settings": ref("SettingsValues"), "labels": {"type": "object", "additionalProperties": S()}}, ["settings", "labels"]),
    "MonitorPatch": obj({"paused": B("true=停止 / false=再開"), "check": B("true=1 回だけ確認")}, desc="どちらか、または両方"),
    "Monitor": obj({"paused": B()}, ["paused"]),
    "Scenarios": obj({"scenarios": {"type": "object", "additionalProperties": S()}, "current": S()}, ["scenarios", "current"]),
    "ScenarioPatch": obj({"scenario": S()}, ["scenario"]),
    "Scenario": obj({"scenario": S()}, ["scenario"]),
    "ModePatch": obj({"id": I("GET /api/buttons に載っている id のみ")}, ["id"]),
    "VolumePatch": obj({"volume": I(minimum=0, maximum=100)}, ["volume"]),
    "LedPatch": obj({"r": I(minimum=0, maximum=255), "g": I(minimum=0, maximum=255), "b": I(minimum=0, maximum=255)}, ["r", "g", "b"]),
    "TtsPatch": obj({"text": S("読み上げる文", minLength=1, maxLength=100), "speaker_id": I("話者 ID（意味は未確認）", enum=[0, 1])}, ["text", "speaker_id"]),
}

TAGS = {"ops": "操作タブ（接続・モード）", "state": "状態タブ", "joints": "関節タブ", "audio": "音声タブ",
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
    ep("GET", "/api/cameras", "getCameras", "camera", "カメラ一覧と、配信が設定済みか（映像は GET /camera/{name}）", READ, "Cameras"),
    ep("GET", "/api/camera/status", "getCameraStatus", "camera", "camera_stream の起動状態と、映像デバイスを掴んでいるプロセス（foreign＝自前以外）", READ, None,
       errors=[(502, E502)]),
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
    ep("POST", "/api/camera/start", "postCameraStart", "camera", "Jetson で camera_stream.py を起動（再起動はしない。他が掴んでいれば失敗しうる）", SETTINGS, None, errors=[(502, E502)]),
    ep("POST", "/api/camera/stop", "postCameraStop", "camera", "camera_stream.py だけを停止（videohub など他のプロセスは止めない）", SETTINGS, None, errors=[(502, E502)]),
    ep("POST", "/api/monitor", "postMonitor", "ops", "自動確認・自動再接続の停止／再開、または 1 回だけ確認", SETTINGS, "Monitor",
       request="MonitorPatch", example={"paused": True}, errors=[(400, E400)],
       notes="停止中はサーバーが ssh も DDS も叩かない。停止中もモード切替などの明示操作は実行できる。"),
    ep("POST", "/api/mode", "postMode", "ops", "G1 のモード（FSM）を切り替える。**ロボットが動く**", MOTION, "ModeAck",
       request="ModePatch", example={"id": 3}, errors=[(400, E400 + "（許可されていない id）"), (502, E502)], unverified=True,
       notes="受理（set_code=0）は到達ではない。実行後は GET /api/status を繰り返し読み、mode.fsm_id が目標になるか確認する（15 秒を目安）。"
             "歩行中（500/501）に切り替えると転倒の恐れがある。実行前に必ず人へ確認する。実機では未検証。"),
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
        resp_200["content"] = {e["content"]: {"schema": {"type": "string", "format": "binary"} if "stream" in e["ok_status"] or "MJPEG" in e["ok_status"] else {"type": "string"}}}
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
