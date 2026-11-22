"""Main area: status bar and the ライブ / 割り当て / ログ / 接続確認 tabs."""
import pandas as pd
import streamlit as st

from .controller import AppController
from .events import SEEN_PREFIX
from .labels import ACTION_LABELS, trigger_label, trigger_options
from .naoqi_robot import read_pepper_info
from .rules import rules_from_records

KIND_LABELS = {"event": "認識", "command": "指令", "result": "結果", "error": "エラー"}
COLUMNS = {"enabled": "有効", "trigger": "きっかけ", "action": "操作",
           "cooldown_s": "待ち時間（秒）", "note": "メモ"}


def status_bar(controller: AppController) -> None:
    snap = controller.snapshot()
    running = controller.running  # the loop exists; snap.running waits for the first frame
    left, right = st.columns([3, 1], vertical_alignment="center")
    with left:
        badges = st.container(horizontal=True)
        if running:
            badges.badge("認識中", icon=":material/visibility:", color="green")
            badges.badge(snap.source_name, color="gray")
            pepper = snap.robot_name == "Pepper"
            badges.badge(f"出力: {snap.robot_name}", color="orange" if pepper else "blue")
            badges.badge("反応する" if snap.reacting else "反応しない",
                         color="violet" if snap.reacting else "gray")
        else:
            badges.badge("停止中", icon=":material/pause:", color="gray")
    if running:
        label = "反応を止める" if snap.reacting else "反応を始める"
        if right.button(label, type="secondary" if snap.reacting else "primary", width="stretch",
                        icon=":material/front_hand:"):
            controller.set_reacting(not snap.reacting)
            st.rerun()


@st.fragment(run_every=0.3)
def live_view(controller: AppController) -> None:
    snap = controller.snapshot()
    if snap.frame_jpeg is None:
        st.info("左の「開始」を押すと、ここに映像と認識結果が出ます。" if snap.error is None
                else f"エラー: {snap.error}")
        return
    image_col, info_col = st.columns([3, 2], gap="large")
    image_col.image(snap.frame_jpeg, width="stretch")
    with info_col:
        row = st.columns(3)
        row[0].metric("fps", f"{snap.fps:.1f}")
        row[1].metric("推論 ms", f"{snap.inference_ms:.0f}")
        row[2].metric("人数", snap.persons)
        row = st.columns(3)
        row[0].metric("首 yaw", f"{snap.head[0]:+.2f}", help="rad。正が左")
        row[1].metric("首 pitch", f"{snap.head[1]:+.2f}", help="rad。正が下")
        row[2].metric("Pepper", "動作中" if snap.robot_busy else "待機")
        objects = ", ".join(f"{k} ×{v}" for k, v in sorted(snap.objects.items())) or "なし"
        st.markdown(f"**物**: {objects}  \n**動作**: {' / '.join(snap.gestures) or 'なし'}")
        if snap.error:
            st.error(snap.error)
        recent = [e for e in snap.log if e.kind != "result"][-8:][::-1]
        if recent:
            st.dataframe(pd.DataFrame([{"時刻": e.time, "種類": KIND_LABELS.get(e.kind, e.kind),
                                        "内容": e.text} for e in recent]),
                         hide_index=True)


def _editor_rows(rules) -> pd.DataFrame:
    return pd.DataFrame([{COLUMNS["enabled"]: r.enabled, COLUMNS["trigger"]: trigger_label(r.trigger),
                          COLUMNS["action"]: ACTION_LABELS[r.action],
                          COLUMNS["cooldown_s"]: r.cooldown_s, COLUMNS["note"]: r.note}
                         for r in rules], columns=list(COLUMNS.values()))


def _records_from_editor(frame: pd.DataFrame, triggers: dict[str, str]) -> list[dict]:
    actions = {label: key for key, label in ACTION_LABELS.items()}
    records = []
    for row in frame.to_dict("records"):
        if all(pd.isna(v) or v == "" for v in row.values()):
            continue  # empty row added by the editor
        enabled, note = row[COLUMNS["enabled"]], row[COLUMNS["note"]]
        records.append({"enabled": False if pd.isna(enabled) else bool(enabled),
                        "trigger": triggers.get(row[COLUMNS["trigger"]], row[COLUMNS["trigger"]]),
                        "action": actions.get(row[COLUMNS["action"]], row[COLUMNS["action"]]),
                        "cooldown_s": row[COLUMNS["cooldown_s"]],
                        "note": "" if pd.isna(note) else str(note)})
    return records


def rules_editor(controller: AppController) -> None:
    rules = controller.rules()
    labels = list(st.session_state.get("object_labels", []))
    labels += [r.trigger[len(SEEN_PREFIX):] for r in rules if r.trigger.startswith(SEEN_PREFIX)]
    triggers = trigger_options(dict.fromkeys(labels))
    st.caption("上の行ほど優先されます。1 回の指令ごとに 1 秒以上あけ、"
               "Pepper が動作中のきっかけは 2 秒まで待ちます。"
               "回る・進むはありません。")
    edited = st.data_editor(
        _editor_rows(rules), num_rows="dynamic", hide_index=True, key="rules_editor",
        column_config={
            COLUMNS["enabled"]: st.column_config.CheckboxColumn(width="small", default=True),
            COLUMNS["trigger"]: st.column_config.SelectboxColumn(options=list(triggers), required=True),
            COLUMNS["action"]: st.column_config.SelectboxColumn(options=list(ACTION_LABELS.values()),
                                                                required=True),
            COLUMNS["cooldown_s"]: st.column_config.NumberColumn(min_value=0.0, max_value=3600.0,
                                                                 step=0.5, default=5.0),
            COLUMNS["note"]: st.column_config.TextColumn(width="large"),
        })
    save_col, reset_col, _ = st.columns([1, 1, 3])
    if save_col.button("保存して反映", type="primary", width="stretch", icon=":material/save:"):
        try:
            controller.save_rules(rules_from_records(_records_from_editor(edited, triggers)))
            st.success("保存しました（rules/rules.yaml）")
        except ValueError as exc:
            st.error(f"保存できません: {exc}")
    if reset_col.button("既定に戻す", width="stretch", icon=":material/restart_alt:"):
        controller.reset_rules()
        st.session_state.pop("rules_editor", None)
        st.rerun()
    _manual_actions(controller)


def _manual_actions(controller: AppController) -> None:
    st.markdown("#### 動作を試す")
    if not controller.running:
        st.caption("「開始」すると使えます。")
        return
    front_col, wave_col, _ = st.columns([1, 1, 3])
    for col, action in ((front_col, "look_front"), (wave_col, "wave")):
        if col.button(ACTION_LABELS[action], width="stretch", key=f"manual_{action}"):
            if not controller.run_action(action):
                st.warning("Pepper が動作中なので見送りました")


@st.fragment(run_every=2.0)
def log_view(controller: AppController) -> None:
    """Refreshes on its own: tabs do not rerun the page when they are switched."""
    log = controller.snapshot().log
    if not log:
        st.caption("まだ記録がありません。")
        return
    kinds = st.pills("種類", list(KIND_LABELS), format_func=KIND_LABELS.get,
                     selection_mode="multi", default=list(KIND_LABELS), key="log_kinds")
    rows = [{"時刻": e.time, "種類": KIND_LABELS.get(e.kind, e.kind), "内容": e.text}
            for e in reversed(log) if e.kind in (kinds or [])]
    frame = pd.DataFrame(rows, columns=["時刻", "種類", "内容"])
    st.dataframe(frame, hide_index=True, height=480)
    st.download_button("CSV で保存", frame.to_csv(index=False).encode("utf-8-sig"),
                       file_name="pepper_app_log.csv", mime="text/csv", icon=":material/download:")


def _use_animation(name: str) -> None:
    st.session_state.wave_animation = name


def connection_check() -> None:
    ip, port = st.session_state.get("pepper_ip", "").strip(), int(st.session_state.get("pepper_port", 9559))
    st.caption("NAOqi API（ポート 9559）に読み取りだけで接続し、Pepper の情報を確かめます。"
               "Pepper は動きません。")
    if not st.button("Pepper に接続して確かめる", type="primary", disabled=not ip,
                     icon=":material/lan:"):
        if not ip:
            st.caption("左の「Pepper の IP」を入れてください。")
        return
    try:
        with st.spinner(f"{ip}:{port} に接続しています…"):
            info = read_pepper_info(ip, port)
    except Exception as exc:
        st.error(f"接続できません: {type(exc).__name__}: {exc}")
        st.caption("確かめること: 同じルータにいるか、IP、ポート 9559。"
                   "NAOqi 2.9 は 9503 番と認証が要るので、この方式では繋がりません。")
        return
    cols = st.columns(3)
    cols[0].metric("名前", info.robot_name)
    cols[1].metric("NAOqi", info.naoqi_version)
    cols[2].metric("起きている", "はい" if info.awake else "いいえ（首と腕は動きません）")
    if not info.wave_animations:
        st.warning("Gestures/Hey のアニメーションが見つかりません")
        return
    for name in info.wave_animations:
        st.button(f"これを使う: {name}", key=f"use_{name}", on_click=_use_animation, args=(name,))
