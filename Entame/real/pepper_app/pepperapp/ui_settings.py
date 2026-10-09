"""Sidebar: source, Pepper connection, robot mode, recognition settings, start / stop."""
from pathlib import Path

import streamlit as st

from .controller import AppController, Settings
from .detector import DetectorConfig
from .labels import OBJECT_CHOICES, object_label
from .robot import LookConfig

SAMPLE_IMAGE = (Path(__file__).resolve().parents[2]
                / "pepper_vlm_brain" / "scenarios" / "images" / "qwen-demo.jpg")
SOURCES = {"file": "動画・画像ファイル", "webcam": "USB カメラ", "pepper": "Pepper のカメラ"}
MODES = {"dry_run": "試運転（Pepper に送らない）", "pepper": "本番（Pepper の首と腕を動かす）"}
# Pepper's head on WiFi Physical_AI (2.4GHz, DHCP). Wired link: 192.168.123.99. Measured 2026-10-09.
DEFAULT_PEPPER_IP = "192.168.0.85"
# Over 2.4GHz WiFi the camera gives QVGA 4.2 fps but VGA only 1.1 fps (wired VGA: 29.7 fps).
DEFAULT_RESOLUTION = "QVGA"


def _defaults() -> None:
    defaults = {"source_kind": "pepper", "file_path": str(SAMPLE_IMAGE), "webcam_index": 0,
                "pepper_ip": DEFAULT_PEPPER_IP, "pepper_port": 9559,
                "camera_resolution": DEFAULT_RESOLUTION, "camera_fps": 10,
                "robot_mode": "dry_run", "pause_awareness": True, "safety_ok": False,
                "wave_animation": LookConfig().wave_animation, "object_labels": ["chair"],
                "imgsz": 640, "confidence": 0.5, "threads": 4}
    for key, value in defaults.items():
        st.session_state.setdefault(key, value)


def _source_inputs() -> None:
    st.radio("映像の入口", list(SOURCES), format_func=SOURCES.get, key="source_kind")
    kind = st.session_state.source_kind
    if kind == "file":
        st.text_input("ファイルのパス（動画か画像）", key="file_path")
    elif kind == "webcam":
        st.number_input("カメラ番号", min_value=0, max_value=9, step=1, key="webcam_index")
    else:
        st.segmented_control("解像度", ["VGA", "QVGA"], key="camera_resolution", required=True,
                             help="Wi-Fi では QVGA（約 4fps）。VGA は有線向け（Wi-Fi だと約 1fps）")
        st.slider("フレーム数（fps）", 1, 30, key="camera_fps")


def _pepper_inputs() -> None:
    st.text_input("Pepper の IP", key="pepper_ip", placeholder=DEFAULT_PEPPER_IP,
                  help="Wi-Fi（Physical_AI）は 192.168.0.85、有線（OMEN 直結）は 192.168.123.99。"
                       "192.168.0.211 はタブレットで、ここには繋がらない")
    st.number_input("ポート（NAOqi API）", min_value=1, max_value=65535, step=1, key="pepper_port")


def _mode_inputs() -> None:
    st.radio("動かす先", list(MODES), format_func=MODES.get, key="robot_mode")
    if st.session_state.robot_mode == "pepper":
        st.checkbox("Pepper の自律的な首振りを止める（見る操作と喧嘩しないように）",
                    key="pause_awareness")
        st.checkbox("Pepper の腕の届く範囲に人や物が無いことを確かめた", key="safety_ok")
    st.text_input("手を振るアニメーション", key="wave_animation",
                  help="「接続確認」タブで Pepper に入っている名前を調べられます")


def _recognition_inputs() -> None:
    st.multiselect("人のほかに見つける物", OBJECT_CHOICES, format_func=object_label,
                   key="object_labels")
    st.select_slider("画像の大きさ（px）", options=[320, 480, 640], key="imgsz",
                     help="小さいほど速いが、遠くの人や小さい物を見落とす。640 を推奨")
    st.slider("信頼度のしきい値", 0.1, 0.9, step=0.05, key="confidence")
    st.number_input("CPU のスレッド数", min_value=1, max_value=16, step=1, key="threads")


def current_settings() -> Settings:
    s = st.session_state
    return Settings(
        source_kind=s.source_kind, file_path=s.file_path, webcam_index=int(s.webcam_index),
        pepper_ip=s.pepper_ip.strip(), pepper_port=int(s.pepper_port),
        camera_resolution=s.camera_resolution or DEFAULT_RESOLUTION, camera_fps=int(s.camera_fps),
        robot_mode=s.robot_mode, pause_awareness=bool(s.pause_awareness),
        detector=DetectorConfig(imgsz=int(s.imgsz), confidence=float(s.confidence),
                                object_labels=tuple(s.object_labels), threads=int(s.threads)),
        look=LookConfig(wave_animation=s.wave_animation.strip()))


def _start(controller: AppController) -> bool:
    s = st.session_state
    needs_ip = s.robot_mode == "pepper" or s.source_kind == "pepper"
    if needs_ip and not s.pepper_ip.strip():
        st.error("Pepper の IP を入れてください")
        return False
    if s.robot_mode == "pepper" and not s.safety_ok:
        st.error("本番では、腕の届く範囲を確かめたチェックが要ります")
        return False
    try:
        settings = current_settings()
        with st.spinner("モデルを読み込み、接続しています…"):
            controller.start(settings)
    except Exception as exc:  # shown to the operator with the reason
        st.error(f"開始できません: {type(exc).__name__}: {exc}")
        return False
    return True


def sidebar(controller: AppController) -> None:
    _defaults()
    with st.sidebar:
        st.markdown("### 映像")
        _source_inputs()
        st.markdown("### Pepper（NAOqi API）")
        _pepper_inputs()
        st.markdown("### 動かす先")
        _mode_inputs()
        st.markdown("### 認識（CPU）")
        _recognition_inputs()
        st.divider()
        start_col, stop_col = st.columns(2)
        label = "再起動" if controller.running else "開始"
        if start_col.button(label, type="primary", width="stretch", icon=":material/play_arrow:"):
            if _start(controller):
                st.rerun()  # redraw the whole page (status bar, button labels) in the new state
        if stop_col.button("停止", width="stretch", icon=":material/stop:",
                           disabled=not controller.running):
            controller.stop()
            st.rerun()
        st.caption("設定の変更は「再起動」で反映されます")
