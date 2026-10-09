"""Pepper 認識アプリ（Streamlit）。起動: ./run_app.sh（または streamlit run app.py）"""
import logging

import streamlit as st

from pepperapp.controller import AppController
from pepperapp.ui_settings import sidebar
from pepperapp.ui_tabs import connection_check, live_view, log_view, rules_editor, status_bar

logging.basicConfig(level=logging.INFO, format="[%(name)s] %(message)s")
st.set_page_config(page_title="Pepper 認識アプリ", page_icon=":material/smart_toy:", layout="wide")


@st.cache_resource
def get_controller() -> AppController:
    """One controller per server: every browser tab sees and drives the same Pepper."""
    return AppController()


controller = get_controller()
sidebar(controller)
st.title("Pepper 認識アプリ")
status_bar(controller)
live, assign, logs, check = st.tabs(["ライブ", "割り当て", "ログ", "接続確認"])
with live:
    live_view(controller)
with assign:
    rules_editor(controller)
with logs:
    log_view(controller)
with check:
    connection_check()
