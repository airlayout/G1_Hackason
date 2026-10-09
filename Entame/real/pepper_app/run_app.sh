#!/usr/bin/env bash
# Serves the app on 0.0.0.0:8501 (.streamlit/config.toml). Open http://<this PC's IP>:8501
set -euo pipefail
cd "$(dirname "$0")"
exec .venv/bin/streamlit run app.py "$@"
