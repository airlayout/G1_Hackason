#!/usr/bin/env bash
# 立ったまま止める（PC の Enter と同じ）。待ち（WAIT）・脱力（DAMP）のときは、これで終わる。何度送っても脱力にはならない
pid=$(cat ~/door_real/logs/door.pid 2>/dev/null) && kill -0 "$pid" 2>/dev/null || { echo "[door] 動いていない"; exit 1; }
kill -HUP "$pid" && echo "[door] 立ったまま止める合図を送った（pid ${pid}）"
