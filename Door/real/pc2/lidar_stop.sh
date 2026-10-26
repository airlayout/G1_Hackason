#!/usr/bin/env bash
# ライダーの推定器を止める（SIGTERM。止める時に推定の全部を npz に書く）。door_run_real.py が DOOR のときに止めると、
# 0.5 s 後に door_run_real.py が「届かない」で立ったまま止まる。ふだんは door_stop.sh の後に
pid=$(cat ~/door_real/logs/lidar.pid 2>/dev/null) && kill -0 "$pid" 2>/dev/null || { echo "[lidar] 動いていない"; exit 1; }
kill -TERM "$pid"
for _ in $(seq 20); do kill -0 "$pid" 2>/dev/null || break; sleep 0.2; done
kill -0 "$pid" 2>/dev/null && echo "[lidar] ⚠️ 4 秒で止まらない（pid ${pid}）" || { echo "[lidar] 止めた"; tail -2 ~/door_real/logs/lidar_console_latest.log; }
