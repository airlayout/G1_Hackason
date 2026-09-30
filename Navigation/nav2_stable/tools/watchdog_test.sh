#!/usr/bin/env bash
# D-10 watchdog 検証: teleop を kill -9 して SDK 側がゼロを出すかを見る
cd ~/nav2_bridge_cpp/build
rm -f /tmp/wd_server.log /tmp/wd_teleop.log /tmp/wd_joints.csv /tmp/wd_rec.log

echo "[1] 関節記録を開始(1kHz)"
setsid nohup /usr/bin/python3 ~/record_joints.py 40 /tmp/wd_joints.csv > /tmp/wd_rec.log 2>&1 &
sleep 3

echo "[2] SDK側プロセスを --arm で起動"
setsid nohup stdbuf -oL -eL ./g1_sdk_bridge_real_server --network-interface eth0 --arm > /tmp/wd_server.log 2>&1 &
sleep 6
grep -E '発進ゲート|起動した' /tmp/wd_server.log | tail -2

echo "[3] teleop を 30 秒指定で起動(vx=0.3)"
setsid nohup stdbuf -oL ./g1_sdk_bridge_teleop --vx 0.3 --seconds 30 > /tmp/wd_teleop.log 2>&1 &
sleep 3
TPID=$(pgrep -f g1_sdk_bridge_teleop | head -1)
echo "  teleop PID=$TPID / 3秒間送信した"
tail -2 /tmp/wd_teleop.log

echo "[4] ★ teleop を kill -9 で強制終了(ゼロ送信をさせない)"
date +%s.%N > /tmp/wd_kill_time
kill -9 "$TPID"
echo "  killed at $(cat /tmp/wd_kill_time)"

sleep 8
echo "[5] SDK側プロセスの反応"
tail -8 /tmp/wd_server.log

pkill -f g1_sdk_bridge_real_server
sleep 1
echo "[6] 片付け完了"
