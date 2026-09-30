#!/usr/bin/env bash
# 通信断試験: teleop を SSH セッションに紐づけて起動し、外から g1-link を切る。
# SDK側プロセスと関節記録は setsid で生き残らせる(systemd常駐を想定＝D-07)。
cd ~/nav2_bridge_cpp/build
rm -f /tmp/cd_server.log /tmp/cd_rec.log /tmp/cd_joints.csv
setsid nohup /usr/bin/python3 ~/record_joints.py 45 /tmp/cd_joints.csv > /tmp/cd_rec.log 2>&1 &
setsid nohup stdbuf -oL -eL ./g1_sdk_bridge_real_server --network-interface eth0 --arm > /tmp/cd_server.log 2>&1 &
sleep 6
echo "[test] SDK側を起動した。teleop をこのSSHセッションで起動する(vx=0.3, 5秒)"
# exec で置き換える → このプロセスが SSH セッションに直接紐づく
exec stdbuf -oL ./g1_sdk_bridge_teleop --vx 0.3 --seconds 5
