"""読み取りのみ: 腕を動かす前の状態確認。ログの最新行から関節状態と lowcmd を見る。Jetson のプロセスも見る。"""
import json
import subprocess
from pathlib import Path

d = Path("/Users/koba/aicle/G1_Hackason/Console/docs/g1_logs")
f = sorted(d.glob("live_*.jsonl"))[-1]
last = {}
for line in f.read_text(encoding="utf-8").splitlines():
    r = json.loads(line)
    if "topic" in r:
        last[r["topic"]] = r
print("log:", f.name, "lines")
low, cmd, bms, odom = (last.get(k) for k in ("rt/lowstate", "rt/lowcmd", "rt/lf/bmsstate", "rt/odommodestate"))
print("battery soc:", bms and bms["v"]["soc"], "current:", bms and bms["v"]["current"])
print("odom mode/gait/height:", odom and (odom["v"]["mode"], odom["v"]["gait_type"], odom["v"]["body_height"]))
print("lowstate mode_machine:", low and low["v"]["mode_machine"], " lowcmd mode_machine:", cmd and cmd["v"]["mode_machine"])
print("imu rpy:", low and low["v"]["imu_state"]["rpy"])
NAMES = ["L_hip_p", "L_hip_r", "L_hip_y", "L_knee", "L_ank_p", "L_ank_r", "R_hip_p", "R_hip_r", "R_hip_y", "R_knee", "R_ank_p", "R_ank_r",
         "waist_y", "waist_r", "waist_p", "L_sh_p", "L_sh_r", "L_sh_y", "L_elbow", "L_wr_r", "L_wr_p", "L_wr_y",
         "R_sh_p", "R_sh_r", "R_sh_y", "R_elbow", "R_wr_r", "R_wr_p", "R_wr_y"]
if low and cmd:
    ms, mc = low["v"]["motor_state"], cmd["v"]["motor_cmd"]
    print("%-8s %8s %8s %8s %6s %5s %5s" % ("joint", "q_state", "q_cmd", "dq", "kp", "kd", "mode"))
    for i, n in enumerate(NAMES):
        print("%-8s %8.3f %8.3f %8.3f %6.1f %5.1f %5s" % (n, ms[i]["q"], mc[i]["q"], ms[i]["dq"], mc[i]["kp"], mc[i]["kd"], mc[i]["mode"]))
r = subprocess.run(["ssh", "-o", "BatchMode=yes", "g1-ts",
                    "ps -eo pid,etime,pcpu,args --sort=-pcpu | grep -v grep | grep -i -E 'python|ros|unitree|g1|master_service|basic' | cut -c1-170 | head -30"],
                   capture_output=True, text=True, timeout=30)
print("--- Jetson processes ---")
print(r.stdout)
