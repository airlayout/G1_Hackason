#!/usr/bin/env bash
# G1 の PC2 の環境を調べる（見るだけ。何も入れない・書き換えない・起動しない）。
# 押す処理を PC2 で動かす準備（PC2_ONBOARD_CHECK.md）と、PC2 で YOLO（人の認識）を動かす準備のための調査。
# Web カメラを PC2 の USB に挿した状態で流すと、そのカメラが PC2 からどう見えるかも記録される。
#
# ラボ PC から（有線。鍵で入る。結果はラボ PC の _local/pc2_survey/ に残す）:
#   mkdir -p _local/pc2_survey
#   ssh -o PubkeyAuthentication=yes -o PreferredAuthentications=publickey -i ~/.ssh/id_ed25519_humanoid_expo g1 'bash -s' \
#       < Button_Press/J1-gen/real/pc2_survey.sh > _local/pc2_survey/$(date +%Y%m%d_%H%M).txt 2>&1
# パスワードで入るなら（ふつうのターミナルで）: ssh g1 'bash -s' < Button_Press/J1-gen/real/pc2_survey.sh > ...
# PC2 の中で直接: bash pc2_survey.sh
set -u

sec() { printf '\n===== %s =====\n' "$1"; }
run() { printf '$ %s\n' "$*"; timeout 20 bash -c "$*" 2>&1 || printf '(終了コード %s)\n' "$?"; }

sec "時刻・OS"
run "date '+%Y-%m-%d %H:%M:%S %Z'"
run "uname -a"
run "grep -E '^(PRETTY_NAME|VERSION_ID)=' /etc/os-release"
run "cat /etc/nv_tegra_release"
run "ldd --version | head -1"
run "strings \$(ldconfig -p | awk '/libstdc\\+\\+\\.so\\.6 / {print \$NF; exit}') | grep -o 'GLIBCXX_3\\.4\\.[0-9]*' | sort -t. -k3,3n | tail -1"
run "timedatectl 2>/dev/null | grep -E 'synchronized|NTP|Time zone'"

sec "CPU・メモリ・ディスク"
run "nproc"
run "uptime"
run "free -h"
run "df -h / \$HOME"
run "ps -eo pid,pcpu,pmem,etime,cmd --sort=-pcpu | head -15"

sec "GPU（Jetson）"
run "ls -d /usr/local/cuda* 2>/dev/null; cat /usr/local/cuda/version.json 2>/dev/null | grep -A1 '\"cuda\"' | head -3"
run "dpkg -l 2>/dev/null | grep -E 'nvidia-jetpack|tensorrt |libnvinfer[0-9]? ' | awk '{print \$2, \$3}' | head -5"
run "/usr/bin/python3 -c 'import tensorrt; print(\"tensorrt\", tensorrt.__version__)'"

sec "システムの Python（3.8）"
run "/usr/bin/python3 --version"
run "/usr/bin/python3 -c 'import pyrealsense2 as rs; print(\"pyrealsense2\", rs.__version__)'"
run "/usr/bin/python3 -m pip --version"

sec "conda の環境（lerobot は Python 3.12 のはず）"
CONDA_BASES=""
for d in "$HOME/miniconda3" "$HOME/miniforge3" "$HOME/anaconda3" /opt/conda; do
    [ -d "$d" ] && CONDA_BASES="$CONDA_BASES $d"
done
echo "conda の場所:${CONDA_BASES:- 見つからない}"
for base in $CONDA_BASES; do
    run "ls $base/envs"
    for py in "$base"/envs/*/bin/python; do
        [ -x "$py" ] || continue
        echo "--- $py"
        run "$py --version"
        "$py" - <<'PY' 2>&1
import importlib
for name in ["numpy", "cv2", "zmq", "yaml", "torch", "torchvision", "ultralytics", "onnxruntime",
             "cyclonedds", "unitree_sdk2py", "pinocchio", "mujoco", "pyrealsense2"]:
    try:
        m = importlib.import_module(name)
        extra = ""
        if name == "torch":
            extra = f" cuda={m.cuda.is_available()}"
        print(f"  {name}: {getattr(m, '__version__', '?')}{extra}")
    except Exception as e:  # noqa: BLE001
        print(f"  {name}: なし（{type(e).__name__}: {str(e)[:80]}）")
PY
    done
done

sec "ビルドの道具（cyclonedds を PC2 で組み立てるとき）"
run "gcc --version | head -1"
run "cmake --version | head -1"
run "git --version"
run "ls ~/work/unitree_sdk2/thirdparty 2>/dev/null; ls ~/work/unitree_sdk2/thirdparty/lib 2>/dev/null"

sec "ネットワーク"
run "ip -br addr"
run "ip route"
run "getent hosts pypi.org files.pythonhosted.org github.com"
run "curl -sS -o /dev/null -w 'pypi.org: HTTP %{http_code}（%{time_total} 秒）\\n' --max-time 8 https://pypi.org/simple/pip/"

sec "カメラ・既存のサービス"
run "ls -l /dev/video*"
run "for d in /sys/class/video4linux/video*; do printf '%s: %s\\n' \"\${d##*/}\" \"\$(cat \$d/name)\"; done"
run "lsusb"
run "command -v v4l2-ctl && v4l2-ctl --list-devices"
run "lsusb | grep -i 8086"
run "ps -o pid,lstart,pcpu,cmd -C videohub_pc4,videohub_pc4_chest"
run "pgrep -af 'rgbd_server|run_g1_server' | grep -v pgrep"

sec "置いてあるもの"
run "ls -la ~/button_press 2>/dev/null | head -20"
run "du -sh ~/button_press ~/lerobot ~/work 2>/dev/null"

printf '\n===== 終わり =====\n'
