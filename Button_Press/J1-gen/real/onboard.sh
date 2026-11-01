#!/usr/bin/env bash
# ノート PC から無線の ssh で、G1 の PC2 の上の押す処理（auto_press.py）を起動・監視・停止する（onboard。ボトル）。
#
#   bash Button_Press/J1-gen/real/onboard.sh sync                 # コードと重みを PC2 に送る（PC2 の configs/obstacles.yaml も上書きされる）
#   bash Button_Press/J1-gen/real/onboard.sh start                # dry-run（何も送らない）
#   bash Button_Press/J1-gen/real/onboard.sh start --execute      # 送る（人がリモコンを持つ。危なければ L2+B）
#   bash Button_Press/J1-gen/real/onboard.sh start -- --overlay reach_legacy.yaml   # `--` のあとは press_bottle.py に渡す
#   bash Button_Press/J1-gen/real/onboard.sh follow               # 出力を流し続けて見る（Ctrl+C で見るのをやめるだけ。押す処理は止まらない）
#   bash Button_Press/J1-gen/real/onboard.sh status               # 動いているか
#   bash Button_Press/J1-gen/real/onboard.sh stop                 # Ctrl+C と同じ: 安全に終了して、来た経路を戻る
#   bash Button_Press/J1-gen/real/onboard.sh stop-now             # 戻っている途中にもう一度: 戻るのをやめて、その場で weight を下げる
#   bash Button_Press/J1-gen/real/onboard.sh fetch                # PC2 の記録（runs・auto・出力）をノート PC の _local/button_press/*_pc2/ に持ってくる
#
# なぜ切り離して起動するか: ssh につないだまま動かすと、Wi-Fi の ssh が切れたときに押す処理が切断の合図（SIGHUP）で終わり、
# 来た経路を戻らずに止まる。setsid で端末から切り離すと、ssh が切れても最後まで動いて戻る。止めるのは別の ssh から合図を送る
# （ArmCommander は腕を動かしている間 SIGINT / SIGTERM を自分で受けるので、Ctrl+C と同じに止まる）。緊急停止はリモコンの L2+B。
#
# 環境変数: PC2（既定 unitree@192.168.0.82。DHCP で変わることがある）、SSH_OPTS（ssh に足すオプション）、
#           OBSTACLES（PC2 の机の箱のファイル。既定は PC2 の ~/button_press/obstacles_*.yaml のうち一番新しいもの）、
#           ONBOARD_ALLOW_OTHERS=1（別の班の g1_e2e が動いていても送る。既定は断る）
set -uo pipefail

PC2="${PC2:-unitree@192.168.0.82}"
SSH_OPTS="${SSH_OPTS:--o BatchMode=yes -o ConnectTimeout=8}"
REMOTE_REPO='~/button_press/repo'
PIDFILE='~/button_press/onboard.pid'
LOGDIR='~/button_press/onboard_logs'
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
FEATURE_DIR="$(cd "$HERE/.." && pwd)"
REPO_ROOT="$(cd "$FEATURE_DIR/../.." && pwd)"

# shellcheck disable=SC2086
remote() { ssh $SSH_OPTS "$PC2" "$@"; }

cmd="${1:-}"
shift || true

case "$cmd" in
sync)
    cd "$REPO_ROOT" || exit 1
    rsync -a --exclude __pycache__ -e "ssh $SSH_OPTS" Button_Press/J1-gen/common Button_Press/J1-gen/configs Button_Press/J1-gen/real \
        Button_Press/J1-gen/weights "$PC2:button_press/repo/Button_Press/J1-gen/" &&
        rsync -a --exclude __pycache__ -e "ssh $SSH_OPTS" Perception/common "$PC2:button_press/repo/Perception/" &&
        echo "[onboard] 送った（$PC2:button_press/repo）"
    ;;
start)
    execute=""
    extra=()
    while [ $# -gt 0 ]; do
        case "$1" in
        --execute) execute="--execute" ;;
        --) shift; extra=("$@"); break ;;
        *) echo "[onboard] 知らない引数: $1（press_bottle.py に渡すものは -- のあとに書く）"; exit 2 ;;
        esac
        shift
    done
    label="onboard${execute:+_exec}"
    # 引数を PC2 の bash にそのまま渡すために、1 つずつクォートする（無いときに printf が '' を 1 つ作らないように、あるときだけ）
    extra_q=""
    if [ ${#extra[@]} -gt 0 ]; then
        extra_q=$(printf ' %q' "${extra[@]}")
    fi
    remote "bash -s" <<EOF
set -u
if [ -f $PIDFILE ] && kill -0 \$(head -1 $PIDFILE) 2>/dev/null; then
    echo "[onboard] ❌ もう動いている（pid \$(head -1 $PIDFILE)）。終わるのを待つか stop する"; exit 1
fi
obs="\${OBSTACLES:-${OBSTACLES:-}}"
[ -n "\$obs" ] || obs=\$(ls -t ~/button_press/obstacles_*.yaml 2>/dev/null | head -1)
[ -n "\$obs" ] && [ -f "\$obs" ] || { echo "[onboard] ❌ 机の箱のファイルが無い（PC2 で make_table_boxes.py --save --obstacles-file ~/button_press/obstacles_<日付>.yaml）"; exit 1; }
pgrep -f "depth_server/rgbd_serve[r].py" >/dev/null || echo "[onboard] ⚠️ カメラの配信（rgbd_server）が動いていない。事前確認で止まる"
# 別の班の G1 を動かすプログラム（2026-11-01: g1_e2e.vla.robot --execute。同時に動いていた回で NoMotionError）。送るときは断る
others=\$(pgrep -fa "g1_e2[e]" | cut -c1-100)
if [ -n "\$others" ]; then
    echo "[onboard] ⚠️ 別の班のプログラムが動いている: \$others"
    if [ -n "$execute" ] && [ -z "${ONBOARD_ALLOW_OTHERS:-}" ]; then
        echo "[onboard] ❌ 送らない（指令が重なるおそれ）。止まってから、またはどうしても送るなら ONBOARD_ALLOW_OTHERS=1 を前に付ける"; exit 1
    fi
fi
mkdir -p $LOGDIR
log=$LOGDIR/\$(date +%Y%m%d_%H%M%S)_$label.log
cd $REMOTE_REPO || exit 1
# setsid -f: 新しいセッション（端末なし）で起動する。bash の pid がそのままプロセスグループの番号になり、exec でも変わらない
setsid -f bash -c "echo \\\$\\\$ > $PIDFILE; echo \$log >> $PIDFILE; exec env OPENBLAS_NUM_THREADS=1 PYTHONUNBUFFERED=1 ~/button_press/venv/bin/python -u \
    Button_Press/J1-gen/real/auto_press.py --target bottle $execute --network-interface eth0 --camera-config camera_onboard.yaml \
    --label $label -- --detector onnx --obstacles-file \$obs$extra_q > \$log 2>&1 < /dev/null"
sleep 1
echo "[onboard] 起動した（pid \$(head -1 $PIDFILE)、${execute:-dry-run}、机の箱 \$obs）"
echo "[onboard] 出力: \$log"
EOF
    ;;
follow)
    remote "tail -n 80 -f \$(sed -n 2p $PIDFILE)"
    ;;
status)
    remote "if [ -f $PIDFILE ] && kill -0 \$(head -1 $PIDFILE) 2>/dev/null; then echo \"[onboard] 動いている（pid \$(head -1 $PIDFILE)）\"; else echo '[onboard] 動いていない'; fi; \
            [ -f $PIDFILE ] && { echo \"[onboard] 出力: \$(sed -n 2p $PIDFILE)\"; tail -n 5 \$(sed -n 2p $PIDFILE); }"
    ;;
stop | stop-now)
    # プロセスグループ全体（auto_press.py と、その子の press_bottle.py）に SIGINT を送る。端末で Ctrl+C を押したときと同じ
    remote "if [ -f $PIDFILE ] && kill -0 \$(head -1 $PIDFILE) 2>/dev/null; then kill -INT -- -\$(head -1 $PIDFILE) && echo '[onboard] SIGINT を送った（$cmd）'; \
            else echo '[onboard] 動いていない'; fi"
    ;;
fetch)
    cd "$REPO_ROOT" || exit 1
    mkdir -p _local/button_press/runs_pc2 _local/button_press/auto_pc2 _local/button_press/onboard_logs_pc2
    for d in runs auto; do
        rsync -a -e "ssh $SSH_OPTS" "$PC2:button_press/repo/_local/button_press/$d/" "_local/button_press/${d}_pc2/"
    done
    rsync -a -e "ssh $SSH_OPTS" "$PC2:button_press/onboard_logs/" _local/button_press/onboard_logs_pc2/
    echo "[onboard] 持ってきた: _local/button_press/{runs_pc2,auto_pc2,onboard_logs_pc2}/"
    ;;
*)
    sed -n '2,20p' "${BASH_SOURCE[0]}"
    exit 2
    ;;
esac
