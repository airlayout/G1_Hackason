#!/usr/bin/env bash
# 案 A（2026-10-26）: 遠隔操作（xr_teleoperate）を PC2 で動かし、Mac から操作する。
# 無線は Quest ↔ PC2 と Mac ↔ PC2（ssh）だけ。機体の制御は PC2 の中の有線（eth0）。
# 計画書: docs/plan/2026-10-26-g1-door-teleop-demos.md の 9 章
#
#   ./scripts/pc2_teleop.sh deploy            PC2 へ道具を配る（~/teleop_tools/）
#   ./scripts/pc2_teleop.sh setup             配ってから、PC2 に遠隔操作の環境を作る（切り離して流す。初回は 10〜20 分）
#   ./scripts/pc2_teleop.sh setup-log         環境づくりのログを流し見る
#   ./scripts/pc2_teleop.sh check             機体の状態を見る。動かさない。最後の 3 行（✓/✗）が立った形・座位の遠隔操作・座位の再生を始めてよいか
#                                             （キットの「→ 腕を動かせる状態です」はモードを見ないので当てにしない）
#   ./scripts/pc2_teleop.sh probe             腕を 3° だけ動かして、腕を借りられるかを確かめる（確認あり）
#   ./scripts/pc2_teleop.sh sensors <名前> [秒] ライダー・IMU・/dog_odom・リモコン・赤外線を録る（切り離す。既定 1800 秒）
#                                             ⚠️ 遠隔操作より先に起こす。中身は Door/real/pc2/rec_take.sh（PC2 の ~/door_real/）
#   ./scripts/pc2_teleop.sh sensors-stop <名前> センサーの記録を早めに止める（書き切ってから止まる）
#   ./scripts/pc2_teleop.sh start <課題名> [--no-legs | --seated]
#                                             遠隔操作を切り離して起こす（--motion・記録あり・素通し）。純正が通常運転 500/501 でないと断る
#                                             --no-legs で足なし（純正が立たせたまま腕だけ。スティックを倒しても歩かない。修正 no-loco）
#                                             --seated で座位（2026-11-04）: 座らせてダンピング（L2+B）のまま腕だけ。純正を止めて rt/lowcmd
#                                             （脚と腰は起動時の角度で固める）。合間は保持係が最後の姿勢を保つ
#   ./scripts/pc2_teleop.sh go                追従の開始（キーボードの r）
#   ./scripts/pc2_teleop.sh rec               記録の開始／保存（s）。1 回ごとに 2 度押す
#   ./scripts/pc2_teleop.sh check-ep <課題名> [番号]  録った 1 本を点検する（歩きの値・時刻・センサー・指令どおりに進んだ割合・
#                                             扉の角度）。既定は最新。終わりに、その課題で扉が 30° を超えた本数（関門）を出す
#   ./scripts/pc2_teleop.sh push-force <課題名> [番号]  手が扉を押した力と、前へ歩けの区間ごとの押し方（check-ep の後に）
#   ./scripts/pc2_teleop.sh state             状態（追従中・記録中）
#   ./scripts/pc2_teleop.sh log               ログを流し見る（Ctrl+C で抜けても遠隔操作は止まらない）
#   ./scripts/pc2_teleop.sh quit              終える（q）。⚠️ 後始末で腕を下ろす。扉から手を離してから
#   ./scripts/pc2_teleop.sh kill              quit が効かないときだけ。Ctrl+C と同じ（やはり腕を下ろす）
#   ./scripts/pc2_teleop.sh list [課題名]      録った記録の一覧（長さ・足あり／なし・歩いた秒。2026-11-03）
#   ./scripts/pc2_teleop.sh replay <課題名> [番号] [--no-legs | --seated] [--execute]
#                                             再生（2026-11-03）。既定は確かめだけ（機体に何も送らない・軌道の図を PC2 の ~/teleop_logs/ に）
#                                             --execute で実機（y/N の確認・Enter か Ctrl+C で中断）。足ありの記録は歩きも流す。--no-legs で腕だけ
#                                             --seated で座位の再生（腕だけ・rt/lowcmd。座位の遠隔操作を 1 回 quit した後に。終わったら保持係が保つ）
#   ./scripts/pc2_teleop.sh hold              座位の保持係の状態（保っているか・モータの有効・温度）
#   ./scripts/pc2_teleop.sh release           座位を終える: 保持係が力を 2 秒で抜いて止まる（その後はゼロトルク。座らせたまま）
#   ./scripts/pc2_teleop.sh restore           release の後、止めた純正を戻す（確認あり。⚠️ 座ったまま戻したときの振る舞いは未確認）
#   ./scripts/pc2_teleop.sh patch             配ってから、PC2 の ~/xr_teleoperate に修正だけを当て直して自己テストを回す（setup より軽い）
#   ./scripts/pc2_teleop.sh install-omen      Mac から: この道具一式を OMEN の ~/g1_teleop/ へ送る（OMEN からも同じ命令で操作できる）
#
# 接続先は G1_PC2_SSH。既定は ~/.ssh/config で 192.168.0.82 を指す名前（Mac は g1wifi・OMEN は g1-wifi）を探して使う。
# 操作する機械（Mac か OMEN）も Physical_AI_5G につなぐこと。2026-11-03 から OMEN からも操作できる:
#   OMEN で ~/g1_teleop/Teleop/scripts/pc2_teleop.sh start …（送るのは Mac で ./scripts/pc2_teleop.sh install-omen）
# Mac が Physical_AI_5G に居ないときは G1_PC2_JUMP=ubuntu@100.99.102.70（OMEN の Tailscale）で中継できる。
# 止めるときの第一は Quest の両スティック 0.5 秒（一時停止）。この道具の quit は終わらせるときに使う。
set -u
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
KIT="$HERE/../vendor/g1-starter-kit"
REPO="$HERE/../.."
if [ -n "${G1_PC2_SSH:-}" ]; then
  PC2="$G1_PC2_SSH"
else
  PC2=""
  for h in g1wifi g1-wifi; do     # Mac と OMEN で ~/.ssh/config の名前が違う（2026-11-03）
    if [ "$(ssh -G "$h" 2> /dev/null | awk '$1 == "hostname" { print $2 }')" = 192.168.0.82 ]; then PC2="$h"; break; fi
  done
  PC2="${PC2:-unitree@192.168.0.82}"
fi
OMEN="${G1_OMEN_SSH:-ubuntu@100.99.102.70}"   # install-omen の送り先
TELEOP_PAT='^[^ ]*python[^ ]* (-u )?teleop_hand_and_arm.py'   # ssh 越しの pgrep が運び役のシェルに当たらない形（10-27）
SELFTESTS="no_loco seated_controls loco_fwd_max stick_pause record_timestamp go_home arm_home arm_effort state_watchdog replay_legs"
PC2_SELFTESTS="seat_hold"                     # Teleop/scripts/pc2/ の自己テスト（保持係を本物の DDS で。lo・domain 7）
TOOLS="teleop_tools"                          # PC2 の $HOME 相対
PY='$HOME/miniforge3/envs/tv/bin/python'      # PC2 の上で展開させる
SSH=(ssh -o ConnectTimeout=8 -o ServerAliveInterval=5 -o ServerAliveCountMax=3)
SCP=(scp -q -O -o ConnectTimeout=8)           # -O＝古い方式。既定の SFTP は無線で 27 本に 97 秒、-O は 11 秒（2026-11-04 実測）
if [ -n "${G1_PC2_JUMP:-}" ]; then
  SSH+=(-J "$G1_PC2_JUMP"); SCP+=(-J "$G1_PC2_JUMP")
fi

say() { echo "[pc2_teleop] $*"; }
pc2() { "${SSH[@]}" "$PC2" "$@"; }

# PC2 へ配るもの（REPO からの相対）。install-omen は同じものを同じ並びで OMEN へ送るので、OMEN からも deploy できる
DEPLOY_FILES=(
  Teleop/scripts/pc2/teleop_start.sh Teleop/scripts/pc2/teleop_ipc.py Teleop/scripts/pc2/teleop_list.py
  Teleop/scripts/pc2/seat_hold.py Teleop/scripts/pc2/form_check.py
  Teleop/scripts/pc2/setup_pc2_tv.sh Teleop/scripts/pc2/tv_constraints.txt
  Teleop/scripts/pc2/ik_bench.py Teleop/scripts/pc2/teleop_episode_check.py Teleop/scripts/pc2/door_push_force.py
  Teleop/vendor/g1-starter-kit/tools/mode_check.py Teleop/vendor/g1-starter-kit/tools/armsdk_probe.py
  Teleop/vendor/g1-starter-kit/setup/apply_patches.py
  Teleop/vendor/g1-starter-kit/tools/replay_arm.py Teleop/vendor/g1-starter-kit/tools/loco_replay.py
  Door/real/dog_odom_bag.py Door/doorman6/usd_fix/urdf_src/g1_29dof_rev_1_0.urdf
)
for t in $SELFTESTS; do DEPLOY_FILES+=("Teleop/vendor/g1-starter-kit/tools/${t}_selftest.py"); done
for t in $PC2_SELFTESTS; do DEPLOY_FILES+=("Teleop/scripts/pc2/${t}_selftest.py"); done

deploy() {
  pc2 "mkdir -p ~/$TOOLS ~/teleop_logs ~/teleop_data" || exit 1
  local srcs=() f
  for f in "${DEPLOY_FILES[@]}"; do srcs+=("$REPO/$f"); done
  "${SCP[@]}" "${srcs[@]}" "$PC2:$TOOLS/" || exit 1
  pc2 "chmod +x ~/$TOOLS/*.sh ~/$TOOLS/*.py" && say "配った: PC2 の ~/${TOOLS}/"
}

# 課題名と番号から PC2 の記録のフォルダを出す式（番号を省くと最新）。PC2 の上で展開させる
episode_expr() {
  local task="$1" num="${2:-}" sel
  sel=$(if [ -n "$num" ]; then printf 'episode_%04d' "$((10#$num))"; else echo 'episode_*'; fi)
  echo "\$(ls -d ~/teleop_data/$task/$sel 2>/dev/null | tail -1)"
}

cmd="${1:-}"
[ $# -gt 0 ] && shift
case "$cmd" in
  install-omen)
    # 2026-11-03: OMEN から PC2 の遠隔操作を動かすため、この入口と配るもの一式をリポジトリの並びのまま OMEN へ送る。
    # 送り先は OMEN の ~/g1_teleop/（他班の写し ~/G1_Hackason・~/G1_Hackason_fresh には触らない）
    files=(Teleop/scripts/pc2_teleop.sh "${DEPLOY_FILES[@]}")
    (cd "$REPO" && rsync -aR -e "ssh -o ConnectTimeout=8" "${files[@]}" "$OMEN:g1_teleop/") || exit 1
    say "送った: OMEN の ~/g1_teleop/（${#files[@]} 本）。OMEN では ~/g1_teleop/Teleop/scripts/pc2_teleop.sh <命令>"
    ;;
  deploy) deploy ;;
  patch)
    deploy
    # setup（環境づくり）より軽い。~/xr_teleoperate に修正だけを当て直す。当てる前の 3 つを日時つきで控える
    pc2 "set -e; B=~/xr_teleoperate_backups/\$(date +%Y%m%dT%H%M%S); mkdir -p \$B
         cp ~/xr_teleoperate/teleop/teleop_hand_and_arm.py ~/xr_teleoperate/teleop/robot_control/robot_arm.py \
            ~/xr_teleoperate/teleop/televuer/src/televuer/televuer.py \$B/ && echo \"[pc2_teleop] 控え: \$B\"
         cd ~/$TOOLS && $PY apply_patches.py | grep -E '✅|⚠️|❌|完了|変更していません' || true
         $PY apply_patches.py --dry-run | grep -E '✅|⚠️|❌' && { echo '[pc2_teleop] ✗ 当たり切っていない'; exit 1; } || true
         ng=0; for t in $SELFTESTS $PC2_SELFTESTS; do printf '  %-18s ' \$t; nice -n 10 $PY \${t}_selftest.py > /tmp/pc2_selftest_\$t.log 2>&1 \
           && echo 合格 || { echo \"不合格（PC2 の /tmp/pc2_selftest_\$t.log）\"; ng=1; }; done; exit \$ng" ;;
  setup)
    deploy
    pc2 'L=~/teleop_logs/setup_$(date +%Y%m%dT%H%M%S).log; nohup setsid bash ~/teleop_tools/setup_pc2_tv.sh > "$L" 2>&1 < /dev/null & echo "[pc2_teleop] 環境づくりを始めた（PID $!）。ログ: $L"'
    say "進み具合は ./scripts/pc2_teleop.sh setup-log"
    ;;
  setup-log) pc2 'f=$(ls -t ~/teleop_logs/setup_*.log 2>/dev/null | head -1); [ -n "$f" ] && tail -n 40 -F "$f" || echo "ログが無い"' ;;
  check)
    # 2026-11-04: キットの mode_check.py は FSM 0 でも「腕を動かせる状態です」と出す（モータの mode しか見ない）。
    # 11-04 朝はそれで FSM 0 のまま起こし、go しても腕が動かなかった。始めてよいかは form_check.py の ✓/✗ で決める
    pc2 "$PY ~/$TOOLS/mode_check.py --iface eth0
         echo; echo '[pc2_teleop] ↑ キットの判定（モードを見ない）。始めてよいかは下の ✓/✗ で決める:'
         $PY ~/$TOOLS/form_check.py --form both --iface eth0; $PY ~/$TOOLS/seat_hold.py status || true" ;;
  probe) "${SSH[@]}" -t "$PC2" "$PY ~/$TOOLS/armsdk_probe.py --iface eth0" ;;
  start)
    task="${1:?課題名を渡す（例 door_room4_1027）}"; shift
    legs=walk
    case "${1:-}" in
      "") ;;
      --no-legs) legs=none ;;
      --seated) legs=seated ;;
      *) say "知らない引数: $1（足なしは --no-legs・座位は --seated）"; exit 1 ;;
    esac
    [[ "$task" =~ ^[A-Za-z0-9_-]+$ ]] || { say "課題名は英数字と _ - だけ"; exit 1; }
    fwd="${TELEOP_FWD_MAX:-}"                 # 前への歩きの上限 [m/s]（修正 loco-fwd-max）。空なら上流と同じ 0.3
    [[ -z "$fwd" ]] || { [[ "$fwd" =~ ^[0-9.]+$ ]] && awk -v v="$fwd" 'BEGIN { exit !(v + 0 > 0 && v + 0 <= 1.0) }'; } ||
      { say "TELEOP_FWD_MAX は 0 より大きく 1.0 以下の数（例 0.5）"; exit 1; }
    [[ -n "$fwd" && "$legs" != walk ]] && { say "足なし・座位では歩かないので TELEOP_FWD_MAX は使えない"; exit 1; }
    pc2 "TELEOP_LEGS=$legs ${fwd:+TELEOP_FWD_MAX=$fwd }~/$TOOLS/teleop_start.sh $task"
    ;;
  list)
    task="${1:-}"
    [[ -z "$task" || "$task" =~ ^[A-Za-z0-9_-]+$ ]] || { say "課題名は英数字と _ - だけ"; exit 1; }
    pc2 "$PY ~/$TOOLS/teleop_list.py $task" ;;
  replay)
    task="${1:?課題名を渡す}"; shift
    num=""; legs=1; seated=0; execute=0
    for a in "$@"; do
      case "$a" in
        --no-legs) legs=0 ;;
        --seated) seated=1; legs=0 ;;          # 座位は腕だけ（歩きは純正に頼むので、純正を止めた形では流せない）
        --execute) execute=1 ;;
        *) [[ "$a" =~ ^[0-9]+$ ]] && num="$a" || { say "知らない引数: $a"; exit 1; } ;;
      esac
    done
    [[ "$task" =~ ^[A-Za-z0-9_-]+$ ]] || { say "課題名は英数字と _ - だけ"; exit 1; }
    # 歩きのある記録で --no-legs が無ければ歩きも流す。記録が読めなければ（終了コード 2）腕だけと取り違えずに止める
    pre="e=$(episode_expr "$task" "$num"); [ -n \"\$e\" ] || { echo '[pc2_teleop] 記録が無い'; exit 1; }; echo \"[pc2_teleop] 記録: \$e\"
         L=; if [ $legs = 1 ]; then $PY ~/$TOOLS/teleop_list.py --has-walk \"\$e\"; r=\$?
           [ \$r = 0 ] && L=--legs; [ \$r -ge 2 ] && exit 1; fi"
    if [ "$execute" = 0 ]; then
      pc2 "$pre; png=~/teleop_logs/replay_${task}_\$(basename \"\$e\").png
           nice -n 10 $PY ~/$TOOLS/replay_arm.py --episode \"\$e\" \$L --no-plot --save-plot \"\$png\""
    else
      # 2026-11-04: 始めてよい機体の状態かを形ごとに確かめる。座位の再生は純正を止めない（止めてから最初の指令までの
      # 数秒、誰も姿勢を保たないため）。座位の遠隔操作を 1 回起こして quit した後（純正が止まり、保持係が保っている）に流す。
      # 保持係を起こしておくと、終わった後も保持係が最後の姿勢を引き継ぐ
      if [ "$seated" = 1 ]; then
        form="$PY ~/$TOOLS/form_check.py --form seated --for replay --iface eth0 && $PY ~/$TOOLS/seat_hold.py ensure --iface eth0"
        topic="--topic rt/lowcmd"
      else
        form="$PY ~/$TOOLS/form_check.py --form standing --iface eth0"
        topic=""
      fi
      # 遠隔操作と再生が同時に腕を書くと取り合うので、動いていれば断る。y/N と Enter の中断に端末が要る（-t）。
      # ⚠️ 無線が落ちて ssh が切れても PC2 の側はすぐには気づかず、再生は最後まで進む（Enter も届かない）。止めるのはリモコン
      "${SSH[@]}" -t "$PC2" "$pre
        if pgrep -f '$TELEOP_PAT' > /dev/null; then echo '[pc2_teleop] ✗ 遠隔操作が動いている。quit してから'; exit 1; fi
        $form || { echo '[pc2_teleop] ✗ 機体の状態がこの形に合わない（上の ✗）'; exit 1; }
        $PY ~/$TOOLS/replay_arm.py --episode \"\$e\" \$L $topic --execute --network-interface eth0"
    fi
    ;;
  hold)    pc2 "$PY ~/$TOOLS/seat_hold.py status" ;;
  release) pc2 "$PY ~/$TOOLS/seat_hold.py release" ;;
  restore)
    # 2026-11-04: 座位で止めた純正（ai）を戻す。保持係を先に止める（純正と取り合わない）。座ったまま戻したときの振る舞いは未確認
    "${SSH[@]}" -t "$PC2" "if pgrep -f '$TELEOP_PAT' > /dev/null; then echo '[pc2_teleop] ✗ 遠隔操作が動いている。quit してから'; exit 1; fi
      echo '[pc2_teleop] ⚠️ 純正を戻す。座ったまま戻したときに純正がどの状態で始まるかは未確認（脚が動きうるので支えておく）'
      read -r -p '[pc2_teleop] 戻しますか? [y/N] ' ans; [ \"\$ans\" = y ] || { echo '[pc2_teleop] やめた'; exit 1; }
      $PY ~/$TOOLS/seat_hold.py release && $PY ~/$TOOLS/mode_check.py --iface eth0 --restore
      $PY ~/$TOOLS/form_check.py --form both --iface eth0" ;;
  sensors)
    name="${1:?名前を渡す（例 t0_1027_sensors）}"; secs="${2:-1800}"
    [[ "$name" =~ ^[A-Za-z0-9_-]+$ && "$secs" =~ ^[0-9]+$ ]] || { say "名前は英数字と _ -、秒は整数"; exit 1; }
    # 10-29: 「nohup setsid … &」だと ssh が閉じずに待ち続けた（記録は動いていた）。setsid -f で完全に切り離し、ssh の入力も閉じる
    "${SSH[@]}" -n "$PC2" "mkdir -p ~/mapping_runs/logs_1021 && setsid -f nohup ~/door_real/rec_take.sh $name $secs ir > ~/mapping_runs/logs_1021/${name}_take.log 2>&1 < /dev/null; sleep 3; tail -n 3 ~/mapping_runs/logs_1021/${name}_take.log"
    ;;
  sensors-stop)
    name="${1:?名前を渡す}"
    [[ "$name" =~ ^[A-Za-z0-9_-]+$ ]] || { say "名前は英数字と _ - だけ"; exit 1; }
    pc2 "~/door_real/rec_stop.sh $name"
    ;;
  check-ep)
    task="${1:?課題名を渡す}"; num="${2:-}"
    [[ "$task" =~ ^[A-Za-z0-9_-]+$ && ( -z "$num" || "$num" =~ ^[0-9]+$ ) ]] || { say "課題名は英数字と _ -、番号は整数"; exit 1; }
    pc2 "$PY ~/$TOOLS/teleop_episode_check.py --task $task ${num:+--episode $num}"
    ;;
  push-force)
    task="${1:?課題名を渡す}"; num="${2:-}"
    [[ "$task" =~ ^[A-Za-z0-9_-]+$ && ( -z "$num" || "$num" =~ ^[0-9]+$ ) ]] || { say "課題名は英数字と _ -、番号は整数"; exit 1; }
    sel=$(if [ -n "$num" ]; then printf 'episode_%04d' "$((10#$num))"; else echo 'episode_*'; fi)   # 既定は最新
    pc2 "e=\$(ls -d ~/teleop_data/$task/$sel 2>/dev/null | tail -1); [ -n \"\$e\" ] || { echo '[pc2_teleop] 記録が無い'; exit 1; }
         nice -n 10 $PY ~/$TOOLS/door_push_force.py --episode-dir \"\$e\""
    ;;
  go)    pc2 "$PY ~/$TOOLS/teleop_ipc.py start" ;;
  rec)   pc2 "$PY ~/$TOOLS/teleop_ipc.py record" ;;
  quit)  pc2 "$PY ~/$TOOLS/teleop_ipc.py stop" ;;
  state) pc2 "$PY ~/$TOOLS/teleop_ipc.py state; pgrep -af '$TELEOP_PAT' | head -1 | grep . || echo '（遠隔操作は動いていない）'
              $PY ~/$TOOLS/seat_hold.py status 2> /dev/null | head -1 || true" ;;
  log)   pc2 'f=$(ls -t ~/teleop_logs/teleop_*.log 2>/dev/null | head -1); [ -n "$f" ] && tail -n 30 -F "$f" || echo "ログが無い"' ;;
  kill)  pc2 'p=$(cat ~/teleop_logs/teleop.pid 2>/dev/null); [ -n "$p" ] && kill -INT "$p" && echo "SIGINT を送った（PID ${p}）" || echo "PID が分からない"' ;;
  *)     awk 'NR > 1 && /^#/ { print; next } NR > 1 { exit }' "$0"; exit 1 ;;
esac
