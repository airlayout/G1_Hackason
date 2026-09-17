#!/usr/bin/env bash
# `live/` の共通部。**2026-09-16 に実機で踏んだ罠を全部ここに閉じ込めてある。**
#
# 読み込むだけ。単体では何もしない:
#   . "$(dirname "${BASH_SOURCE[0]}")/_common.sh"

# ── 宛先 ───────────────────────────────────────────────────────────
# ⚠️ **既定を有線にした（2026-09-17）。**PC2 を 192.168.123.0/24 に有線で繋ぐと
# コンテナ（col0 = 192.168.123.201）から機体の DDS が全部見え、foxglove 中継も
# AP も要らない。無線でやるときだけ G1_PC2_HOST=10.42.0.76 を渡す。
G1_PC2_WIRED="${G1_PC2_WIRED:-192.168.123.164}"   # PC2（有線）
G1_PC2_WIRELESS="${G1_PC2_WIRELESS:-10.42.0.76}"  # PC2（AP 経由）
G1_PC2_HOST="${G1_PC2_HOST:-$G1_PC2_WIRED}"
G1_AP_HOST="${G1_AP_HOST:-192.168.123.200}"       # AP を出している Ubuntu（OMEN）
G1_AP_USER="${G1_AP_USER:-ubuntu}"
G1_PC2_USER="${G1_PC2_USER:-unitree}"
G1_KEY="${G1_KEY:-$HOME/.ssh/id_ed25519_g1}"      # ⚠️ ~/.ssh/config の Host g1 は有線向き
G1_VM_IP="${G1_VM_IP:-192.168.123.201}"           # コンテナ（col0）
G1_CONTAINER="${G1_CONTAINER:-rviz}"

# ⚠️ **PC2 の DDS は必ず eth0 に固定する。**
# 未設定だと CycloneDDS が自動選択で **wlan0 を掴み**、PC1 が出す LiDAR も
# Nav2 の /cmd_vel も見えなくなる（2026-09-16 実測。cmd_vel_bridge がこれで
# 足に繋がっていなかった）。2 NIC にしても直らない（live/README.md の表）。
G1_DDS_ETH0='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="eth0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'
# コンテナ側は col0。
G1_DDS_COL0='<CycloneDDS><Domain><General><Interfaces><NetworkInterface name="col0" priority="default" multicast="default"/></Interfaces></General></Domain></CycloneDDS>'

# ⚠️ **記録は /tmp に置かない。** 2026-09-16 に電源が落ちて歩行 2 回分を失った。
G1_RUN_DIR="${G1_RUN_DIR:-\$HOME/g1_runs}"        # PC2 側で展開させるので \$ をエスケープ

say()  { printf '[live] %s\n' "$*" >&2; }
ok()   { printf '  \033[32mOK\033[0m   %s\n' "$*" >&2; }
warn() { printf '  \033[33mWARN\033[0m %s\n' "$*" >&2; }
bad()  { printf '  \033[31mNG\033[0m   %s\n' "$*" >&2; }
die()  { printf '[live] ⛔ %s\n' "$*" >&2; exit 1; }
note() { printf '  --   %s\n' "$*" >&2; }

# ── PC2 でコマンドを走らせる ────────────────────────────────────────
pc2() { ssh -i "$G1_KEY" -o BatchMode=yes -o ConnectTimeout=15 \
            -o IdentitiesOnly=yes "$G1_PC2_USER@$G1_PC2_HOST" "$@"; }
ap()  { ssh -o BatchMode=yes -o ConnectTimeout=10 "$G1_AP_USER@$G1_AP_HOST" "$@"; }

# ── PC2 のプロセスを数える／殺す ────────────────────────────────────
# ⚠️⚠️ **PC2 側で grep してはいけない。必ずこの 4 つを通す。**
# 2026-09-17 の 1 セッションで、同じ型の誤診を **4 回**繰り返した:
#
# 1. `pc2 'ps -eo args | grep -cE "..."`  → ssh が送った `bash -c` の**コマンド行に
#    パターンが載る**ので自分に一致する。「測位が 1 個動いている」と誤報（実際は 0）
# 2. `grep -c ... || echo 0` → `grep -c` は 0 件でも「0」を印字した上で**終了コード 1**
#    を返すので、数字が 2 個出て `"00"` になる。`!= "0"` が必ず真 ＝ ガードが常に発火
# 3. `comm` は 15 文字で切れる。しかも PC2 の ROS ノードは jammy のローダ経由なので
#    **全部 `ld-linux-aarch6`** になり、comm では種類を見分けられない
# 4. 「残り 2 個」と出たので二重起動を疑って掃除したが、実体は 1 個だった（1 と同じ）
#
# ⇒ **PC2 では `ps` を素で出し、絞り込みは Mac 側でやる。**これで 1 と 3 が原理的に
#    起きない（PC2 のコマンド行はいつも `ps -eo ...` だけ）。数えるのは `wc -l` で
#    行い、`grep -c` を使わない（2 が起きない）。

# $1 = Mac 側で当てる拡張正規表現（省略で全件）
pc2_procs() {
    pc2 'ps -eo pid,etime,args --no-headers' 2>/dev/null \
      | grep -E "${1:-.}" | grep -v 'ps -eo pid,etime,args'
}
# ⚠️ `wc -l` を使う（`grep -c` は 0 件で終了コード 1 を返す）
pc2_count() { pc2_procs "$1" | wc -l | tr -d '[:space:]'; }
pc2_pids()  { pc2_procs "$1" | awk '{print $1}'; }

# $1=正規表現 / $2=最初に送るシグナル（既定 TERM）。死ななければ KILL まで面倒を見る。
# ⚠️ **`--dry-run` の loco_driver は SIGINT を無視する**（2026-09-17 に 19 分残し、
# 次の `--arm` が `Address already in use` で失敗した）。確かめずに終わらせない。
pc2_kill_procs() {
    local pattern="$1" sig="${2:-TERM}" pids left
    pids="$(pc2_pids "$pattern" | tr '\n' ' ')"
    [ -n "${pids// /}" ] || { note "止めるものが無い: $pattern"; return 0; }
    pc2 "kill -$sig $pids 2>/dev/null; true"
    for _ in 1 2 3 4 5; do
        [ "$(pc2_count "$pattern")" = "0" ] && break
        sleep 1
    done
    left="$(pc2_pids "$pattern" | tr '\n' ' ')"
    if [ -n "${left// /}" ]; then
        warn "$sig で死ななかった（$left）。KILL する"
        pc2 "kill -KILL $left 2>/dev/null; true"
        sleep 1
    fi
    note "残り $(pc2_count "$pattern") 個: $pattern"
}

# PC2 で ROS 環境を整えて走らせる。
# ⚠️ `jros2` は**シェル関数**なので `timeout jros2 ...` は動かない
#    （timeout は実行ファイルしか起動できない。2026-09-16 に踏んだ）。
# ⚠️⚠️ **env.sh を先に source してから上書きする。**順序が逆だと env.sh の既定
# （`ROS_DOMAIN_ID=42` と `CYCLONEDDS_URI=lo` ＝ 再生用の隔離）が eth0 の設定を
# 上書きしてしまい、**機体の DDS が 1 件も見えない**。
# 症状は「topic does not appear to be published yet」で、センサが正常でも
# 「来ていない」と出る（2026-09-17 に実機で誤診した。手で測ると 200 Hz 出ていた）。
pc2_ros() {
    pc2 ". \"\$HOME/jammy_ros/env.sh\" >/dev/null 2>&1
         export ROS_DOMAIN_ID=0
         export CYCLONEDDS_URI='$G1_DDS_ETH0'
         $*"
}

# PC2 の ros2 を **timeout に渡せる形**で走らせる。
# ⚠️ `jros2` は env.sh が定義する**シェル関数**なので `timeout jros2 ...` は
# `command not found` になり、**出力が空 ＝「来ていない」と誤診する**
# （2026-09-17 に「センサが全部来ていない」と誤報した。実際は 10/200/1037 Hz）。
# ローダの実体を直に叩けば timeout で囲める。
pc2_ros2() {                    # $1=秒 以降が ros2 の引数
    local secs="$1"; shift
    pc2_ros "timeout $secs env \$JAMMY_ENV \"\$LOADER\" --library-path \"\$JAMMY_LIBS\" \
             \"\$PREFIX/usr/bin/python3.10\" \"\$ROS/bin/ros2\" $*"
}

# コンテナでコマンドを走らせる。
# ⚠️ コンテナの /bin/sh は **dash** なので `/dev/tcp` が使えない。必ず bash。
ctr() { docker exec -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI="$G1_DDS_COL0" \
              "$G1_CONTAINER" bash -c "$*"; }
ctr_bg() { docker exec -d -e ROS_DOMAIN_ID=0 -e CYCLONEDDS_URI="$G1_DDS_COL0" \
                 "$G1_CONTAINER" bash -c "$*"; }

# ── TCP 疎通（ping は入っていない環境がある）──────────────────────
# ⚠️ **このスクリプトを zsh で走らせない。** zsh に `/dev/tcp` は無い。
#    `#!/usr/bin/env bash` を守ること（2026-09-16 に誤判定した）。
tcp_ok() { (exec 3<>"/dev/tcp/$1/$2") 2>/dev/null && { exec 3<&-; return 0; }; return 1; }

# ── プロセスを名前で止める（自分を巻き添えにしない）──────────────
# ⚠️ `pkill -f foo` も `pkill -f fo[o]` も、**呼び出し側のコマンド行に foo が
#    含まれていると自分を殺す**（2026-09-16 に 4 回踏んだ。ssh が落ちて
#    「止まったのか分からない」状態になる）。
#    文字列を分割して組み立て、PID を取ってから kill する。
pc2_kill() {  # pc2_kill <前半> <後半> [シグナル]
    local sig="${3:-TERM}"
    pc2 "P=\$(ps -eo pid,args | grep -F \"\$(printf '%s%s' '$1' '$2')\" \
              | grep -v ' grep ' | awk '{print \$1}')
         for p in \$P; do kill -$sig \"\$p\" 2>/dev/null; done
         sleep 2
         N=\$(ps -eo args | grep -cF \"\$(printf '%s%s' '$1' '$2')\")
         echo \"  残り \$((N-1)) 個\""
}
ctr_kill() { docker exec "$G1_CONTAINER" bash -c \
    "P=\$(ps -eo pid,args | grep -F \"\$(printf '%s%s' '$1' '$2')\" \
         | grep -v ' grep ' | awk '{print \$1}')
     for p in \$P; do kill -9 \"\$p\" 2>/dev/null; done; sleep 1"; }
