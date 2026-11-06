#!/usr/bin/env bash
# **初期姿勢を決める（up.sh の段 0）。** 標準出力に `X Y YAW_DEG` を 1 行だけ出す。
#
#   INIT="$(bash quickstart/live/initial_pose.sh)"
#
# 案内は全部 stderr に出る。決められなければ非ゼロで終わる（黙って間違った種を返さない）。
#
# ## 順番
#
# | 段 | 手段 | 次へ進む条件 |
# |---|---|---|
# | A | **タグ**（`localize_from_tags.py --live`）| タグが 1 枚も読めない／カメラが開けない |
# | B | **その場で 45° ずつ回ってタグを見直す**（最大 7 回 = 315°）| 1 周しても読めない／回せない |
# | C | **LiDAR の指紋**（UpDown-SC → 事前地図へ ICP、Mac で解く）| 「自信なし」なら止まる |
#
# ⚠️ **タグが読めたのに自己検査が落ちたときは、B にも C にも進まない。**
# 落ちるのは機体が傾いているとき（2026-09-17）で、回っても指紋でも直らない。
#
# ### B について
#
# - 回った量は追わない。**見えた時点の姿勢をそのまま種にする**（FAST_LIO はまだ起きていない）
# - 回すのに `loco_driver.py --arm` を**この段の間だけ**起こし、抜けるとき必ず落とす
#   （trap。残すと後の `legs.sh` が 47600 を掴めない。2026-09-17 の罠）
# - FSM が通常運転（500/501）でなければ回さない（立位ロックでは指令が黙って捨てられる）
# - カメラが開けないとき（2026-09-26: RealSense が USB に出なかった）は**回っても無駄**なので飛ばす
#
# ### C について
#
# - PC2 で 3 秒ぶんの点群と IMU を取り（`fingerprint/grab_scan.py`）、Mac で解く
# - 台帳は `~/g1_cfg/map` の事前地図から作る。**地図が変わると作り直す**（md5 で見る。初回 約 3 分）
# - 結果は PNG で開き、**人が yes と答えたときだけ採用する**
#
# ## 環境変数
#   G1_INIT_SPIN=0   B を飛ばす（機体を動かしたくないとき）
#   G1_INIT_FP=0     C を飛ばす
set -uo pipefail
# ⚠️ 標準出力は姿勢の 1 行だけにする。途中の道具が stdout に何か書いても混ざらないよう、
# 元の stdout を fd 3 に退避して、以降の stdout は全部 stderr へ回す
exec 3>&1 1>&2
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
. "$HERE/_common.sh"
QS="$(dirname "$HERE")"
FP="$QS/fingerprint"
REPO="$(cd "$QS/../../.." && pwd)"
VENV="$REPO/Navigation/.venv/bin/python"
UPDOWN_BIN="${G1_UPDOWN_BIN:-$FP/.updown-sc/build/updown-sc}"
SPIN_STEPS=7
SPIN_DEG=45
SETTLE_S=2
FP_YAWS=24        # 台帳の向きの数（15° おき）。置き場の名前に入るので、変えれば作り直される

DRIVER_PY='loco_driv''er.py'      # ⚠️ 分割して書く（pkill の自己一致よけ）

# ── A. タグ ──────────────────────────────────────────────────────────
# 結果は TAG_STATE に入れる: ok / notag / nocamera（それ以外は die）
TAG_INIT=""
TAG_STATE=""
try_tags() {
    TAG_INIT=""
    # ⚠️ 2026-09-25: 地図の座標系をタグの登録と照合する（違えば localize_from_tags.py が止まる）。
    # map_frame.txt は地図と一緒に配る（quickstart/map_frame.py --write が作る）。
    # 無ければ照合せずに警告だけ出る（配布前の構成を壊さないため）
    local out
    out="$(pc2 'cd "$HOME/g1_cfg/apriltag" && MF="$HOME/g1_cfg/map/map_frame.txt" &&
                 timeout 150 python3 localize_from_tags.py \
                 --live --seconds 5 \
                 --registry "$HOME/g1_cfg/apriltag/tag_registry.json" \
                 --extrinsics "$HOME/g1_cfg/apriltag/extrinsics.json" \
                 $( [ -s "$MF" ] && printf -- "--map-frame %s" "$(cat "$MF")" ) 2>&1' || true)"
    printf '%s\n' "$out" | grep -E '^\*\*|\*\*\(x|食い違い|z .*roll|ばらつき|座標系|map_frame|1 枚も|開けない' \
        | sed 's/^/     /' >&2
    case "$out" in
        *"[NG] 座標系が違う"*)
            die "タグの登録と地図の座標系が違う（上の行）。~/g1_cfg/map を戻すか、タグを登録し直す" ;;
        *"を開けない"*)
            TAG_STATE=nocamera; return ;;
        *"1 枚も読めなかった"*)
            TAG_STATE=notag; return ;;
    esac
    # ⚠️ **文字クラスに `+` を入れる。**localize_from_tags.py は符号を必ず付けて
    # `**(x -0.139, y +0.184, yaw +11.98 deg)**` と出す。`[-0-9.]` だと `+0.184` を
    # 拾えず、**タグ測位が成功しているのに「取れなかった」と die する**
    # （2026-09-17 に踏んだ。x が負だったので余計に気づきにくかった）
    TAG_INIT="$(printf '%s\n' "$out" | sed -n 's/.*\*\*(x *\([-+0-9.]*\), *y *\([-+0-9.]*\), *yaw *\([-+0-9.]*\) deg)\*\*.*/\1 \2 \3/p' | head -1)"
    [ -n "$TAG_INIT" ] || { printf '%s\n' "$out" | tail -8 | sed 's/^/     /' >&2
                            die "タグの出力が読めなかった（上の行）。--init \"X Y YAW\" で渡すこと"; }
    # ⚠️ 自己検査が落ちていたら採用しない（誤った種は FAST_LIO では復帰しない）
    #
    # ⚠️⚠️ **どの検査が落ちたかで手当てが正反対になる。**
    # 2026-09-17 に、ここの旧い案内（「貼り直すか近づく」）に従って 1 時間近く失った。
    # 落ちていたのは **z/roll/pitch（水平）だけ**で、タグ間の食い違い 71 mm も
    # 枚ごとのばらつき 2 mm も合格していた。原因は機体が立位ロック（FSM 802）で
    # 前傾していたことで、**タグは一切関係なかった**。
    #
    # ⚠️ `case` で見る（`printf | grep -q` にしない）。pipefail 下では grep -q が
    # 先に抜けて printf が SIGPIPE で落ち、**一致しているのに非ゼロが返りうる**。
    case "$out" in
        *"⛔ z "*)
            bad "水平の検査だけが落ちた（z / roll / pitch）"
            echo "     ⚠️ **タグは正常。貼り直しても近づいても直らない。**機体側が傾いている" >&2
            echo "     1. 通常運転（FSM 500/501）に入っているか。立位ロックのままなら" >&2
            echo "        リモコンで R1 + X（⑦ 通常運転）を押してから やり直す" >&2
            echo "        ＝ preflight.sh の 6. が見る項目（2026-09-17 はここが 802 だった）" >&2
            echo "     2. それでも落ちるなら base_link の取付（LIVOX_RPY）を疑う" >&2
            echo "     ⛔ **measure_extrinsics.py で測り直して extrinsics.json を書き換えてはいけない。**" >&2
            echo "        あれは「base_link は床と水平」を前提に床タグから測るので、機体の傾きが" >&2
            echo "        カメラ取付へ吸収され、**検査は通るのに姿勢は狂ったまま**になる（循環）。" >&2
            echo "        2026-09-16 と 2026-09-17 に 2 回踏んだ。決着は床平面フィットでつける" >&2
            die "機体の姿勢を直してから もう一度"
            ;;
        *"⛔"*) die "タグの自己検査が落ちている（タグ間の食い違い）。貼り直すか近づく" ;;
    esac
    TAG_STATE=ok
}

# ── B. その場で回って見直す ──────────────────────────────────────────
DRIVER_UP=0
stop_driver() {
    [ "$DRIVER_UP" = 1 ] || return 0
    # SDK 側は INT で StopMove を通す。死ななければ pc2_kill_procs が KILL まで面倒を見る
    pc2_kill_procs 'loco_driver' INT
    DRIVER_UP=0
    note "足のドライバを落とした（後の legs.sh が 47600 を使えるように）"
}
trap stop_driver EXIT INT TERM

start_driver() {
    local held
    held="$(pc2 "ss -lun 2>/dev/null | grep -c 47600 || true" 2>/dev/null | tr -d '[:space:]')"
    if [ "${held:-0}" != "0" ]; then
        warn "127.0.0.1:47600 が既に埋まっている（legs.sh か古い ${DRIVER_PY}）。回さない"
        return 1
    fi
    pc2 "nohup setsid python3 \$HOME/nav_tools/$DRIVER_PY \
           --network-interface eth0 --arm > \$HOME/g1_runs/loco_spin.log 2>&1 < /dev/null &"
    DRIVER_UP=1
    pc2_wait "足のドライバの起動" 20 'grep -qE "FSM|Traceback|Address already" $HOME/g1_runs/loco_spin.log' \
        || { warn "ドライバが起動を報告しない。回さない"; return 1; }
    pc2 "grep -E 'FSM|Traceback|Address' \$HOME/g1_runs/loco_spin.log | head -3 | sed 's/^/     /'" >&2
    # ⚠️ 「歩ける」と言わない限り回さない（立位ロックでは指令が黙って捨てられる）
    pc2 "grep -q '通常運転。歩ける' \$HOME/g1_runs/loco_spin.log" \
        || { warn "FSM が通常運転でない。回さない（リモコンで R1 + X）"; return 1; }
}

spin_search() {
    if [ "${G1_INIT_SPIN:-1}" = "0" ]; then
        note "G1_INIT_SPIN=0 なので回らない"; return 1
    fi
    say "B. タグが見えない。その場で ${SPIN_DEG}° ずつ回って探す（最大 ${SPIN_STEPS} 回）"
    printf '  \033[31m⚠️  これから機体がその場で回ります。\033[0m\n' >&2
    printf '  リモコンで止められる人が横に居ますか？ [yes と入力] ' >&2
    local ans
    read -r ans < /dev/tty   # ⚠️ stdin は途中の ssh に食われるので端末から直に読む
    [ "$ans" = "yes" ] || { note "回らない（yes でなかった）"; return 1; }
    pc2 "mkdir -p \$HOME/g1_cfg/fingerprint \$HOME/g1_runs"
    scp -q -i "$G1_KEY" "$FP/spin_step.py" "$G1_PC2_USER@$G1_PC2_HOST:g1_cfg/fingerprint/" \
        || { warn "spin_step.py を送れない"; return 1; }
    start_driver || return 1
    local i
    for i in $(seq 1 "$SPIN_STEPS"); do
        say "   ${i}/${SPIN_STEPS}: ${SPIN_DEG}° 回る"
        pc2 "python3 \$HOME/g1_cfg/fingerprint/spin_step.py --deg $SPIN_DEG" | sed 's/^/     /' >&2
        sleep "$SETTLE_S"      # 止まってから撮る（歩行の揺れを入れない）
        try_tags
        if [ "$TAG_STATE" = ok ]; then
            stop_driver
            return 0
        fi
        [ "$TAG_STATE" = nocamera ] && { warn "途中でカメラが開けなくなった"; break; }
    done
    stop_driver
    return 1
}

# ── C. LiDAR の指紋 ──────────────────────────────────────────────────
FP_INIT=""
fingerprint_search() {
    if [ "${G1_INIT_FP:-1}" = "0" ]; then
        note "G1_INIT_FP=0 なので指紋は使わない"; return 1
    fi
    say "C. LiDAR の指紋で探す（UpDown-SC → 事前地図へ ICP）"
    [ -x "$UPDOWN_BIN/scan_context_rebuild" ] \
        || { bad "UpDown-SC が無い → bash quickstart/fingerprint/setup.sh"; return 1; }
    [ -x "$VENV" ] || { bad "Python が無い: $VENV"; return 1; }

    local stamp work cache sum
    stamp="$(date +%Y%m%dT%H%M%S)"
    work="$QS/../runs/fp_$stamp"
    mkdir -p "$work"

    # 1) PC2 で点群と IMU を取る（jammy の python3.10。姿勢ロガーと同じ起こし方）
    pc2 "mkdir -p \$HOME/g1_cfg/fingerprint \$HOME/g1_runs"
    scp -q -i "$G1_KEY" "$FP/grab_scan.py" "$G1_PC2_USER@$G1_PC2_HOST:g1_cfg/fingerprint/" \
        || { bad "grab_scan.py を送れない"; return 1; }
    pc2 "P=\"\$HOME/jammy_ros/rootfs\"
         . \"\$HOME/jammy_ros/env.sh\" >/dev/null 2>&1
         export ROS_DOMAIN_ID=0
         export CYCLONEDDS_URI='$G1_DDS_ETH0'
         timeout 40 env \$JAMMY_ENV \"\$LOADER\" --library-path \"\$JAMMY_LIBS\" \
           \"\$P/usr/bin/python3.10\" \$HOME/g1_cfg/fingerprint/grab_scan.py \
           --seconds 3 --out \$HOME/g1_runs/fp_$stamp.npz" 2>&1 | tail -2 | sed 's/^/     /' >&2
    scp -q -i "$G1_KEY" "$G1_PC2_USER@$G1_PC2_HOST:g1_runs/fp_$stamp.npz" "$work/query.npz" \
        || { bad "点群が取れなかった（上の行。LiDAR が出ているか）"; return 1; }

    # 2) 走る地図を取ってくる。台帳は地図の md5 ごとに作り置く
    local f
    for f in map_octomap_r4_s5_floor0.pcd nav_map_run.pgm nav_map_run.yaml; do
        scp -q -i "$G1_KEY" "$G1_PC2_USER@$G1_PC2_HOST:g1_cfg/map/$f" "$work/$f" \
            || { bad "~/g1_cfg/map/$f が取れない"; return 1; }
    done
    sum="$(md5 -q "$work/map_octomap_r4_s5_floor0.pcd")"
    cache="$FP/cache/$sum"
    if [ ! -f "$cache/db_session_y$FP_YAWS/build_meta.json" ]; then
        say "   台帳が無い（この地図は初めて）。作る（約 9 分）"
        mkdir -p "$cache"
        "$VENV" "$FP/build_db.py" --map-pcd "$work/map_octomap_r4_s5_floor0.pcd" \
            --nav-map "$work/nav_map_run.yaml" --ray-template "$work/query.npz" \
            --out "$cache/db_session_y$FP_YAWS" --yaws "$FP_YAWS" 2>&1 | grep -v Warning | tail -2 | sed 's/^/     /' >&2
        [ -f "$cache/db_session_y$FP_YAWS/build_meta.json" ] || { bad "台帳が作れなかった"; return 1; }
    fi

    # 3) 解く
    "$VENV" "$FP/localize.py" --query "$work/query.npz" --db "$cache/db_session_y$FP_YAWS" \
        --map-pcd "$work/map_octomap_r4_s5_floor0.pcd" --nav-map "$work/nav_map_run.yaml" \
        --updown-bin "$UPDOWN_BIN" --out "$work" 2>&1 \
        | grep -E '^\[fp\]|^  #' | sed 's/^/     /' >&2
    [ -f "$work/result.json" ] || { bad "指紋の推定が落ちた（${work}）"; return 1; }
    local verdict
    verdict="$("$VENV" -c 'import json,sys; v=json.load(open(sys.argv[1]))["verdict"]; b=v["best"]
print("%d %.3f %.3f %.2f" % (v["confident"], b["base_x"], b["base_y"], b["base_yaw_deg"]))' \
        "$work/result.json")"
    open "$work/result.png" 2>/dev/null || true
    note "図: $work/result.png"
    if [ "${verdict%% *}" != "1" ]; then
        bad "指紋は「自信なし」と言った（図の上の行）。採用しない"
        return 1
    fi
    printf '  指紋の推定: \033[1m%s\033[0m （x y yaw）\n' "${verdict#* }" >&2
    printf '  図で赤いスキャンが黒い地図に乗っていますか？ この姿勢で起こす [yes と入力] ' >&2
    local ans
    read -r ans < /dev/tty   # ⚠️ stdin は途中の ssh に食われるので端末から直に読む
    [ "$ans" = "yes" ] || { note "採用しない（yes でなかった）"; return 1; }
    FP_INIT="${verdict#* }"
}

# ── 本体 ────────────────────────────────────────────────────────────
say "0. 初期姿勢を決める（A タグ → B 回って見直す → C LiDAR の指紋）"
# ⚠️ 映像の配信（live/camera.sh）が IR カメラを握っているとタグが読めない（「開けない」になり B も飛ぶ）
pc2 "bash \$HOME/g1_cfg/apriltag/serve_view.sh stop" >/dev/null 2>&1 || true
say "A. タグ"
try_tags
if [ "$TAG_STATE" = ok ]; then
    ok "タグ由来の初期姿勢: $TAG_INIT"
    echo "$TAG_INIT" >&3; exit 0
fi
if [ "$TAG_STATE" = nocamera ]; then
    warn "カメラ（/dev/video2）が開けない。回っても無駄なので B を飛ばす"
    note "PC2 の lsusb に RealSense が出ているかを見る（2026-09-26 は出ていなかった）"
elif spin_search; then
    ok "回った先でタグが見えた: $TAG_INIT"
    echo "$TAG_INIT" >&3; exit 0
fi
if fingerprint_search; then
    ok "指紋由来の初期姿勢: $FP_INIT"
    echo "$FP_INIT" >&3; exit 0
fi
die "初期姿勢が決められなかった。--init \"X Y YAW\" で渡すこと（RViz2 で地図を見て決める）"
