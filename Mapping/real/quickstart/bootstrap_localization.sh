#!/usr/bin/env bash
# **起動のたびに「大域 → 局所」の順で測位する。**
#
#   bash quickstart/bootstrap_localization.sh                    # live（LiDAR から 1 スキャン録る）
#   bash quickstart/bootstrap_localization.sh --bag runs/X/bag   # 記録から（再生・試験）
#   bash quickstart/bootstrap_localization.sh --scan a.xyz       # 既に出してある XYZ から
#   bash quickstart/bootstrap_localization.sh --calibrate a.xyz X Y YAW   # 基準 r0 を作る
#
# ## なぜ要るのか（2026-09-13 の実機で起きたこと）
#
# `nav_stack.sh live` は地図の datum を `InitLocalization::FixedPose` に焼いて MOLA を起こす。
# 機体がそこに居なければ ICP は引き込もうとして**偽の極大に落ちる**。09-13 はこれで
# **真値から 2.26 m・16.4° 外した場所**で 1 時間以上走り続けた。しかも:
#
#   - `preflight.sh` は**全項目 OK**で「歩かせてよい」と答えた
#   - `pose_quality` は**誤った姿勢の方が高い**（0.855 対 0.818）
#   - 初期姿勢を 0.9 m ずらして起こし直しても**同じ極大に戻る**
#     （`/relocalize_near_pose` は探索をしない。渡された姿勢を置くだけ）
#
# **落ちたことに気づく手段（重畳）は記録 40〜60 秒＋解析が要るのに、探索は数十秒で済む。**
# 確かめるより探す方が安い。だから毎回通す。
#
# ## 二重のゲート（片方だけでは足りない）
#
# | ゲート | 見る地図 | 落ちたときの意味 |
# |---|---|---|
# | A 残差 r/r0 | **3D** `map.mm` | 部屋の中だが合っていない / 地図の外 |
# | B 重畳 | **2D** `nav_map_ref` | A が通っても場所が違う |
#
# ⚠️ **A だけでは誤採用する。** 尤度の面は平らで、09-13 は上位 8 件の差が **0.7 %** しか
# なかった。2D に当てて初めて 28.8 / 50.5 / **73.1 %** と分かれ、そこで確定した。
# ⚠️ **B だけでも足りない。** 重畳は薄い地図の場所で下がる（09-13 の 4 本目は
# ずれ 0.00 m でも 46.8 %）。**2 つとも通ったときだけ採用する。**
#
# ## 基準 r0 について（ここが設計の穴だった）
#
# ゲート A の基準 `r0` は「信頼できる姿勢」で測るものだが、**起動時にはそれが未知**である
# （それを探しているのだから）。そこで **地図ごとに 1 回だけ較正してファイルに置く**。
#
#   bash quickstart/bootstrap_localization.sh --calibrate <scan.xyz> <X> <Y> <YAW_DEG>
#   → <map.mm と同じディレクトリ>/reloc_r0.txt
#
# ⚠️ **r0 が無ければ起動しない。** 適当な既定値を焼くと、地図を替えたときに
# 黙って緩いゲートで通る。**「分からないなら止まる」が正しい。**
#
# ## 終了コード
#
#   0  採用。`G1_MOLA_INIT_POSE=...` を標準出力に出す
#   2  使い方・前提の誤り（地図が無い、r0 が無い、コンテナが無い…）
#   3  **ゲート A が落ちた。MOLA を起こしてはいけない**
#   4  **ゲート B が落ちた。MOLA を起こしてはいけない**
set -uo pipefail

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=_common.sh
. "$HERE/_common.sh"
G1_TAG=boot

NAME="$G1_RVIZ_NAME"
SESSION="${G1_SESSION:-20260906T135940_UiS_room_v3}"
RUNS_C="/work/G1_Hackason/Mapping/real/runs"
RUNS_L="$G1_REPO_ROOT/G1_Hackason/Mapping/real/runs"
VENV="$G1_REPO_ROOT/G1_Hackason/Navigation/.venv/bin/python"
PROBE="/tmp/reloc_build/bin/reloc_probe"

MOLA_MAP_C="${G1_MOLA_MAP:-$RUNS_C/$SESSION/mola_floor0/map.mm}"
MOLA_MAP_L="${MOLA_MAP_C/#\/work/$G1_REPO_ROOT}"
REF_MAP_L="${G1_OVERLAY_REF_MAP:-$RUNS_L/$SESSION/map/nav_map_ref.yaml}"
R0_FILE="$(dirname "$MOLA_MAP_L")/reloc_r0.txt"

# 探索の範囲。⚠️⚠️ **±5 m にしてはいけない（2026-09-14 実測）。**
# 計画 §2.1 は「±3 m にせず ±5 m から始めよ」と書いていたが、**実測は逆だった**:
#
# | ROI | 所要 | 09-13 採用姿勢との差 | r/r0 | 一致率 | 判定 |
# |---|---|---|---|---|---|
# | ±2.0 m | 9.8 s | 0.560 m / -5.0 deg | 1.13x | 100% | 採用 |
# | ±3.0 m | 20.4 s | 0.560 m / -5.0 deg | 1.13x | 100% | 採用 |
# | ±4.0 m | 33.8 s | 0.560 m / -5.0 deg | 1.13x | 100% | 採用 |
# | **±5.0 m** | 50.3 s | **6.99 m / 180 deg** | 0.83x | **48%** | **自信なし** |
#
# ±5 m にすると探索が **ROI の縁ちょうど**（x = 中心 -5 m）の**地図の外**を 1 位にする。
# 原因は**尤度が「当たった点の数」で正規化されていない**こと —— 地図の外では
# 対応がつく点が少ない（一致率 48%）が、その少数がよく合えば平均は高く出る。
# ⚠️ 上位 80 件が全部 ROI の縁に固まり、真値は 1 件も入らなかった
# （`--candidates 500` と指定しても内部の保持は 80 件が上限）。
# ⇒ **ROI は部屋の中に収める。** ±2〜4 m はどれも同じ答えに収束したので中を取る。
# ✅ なお ±5 m でもゲートは正しく拒否した（fail closed は効いている）。
ROI="${G1_BOOT_ROI:-3.0}"
# ⚠️⚠️ **刻みは細かくしない（2026-09-14 実測）。細かい方が遅くて悪い。**
# 同じスキャン・同じ中心（datum）・ROI ±3 m で:
#
# | 刻み | 所要 | 09-13 採用姿勢との差 | r/r0 |
# |---|---|---|---|
# | 0.25 m / 5 deg | **20.4 s** | **0.560 m** | 1.13x |
# | **0.5 m / 15 deg** | **2.0 s** | **0.021 m** | **1.01x** |
#
# 尤度の面がほぼ平ら（上位 8 件の差 0.7%）なので、細かくすると argmax が
# 隣のセルの間で揺れるだけで良くならない。reloc/README.md の 09-11 の掃引
# （0.25 m/10 deg で 0.250 m 悪化）と同じ現象を、局所探索でも踏んだ。
# **粗く捕まえて、`--relocalize` の内部の詰めに任せる**のが正しい。
RES_XY="${G1_BOOT_RES_XY:-0.5}"
RES_PHI="${G1_BOOT_RES_PHI:-15}"
# 尤度の 1 位を鵜呑みにせず、上位を帯の残差で選び直して地図の外を落とす
CANDIDATES="${G1_BOOT_CANDIDATES:-80}"
# 探索の中心。既定は地図の datum（= nav_stack.sh が焼いている FixedPose の xy）
CENTER_X="${G1_BOOT_CENTER_X:--0.0168}"
CENTER_Y="${G1_BOOT_CENTER_Y:-0.0264}"
# ゲート
GATE_N="${G1_BOOT_GATE_N:-1.5}"        # 2.0 ではない。**誤採用の方が危険**（reloc/README.md）
MIN_MATCH="${G1_BOOT_MIN_MATCH:-0.50}"
BAND_LO="${G1_BOOT_BAND_LO:-1.3}"
# ⚠️ **これは「詰める前の粗い姿勢」に当てる線であって、計画 §2.1 の合否 70% とは別物。**
# 合否 70% は MOLA が収束した**後**に record_still + measure_overlay で測るもの。
# 2026-09-14 の実測（同じスキャン・同じ基準地図 nav_map_ref）:
#   探索の答え (0.983,-0.724,0.0)      **68.3%**   ← 70 にすると**正解を弾く**
#   09-13 の採用 (0.831,-0.203,2.50)   **71.3%**
#   MOLA の偽の極大 (3.106,0.072,16.36) **28.8%**
#   地図の外 (-4.792,-4.349,182.5)      **8.0%**
# 50 なら正しい側に +37%・誤りの側に -42% の余裕がある。
OVERLAY_MIN="${G1_BOOT_OVERLAY_MIN:-50}"
# 広域（タイル）探索。粗い格子で部屋を敷き詰め、**タイルをまたぐ比較は
# 尤度ではなく帯の残差と一致率で行う**（尤度は当たった点の数で正規化されていない）。
# 2026-09-14 実測: 1 枚 1.85 s・36 枚で 67 s。しかも粗い方が真値に近い
#   （粗 0.5 m/15 deg -> 差 0.084 m  対  細 0.25 m/5 deg -> 差 0.560 m）
WIDE_RES_XY="${G1_BOOT_WIDE_RES_XY:-0.5}"
WIDE_RES_PHI="${G1_BOOT_WIDE_RES_PHI:-15}"
WIDE_STEP="${G1_BOOT_WIDE_STEP:-5.0}"
WIDE_MIN_OCC="${G1_BOOT_WIDE_MIN_OCC:-200}"
# 広域で当たった所を詰めるときの範囲
NARROW_ROI="${G1_BOOT_NARROW_ROI:-1.0}"

# live で 1 スキャン録る秒数
SECS="${G1_BOOT_SECONDS:-4}"

say() { echo "[boot] $*"; }

usage() { sed -n '2,8p' "$0" | sed 's/^# \{0,1\}//'; exit 2; }

MODE=bootstrap
WIDE="${G1_BOOT_WIDE:-0}"
# 局所で落ちたら広域に落とす。⚠️ **黙って諦めるより、広く探して落ちる方が良い**
AUTOWIDE="${G1_BOOT_AUTOWIDE:-1}"
SCAN_L=""; BAG_L=""; IDX=""
CAL_X=""; CAL_Y=""; CAL_YAW=""
while [ $# -gt 0 ]; do
    case "$1" in
        --scan)  SCAN_L="$2"; shift 2 ;;
        --bag)   BAG_L="$2"; shift 2 ;;
        --index) IDX="$2"; shift 2 ;;
        --calibrate)
            MODE=calibrate; SCAN_L="$2"; CAL_X="$3"; CAL_Y="$4"; CAL_YAW="$5"; shift 5 ;;
        --wide) WIDE=1; shift ;;
        --no-autowide) AUTOWIDE=0; shift ;;
        live) shift ;;
        -h|--help) usage ;;
        *) echo "[boot] 知らない引数: $1" >&2; usage ;;
    esac
done

# ── 前提の検査（黙って進まない）─────────────────────────────────────
[ -f "$MOLA_MAP_L" ] || { echo "[boot] 3D 地図が無い: $MOLA_MAP_L" >&2; exit 2; }
[ -f "$REF_MAP_L" ]  || { echo "[boot] 2D の基準地図が無い: $REF_MAP_L" >&2; exit 2; }
[ -x "$VENV" ]       || { echo "[boot] venv が無い: $VENV" >&2; exit 2; }
docker inspect "$NAME" >/dev/null 2>&1 || { echo "[boot] コンテナ $NAME が居ない" >&2; exit 2; }

# reloc_probe は手で 1 回建てる運用だった（計画 §2.1-a）。無ければここで建てる
if ! docker exec "$NAME" test -x "$PROBE" 2>/dev/null; then
    say "reloc_probe が無いので建てる（初回は 1〜2 分）"
    docker exec -u ubuntu "$NAME" bash -c \
        "source /opt/ros/humble/setup.bash && \
         cmake -S /work/G1_Hackason/Mapping/real/quickstart/reloc -B /tmp/reloc_build \
               -DCMAKE_BUILD_TYPE=Release >/dev/null && \
         cmake --build /tmp/reloc_build -j4 >/dev/null" \
        || { echo "[boot] reloc_probe を建てられない" >&2; exit 2; }
fi

tocont() { printf '%s\n' "${1/#$G1_REPO_ROOT/\/work}"; }

# ⚠️ **ROS を source しないと reloc_probe は起動しない**
# （`libmp2p_icp.so.2.12: cannot open shared object file`）。2026-09-14 に踏んだ。
probe() { docker exec -u ubuntu "$NAME" bash -c \
              "source /opt/ros/humble/setup.bash && $PROBE $*"; }

# ── 較正モード ───────────────────────────────────────────────────────
if [ "$MODE" = calibrate ]; then
    [ -f "$SCAN_L" ] || { echo "[boot] スキャンが無い: $SCAN_L" >&2; exit 2; }
    say "較正: $SCAN_L を ($CAL_X, $CAL_Y, ${CAL_YAW} deg) で測る"
    # ⚠️ **`--residuals` は使わない。** あれは**全点**で測る（11917 点）が、
    # ゲートは**帯**（z > band-lo、1135 点）で測る。別の量を基準にすると
    # しきい 1.5 の意味が変わる。**ゲートと同じ経路（較正段）から取る。**
    cal_scan="$(cd "$(dirname "$SCAN_L")" && pwd)/$(basename "$SCAN_L")"
    out="$(probe --map "$MOLA_MAP_C" --scan "$(tocont "$cal_scan")" --relocalize \
              --trusted-pose "$CAL_X" "$CAL_Y" "$CAL_YAW" \
              --center "$CAL_X" "$CAL_Y" --roi 0.5 --res-xy 0.5 --res-phi 90 \
              --band-lo "$BAND_LO" 2>&1)"
    echo "$out"
    r0="$(printf '%s\n' "$out" | sed -nE 's/.*基準 r0 = ([0-9]+\.[0-9]+) m.*/\1/p' | head -1)"
    [ -n "$r0" ] || { echo "[boot] 残差の中央値を読めなかった" >&2; exit 2; }
    {
        echo "# ゲート A の基準残差 r0 [m]。bootstrap_localization.sh --calibrate が書いた"
        echo "# 地図: $MOLA_MAP_C"
        echo "# 素材: $SCAN_L"
        echo "# 姿勢: $CAL_X $CAL_Y $CAL_YAW deg / 帯 z>$BAND_LO m"
        echo "# 作成: $(date -u +%Y-%m-%dT%H:%M:%SZ)"
        echo "$r0"
    } > "$R0_FILE"
    say "書いた: $R0_FILE （r0 = $r0 m）"
    exit 0
fi

# ── r0 が無ければ起動しない ──────────────────────────────────────────
if [ ! -f "$R0_FILE" ]; then
    echo "[boot] 基準 r0 が無い: $R0_FILE" >&2
    echo "[boot] 先に --calibrate を 1 回通すこと（この地図について未較正）" >&2
    exit 2
fi
R0="$(grep -vE '^[[:space:]]*#' "$R0_FILE" | head -1 | tr -d ' ')"
[ -n "$R0" ] || { echo "[boot] $R0_FILE から r0 を読めない" >&2; exit 2; }

T_ALL0=$SECONDS

# ── 1. スキャンを 1 枚用意する ───────────────────────────────────────
if [ -z "$SCAN_L" ]; then
    if [ -z "$BAG_L" ]; then
        BAG_L="$RUNS_L/_boot/bag"
        say "live: LiDAR を ${SECS} 秒録る -> $BAG_L"
        rm -rf "$(dirname "$BAG_L")"
        mkdir -p "$(dirname "$BAG_L")"
        DDS="$(g1_dds_uri live)"
        docker exec -d -u ubuntu -e RMW_IMPLEMENTATION=rmw_cyclonedds_cpp \
            -e CYCLONEDDS_URI="$DDS" -e ROS_DOMAIN_ID=0 "$NAME" \
            bash -c "source /opt/ros/humble/setup.bash && cd $RUNS_C/_boot && \
                     trap 'pkill -INT -f \"ros2 bag recor[d]\"' TERM INT && \
                     ros2 bag record -o bag /utlidar/cloud_livox_mid360 /tf_static"
        sleep "$SECS"
        docker exec "$NAME" bash -c 'pkill -INT -f "ros2 bag recor[d]"' 2>/dev/null
        sleep 1.5
        [ -d "$BAG_L" ] || { echo "[boot] 記録できなかった（LiDAR は出ているか）" >&2; exit 2; }
    fi
    SCAN_L="$RUNS_L/_boot/scan"
    mkdir -p "$(dirname "$SCAN_L")"
    say "1 スキャンを base_link 系で出す"
    "$VENV" "$HERE/export_scan.py" "$BAG_L" --out "$SCAN_L" ${IDX:+--index "$IDX"} \
        || { echo "[boot] export_scan.py が失敗した" >&2; exit 2; }
    SCAN_L="$SCAN_L.xyz"
fi
[ -f "$SCAN_L" ] || { echo "[boot] スキャンが無い: $SCAN_L" >&2; exit 2; }
SCAN_ABS="$(cd "$(dirname "$SCAN_L")" && pwd)/$(basename "$SCAN_L")"

# ── 2. SE(2) 探索 + 詰め + ゲート A ─────────────────────────────────
JSON_L="$RUNS_L/_boot/reloc.json"
mkdir -p "$(dirname "$JSON_L")"

# 1 回ぶんの探索。$1 $2 中心 / $3 ROI / $4 刻みxy / $5 刻みphi / $6 json / $7 静かに
run_probe() {
    local _q=""
    [ "${7:-}" = quiet ] && _q=1
    rm -f "$6"
    if [ -n "$_q" ]; then
        probe --map "$MOLA_MAP_C" --scan "$(tocont "$SCAN_ABS")" --relocalize \
            --center "$1" "$2" --roi "$3" --res-xy "$4" --res-phi "$5" \
            --r0 "$R0" --gate-n "$GATE_N" --min-match "$MIN_MATCH" \
            --band-lo "$BAND_LO" --candidates "$CANDIDATES" \
            --json "$(tocont "$6")" >/dev/null 2>&1
    else
        probe --map "$MOLA_MAP_C" --scan "$(tocont "$SCAN_ABS")" --relocalize \
            --center "$1" "$2" --roi "$3" --res-xy "$4" --res-phi "$5" \
            --r0 "$R0" --gate-n "$GATE_N" --min-match "$MIN_MATCH" \
            --band-lo "$BAND_LO" --candidates "$CANDIDATES" \
            --json "$(tocont "$6")"
    fi
}

# 広域: 粗い格子でタイルを敷き詰め、**帯の残差と一致率**で選ぶ
search_wide() {
    local tiles="$RUNS_L/_boot/tiles.txt" picks="$RUNS_L/_boot/picks.txt"
    "$VENV" "$HERE/boot_tiles.py" "$REF_MAP_L" --roi "$ROI" \
        --step "$WIDE_STEP" --min-occ "$WIDE_MIN_OCC" > "$tiles" || return 2
    local n; n="$(wc -l < "$tiles" | tr -d ' ')"
    say "広域: ${n} 枚を粗く掃く（${WIDE_RES_XY} m・${WIDE_RES_PHI} deg / 1 枚 約 2 秒）"
    : > "$picks"
    local i=0 tx ty occ
    while read -r tx ty occ; do
        i=$((i + 1))
        run_probe "$tx" "$ty" "$ROI" "$WIDE_RES_XY" "$WIDE_RES_PHI" \
                  "$RUNS_L/_boot/tile.json" quiet
        [ -f "$RUNS_L/_boot/tile.json" ] || continue
        "$VENV" -c "
import json
d=json.load(open('$RUNS_L/_boot/tile.json'))
p=d['pose']; r=d.get('residual',-1); m=d.get('match_rate',0)
if r is not None and r >= 0 and m >= $MIN_MATCH:
    print('%.5f %.4f %.4f %.4f %.3f' % (r, m, p[0], p[1], p[2]))
" >> "$picks" 2>/dev/null
        printf '\r[boot]   %d/%d' "$i" "$n" >&2
    done < "$tiles"
    printf '\r' >&2
    [ -s "$picks" ] || { echo "[boot] 広域でも一致率を満たす候補が無い" >&2; return 3; }
    # ⚠️ 残差の**小さい順**。尤度では選ばない（正規化されていないため）
    read -r BX BY BYAW <<EOF2
$(sort -n "$picks" | head -1 | awk '{print $3, $4, $5}')
EOF2
    say "広域の best -> (${BX}, ${BY}, ${BYAW} deg)。ここを ±${NARROW_ROI} m で詰める"
    run_probe "$BX" "$BY" "$NARROW_ROI" "$RES_XY" "$RES_PHI" "$JSON_L"
    return $?
}

# ── 2〜3. 探索 → ゲート A → ゲート B ────────────────────────────────
# 局所で落ちたら**広域に落として測り直す**。⚠️ 黙って諦めるより広く探す方が良いが、
# **広く探しても落ちたら起動しない**（fail closed は崩さない）。
FAIL_RC=0
for _attempt in 1 2; do
    T0=$SECONDS
    if [ "$WIDE" = 1 ]; then
        search_wide; RC_A=$?
    else
        say "探索 ROI ±${ROI} m / 刻み ${RES_XY} m・${RES_PHI} deg / 中心 (${CENTER_X}, ${CENTER_Y})"
        run_probe "$CENTER_X" "$CENTER_Y" "$ROI" "$RES_XY" "$RES_PHI" "$JSON_L"
        RC_A=$?
    fi
    say "探索に $((SECONDS - T0)) 秒"

    PX=""; RATIO="-1"; RATE="0"
    if [ -f "$JSON_L" ]; then
        read -r PX PY PYAW RATIO RATE <<EOF
$("$VENV" -c "
import json
d=json.load(open('$JSON_L'))
p=d['pose']
print('%.4f %.4f %.3f %.3f %.4f' % (p[0],p[1],p[2],d.get('ratio',-1),d.get('match_rate',0)))
" 2>/dev/null)
EOF
    fi

    FAIL_RC=0
    if [ -z "$PX" ] || [ "$RC_A" -ne 0 ]; then
        say "⛔ ゲート A が落ちた（r/r0 = ${RATIO} / しきい ${GATE_N}、一致率 ${RATE}）"
        FAIL_RC=3
    else
        say "ゲート A 通過: 姿勢 (${PX}, ${PY}, ${PYAW} deg) / r/r0 = ${RATIO}"
        # ⚠️ **ここを省かない。** 3D の尤度の面は平らで、1 位を鵜呑みにすると 09-13 を繰り返す
        T0=$SECONDS
        OV="$("$VENV" "$HERE/overlay_at_pose.py" "$SCAN_ABS" "$REF_MAP_L" \
                --pose "$PX" "$PY" "$PYAW" --quiet 2>/dev/null)"
        [ -n "$OV" ] || { echo "[boot] overlay_at_pose.py が数字を返さない" >&2; exit 2; }
        say "ゲート B: 2D 重畳 ${OV} %（しきい ${OVERLAY_MIN} %、$((SECONDS - T0)) 秒）"
        if [ "$("$VENV" -c "print(1 if float('$OV') >= float('$OVERLAY_MIN') else 0)")" != "1" ]; then
            say "⛔ ゲート B が落ちた（重畳 ${OV} % < ${OVERLAY_MIN} %）"
            FAIL_RC=4
        fi
    fi

    [ "$FAIL_RC" = 0 ] && break
    if [ "$WIDE" != 1 ] && [ "$AUTOWIDE" = 1 ]; then
        say "→ 局所では決まらない。**広域に落として測り直す**"
        WIDE=1
        continue
    fi
    break
done

if [ "$FAIL_RC" != 0 ]; then
    echo "[boot] ⛔⛔ **決まらなかった。MOLA を起こしてはいけない。**" >&2
    echo "[boot] 機体を地図の濃い所へ移すか、地図を作り直すこと" >&2
    exit "$FAIL_RC"
fi

# ── 4. 採用 ─────────────────────────────────────────────────────────
T_ALL=$((SECONDS - T_ALL0))
say "✅ 採用。合計 ${T_ALL} 秒（合否の上限 90 秒）"
[ "$T_ALL" -le 90 ] || say "⚠️ 90 秒を超えた。ROI か刻みを見直すこと"
printf 'G1_MOLA_INIT_POSE=[%s, %s, 0.0, %s, 0.0, 0.0]\n' "$PX" "$PY" "$PYAW"
exit 0
