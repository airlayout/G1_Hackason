#!/usr/bin/env bash
# 段 4 の記録: **測位が出す map->base_link** と **カメラ画像** を同時に録る。
#
# ## 測位アルゴリズムに依存しない
#
# `log_tf_pose.py` は tf2 の `lookup_transform(map, base_link)` を使う。
# tf2 は鎖を自動で辿るので、MOLA（直接）でも AMCL（map->odom->base_link）でも
# FAST-LIO（静的 2 本を挟む）でも**同じこの 1 本で取れる**。
# 測位側は何が動いていてもよく、このスクリプトは何も起動しない。
#
# ## 使い方（PC2 で。測位を先に動かしておくこと）
#
#   bash record_registration.sh /tmp/reg_A 15
#   # 機体を別の場所へ移してから
#   bash record_registration.sh /tmp/reg_B 15
#
# ⚠️ **録っている間は機体を止めておく。** register_tags.py が姿勢の振れを見て、
# 動いていたら受け付けない（位置 50 mm / 向き 2 deg）。
#
# ⚠️ **`pkill -f` で止めない。** ssh 越しだと自分の shell を巻き添えにする
# （2026-09-15 に 2 回踏んだ）。ここでは PID を直接持って止める。
set -euo pipefail

OUT_DIR="${1:?置き場を指定する（例 /tmp/reg_A）}"
SECONDS_TO_RECORD="${2:-15}"
TAG_MM="${G1_TAG_MM:-160}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
JAMMY="${G1_JAMMY:-$HOME/jammy_ros}"
LOGGER="${G1_TF_LOGGER:-$HERE/log_tf_pose.py}"

# ⚠️ **測位を動かさない場合はこちら（2026-09-16 以降の既定の使い方）。**
# 姿勢は `global_localize.py` がスキャンから解いて pose.txt を書く。
# 測位より正確で、測位スタックを起こす必要も無い。
NO_POSE="${G1_REG_NO_POSE:-1}"

[ -f "$JAMMY/env.sh" ] || { echo "[reg] env.sh が無い: $JAMMY/env.sh" >&2; exit 2; }
if [ "$NO_POSE" != "1" ]; then
    [ -f "$LOGGER" ] || { echo "[reg] log_tf_pose.py が無い: $LOGGER" >&2; exit 2; }
fi

mkdir -p "$OUT_DIR"
rm -f "$OUT_DIR"/raw_*.png "$OUT_DIR"/pose.txt

# shellcheck source=/dev/null
. "$JAMMY/env.sh"
# ⚠️ env.sh は再生用に lo/domain 42 へ閉じる。live は実機と同じ面に出す。
export ROS_DOMAIN_ID="${G1_PC2_DOMAIN:-0}"
if [ -n "${G1_PC2_DDS_URI:-}" ]; then
    export CYCLONEDDS_URI="$G1_PC2_DDS_URI"
else
    NIC="${G1_PC2_DDS_NIC:-eth0}"
    export CYCLONEDDS_URI="<CycloneDDS><Domain><General><Interfaces><NetworkInterface name=\"$NIC\" priority=\"default\" multicast=\"default\"/></Interfaces></General></Domain></CycloneDDS>"
fi

# ⚠️ **trap を必ず張る。** これが無いと Ctrl-C や途中終了で姿勢ロガーが孤児になり、
# 1 個あたり 14 % の CPU を食い続ける（2026-09-16 に 5 個溜めた）。
# 測位と CPU を取り合うので、気づかないまま測位を痩せさせる。
LOGGER_PID=""
cleanup() {
    [ -n "$LOGGER_PID" ] || return 0
    kill "$LOGGER_PID" 2>/dev/null || true
    for _ in $(seq 20); do kill -0 "$LOGGER_PID" 2>/dev/null || return 0; sleep 0.2; done
    kill -9 "$LOGGER_PID" 2>/dev/null || true
}
trap cleanup EXIT INT TERM

if [ "$NO_POSE" = "1" ]; then
    echo "[reg] 姿勢は録らない（G1_REG_NO_POSE=1）。スキャンから global_localize.py で解く"
else
echo "[reg] map->base_link を録る（domain ${ROS_DOMAIN_ID}）"
# ⚠️ **`jrun` は使わない。** あれはシェル関数（`local` を含む複数コマンド）なので、
# `&` で起こすと bash が exec 最適化をせず、`$!` が**中間のサブシェル**を指す。
# その PID を kill しても孫の python は生き残り、孤児になる（2026-09-16 実測）。
# ここでは env/loader を直に起こして `$!` を実体に一致させる。
# shellcheck disable=SC2086
env $JAMMY_ENV "$LOADER" --library-path "$JAMMY_LIBS" \
    "$PREFIX/usr/bin/python3.10" "$LOGGER" \
    --target map --source base_link --out "$OUT_DIR/pose.txt" \
    > "$OUT_DIR/pose.log" 2>&1 &
LOGGER_PID=$!
sleep 2
if ! kill -0 "$LOGGER_PID" 2>/dev/null; then
    echo "[reg] ⛔ 姿勢の記録がすぐ落ちた。ログ:" >&2
    tail -20 "$OUT_DIR/pose.log" >&2
    exit 3
fi
fi

echo "[reg] カメラを ${SECONDS_TO_RECORD} 秒録る（機体は動かさないこと）"
# ⚠️ ここは軽くしておく。検出は register_tags.py が後で精密化ありでやり直す。
python3 "$HERE/see_tags.py" --seconds "$SECONDS_TO_RECORD" --tag-mm "$TAG_MM" \
    --quiet --fast --save-dir "$OUT_DIR" --save-every 8 2>&1 \
    | grep -vE '^\[ (WARN|ERROR)' || true

cleanup
LOGGER_PID=""

# ⚠️ **スキャンも 1 枚落としておく。** 姿勢はこれを `global_localize.py` に通して
# 求める。撮った瞬間のスキャンが無いと、あとから姿勢を解けない（2026-09-16 の教訓）。
# shellcheck disable=SC2086
env $JAMMY_ENV "$LOADER" --library-path "$JAMMY_LIBS" \
    "$PREFIX/usr/bin/python3.10" "$ROS/bin/ros2" topic echo /scan --once --full-length \
    > "$OUT_DIR/scan.yaml" 2>"$OUT_DIR/scan.log" &
SCAN_PID=$!
for _ in $(seq 60); do kill -0 "$SCAN_PID" 2>/dev/null || break; sleep 0.5; done
kill -9 "$SCAN_PID" 2>/dev/null || true
SCAN_LINES=$(awk 'END {print NR+0}' "$OUT_DIR/scan.yaml" 2>/dev/null || echo 0)
# ⚠️ **`--full-length` を付けないと配列が `'...'` で省略される。**
# 行数だけ見ると 144 行あって通ってしまうので、省略記号そのものを検査する
# （2026-09-16 に踏んだ。行数のガードは弱すぎた）。
SCAN_TRUNCATED=$(grep -c "^\.\.\.$" "$OUT_DIR/scan.yaml" 2>/dev/null || true)
echo "[reg] スキャン ${SCAN_LINES} 行 → $OUT_DIR/scan.yaml"
if [ "$SCAN_LINES" -lt 400 ] || [ "${SCAN_TRUNCATED:-0}" -gt 0 ]; then
    echo "[reg] ⛔ /scan が取れていない（${SCAN_LINES} 行・省略 ${SCAN_TRUNCATED:-0} 箇所）。" >&2
    echo "        run_scan_only.sh start と --full-length を確かめる" >&2
    tail -5 "$OUT_DIR/scan.log" >&2
    exit 6
fi

# ⚠️ `grep -c` は 0 件のとき終了コード 1 を返す。`|| echo 0` と繋ぐと出力が
# 「0<改行>0」になり、その後の `[ -lt ]` が壊れて**黙って合格する**
# （2026-09-16 に踏んだ）。数えるのは常に成功する awk にする。
IMAGES=$(find "$OUT_DIR" -maxdepth 1 -name 'raw_*.png' | awk 'END {print NR+0}')
POSES=$(awk '!/^#/ && NF {count++} END {print count+0}' "$OUT_DIR/pose.txt" 2>/dev/null || echo 0)
echo "[reg] 画像 ${IMAGES} 枚 / 姿勢 ${POSES} 件 → $OUT_DIR"
if [ "$NO_POSE" != "1" ] && [ "$POSES" -lt 5 ]; then
    echo "[reg] ⛔ 姿勢がほとんど取れていない。測位が map->base_link を出しているか確かめる:" >&2
    echo "        tail -5 $OUT_DIR/pose.log" >&2
    exit 4
fi
if [ "$IMAGES" -lt 3 ]; then
    echo "[reg] ⛔ 画像が足りない。カメラを他のプロセスが握っていないか（serve_view.sh stop）" >&2
    exit 5
fi
echo "[reg] OK"
