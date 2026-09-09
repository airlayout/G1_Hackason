#!/usr/bin/env bash
# Isaac Sim + Nav2 を**確実に**止める。停止はこの入口だけを使うこと。
#
# ## なぜ専用スクリプトが要るのか（2026-09-09 に 2 種類の事故を踏んだ）
#
# 停止に必要な signal が**送る先で逆になる**。手で kill を打つと必ず間違える。
#
#   | 対象                            | 正しい signal | 間違えると                        |
#   |---------------------------------|---------------|-----------------------------------|
#   | run_nav2.sh（外側のスクリプト） | SIGTERM       | SIGINT は SIG_IGN で**黙って無視**|
#   | ros2 launch（cleanup 内の子）   | SIGINT        | SIGTERM だと**子を孤児化**        |
#
# 1. run_nav2.sh は run_long_nav.sh 等から `setsid ... &` で起動されるため、
#    非対話シェルの非同期ジョブとして SIGINT / SIGQUIT が SIG_IGN にされる（POSIX）。
#    bash は起動時に無視されていた signal を trap で捕まえ直せないので、
#    スクリプト側では対処不能。`kill -INT` を送ると**15 分間何も起きない**
#    （ps の STAT は Ss のまま、Isaac Sim は R で走り続ける）。
# 2. 一方 ros2 launch は SIGTERM ではサブプロセスを止めずに自分だけ終了する
#    （launch_service.py の _on_sigterm にソースのまま
#    "using SIGTERM can result in orphaned processes" と書かれている）。
#    そのため component_container_isolated が PPID=1 の孤児として残り続ける。
#    run_nav2.sh の cleanup() は SIGINT を送るよう直してある。
#
# このスクリプトは 1 の正しい signal を送り、2 を cleanup() に任せ、
# **最後に本当に消えたかを検証する**（消えなければ強制 kill する）。
#
# 使い方:
#   bash stop_nav2.sh          # 止める（何も動いていなければ何もしない）
set -uo pipefail

# ⚠️⚠️ pgrep -f は**自分自身や呼び出し元のコマンドラインにもマッチする**。
# ssh 越しに `ssh host "... run_nav2.sh ..."` と打つと、その `bash -c` ラッパの
# コマンドライン文字列にパターンが含まれるため引っかかる。$$ の除外だけでは
# 足りない（ラッパは別 PID）。**すべての pgrep で `bash -c` を除外すること。**
# これを 1 箇所忘れて、検証中に自分の ssh セッションを SIGTERM で殺した
# （2026-09-09。exit 255 になって初めて気づいた）。
#
# $1: pgrep のパターン。出力は「PID cmd」の行。自分・呼び出し元・このスクリプトは除く。
find_procs() {
    pgrep -af "$1" 2>/dev/null \
        | grep -v 'bash -c' \
        | grep -v 'stop_nav2' \
        | awk -v self="$$" -v parent="$PPID" '$1 != self && $1 != parent' \
        || true
}
alive()      { find_procs 'run_g1_twin|component_container_isolated'; }
alive_pids() { alive | awk '{print $1}'; }

GRACE_S=60          # cleanup() が Isaac Sim を畳むのを待つ上限（実測は 30 秒未満）
POLL_S=3

if [[ -z "$(alive)" ]]; then
    echo "[OK] Isaac Sim / Nav2 は動いていません（何もしません）"
    exit 0
fi

echo "[INFO] 動いているもの:"
alive | sed 's/^/       /'

# run_nav2.sh に SIGTERM を送る。これで EXIT トラップの cleanup() が走り、
# その中から ros2 launch へ SIGINT が飛ぶ（順序が重要。逆にすると孤児が出る）。
mapfile -t SCRIPT_PIDS < <(find_procs 'run_nav2\.sh' | awk '{print $1}')
if [[ ${#SCRIPT_PIDS[@]} -gt 0 ]]; then
    echo "[INFO] run_nav2.sh (${SCRIPT_PIDS[*]}) に SIGTERM を送ります"
    for pid in "${SCRIPT_PIDS[@]}"; do
        kill -TERM "$pid" 2>/dev/null || true
    done
else
    echo "[WARN] run_nav2.sh が見つかりません（前回の孤児だけが残っている状態）"
fi

echo "[INFO] 終了を待っています（上限 ${GRACE_S} 秒）..."
WAITED=0
while [[ -n "$(alive)" && "$WAITED" -lt "$GRACE_S" ]]; do
    sleep "$POLL_S"
    WAITED=$((WAITED + POLL_S))
done

# 検証。ここで残っていたら cleanup() が届いていない（＝また何か壊れている）。
if [[ -n "$(alive)" ]]; then
    echo "[WARN] ${GRACE_S} 秒たっても残っています。強制的に止めます:"
    alive | sed 's/^/       /'
    for pid in $(alive_pids); do
        kill -9 "$pid" 2>/dev/null || true
    done
    sleep 3
fi

if [[ -z "$(alive)" ]]; then
    echo "[OK] 停止しました（${WAITED} 秒）。孤児なし"
    exit 0
fi
echo "[NG] まだ残っています。手で確認してください:"
alive | sed 's/^/       /'
exit 1
