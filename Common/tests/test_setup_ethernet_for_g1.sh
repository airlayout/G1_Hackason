#!/usr/bin/env bash
# setup_ethernet_for_g1.sh のロジック検証。
#
# nmcli / arping / ip / sudo をモックに差し替えるので、実際のネットワークには
# 一切触れない。確認するのは「どのアドレスを選び、nmcli に何を渡すか」だけ。
#
#   bash Common/tests/test_setup_ethernet_for_g1.sh
#
# arping -D の戻り値（使用中=1 / 空き=0）は 2026-09-06 に G1 の実リンクで確認済み。
# 自分が既に持っているアドレスは「空き」と出る（DAD は自分のMACからの応答を無視する）
# ので、同じPCで再実行しても自分自身とは衝突しない。
set -uo pipefail

if [ "${BASH_VERSINFO[0]}" -lt 4 ]; then
    echo "skip: bash 4 以上が要ります（現在 ${BASH_VERSION}）。操作PCとCIはUbuntuなので対象外。" >&2
    exit 0
fi

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
SUT="${1:-$HERE/../network/setup_ethernet_for_g1.sh}"
[ -f "$SUT" ] || { echo "対象が見つかりません: $SUT" >&2; exit 1; }
WORK="$(mktemp -d)"
BIN="$WORK/bin"
mkdir -p "$BIN"
trap 'rm -rf "$WORK"' EXIT

cat > "$BIN/sudo" <<'EOF'
#!/usr/bin/env bash
exec "$@"
EOF

# IN_USE に載っているアドレスだけ「使用中」を返す
cat > "$BIN/arping" <<'EOF'
#!/usr/bin/env bash
target="${@: -1}"
for ip in ${IN_USE:-}; do [ "$ip" = "$target" ] && exit 1; done
exit 0
EOF

# 呼ばれた引数を $CALLS に記録する。EXISTING_CONN があればプロファイル在りとして振る舞う
cat > "$BIN/nmcli" <<'EOF'
#!/usr/bin/env bash
echo "nmcli $*" >> "$CALLS"
case "$*" in
    *"-t -f NAME connection show"*) [ -n "${EXISTING_CONN:-}" ] && echo "g1-link"; exit 0 ;;
    "connection show g1-link")      echo "ipv4.method: manual"; echo "ipv4.addresses: ${APPLIED:-}"; exit 0 ;;
esac
exit 0
EOF

cat > "$BIN/ip" <<'EOF'
#!/usr/bin/env bash
case "$*" in
    *"-br link show"*)       echo "lo   UNKNOWN  00:00:00:00:00:00"; exit 0 ;;
    *"-4 -o addr show dev"*) echo "1: lo    inet ${MOCK_ADDR:-}/24 scope global lo"; exit 0 ;;
    *"-br addr show"*)       echo "lo   UNKNOWN  ${MOCK_ADDR:-}/24"; exit 0 ;;
esac
exit 0
EOF
chmod +x "$BIN"/*

pass=0; fail=0
run() {  # run <name> <expected_rc> <env assignments...> -- <args...>
    local name="$1" want_rc="$2"; shift 2
    local -a envs=() args=()
    while [ "$1" != "--" ]; do envs+=("$1"); shift; done
    shift
    args=("$@")
    export CALLS="$WORK/calls"; : > "$CALLS"
    OUT="$(echo y | env PATH="$BIN:$PATH" G1_IFACE=lo "${envs[@]}" bash "$SUT" "${args[@]}" 2>&1)"
    RC=$?
    if [ "$RC" = "$want_rc" ]; then
        echo "  ok   $name (rc=$RC)"; pass=$((pass+1))
    else
        echo "  FAIL $name (rc=$RC, want $want_rc)"; echo "$OUT" | sed 's/^/       /'; fail=$((fail+1))
    fi
}
assert_out() { if echo "$OUT" | grep -q "$1"; then echo "  ok   └ 出力に「$1」"; pass=$((pass+1)); else echo "  FAIL └ 出力に「$1」が無い"; echo "$OUT" | sed 's/^/       /'; fail=$((fail+1)); fi; }
assert_call() { if grep -q -- "$1" "$WORK/calls"; then echo "  ok   └ nmcli 呼び出し「$1」"; pass=$((pass+1)); else echo "  FAIL └ nmcli 呼び出し「$1」が無い"; cat "$WORK/calls" | sed 's/^/       /'; fail=$((fail+1)); fi; }

echo "== 1. --help =="
run "--help は 0 で終わる" 0 -- --help
assert_out "使い方:"

echo "== 2. 明示指定 (空き) =="
run ".222 を指定して成功" 0 IN_USE= MOCK_ADDR=192.168.123.222 -- 222
assert_out "OK: lo = 192.168.123.222"
assert_call "ipv4.addresses 192.168.123.222/24"

echo "== 3. 明示指定を FQDN 風に書いた場合 =="
run "192.168.123.222 でも通る" 0 IN_USE= MOCK_ADDR=192.168.123.222 -- 192.168.123.222
assert_call "ipv4.addresses 192.168.123.222/24"

echo "== 4. 予約アドレスは拒否 =="
run ".164 (PC2) は拒否" 1 IN_USE= -- 164
assert_out "G1側が使っています"

echo "== 5. サブネット外は拒否 =="
run "10.0.0.5 は拒否" 1 IN_USE= -- 10.0.0.5
assert_out "の外です"

echo "== 6. 不正な値は拒否 =="
run "999 は拒否" 1 IN_USE= -- 999
assert_out "アドレスの指定が不正"

echo "== 7. 明示指定が使用中なら止まる =="
run ".222 が使用中なら失敗" 1 IN_USE=192.168.123.222 -- 222
assert_out "既に使われています"

echo "== 8. 自動選択 (.200 .201 が埋まっていれば .202) =="
run "空きを自動で選ぶ" 0 IN_USE="192.168.123.200 192.168.123.201" MOCK_ADDR=192.168.123.202 --
assert_out "OK: lo = 192.168.123.202"
assert_call "ipv4.addresses 192.168.123.202/24"

echo "== 9. 既存プロファイルはアドレスを入れ直す =="
run "modify が呼ばれる" 0 EXISTING_CONN=1 IN_USE= MOCK_ADDR=192.168.123.222 -- 222
assert_call "connection modify g1-link"
assert_out "既存の g1-link プロファイルを 192.168.123.222/24 に更新"

echo "== 10. 反映されなければ失敗させる =="
run "アドレスが載らなければ失敗" 1 IN_USE= MOCK_ADDR=192.168.123.9 -- 222
assert_out "が lo に載っていません"

echo "== 11. 走査範囲に空きが無い =="
run "空きが無ければ失敗" 1 G1_SCAN_FROM=200 G1_SCAN_TO=201 IN_USE="192.168.123.200 192.168.123.201" --
assert_out "空きがありませんでした"

echo
echo "pass=$pass fail=$fail"
[ "$fail" -eq 0 ]
