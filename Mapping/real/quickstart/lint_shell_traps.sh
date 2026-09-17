#!/usr/bin/env bash
# シェルで**静かに壊れる**既知の型を機械的に探す。落ちないバグだけを狙う。
#
#   bash quickstart/lint_shell_traps.sh          # quickstart 配下を見る
#   bash quickstart/lint_shell_traps.sh path...  # 指定したものだけ
#
# ## なぜ要るか
#
# 2026-09-17 の 1 セッションで、**同じ型を自分で 4 回作った**。どれも
# 「落ちない・数字だけが変わる」型なので、動かしてみるまで気づけない。
# 注記を書いても再発したので、**書いた直後に機械で見る**形にする。
#
# 見る型:
#   1. `$VAR` の直後に全角文字 … `set -u` で `unbound variable`。
#      bash が「VAR（」までを変数名に取る。⚠️ **合格側の経路では出ない**
#   2. `grep -c` / `pgrep -c` に `|| echo 0` … 0 件でも「0」を印字して終了コード 1 を
#      返すので、数字が 2 個出て `"00"` になる。比較に使うと**必ず誤発火**する
#   3. PC2 側での `ps ... | grep` … ssh が送るコマンド行にパターンが載って**自己マッチ**。
#      `_common.sh` の `pc2_procs` / `pc2_count` / `pc2_pids` を使う
#   4. `timeout` にシェル関数（`jros2` など）… `command not found` で**出力が空**になり
#      「来ていない」と誤診する。ローダの実体を叩く（`pc2_ros2`）
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
TARGETS=("$@")
[ ${#TARGETS[@]} -gt 0 ] || TARGETS=("$HERE")

found=0
report() { printf '  \033[31m%s\033[0m %s\n' "$1" "$2" >&2; found=$((found + 1)); }

files=$(find "${TARGETS[@]}" -name '*.sh' -type f 2>/dev/null | grep -v '/\.venv/' | sort)

echo "[lint] $(printf '%s\n' "$files" | grep -c . ) 本の .sh を見る" >&2

# 1. $VAR の直後に全角文字（コメント行は無害なので除く）
while IFS= read -r hit; do
    report "全角の直後" "$hit"
done < <(printf '%s\n' "$files" | xargs grep -nP '^\s*[^#].*\$[A-Za-z_][A-Za-z0-9_]*[^\x00-\x7F]' 2>/dev/null || true)

# 2. grep -c / pgrep -c に || echo 0
while IFS= read -r hit; do
    report "|| echo 0 " "$hit"
done < <(printf '%s\n' "$files" | xargs grep -nE '^[0-9]*:?[[:space:]]*[^#[:space:]].*(grep|pgrep) +-[a-z]*c[a-z]* .*\|\| *echo +0' 2>/dev/null || true)

# 3. PC2 側での ps | grep（pc2 '...' の中に ps と grep が同居）
while IFS= read -r hit; do
    case "$hit" in *pc2_procs*|*pc2_count*|*pc2_pids*|*_common.sh*|*lint_shell_traps*) continue ;; esac
    report "PC2 で grep" "$hit"
done < <(printf '%s\n' "$files" | xargs grep -nE '^[0-9]*:?[[:space:]]*[^#[:space:]].*ps +-e?o[^|]*\| *grep' 2>/dev/null || true)

# 4. timeout にシェル関数
while IFS= read -r hit; do
    report "timeout+関数" "$hit"
done < <(printf '%s\n' "$files" | xargs grep -nE '^[0-9]*:?[[:space:]]*[^#[:space:]].*timeout +[0-9.]+ +(jros2|jrun|pc2|ctr)\b' 2>/dev/null || true)

printf '\n' >&2
if [ "$found" = 0 ]; then
    echo "[lint] 既知の型は見つからなかった" >&2; exit 0
fi
echo "[lint] ⛔ $found 件。直し方は各項目の注記（このファイルの冒頭）" >&2
exit 1
