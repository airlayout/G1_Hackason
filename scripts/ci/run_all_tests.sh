#!/usr/bin/env bash
# 各機能フォルダのテストを自動で見つけて実行する。
# ローカルでも実行できる: bash scripts/ci/run_all_tests.sh
#
# 規約: テストを追加するときは <フォルダ>/tests/run_tests.sh を作る。
# ここが自動で見つけて実行するので、CI設定(.github/workflows/ci.yml)を触る必要はない。
#
# テストが無いフォルダも一覧に出す。CIが緑でも「検証されていない」ことが
# 見えるようにするため。緑＝安全ではない。
#
# PR では、変えたフォルダのテストだけを実行する（2026-10-20）:
#   CI_CHANGED_FILES（変わったファイルの一覧。改行区切り）があれば、そのファイルが属する機能フォルダと、
#   それに依存する機能フォルダ（下の DEPENDS_ON）のテストだけを実行する。共通の部分（scripts/、.github/、
#   リポジトリ直下のファイル）が変わっていればすべて実行する。
#   理由: ある班のフォルダが壊れていると全員の PR が赤になり、赤が常態化して CI を見なくなっていた
#   （直近 8 件のマージのうち緑は 1 件）。「赤 = 自分の変更が原因」にする。
#   環境変数が無ければ（ローカル、main への push、夜間の実行）すべて実行する。
#   ローカルで PR と同じ選び方を試す: CI_CHANGED_FILES="$(git diff --name-only origin/main...HEAD)" bash scripts/ci/run_all_tests.sh
set -euo pipefail

cd "$(git rev-parse --show-toplevel)"

# 棚上げ中のフォルダは対象外（IsaacSim_Envのtest_*.pyはIsaac Sim本体が要る）
FEATURE_DIRS=(Common SimpleWalk Perception Mapping Navigation Entame Button_Press)

# 機能フォルダが、コードを読み込んでいるほかの機能フォルダ（"使う側 使われる側"）。
# 使われる側が変わったら、使う側のテストも実行する。コードの依存を足したらここにも足す
# （Button_Press/J1-gen の perception_bridge.py が Perception/common を読み込む）
DEPENDS_ON=(
    "Button_Press Perception"
)

# ---- 実行する機能フォルダを決める -------------------------------------------
declare -A SELECTED=()
SELECT_ALL=1
if [ -n "${CI_CHANGED_FILES:-}" ]; then
    SELECT_ALL=0
    while IFS= read -r f; do
        [ -z "$f" ] && continue
        top="${f%%/*}"
        if [ "$top" = "$f" ] || [ "$top" = "scripts" ] || [ "$top" = ".github" ]; then
            SELECT_ALL=1  # 直下のファイル・共通のスクリプト・CI の設定
            break
        fi
        SELECTED["$top"]=1
    done <<< "$CI_CHANGED_FILES"
    # 依存している側も足す（1 段。依存の連鎖は今のところ無い）
    for pair in "${DEPENDS_ON[@]}"; do
        read -r user used <<< "$pair"
        if [ -n "${SELECTED[$used]:-}" ]; then
            SELECTED["$user"]=1
        fi
    done
fi

if [ "$SELECT_ALL" = 1 ]; then
    echo "=== 対象: すべての機能フォルダ ==="
else
    echo "=== 対象: PR で変えたフォルダと、それに依存するフォルダ（ほかは実行しない） ==="
    for dir in "${FEATURE_DIRS[@]}"; do
        [ -n "${SELECTED[$dir]:-}" ] && echo "  $dir"
    done
fi
echo

echo "=== テストの所在 ==="
declare -a RUNNERS=()
for dir in "${FEATURE_DIRS[@]}"; do
    # フォルダの中の run_tests.sh をすべて実行する（以前は最初の 1 つだけで、Button_Press/Yada などが
    # CI で動いていなかった。2026-10-19 に修正）
    mapfile -t found < <(git ls-files "$dir/tests/run_tests.sh" "$dir/*/tests/run_tests.sh")
    if [ ${#found[@]} -gt 0 ]; then
        for runner in "${found[@]}"; do
            sub="$(dirname "$(dirname "$runner")")"
            count=$(git ls-files "$sub/**/test_*.py" "$sub/test_*.py" | sort -u | wc -l)
            if [ "$SELECT_ALL" = 1 ] || [ -n "${SELECTED[$dir]:-}" ]; then
                printf '  %-12s %s（test_*.py %s ファイル）\n' "$dir" "$runner" "$count"
                RUNNERS+=("$runner")
            else
                printf '  %-12s %s（この PR では変えていないので実行しない）\n' "$dir" "$runner"
            fi
        done
    else
        printf '  %-12s テストなし（このフォルダは検証されていない）\n' "$dir"
    fi
done

if [ ${#RUNNERS[@]} -eq 0 ]; then
    echo
    echo "実行するテストが無い（変えたフォルダにテストが無い、またはテストのあるフォルダを変えていない）。"
    exit 0
fi

# 1 つが失敗しても残りを実行し、最後に失敗したものをまとめて示す
declare -a FAILED=()
for runner in "${RUNNERS[@]}"; do
    echo
    echo "=== 実行: $runner ==="
    bash "$runner" || FAILED+=("$runner")
done

echo
if [ ${#FAILED[@]} -gt 0 ]; then
    echo "=== 失敗: ${#FAILED[@]} / ${#RUNNERS[@]} ==="
    printf '  %s\n' "${FAILED[@]}"
    exit 1
fi
echo "=== すべて成功: ${#RUNNERS[@]} ==="
