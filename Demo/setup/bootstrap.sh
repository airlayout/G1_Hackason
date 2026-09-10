#!/usr/bin/env bash
# デモ機に G1_Hackason を **clone し直す**。1 回だけ実行する。
#
#   bash Demo/setup/bootstrap.sh
#   bash Demo/setup/bootstrap.sh --dry-run
#
# ## なぜ要るのか
#
# デモ機の ~/G1_Hackason は **.git の無い単純コピー**だった。中身を git と突き合わせ
# られず、更新もできない。ちゃんと clone した状態にする。
#
# ## 消さないもの
#
# - 既存の ~/G1_Hackason は **~/G1_Hackason.bak.<日付> に退避する。消さない**
# - ~/g1-starter-kit も **消さない**。取り込み側と食い違ったときの比較元になる
# - 退避側の logs/ runs/ など git 管理外の作業物を戻すかは最後に聞く
set -eo pipefail

REPO_URL="${REPO_URL:-https://github.com/airlayout/G1_Hackason.git}"
BRANCH="${BRANCH:-Dev/Navigation}"
TARGET="${TARGET:-${HOME}/G1_Hackason}"
STAMP="$(date +%Y%m%d-%H%M%S)"
BACKUP="${TARGET}.bak.${STAMP}"

DRY=0
[ "${1:-}" = "--dry-run" ] && DRY=1

_ok()   { printf '\033[32m[OK]\033[0m   %s\n' "$*"; }
_ng()   { printf '\033[31m[NG]\033[0m   %s\n' "$*"; }
_warn() { printf '\033[33m[注意]\033[0m %s\n' "$*"; }
_info() { printf '[INFO] %s\n' "$*"; }
_die()  { printf '\n\033[31m[中断]\033[0m %s\n' "$*" >&2; exit 1; }
_head() { printf '\n\033[1m=== %s ===\033[0m\n' "$*"; }
run()   { if [ "${DRY}" = "1" ]; then printf '  (dry-run) %s\n' "$*"; else eval "$@"; fi; }

_head "clone し直す"
_info "取得元 : ${REPO_URL}"
_info "ブランチ: ${BRANCH}"
_info "置き場所: ${TARGET}"
[ "${DRY}" = "1" ] && _warn "--dry-run。実際には何も変えません"

# 既に clone 済みなら何もしない
if [ -d "${TARGET}/.git" ]; then
    _ok "すでに git リポジトリです。何もしません"
    git -C "${TARGET}" remote -v
    git -C "${TARGET}" branch --show-current
    exit 0
fi

if [ -e "${TARGET}" ]; then
    _head "1. いまの ${TARGET} を退避する（消しません）"
    _info "→ ${BACKUP}"
    run "mv '${TARGET}' '${BACKUP}'"
    _ok "退避しました"
else
    _info "${TARGET} はありません。そのまま clone します"
fi

_head "2. clone する"
run "git clone --branch '${BRANCH}' '${REPO_URL}' '${TARGET}'"
if [ "${DRY}" = "0" ]; then
    git -C "${TARGET}" branch --show-current
    git -C "${TARGET}" log --oneline -1
fi

_head "3. Teleop の設定を戻す"
# g1.env.omen は .gitignore 済みなので clone には入らない。退避側かキットから拾う。
ENV_DST="${TARGET}/Teleop/config/g1.env.omen"
KIT_SRC="${HOME}/g1-starter-kit/config/g1.env"
BAK_SRC="${BACKUP}/Teleop/config/g1.env.omen"
if [ -f "${BAK_SRC}" ]; then
    _info "退避側から戻します"
    run "cp '${BAK_SRC}' '${ENV_DST}'"
elif [ -f "${KIT_SRC}" ]; then
    _info "~/g1-starter-kit から拾います"
    run "cp '${KIT_SRC}' '${ENV_DST}'"
else
    _warn "設定が見つかりません。Teleop/config/README.md の手順で作ってください"
fi
# キット配下にも置く（キットの .gitignore で追跡されないため）
if [ "${DRY}" = "0" ] && [ -f "${ENV_DST}" ]; then
    cp "${ENV_DST}" "${TARGET}/Teleop/vendor/g1-starter-kit/config/g1.env"
    _ok "キット配下にも置きました"
fi

_head "4. git 管理外の作業物について"
# dry-run では退避先がまだ無いので、いまの中身を見る
INSPECT="${BACKUP}"
[ "${DRY}" = "1" ] && INSPECT="${TARGET}"
if [ -d "${INSPECT}" ]; then
    printf '  退避側に残っているもの:\n'
    FOUND=0
    for d in IsaacSim_Env/logs Mapping/real/runs Teleop/config; do
        if [ -d "${INSPECT}/${d}" ]; then
            printf '    %-28s %s\n' "${d}" "$(du -sh "${INSPECT}/${d}" 2>/dev/null | cut -f1)"
            FOUND=1
        fi
    done
    [ "${FOUND}" = "0" ] && printf '    （ありません）\n' 
    printf '\n  戻したいものがあれば手でコピーしてください:\n'
    printf '    cp -r %s/Mapping/real/runs %s/Mapping/real/\n' "${BACKUP}" "${TARGET}"
    printf '\n  ⚠️ 退避したディレクトリは通し稽古（Phase 5）が通るまで消さないこと。\n'
fi

_head "5. 次にやること"
printf '  bash %s/Demo/preflight.sh        … 何が使えるか確認\n' "${TARGET}"
printf '  bash %s/Demo/03_isaac_nav2.sh    … 実演3 が通るか実測\n' "${TARGET}"
