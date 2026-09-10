#!/usr/bin/env bash
# デモの入口。番号を選ぶだけ。
#
#   bash Demo/demo.sh
#
# 当日はこれと Demo/README.html だけ見ればよい（README.html は Phase 5 で作成）。
set -eo pipefail

source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/lib.sh"

have() { [ -f "${DEMO_DIR}/$1" ]; }

while true; do
    ROBOT="実機: \033[31m未接続\033[0m"
    if robot_reachable; then ROBOT="実機: \033[32m接続\033[0m"; fi

    printf '\n\033[1m====== G1 デモ ======\033[0m   %b\n\n' "${ROBOT}"
    printf '  1) LiDAR で計測を見せる        （実機が無ければ記録の再生に落ちます）\n'
    printf '  2) Isaac Sim で Nav2 走行      （実機不要）\n'
    if have 03_teleop.sh; then
        printf '  3) Quest でエピソード記録・再生 \033[31m※実機の腕が動きます\033[0m\n'
    else
        printf '  3) Quest でエピソード記録・再生 \033[33m（未実装 — Phase 4）\033[0m\n'
    fi
    printf '\n'
    printf '  p) 事前確認（何が使えるか調べる）\n'
    printf '  s) 全部止める\n'
    printf '  q) 終了\n\n'
    printf '番号を入れてください > '
    read -r CHOICE

    case "${CHOICE}" in
        1) bash "${DEMO_DIR}/01_lidar.sh" || _warn "終了コード $?" ;;
        2) bash "${DEMO_DIR}/02_isaac_nav2.sh" || _warn "終了コード $?" ;;
        3)
            if have 03_teleop.sh; then
                printf '\n\033[31m⚠️ 実機の腕が動きます。起動直後に両腕がゼロ姿勢へ移動します。\033[0m\n'
                printf '   可動範囲に人と物が無いこと、非常停止に手が届くことを確認しましたか？ [yes/no] > '
                read -r OKAY
                if [ "${OKAY}" = "yes" ]; then
                    bash "${DEMO_DIR}/03_teleop.sh" || _warn "終了コード $?"
                else
                    _info "やめました"
                fi
            else
                _warn "まだ作っていません（Phase 4）。いまは Teleop/vendor/g1-starter-kit/README.md の手順で動かしてください"
            fi ;;
        p) bash "${DEMO_DIR}/preflight.sh" || true ;;
        s) bash "${DEMO_DIR}/stop.sh" || true ;;
        q) _info "終了します"; exit 0 ;;
        *) _warn "1 / 2 / 3 / p / s / q のどれかを入れてください" ;;
    esac
done
