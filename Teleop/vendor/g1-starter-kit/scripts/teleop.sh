#!/usr/bin/env bash
# Quest のコントローラで G1 の腕を操作する（xr_teleoperate の起動ラッパー）。
#
#   ./scripts/teleop.sh                      # 操作のみ
#   ./scripts/teleop.sh --record             # 動きを記録する（あとで再生できる）
#   ./scripts/teleop.sh --record --task demo # 記録の保存先名を指定
#   ./scripts/teleop.sh --arm G1_29          # 機体を明示（既定は自動判定）
#   ./scripts/teleop.sh --motion             # モーションコントローラ稼働中はこちら
#
# ⚠️ --motion の有無は「好み」ではなく、ロボットの状態で決まります。
#    稼働中 → rt/arm_sdk（--motion 必要） / 停止中 → rt/lowcmd（--motion 不要）。
#    間違えると **エラーが出ないまま腕が動きません**。起動前に自動で確認します。
#    状態を見るには: python3 tools/mode_check.py --iface <有線iface>
#
# ⚠️ このスクリプトはキーボード操作が必要なので、必ず対話ターミナルで動かすこと。
#
# ⚠️ 起動直後、両腕が自動でゼロ姿勢へ動きます。周囲をクリアにしてから実行すること。
#    setup/apply_patches.py を適用してあれば、この初動は低速（2 rad/s）になります。

set -eo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/lib.sh"

ARM=""
TASK="demo"
MOTION=0
EXTRA=()

while [ $# -gt 0 ]; do
  case "$1" in
    --arm)     ARM="$2"; shift ;;
    --task)    TASK="$2"; shift ;;
    --record)  EXTRA+=("--record") ;;
    --motion)  MOTION=1; EXTRA+=("--motion") ;;
    -h|--help) sed -n '2,18p' "$0"; exit 0 ;;
    *)         EXTRA+=("$1") ;;
  esac
  shift
done

load_config
require_wired

TELEOP_DIR="$HOME/xr_teleoperate/teleop"
[ -d "$TELEOP_DIR" ] || _die "xr_teleoperate が見つかりません: $TELEOP_DIR
     setup/install_env.sh を実行してください。"

use_tv
[ -n "$ARM" ] || ARM="$(resolve_arm)"
require_motion_mode_match "$MOTION"
require_motors_enabled

cat <<EOF
==============================================
 XR テレオペ
  機体            : $ARM
  インターフェース: $G1_WIRED_IFACE
  指令トピック    : $([ "$MOTION" = "1" ] && echo "rt/arm_sdk (--motion)" || echo "rt/lowcmd")
  記録            : $([ ${#EXTRA[@]} -gt 0 ] && echo "あり → ~/xr_teleoperate/teleop/utils/data/$TASK/" || echo なし)
----------------------------------------------
 ⚠️ 起動直後に両腕がゼロ姿勢へ動きます。周囲をクリアに。

 キー操作:
   r … 追従開始（先にコントローラを今の腕の位置・向きに合わせる）
   s … 記録の開始 / もう一度押すと保存（--record 時のみ・r の後）
   q … 終了（両腕が約5秒で初期姿勢へ戻ってから停止）
   ※ Ctrl+C は使わないこと（腕が戻らないまま止まります）

 29DoF 機の初回起動は、逆運動学モデルの再構築で 1 分ほど無反応になります（正常）。
EOF
print_quest_url
echo "=============================================="

cd "$TELEOP_DIR"
exec python teleop_hand_and_arm.py \
  --input-mode=controller \
  --arm="$ARM" \
  --display-mode=pass-through \
  --network-interface="$G1_WIRED_IFACE" \
  --task-name="$TASK" \
  "${EXTRA[@]}"
