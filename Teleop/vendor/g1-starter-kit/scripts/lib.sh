#!/usr/bin/env bash
# 各スクリプトが読み込む共通処理。単体では実行しない。
#
#   source "$(dirname "$0")/lib.sh"
#   load_config          # config/g1.env を読んで検証する
#   use_ros              # ROS 2 のスタックを有効化（LiDAR 用）
#   use_tv               # conda 環境 tv を有効化（テレオペ / リプレイ用）
#
# なぜ 2 つのスタックを分けるのか:
#   xr_teleoperate は conda 環境 `tv` の python で動く。
#   ROS 2 はシステムの python で動く。
#   両方を同時に有効にすると python とライブラリが衝突するので、
#   用途ごとに片方だけを有効にする。

# ROS の setup.bash は未定義変数を参照するので set -u とは併用できない
set +u

REPO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
export REPO_DIR

_die() { printf '\n\033[31m[中断]\033[0m %s\n' "$*" >&2; exit 1; }
_warn() { printf '\033[33m[注意]\033[0m %s\n' "$*"; }
_ok() { printf '\033[32m[OK]\033[0m   %s\n' "$*"; }

# ---------------------------------------------------------------------------
# 設定の読み込み
# ---------------------------------------------------------------------------
load_config() {
  local env_file="$REPO_DIR/config/g1.env"
  if [ ! -f "$env_file" ]; then
    _die "設定ファイルがありません: config/g1.env
     まず雛形をコピーしてください:
       cp config/g1.env.example config/g1.env
     値が分からない項目は空のままで構いません。
     tools/preflight.py と tools/lidar_probe.py が実機から読み取って教えてくれます。"
  fi

  # コメントと空行を除いて読み込む（値に空白を含まない前提）
  set -a
  # shellcheck disable=SC1090
  source "$env_file"
  set +a

  : "${G1_WIRED_IFACE:=}"
  : "${G1_HOST_IP:=}"
  : "${G1_ARM:=auto}"
  : "${G1_LIDAR_FLIP:=true}"
  : "${G1_ALLOW_WIRELESS_CONTROL:=false}"
  : "${G1_WIFI_IFACE:=}"
  : "${ROS_DOMAIN_ID:=42}"
  export ROS_DOMAIN_ID

  [ -n "$G1_WIRED_IFACE" ] || _die "config/g1.env の G1_WIRED_IFACE が空です。
     \`ip -br addr\` で有線インターフェース名を確認して設定してください。"
}

# インターフェースが無線かどうか
iface_is_wireless() {
  [ -d "/sys/class/net/$1/wireless" ] || [ -e "/sys/class/net/$1/phy80211" ]
}

# ロボットと同じサブネットに居るかだけを確認する（読み取り専用の用途向け）。
# 無線でも通すが、値がばらつきうることは警告する。
require_robot_link() {
  local state
  state="$(cat "/sys/class/net/$G1_WIRED_IFACE/operstate" 2>/dev/null || echo missing)"
  case "$state" in
    missing) _die "インターフェース $G1_WIRED_IFACE が存在しません。\`ip -br addr\` で名前を確認してください。" ;;
    up)      ;;
    *)       _die "$G1_WIRED_IFACE のリンクが上がっていません（state=$state）。
     LAN ケーブルの接続と、ロボットの起動完了を確認してください。" ;;
  esac

  local addr
  addr="$(ip -4 -br addr show "$G1_WIRED_IFACE" 2>/dev/null)"
  if [[ "$addr" != *"192.168.123."* ]]; then
    _die "$G1_WIRED_IFACE に 192.168.123.x の IP が付いていません。
     現在: ${addr:-（アドレスなし）}
     docs/02-network.md の手順で固定 IP を設定してください。"
  fi

  if iface_is_wireless "$G1_WIRED_IFACE"; then
    _warn "$G1_WIRED_IFACE は無線です。読み取りは動きますが、受信レートや
     取りこぼしが有線より不安定になります（docs/02-network.md 参照）。"
  fi
}

# ロボットを動かす用途の前提確認。読み取りに加えて、経路が有線であることを要求する。
#
# なぜ無線を拒否するか（docs/02-network.md に根拠と出典）:
#   ・50Hz の閉ループ指令で遅延ジッタとロスが直接動作に出る
#   ・中断・解放の指令も同じ経路を通るので、切れたときに止められる保証がない
#   ・Unitree DDS はマルチキャストでディスカバリし、AP による扱いの差が大きい
# どうしても試す場合は config/g1.env で G1_ALLOW_WIRELESS_CONTROL=true（実験扱い）。
require_wired() {
  require_robot_link
  if iface_is_wireless "$G1_WIRED_IFACE"; then
    if [ "$G1_ALLOW_WIRELESS_CONTROL" = "true" ]; then
      _warn "無線経路でロボットを動かします（G1_ALLOW_WIRELESS_CONTROL=true）。
     実験用の設定です。必ず吊り下げ等で支持し、人を近づけないでください。"
    else
      _die "$G1_WIRED_IFACE は無線です。ロボットを動かす用途では有線を使ってください。
     理由は docs/02-network.md の「なぜ有線か」を参照してください。
     どうしても試す場合は config/g1.env に次を書いてください（実験扱い）:
       G1_ALLOW_WIRELESS_CONTROL=true"
    fi
  fi
}

# 機体の腕タイプを決める。auto なら preflight に判定させる。
resolve_arm() {
  if [ "$G1_ARM" != "auto" ]; then
    echo "$G1_ARM"; return
  fi
  local detected
  detected="$(use_tv_quiet && python3 "$REPO_DIR/tools/preflight.py" \
                --iface "$G1_WIRED_IFACE" --print-arm 2>/dev/null | tail -1)"
  case "$detected" in
    G1_23|G1_29) echo "$detected" ;;
    *) _die "機体の DoF を自動判定できませんでした。
     config/g1.env の G1_ARM に G1_23 か G1_29 を明示してください。
     見分け方: 手首の軸が 1 つ = G1_23 / 3 つ = G1_29" ;;
  esac
}

# 指令トピックと機体の状態が食い違っていないか確認する。
#
# 腕を動かす経路は 2 つあり、どちらが効くかはロボットの状態で変わる:
#   モーションコントローラ稼働中 → rt/arm_sdk （--motion が必要）
#   デバッグ状態（停止）        → rt/lowcmd  （--motion を付けない）
#
# 食い違うと、どちらも「エラーを出さずに腕が動かない」形で失敗する。
#   ・稼働中に rt/lowcmd を出す → コントローラが 1000Hz で上書きし続けて無視される
#   ・停止中に rt/arm_sdk を出す → 制御権を渡す相手が居ないので何も起きない
# 原因が分かりにくいので、起動前にここで弾く。
#
#   $1 … --motion を付けるつもりなら 1、付けないなら 0
require_motion_mode_match() {
  local want="$1" name
  name="$(python3 "$REPO_DIR/tools/mode_check.py" --iface "$G1_WIRED_IFACE" --print 2>/dev/null | tail -1)"

  case "$name" in
    unknown|"")
      _warn "モーションコントローラの状態を判定できませんでした。
     そのまま起動しますが、腕が動かない場合は次で状態を確認してください:
       python3 tools/mode_check.py --iface $G1_WIRED_IFACE"
      ;;
    debug)
      if [ "$want" = "1" ]; then
        _die "ロボットはデバッグ状態（モーションコントローラ停止）です。
     この状態では rt/arm_sdk は効きません。制御権を渡す相手が居ないためです。
     --motion を外して実行してください:
       ./scripts/teleop.sh${TASK:+ --task $TASK}
     ai モードに戻したい場合:
       python3 tools/mode_check.py --iface $G1_WIRED_IFACE --restore"
      fi
      _ok "デバッグ状態 → rt/lowcmd を使います（--motion なしで正しい）"
      ;;
    *)
      if [ "$want" != "1" ]; then
        _die "モーションコントローラ '$name' が稼働中です。
     この状態では rt/lowcmd は無視されます（'$name' が 1000Hz で上書きし続けるため）。
     どちらかを選んでください:
       (a) '$name' を解除してデバッグ状態にする（確実。検証済みの経路）
           python3 tools/mode_check.py --iface $G1_WIRED_IFACE --release
           ⚠️ 解除するとバランス制御が止まります。必ず支持された状態で。
           ⚠️ 解除後はゼロトルクに落ちることがあります。動かなければリモコンでダンピングへ。
       (b) --motion を付けて rt/arm_sdk を使う（'$name' から腕の制御権を借りる）
           ⚠️ 機体によっては委譲に応じません（検証機では 0.000 rad）。
              効くかは python3 tools/armsdk_probe.py --iface $G1_WIRED_IFACE で確認できます。
           ⚠️ --motion はサムスティックで歩行指令も送ります。座位・吊り下げでは注意。"
      fi
      _ok "'$name' 稼働中 → rt/arm_sdk を使います（--motion ありで正しい）"
      _warn "rt/arm_sdk は機体によって委譲に応じません。腕が動かなければ
     python3 tools/armsdk_probe.py --iface $G1_WIRED_IFACE で確認し、
     駄目なら --motion を外して mode_check.py --release で進めてください。"
      ;;
  esac
}

# 腕のモータが有効か確認する。
#
# ロボットが FSM 0（ゼロトルク）だと rt/lowstate の motor_state[].mode が 0 になり、
# rt/lowcmd も rt/arm_sdk も物理的に効かない。送信は成功と表示されるので気づきにくい。
# 指令メッセージ側で motor_cmd[].mode=1 を送っても有効にはならず、有効化はロボットの
# FSM 側（リモコンでダンピング = FSM 1）でしか行えない。しかも **テレオペや再生を
# 終了するたびにゼロトルクへ戻る** ので、実行のたびに確認する。
require_motors_enabled() {
  local st
  st="$(python3 "$REPO_DIR/tools/mode_check.py" --iface "$G1_WIRED_IFACE" --print-motors 2>/dev/null | tail -1)"
  case "$st" in
    enabled)  _ok "腕モータ有効（mode=1）" ;;
    disabled)
      _die "腕のモータが無効です（rt/lowstate の motor_state[].mode = 0 = ゼロトルク）。
     この状態では指令を送っても腕は動きません（送信は成功と表示されます）。
     リモコンでダンピング（FSM 1）に入れてから実行してください。
     ※ テレオペや再生を終了するたびにゼロトルクへ戻るため、毎回必要です。
     確認: python3 tools/mode_check.py --iface $G1_WIRED_IFACE" ;;
    *)
      _warn "腕モータの状態を判定できませんでした（rt/lowstate 未受信）。
     そのまま続行しますが、動かない場合はリモコンでダンピングに入れてください。" ;;
  esac
}

# ---------------------------------------------------------------------------
# ROS 2 スタック（LiDAR）
# ---------------------------------------------------------------------------
use_ros() {
  # conda の python が ROS の python と衝突するので PATH から外す
  if [[ "$PATH" == *conda* || "$PATH" == *miniforge* ]]; then
    PATH="$(echo "$PATH" | tr ':' '\n' | grep -v -E 'conda|miniforge' | paste -sd:)"
    export PATH
  fi
  unset PYTHONPATH CONDA_PREFIX CONDA_DEFAULT_ENV

  [ -f /opt/ros/humble/setup.bash ] \
    || _die "ROS 2 Humble が見つかりません。setup/install_apt.sh を実行してください。"
  # shellcheck disable=SC1091
  source /opt/ros/humble/setup.bash

  [ -f "$HOME/ws_livox/install/setup.bash" ] \
    || _die "Livox ドライバが未ビルドです。setup/install_livox.sh を実行してください。"
  # shellcheck disable=SC1091
  source "$HOME/ws_livox/install/setup.bash"

  # Livox SDK2 を sudo なしで ~/.local に入れているため、実行時に見つけさせる
  export LD_LIBRARY_PATH="$HOME/.local/lib:${LD_LIBRARY_PATH:-}"
}

# ---------------------------------------------------------------------------
# conda 環境 tv（テレオペ / リプレイ）
# ---------------------------------------------------------------------------
use_tv() {
  local hook="$HOME/miniforge3/etc/profile.d/conda.sh"
  [ -f "$hook" ] || hook="$HOME/miniconda3/etc/profile.d/conda.sh"
  [ -f "$hook" ] || _die "conda が見つかりません。setup/install_env.sh を実行してください。"
  # shellcheck disable=SC1090
  source "$hook"
  conda activate tv 2>/dev/null \
    || _die "conda 環境 'tv' がありません。setup/install_env.sh を実行してください。"
  export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
}

use_tv_quiet() { use_tv > /dev/null 2>&1; }

# ---------------------------------------------------------------------------
# Quest 接続先 URL
# ---------------------------------------------------------------------------
# Quest と同じ WiFi に繋がっているインターフェースを推測する
detect_wifi_iface() {
  if [ -n "$G1_WIFI_IFACE" ]; then echo "$G1_WIFI_IFACE"; return; fi
  local dev
  dev="$(nmcli -t -f DEVICE,TYPE,STATE dev status 2>/dev/null \
         | awk -F: '$2=="wifi" && $3=="connected" {print $1; exit}')"
  [ -n "$dev" ] || dev="$(ls /sys/class/net/*/wireless -d 2>/dev/null | head -1 | cut -d/ -f5)"
  echo "$dev"
}

print_quest_url() {
  local dev ip
  dev="$(detect_wifi_iface)"
  if [ -z "$dev" ]; then
    _warn "WiFi インターフェースが見つかりません。Quest と同じ WiFi に繋いでください。"
    return
  fi
  ip="$(ip -4 -br addr show "$dev" 2>/dev/null | grep -oP '\d+\.\d+\.\d+\.\d+(?=/)' | head -1)"
  if [ -z "$ip" ]; then
    _warn "$dev に IPv4 アドレスがありません（WiFi 未接続 / 認証未通過）。"
    return
  fi
  echo "  Quest のブラウザで開く URL:"
  echo "    https://${ip}:8012/?ws=wss://${ip}:8012"
}
