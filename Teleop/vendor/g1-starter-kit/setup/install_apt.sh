#!/usr/bin/env bash
# 【要 root / 1 回だけ】apt で入れるものをまとめて導入する。
#
#   bash setup/install_apt.sh
#
# ⚠️ VSCode の統合ターミナルでは sudo が通らない環境があります（no_new_privs）。
#    その場合はデスクトップのネイティブ端末（Ctrl+Alt+T）で実行してください。
#
# ⚠️ 長いコマンドを手でコピペすると、端末の設定によって先頭に `^[[200~` が
#    混ざって失敗することがあります。だからスクリプトにしてあります。
#    実行するときも「1 行を手で打つ」形にしてください。
#
# 入れるもの:
#   ビルド一式 / ROS 2 Humble desktop / PlotJuggler / PCL（Livox ドライバ用）
# ダウンロード量は合計 3GB 程度。回線次第で 15〜40 分かかります。

set -euo pipefail

log() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()  { printf '\033[32m[OK]\033[0m %s\n' "$*"; }
die() { printf '\n\033[31m[中断]\033[0m %s\n' "$*" >&2; exit 1; }

log "実行環境を確認"

if [ "$(grep -c '^NoNewPrivs:.*1' /proc/self/status || true)" -ne 0 ]; then
  die "この端末は no_new_privs=1 なので sudo が使えません（VSCode 配下の端末など）。
     デスクトップのネイティブ端末（Ctrl+Alt+T）で実行し直してください。
     確認: grep NoNewPrivs /proc/self/status  →  0 なら OK"
fi
ok "sudo が使える端末"

sudo -v || die "sudo に失敗しました。"

. /etc/os-release
[ "${VERSION_CODENAME:-}" = "jammy" ] || die "Ubuntu 22.04 (jammy) 専用です。検出: ${VERSION_CODENAME:-不明}
     xr_teleoperate は 20.04 / 22.04 でのみ検証されています。24.04 では動作しません。"
ok "Ubuntu 22.04 jammy"

avail_gb=$(df --output=avail -BG / | tail -1 | tr -dc '0-9')
[ "$avail_gb" -ge 12 ] || die "ディスク空きが ${avail_gb}GB しかありません（12GB 以上必要）。"
ok "ディスク空き ${avail_gb}GB"

# ---------------------------------------------------------------------------
log "基本のビルドツールを導入"
sudo apt-get update
sudo apt-get install -y \
  git cmake build-essential pkg-config \
  curl wget ca-certificates gnupg software-properties-common \
  net-tools iputils-ping openssl python3-pip

# ---------------------------------------------------------------------------
log "ROS 2 のリポジトリを登録"
sudo add-apt-repository -y universe

# 公式の新方式（ros2-apt-source パッケージ）。取得に失敗したら旧方式に落とす。
if ROS_APT_SOURCE_VERSION=$(curl -fsSL --max-time 30 \
      https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
      | grep -F '"tag_name"' | awk -F'"' '{print $4}') \
   && [ -n "$ROS_APT_SOURCE_VERSION" ] \
   && curl -fsSL --max-time 60 -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.${VERSION_CODENAME}_all.deb"
then
  sudo apt-get install -y /tmp/ros2-apt-source.deb
  ok "ros2-apt-source ${ROS_APT_SOURCE_VERSION}"
else
  printf '\033[33m[代替]\033[0m 旧方式（鍵の直置き）に切り替えます\n'
  sudo curl -fsSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu ${VERSION_CODENAME} main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
  ok "旧方式で登録"
fi

sudo apt-get update
apt-cache show ros-humble-desktop > /dev/null 2>&1 \
  || die "ros-humble-desktop が見つかりません。リポジトリ登録に失敗しています。"

# ---------------------------------------------------------------------------
log "ROS 2 Humble 一式を導入（3GB 程度・時間がかかります）"
sudo apt-get install -y \
  ros-humble-desktop \
  ros-dev-tools \
  ros-humble-plotjuggler-ros \
  ros-humble-pcl-conversions \
  libpcl-dev

# ---------------------------------------------------------------------------
log "検証"
missing=()
for p in ros-humble-desktop ros-dev-tools ros-humble-plotjuggler-ros \
         ros-humble-rviz2 ros-humble-pcl-conversions libpcl-dev; do
  if dpkg-query -W -f='${Status}' "$p" 2>/dev/null | grep -q 'install ok installed'; then
    ok "$p"
  else
    missing+=("$p")
  fi
done
[ ${#missing[@]} -eq 0 ] || die "未導入が残っています: ${missing[*]}"

# ROS の setup.bash は未定義変数を参照するので set -u を外してから読む
set +u
# shellcheck disable=SC1091
source /opt/ros/humble/setup.bash
ok "ROS_DISTRO=${ROS_DISTRO}"
command -v rviz2 >/dev/null && ok "rviz2"
command -v colcon >/dev/null && ok "colcon"
ros2 pkg executables plotjuggler >/dev/null 2>&1 \
  && ok "plotjuggler（起動は ros2 run plotjuggler plotjuggler）"
set -u

log "完了。次は root 不要の手順です"
cat <<'EOF'
  bash setup/install_env.sh     # conda 環境・xr_teleoperate・DDS
  bash setup/install_livox.sh   # LiDAR ドライバ（点群を見る場合）
EOF
