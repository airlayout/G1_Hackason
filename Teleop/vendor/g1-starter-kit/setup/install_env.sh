#!/usr/bin/env bash
# 【root 不要】テレオペに必要な環境を作る。
#
#   bash setup/install_env.sh
#
# 作るもの:
#   conda 環境 `tv`（python 3.10 / pinocchio 3.1.0 / numpy 1.26.4）
#   ~/xr_teleoperate       … テレオペ本体（Unitree 公式）
#   ~/cyclonedds           … DDS の C 実装（0.10.2）
#   ~/unitree_sdk2_python  … ロボットとの通信
#   Quest 用の自己署名証明書
#
# バージョンは固定です。**変更しないでください**:
#   xr_teleoperate はこの組み合わせでのみ検証されており、
#   pinocchio や numpy を上げると逆運動学が動かなくなります。

set -euo pipefail

# このスクリプトは途中で cd するので、apply_patches.py の場所を最初に絶対パスで押さえる。
# （相対パスのまま末尾で dirname すると、直前の cd "$HOME/cyclonedds" の影響で失敗する）
SETUP_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

log() { printf '\n\033[1;36m==> %s\033[0m\n' "$*"; }
ok()  { printf '\033[32m[OK]\033[0m %s\n' "$*"; }
skip(){ printf '\033[90m[済]\033[0m %s\n' "$*"; }
die() { printf '\n\033[31m[中断]\033[0m %s\n' "$*" >&2; exit 1; }

# ---------------------------------------------------------------------------
log "conda を用意"
CONDA_HOOK=""
for h in "$HOME/miniforge3/etc/profile.d/conda.sh" "$HOME/miniconda3/etc/profile.d/conda.sh"; do
  [ -f "$h" ] && CONDA_HOOK="$h" && break
done

if [ -z "$CONDA_HOOK" ]; then
  log "Miniforge を導入（~/miniforge3）"
  cd "$HOME"
  wget -q --show-progress \
    "https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh" \
    -O Miniforge3-Linux-x86_64.sh
  bash Miniforge3-Linux-x86_64.sh -b -p "$HOME/miniforge3"
  CONDA_HOOK="$HOME/miniforge3/etc/profile.d/conda.sh"
  # shellcheck disable=SC1090
  source "$CONDA_HOOK"
  conda init bash
  ok "Miniforge を導入しました（新しいターミナルから conda が使えます）"
else
  skip "conda は既にあります: $CONDA_HOOK"
fi
# shellcheck disable=SC1090
source "$CONDA_HOOK"

# ---------------------------------------------------------------------------
log "conda 環境 tv を作成（バージョン固定）"
if conda env list | grep -qE '^tv\s'; then
  skip "環境 tv は既にあります"
else
  conda create -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge -y
  ok "環境 tv を作成しました"
fi
conda activate tv
ok "python $(python --version 2>&1 | cut -d' ' -f2)"

# ---------------------------------------------------------------------------
log "xr_teleoperate を取得"
if [ -d "$HOME/xr_teleoperate/.git" ]; then
  skip "~/xr_teleoperate は既にあります"
else
  git clone https://github.com/unitreerobotics/xr_teleoperate.git "$HOME/xr_teleoperate"
fi
cd "$HOME/xr_teleoperate"
git submodule update --init --depth 1
for d in teleop/teleimager teleop/televuer teleop/robot_control/dex-retargeting; do
  [ -n "$(ls -A "$d" 2>/dev/null)" ] || die "サブモジュールが空です: $d"
done
ok "本体とサブモジュール"

# teleimager は公式手順どおり --no-deps で入れる（依存解決を任せると環境が壊れる）
log "サブモジュールを導入"
pip install -q -e "$HOME/xr_teleoperate/teleop/teleimager" --no-deps
pip install -q -e "$HOME/xr_teleoperate/teleop/televuer"
pip install -q -e "$HOME/xr_teleoperate/teleop/robot_control/dex-retargeting"
pip install -q -r "$HOME/xr_teleoperate/requirements.txt"

# vuer 0.0.60 は params_proto 2.x の API を使う。自動解決だと 3.x が入って
# `ImportError: cannot import name 'Vuer'` になるので明示的に固定する。
pip install -q 'params_proto==2.13.2'
ok "依存パッケージ（params_proto は 2.13.2 に固定）"

# ---------------------------------------------------------------------------
log "Quest 用の自己署名証明書を生成"
CERT_DIR="$HOME/xr_teleoperate/teleop/televuer"
if [ -f "$CERT_DIR/cert.pem" ] && [ -f "$CERT_DIR/key.pem" ]; then
  skip "証明書は既にあります"
else
  openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \
    -keyout "$CERT_DIR/key.pem" -out "$CERT_DIR/cert.pem" \
    -subj "/CN=g1-starter-kit" 2>/dev/null
  chmod 600 "$CERT_DIR/key.pem"
  ok "生成しました（Quest では初回に警告が出るので「詳細設定→アクセスする」で進む）"
fi

# ---------------------------------------------------------------------------
log "CycloneDDS 0.10.2 をビルド"
# unitree_sdk2_python がビルド時に要求する。バージョンは厳密に一致させること。
if [ -f "$HOME/cyclonedds/install/lib/libddsc.so" ]; then
  skip "~/cyclonedds は既にビルド済み"
else
  if [ ! -d "$HOME/cyclonedds/.git" ]; then
    git clone https://github.com/eclipse-cyclonedds/cyclonedds -b releases/0.10.x "$HOME/cyclonedds"
  fi
  cd "$HOME/cyclonedds"
  git checkout -q tags/0.10.2
  mkdir -p build install && cd build
  cmake .. -DCMAKE_INSTALL_PREFIX=../install -DCMAKE_BUILD_TYPE=Release > /dev/null
  cmake --build . --target install -j"$(nproc)" > /dev/null
  ok "ビルドしました"
fi

# ---------------------------------------------------------------------------
log "unitree_sdk2_python を導入"
# 罠: CYCLONEDDS_HOME に "~" を書くとダブルクォート内で展開されず失敗する。
#     必ず $HOME を使う（公式 issue #121）。
export CYCLONEDDS_HOME="$HOME/cyclonedds/install"
if [ ! -d "$HOME/unitree_sdk2_python/.git" ]; then
  git clone https://github.com/unitreerobotics/unitree_sdk2_python.git "$HOME/unitree_sdk2_python"
fi
pip install -q -e "$HOME/unitree_sdk2_python"
ok "導入しました"

if ! grep -q 'CYCLONEDDS_HOME' "$HOME/.bashrc"; then
  echo 'export CYCLONEDDS_HOME="$HOME/cyclonedds/install"' >> "$HOME/.bashrc"
  ok "~/.bashrc に CYCLONEDDS_HOME を追記しました"
fi

# ---------------------------------------------------------------------------
log "検証"
python - <<'PY'
import importlib, sys
bad = []
for mod, want in (("numpy", "1.26.4"), ("pinocchio", "3.1.0")):
    m = importlib.import_module(mod)
    got = getattr(m, "__version__", "?")
    print(f"  {mod:12} {got}" + ("" if got == want else f"   ← 期待は {want}"))
    if got != want:
        bad.append(mod)
for mod in ("cyclonedds", "unitree_sdk2py", "vuer", "televuer", "teleimager"):
    try:
        importlib.import_module(mod)
        print(f"  {mod:12} OK")
    except Exception as e:
        print(f"  {mod:12} 失敗 ({type(e).__name__}: {e})")
        bad.append(mod)
sys.exit(1 if bad else 0)
PY
ok "python 側の import はすべて通りました"

log "テレオペ本体に必要な修正を当てる"
python "$SETUP_DIR/apply_patches.py"

log "完了"
cat <<'EOF'
  次にやること:

    1. cp config/g1.env.example config/g1.env  してネットワーク設定を書く
       （docs/02-network.md）
    2. ./scripts/preflight.sh   でロボットとの疎通と機体判定
    3. ./scripts/teleop.sh      でテレオペ開始

  LiDAR の点群も見る場合は bash setup/install_livox.sh も実行してください。
EOF
