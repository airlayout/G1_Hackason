#!/usr/bin/env bash
# 案 A（2026-10-26）: PC2（Orin NX・Ubuntu 20.04・arm64）に遠隔操作の環境を作る。root 不要・何度流してもよい。
# Mac の scripts/pc2_teleop.sh setup から配られ、切り離して流される（ログは ~/teleop_logs/setup_*.log）。
#
# 作るもの（既存の物には書き込まない）:
#   ~/miniforge3/envs/tv   … 既存の miniforge に環境を 1 つ足す（base・lerobot には触らない）
#   ~/xr_teleoperate       … 817fb00＋サブモジュール televuer・teleimager（手の部品 dex-retargeting は入れない）
#   televuer の自己署名証明書
#   キットの修正（~/teleop_tools/apply_patches.py）
# 触らないもの: ~/.bashrc（conda init もしない）・~/unitree_sdk2_python の中（.pth でつなぐだけ）・~/cyclonedds（0.10.5）
#
# 決めたこと（2026-10-27 に PC2 で確かめた）:
#   - DDS の C の部品は機械全体の /usr/local（0.10.2）を使う。既存の Python 3.8 の cyclonedds もこれにつながって動いている。
#     ~/cyclonedds は 0.10.5 で、キットが求める 0.10.2 と合わない
#   - cyclonedds 0.10.2 は arm64 向けの既製の配布物が無いので、/usr/local に対して組み立てる
#   - 部品の版は OMEN の動いている環境から作った tv_constraints.txt に固定する（numpy 2 を引き込ませない）
set -euo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
CONDA="$HOME/miniforge3/bin/conda"
ENV="$HOME/miniforge3/envs/tv"
PY="$ENV/bin/python"
XR="$HOME/xr_teleoperate"
REV="817fb00"
CONSTRAINTS="$HERE/tv_constraints.txt"

log() { echo "[setup $(date +%H:%M:%S)] $*"; }
die() { echo "[setup $(date +%H:%M:%S)] ✗ $*"; exit 1; }
pip_c() { "$PY" -m pip install -q --disable-pip-version-check -c "$CONSTRAINTS" "$@"; }

[ -x "$CONDA" ] || die "~/miniforge3 が無い"
[ -f "$CONSTRAINTS" ] || die "版の固定表が無い: ${CONSTRAINTS}"
[ "$(uname -m)" = aarch64 ] || die "arm64 の PC2 向け（今は $(uname -m)）"

# ---------------------------------------------------------------------------
if [ -x "$PY" ]; then
  log "[済] conda 環境 tv"
else
  log "conda 環境 tv を作る（python 3.10・pinocchio 3.1.0・numpy 1.26.4。数分かかる）"
  "$CONDA" create -y -q -n tv python=3.10 pinocchio=3.1.0 numpy=1.26.4 -c conda-forge
fi
"$PY" -c "import pinocchio, numpy, casadi; print('[setup] pinocchio', pinocchio.__version__, '・numpy', numpy.__version__, '・casadi', casadi.__version__)"

# ---------------------------------------------------------------------------
if [ -d "$XR/.git" ]; then
  log "[済] ~/xr_teleoperate"
else
  log "xr_teleoperate を取る"
  git clone -q https://github.com/unitreerobotics/xr_teleoperate.git "$XR"
fi
if [ "$(git -C "$XR" rev-parse --short=7 HEAD)" != "$REV" ]; then
  [ -z "$(git -C "$XR" status --short --untracked-files=no)" ] || die "~/xr_teleoperate に手元の変更がある。版を動かす前に確かめる"
  git -C "$XR" fetch -q origin
  git -C "$XR" checkout -q "$REV"
fi
# 手の部品（dex-retargeting）は取らない: torch 2.3.0 と pip の pin を引き込み conda の pinocchio とぶつかる。手なしでは読み込まれない
git -C "$XR" submodule update --init -q teleop/televuer teleop/teleimager
log "xr_teleoperate $(git -C "$XR" rev-parse --short=7 HEAD)・televuer $(git -C "$XR/teleop/televuer" rev-parse --short=7 HEAD)・teleimager $(git -C "$XR/teleop/teleimager" rev-parse --short=7 HEAD)"

# ---------------------------------------------------------------------------
log "部品を入れる（版は tv_constraints.txt）"
pip_c -e "$XR/teleop/teleimager" --no-deps
pip_c logging_mp pyyaml pyzmq opencv-python          # teleimager の依存（--no-deps で入れたので別に）
pip_c -e "$XR/teleop/televuer"
pip_c -r "$XR/requirements.txt"
pip_c "params_proto==2.13.2"                         # vuer 0.0.60 は params_proto 2.x の API を使う

if "$PY" -c "import cyclonedds, importlib.metadata as m; assert m.version('cyclonedds') == '0.10.2'" 2> /dev/null; then
  log "[済] cyclonedds 0.10.2"
else
  # 2026-10-27: /usr/local（0.10.2）には動かす部品しか無く、組み立て用のヘッダ（idl/retcode.h）が無かった。
  # ~/cyclonedds は 0.10.5。そこで 0.10.2 を新しい別のフォルダに組み立て、そのヘッダで Python の部品を組み立てる。
  # 動かすときは機械全体の /usr/local/lib の 0.10.2（Python 3.8 側と同じ物）につながるので版がそろう
  CDDS_SRC="$HOME/cyclonedds_0102"
  CDDS_HOME="$CDDS_SRC/install"
  if [ ! -f "$CDDS_HOME/include/idl/retcode.h" ]; then
    log "CycloneDDS の C の部品 0.10.2 を ~/cyclonedds_0102 に組み立てる（5〜10 分）"
    [ -d "$CDDS_SRC/.git" ] || git clone -q --depth 1 --branch 0.10.2 https://github.com/eclipse-cyclonedds/cyclonedds.git "$CDDS_SRC"
    cmake -S "$CDDS_SRC" -B "$CDDS_SRC/build" -DCMAKE_INSTALL_PREFIX="$CDDS_HOME" -DCMAKE_BUILD_TYPE=Release \
      -DENABLE_SHM=OFF -DBUILD_TESTING=OFF > /dev/null
    cmake --build "$CDDS_SRC/build" --target install -j"$(nproc)" > /dev/null
  fi
  log "cyclonedds 0.10.2（Python の部品）を ~/cyclonedds_0102 のヘッダで組み立てる"
  CYCLONEDDS_HOME="$CDDS_HOME" pip_c "cyclonedds==0.10.2"
fi
CLAYER="$("$PY" -c 'import cyclonedds, os, glob; print(glob.glob(os.path.join(os.path.dirname(cyclonedds.__file__), "_clayer*.so"))[0])')"
log "cyclonedds がつながる C の部品: $(ldd "$CLAYER" | grep -o 'libddsc[^ ]* => [^ ]*' | head -1)"

# unitree_sdk2_python は Python 3.8 側と共用のフォルダ。pip -e で中に書き込まないよう .pth でつなぐ
SITE="$("$PY" -c 'import site; print(site.getsitepackages()[0])')"
echo "$HOME/unitree_sdk2_python" > "$SITE/g1h_unitree_sdk2_python.pth"
log "unitree_sdk2_python を .pth でつないだ（$(git -C "$HOME/unitree_sdk2_python" rev-parse --short=7 HEAD)）"

# ---------------------------------------------------------------------------
CERT_DIR="$XR/teleop/televuer"
if [ -f "$CERT_DIR/cert.pem" ] && [ -f "$CERT_DIR/key.pem" ]; then
  log "[済] 証明書"
else
  openssl req -x509 -nodes -days 3650 -newkey rsa:2048 \
    -keyout "$CERT_DIR/key.pem" -out "$CERT_DIR/cert.pem" -subj "/CN=g1-pc2-teleop" 2> /dev/null
  chmod 600 "$CERT_DIR/key.pem"
  log "証明書を作った（Quest では初回に警告が出るので「詳細設定→アクセスする」で進む）"
fi

# ---------------------------------------------------------------------------
log "キットの修正を当てる"
"$PY" "$HERE/apply_patches.py" | grep -E "✅|⚠️|❌|変更していません|完了" || true
"$PY" "$HERE/apply_patches.py" --dry-run > /tmp/g1h_patch_check.txt || die "修正が全部は当たっていない: $(grep -E '⚠️|❌' /tmp/g1h_patch_check.txt | head -3)"
log "修正 $(grep -c '既に適用済み' /tmp/g1h_patch_check.txt) か所が適用済み"

log "確かめ"
cd "$XR/teleop"
"$PY" - << 'PY'
import importlib
bad = []
for m in ["pinocchio", "casadi", "vuer", "televuer", "teleimager", "cyclonedds", "unitree_sdk2py",
          "rerun", "meshcat", "sshkeyboard", "logging_mp", "zmq", "cv2"]:
    try:
        importlib.import_module(m)
    except Exception as e:  # noqa: BLE001
        bad.append(f"{m}: {e}")
import numpy
print("[setup] numpy", numpy.__version__, "（2 未満であること）")
if bad:
    raise SystemExit("[setup] ✗ 読み込めない: " + " / ".join(bad))
print("[setup] ✓ 部品は全部読み込めた")
PY
"$PY" "$HERE/stick_pause_selftest.py" | tail -1
"$PY" "$HERE/record_timestamp_selftest.py" | tail -1
"$PY" "$HERE/go_home_selftest.py" | tail -1
"$PY" "$HERE/arm_effort_selftest.py" | tail -1
log "✓ 終わり"
