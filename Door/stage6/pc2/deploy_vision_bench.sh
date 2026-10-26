#!/usr/bin/env bash
# **Mac の上で**動かす。段 6 (b) 映像の生徒の書き出し（OMEN にある）と計測スクリプトを PC2 の ~/door_real/stage6_va/ へ配る。
# Mac から機体へは直に入れないので OMEN を踏み台にする（ssh -J）。OMEN には PC2 の鍵が無いので、書き出しは
# OMEN → Mac → PC2 と tar で流す（Mac には残さない）。
#
#   bash Door/stage6/pc2/deploy_vision_bench.sh check    # 配らずに PC2 の環境（Python・torch・cv2・TensorRT・電源の設定）を見るだけ
#   bash Door/stage6/pc2/deploy_vision_bench.sh          # 配って md5 で確かめる
#   bash Door/stage6/pc2/deploy_vision_bench.sh run      # 配った後、PC2 で bench_vision_student.sh を回して結果を Mac へ持ち帰る
#
# 接続先: G1_JUMP（既定 ubuntu@100.99.102.70）・G1_PC2_SSH（既定 g1wifi＝~/.ssh/config の 192.168.0.82）。G1_JUMP= で直に入る
# 書き出しの場所: EXPORT（既定 OMEN の ~/work_inoue/stage6_va/export_rand）
# 予行演習（OMEN を PC2 に見立てる）: G1_JUMP= G1_PC2_SSH=ubuntu@100.99.102.70 DEST=work_inoue/stage6_va/fake_pc2 \
#   PC2_PY=/home/ubuntu/miniforge3/envs/doorman6/bin/python bash deploy_vision_bench.sh run
# ⚠️ 機体は動かさない（DDS も触らない）。ただし機体の電源（PC2）は入っている必要がある
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
OMEN="${OMEN:-ubuntu@100.99.102.70}"
G1_JUMP="${G1_JUMP-ubuntu@100.99.102.70}"
G1_PC2_SSH="${G1_PC2_SSH:-g1wifi}"
EXPORT="${EXPORT:-work_inoue/stage6_va/export_rand}"  # OMEN の $HOME 相対
DEST="${DEST:-door_real/stage6_va}"                   # PC2 の $HOME 相対（予行演習で OMEN を PC2 に見立てるときだけ変える）
NAME="$(basename "$EXPORT")"
say() { echo "[deploy6] $*"; }
omen() { ssh -o ConnectTimeout=40 -o BatchMode=yes "$OMEN" "$@"; }
pc2() { ssh -o ConnectTimeout=40 -o BatchMode=yes ${G1_JUMP:+-J "$G1_JUMP"} "$G1_PC2_SSH" "$@"; }

# ⚠️ -o BatchMode・ConnectTimeout は -J の踏み台の側には効かない。踏み台で止まらないよう、先に踏み台へ鍵だけで入れるかを見る
if ! omen true 2>/dev/null; then
  say "OMEN（${OMEN}）に鍵だけで入れない（Tailscale・鍵を確かめる）。踏み台にも使うので先へ進めない"
  exit 1
fi
if ! pc2 true 2>/dev/null; then
  say "PC2（${G1_PC2_SSH}、踏み台 ${G1_JUMP:-なし}）に入れない。機体の電源・Wi-Fi（Physical_AI_5G）・OMEN の Tailscale を確かめる"
  exit 1
fi
say "PC2 OK: $(pc2 hostname)"

check_env() {
  say "PC2 の環境（読むだけ）"
  pc2 'PY=$HOME/miniforge3/envs/lerobot/bin/python
    echo "  lerobot: $($PY -c "import sys, torch; print(sys.version.split()[0], \"torch\", torch.__version__, \"cuda\", torch.cuda.is_available(), \"threads\", torch.get_num_threads())" 2>&1 | tail -1)"
    echo "  cv2: $($PY -c "import cv2; print(cv2.__version__)" 2>&1 | tail -1)"
    echo "  onnxruntime: $($PY -c "import onnxruntime as o; print(o.__version__, o.get_available_providers())" 2>&1 | tail -1)"
    echo "  trtexec: $(command -v trtexec || ls /usr/src/tensorrt/bin/trtexec 2>/dev/null || echo 無い)"
    echo "  tensorrt: $(dpkg -l 2>/dev/null | awk "/^ii +(tensorrt|libnvinfer8) /{print \$2, \$3}" | tr "\n" " ")"
    echo "  python3 の tensorrt: $(python3 -c "import tensorrt; print(tensorrt.__version__)" 2>&1 | tail -1)"
    nv="$(nvpmodel -q 2>/dev/null | tr "\n" " ")"; echo "  nvpmodel: ${nv:-取れない（無いか root が要る）}"
    echo "  使えるコア: $(cat /sys/devices/system/cpu/online 2>/dev/null)（nproc $(nproc)）"
    echo "  L4T: $(head -1 /etc/nv_tegra_release 2>/dev/null)"
    echo "  負荷: $(cut -d" " -f1-3 /proc/loadavg)・空き $(free -m | awk "/Mem:/{print \$7}") MB・ディスク $(df -h ~ | awk "NR==2{print \$4}")"'
}

if [ "${1:-}" = check ]; then
  check_env
  exit 0
fi

omen "test -f $EXPORT/meta.json" || { say "OMEN に書き出しが無い: ~/$EXPORT"; exit 1; }
pc2 "mkdir -p ~/$DEST/pc2 ~/$DEST/$NAME" || exit 1
say "書き出しを流す（OMEN ~/${EXPORT} → PC2 ~/${DEST}/${NAME}、約 140 MB）"
t0=$(date +%s)
omen "tar -C $EXPORT --exclude='bench_*' -cf - ." | pc2 "tar -C ~/$DEST/$NAME -xf -" || { say "転送に失敗"; exit 1; }
say "転送 $(( $(date +%s) - t0 )) 秒"
scp -q -o ConnectTimeout=40 ${G1_JUMP:+-J "$G1_JUMP"} "$HERE/bench_vision_student.py" "$HERE/bench_vision_student.sh" \
  "$G1_PC2_SSH:$DEST/pc2/" || { say "scp に失敗"; exit 1; }

# md5 を meta.json（書き出した時の値）と突き合わせる。スクリプトは Mac の手元と突き合わせる
# ⚠️ ループの中で ssh を呼ぶと、ssh が while の標準入力（残りの行）を食べて 1 回で終わる。先に全部取ってから比べる
want=$(omen "python3 -c 'import json,sys; [print(v, k) for k, v in json.load(open(sys.argv[1]))[\"md5\"].items()]' $EXPORT/meta.json")
want+=$'\n'"$(md5 -q "$HERE/bench_vision_student.py") ../pc2/bench_vision_student.py"
want+=$'\n'"$(md5 -q "$HERE/bench_vision_student.sh") ../pc2/bench_vision_student.sh"
got=$(pc2 "cd ~/$DEST/$NAME && md5sum $(awk '{print $2}' <<< "$want" | tr '\n' ' ')" < /dev/null)
bad=0
while read -r sum f; do
  if grep -Fqx "$sum  $f" <<< "$got"; then say "md5 一致: ${f#../pc2/}"; else say "⚠️ md5 不一致: ${f}"; bad=1; fi
done <<< "$want"
[ "$bad" = 0 ] || exit 1
check_env

if [ "${1:-}" = run ]; then
  # 出力先の名前をこちらで決めて渡す（「いちばん新しい bench_*」を拾うと、今回が失敗したとき前回の結果を持ち帰ってしまう）
  TAG="$(date +%m%d_%H%M%S)"
  say "PC2 で計測する（数分。TensorRT があればエンジン作りにさらに数分）。出力 bench_${TAG}"
  pc2 "BENCH_TAG=$TAG PY=${PC2_PY:-} bash ~/$DEST/pc2/bench_vision_student.sh ~/$DEST/$NAME"
  rc=$?
  LOCAL="$HERE/../../real/logs_real/stage6_bench"
  mkdir -p "$LOCAL"
  if ! pc2 "test -f ~/$DEST/$NAME/bench_$TAG/env.txt"; then
    say "⚠️ 今回の結果のフォルダ（bench_${TAG}）が PC2 に無い。計測が始まる前に落ちた"
    exit 1
  fi
  if pc2 "tar -C ~/$DEST/$NAME --exclude='*.engine' -cf - bench_$TAG" | tar -C "$LOCAL" -xf -; then
    say "結果を持ち帰った: $LOCAL/bench_${TAG}"
  else
    say "⚠️ 結果の持ち帰りに失敗（PC2 の ~/$DEST/$NAME/bench_${TAG} に残っている）"; rc=1
  fi
  [ "$rc" = 0 ] || say "⚠️ 計測のどこかが失敗したか golden が合わなかった（終了コード ${rc}）。summary.txt と t*.log を見る"
  exit "$rc"
fi
