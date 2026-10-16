#!/usr/bin/env bash
# DoorMan を Isaac Sim 6.0.1 ＋ Isaac Lab 3.0（release/3.0.0-beta2）で動かす環境を OMEN に作る。
#
# なぜ 6.0.1 か: OMEN のドライバ 595.91 で Isaac Sim 5.1 のカメラの描画が落ちる（5.1 の検証済みは 580.65.06）。
# 6.0.1 の検証済みは 595.58.03 で、チームメイトの環境（~/NVIDIA/env_isaaclab）も 6.0.1 で動いている。
#
# チームメイトの環境には触らない。主要な版（torch・numpy・warp・transformers・newton など 16 個）だけ、
# その pip freeze から写した制約ファイル（constraints_from_teammate.txt）で揃える。
# ⚠️ freeze を丸ごと制約にすると解けない（10-07 実測: チームメイトの aiohttp 3.13.3 に対し isaacsim-kernel 6.0.1 は 3.13.4 を要求）。
# ⚠️ torch はチームメイトの 2.10.0 ではなく、isaacsim-core 6.0.1 が固定で要求する 2.11.0・torchvision 0.26.0・torchaudio 2.11.0（cu128）にする。
# Isaac Lab はチームメイトと同じ commit を別に取る。
#
# 置き場所: ~/work_inoue/doorman6/{IsaacLab,VIRAL6}、conda 環境: doorman6（Python 3.12）
# 使い方（OMEN で）: ./setup_doorman6.sh > setup6.log 2>&1   # 完了の目印は SETUP6_DONE
set -euo pipefail

BASE="$HOME/work_inoue/doorman6"
OLD="$HOME/work_inoue/doorman"           # 5.1 版（VIRAL・LAFAN-G1・パッチ）
ENV_NAME="doorman6"
CONDA="$HOME/miniforge3/bin/conda"
ENV_BIN="$HOME/miniforge3/envs/$ENV_NAME/bin"
ISAACLAB_REF="99f1423e"                   # release/3.0.0-beta2（チームメイトと同じ。2026-09-04）
DOORMAN_COMMIT="016c70c1e4e76f521963c36691ee69a6ab3ac9cd"
CU128="https://download.pytorch.org/whl/cu128"
C="-c $BASE/constraints_from_teammate.txt"

step() { echo "=== [$(date +%T)] $*"; }
torch_check() {
  "$ENV_BIN/python" -c "import torch, sys; v=torch.__version__; print('torch', v, 'cuda', torch.cuda.is_available()); sys.exit(0 if v.startswith('2.11.0+cu128') and torch.cuda.is_available() else 1)"
}

cd "$BASE"
[ -s constraints_from_teammate.txt ] || { echo "constraints_from_teammate.txt が無い"; exit 1; }

step "1. conda 環境 $ENV_NAME（Python 3.12・git-lfs）"
ENVS="$("$CONDA" env list)"
if ! grep -q "^$ENV_NAME " <<< "$ENVS"; then
  "$CONDA" create -y -n "$ENV_NAME" -c conda-forge python=3.12 git-lfs
fi
export PATH="$ENV_BIN:$PATH"
PIP="$ENV_BIN/python -m pip"
$PIP install -q --upgrade pip

step "2. PyTorch 2.11.0+cu128（isaacsim-core 6.0.1 が固定で要求する版）"
$PIP install torch==2.11.0 torchvision==0.26.0 torchaudio==2.11.0 --index-url "$CU128"
torch_check

step "3. Isaac Sim 6.0.1（all・extscache）"
$PIP install $C "isaacsim[all,extscache]==6.0.1.0" --extra-index-url https://pypi.nvidia.com --extra-index-url "$CU128"
torch_check

step "4. Isaac Lab release/3.0.0-beta2 @ $ISAACLAB_REF（source の全パッケージ。チームメイトと同じ構成）"
if [ ! -d IsaacLab ]; then
  git clone -b release/3.0.0-beta2 https://github.com/isaac-sim/IsaacLab.git IsaacLab
fi
git -C IsaacLab checkout -q "$ISAACLAB_REF"
EDITABLE=()
for d in IsaacLab/source/*/; do
  [ -f "$d/setup.py" ] || [ -f "$d/pyproject.toml" ] || continue
  EDITABLE+=(-e "$d")
done
$PIP install $C --extra-index-url "$CU128" --extra-index-url https://pypi.nvidia.com --no-build-isolation "${EDITABLE[@]}"
torch_check

step "5. DoorMan の作業ツリー VIRAL6（5.1 と共通の 0001〜0003・記録係 0005、6.0.1 への移植 0006〜0011、段 2-1 の修正 0012〜0015、段 2-2〜2-4 のドア 0016〜0044、段 2.5 の状態入力の生徒 0045〜0049、段 5 の実機用の確かめ 0050〜0051 を当てる）"
# 0006: Isaac Lab 3.0 への移植（クォータニオンの境界・ProxyArray・API の改名）
# 0007: 拡張の読み込み（omni.kit.material.library を起動時に有効にする。固まる対策）
# 0008: 扉の関節の取付位置を env からの相対で求める（直さないと部品が env の原点ぶんずれ、扉に届かない。10-07 実測）
# 0009: 記録係に扉の部品の位置を足す（5.1 と 6.0.1 の比べ用。どちらの版でも動く）
# 0010: 29 関節のカメラ付き生徒（DAgger）の設定。記録係がカメラの RGB を PNG で書けるように（rgb_every）
# 0011: BehaviorScript を Kit 110 の omni.behavior.scripting.core で使う（生徒のドームライトの乱択）
# 0012: 把手と同じ側の手で取りに行く（lever_side_hand。元は反対の手で体の前を横切り、指なしでは届かなかった）。記録係の区間 1→2 の診断
# 0013: 評価で行動を分布から引く切り替え（eval.sample_actions）。記録係に手と把手の接触の力
# 0014: 構える点を把手の上でなく把手そのものに（pregrasp_height・pregrasp_above_margin。指の無い手は下から当たって上へ回れない）
# 0015: 評価の動画（render_results）のカメラを扉の手前に固定できるように（render_eye・render_target・解像度）
# 0016: 段 2-2〜2-4 のドア（5.1 版の untested/0004 を 6.0.1 へ移したもの。EN 1154 の等級・四角いレバー・ラッチの出・開いたまま止まる角度）
# 0017: 閉じた位置での押し付け（hinge_closed_preload_range）。等級の扉で押し付けがラッチの斜面を押しレバーが勝手に回るのを止める
# 0018: 手でレバーを下へ押す力の報酬（press_door_handle_down。29 関節版で 4.0）。既定のレバーは約 17〜33 N まで全く動かず手がかりが無い
# 0019: 手のひらを当てる点・構える点のずらし（grasp_target_offset・pregrasp_offset。29 関節版は棒の上から押し、手前から上げて回る）
# 0020: レバーを 30° 以上下げている間に前へ押す力の報酬（push_door_unlatched。29 関節版で 6.0）。押し下げだけ覚えて前へ押さなくなったため
# 0021: 押す力の報酬を 4.0 → 1.0（押し付けるだけで満点になる行き止まりになった）
# 0022: 当てる点を棒の中心の上へ（届かない所に置いてレバーを前下へ押させる）。構える点は同じ場所
# 0023: 前へ押す報酬でレバーを手前へ引く力を負に数える（push_door_force_penalize_pull。指の無い手は引きながら押し下げていた）
# 0024: 押す点をレバーの先の向きへずらす（grasp_target_offset_along_lever。29 関節版は 4 cm＝必要な力が約半分）
# 0025: レバーの報酬を角度だけに（push_door_handle_pos_only。小刻みに押し戻す動きで稼げた）
# 0026: 扉の報酬も角度だけに（push_door_hinge_pos_only。押してはクローザーで戻される繰り返しで稼げた）
# 0027: 区間 4 で止まっている罰を -1 → -5（扉 約 20° のまま歩かなかった）
# 0028: 1 回の上限を 20 → 30 秒（腕で扉を 10° 押すのに約 250 ステップかかり、通り抜ける時間が無かった）
# 0029: 区間 3・4 の持ち時間を 100 → 300 ステップ（区間ごとの時間切れで 1 回が約 400 ステップで打ち切られていた）
# 0030: 区間 3 は 100 に戻す（300 だと扉を開けずに区間 3 の報酬を取り続ける方が得になった）
# 0031: 前へ押す力の報酬の頭打ちを設定に（push_door_force_max、既定は元の 20 N。等級 3 以上はレバーの位置で 20 N を超える力が要る）
# 0032: 段 2-3 の学習用の扉 door_en1154_upto_g1〜g5（クローザーの上限を既定 2.5 N·m〜等級 k の上限で混ぜる。⚠️ 混ぜて学習すると等級 1 が 50→5/128 に落ちた）
# 0033: 台ごとのクローザーの上限・ばね・目標角を static.npz に記録する（扉の強さと成否を突き合わせる。学習には使わない）
# 0034: 毎ステップ、手がレバーから受ける力と扉の板が受ける接触の力を記録する（どれだけの力で押しているか。学習には使わない）
# 0035: 段 2-3 の実験 E3 の扉 door_en1154_g4g5（クローザーの上限 26〜54 N·m。等級 4 と 5 を混ぜる）
# 0036: 段 2-3 の案 2 の扉 door_en1154_g4g5_82（台 i % 5 == 4 が等級 5、ほかは等級 4。台の 2 割が等級 5）
# 0037: 段 2-4 の実物の形のクローザー（closer_shaped。閉じ際が最大・90° で 1/3〜1/2・開ける向きは効率ぶん重い・バックチェック。
#       蝶番のドライブを切り、env が毎物理ステップ力を掛ける）。評価用 door_en1154_g1〜g4_shaped・学習用 door_room4_g13。tests/test_door_closer.py
# 0038: 把手が左の扉（door_open_lr=+1＝実物の UIS Room 4 の向き）で観測と行動を鏡写しにする（mirror_left_doors。今の方策は把手が右の扉しか
#       学んでいない）。評価用 door_en1154_g1〜g4_left。tests/test_door_mirror.py
# 0039: 段 2-4a の学習用の扉 door_en1154_g13_shaped（等級 1〜3 を台ごとに 1/3 ずつ・実物の形のクローザー。ほかは段 2-3 の扉のまま）
# 0040: その実物の向き（把手が左）の扉 door_en1154_g13_shaped_left（mirror_left_doors と組で使う）
# 0041: 窓の報酬（push_door_hinge_max_deg で扉を開ける点の頭打ちの角度、door_overopen で開いたまま止まる角度の手前から減点）。既定は元のまま
# 0042: reset_from_dataset.enabled=False で、人の動きのデータではなく立った姿勢から始める（評価用。傾いた姿勢から始まり倒れる台をなくす）
# 0043: 段 2-4b の学習用の扉 door_en1154_g13_shaped_left_lpos（レバーの位置を広く）・_lever（さらにレバーの形を広く）。実物のレバーを範囲に入れる
# 0044: 段 2-4b-3 の学習用の扉 door_en1154_g13_shaped_left_weight（_lever に扉の重さ 15〜120 kg を足す。既定は 80〜120 kg）
# 0045: 段 2.5 の状態入力の生徒（DAgger・映像なし）。扉の角度は頭の LiDAR、枠の位置は地図と測位、寸法は巻き尺の値を
#       真似て雑音・遅れ・欠けを入れる（door_state_sensor.py）。教師の観測（teacher_obs）にも鏡写しを効かせる
# 0046: 蒸留の評価を教師の評価と同じ始め方に（回の経過時間を乱数で進めない）。元のままだと教師に運転させても 25/64
# 0047: 続きから回すときの学習率を algo.config.resume_learning_rate で替えられるように（元は重みのファイルの値に戻る）
# 0048: 生徒に腕の関節のトルク（実機の tau_est）を見せ、3 割の台は教師に運転させる。評価は enforce_teacher_rollout=False で
# 0049: 生徒を強化学習（PPO）で磨く観測と exp（actor は生徒の値・critic は教師と同じ）。出発の重みは doorman/make_state_ppo_ckpt.py
# 0050: 評価の 1 ステップごとに方策・HOMIE の入出力と生の値を npz に書く（algo.config.eval.golden_trace_path）。real/check_golden.py で実機用のコードと突き合わせる
# 0051: 関節の目標を可動域の内側で切る（env.config.clip_targets_to_limits_margin、既定は切らない）。実機と同じ切り方の成功率を測る用
if [ ! -d VIRAL6 ]; then
  git -C "$OLD/VIRAL" worktree add -q "$BASE/VIRAL6" "$DOORMAN_COMMIT"
  for p in "$OLD/0001-g1-29dof-nofinger.patch" "$OLD/0002-door-nolatch-drop-latch-init.patch" \
           "$OLD/0003-skip-finger-reset-without-fingers.patch" "$OLD/0005-state-recorder.patch" \
           "$BASE/0006-isaaclab3-port.patch" "$BASE/0007-isaacsim601-extension-loading.patch" \
           "$BASE/0008-door-joint-frames-relative-to-env.patch" "$BASE/0009-state-recorder-door-bodies.patch" \
           "$BASE/0010-29dof-vision-student-configs.patch" "$BASE/0011-behavior-scripting-kit110.patch" \
           "$BASE/0012-lever-side-hand-and-pregrasp-diag.patch" "$BASE/0013-eval-sample-actions-and-contact-diag.patch" \
           "$BASE/0014-pregrasp-at-handle-for-handless.patch" "$BASE/0015-render-results-fixed-camera.patch" \
           "$BASE/0016-door-stage22-24-options-isaaclab3.patch" "$BASE/0017-door-hinge-closed-preload.patch" \
           "$BASE/0018-door-press-handle-down-reward.patch" "$BASE/0019-door-grasp-pregrasp-offsets.patch" \
           "$BASE/0020-door-push-unlatched-reward.patch" "$BASE/0021-29dof-press-reward-1.patch" \
           "$BASE/0022-29dof-grasp-target-over-bar.patch" "$BASE/0023-door-push-force-penalize-pull.patch" \
           "$BASE/0024-door-grasp-target-along-lever.patch" "$BASE/0025-door-handle-reward-pos-only.patch" \
           "$BASE/0026-door-hinge-reward-pos-only.patch" "$BASE/0027-29dof-standing-still-penalty-5.patch" \
           "$BASE/0028-29dof-episode-30s.patch" "$BASE/0029-29dof-stage-time-300.patch" \
           "$BASE/0030-29dof-open-stage-time-100.patch" "$BASE/0031-door-push-force-max.patch" \
           "$BASE/0032-door-en1154-upto-grade-mix.patch" "$BASE/0033-door-record-closer-params.patch" \
           "$BASE/0034-recorder-hand-and-panel-force.patch" "$BASE/0035-door-en1154-g4g5.patch" \
           "$BASE/0036-door-en1154-g4g5-82.patch" "$BASE/0037-door-closer-shaped.patch" "$BASE/0038-door-mirror-left-handle.patch" "$BASE/0039-door-en1154-g13-shaped.patch" "$BASE/0040-door-en1154-g13-shaped-left.patch" "$BASE/0041-door-window-reward.patch" "$BASE/0042-door-reset-standing-option.patch" "$BASE/0043-door-en1154-g13-shaped-left-lever.patch" "$BASE/0044-door-en1154-g13-shaped-left-weight.patch" "$BASE/0045-door-state-input-student.patch" "$BASE/0046-door-dagger-eval-like-ppo-eval.patch" "$BASE/0047-resume-learning-rate.patch" "$BASE/0048-door-state-student-arm-torque-teacher-mix.patch" "$BASE/0049-door-state-student-ppo-finetune.patch" "$BASE/0050-door-golden-trace.patch" "$BASE/0051-door-clip-targets-to-limits.patch"; do
    git -C VIRAL6 am -q "$p"
  done
fi
git -C VIRAL6 log --oneline -5
$PIP install $C --extra-index-url "$CU128" -e VIRAL6
# DoorMan の依存の opencv-python（5.x）と Isaac Sim の opencv-python-headless（4.13）は同じ cv2 を入れる。
# 片方を消すと共有の cv2 が壊れるので、opencv-python を消してから headless を入れ直す（10-07 実測）
$PIP uninstall -y -q opencv-python || true
$PIP install -q --force-reinstall --no-deps opencv-python-headless==4.13.0.90
"$ENV_BIN/python" -c "import cv2; print('cv2', cv2.__version__)"
torch_check

step "6. 確認"
$PIP check || true
"$ENV_BIN/python" -c "import isaaclab, isaaclab_physx, numpy, warp; print('isaaclab', isaaclab.__file__); print('numpy', numpy.__version__, 'warp', warp.__version__)"
"$ENV_BIN/python" -c "import gr00t; print('gr00t', gr00t.__file__)"
du -sh "$BASE"/* 2>/dev/null
df -h "$HOME" | tail -1
echo "SETUP6_DONE"
