#!/usr/bin/env bash
# G1 の WBC sim（MuJoCo）で rollout する。方策サーバー（run_server.sh）が先に起動している必要がある。
# 動画は Isaac-GR00T 側が /tmp/sim_eval_videos_<env>/<env>_ac<N>_<uuid>/*.mp4 に書く。
#
# 使い方: N_EP=3 MAX_STEPS=1440 bash run_client.sh
#   N_EP       エピソード数（既定 1）
#   MAX_STEPS  1 エピソードの最大ステップ（既定 1440。20fps で約 72 秒）
set -uo pipefail
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck source=env.sh
source "$HERE/env.sh"
require_groot_dir || exit 1

# ヘッドレス（画面なし）でも描画できるよう EGL を使う。
export MUJOCO_GL=egl PYOPENGL_PLATFORM=egl
cd "$GROOT_DIR" || exit 1
"$SIM_PY" gr00t/eval/rollout_policy.py \
  --policy_client_host "$POLICY_HOST" --policy_client_port "$POLICY_PORT" \
  --n_episodes "${N_EP:-1}" --max_episode_steps="${MAX_STEPS:-1440}" \
  --env_name "$ENV_NAME" \
  --n_action_steps "$N_ACTION_STEPS" --n_envs 1
STATUS=$?
echo "[client] exit=$STATUS"
exit "$STATUS"
