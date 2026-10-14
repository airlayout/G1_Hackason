# sim2sim — DoorMan の方策を MuJoCo で動かす（段 3）

## HOMIE の脚だけ（2026-10-06）

DoorMan 同梱の HOMIE（`models/model_walk.pt`・`model_stand.pt`、下半身 15 関節の歩行の方策）を、
Unitree 公式の MuJoCo のモデル（`unitreerobotics/unitree_mujoco` の `unitree_robots/g1/scene_29dof.xml`、commit 1eb6642）で動かした。
腕は既定の姿勢（0）に PD で保持。床は平ら。

```bash
# Mac。G1_Hackason/Navigation/.venv（Python 3.10・torch 2.13・mujoco）で動く。動画は Homebrew の ffmpeg
python homie_mujoco.py --mjcf <unitree_mujoco>/unitree_robots/g1/scene_29dof.xml \
    --homie-modules <VIRAL>/gr00t/rl/trl/modules/homie_modules.py \
    --walk <VIRAL>/models/model_walk.pt --stand <VIRAL>/models/model_stand.pt \
    --cmd 0.8,0,0 --seconds 10 [--video out.mp4]
```

**10 秒ずつ、どの指令でも転ばなかった**（高さ 0.73〜0.75 m を保つ）。

| 指令（観測の値 vx, vy, wz） | 動いた量（10 秒） | 実際の速さ | 指令の尺度で直した追従 |
|---|---|---|---|
| 0, 0, 0（立つ方策） | 0.07 m | — | — |
| 0.4, 0, 0 | 0.07 m（立つ方策に切り替わる） | — | — |
| 0.6, 0, 0 | 前 2.42 m | 0.24 m/s | 81% |
| 0.8, 0, 0 | 前 3.25 m | 0.33 m/s | 81% |
| 1.0, 0, 0 | 前 4.14 m | 0.41 m/s | 83% |
| 0, 0.6, 0 | 横 2.04 m | 0.20 m/s | 68% |
| 0, 0, 0.8 | その場で 907° | 1.58 rad/s | 99% |
| 0, 0, −0.6 | その場で −697° | −1.22 rad/s | 101% |

分かったこと:
- **観測のコマンドは実際の速度に尺度が掛かっている。** 前後・横は実際の 2 倍、旋回は実際の 0.5 倍（HIMLoco 系の `lin_vel 2.0`・`ang_vel 0.5`）。
  DoorMan の `get_physical_homie_commands()` が返すのは「観測の尺度の値」で、実際の速度は `_homie_commands`（方策の出力 × 0.25、±0.5 で頭打ち）
- **観測の尺度で |(vx, vy, wz)| が 0.5 以下だと「立つ方策」になり、その場で止まる。** 実際の速さでは前後 0.25 m/s・旋回 1.0 rad/s が境目。
  DoorMan の頭打ち（実際の旋回 ±0.5 rad/s）だと、旋回だけの指令では歩く方策に切り替わらない（Nav2 とつなぐ段 7 で効く）
- 前後・横の追従が 7〜8 割なのは、Isaac Sim と MuJoCo の違い（関節の摩擦・armature）か、HOMIE そのものの癖かは未切り分け（Isaac Sim 側で同じ指令を測ると分かる）

動画（git 管理外）: `docs/動画_写真/ドア/sim2sim_homie_2026-10-06/`（`homie_fwd.mp4`・`homie_turn.mp4`・`homie_side.mp4`、各 6 秒）

再現の要点（DoorMan のコードから読んだ）:
- 1 ステップ 86 = コマンド 7・角速度 3（×0.5）・重力の向き 3・関節角 29（既定の姿勢との差）・関節速度 29（×0.05）・前回の脚の行動 15
- 6 ステップ（古い → 新しい、今のステップが末尾）＝ 516。立つ方策には全ステップのコマンドの速度 3 つを 0 にして渡す
- 目標角 = 行動 × 0.25 + 既定の姿勢。PD は `g1_43dof.yaml` のゲイン、トルクの上限は ×0.95。方策 50 Hz・物理 200 Hz
- 置いた直後から方策を動かす（PD で立たせてから渡すと倒れる。unitree_rl_lab で確かめた落とし穴）
