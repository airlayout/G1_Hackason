# Door — G1 でドアを開けて部屋に入り、閉める

UIS Room 4 のドア（内開き・レバー左・蝶番右・ラッチ・クローザー）を、指の動かない純正のエンドハンドの G1（29 関節）に開けさせる。
方式はシミュレータで強化学習 → 実機。土台は NVIDIA の DoorMan（`NVlabs/GR00T-VisualSim2Real` の `doorman` ブランチ、Apache-2.0、Isaac Sim 5.1）。2026-10-07 から Isaac Sim 6.0.1 に移して動かしている（下の「Isaac Sim 6.0.1 で動かす」）。
計画書は `docs/plan/2026-10-06-g1-door-open-enter-close.md`（git 管理外）。

ここには DoorMan 本体を置かない。**commit を固定して取り込み、自分たちの変更はパッチで持つ。**

```text
Door/
├── doorman/
│   ├── setup_doorman.sh      OMEN に環境を作る（conda doorman51・Isaac Sim 5.1・Isaac Lab v2.3.2・DoorMan・LAFAN-G1・VIRAL29）
│   ├── constraints.txt       torch 2.7.0+cu128 などの固定（無いと pip が torch を 2.14 に上げる）
│   ├── gate.sh               GPU を使ってよいかの判定（自分以外の GPU の使用・空きメモリ・画面の前の人の離席）と、SIGINT を戻して起動する仕掛け
│   ├── run_bench.sh          学習を短く回して VRAM・RAM・速さを測る（共用機向けの安全装置つき）
│   ├── run_train_segments.sh 長い学習を区切って回す。人が戻ったら止めて譲り、空いたら last.pt から続きを学習する
│   ├── run_eval_record.sh    チェックポイントを評価しながら機体と扉の状態を npz に記録する（カメラは使わない）
│   ├── night_queue.sh        2026-10-06 夜のキュー（記録の確認 → 段 2-1 の学習を朝 8:30 まで）
│   ├── run_status.sh         見張り用に状態を 1 行ずつ出す（N 反復ごとの節目・エラー・NaN）
│   ├── train_progress.py     train.log から反復ごとの数字を CSV に抜き出す（標準ライブラリだけ）
│   ├── bench_queue.sh        計測を順番に流す
│   ├── eval_from_start.sh    学習を札で止め、最初の区間から 128 回評価して成功率を数える（段 2 の合否。VIDEO=1 で動画）
│   ├── count_success.py      評価の記録から成功率・区間ごとの到達・扉の角度・レバーを下げた回を数える
│   ├── diag_pregrasp.py      区間 1→2 の条件を項目ごとに測る
│   ├── stage_chain.sh        段 2-2・2-3 を順に「学習 → 評価 → 合格なら次」で回す（止め札 STOP_DOOR_CHAIN）
│   ├── after_chain.sh        チェーンの終わりを待って、その最後の ckpt から次のチェーンを起動する（REQUIRE=all_pass で合格のときだけ）
│   ├── relaunch_chain.sh     段 2-2 で不合格のまま終わったチェーンを、最新の last.pt から起動し直す（10-08 の夜間用）
│   ├── sweep_tasks.sh        1 つの ckpt を学習せずに複数の扉で続けて評価する（等級ごとの見通し。チェーンは札で止めたまま）
│   ├── switch_after_pass.sh  走っているチェーンで、ある段が合格したら次の段の前で止め、合格の ckpt から別の段の並びで起動し直す
│   ├── door_force_curve.py   評価の記録から、クローザーの上限ごとの成功率と、成功率 50%・80% になる上限を出す（0033 から記録した static.npz が要る）
│   ├── push_force_stats.py   評価の記録から、手でレバーを押す力・扉の板が受ける力をまとめ、成功と失敗で比べる（0034 から記録）
│   ├── exp_*_1009.sh         2026-10-09 の等級 5 の原因調べの実験 E2〜E5（作業ログ 9.5）と案 2（exp_g5_82_1009.sh、9.6）・案 3（exp_g5_82_fix_1010.sh、9.7）
│   └── patches/              指の無い 29 関節版（段 2-1）。番号順に git am で当てる
│       ├── 0001-g1-29dof-nofinger.patch
│       ├── 0002-door-nolatch-drop-latch-init.patch
│       └── 0003-skip-finger-reset-without-fingers.patch
│       └── untested/         Isaac Sim でまだ動かしていないもの（setup_doorman.sh は当てない）
│           ├── 0004-door-stage22-24-options.patch   段 2-2〜2-4 のドア（EN 1154 の等級・四角いレバー・ラッチの出・開いたまま止まる角度）
│           └── 0005-state-recorder.patch            状態の記録係（0003 の上に単独で当たる。OMEN の VIRAL29rec に当ててある）
├── doorman6/                 Isaac Sim 6.0.1 ＋ Isaac Lab 3.0（release/3.0.0-beta2）版。OMEN のドライバ 595 のままカメラが使える
│   ├── setup_doorman6.sh     OMEN に環境を作る（conda doorman6・Python 3.12・torch 2.11.0+cu128・VIRAL6）
│   ├── constraints_from_teammate.txt  チームメイトの環境の主要な 16 個の版
│   ├── smoke6.sh             短く学習を回して確かめる（STUDENT=1 でカメラ付きの生徒）
│   ├── stackdump_run.py      固まったときに全スレッドの呼び出しの履歴を書く包み（既定では書かない）
│   ├── 0006-isaaclab3-port.patch                 Isaac Lab 3.0 への移植（四元数の境界・ProxyArray・API の改名）
│   ├── 0007-isaacsim601-extension-loading.patch  起動時に omni.kit.material.library を有効にする（固まる対策）
│   ├── 0008-door-joint-frames-relative-to-env.patch  扉の関節の取付位置を env からの相対で求める（直さないと扉に届かない）
│   ├── 0009-state-recorder-door-bodies.patch     記録係に扉の部品の位置を足す（5.1 と比べる用）
│   ├── 0010-29dof-vision-student-configs.patch   29 関節のカメラ付き生徒（DAgger）の設定・記録係が RGB を PNG で書く
│   ├── 0011〜0015            Kit 110 の BehaviorScript・把手と同じ側の手・評価で行動を引く・把手そのものに構える・評価の動画のカメラ
│   ├── 0016-door-stage22-24-options-isaaclab3.patch  段 2-2〜2-4 のドア（5.1 版 untested/0004 の移植）
│   ├── 0017-door-hinge-closed-preload.patch          閉じた位置の押し付けを既定のままに（強いクローザーでレバーが勝手に回るのを止める）
│   ├── 0018-door-press-handle-down-reward.patch      手でレバーを下へ押す力の報酬（段 2-2）
│   ├── 0019-door-grasp-pregrasp-offsets.patch        手のひらを当てる点・構える点のずらし（棒の上から押す。段 2-2）
│   ├── 0020-door-push-unlatched-reward.patch         レバーを 30° 以上下げている間に前へ押す力の報酬（段 2-2）
│   ├── 0021〜0030            押す力の報酬 4→1・当てる点を棒の中心の上へ・手前へ引く力を負に・押す点をレバーの先へ・レバーと扉の報酬は角度だけ・区間 4 で止まる罰 -5・1 回 30 秒・区間 4 の持ち時間 300（段 2-2）
│   ├── 0031-door-push-force-max.patch  前へ押す力の報酬の頭打ちを設定に（push_door_force_max、既定 20 N。段 2-3 の等級 3 以上で上げる）
│   ├── 0032-door-en1154-upto-grade-mix.patch  段 2-3 の学習用の扉 door_en1154_upto_g1〜g5（既定〜等級 k の強さを混ぜる）
│   ├── 0033-door-record-closer-params.patch   台ごとのクローザーの上限・ばね・目標角を static.npz に記録する
│   ├── 0034-recorder-hand-and-panel-force.patch  毎ステップ、手がレバーから受ける力と扉の板が受ける力を記録する
│   ├── 0035-door-en1154-g4g5.patch            等級 4＋5 を混ぜた扉 door_en1154_g4g5（26〜54 N·m）
│   ├── 0036-door-en1154-g4g5-82.patch         台の 8 割が等級 4・2 割が等級 5 の扉 door_en1154_g4g5_82（台 i % 5 == 4 が等級 5）
│   ├── 0037-door-closer-shaped.patch          段 2-4 の実物の形のクローザー（door_closer.py・closer_shaped）。評価用 door_en1154_g1〜g4_shaped・学習用 door_room4_g13
│   ├── 0038-door-mirror-left-handle.patch     把手が左の扉で観測と行動を鏡写し（door_mirror.py・mirror_left_doors）。評価用 door_en1154_g1〜g4_left
│   ├── 0039-door-en1154-g13-shaped.patch      段 2-4a の学習用の扉（等級 1〜3・実物の形）
│   ├── 0040-door-en1154-g13-shaped-left.patch その把手が左（実物の向き）の扉
│   ├── 0041-door-window-reward.patch          窓の報酬（push_door_hinge_max_deg・door_overopen）
│   └── 0042-door-reset-standing-option.patch  評価で立った姿勢から始める（reset_from_dataset.enabled=False）
├── sim2sim/
│   ├── homie_mujoco.py       HOMIE の脚だけを Unitree 公式の MuJoCo のモデルで歩かせる（段 3。Mac で動く）
│   ├── replay_states.py      記録した状態を MuJoCo で再生して mp4 にする（DoorMan と同じ寸法・開き方の扉つき）
│   └── README.md             結果（10 秒ずつ転ばない・速度の尺度・立つ方策への切り替え）
└── tools/
    ├── make_g1_29dof_yaml.py   g1_43dof.yaml から g1_29dof.yaml を作る（パッチに入っている yaml の生成元）
    └── make_door_scenarios.py  段 2-3・2-4 のドアの場面ファイルを作る（パッチ 0004・6.0.1 では 0016〜0017・0032・0035〜0037 の生成元）
```

段の切り替えはコマンドの引数で行う（6.0.1 ではパッチ 0016〜0017 を当てたあと。2-2・2-3 は 2026-10-07 に動かした。2-4 は未検証）:

| 段 | 足す引数 |
|---|---|
| 2-1 | なし（`door_open_homie_29dof_lstm` の既定は `door_nolatch`） |
| 2-2 | `++env.config.task.name=door`（ラッチあり・クローザーは DoorMan の既定） |
| 2-3 | `++env.config.task.name=door_en1154_g1`（g1〜g5）。閉じた位置の押し付けは既定と同じ 2.5〜4.5 N·m で、開けるほど強くなり等級の上限で頭打ち（0017）。易しい扉を混ぜた `door_en1154_upto_g1`（〜g5。上限 2.5 N·m〜等級の上限、0032）もあるが、混ぜて学習すると等級 1 が落ちた（10-08） |
| 2-4 | `++env.config.task.name=door_room4_g13 ++env.config.door_hold_open_angle_deg=90 ++env.config.door_through_angle_deg=70`（等級 1〜3・実物の形のクローザー、0037。旧版の `door_room4_wide` は PD のクローザーで等級 1〜4） |
| 2-4a（10-14 合格） | `++env.config.task.name=door_en1154_g13_shaped_left ++env.config.mirror_left_doors=True ++env.config.door_through_angle_deg=70 ++env.config.door_hold_open_angle_deg=90 ++env.config.push_door_hinge_max_deg=80 ++env.config.door_overopen_start_deg=85 ++rewards.reward_scales.door_overopen=-3.0`（＋押す力の頭打ち 50 N）。評価は `++env.config.reset_from_dataset.enabled=False ++env.config.max_stage_time=[500,200,200,200,600,400] ++env.config.max_episode_length_s=60`（`exp_relaxtime_1014.sh`）。合格の重みは OMEN `ckpt/s24a_pass_it40600/` |
| 2-4 の評価 | `door_en1154_g1_shaped`〜`g4_shaped`（`door_en1154_g<k>` とクローザーの形だけが違う。学習の前に今の方策で崖が動くかを見る） |

⚠️ **等級の扉で、閉じた位置まで等級の力で押し付けると、ラッチの斜面が押されてレバーが勝手に回る**（2026-10-07、0017 の前の等級 1 で 16 台中 12 台が 85〜99° まで回った。
蝶番は閉位置を 0.003〜0.046° 越えていた）。0017 で閉じた位置の押し付けを DoorMan の既定と同じにし、等級 5 でもレバーは 0° のまま動かないことを確かめた。
実物のクローザーは閉じる直前がいちばん強いので、閉じる直前の強さはシミュレータの方が弱い。

**PD のクローザーは、扉が動いている間はどの角度でも力の上限（等級の値）で一定**（減衰 50 N·m·s/deg が大きく上限に張り付く。「開けるほど強くなる」のは止めて押しているときだけ）。0037 の `closer_shaped=True` は蝶番のドライブを切り、env が毎物理ステップ `door_closer.py` の力を掛ける: 閉鎖モーメントは 0° で押し付け（0017 と同じ）→ 2° で等級の値 → 90° で 1/3〜1/2、開ける向きは 閉鎖 ÷ 効率（EN 1154 の最小〜+0.15）、70〜80° から先はバックチェック、閉じる向きは 90° から 5〜8 秒で閉まる減衰。15°/s で押すと、ラッチを外した直後は PD の 1.4〜1.7 倍重く、60〜70° では 0.7〜1.2 倍＝ほぼ同じか軽い（Mac の 1 自由度の計算、2026-10-13）。単体テストは Isaac Sim なしで `python -m pytest tests/test_door_closer.py`（torch だけ）。

段 2-4 の最初の評価（0037 を当てたあと・学習なし。2026-10-13 に用意し、OMEN が offline でまだ回していない）:

```bash
# Mac の G1_Hackason/Door から送る
scp doorman6/0037-door-closer-shaped.patch ubuntu@100.99.102.70:~/work_inoue/doorman6/
scp doorman/check_closer_in_sim.py doorman/compare_closer_shape.py ubuntu@100.99.102.70:~/work_inoue/doorman/
# OMEN で当てる（git は conda の中。commit になるので単独の ssh で）
cd ~/work_inoue/doorman6/VIRAL6 && PATH=~/miniforge3/envs/doorman6/bin:$PATH git am ../0037-door-closer-shaped.patch
~/miniforge3/envs/doorman6/bin/python -m pytest -q tests/test_door_closer.py
# 1. 計器の確かめ: 1/3 の台は扉が開いて始まる。力が Isaac Sim の中で効いているか（合格が出るまで 2 へ進まない）
cd ~/work_inoue/doorman
EXTRA_ARGS="++env.config.task.name=door_en1154_g3_shaped ++env.config.randomize_door_init_state=True" \
  ./eval_from_start.sh ckpt/g4_pass_it34950/last.pt 32
~/miniforge3/envs/doorman6/bin/python check_closer_in_sim.py records/start_<日時>
# 2. 34,950 を PD と実物の形の等級 1〜4 で（変えるのはクローザーの形だけ）
HOLD_FILE=$HOME/work_inoue/PAUSE_DOOR_STAGE2B COUNT_ARGS="--criterion door30" ./sweep_tasks.sh ckpt/g4_pass_it34950/last.pt \
  door_en1154_g1 door_en1154_g1_shaped door_en1154_g2 door_en1154_g2_shaped door_en1154_g3 door_en1154_g3_shaped door_en1154_g4 door_en1154_g4_shaped
~/miniforge3/envs/doorman6/bin/python compare_closer_shape.py sweep/<日時>/results.tsv
```

長く走るもの（2 の sweep）は ssh の中で `setsid nohup ... > log 2>&1 < /dev/null &` を単独の行にして起動する。

**⚠️ 段 2-3 までの方策は把手が右の扉（`door_open_lr=-1`）しか学んでいない。実物の UIS Room 4 は把手が左（+1）**（2026-10-13 に +1 の扉は 0/70）。0038 の `++env.config.mirror_left_doors=True` で、+1 の台だけ方策に渡す観測と方策の行動を左右の鏡写しにする（関節は left_↔right_、roll・yaw は符号反転、横・旋回の指令も反転。物理・報酬・区間の判定は本当の向きのまま）。反復 34,950 で、把手が左の等級 1・3・4 が 93/126・98/128・89/128（右は 111・114・104）。4 本の評価は `doorman/exp_mirror_1013.sh`、記録は `docs/動画_写真/ドア/isaac_eval_2026-10-13_shaped/`（`exp_mirror_1013.log`）。

段 2-2・2-3 は `doorman/stage_chain.sh` で続けて回す（段ごとに「学習 60 分 → 最初の区間からの 128 回の評価」を繰り返し、80% 以上で次の段へ。合格した ckpt を `ckpt/<段>_pass_it<反復>/` に残して 8 台の動画を撮る）:

```bash
cd ~/work_inoue/doorman
MEM_MAX=9G NEED_MARGIN_MB=1024 ./stage_chain.sh ckpt/start_1007_1704/last.pt \
  s22:door:6:60 g1:door_en1154_g1:3:60 g2:door_en1154_g2:3:60 g3:door_en1154_g3:3:60 g4:door_en1154_g4:3:60 g5:door_en1154_g5:3:60
# 結果は chain/<日時>/results.tsv と chain.log。札は PAUSE_FILE（既定 ~/work_inoue/PAUSE_DOOR_STAGE2）
```

- 段の 5 つ目の欄は成功の定義（`count_success.py --criterion`。`through`＝通り抜け〔既定〕・`door30`＝レバーを 40° 以上下げて扉を 30° 以上）
- 6 つ目の欄は学習だけに足す hydra の引数（`,` 区切り）。段 2-3 の等級 3〜5 は、前へ押す力の報酬の頭打ち（既定 20 N）を等級で要る力の 1.5 倍に上げ、重みを下げて 1 ステップの上限を保つ（`++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12` など。パッチ 0031）
- 走っている stage_chain.sh は書き換えても読み直さない（差し替えは mv で。上書きすると走っている bash が壊れる）。後ろの段の設定を変えるときは `switch_after_pass.sh`
- 7 つ目の欄は合否を見る扉（省くと学習と同じ扉）。例えば、易しい扉を混ぜた `door_en1154_upto_g<k>`（クローザーの上限 2.5 N·m〜等級 k の上限）で学習し、純粋な `door_en1154_g<k>` で合否を見る。⚠️ 2026-10-08 に試すと等級 1 が 50 → 5/128 に落ちた（易しい扉に合わせて押しを弱める）。段 2-3 は純粋な等級の扉で学習している（`relaunch_pure_1008.sh`）:

```bash
EVAL_FIRST=1 MEM_MAX=9G NEED_MARGIN_MB=1024 ./stage_chain.sh ckpt/start_1008_1221/last.pt \
  m1:door_en1154_upto_g1:3:60:door30::door_en1154_g1 m2:door_en1154_upto_g2:3:60:door30::door_en1154_g2 \
  m3:door_en1154_upto_g3:3:60:door30:++env.config.push_door_force_max=50,++env.config.push_unlatched_force_max=50,++rewards.reward_scales.push_door_force=0.12:door_en1154_g3
```
- `EVAL_FIRST=1` で、段ごとに学習の前にまず評価する（results.tsv の回 0）。合格ならその段は学習しない（2026-10-08。等級の扉で、前の等級の方策がそのまま通るときの 60 分を省く）
- 学習せずに等級ごとの成功率だけを見るときは `sweep_tasks.sh`（チェーンの札 `HOLD_FILE` を置いたまま通しで評価し、終わったら外す）:

```bash
HOLD_FILE=$HOME/work_inoue/PAUSE_DOOR_STAGE2B COUNT_ARGS="--criterion door30" \
  ./sweep_tasks.sh ckpt/start_1008_1118/last.pt door_en1154_g1 door_en1154_g2   # 1 つ 3〜4 分。結果は sweep/<日時>/results.tsv
```

## OMEN での置き場所

| 場所 | 中身 |
|---|---|
| `~/work_inoue/doorman/VIRAL` | DoorMan `016c70c`（元のまま。43 関節・Dex3 の指つき） |
| `~/work_inoue/doorman/VIRAL29` | 同じ commit の worktree にパッチ 1〜3 を当てたもの（指の無い 29 関節版。夜通しの学習はここ） |
| `~/work_inoue/doorman/VIRAL29rec` | パッチ 1〜3 ＋ 記録係（0005）。評価して記録するときに使う |
| `~/work_inoue/doorman/records/` | 記録した状態（`static.npz`・`states_*.npz`）。Mac へ持ってきて `replay_states.py` で再生 |
| `~/work_inoue/doorman/LAFAN-G1` | リセット用の動作データ（29 関節・30 剛体。29 関節版にもそのまま使える） |
| `~/miniforge3/envs/doorman51` | conda 環境。git-lfs もこの中 |
| `~/work_inoue/doorman/bench/` | 計測の記録（`status`・`train.log`・`mem.csv`・`gpu.csv`） |
| `~/work_inoue/doorman6/VIRAL6` | 6.0.1 版。同じ commit の worktree にパッチ 0001〜0003・0005〜0010 |
| `~/work_inoue/doorman6/IsaacLab` | Isaac Lab release/3.0.0-beta2（`99f1423e`。チームメイトと同じ commit） |
| `~/miniforge3/envs/doorman6` | 6.0.1 版の conda 環境（Python 3.12）。チームメイトの `~/NVIDIA/env_isaaclab` とは別 |
| `~/work_inoue/doorman6/tests/` | `smoke6.sh` の記録 |

`gr00t` は `VIRAL` に editable で入っている。`VIRAL29` で動かすときは `PYTHONPATH=~/work_inoue/doorman/VIRAL29` を付ける（`run_bench.sh` は `REPO=VIRAL29` で付ける）。

## 動かし方

```bash
# OMEN で。元の DoorMan の教師（43 関節）
cd ~/work_inoue/doorman
MEM_MAX=8G KIND=teacher ./run_bench.sh 256 15

# 指の無い 29 関節版（段 2-1: ラッチ無しのドア）
REPO=VIRAL29 EXP=wbmanip/door_open_homie_29dof_lstm KIND=t29 MEM_MAX=8G ./run_bench.sh 256 12
```

中で呼んでいるのは DoorMan の README の教師の学習コマンド
（`python gr00t/rl/train_agent_trl.py +exp=... ++num_envs=... ++algo.config.entropy_coef=0.001 ++algo.config.num_steps_per_env=32 ++env.config.delta_action_scale=0.3`）に、
`++use_wandb=False ++headless=True ++env.config.reset_from_dataset.motion_file_dir=<LAFAN-G1>` を足したもの。
初回の Isaac Sim の起動では `OMNI_KIT_ACCEPT_EULA=YES`（NVIDIA の EULA に同意）を付けている。

## Isaac Sim 6.0.1 で動かす（doorman6）

2026-10-07 から、DoorMan は Isaac Sim 6.0.1 ＋ Isaac Lab 3.0 で動かす（5.1 は OMEN のドライバ 595.91 でカメラの描画が落ちるため）。
パッチは 5.1 と共通の 0001〜0003・0005 の上に 0006〜0010 を積む。チェックポイントは 5.1 と 6.0.1 で共通。

```bash
# OMEN で。環境を作る（doorman6/ の中身を ~/work_inoue/doorman6/ に置いてから）
cd ~/work_inoue/doorman6 && ./setup_doorman6.sh > setup6.log 2>&1   # 完了の目印は SETUP6_DONE

# 教師を短く回す
./smoke6.sh t29 10 12G +exp=wbmanip/door_open_homie_29dof_lstm ++num_envs=512

# カメラ付きの生徒を短く回す（教師の last.pt を渡す。RGB を 25 ステップごとに PNG で書く）
STUDENT=1 ./smoke6.sh s29cam 10 12G +exp=wbmanip/door_open_homie_29dof_dagger-lstm ++num_envs=16 \
  ++teacher_actor_path=<教師の last.pt の絶対パス> \
  ++env.config.record_states.dir=$HOME/work_inoue/doorman/records/s29cam ++env.config.record_states.rgb_every=25

# 区切って長く学習する（5.1 の last.pt から続ける例）
cd ~/work_inoue/doorman
WORK=$HOME/work_inoue/doorman6 REPO=VIRAL6 ENV_NAME=doorman6 IDLE_GATE=1 PROJECT=door29 KIND=door29_601 MEM_MAX=9G \
  RESUME_CKPT=<last.pt> ./run_train_segments.sh 512 20:00

# 評価しながら状態を記録する
WORK=$HOME/work_inoue/doorman6 REPO=VIRAL6 ENV_NAME=doorman6 MEM_MAX=10G ./run_eval_record.sh <last.pt> <出力先> 16 8 15
```

5.1 と比べた結果（2026-10-07、OMEN）:

| 確かめたこと | 5.1 | 6.0.1 |
|---|---|---|
| 同じチェックポイント（約 990 反復）で区間 1 に届いた env（16 台・8 エピソード） | 9/16 | 9/16（同じ env）。0008 を当てる前は 0/16 |
| 教師を一から学習: 報酬（反復 50／100／150） | −55／−46／−31 | −57／−40／−32 |
| 教師の速さ（512 台） | 約 5,300 steps/s | 約 5,700 steps/s。RAM の山 6.8 GB・VRAM 4.6 GB |
| カメラ付きの生徒（16 台・216×384 RGB） | 描画が落ちる | 動く。集めるのに 0.89 s・学習 0.44 s（8 ステップ×16 台ごと）。RAM は初回 11 GB 前後 |

残っていること:

- 生徒のドームライトの乱択（`omni.kit.scripting` が Kit 110 で `omni.behavior.scripting.core` に改名された）。今の 29 関節の生徒の設定では切ってある
- 生徒を何台まで並べられるか（16 台で VRAM 8 GB）

## 共用機での注意（OMEN）

- **他人の処理が動いているときは始めない。** `run_bench.sh` は「GPU に計算プロセスが無い」かつ「空きメモリ > MEM_MAX + 1.5 GB」が 2 分続くまで待つ。
  `IDLE_GATE=1` を付けると「画面（seat0）の前の人が離席している（GNOME の IdleHint か画面ロック）」も待つ（10/6 夜に人が Gradle のビルドなどをしていた）
- **学習は cgroup の中で動かす**（`systemd-run --user --scope -p MemoryMax=… -p MemorySwapMax=0`）。溢れたら自分だけが止まる。
  2026-10-06 に、512 台の起動とチームメイトの 8 並列の評価が重なって空きが 5 秒で 5.9→0.8 GB に落ち、デスクトップの ChatGPT アプリが落ちた
- 標準入力は `/dev/null` にする。DoorMan は報酬関数の例外で `ipdb` に入る作りで、端末につながっていると固まる
- **背景で起動したプロセスは SIGINT を無視したまま始まる**（非対話の bash の仕様。10/6 に OMEN で確かめた）。そのままだと止めるたびに KILL になり、
  最後の保存や記録の書き出しが失われる。`gate.sh` の `SIGINT_DEFAULT_EXEC` で既定に戻してから起動する
- **始めるときだけでなく、走っている間も譲る**（10/6 のレビューの指摘）。`run_train_segments.sh` は 30 秒おきに、画面の前の人が戻ったか・
  自分以外が GPU を 2 GiB より多く使い始めたかを見て、そうなら止める。`nvidia-smi --query-compute-apps` は描画のプロセスを数えないので、`nvidia-smi` の表から足し合わせる
- cgroup には `CPUWeight=20 IOWeight=20` も付けて、CPU とディスクで他人を優先する
- `pkill -f <名前>` は ssh で送ったコマンド自身の文字列にも一致して自分を止める。`[b]ench` の形でも、同じ ssh の中で `bench_queue.sh` という文字列を使うと一致する

## 計測（段 1）

| 構成 | 台数 | 歩/秒 | VRAM | RSS（定常／起動時の山） | 記録 |
|---|---|---|---|---|---|
| 元の教師（43 関節） | 256 | 2,689 | 4.5 GB | 5.4／6.5 GB | `bench/teacher_n256_1006_1836` |
| 元の教師（43 関節） | 512 | 約 4,640 | 5.5 GB | 6.4 GB | `bench/teacher_n512_1006_1900` |
| 29 関節版（ラッチ無し） | 256 | 約 3,100 | 4.8 GB | 5.9 GB | `bench/t29_n256_1006_1953`（245 反復で報酬 −61 → −32、NaN なし） |
| 映像の生徒（43 関節・カメラ） | 16 | — | — | — | `bench/student_n16_1006_1914`。**RTX 描画の初期化で Segfault**（下） |

**カメラ（RTX 描画）は OMEN では動かない。** ドライバ 595.91 は新機能ブランチで、Isaac Sim 5.1 の検証済みは 580.65.06。
`librtx.scenedb.plugin.so` の `createHydraEngine` で落ちる（NVIDIA のフォーラムに同じ報告が多数）。映像の生徒・評価の動画（`render_results`）・GR00T 用の IR のデモが作れない。
ドライバの入れ替えは共用機の全員に効くので未実施（判断待ち）。

## 29 関節版で直したこと（パッチの中身）

元のコードは「剛体名に `left_hand` を含むものが手」「指の関節 14 本が後ろにある」「ドアの関節は 3 本」を決め打ちしていた。
29 関節の USD では手（rubber hand）は `wrist_yaw_link` に統合されていて剛体ではない。

| 場所 | 直したこと |
|---|---|
| `config/robot/g1/g1_29dof.yaml`（生成） | 関節 29・剛体 30・手は `wrist_yaw_link`。palm は Dex3 と同じ座標になるオフセット (0.0415, ±0.003, 0)。頭は `torso_link` から (0.0039635, 0, −0.044) |
| `config/obs/wbmanip/door_open_homie_29dof.yaml` | 手の力 48 → 6 次元。HOMIE へのコマンドから指 2 つを外した |
| `config/exp/wbmanip/door_open_homie_29dof_lstm.yaml` | 方策の出力 19 → 17、delta の番号 0..16、指の報酬 3 項目を 0、区間 2 の完了は手の剛体 1 個の接触、ドアはラッチ無し |
| `data/tasks/door_nolatch/` | ラッチだけ外したドア（段 2-1） |
| `utils/hand_links.py`（新規） | 手の剛体名を設定から引く。設定に無ければ元の部分一致（43 関節版は元のまま動く） |
| `envs/door/door_open_homie.py` | 指が無いときは指の条件を外す（元は空の平均で NaN になり黙って止まる）、palm の位置にオフセット、ドアの関節数をドアから読む、`[:, :-14]` をやめる |
| `simulator/isaacsim/isaacsim.py` | 接触センサの filter・手の FrameTransformer・右 palm の番号を設定から |

洗い出しの全リスト（落ちる 27・黙って壊れる 10・設定だけ）は `docs/plan/2026-10-06-doorman-29dof-hardcodes.md`（git 管理外）。番号（A-1 など）はそこの番号。

## 描画の代わりに MuJoCo で見る

OMEN では Isaac Sim のカメラの描画が落ちるので、評価中の状態（胴の位置と向き・29 関節・扉の 2 関節・区間）だけを記録し、
Mac の MuJoCo で Unitree 公式の G1 と同じ寸法の扉を置いて再生する。物理は回さず姿勢を置くだけなので、記録どおりの動きになる。

```bash
# OMEN（記録）
REPO=VIRAL29rec IDLE_GATE=1 MEM_MAX=8G ./run_eval_record.sh <checkpoint.pt> ~/work_inoue/doorman/records/<名前> 4 8 15
# Mac（再生。区間 3 以上まで進んだ回だけ、など）
scp -r ubuntu@100.99.102.70:work_inoue/doorman/records/<名前> .
G1_Hackason/Navigation/.venv/bin/python sim2sim/replay_states.py --record <名前> \
    --mjcf <unitree_mujoco>/unitree_robots/g1/scene_29dof.xml --env 0 --out-dir <出力先> --min-stage 3
```

扉の向きは作り物の記録で確かめた（`door_open_lr` の ±1 で蝶番とレバーの左右が入れ替わり、どちらも機体から見て奥へ開く）。
⚠️ `door_open_lr < 0` のとき DoorMan は **左手**を使う報酬になっている。機体は +x を向いて進むので、レバーは機体の右にある（逆の手に見える）。
描画が無いので確かめていない。記録を再生して確かめる（この扉はレバー左なので、合わせるときに効く）。

