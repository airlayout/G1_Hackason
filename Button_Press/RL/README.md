# Button_Press / RL（エレベーターのボタン押し：Isaac Lab の強化学習）

G1 の右腕で、エレベーター乗り場の呼びボタン（一般用の ▼）を押す方策を、Isaac Lab の強化学習で作る。
作った方策は、評価環境（[../Yada/](../Yada/README.md)）の `Agent` として、MuJoCo・Isaac Sim・模擬 G1 で採点する。

- 環境の作り方（新しいマシン）: [SETUP.md](SETUP.md)
- 失敗と反省: [FAILURES.md](FAILURES.md)

## 方針（2026-10-19 決定）

| 項目 | 決めたこと |
|---|---|
| 動かす範囲 | 右腕 7 関節。腰（pelvis）と腰の関節は固定（J1-gen・Yada と同じ。実機は arm_sdk で腕だけ動かす） |
| 押すボタン | 一般用の ▼ だけ。評価は `evaluate.py --target down` |
| 学び方 | 教師・生徒の 2 段階。教師はボタンの位置（正解）を見て PPO で学ぶ。生徒は頭カメラの深度とカラーを見て、教師の行動をまねる（蒸留） |
| 行動 | 右腕の目標の角度の差分（1 周期 0.02 rad まで = 評価のランナーの速度の上限と同じ） |
| 押す力 | 適正な範囲（3〜8 N）で押し、点灯したら指を離す。報酬と評価の両方で見る |
| 版 | Isaac Sim 6.0.1.0（pip 版）、Isaac Lab `release/3.0.0-beta2`、rsl_rl 5.0.1（[configs/versions.yaml](configs/versions.yaml)） |
| 完成の条件 | 10 月末に、評価環境の `Agent` として採点できること |

## 状態

| # | 段階 | 状態 |
|---|---|---|
| 0 | 環境の土台（版の固定、setup.sh / check_setup.sh、マシンごとの並列数） | 済み（このマシンで確認。新しいマシンでの setup.sh は未確認） |
| 1 | 評価環境に「押す力」と `--target` を足す（Yada） | 済み（MuJoCo・Isaac Sim・模擬 G1 で確認） |
| 2 | 教師の学習環境 `G1-ButtonPress-Teacher-v0` | 2500 回の学習で、学習環境 2000 試行: 点灯・中指・指を離す 100%、適正な力 85%（2026-10-20） |
| 3 | 生徒（深度 + カラー、蒸留） | まだ |
| 4 | 評価環境の `Agent` にする | `agent/agent.py`（教師 ＋ 画像の検出）。MuJoCo basic 100%（40 試行）、realistic 98%、Isaac Sim 100%（10 試行）。押す力は MuJoCo で強すぎる（basic 平均 14 N、realistic 31 N。調整中） |

## 使い方

```bash
bash Button_Press/RL/scripts/check_setup.sh                    # 版がそろっているか（数秒）
bash Button_Press/RL/scripts/train.sh --num_envs 16 --max_iterations 3   # 動作の確認（起動に 2〜5 分）
bash Button_Press/RL/scripts/train.sh                          # 教師を学習する（並列数は GPU から決める）
bash Button_Press/RL/scripts/play.sh --num_envs 16             # 学習した方策を見る（最新の checkpoint）
bash Button_Press/RL/scripts/eval_teacher.sh --episodes 2000    # 成功率と押す力の分布を測る（結果は JSON にも）
```

- ログと checkpoint: `_local/button_press_rl/logs/rsl_rl/button_press_teacher/<日時>/`（git の対象外）
- 学習の経過は TensorBoard で見られる:
  `$ISAAC_SIM/python.sh -m tensorboard.main --logdir _local/button_press_rl/logs`
- 並列の環境の数は [configs/machines.yaml](configs/machines.yaml)（L40S 2048、RTX 5060 Ti 1024）。`--num_envs` で変えられる
- ⚠️ `train.sh` は、Python が例外で止まっても終了コード 0 を返すことがある（Isaac Lab の性質）。ログの `Traceback` を見る

### 学習のログの見方（Metrics/）

| 項目 | 意味 |
|---|---|
| `success_rate` | ▼ が点灯した試行の割合 |
| `min_dist_m` | 試行の中での、指先の球とボタンの面の中心の最も近い距離（学習が進むと小さくなる） |
| `in_band_rate` | 成功した試行のうち、試行全体の押す力の最大が適正な範囲（3〜8 N）だった割合 |
| `tip_press_rate` | 成功した試行のうち、中指の先で押した割合 |
| `released_rate` | 点灯のあと 0.5 秒で指を離していた割合 |
| `max_press_force_n` | 成功した試行の、押す力の最大の平均 |
| `wrong_rate`、`force_abort_rate` | 違うボタンを点灯させた割合、押す力が 40 N を超えた割合 |

`Episode_Reward/<項目>` は報酬の項ごとの値（1 秒あたり）。

## 構成

- `button_press_rl/` — Python のパッケージ（`import button_press_rl.tasks` で学習環境を登録する）
  - `spec.py` — 学習の条件を評価環境の設定（Yada の `configs/contest.yaml`・`elevator_prod.yaml`、J1-gen の `robot.yaml`）から決める
  - `proprio.py` — 行動と体の感覚の観測の計算（学習環境と評価用の `Agent` で同じものを使う）
  - `rewards.py` — 報酬（シミュレーターに依存しない。テストできる）
  - `hall_usd.py` — 乗り場を USD に書き出す（Yada の `isaac_hall.build_hall` をそのまま使う）
  - `tasks/press_env.py`、`tasks/press_env_cfg.py` — 学習環境（Isaac Lab の DirectRLEnv）
  - `tasks/agents/rsl_rl_ppo_cfg.py` — PPO の設定
- `agent/agent.py` — 評価環境の `Agent`（画像の検出 ＋ 教師の方策）。`evaluate.py --agent Button_Press/RL/agent --target down`
- `scripts/` — `setup.sh`（新しいマシン）、`check_setup.sh`、`env.sh`、`train.sh`、`play.sh`、`eval_teacher.sh`（評価・方策の書き出し）、`run_isaaclab.py`
- `configs/` — `versions.yaml`（版）、`machines.yaml`（並列数）
- `tests/` — シミュレーターに依存しない部分のテスト（`bash Button_Press/RL/tests/run_tests.sh`）

評価環境と同じにするため、乗り場・ボタン・G1 のモデル・PD の強さ・初期の姿勢・物理の刻み（0.002 s）・制御の周期（50 Hz）は
複製せずに Yada / J1-gen から読み込む。

## 教師の学習環境（G1-ButtonPress-Teacher-v0）

### 乗り場の違いの作り方

評価では試行ごとに乗り場の寸法が変わる（`configs/contest.yaml` の `randomize`）。学習では乗り場を 1 通り（`elevator_prod.yaml`
のまま）だけ作り、**ロボットの立つ位置を毎回ずらして**、ボタンの位置の違いを作る（前後 −5〜+6 cm、左右 ±4.5 cm、
高さ −4〜+5 cm、向き ±3°。評価の範囲を 20% 広げた値。`spec.offset_ranges`。評価の範囲を覆うことは `tests/test_spec.py` で確かめる）。
ボタンの間隔・直径・出っ張りの違いは、まだ入れていない（生徒の段階で、画像の見え方のために足す）。

### 観測

| 方策（教師）44 個 | critic（方策には見せない値も見る） |
|---|---|
| 体の感覚 35（右腕の角度・速度、目標との差、目標、前の行動。`proprio.py`） | 方策と同じもの |
| ▼ の面の位置、指先の位置、その差 × 10（腰の座標） | ▼ の沈み、点灯したか、点灯のあとの時間 |
| | 押す力、指先以外の接触力、ほかのボタンの点灯 |

方策には**実機でも用意できるものだけ**を渡す。ボタンの沈み・点灯は実機では直接は取れないので critic だけに渡す
（押した手応えは「目標との差」から分かる。PD で追っているので、押すと差が大きくなる。実機の lowstate からも取れる）。
こうしておくと、教師を「画像でボタンを見つける（Yada の見本と同じ方法）＋ 方策」の形でそのまま評価環境の
`Agent` にできる（`agent/agent.py`）。生徒は、面の位置を画像から推定する部分だけを置き換えればよい。

### 報酬（`rewards.py`）

毎周期の項はすべて減点（0 以下）、＋は出来事のときだけ。毎周期 ＋ があると、押して終わるよりボタンの前に居続けるほうが得になるため。

| 項目 | 内容 |
|---|---|
| 近づく | 指先とボタンの手前 3 cm の点の距離（遠くと近くの 2 つの尺度） |
| 横をそろえる → 面に近づける → 沈める | 段階ごとの減点。段階に入る前は最大の減点。沈めるのは行き止まり（3 mm）まで（点灯の 2 mm で止めると力が 3 N を割る） |
| 押しすぎ | 押す力が 8 N を超えた分の 2 乗 |
| 衝撃 | 面の 2 cm 手前から触れるまでの、押す向きの速さが 0.1 m/s を超えた分の 2 乗 |
| 点灯のあとも押している | 点灯のあと 0.5 秒の間、力が 0.3 N 以上 |
| 指先以外の接触 | 右腕のリンクと手のひらが、柱・壁・体に当たる力 |
| 滑らかさ | 行動の変化、関節の速さ、目標が可動範囲の端に張りついた関節の数 |
| 点灯（＋） | 30。中指の先で押したときだけ |
| 押す力の良し悪し（±） | 点灯のあとの時間の終わりに、試行全体の押す力の最大が 3〜8 N なら ＋20、外なら −20（評価環境の `press_force_in_range` と同じ量） |
| 指を離した（＋） | 点灯のあと 0.5 秒で離していれば 10 |
| 違うボタン・押す力 40 N 超 | 大きく減点する。違うボタンが点灯している間は毎周期も減点。**打ち切らない**（打ち切ると、わざと違うボタンを押して早く終えることを覚えた。[FAILURES.md](FAILURES.md)） |

### 乱し（`press_env_cfg.DomainRandCfg`。`contest.yaml` の `realism` を目安にした値）

ロボットの立つ位置、右腕の PD の強さ（±10%）、回転子の見かけの慣性、指令の 1 周期の遅れ（半分の環境）、関節のセンサのノイズ、
試行の始めの右腕の角度のずれ（±0.03 rad）。

## まだできていないこと

- 生徒（深度 + カラーの画像、蒸留）と、評価環境の `Agent`
- 乗り場の形の違い（ボタンの間隔・直径・出っ張り、柱の幅）。体の揺れ（`realism.sway`）
- 新しいマシンでの `setup.sh` の通しの確認（このマシンでは、すでに入っているものを飛ばすことだけ確かめた）
