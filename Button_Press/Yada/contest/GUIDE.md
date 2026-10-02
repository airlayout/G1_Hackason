# エレベーターのボタン押し：シミュレーションの評価環境

G1 がエレベーター乗り場の呼びボタン（▲ / ▼）を押す動作を、シミュレーションで評価するための環境。
作り方（画像認識 + IK、VLA、強化学習など）や、プログラムの作り（制御の繰り返しの回し方）は自由。
最後は同じコードを実機の G1 で動かすことを目指す。

## 使い方は 2 通り

| | A. 実機と同じ口（おすすめ） | B. Python の API |
|---|---|---|
| つなぎ方 | DDS（`rt/lowstate`、`rt/arm_sdk`、`rt/lowcmd`、`LocoClient`）+ 頭カメラの RGB-D（ZMQ）。実機と同じ | `contest/interface.py` の `Agent` を作り、`act()` で目標を返す |
| チームのコード | 実機用のコードをそのまま使う（つなぎ先だけ変える）。制御の繰り返しは自分で回す | 制御の繰り返しは、評価環境のランナーが回す |
| 速さ | 実時間（1 試行 15〜20 秒） | 実時間より速い（MuJoCo で 1 試行約 1 秒） |
| 向いている使い方 | 実機に持っていく前の確認、J1-gen のような実機用の作り | VLA の学習・データ集め、たくさん回す評価 |
| シミュレーター | MuJoCo（Isaac Sim は準備中） | MuJoCo / Isaac Sim |

どちらも、シーン（エレベーター乗り場）、試行の条件（乱数の種）、判定は同じ。

## A. 実機と同じ口（模擬 G1）

### 模擬 G1 を起動する

```bash
P=~/miniconda3/envs/lerobot/bin/python      # unitree_sdk2py と mujoco が入った Python
$P Button_Press/Yada/sim/mujoco/g1_sim_server.py --seed 3            # --view で画面も開く
```

起動すると、指示を `_local/button_press_yada/sim_server/task.json` に書く（`seed`、`target`、`instruction`、つなぎ先）。

### 実機用のコードをつなぐ

実機との違いは、つなぎ先だけ。

| | 実機 | 模擬 G1 |
|---|---|---|
| DDS の口 | G1 につないでいる有線 NIC（domain 0） | `lo`（domain 1） |
| 頭カメラ | PC2 の IP の 5556（深度付き）/ 5555（RGB） | `127.0.0.1` の 5556 / 5555 |
| 見分け方 | — | `rt/lowstate` の `reserve[0]` に目印（J1-gen の `dds.SIM_MARKER`）が入る |

J1-gen の実機用のスクリプトなら `--network-interface lo --camera-config camera_sim.yaml` を付けるだけで動く。

### 模擬 G1 がすること・しないこと

- 上半身: `rt/arm_sdk` を受け取り、weight で内蔵コントローラの保持とブレンドする（J1-gen の SimBackend と同じ近似）。
  `rt/lowcmd` も受け取る
- 下半身: `LocoClient`（"sport" サービス）の依頼に応答する。**ただし腰（pelvis）は固定なので、歩く指令では動かない**
  （受け付けて記録するだけ）
- 腕の重力: Unitree の内蔵コントローラは重力を補償しない、と仮定している（`configs/contest.yaml` の
  `arm_sdk_gravity_compensation: false`。実機では未確認）。腕は PD の強さだけで支えるので、重さの分だけ下がる
- 指令が 0.5 秒途切れると、内蔵コントローラが姿勢を保持する状態に戻る

### 判定

最初の指令（`arm_sdk` / `lowcmd` / `LocoClient`）が届いた時刻から計る。
ボタンが点灯した時点で判定し、`_local/button_press_yada/sim_server/result_seed<N>.json` に書いて、5 秒後に止まる。

| 結果 | 条件 |
|---|---|
| `success` | 指定のボタンが点灯した（もう一方は点灯していない） |
| `wrong` | 違うボタンが点灯した |
| `timeout` | 制限時間（30 秒）内に点灯しなかった |
| `no_command` | 300 秒待っても指令が来なかった |

記録だけするもの: 壁・扉・盤との接触力の最大、受け取った指令の数。

### たくさんの試行を評価する

試行ごとに模擬 G1 を起動し、チームのコマンドを動かし、判定を集める。

```bash
$P Button_Press/Yada/contest/evaluate_dds.py --seeds practice --client "<自分のコマンド>"
```

チームのコマンドは試行ごとに新しく起動される。指示のファイルのパスは環境変数 `YADA_TASK_FILE` で渡す。

### B のエージェントを、実機と同じ口で動かす

`contest/run_dds.py` は、B の `Agent` を DDS + ZMQ で動かす（`contest/robots/real_g1.py`）。
同じコマンドのまま、つなぎ先を変えると実機で動く（**実機ではまず `--dry-run`**。J1-gen の
`real/REAL_DAY_PROCEDURE.md` の手順に従う）。

```bash
$P Button_Press/Yada/contest/run_dds.py --agent Button_Press/Yada/contest/example_agent              # 模擬 G1
$P Button_Press/Yada/contest/run_dds.py --agent ... --network-interface <NIC> --camera-host <PC2> \
    --target up --instruction "上のボタンを押して" --dry-run                                        # 実機
```

## B. Python の API

`Button_Press/<チーム名>/agent.py` に `Agent` を継承したクラスと `make_agent()` を書く。
見本は `contest/example_agent/agent.py`（深度でボタンを見つける → IK → 押す）。

```python
from contest.interface import Action, Agent, Observation, TaskInfo, UPPER_BODY_IDX

class MyAgent(Agent):
    def reset(self, task: TaskInfo) -> None: ...      # 試行の開始に 1 回
    def act(self, obs: Observation) -> Action: ...    # 50 Hz（0.02 秒ごと）

def make_agent() -> Agent:
    return MyAgent()
```

| | 項目 | 内容 |
|---|---|---|
| `TaskInfo` | `instruction` / `target` | 言葉の指示 / 同じ指示の記号（`"up"` / `"down"`） |
| | `sim` | `"mujoco"` / `"isaac"` / `"sim_dds"` / `"real"` |
| | `time_limit`, `control_dt` | 制限時間（30 秒）、`act()` の周期（0.02 秒） |
| | `base_enabled` | 下半身の指令が効くか |
| | `upper_kp`, `upper_kd` | 上半身の PD の強さ |
| `Observation` | `rgb`, `depth`, `K` | 頭カメラのカラー (480, 640, 3)、深度 [m]、内部パラメータ |
| | `q`, `dq`, `t` | 29 関節の角度と速度（motor 番号順）、試行の開始からの時間 |
| | `imu_quat`, `image_t` | 腰の IMU の姿勢 (w, x, y, z)、画像を撮った時刻 |
| `Action` | `q_target` | 上半身 17 関節（motor 12〜28）の目標の角度。実機の `rt/arm_sdk` と同じ |
| | `base_cmd` | 下半身の速度 `(vx, vy, yaw_rate)`。実機の `LocoClient.Move` と同じ |
| | `done` | 押し終わったら `True` |

### 試行回数と並列数（MuJoCo）

- **並列数は PC の性能から自動で決める**: 使えるコアの数 − 1 と、空きメモリの 7 割 ÷ 0.8 GB（1 プロセスのメモリ）の
  小さいほう。`--workers N` で変えられる。`--view` のときは 1。同じ種なら、並列でも 1 つずつでも結果は同じ
- **新しい PC で初めて評価するときは、試行回数を聞く**: CPU・メモリ・GPU と MuJoCo の速さを調べ、
  試行回数ごとの見積もりの時間を示してから聞く。答えはその PC の設定（`_local/button_press_yada/machine_profiles.json`）
  に保存し、次からは聞かない。見積もりは、評価のたびに実際にかかった時間で直していく
- 試行の決め方（上ほど優先）: `--seed`（種を直接）> `--trials N`（種 0〜N−1）> `--seeds`（seeds.yaml の組）> この PC の設定
- 設定をやり直す: `--reset-machine`

参考（2026-10-02、AMD EPYC 8 コア / 62 GB、並列数 7）: 20 試行 約 10 秒、50 試行 約 20 秒（1 つずつだと 1 試行 約 1.3 秒）。

```bash
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名>                          # MuJoCo（試行回数は PC の設定）
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --trials 100 --set realistic --report
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --seed 3 --view
bash Button_Press/Yada/contest/evaluate_isaac.sh --agent Button_Press/<チーム名> --seeds practice  # Isaac Sim
```

Isaac Sim では、エージェントは Isaac Sim の Python（`env_isaaclab`）で動くので、使うライブラリはそこに入れておく。
壁・扉・盤との接触力はまだ測っていない（結果では `null`）。

ランナーの安全のための処理: 関節の可動範囲から 0.05 rad 内側に丸める。1 周期の移動量を 1.0 rad/s × 0.02 s に丸める。

## 試行ごとに変わる条件（A と B で同じ）

`configs/contest.yaml` の `randomize`: 壁までの距離（0.40〜0.44 m）、盤の左右の位置、盤の高さ（床から 0.96〜1.02 m）、
床の明るさ、押すボタン、指示の文。G1 は腰を固定し、両腕を下ろした姿勢から始まる。
練習用の種は `contest/seeds.yaml` の `practice`。

## 評価セット（`--set`）

| セット | 中身 |
|---|---|
| `basic`（既定） | 上の「試行ごとに変わる条件」だけ。センサもモータも理想的 |
| `realistic` | `basic` に加えて、実機に近づける乱しを入れる（`configs/contest.yaml` の `realism`） |

`realistic` の乱し（どれも目安。実機で測った値ではない）:

| 分類 | 中身 | 範囲 |
|---|---|---|
| 深度 | 距離の 2 乗に比例するノイズ（空間的にむらがある）、穴、物の縁の欠け、1 mm 単位 | 1 m で 4〜12 mm |
| カラー | ノイズ、明るさ | ±15% |
| カメラの取り付け | 位置と角度の誤差（`robot.yaml` の値はずらさない = エージェントは知らない） | ±5 mm、±1.5° |
| 遅れ | 画像の遅れ、画像のコマ数、上半身の指令の遅れ | 40〜120 ms、12〜30 fps、0〜20 ms |
| 関節のセンサ | 角度と速度のノイズ | 0.0005 rad、0.01 rad/s |
| モータ | PD の強さのばらつき（`upper_kp` は公称の値のまま）、armature、摩擦 | ±10%、0.001〜0.03、0〜0.1 Nm |
| 体の揺れ | 腰の前後・左右・上下・傾きの、ゆっくりした揺れ（腰をばねで揺らす。押した反動でもわずかに動く） | 2〜6 mm、0.2〜0.8°、0.2〜1 Hz |
| 立ち位置 | 乗り場に対する位置と向きのずれ | ±1.5 cm、±3° |

乱しは基本の条件とは別の乱数で決めるので、同じ種なら盤の配置などは `basic` と同じ（乱しの影響だけを比べられる）。
観測の `imu_quat`（腰の傾き）と `image_t`（画像を撮った時刻）は、揺れと遅れに対処するのに使える。

```bash
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --set realistic --seeds practice
```

今のところ `realistic` は B の MuJoCo だけ（A の模擬 G1 と Isaac Sim はこれから）。

## 弱点のレポート

評価のあとに、どの条件に弱かったかをまとめた Markdown を作る（`_local/button_press_yada/reports/`）。

```bash
# 評価と一緒に作る
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --set realistic --seeds practice --report
# 乱しを 1 種類ずつ入れて、原因を切り分ける（乱しなし・各種類だけ・すべて × 試行。MuJoCo で 200 試行 約 10 分）
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --ablation --seeds practice --report
# 保存した結果から作る
$P Button_Press/Yada/contest/report.py _local/button_press_yada/results/<結果>.json
```

| 節 | 中身 |
|---|---|
| どこで失敗したか | 失敗の分類と回数（下の表） |
| 乱しを 1 種類ずつ入れた評価 | `--ablation` のときだけ。種類ごとの成功率と、乱しなしとの差。**原因が分かる** |
| 条件ごとの成功率 | 条件の値で試行を半分に分けて、成功率を比べる。一緒に変わった条件の影響も混ざる（相関） |
| 失敗した試行 | 目立って厳しかった条件と、再現のコマンド |

失敗の分類は、シミュレーターから見て分かることだけで決める（エージェントの中身には依らない）:

| 分類 | 決め方 |
|---|---|
| 腕をほとんど動かさなかった | 上半身の関節の変化が 0.05 rad 未満（見つけられなかった・計画できなかった可能性） |
| 壁・扉・盤に強くぶつかった | 接触力が 20 N を超えた |
| 触れたが点灯しなかった | 目標のボタンに触れたが、2.5 mm まで沈まなかった |
| 近くまで来たが触れなかった | 指先の球の中心とボタンの面の中心が 3 cm 以内まで来た |
| 近づけなかった | それより遠い |
| 違うボタンを押した | もう一方が点灯した |

指先の距離などは MuJoCo（B）でだけ測っている。Isaac Sim と模擬 G1 は、まだ「分類できない」になる。

## 守ってほしいこと

1. **シミュレーターの中身を使わない。** ボタンの位置は観測（画像・深度）から求める。
   機体の公式モデル（URDF）を Pinocchio などで読むのはよい（J1-gen の IK やカメラの変換も使ってよい）。
2. **B のエージェントはシミュレーター（`mujoco` / `isaacsim` / `isaaclab`）を import しない。** 実機で同じコードを動かすため。
3. **練習用の種に合わせて作り込まない。** 条件が少し変わっても押せることを目指す。
