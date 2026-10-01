# エレベーターのボタン押しコンテスト：ルール

G1 の頭カメラで乗り場の呼びボタン（▲ / ▼）を見つけ、指示されたほうを押して点灯させる。
作り方は自由（画像認識 + IK、VLA、強化学習など）。最後は同じコードを実機の G1 で動かすことを目指す。

## 作るもの

`Button_Press/<チーム名>/agent.py` に、`contest/interface.py` の `Agent` を継承したクラスと、
それを返す `make_agent()` を書く。最初は見本（`contest/example_agent/agent.py`）をコピーするとよい。

```python
from contest.interface import Action, Agent, Observation, TaskInfo, UPPER_BODY_IDX

class MyAgent(Agent):
    def reset(self, task: TaskInfo) -> None: ...      # 試行の開始に 1 回
    def act(self, obs: Observation) -> Action: ...    # 50 Hz（0.02 秒ごと）

def make_agent() -> Agent:
    return MyAgent()
```

### 入力

| | 項目 | 内容 |
|---|---|---|
| `TaskInfo` | `instruction` | 言葉の指示（例:「上のボタンを押して」「Press the down button.」） |
| | `target` | 同じ指示の記号（`"up"` / `"down"`） |
| | `sim` | `"mujoco"` / `"isaac"` / `"real"` |
| | `time_limit`, `control_dt` | 制限時間（30 秒）、`act()` の周期（0.02 秒） |
| | `base_enabled` | 下半身の指令が効くか（今のシミュレーションは `False`） |
| | `upper_kp`, `upper_kd` | 上半身の PD の強さ（実機の arm_sdk と同じ値） |
| `Observation` | `rgb` | 頭カメラのカラー画像 (480, 640, 3) uint8 |
| | `depth` | 頭カメラの深度 (480, 640) float32 [m] |
| | `K` | カメラの内部パラメータ (3, 3) |
| | `q`, `dq` | 29 関節の角度 [rad] と速度（motor 番号順） |
| | `t` | 試行の開始からの時間 [秒] |

### 出力（`Action`）

| 項目 | 内容 |
|---|---|
| `q_target` | 上半身 17 関節（腰 3 + 左腕 7 + 右腕 7 = motor 12〜28）の目標の角度 [rad]。実機の `rt/arm_sdk` と同じ |
| `base_cmd` | 下半身の速度の指令 `(vx, vy, yaw_rate)`。実機の `LocoClient.Move` と同じ。今のシミュレーションでは無視される |
| `done` | 押し終わったら `True` |

上半身は PD で目標を追う（腕 kp 60、手首 kp 40。実機と同じで弱い）。重力で腕が垂れるので、見本は
「重力のトルク ÷ kp」だけ目標をずらしている。

## 守ること

1. **シミュレーター（`mujoco` / `isaacsim` / `isaaclab`）を import しない。** 実機で同じコードを動かすため。
   機体の公式モデル（URDF）を Pinocchio などで読むのはよい（J1-gen の IK やカメラの変換も使ってよい）。
2. **正解の値を使わない。** ボタンの位置は観測（画像・深度）から求める。シミュレーターの中身を覗かない。
3. **乱数の種に合わせて作り込まない。** 練習用の種（`seeds.yaml` の `practice`）は公開、本番用は運営だけが持つ。

## 採点

1 試行ごとに次のどれかになる。成功率が高い順、同じなら成功時の平均時間が短い順。

| 結果 | 条件 |
|---|---|
| `success` | 制限時間内に指定のボタンが点灯した（もう一方は点灯していない） |
| `wrong` | 違うボタンを点灯させた |
| `timeout` | 制限時間内に点灯しなかった |
| `gave_up` | `done` を返したが、点灯していない |
| `error` | 例外を出した、または出力の形が違う |

記録だけするもの（実機に持っていく前に見る）: 壁・扉・盤との接触力の最大、安全のための丸めの回数、`act()` の時間。

## 試行ごとに変わる条件

`configs/contest.yaml` の `randomize`: 壁までの距離（0.40〜0.44 m）、盤の左右の位置、盤の高さ（床から 0.96〜1.02 m）、
床の明るさ、押すボタン、指示の文。G1 は腰を固定し、両腕を下ろした姿勢から始まる。

## 動かし方

```bash
P=~/miniconda3/envs/lerobot/bin/python
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --seeds practice
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --seed 3 --view   # 画面で見る
```

シミュレーターはどちらを選んでもよい（今は MuJoCo のみ。Isaac Sim は準備中）。

## 安全のための処理（ランナーが行う。シミュレーションと実機で同じ）

- 関節の可動範囲から 0.05 rad 内側に丸める
- 1 周期の移動量を 1.0 rad/s × 0.02 s に丸める（目標が飛んでも腕が急に動かない）
- 実機では、さらに腕の weight を少しずつ上げる・非常停止などを足す
