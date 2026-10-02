# エレベーターのボタン押し：シミュレーションの評価環境

G1 がエレベーター乗り場の呼びボタン（▲ / ▼）を押す動作を、シミュレーションで評価するための環境。
作り方（画像認識 + IK、VLA、強化学習など）や、プログラムの作り（制御の繰り返しの回し方）は自由。
最後は同じコードを実機の G1 で動かすことを目指す。

コマンドはすべてリポジトリ直下（`G1_Hackason/`）で実行する。`$P` は、下の「準備」で用意した Python を指す。

## 目次

1. [まず動かす（5 分）](#1-まず動かす5-分)
2. [準備](#2-準備)
3. [使い方は 2 通り（A / B）](#3-使い方は-2-通りa--b)
4. [A. 実機と同じ口（模擬 G1）](#4-a-実機と同じ口模擬-g1)
5. [B. Python の API](#5-b-python-の-api)
6. [試行の条件と評価セット](#6-試行の条件と評価セット)
7. [試行回数と並列数](#7-試行回数と並列数)
8. [弱点のレポート](#8-弱点のレポート)
9. [画面で見る・画像を作る](#9-画面で見る画像を作る)
10. [座標と関節の決まり](#10-座標と関節の決まり)
11. [エージェントを作るときのコツ](#11-エージェントを作るときのコツ)
12. [結果のファイル](#12-結果のファイル)
13. [うまくいかないとき](#13-うまくいかないとき)
14. [まだできていないこと](#14-まだできていないこと)
15. [守ってほしいこと](#15-守ってほしいこと)

---

## 1. まず動かす（5 分）

準備（[2.](#2-準備)）が済んでいれば、次の 4 行で、見本のエージェントがボタンを押すところまで確かめられる。

```bash
P=~/miniconda3/envs/lerobot/bin/python                    # 自分の Python に置き換える
bash Button_Press/J1-gen/sim/fetch_models.sh                # G1 の公式モデルを取得する（初回だけ）
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/Yada/contest/example_agent --seeds smoke
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/Yada/contest/example_agent --seed 1 --view   # 画面で見る
```

3 行目で `成功率 100%（{'success': 2}）` と出れば動いている。

自分のエージェントを作るときは、見本をコピーして始めるとよい。

```bash
mkdir -p Button_Press/<チーム名>
cp Button_Press/Yada/contest/example_agent/agent.py Button_Press/<チーム名>/agent.py
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名>
```

---

## 2. 準備

### 2.1 Python（B だけ使う場合）

Python 3.12 の環境に、次を入れる（J1-gen の `requirements.txt` + 見本が使う `scipy`・画像を保存する `Pillow`）。

```bash
$P -m pip install "numpy>=1.26" "PyYAML>=6.0" "mujoco>=3.2" "pin>=3.0" scipy pillow opencv-python "pyzmq>=26.0"
```

- `pin` は Pinocchio（IK と重力の計算）の pip の配布名
- このマシンでは conda の `lerobot` 環境（`~/miniconda3/envs/lerobot`）に入っている
  （2026-10-02: mujoco 3.11.0、pin 3.9.0、scipy 1.18.0、pyzmq 27.1.0、opencv 4.13.0）

### 2.2 DDS（A の模擬 G1 を使う場合）

A では、さらに CycloneDDS と `unitree_sdk2py` が要る。入れ方はリポジトリ直下の [SETUP.md](../../../SETUP.md) の
「1.2 CycloneDDS をソースからビルド」と「1.3 unitree_sdk2_python」。

### 2.3 G1 の公式モデル

```bash
bash Button_Press/J1-gen/sim/fetch_models.sh
```

`_local/button_press/models/g1_description/` に取り出す（unitree_ros のコミットを固定している。J1-gen と共通）。
`_local/` は git の対象外なので、PC ごとに 1 回実行する。

### 2.4 Isaac Sim（Isaac Sim でも評価する場合）

pip 版 Isaac Sim 6.0.1 + IsaacLab 3.0.0-beta2。入れ方はリポジトリ直下の `CLAUDE.md` と `IsaacSim_Env/SETUP.md`。
置き場所が違うときは `Button_Press/Yada/sim/isaac/env.sh` の `ISAAC_SIM` と `ISAACLAB` を直す（環境変数でも変えられる）。

Isaac Sim では、エージェントは Isaac Sim の Python（`env_isaaclab`）で動く。エージェントが使うライブラリは、
そこにも入れておく（見本なら `pin` と `scipy`）。

### 2.5 画面

画面で見る（`--view`）・画像を作るには、描画（OpenGL）が要る。

- 画面のある PC: そのまま（MuJoCo は glfw を使う）
- 画面の無い端末（ssh など）: `MUJOCO_GL=egl` か、仮想の画面（`DISPLAY=:2` など）を使う。
  このマシンでは EGL が動かず、`DISPLAY=:2` の glfw なら動いた

---

## 3. 使い方は 2 通り（A / B）

| | A. 実機と同じ口（おすすめ） | B. Python の API |
|---|---|---|
| つなぎ方 | DDS（`rt/lowstate`、`rt/arm_sdk`、`rt/lowcmd`、`LocoClient`）+ 頭カメラの RGB-D（ZMQ）。実機と同じ | `contest/interface.py` の `Agent` を作り、`act()` で目標を返す |
| チームのコード | 実機用のコードをそのまま使う（つなぎ先だけ変える）。制御の繰り返しは自分で回す | 制御の繰り返しは、評価環境のランナーが回す |
| 速さ | 実時間（1 試行 約 14 秒） | 実時間より速い（MuJoCo で 1 試行 約 1 秒。並列にするとさらに速い） |
| 向いている使い方 | 実機に持っていく前の確認、J1-gen のような実機用の作り | VLA の学習・データ集め、たくさん回す評価 |
| シミュレーター | MuJoCo | MuJoCo / Isaac Sim |

どちらも、シーン（エレベーター乗り場）、試行の条件（乱数の種）、判定は同じ。
見本のエージェントは、どの経路でも練習用の 20 試行すべてで成功した（MuJoCo 5.47 秒、Isaac Sim 5.48 秒、
模擬 G1 5.46 秒。点灯までの平均）。

---

## 4. A. 実機と同じ口（模擬 G1）

### 4.1 模擬 G1 を起動する

```bash
$P Button_Press/Yada/sim/mujoco/g1_sim_server.py --seed 3            # --view で画面も開く
```

起動すると、指示を `_local/button_press_yada/sim_server/task.json` に書く（`seed`、`target`、`instruction`、つなぎ先）。

### 4.2 実機用のコードをつなぐ

実機との違いは、つなぎ先だけ。

| | 実機 | 模擬 G1 |
|---|---|---|
| DDS の口 | G1 につないでいる有線 NIC（domain 0） | `lo`（domain 1） |
| 頭カメラ | PC2 の IP の 5556（深度付き）/ 5555（RGB） | `127.0.0.1` の 5556 / 5555 |
| 見分け方 | — | `rt/lowstate` の `reserve[0]` に目印（J1-gen の `dds.SIM_MARKER`）が入る |

J1-gen の実機用のスクリプトなら `--network-interface lo --camera-config camera_sim.yaml` を付けるだけでつながる。

### 4.3 模擬 G1 がすること・しないこと

- 上半身: `rt/arm_sdk` を受け取り、weight で内蔵コントローラの保持とブレンドする（J1-gen の SimBackend と同じ近似）。
  `rt/lowcmd` も受け取る
- 下半身: `LocoClient`（"sport" サービス）の依頼に応答する。**ただし腰（pelvis）は固定なので、歩く指令では動かない**
  （受け付けて記録するだけ）
- 腕の重力: Unitree の内蔵コントローラは重力を補償しない、と仮定している（`configs/contest.yaml` の
  `arm_sdk_gravity_compensation: false`。実機では未確認）。腕は PD の強さだけで支えるので、重さの分だけ下がる
- 指令が 0.5 秒途切れると、内蔵コントローラが姿勢を保持する状態に戻る

### 4.4 判定

最初の指令（`arm_sdk` / `lowcmd` / `LocoClient`）が届いた時刻から計る。
ボタンが点灯した時点で判定し、`_local/button_press_yada/sim_server/result_seed<N>.json` に書いて、5 秒後に止まる。

| 結果 | 条件 |
|---|---|
| `success` | 指定のボタンが点灯した（もう一方は点灯していない） |
| `wrong` | 違うボタンが点灯した |
| `timeout` | 制限時間（30 秒）内に点灯しなかった |
| `no_command` | 300 秒待っても指令が来なかった |
| `aborted` | 判定の前に模擬 G1 を止めた（Ctrl+C など） |

記録だけするもの: 壁・扉・盤との接触力の最大、受け取った指令の数。

### 4.5 たくさんの試行を評価する

試行ごとに模擬 G1 を起動し、チームのコマンドを動かし、判定を集める。

```bash
$P Button_Press/Yada/contest/evaluate_dds.py --seeds practice --client "<自分のコマンド>"
```

- チームのコマンドは試行ごとに新しく起動される。指示のファイルのパスは環境変数 `YADA_TASK_FILE` で渡す
- 模擬 G1 とコマンドのログは `_local/button_press_yada/logs/evaluate_dds/` に残る
- 1 つの PC で同時に動かせる模擬 G1 は 1 つだけ（DDS の口とカメラのポートが同じため）

### 4.6 B のエージェントを、実機と同じ口で動かす

`contest/run_dds.py` は、B の `Agent` を DDS + ZMQ で動かす（`contest/robots/real_g1.py`）。
同じコマンドのまま、つなぎ先を変えると実機で動く（**実機ではまず `--dry-run`**。J1-gen の
`real/REAL_DAY_PROCEDURE.md` の手順に従う）。

```bash
$P Button_Press/Yada/contest/run_dds.py --agent Button_Press/Yada/contest/example_agent              # 模擬 G1
$P Button_Press/Yada/contest/run_dds.py --agent ... --network-interface <NIC> --camera-host <PC2> \
    --target up --instruction "上のボタンを押して" --dry-run                                        # 実機
$P Button_Press/Yada/contest/evaluate_dds.py --seeds practice \
    --client "$P Button_Press/Yada/contest/run_dds.py --agent Button_Press/<チーム名>"              # 模擬 G1 でたくさん
```

`real_g1.py` に入っている安全の仕組み: weight を 0 → 1、1 → 0 へ 2 秒かけて変える、lowstate が 0.5 秒途切れたら止める、
Ctrl+C で安全に終わる。J1-gen の `ArmCommander` にある作業空間の箱・動いたかの確認・腰のずれの確認は、まだ無い。

---

## 5. B. Python の API

### 5.1 エージェントの形

`Button_Press/<チーム名>/agent.py` に、`Agent` を継承したクラスと `make_agent()` を書く。
見本は `contest/example_agent/agent.py`（深度でボタンを見つける → IK → 押す）。

```python
from contest.interface import Action, Agent, Observation, TaskInfo, UPPER_BODY_IDX

class MyAgent(Agent):
    def reset(self, task: TaskInfo) -> None: ...      # 試行の開始に 1 回
    def act(self, obs: Observation) -> Action: ...    # 50 Hz（0.02 秒ごと）
    def close(self) -> None: ...                      # 全部の試行の終わりに 1 回（省略してよい）

def make_agent() -> Agent:
    return MyAgent()
```

- `agent.py` のあるフォルダは import の検索先に入るので、同じフォルダの別ファイルを import できる
- `Button_Press/Yada/` も検索先に入るので、`from common.j1gen_bridge import j1gen` で J1-gen の部品（IK など）を使える
- エージェントの同じオブジェクトが、たくさんの試行で使い回される（試行ごとに `reset()` が呼ばれる）。
  並列のときは、プロセスごとに 1 つずつ作られる

### 5.2 入力と出力

| | 項目 | 内容 |
|---|---|---|
| `TaskInfo` | `instruction` / `target` | 言葉の指示 / 同じ指示の記号（`"up"` / `"down"`） |
| | `sim` | `"mujoco"` / `"isaac"` / `"sim_dds"` / `"real"` |
| | `time_limit`, `control_dt` | 制限時間（30 秒）、`act()` の周期（0.02 秒） |
| | `base_enabled` | 下半身の指令が効くか（今のシミュレーションは `False`） |
| | `upper_kp`, `upper_kd` | 上半身の PD の強さ（公称の値） |
| `Observation` | `rgb`, `depth`, `K` | 頭カメラのカラー (480, 640, 3) uint8、深度 (480, 640) float32 [m]（測れない画素は 0）、内部パラメータ (3, 3) |
| | `q`, `dq`, `t` | 29 関節の角度 [rad] と速度（motor 番号順）、試行の開始からの時間 [秒] |
| | `imu_quat`, `image_t` | 腰の IMU の姿勢 (w, x, y, z)、画像を撮った時刻（遅れがあると `t` より前） |
| `Action` | `q_target` | 上半身 17 関節（motor 12〜28）の目標の角度 [rad]。実機の `rt/arm_sdk` と同じ |
| | `base_cmd` | 下半身の速度 `(vx [m/s], vy [m/s], yaw_rate [rad/s])`。実機の `LocoClient.Move` と同じ |
| | `done` | 押し終わったら `True`（点灯していなければ `gave_up` になる） |

上半身は PD で目標を追う（腰 kp 300 / kd 5、腕 kp 60 / kd 1.5、手首 kp 40 / kd 1.5。腕と手首は実機の arm_sdk と同じ）。

### 5.3 評価する

```bash
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名>                          # MuJoCo（試行回数は PC の設定）
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --trials 100 --set realistic --report
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --seed 3 --view           # 1 試行を画面で見る
bash Button_Press/Yada/contest/evaluate_isaac.sh --agent Button_Press/<チーム名> --seeds practice  # Isaac Sim
bash Button_Press/Yada/contest/evaluate_isaac.sh --agent Button_Press/<チーム名> --seed 3 --viz kit # Isaac Sim を画面で見る
```

| 引数 | 意味 |
|---|---|
| `--set basic` / `realistic` | 評価セット（[6.](#6-試行の条件と評価セット)） |
| `--seed N`（複数可）/ `--trials N` / `--seeds practice` | 試行の決め方（[7.](#7-試行回数と並列数)） |
| `--report` | 弱点のレポートも作る（[8.](#8-弱点のレポート)） |
| `--ablation` / `--ablation-only <種類>` | 乱しを 1 種類ずつ入れて評価する（[8.](#8-弱点のレポート)） |
| `--workers N` | 並列数（MuJoCo。省略すると PC の性能から決める） |
| `--view` | 画面で見る（実時間で進む。並列にしない） |
| `--out <パス>` | 結果の JSON の保存先 |

Isaac Sim は、アプリを 1 回だけ起動し（2〜5 分）、試行ごとにシーンを作り直す。1 試行に実時間で約 37 秒かかる（並列にしていない）。

ランナーの安全のための処理（MuJoCo・Isaac Sim・実機で同じ）: 関節の可動範囲から 0.05 rad 内側に丸める。
1 周期の移動量を 1.0 rad/s × 0.02 s に丸める。丸めた回数は結果に残る。

---

## 6. 試行の条件と評価セット

### 6.1 試行ごとに変わる条件（A と B で同じ）

`configs/contest.yaml` の `randomize`: 壁までの距離（0.40〜0.44 m）、盤の左右の位置（y = −0.13〜−0.07 m）、
盤の高さ（床から 0.96〜1.02 m）、床の明るさ、押すボタン、指示の文（ボタンごとに 4 通り）。
G1 は腰を固定し、両腕を下ろした姿勢から始まる。同じ種なら、どのシミュレーターでもどの PC でも同じ条件になる。
練習用の種は `contest/seeds.yaml` の `practice`（0〜19）。

### 6.2 評価セット（`--set`）

| セット | 中身 |
|---|---|
| `basic`（既定） | 上の条件だけ。センサもモータも理想的 |
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

今のところ `realistic` は B の MuJoCo だけ（A の模擬 G1 と Isaac Sim はこれから）。

参考: 見本のエージェントは `realistic` で成功率 0〜12%（`basic` は 100%）。弱点はカメラの取り付けの角度と、
立ち位置の向き（壁がまっすぐ正面にあることを前提にボタンを探しているため。1〜2° のずれで見つけられなくなる）。

---

## 7. 試行回数と並列数

- **並列数は PC の性能から自動で決める**（MuJoCo）: 使えるコアの数 − 1 と、空きメモリの 7 割 ÷ 0.8 GB
  （1 プロセスのメモリ）の小さいほう。`--workers N` で変えられる。`--view` のときは 1。
  同じ種なら、並列でも 1 つずつでも結果は同じ
- **新しい PC で初めて評価するときは、試行回数を聞く**: CPU・メモリ・GPU と MuJoCo の速さを調べ、
  試行回数ごとの見積もりの時間を示してから聞く。答えはその PC の設定（`_local/button_press_yada/machine_profiles.json`）
  に保存し、次からは聞かない。見積もりは、評価のたびに実際にかかった時間で直していく。
  画面から入力できないとき（ほかのスクリプトから呼んだときなど）は、聞かずに既定の 20 にする
- 試行の決め方（上ほど優先）: `--seed`（種を直接）> `--trials N`（種 0〜N−1）> `--seeds`（seeds.yaml の組）> この PC の設定
- 設定をやり直す: `--reset-machine`

参考（2026-10-02、AMD EPYC 8 コア / 62 GB、並列数 7）: 20 試行 約 10 秒、50 試行 約 20 秒（1 つずつだと 1 試行 約 1.3 秒）。
Isaac Sim と模擬 G1 は並列にしていない（Isaac Sim 1 試行 約 37 秒、模擬 G1 1 試行 約 14 秒）。

---

## 8. 弱点のレポート

評価のあとに、どの条件に弱かったかをまとめた Markdown を作る（`_local/button_press_yada/reports/`）。

```bash
# 評価と一緒に作る
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --set realistic --trials 50 --report
# 乱しを 1 種類ずつ入れて、原因を切り分ける（乱しなし・各種類だけ・すべての 10 通り × 試行）
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/<チーム名> --ablation --trials 20 --report
# 保存した結果から作る（複数の結果をまとめてもよい）
$P Button_Press/Yada/contest/report.py _local/button_press_yada/results/<結果>.json
```

| 節 | 中身 |
|---|---|
| どこで失敗したか | 失敗の分類と回数（下の表）と、見直すところの目安 |
| 乱しを 1 種類ずつ入れた評価 | `--ablation` のときだけ。種類ごとの成功率と、乱しなしとの差。**原因が分かる** |
| 条件ごとの成功率 | 条件の値で試行を半分に分けて、成功率を比べる。`--ablation` のときは乱しの種類ごとに、どの値から弱くなるかが分かる |
| 失敗した試行 | 目立って厳しかった条件と、再現のコマンド（`--view` で画面で見られる） |

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
条件ごとの成功率は相関なので、ほかの条件の影響も混ざる。試行が少ない（目安 20 未満）と、差はたまたまのことがある。

---

## 9. 画面で見る・画像を作る

| やりたいこと | コマンド |
|---|---|
| シーンだけを見る（MuJoCo） | `$P Button_Press/Yada/sim/mujoco/view_hall.py`（Ctrl + 右ドラッグでボタンを押せる、R で消灯） |
| シーンだけを見る（Isaac Sim） | `bash Button_Press/Yada/sim/isaac/run.sh`（Shift + 左ドラッグでボタンを押せる） |
| エージェントの 1 試行を見る | `evaluate.py --seed N --view`（MuJoCo）/ `evaluate_isaac.sh --seed N --viz kit`（Isaac Sim） |
| 模擬 G1 を見る | `g1_sim_server.py --seed N --view` |
| サンプルの画像（MuJoCo） | `$P Button_Press/Yada/sim/mujoco/sample_images.py --seeds 0 1 2` |
| サンプルの画像（Isaac Sim） | `bash Button_Press/Yada/sim/isaac/sample_images_isaac.sh --seed 1` |

サンプルの画像は `_local/button_press_yada/samples/` にできる:

- `sample_sheet.png`: 種ごとに、全体のカメラ / 頭カメラのカラーと深度（basic と realistic）を並べたもの
- `pressed_seed<N>.png`: 見本のエージェントが押して点灯した瞬間
- `overview_one_isaac.png`: Isaac Sim の 1 枚のまとめ（MuJoCo の画像を先に作っておくと、比較の画像も入る）

深度の色は、近い（0.2 m）ほど明るい黄色、遠い（1.5 m）ほど暗い青、測れない画素は黒。
MuJoCo と Isaac Sim は照明が違うので、同じシーンでも明るさが違う（Isaac Sim は白っぽく、点灯の色が見えにくい）。

---

## 10. 座標と関節の決まり

| 項目 | 決まり |
|---|---|
| 座標 | **pelvis 座標**: x 前方、y 左、z 上 [m]。IK（J1-gen の `ArmKinematics`）と同じ。床は z = −0.793 |
| 関節の並び | **motor 番号**（0〜28。lowcmd / lowstate の `motor_cmd[i]` と同じ）。名前は `contest.interface.JOINT_NAMES`。0〜11 脚、12〜14 腰（yaw, roll, pitch）、15〜21 左腕、22〜28 右腕 |
| `q_target` の並び | motor 12〜28 の 17 個（`UPPER_BODY_IDX`） |
| 四元数 | `imu_quat` は (w, x, y, z)（実機の lowstate と同じ）。IsaacLab 3.0 の中は (x, y, z, w) なので注意 |
| 深度 | 光学軸の向きの距離 [m]（RealSense と同じ）。測れない画素は 0 |
| 頭カメラ | `torso_link` に付いた RealSense D435（J1-gen の `configs/robot.yaml`）。47.6° 下向き、640 × 480、上下の画角 42.6° |
| カメラ → pelvis | J1-gen の `HeadCameraTransform.pelvis_from_optical(q の腰 3 関節)` |
| ボタン | 直径 40 mm、壁から 2 cm 出ている。押す向きは +x。2.5 mm 沈むと点灯（2.5 N 以上の力が要る）、最大 4 mm 沈む |
| 指先 | 右手・左手の中指の先に半径 8 mm の衝突判定の球（`robot.yaml` の `end_effector`。画像には映らない） |

---

## 11. エージェントを作るときのコツ

見本のエージェントを作ったときに、実際にはまった点（実機でも同じことが起きるはず）:

1. **腕は弱い PD なので、重力で垂れる**（kp 60）。見本は「重力のトルク ÷ kp」だけ目標をずらしている
   （Pinocchio の `computeGeneralizedGravity`）
2. **減衰が小さいので、止まったあともしばらく揺れる**。見本は、押す前に手前の位置で 1 秒待つ
3. **押し返される反動で指先が下がり、ボタンの縁から外れる**。見本は、実際の関節の角度から指先の位置を計算し（FK）、
   ずれを少しずつ足し込む（積分の補正）。**ボタンに触れる直前で足し込みを止める**（触れたあとは摩擦で指先が動けず、
   補正だけが膨らんで、力が横に逃げた）
4. **押す力を出すには、目標をボタンの面より奥に置く**（見本は面から 1.5 cm 奥。ボタンは 4 mm しか沈まない）
5. **IK を始める姿勢で、解が変わる**。腕を下ろした姿勢から始めると、肘を内側にたたむ解になり、肩が胴に当たった。
   見本は、肘を外に張って腕を上げた姿勢から始める
6. **カメラと体の向きのずれに弱い**（`realistic` で分かった）。壁がまっすぐ正面にあると決めつけず、深度から壁の面の
   向きを推定するとよい

---

## 12. 結果のファイル

`_local/button_press_yada/` の下（git の対象外）:

| 場所 | 中身 |
|---|---|
| `results/<エージェント>_<シミュレーター>_<セット>_<日時>.json` | 評価の結果（`evaluate.py`）。`summary`（成功率など）と、試行ごとの `results` |
| `reports/*.md` | 弱点のレポート |
| `sim_server/task.json`、`result_seed<N>.json` | 模擬 G1 の指示と判定 |
| `logs/` | Isaac Sim と模擬 G1 のログ |
| `samples/` | サンプルの画像 |
| `machine_profiles.json` | PC ごとの設定（並列数、試行回数、速さ） |
| `usd/` | Isaac Sim 用に変換した G1 の USD（初回に自動で作る） |

試行ごとの結果の主な項目:

| 項目 | 内容 |
|---|---|
| `outcome` | `success` / `wrong` / `timeout` / `gave_up` / `error` |
| `stage` | どこで失敗したか（[8.](#8-弱点のレポート)の分類） |
| `time_s` | 点灯までの時間 |
| `max_contact_force_n` | 壁・扉・盤との接触力の最大（Isaac Sim は `null`） |
| `clipped_limit_steps`、`clipped_rate_steps` | 安全のための処理で丸めた周期の数 |
| `act_time_mean_ms` | `act()` にかかった時間の平均 |
| `extra.conditions` | この試行の条件の値（盤の位置、乱しの大きさなど） |
| `extra.diag` | 指先とボタンの最も近い距離、ボタンの最大の沈み、触れたか、腕を動かした量 |
| `error` | エージェントの例外（あれば） |

---

## 13. うまくいかないとき

| 症状 | 対処 |
|---|---|
| `公式モデルが無い` | `bash Button_Press/J1-gen/sim/fetch_models.sh` を実行する |
| 画像を作るところで落ちる（`EGLError`、`glGetError` など） | 描画ができていない。`DISPLAY` を設定して `MUJOCO_GL=glfw` にするか、`MUJOCO_GL=egl` を試す（[2.5](#25-画面)） |
| Isaac Sim がなかなか始まらない | 起動に 2〜5 分かかる。ログ（`_local/button_press_yada/logs/`）を見る |
| Isaac Sim の画面がすぐ閉じる | `--headless` を付けていないか確かめる。IsaacLab 3.0 は `--viz kit` が無いと画面を開かない（`run.sh` は自動で付ける） |
| Isaac Sim でエージェントの import に失敗する | エージェントが使うライブラリを Isaac Sim の Python（`env_isaaclab`）に入れる |
| 模擬 G1 につながらない（`lowstate が届かない`） | 模擬 G1 を先に起動したか、ほかの模擬 G1 が動いていないか（ポートが同じ）を確かめる |
| 模擬 G1 で `interface "lo" is not multicast-capable` と出る | 問題ない（ループバックなのでマルチキャストを使わないだけ） |
| `run_dds.py` の終わりに `lowstate が … 届いていない` と出る | 模擬 G1 が判定のあとに止まったため。正常 |
| 見積もりの時間が実際と合わない | 何回か評価すると、実測で直っていく。`--reset-machine` で調べ直せる |
| 並列にすると遅い・落ちる | `--workers` を小さくする（メモリが足りない、描画の取り合いなど） |

---

## 14. まだできていないこと

| 項目 | 状態 |
|---|---|
| 下半身 | 腰を固定している。歩く指令（`base_cmd`、`LocoClient`）は受け付けるが動かない。脚だけの歩行のポリシーに替える予定 |
| `realistic` の乱し | B の MuJoCo だけ。Isaac Sim と模擬 G1 にはまだ入っていない |
| 失敗の分類・接触力 | Isaac Sim と模擬 G1 は、指先の距離などを測っていない（分類できない） |
| 並列化 | MuJoCo だけ。Isaac Sim（IsaacLab のように何組も並べる）と模擬 G1 は 1 つずつ |
| 模擬 G1 の Isaac Sim 版 | まだ無い |
| 照明 | MuJoCo と Isaac Sim で違う（揃えていない）。照明の変化も入れていない |
| 実機 | `real_g1.py` は模擬 G1 でだけ確かめた。実機では未確認 |
| 実機で確かめる仮定 | 腕の重力の補償（`arm_sdk_gravity_compensation`）、腰の PD の強さ、`realistic` の乱しの範囲 |
| J1-gen の実機用のスクリプト | 模擬 G1 につながるが、エレベーターのボタン用の教えた姿勢が無く、計画の段階で止まる |

---

## 15. 守ってほしいこと

1. **シミュレーターの中身を使わない。** ボタンの位置は観測（画像・深度）から求める。
   機体の公式モデル（URDF）を Pinocchio などで読むのはよい（J1-gen の IK やカメラの変換も使ってよい）。
2. **B のエージェントはシミュレーター（`mujoco` / `isaacsim` / `isaaclab`）を import しない。** 実機で同じコードを動かすため。
3. **練習用の種に合わせて作り込まない。** 条件が少し変わっても押せることを目指す。
