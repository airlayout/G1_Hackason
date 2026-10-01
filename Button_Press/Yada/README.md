# Button_Press / Yada（エレベーターのボタン押し：VLA・画像認識の比較）

シミュレーション上でエレベーターのボタンを押す動作を、複数の方式
（VLA モデル、画像認識 + IK など）で作り、比べる。
シミュレーターは Isaac Sim と MuJoCo の両方を使う。

従来型（画像認識 → IK → 決めた軌道で押す）の実装は [../J1-gen/](../J1-gen/README.md) にある。
共通に使える部分（機体モデル、IK、カメラの取り付け位置など）は複製せず、J1-gen から読み込んで使う
（`common/j1gen_bridge.py`。どちらもパッケージ名が `common` なので別名で読み込む）。

## 構成

- `configs/elevator_hall.yaml` — 乗り場のシーンの寸法（MuJoCo と Isaac Sim で共通）
- `configs/contest.yaml` — コンテストの設定（制限時間、PD の強さ、安全のための処理、試行ごとに変える条件、指示の文）
- `contest/` — ボタン押しコンテスト（ルールは [contest/RULES.md](contest/RULES.md)）
  - `interface.py` — エージェントとロボットの間の決まり（`Agent` / `Observation` / `Action` / `TaskInfo`）
  - `runner.py` — 1 試行を動かす（観測 → `act()` → 安全のための処理 → 送る → 採点）。シミュレーションと実機で共通
  - `robots/` — ロボットの差し替え部分（今は `mujoco_robot.py`。Isaac Sim と実機はこれから）
  - `task.py` — 試行の条件を乱数の種から決める。`seeds.yaml` — 練習用の種
  - `evaluate.py` — 評価と採点
  - `example_agent/` — 見本のエージェント（深度でボタンを見つける → IK → 押す）
- `common/` — シミュレーターに依存しない部分
  - `scene_spec.py` — 設定を、箱とボタンの一覧（pelvis 座標）にする。呼びボタンの点灯の状態（`CallButtonState`）
  - `j1gen_bridge.py` — J1-gen/common を `j1gen_common` という別名で読み込む
  - `config.py` — 設定ファイルの読み込み
- `sim/mujoco/` — MuJoCo 版（`mujoco_hall.py` がシーンを作る、`view_hall.py` で見る・確かめる）
- `sim/isaac/` — Isaac Sim 版（`isaac_hall.py` がシーンを作る、`run.sh` → `view_hall_isaac.py` で見る・確かめる）
- `real/` — 実機用（まだ無い）
- `tests/` — テスト

## シーン：エレベーター乗り場の呼びボタン

画像は `view_hall.py --png` / `run.sh --png` で `_local/button_press_yada/` に作れる。

- G1 は胴体（pelvis）を固定し、両腕を下ろした姿勢（J1-gen の検出用の姿勢と同じ）
- 前方 0.42 m に壁、その右寄りにボタン盤（床から 1.00 m）、左に閉じた扉
- 呼びボタンは ▲（up）と ▼（down）の 2 つ。直径 40 mm、ばね付きで 4 mm 沈む。2.5 mm 以上沈むと「押した」と判定して点灯し、
  `reset_lights()` まで点灯したまま（実物の呼びボタンと同じ）。押すのに要る力は 2.5 N
- 公式モデルのハンドには衝突判定が無いので、中指の先に半径 8 mm の球（画像には映らない）を足している
- MuJoCo 版だけ、ボタンの重さを 0.2 kg にし、接触を硬くしている（`mujoco_hall.py` の `CONTACT_SOLREF`）。
  MuJoCo の接触の硬さは物の重さで決まり、20 g のままでは指がボタンにめり込んで押せなかったため。
  止まっているときの押す力はばねで決まるので変わらない
- MuJoCo と Isaac Sim は同じ `HallScene` から作るので寸法は同じ。G1 も同じ公式モデル（unitree_ros の
  `g1_29dof_rev_1_0`。MuJoCo は XML、Isaac Sim は URDF）

### 寸法の決め方（実測・確認したこと）

| 項目 | 値 | 理由 |
|---|---|---|
| ボタンの面の位置 | pelvis から前に 0.40 m、右に 0.10 m | J1-gen の IK で、右手の中指の先を押す向き（+x）にそろえて、手前 5 cm と押し込んだ位置の両方に届く（`tests/test_mujoco_hall.py`） |
| 盤の左右の位置 | y = −0.10 | y = −0.18 では頭カメラの視野（水平 ±27.6°）の外だった |
| 盤の高さ | 床から 1.00 m | 頭カメラは 47.6° 下向きで上下の画角が ±21.3°。1.05 m では盤の上の端が切れた |

## 使い方

前提: 公式モデルを取得しておく（J1-gen と共通）。

```bash
bash Button_Press/J1-gen/sim/fetch_models.sh
```

### MuJoCo

`mujoco`・`pinocchio`・`PyYAML`・`Pillow` が入った Python で動かす（このマシンでは conda の `lerobot` 環境）。

```bash
P=~/miniconda3/envs/lerobot/bin/python
$P Button_Press/Yada/sim/mujoco/view_hall.py                       # 画面で見る（Ctrl + 右ドラッグで押す、R で消灯）
$P Button_Press/Yada/sim/mujoco/view_hall.py --png _local/button_press_yada/hall     # 頭カメラと全体の画像
$P Button_Press/Yada/sim/mujoco/view_hall.py --press up --force 6  # 外から 6 N で押して、沈み量と点灯を確かめる
```

画面の無い端末では `MUJOCO_GL=egl` などを指定する（このマシンの EGL は動かず、`DISPLAY=:2` の glfw なら動いた）。

### Isaac Sim

pip 版 Isaac Sim 6.0.1 + IsaacLab 3.0.0-beta2（リポジトリ直下の CLAUDE.md）。パスは `sim/isaac/env.sh` で変える。
起動に 2〜5 分かかる。

```bash
bash Button_Press/Yada/sim/isaac/run.sh                                     # 画面で見る（Shift + 左ドラッグで押す）
bash Button_Press/Yada/sim/isaac/run.sh --headless --press up --png _local/button_press_yada/isaac_pressed
```

ボタンはばねの目標を変えて押す（`--press`）。指で押す確認は、Isaac Sim のロボットを作ってから。

### コンテスト（エージェントの評価）

```bash
P=~/miniconda3/envs/lerobot/bin/python
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/Yada/contest/example_agent --seeds practice
$P Button_Press/Yada/contest/evaluate.py --agent Button_Press/Yada/contest/example_agent --seed 3 --view
```

見本のエージェントは、練習用の 20 試行すべてで成功した（平均 5.5 秒、壁・盤への接触なし。MuJoCo）。

### テスト

```bash
PYTHON=~/miniconda3/envs/lerobot/bin/python bash Button_Press/Yada/tests/run_tests.sh
```

mujoco / pinocchio / 公式モデルが無いテストはスキップする。Isaac Sim は起動が重いのでテストに入れていない
（上の `run.sh --headless --press up` の出力で確かめる）。

## 状態

- [x] 乗り場のシーン（MuJoCo、Isaac Sim）。ボタンの押し込みと点灯を両方で確認した
- [x] コンテストの土台（インターフェース、ランナー、採点、MuJoCo のロボット、見本のエージェント）
- [x] 腕を動かして指で押す（見本のエージェント。MuJoCo で 20/20 成功）
- [ ] Isaac Sim のロボット（`contest/robots/isaac_robot.py`）
- [ ] 実機のロボット（`contest/robots/real_g1.py`。下半身は LocoClient、上半身は arm_sdk）
- [ ] シミュレーションの下半身を、脚だけのポリシーに差し替える（今は腰を固定）
- [ ] 画像（色）でボタンを見つける（見本は深度だけを使う）
- [ ] VLA 用のデータ収集・学習・評価
