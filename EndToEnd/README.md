# EndToEnd

Unitree G1のモデル開発に共通で使う、Isaac Sim / Isaac Labのシミュレーション環境。
身体29関節・平地・1体の`reset/step`環境と、任意コントローラーを接続する入口を実装した。
NN構造・正規化・履歴・タスク報酬は環境の外で扱う。

## まず動かす

このPCの既存Isaac Sim `/opt/isaac-sim`とIsaac Lab `_local/IsaacLab`を自動検出する。
Python依存は`EndToEnd/.venv`に分離し、既存のIsaacインストールは編集しない。
`uv`が必要。システムのPython 3.10ではなくIsaac同梱のPython 3.12を使う。

リポジトリのルートから実行する。

```bash
# 共通環境＋検証用の公開歩行モデルを準備する
bash EndToEnd/setup.sh --with-demo

# 平地を前進する。画面なし、シミュレーション10秒で終了
bash EndToEnd/run.sh --controller unitree --velocity 0.3 0 0 --steps 500

# GUIで同じ環境を表示する（デスクトップの端末から実行）
bash EndToEnd/run.sh --controller unitree --velocity 0.3 0 0 --steps 500 --viz kit --real-time
```

画面表示には接続先のデスクトップの`DISPLAY`・認証と、選択したRTX GPUから表示できる画面が必要。
このPCの現在のX画面では、RTXからの直接表示で`createSwapchain failed`を確認した。
画面なしの物理実行には影響しない。直接GUIを使う場合はRTX側で表示できるデスクトップから起動する。
ウィンドウなしのviewport画像取得も黒画像になったため、GUI表示・画像出力は追加調整が必要。
現時点の検証済み範囲は、状態観測と関節指令による画面なしの物理シミュレーション。
RTX GPUを自動選択する。このPCのGPU 0はGTX 1660 Ti、GPU 1がRTX 5060 Ti。
CUDAの番号はPCI順へ揃える。`CUDA_VISIBLE_DEVICES`を指定せず起動し、GPUは`.env`で選ぶ。
別マシンでは`configs/runtime.env.example`を参考に`.env`でインストール先とGPUを指定する。
検証した組合せは`configs/runtime.lock.json`に記録している。Isaac Labの最新mainへ自動更新はしない。

公開モデルを使わず、指令経路だけを検証する場合:

```bash
bash EndToEnd/setup.sh
bash EndToEnd/run.sh --controller hold --fixed-base --steps 100
bash EndToEnd/run.sh --controller examples.arm_sweep:create_controller --fixed-base --steps 200
```

`hold`はバランスを学習したモデルではなく、基準姿勢を返すだけの実装。
`arm_sweep`は独自コントローラーの接続例で、左肩を動かす。
`--fixed-base`は接続検証専用。歩行・立位の評価は基部固定なしで行う。

## モデルを差し替える

環境のAPIは`sim.envs.flat.G1FlatEnv`の`reset(seed=...)`、`step(command)`、`close()`。
アプリの起動・終了は呼び出し元が担当し、環境モジュールはAppLauncherの起動後にimportする。
共通の`g1_e2e.contracts.Controller`に合わせたアダプターを用意する。

```python
from g1_e2e.contracts import JointCommand

class MyController:
    def reset(self):
        # モデルの履歴・RNNの隠れ状態などを初期化
        ...

    def act(self, observation):
        # 必要な観測を選択し、モデル固有の前処理・推論を行う
        q_target_sdk = ...
        return JointCommand(position_rad=q_target_sdk)

def create_controller(*, robot, control_dt, options):
    return MyController()
```

```bash
bash EndToEnd/run.sh --controller my_package.my_model:create_controller --controller-options model.json
```

`my_package`を専用venvへインストールするか`PYTHONPATH`に追加する。
`options`は任意のJSONで、重みの場所やモデル固有パラメーターを渡せる。
コントローラーはIsaacのArticulationを直接操作せず、共通の物理量だけを受け渡す。
公開ONNX用の480次元入力・履歴5・action scaleは`inference/unitree_onnx.py`内に閉じている。

## 入出力の約束

- 身体29関節は常に`G1_IO_SPECIFICATION.md`付録AのSDK順。名前でIsaac側の順番へ変換する。
- 観測は関節角rad・関節速度rad/s・トルク推定値、身体角速度、姿勢xyzw、重力方向。
  `privileged`の位置・速度真値は、評価やcriticで使う値として分離する。
- `JointCommand`は29関節の位置rad・速度rad/s・feedforwardトルクN m。
  環境がPD制御を行い、ゲインは`configs/robot/g1_29dof.yaml`で変更できる。
  純粋なトルク制御には同設定のstiffness/dampingを0にした構成を使う。
- 環境はモデル入力の正規化・履歴連結・希望速度の付加を行わない。
- NaN/Inf・次元不一致は指令適用前に拒否。位置・速度・feedforwardトルクを設定の範囲へ制限する。
- 物理周期は200 Hz、指令更新は50 Hzが初期値。`configs/task/flat.yaml`で変更可能。
- 平地と照明はローカルに生成する。ロボットのURDFとmeshは取得済みの固定版を使う。
- 転倒を`terminated`、制限時間を`truncated`として通知。終了後はresetが必要。
  タスク固有の報酬・成功判定は学習側へ追加する。

RGB-D・LiDAR、並列環境、PPO・模倣学習は今後この土台へ追加する。
現時点では上記の状態観測とhybrid関節指令を使うモデルを接続できる。

## 再帰メモリモデル

`src/g1_e2e/models/`に、Isaacから独立したPyTorchのモデル定義を実装した。
初期設定は`configs/model/recurrent_perceiver.yaml`。
共通のシミュレーション環境は従来の物理量を受け渡し、このNN固有のトークン化や再帰状態を持たない。

```text
RGB / depth / LiDAR / joints / IMU / previous_action
                      ↓ modality別の投影
                 観測トークン O_t
                      ↓ cross-attention（queryはメモリ）
M_(t-1) ────────→ メモリ更新 F ────→ M_t ────→ 次周期
                                    ↓ query cross-attention
                            velocity / BEV / trajectory
```

メモリは64トークン×256次元、4層、8 headsを初期値とする。すべて設定で変更可能。
観測全体のself-attentionを避け、self-attentionは小さいメモリ集合内で行う。
観測の種類・入力次元・トークン上限、queryの種類・出力次元も設定で変更する。

| query | 入力 | 初期出力 |
|---|---|---|
| `velocity` | 1個の学習可能query | `[vx, vy, yaw_rate]`、m/s・m/s・rad/s |
| `bev` | 現在のbase座標のXYZ、m | その地点の学習した32次元特徴 |
| `trajectory` | 未来時間Δt、s | 現在のbaseに対する未来の位置XYZ、m + 姿勢quaternion xyzw |

base座標はX前・Y左・Z上。trajectoryの姿勢は現在のbaseからの相対回転とし、
quaternionは単位長へ正規化する。初版は未来姿勢を予測する単一の軌跡で、
障害物回避や速度出力との整合性は学習目標として別途設計する。
BEVは任意のXYZを照会するNeural Fieldで、格子全体の出力を固定サイズのheadに埋め込まない。
`grid_queries()`でセル中心を列挙し、出力を`[B,Z,Y,X,32]`にreshapeできる。

`SensorTokens`は`features=[B,N,C]`、任意の`valid=bool[B,N]`、
位置`positions=[B,N,P]`、観測の経過時間`age_s=[B]`または`[B,N]`を持つ。
RGB・depthには`image_patches()`を使える。各センサーは省略可能で、全欠損にも対応する。
欠測はmaskで示し、有効値の正規化・LiDARの選択/集約・時刻合わせ・座標変換は入力側が行う。
実際のRGB-D/LiDAR取得、カメラ較正の反映、センサーからの学習データ収集は今後接続する。

再帰状態は呼び出し側が明示的に渡す。基本の使い方:

```python
from pathlib import Path
import torch
from g1_e2e.models import ModelConfig, QueryRequest, RecurrentPerceiver, SensorTokens

model = RecurrentPerceiver(ModelConfig.load(Path("EndToEnd/configs/model/recurrent_perceiver.yaml")))
state = model.initial_state(batch_size=1)
observations = {"imu": SensorTokens(torch.zeros(1, 1, 10))}  # 形状例。実際は観測値を渡す
output = model(observations, state)  # query省略時はvelocityだけ読む
state = output.memory.state
xyz = torch.zeros(1, 100, 3)         # 任意の100地点に置き換える
features = model.decode(output.memory.latents, {"bev": QueryRequest(xyz)})["bev"]
```

`encode_step()`と`decode()`は別々に呼べる。速度は50 Hzを目標に読み出し、
密なBEVは必要な範囲・頻度で読む運用を想定する。query集合内のself-attentionは行わず、
推論時は地点の順序・分割方法によらず同じ場を返す。queryは既定で1024個ずつ処理する。
分割しても最終出力のサイズや、学習時に保持する勾配計算用メモリは地点数に依存するため、
学習では地点をサンプリングして損失を計算する。

`recurrence_period: 1`は毎周期の通常再帰。
`2`は「前回メモリを使用して更新→現在観測だけから表現を生成」を交互に行う実験設定。
観測のみの周期でも今回の表現からqueryを読み出し、次周期用の長期メモリは前回値を保持する。
この周期の観測は長期メモリへ保存されず、周期を変えるだけでは演算量も減らない。
この設定は意識研究から着想した仮説で、有効性を示すものではない。

並列episodeのresetは`reset_mask=bool[B]`でメモリと周期カウンターを一緒に初期化する。
時刻をまたぐ勾配は自動では切らない。学習時のtruncated BPTTの境界で`state.detach()`を呼び、
推論時は`model.eval()`と`torch.inference_mode()`を使う。

モデルだけの環境を作る場合は、リポジトリルートから実行する。シミュレーター用venvと分離する。

```bash
uv venv --python 3.12 EndToEnd/.venv-model
# CPUでモデル定義を確認する場合。GPU用PyTorchの選択は学習環境に合わせる
uv pip install --python EndToEnd/.venv-model/bin/python 'torch==2.11.0+cpu' --index https://download.pytorch.org/whl/cpu
uv pip install --python EndToEnd/.venv-model/bin/python -e 'EndToEnd[model,dev]'
EndToEnd/.venv-model/bin/python EndToEnd/examples/model_forward.py
EndToEnd/.venv-model/bin/python -m pytest EndToEnd/tests -q
```

現時点では未学習の構造定義。BEV特徴には表現を成立させる学習目標が別途必要で、
32次元とするだけでは3Dの意味や幾何は保証されない。
速度出力から29関節指令への低レベル歩行制御は別アダプターで接続する。
CPUでforward/backward・再帰・欠損・query分割を検証した。50 Hz達成とIsaac Sim 6.2.0との接続は未検証。
既存の`runtime.lock.json`は更新前に検証した環境の記録として残している。

## 構成

| ディレクトリ | 役割 |
|---|---|
| `src/g1_e2e/contracts.py`・`robot.py` | 共通の観測・指令・29関節対応 |
| `src/g1_e2e/inference/` | コントローラーのアダプター |
| `src/g1_e2e/models/` | 再帰メモリNN、観測トークン化、空間・時間query decoder |
| `src/g1_e2e/observations/`・`actions/`・`data/` | 今後の入力アダプター・出力変換・データ処理 |
| `sim/envs/`・`robots/` | モデルから独立したIsaac環境と物理モデル |
| `sim/play.py` | 任意コントローラーを接続し、実行・記録 |
| `sim/sensors/`・`scenes/` | 今後のセンサー・シーン追加先 |
| `train/` | 今後のデータ収集・学習入口 |
| `real/` | 実機との接続・実行 |
| `configs/robot/`・`task/` | 物理モデル・環境設定 |
| `configs/model/` | 再帰メモリ・観測・queryのモデル定義 |
| `configs/train/`・`eval/` | 今後の学習・評価設定 |
| `assets/` | 外部取得先とSHA-256のmanifest。`cache/`はGit管理外 |
| `examples/` | 共通APIへの接続例 |
| `tests/` | 実機仕様との一致・入力検査・関節対応の検証 |

## 実行記録と検証

実行ごとに`runs/<UTC時刻>/`へ次のファイルを保存する。

- `provenance.json`: コードのcommit・ソースhash・設定・Isaac/Lab/PyTorch版・アセット版。
- `trajectory.npz`: 指令前の観測、要求指令、制限後の指令、次の関節状態、終了フラグ。
- `summary.json`: ステップ数、転倒数、移動量、基部高さ、所要時間。

時刻はepisode開始からのsimulation time。同じ行の観測→指令→次の状態が対応する。
`--output PATH`で保存先を変更できる。使用済みの出力ファイルは上書きしない。
転倒・途中終了では終了コード1を返し、失敗を成功扱いにしない。
`--reset-on-done`で複数episodeを連続実行できる。

```bash
EndToEnd/.venv/bin/python -m pytest EndToEnd/tests -q
EndToEnd/.venv/bin/ruff check EndToEnd
```

2026-10-20の実動検証:
基部固定で100 step、独自の肩動作で200 step、公開モデルで前進500 step。
RTX 5060 Ti上で前進0.3 m/s指令・10秒を完走し、約2.67 m前進、転倒0回。
詳細な環境と結果は`configs/runtime.lock.json`および各runへ保存する。

## 開発方針

- 依存関係は `pyproject.toml` で管理し、このディレクトリ専用の環境を使う。
- 既存機能との接続は `sim/`・`real/` にまとめ、NN本体から通信やシミュレーターの詳細を分離する。
- データセット・重み・実験ログは `datasets/`・`checkpoints/`・`runs/` に保存し、Git管理外とする。
  再取得先、学習設定、使用したコードのコミットを記録して再現できるようにする。
