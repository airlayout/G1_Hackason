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

## 構成

| ディレクトリ | 役割 |
|---|---|
| `src/g1_e2e/contracts.py`・`robot.py` | 共通の観測・指令・29関節対応 |
| `src/g1_e2e/inference/` | コントローラーのアダプター |
| `src/g1_e2e/models/` | 今後のNN本体 |
| `src/g1_e2e/observations/`・`actions/`・`data/` | 今後の入力アダプター・出力変換・データ処理 |
| `sim/envs/`・`robots/` | モデルから独立したIsaac環境と物理モデル |
| `sim/play.py` | 任意コントローラーを接続し、実行・記録 |
| `sim/sensors/`・`scenes/` | 今後のセンサー・シーン追加先 |
| `train/` | 今後のデータ収集・学習入口 |
| `real/` | 実機との接続・実行 |
| `configs/robot/`・`task/` | 物理モデル・環境設定 |
| `configs/model/` | 今後のモデル設定 |
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
