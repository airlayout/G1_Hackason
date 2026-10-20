# 開発ガイド

29関節G1の共通平地環境、センサー収集、再帰モデルのforwardを実装済み。
学習ループ・占有情報の予測head・損失・このNNによる歩行制御は未実装。
機体の仕様は[G1_IO_SPECIFICATION.md](../G1_IO_SPECIFICATION.md)、
実行環境の版は[configs/runtime.lock.json](../configs/runtime.lock.json)を参照。

## 構成と責務

```text
EndToEnd/
├── configs/                 # 機体・物理環境・センサー・モデルの設定
├── sim/
│   ├── envs/                # NNに依存しない物理環境と設定
│   ├── robots/              # Isaacへの機体定義
│   ├── sensors/             # RGB-D・LiDARの取得と収集用シーン
│   ├── runtime.py           # 起動設定・controller生成・再現情報
│   ├── play.py              # controllerの動作検証
│   └── collect.py           # 生の観測・行動・次状態の収集
├── src/g1_e2e/
│   ├── contracts.py         # 物理単位のObservation / JointCommand / Controller
│   ├── robot.py             # SDK順の29関節とシミュレーターの関節対応
│   ├── data/                # センサー設定・座標変換・HDF5の読み書き
│   ├── models/              # NN設定・入力変換・M履歴・attention・query
│   └── inference/           # 共通Controller契約を実装する推論処理
├── examples/                # 接続例とforward例
├── tests/                   # 共通契約・保存データ・モデルのCPUテスト
├── assets/                  # 取得元とハッシュ。取得物はcache/へ保存
└── docs/                    # この開発ガイド
```

生成物は`datasets/`・`checkpoints/`・`runs/`へ保存し、Gitへ含めない。
物理環境は`reset/step`だけを担当し、特徴量・NNの履歴・損失・報酬を所有しない。
新しいcontrollerは`reset()`と`act(Observation) -> JointCommand`、
factoryは`create_controller(*, robot, control_dt, options)`を実装する。

## 起動と収集

Isaac Sim 6.2.0とIsaac Labの既存インストールを使用する。
[runtime.env.example](../configs/runtime.env.example)を参考に、必要なパスとGPU番号だけを`.env`へ設定する。
依存関係は`uv.lock`、外部アセットは[manifest.json](../assets/manifest.json)で固定する。
取得元・ライセンスは[assets/README.md](../assets/README.md)を参照。

以下は`EndToEnd/`を作業ディレクトリとして実行する。

```bash
bash setup.sh --with-demo

# 関節指令の確認 / 公開ポリシーによる平地歩行 / 独自controllerの接続
bash run.sh --controller hold --fixed-base --steps 100
bash run.sh --controller unitree --velocity 0.3 0 0 --steps 500
bash run.sh --controller examples.arm_sweep:create_controller --fixed-base --steps 100

# 50 Hz × 500 step = シミュレーション上の10秒。既定で画面なし。
bash collect.sh --steps 500 --velocity 0.3 0 0 --output datasets/walk_001
```

収集先は新しいディレクトリを指定する。`--reset-on-done`で転倒・時間制限後も新しいepisodeを収集できる。
`--controller-options settings.json`で独自設定を渡す。`hold`は初期姿勢の位置目標を保持する。
`run.sh`は動作検証用の`trajectory.npz`・`provenance.json`・`summary.json`を保存し、
`collect.sh`は学習用の`dataset.h5`・`metadata.json`・`summary.json`を保存する。
収集はモデル定義へ依存せず、生の観測を保存する。

## 再帰モデル

[モデル設定](../configs/model/recurrent_perceiver.yaml)は幅256・4層・8 heads。
各modalityを共通幅へ写し、1つの出力tokenから観測・M履歴へcross-attentionする。
用途queryも共通のcross-attentionを使い、最後の投影で速度・空間特徴・軌跡を出力する。
観測全体やM履歴全体のself-attentionは行わない。

| 制御周期 | 今回の表現の生成 | 履歴への保存 | queryの参照先 |
|---|---|---|---|
| 0、2、4… | 観測O＋M履歴 → Mを1 token | Mを追加 | 今回のMまでを含むM履歴 |
| 1、3、5… | 観測Oのみ → Uを1 token | 保存しない | M履歴＋今回のU |

観測・queryは50 Hz、Mの生成・履歴追加は25 Hz。Uは次周期へ持ち越さない。
M履歴は最大4096 tokens（約163.84秒分）で、上限を超えた最古のMだけを除く。
非再帰周期の読み出しは最大4096M＋1U。今回のMを二重には含めない。
保存したMは変更せず、参照時に生成時刻からの経過周期のsin/cos符号を加える。

| query | 入力 | 出力 |
|---|---|---|
| `velocity` | 学習可能query | 身体座標の速度指令`vx, vy, yaw_rate` |
| `bev` | 現在のbase座標XYZ（m） | 学習する32次元の3D特徴 |
| `trajectory` | 未来時間Δt（s） | 現在のbaseに対する未来のXYZ＋単位四元数xyzw |

```python
state = model.initial_state(batch_size=1)
memory = model.encode_step(observations, state, reset_mask=reset_mask)
predictions = model.decode(memory, queries)
state = memory.state  # 次周期へ渡すM履歴。Uは持ち越さない。
```

`model(observations, state, queries)`でも同じ処理を行う。
保存済みMだけを調べる場合は`model.decode(state, queries)`を使う。
`decode()`は周期を進めず、空間queryは順序・集合・分割サイズに依存せず評価できる。
`reset_mask=bool[B]`は該当episodeの履歴を無効化し、周期を0へ戻す。
batch内の異なる位相と履歴長はvalid maskで扱う。
学習時は勾配を自動で切らないため、truncated BPTTの境界で`state.detach()`を使う。
推論には`model.eval()`と`torch.inference_mode()`を使う。

学習方針は共通表現とBEV・trajectory・velocityの同時学習。
空間特徴には、LiDARの反射点・確認できた自由空間の再構築から自己教師信号を与える。
占有情報はBEV特徴からheadで読み出す。未観測・無反射の領域を空きとみなさない。
速度指令の良さを決めるタスク目標と学習方式、損失の詳細は未決定。
外部の学習済み画像encoderを前提にしない。

## 保存データと入力変換

取り付け・解像度・周期は[センサー設定](../configs/sensors/g1_rgbd_lidar.yaml)で変更する。
周期は制御周期0.02秒の整数倍。取り付けは実機へ未校正の仮設定で、機体の質量・関節を変えない。

| 観測 | 保存内容・既定の周期 | 初期モデルの入力形状 |
|---|---|---|
| RGB | uint8、128×128、25 Hz | `[B,64,768]`：16×16パッチ、0〜1 |
| 深度 | m＋画素valid、RGBと同じ格子・周期 | `[B,64,512]`：深度＋validの2チャネル |
| LiDAR | 32 ring×360 beam、10 Hz | `[B,128,6]`：XYZ・intensity・ring・時刻offset |
| 関節 | SDK順のq・dq・推定tau、50 Hz | `[B,29,3]` |
| IMU | 姿勢・gyro・projected gravity、50 Hz | `[B,1,10]` |
| 前回action | 前周期に適用した位置目標、50 Hz | `[B,29,1]`。episode先頭は無効 |

カメラは理想RGB-D、LiDARは静的USD形状への瞬時360°raycast。
D435iの光学誤差やMID-360の走査方式・自己反射・動体・歪み・ノイズは再現しない。
LiDARのintensityは入射角のcosineによる幾何的な応答、点ごとの時刻offsetは0。
収集用の箱・壁は`landmarks: false`で外せる。LiDAR用メッシュは同じUSD形状から生成する。

HDF5には現在の`state`、`previous_action`、今回の`requested_action` / `applied_action`、
教師・評価用の`next`、生の`sensors/camera` / `sensors/lidar`、`calibration`を保存する。
低周期frameは複製せず、control行からindexで参照する。
センサーと身体状態は同じepisodeで`source_time <= observation_time`を満たすものだけを結合する。
座標変換は列ベクトルの4×4行列、baseはX前・Y左・Z上、camera opticalはX右・Y下・Z前、姿勢はxyzw。
無効な深度・LiDARの0は保存用の穴埋めなので、必ずvalidと組にして扱う。
COM比力も保存するが、モデルのIMU入力10次元には含めない。関節tauはimplicit PDの推定値。
commit済みの行だけを`DatasetReader`で読み、`complete`と転倒数は別々に確認する。

`g1_e2e.models.ModelInputAdapter`が指定されたmodalityだけを`SensorTokens`へ変換する。
今回のactionや`next`を入力に使わない。LiDARは保存したpose真値で現在のbase座標へ位置合わせする。
実機では同時刻までのodom等による因果的なpose推定へ置き換える。
RGB-Dは0.2秒、LiDARは0.3秒を超えた観測をmaskし、未来・別episodeのframeを拒否する。

## CPUでの確認

```bash
source scripts/runtime.sh
"$ISAAC_SIM_PATH/python.sh" examples/model_forward.py --steps 6
"$ISAAC_SIM_PATH/python.sh" examples/dataset_forward.py datasets/walk_001/dataset.h5 --steps 6
"$ISAAC_SIM_PATH/python.sh" -m pytest tests -q
.venv/bin/ruff check .
```

これらはシミュレーターを起動しない。独立環境では`pyproject.toml`のmodel・sim・dev依存を用意する。
未学習の出力は形状と接続の確認用で、歩行方策や予測精度の確認には使わない。
Isaac Sim 6.2.0の収集記録はあるが、このNNを歩行controllerとして使う動作と50 Hzの実時間性能は未検証。
ブラウザ・シミュレーターでの動作確認は[AGENTS.md](../AGENTS.md)に従って人間が行う。
