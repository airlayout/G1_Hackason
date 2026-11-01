# G1アームのπ0.5制御と指示UI

**既定モデルはG1向けFT済みの `xiaopeng-wu/pi05_unitree_g1`。**
推論をRTX 5060 Ti 16GB、カメラと腕の50Hz制御をG1のOrin NXに置く。
UIから指示を入力し、単眼RGBと身体29関節の実測値から左右の腕を動かす。

## 今回選んだ重み

| 項目 | 内容 |
|---|---|
| モデル | [xiaopeng-wu/pi05_unitree_g1](https://huggingface.co/xiaopeng-wu/pi05_unitree_g1) |
| 元モデル | `lerobot/pi05_base` |
| FTデータ | [nepyope/unitree_box_move_blue_full](https://huggingface.co/datasets/nepyope/unitree_box_move_blue_full) |
| データ仕様 | G1・550 episodes・30Hz・640×480のRGBカメラ1台 |
| 公開された学習設定 | 3,000 steps・action expertのみ学習 |
| 入力 | 身体29関節の角度と`observation.images.global_view` |
| 出力 | 腕14関節の絶対角度＋remote 4要素。実機には腕14関節だけ送る |
| 精度 | bf16。visionなど公式実装でfloat32の層は維持する |
| 固定した版 | [configs/vla/pi05.json](configs/vla/pi05.json)にモデル・データセット・tokenizer・LeRobotのrevisionを記録 |

公開モデルを比較し、現在の機体にある単眼カメラと29関節だけで入力を作れる重みを選んだ。
手首カメラの複製、固定値によるハンド状態の補完、手先姿勢の推定は行わない。
モデル名だけを差し替えず、**FT時の正規化統計・stateの順序・画像処理・actionの逆正規化**を利用する。

**箱移動という単一タスクでFTされたモデルであり、G1の汎用的な操作能力を保証する重みではない。**
学習データの指示は`move blue box back and forth between tables`。
UIの「箱を往復」とbenchmarkの既定指示はこれに合わせている。最初はこの英語指示で比較する。
ボタン押しや、学習時と異なる視点・物体・指示に対する改善は実機で比較する必要がある。
元モデルより必ず良くなるとは断定しない。箱を扱う動作から確認し、必要なら現場データで追加学習する。
ハンドの操作は今回も対象外。把持の成立には、実機の手先形状や追加制御との整合が必要になる。

## 移行・初回セットアップ

以前のGR00T用`.venv-vla`とモデルは残し、π0.5は専用の`.venv-pi05`へ導入する。
Isaac環境とG1側のJetPack・システムPyTorchは更新しない。

GPU PCの条件:

- Linux x86_64、Python 3.12、`uv`、RTX 5060 Ti 16GB。
- CUDA 12.8対応ドライバ。専用環境へ`torch==2.9.0+cu128`を入れる。
- 初回は空きディスク30GB以上。FT済み重みは約9.35GBで、CUDAの依存も別途必要。
- CPUでの重みロードにもメモリが必要。RAM 32GB以上を推奨する。
- [google/paligemma-3b-pt-224](https://huggingface.co/google/paligemma-3b-pt-224)の利用許可とHugging Face認証。
  ここから取得するのはtokenizerだけ。本体のPaliGemma重みは追加でダウンロードしない。

リポジトリのルートで実行する:

```bash
uvx --from 'huggingface-hub==1.6.0' hf auth login
bash EndToEnd/arm-setup.sh --model
```

`HF_TOKEN`による認証も可能。トークンを設定JSONやソースへ保存しない。
モデルは`assets/cache/vla/pi05-g1`に固定版ファイルへの参照を作る。HF共有キャッシュを書き換えない。
実装と依存は`requirements-pi05.in` / `requirements-pi05.txt`で固定する。
FlashAttention2、TensorRT、NVIDIAのGR00Tパッケージはπ0.5環境に不要。

**G1側の以前のブリッジが動いている場合も、HTTPとarm_sdkの契約は同じなので接続できる。**
G1へ最新版を配り直す場合は、次の配備手順を使う。

## G1側の準備

仕様は[G1_IO_SPECIFICATION.md](G1_IO_SPECIFICATION.md)の1・3・10章に基づく。
G1側ではnumpy、OpenCV、既存の`unitree_sdk2py`とCycloneDDSを利用し、GPU PC用requirementsは入れない。
配備用コードの構文はPython 3.8以降を想定している。

GPU PCで配備ファイルを作る:

```bash
bash EndToEnd/arm-pack.sh
scp EndToEnd/assets/cache/vla/g1-arm-bridge.tar.gz unitree@192.168.0.82:/tmp/
ssh unitree@192.168.0.82
```

以下はG1側で実行する。IPは調査時の値なので現場の値へ置き換える。

```bash
mkdir -p /home/unitree/work/G1_Hackason
tar -xzf /tmp/g1-arm-bridge.tar.gz -C /home/unitree/work/G1_Hackason
cd /home/unitree/work/G1_Hackason
python3 -c 'import numpy, cv2; print(numpy.__version__, cv2.__version__)'
ls -l /dev/v4l/by-id/
bash EndToEnd/arm-robot.sh --camera /dev/v4l/by-id/実際のRGBデバイス名
```

RGBカメラは640×480で使う。番号が変わる`/dev/videoN`より安定したby-id名を選ぶ。
既存のカメラ配信プロセスとUSBカメラを同時に開かない。
SDKが別の場所にある場合は`G1_SDK_PATH`、Pythonは`G1_ARM_PYTHON`で指定する。
最初のブリッジは送信なし。モーターのpublisherを作らず、状態とカメラだけ取得する。

## UIから実行する

GPU PCの別ターミナルで、機体のローカルポートを転送する:

```bash
ssh -N -L 8766:127.0.0.1:8766 unitree@192.168.0.82
```

UIを起動する:

```bash
bash EndToEnd/arm-ui.sh --vla
```

[http://127.0.0.1:8765](http://127.0.0.1:8765)を開く。
右上に`π0.5 · G1 FT`と表示され、指示、カメラ、関節角、推論時間、実行・停止を確認できる。
`Esc`で停止、`Ctrl+Enter` / `⌘+Enter`で実行する。
G1側を送信なしで起動していれば、本物のπ0.5を使っても腕へは送らない。

GPU選択と導入状態を確認する:

```bash
PYTHONPATH=EndToEnd/src EndToEnd/.venv-pi05/bin/python -m g1_e2e.vla.tools doctor
```

GPUが複数ある場合は16GBの方を選ぶ。例: `arm-ui.sh --vla --device cuda:0`。

腕の制御を開始せず、一度だけ実推論を計測する:

```bash
PYTHONPATH=EndToEnd/src EndToEnd/.venv-pi05/bin/python -m g1_e2e.vla.tools benchmark \
  --robot http://127.0.0.1:8766
```

`shape=[8,14]`、`robot_commands_sent=0`と、推論時間・VRAM使用量が出る。
π0.5の入出力と重みの互換性を検査し、重みや正規化統計が欠けている場合は停止する。
上流の重みロード処理には例外を警告のみで返す経路があるため、ここではstrictなロードを使い、
未学習のモデルで推論が続くことを防ぐ。

実機で動かす際は他の腕制御プロセスを終了し、機体を安定して立たせる。
最初は腕の周囲を空け、リモコンの非常停止を用意する。
G1側のブリッジを終了し、`--execute`を追加して起動する:

```bash
bash EndToEnd/arm-robot.sh --camera /dev/v4l/by-id/実際のRGBデバイス名 --execute
```

UIの「実送信」を確認してから指示する。`--execute`だけでは腕の制御を開始しない。
UIの停止はarm_sdkのweightを下げて制御を返す操作で、実機の非常停止とは別。
歩行・腰・ハンドには指令を送らず、`lowcmd`への自動切替や`ReleaseMode()`は行わない。

## 入出力と停止処理

```text
GPU PC                                              G1 / Orin NX
UI → π0.5 → 学習時の逆正規化 → 腕14関節 [rad] ─ HTTP → 50Hz制御 → arm_sdk
         ↑                                          ↓
         └──── 同じ観測のRGB画像と身体29関節 ─── カメラ / lowstate
```

- state: SDK順の身体29関節の実測角度[rad]。
- image: JPEGをRGBへ復元し、CHW・float32・0..1でLeRobotへ渡す。224角へのpad/resizeは公式policyが担当する。
- preprocessing: checkpointのQUANTILES統計で正規化し、π0.5のstate tokenと指示を作る。
- inference: 毎回`predict_action_chunk`で再計画する。前タスクのaction queueを引き継がない。
- postprocessing: 学習時の統計で物理単位へ復元した後、先頭14要素だけを抽出する。現在角度を加算しない。
- timing: 50ステップの予測から先頭8ステップを30Hzで実行し、G1側が50Hzで追従する。
  推論待ち中は直前の目標を保持するため、動作に間が空く可能性がある。

共通の制限値は各モデル設定の`arm`にある。既定値は以下のとおり。

| 条件 | 挙動 |
|---|---|
| NaN/Inf、次元・関節順の不一致、可動域外 | 拒否・停止 |
| 開始姿勢からの予測変化 | 各関節0.8radを超えたら停止 |
| 目標速度 | 各関節0.4rad/sまで |
| 状態が0.25秒途絶 / カメラが1秒途絶 | 停止・weightを返す |
| 指令が3秒途絶 / 推論元の観測が3秒超経過 | 停止・拒否 |
| 目標と実角度のずれが0.5rad超 | 停止 |
| 1回の実行が30秒経過 | 自動終了 |
| 停止後の予測・観測IDの再送 | 拒否 |

watchdogはG1側の独立した制御スレッドで動く。UIや推論の停止はGPU処理の終了を待たない。
`SIGINT` / `SIGTERM`ではweightを戻して終了する。
自己衝突・障害物回避・接触力制御は未実装で、モデル予測だけで接触安全性を判断しない。
機体側プロセスの強制終了・OS停止・電源断からのソフトウェア復帰は保証しない。

## デモとGR00Tとの比較

実機もモデルも使わず、UIと通信だけを見る場合:

```bash
bash EndToEnd/arm-setup.sh --ui
bash EndToEnd/arm-ui.sh --demo
```

DEMOは現在角度の保持だけを行い、π0.5の性能確認には使えない。

GR00Tを選択して比較する場合は、設定を明示する。旧モデルの環境と重みをそのまま利用できる:

```bash
bash EndToEnd/arm-ui.sh --vla --config EndToEnd/configs/vla/groot.json
```

GR00Tを再導入する場合は`arm-setup.sh --model --config EndToEnd/configs/vla/groot.json`を使う。
別のFT済み重みを使う場合は`--model-path /path/to/checkpoint`を指定できるが、
単眼・身体29関節・腕14＋remote 4の絶対角・30Hzの契約と、`dataset_info.json`と正規化統計が必要。
単に同じ次元であるだけの重みや、手首カメラ・ハンド状態を必要とする重みは受け付けない。

## 検証

```bash
EndToEnd/.venv/bin/python -m pytest EndToEnd/tests/vla EndToEnd/tests/test_dds.py -q
```

公開checkpointから取得した実際のconfig・前後処理・統計をfixtureとして保存している。
画像のRGB/CHW/値域、身体とactionの順序、remoteの除外、欠けた統計・重みの拒否を検証する。
π0.5とGR00Tの両方でHTTP実行と停止を検証する。
固定したLeRobotを入れたCPU環境では、813個すべての重みの名前・形状の一致と、
実際の保存済み正規化・逆正規化処理も検証した。これらは実重みの推論テストではない。
ブラウザ・シミュレータ・実機での動作確認は`AGENTS.md`に従い現場で行う。
GPUでの実重み推論と現場の成功率は未検証。導入後は送信なしのbenchmarkから確認する。
