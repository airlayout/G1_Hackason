# Button_Press / RL の環境の作り方

自分のマシン（ラボ PC など）に、ボタン押しの強化学習の環境を作る手順。
この手順のとおりに進めると、次のことができるようになる。

- 学習済みの方策で、ボタン押しを評価環境（MuJoCo・Isaac Sim）で採点する
- Isaac Lab で教師を学習する（一からでも、続きからでも）
- 学習していく様子の動画を作る

使う版は [configs/versions.yaml](configs/versions.yaml) に固定してある。

| ソフト | 版 |
|---|---|
| Isaac Sim | 6.0.1.0（pip 版） |
| Isaac Lab | `release/3.0.0-beta2`（コミット `2e44ddb2e`） |
| rsl_rl | 5.0.1 |
| torch | 2.10.0+cu128 |
| Python | 3.12 |

## 1. 必要なもの

| 項目 | 内容 |
|---|---|
| OS | Ubuntu 22.04 / 24.04（24.04 で確認） |
| GPU | NVIDIA の GPU。ドライバ **570 以上**（`nvidia-smi` で確かめる）。L40S 46GB で確認、RTX 5060 Ti 16GB を想定 |
| Python | 3.12。無ければ `sudo apt install python3.12 python3.12-venv` |
| ディスク | 50 GB 以上の空き（Isaac Sim の環境だけで約 33 GB） |
| ネットワーク | pypi.nvidia.com、download.pytorch.org、github.com に出られること |
| そのほか | `git`、`ffmpeg`（動画を作るときだけ。`sudo apt install ffmpeg`） |

## 2. 手順

### 2.1 リポジトリを取得する

```bash
git clone https://github.com/airlayout/G1_Hackason.git
```

```bash
cd G1_Hackason
```

```bash
git checkout Dev/ButtonPress_RL
```

以下のコマンドは、すべてリポジトリの直下（`G1_Hackason/`）で実行する。

### 2.2 Isaac Sim・Isaac Lab・rsl_rl を入れる

```bash
bash Button_Press/RL/scripts/setup.sh
```

- 20〜40 分かかる（大半は Isaac Sim のダウンロード）。途中で止まっても、もう一度実行すれば、入っているものは飛ばして続きから入れる。
- 入る場所は `~/NVIDIA/env_isaaclab`（Isaac Sim の Python 環境）と `~/NVIDIA/IsaacLab`。
- 最後に `check_setup.sh` が動く。すべて `[OK]` なら成功。

別の場所に入れたいときは、場所を指定して実行する。

```bash
ISAAC_SIM=/path/to/env_isaaclab ISAACLAB=/path/to/IsaacLab bash Button_Press/RL/scripts/setup.sh
```

**別の場所にした場合は、このあとのコマンドを実行する前に、毎回同じ 2 つを `export` する。**
（`~/.bashrc` に書いておくと楽。）

```bash
export ISAAC_SIM=/path/to/env_isaaclab ISAACLAB=/path/to/IsaacLab
```

### 2.3 版がそろっているかを確かめる（数秒）

```bash
bash Button_Press/RL/scripts/check_setup.sh
```

GPU とドライバ、Isaac Sim・rsl_rl・torch の版、Isaac Lab のコミット、G1 のモデル、テストを確かめる。
すべて `[OK]` ならよい。`[NG]` があれば、その行に直し方が出る。

### 2.4 学習環境が動くかを確かめる（5 分ほど）

```bash
bash Button_Press/RL/scripts/train.sh --num_envs 16 --max_iterations 3
```

- Isaac Sim の起動に 2〜5 分かかる。初めての起動はもっとかかることがある（シェーダーの準備のため）。
- ログに `Learning iteration 2/3` まで出れば成功。
- ⚠️ 途中で失敗しても、終了コードは 0 になることがある。ログに `Traceback` が無いかを必ず見る（[7.](#7-うまくいかないとき)）。

### 2.5 学習済みの方策で採点する（数分）

学習済みの方策（`agent/policy.pt`）がリポジトリに入っているので、学習しなくても採点できる。

MuJoCo（速い。20 試行で 1 分ほど）:

```bash
$HOME/NVIDIA/env_isaaclab/python.sh Button_Press/Yada/contest/evaluate.py --agent Button_Press/RL/agent --target down --trials 20
```

Isaac Sim（1 試行に 20〜50 秒）:

```bash
bash Button_Press/Yada/contest/evaluate_isaac.sh --agent Button_Press/RL/agent --target down --trials 5
```

- `--target down` は「▼ を押す試行だけ」にする指定。この方策は一般用の ▼ だけを押す。
- 結果は `_local/button_press_yada/results/` に保存される。`--report` を付けると弱点のレポートも作る。
- `--set realistic` で、ノイズ・遅れ・体の揺れを入れた評価になる。

ここまでできれば、環境づくりは完了。

## 3. 使い方

| やりたいこと | コマンド | かかる時間（L40S） |
|---|---|---|
| 教師を一から学習する | `bash Button_Press/RL/scripts/train.sh` | 2500 回で約 3.5 時間 |
| 続きから学習する | `bash Button_Press/RL/scripts/train.sh --resume --load_run <フォルダ名>` | |
| 学習の経過を見る | `$HOME/NVIDIA/env_isaaclab/python.sh -m tensorboard.main --logdir _local/button_press_rl/logs` | |
| 成功率と押す力の分布を測る | `bash Button_Press/RL/scripts/eval_teacher.sh --episodes 2000` | 数分 |
| 方策を評価用に書き出す | `bash Button_Press/RL/scripts/eval_teacher.sh --no-eval --export Button_Press/RL/agent/policy.pt` | 数分 |
| 学習した方策を画面で見る | `bash Button_Press/RL/scripts/play.sh --num_envs 16` | 画面のあるマシンだけ |
| 学習していく様子の動画を作る | `bash Button_Press/RL/scripts/record_progress.sh` | 約 30 分 |

- 学習のログと途中の方策（100 回ごと）は `_local/button_press_rl/logs/rsl_rl/button_press_teacher/<日時>/` に保存される（git の対象外）。
- 動画は `_local/button_press_rl/videos/` に保存される。
- 学習のログの見方と、報酬・観測の中身は [README.md](README.md) にまとめてある。

## 4. マシンごとの設定（並列の環境の数）

`train.sh` は、GPU の名前を見て、並列の環境の数を [configs/machines.yaml](configs/machines.yaml) から選ぶ。

| マシン | 並列の環境の数 | 確認 |
|---|---|---|
| L40S（46 GB） | 2048 | 確認済み |
| RTX 5060 Ti（16 GB） | 1024 | **未確認**（メモリから見積もった値） |
| それ以外 | 512 | 未確認 |

- メモリが足りないと `CUDA out of memory` で止まる。`--num_envs 512` のように半分にして試す。
- うまく動いた値が分かったら、`configs/machines.yaml` を直してコミットしてほしい。
- 並列の数を減らすと、1 回の学習で集める経験が減るので、同じ回数でも上達が遅くなる。

## 5. setup.sh がすること

1. GPU とドライバの版を確かめる。
2. Python 3.12 の環境（venv）を作り、Isaac Sim（pip 版）を入れる。pip 版には `python.sh` が無いので、薄いラッパーを置く
   （[IsaacSim_Env/SETUP.md](../../IsaacSim_Env/SETUP.md) の「環境の例②」と同じ）。
3. Isaac Lab を取得して決めたコミットにし、`./isaaclab.sh --install 'rl[rsl-rl]'` で入れる。torch は CUDA 12.8 版が入る。
4. G1 の公式モデルを取得する（`Button_Press/J1-gen/sim/fetch_models.sh`。J1-gen・Yada と共通）。

⚠️ `setup.sh` は、何も入っていない新しいマシンでの通しの実行を、まだ確かめていない。
このマシンでは、すでに入っているものを飛ばして何も変えないことだけを確かめた。
新しいマシンで試したら、結果（うまくいった・どこで止まった）を [FAILURES.md](FAILURES.md) に書いてほしい。

## 6. 守ること

- **torch や rsl-rl-lib を `pip install` で単独に入れ直さない。** 依存で torch が PyPI の版（CUDA 13 版）に入れ替わり、
  ドライバ 570 では GPU で動かなくなる（2026-10-19 に実際に起きた。[FAILURES.md](FAILURES.md)）。
  版が違えば `check_setup.sh` が知らせるので、そのときは相談する。
- `pip check` は、Isaac Sim と Isaac Lab の版の食い違いをいくつか表示するが、これは Isaac Lab の入れ方の性質なので直さない。
- Isaac Lab（`~/NVIDIA/IsaacLab`）のファイルは編集しない。学習環境の設定は、すべて `Button_Press/RL/` の中にある。
- 共有のマシンでは、ほかの人の Isaac Sim の環境に `pip install` をしない（全員の学習と評価が止まる）。

## 7. うまくいかないとき

| 症状 | 原因と直し方 |
|---|---|
| 終了コードは 0 なのに、学習が始まらない | Isaac Lab は Python が例外で止まっても 0 を返すことがある。ログの `Traceback` の行を見る |
| `CUDA out of memory` | 並列の環境の数を減らす（`--num_envs 512`）。ほかの GPU の処理（評価や動画）と同時に動かしていないかも見る |
| 起動に 10 分以上かかる | 初めての起動はシェーダーとアセットの準備で長い。2 回目からは 2〜5 分 |
| `PermissionError: ... /tmp/Assets` | 共有のマシンで、ほかの人の作ったキャッシュに書けない。`env.sh` が `TMPDIR` を `~/NVIDIA/.isaac_asset_cache` にする。自分で `TMPDIR` を `/tmp` にしていないか確かめる |
| `No space left on device for creating change watches` と出る | 害は無い（ファイルの変更の見張りの上限の警告）。無視してよい |
| 起動時に `blas_thread_shutdown` で落ちる | `env.sh` が `OPENBLAS_NUM_THREADS=1` などを設定する。`env.sh` を通さずに起動していないか確かめる |
| `check_setup.sh` で torch が `[NG]` | [6.](#6-守ること) の入れ替わり。自分で直さず相談する |
| 採点で `方策のファイルが無い` | `Button_Press/RL/agent/policy.pt` が無い。`git pull` するか、[3.](#3-使い方) の書き出しで作る |
| 学習の様子がおかしい | まず [FAILURES.md](FAILURES.md) を読む。これまでの失敗（報酬の抜け道、自己衝突の無効など）と直し方がある |
