# Button_Press / RL の環境の作り方

ほかのマシン（ラボ PC など）で、学習の環境を作る手順。版は [configs/versions.yaml](configs/versions.yaml) に固定してある。

## 必要なもの

| 項目 | 内容 |
|---|---|
| OS | Ubuntu 22.04 / 24.04 |
| GPU | NVIDIA（RTX 5060 Ti 16GB、L40S 46GB で想定）。ドライバ **570 以上** |
| Python | 3.12（`sudo apt install python3.12 python3.12-venv`） |
| ディスク | 60 GB 程度（Isaac Sim が大きい） |
| ネットワーク | pypi.nvidia.com、github.com、download.pytorch.org に出られること |

## 手順

リポジトリを取得し、学習用のブランチにする。

```bash
git clone https://github.com/airlayout/G1_Hackason.git
cd G1_Hackason
git checkout Dev/ButtonPress_RL
```

Isaac Sim・Isaac Lab・rsl_rl を入れる（20〜40 分。すでにあるものは飛ばす）。

```bash
bash Button_Press/RL/scripts/setup.sh
```

既定の場所は `~/NVIDIA/env_isaaclab`（Isaac Sim の venv）と `~/NVIDIA/IsaacLab`。別の場所にするときは次のようにする。

```bash
ISAAC_SIM=/path/to/venv ISAACLAB=/path/to/IsaacLab bash Button_Press/RL/scripts/setup.sh
```

**別の場所にした場合は、以後のコマンドの前に毎回 `export ISAAC_SIM=... ISAACLAB=...` をする**
（`Button_Press/Yada/sim/isaac/env.sh` の既定の値は、このマシンのパス）。

`setup.sh` の最後に `check_setup.sh` が動き、すべて `[OK]` になればよい。次に動作を確かめる（起動に 2〜5 分）。

```bash
bash Button_Press/RL/scripts/train.sh --num_envs 16 --max_iterations 3
```

ログに `Learning iteration 2/3` まで出れば動いている。`Traceback` が出ていたら [FAILURES.md](FAILURES.md) を見る。

## setup.sh がすること

1. GPU とドライバの版を確かめる
2. Python 3.12 の venv を作り、Isaac Sim（pip 版）を入れる。venv に `python.sh`（薄いラッパー）を置く
   （pip 版には無いため。[IsaacSim_Env/SETUP.md](../../IsaacSim_Env/SETUP.md) の「環境の例②」と同じ）
3. Isaac Lab を取得し、決めたコミットにする。`./isaaclab.sh --install 'rl[rsl-rl]'` で入れる
   （torch は CUDA 12.8 版 `2.10.0+cu128` が入る）
4. G1 の公式モデルを取得する（`Button_Press/J1-gen/sim/fetch_models.sh`）

## 注意

- **torch や rsl-rl-lib を `pip install` で単独に入れ直さない。** 依存で torch が PyPI の版（CUDA 13 版）に入れ替わり、
  ドライバ 570 では GPU で動かなくなる（2026-10-19 に実際に起きた。[FAILURES.md](FAILURES.md)）。
  `pip check` が Isaac Sim と Isaac Lab の版の食い違いを出すが、これは Isaac Lab の入れ方の性質なので直さない
- 共有のマシンでは、Isaac Lab のアセットのキャッシュを自分のフォルダにする（`env.sh` が `TMPDIR` を設定する）
- 画面のあるマシンでは `play.sh` で学習した方策を見られる。画面が無ければ `--headless --video` で動画を残す
