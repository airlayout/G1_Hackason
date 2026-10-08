# VLA / sim

G1 の sim で、学習済み VLA（GR00T N1.6）を動かす手順。実験の結果は
[experiments/2026-10-08-groot-n16-g1-apple-to-plate.md](experiments/2026-10-08-groot-n16-g1-apple-to-plate.md)。

## 構成

```
sim/
├── scripts/                 実行・導入・可視化のスクリプト（パスはすべて引数か環境変数。env.sh に既定値）
├── experiments/             実験記録（日付つき）
├── results/
│   ├── episodes.json        エピソード一覧（名前・動画・checkpoint・結果・所見）。make_rrd.py の入力
│   ├── videos/*.mp4         rollout の動画（4 本、計 16MB）
│   └── contact_sheets/*.jpg 動画から等間隔 6 フレームを並べた一覧
└── requirements-viewer.txt  可視化に要る Python 依存
```

## 動かす計算機の条件

動作を確認したのは次の 1 台だけ（それ以外は未確認）。この PC を、以降の文書では **omen**（ssh の接続名）と呼ぶ。

- omen: Ubuntu 24.04、NVIDIA RTX 5060 Ti 16GB（sm_120）、RAM 15GB、nvcc なし、sudo なし。
- `uv` が要る（無ければ [公式の手順](https://docs.astral.sh/uv/) で `~/.local/bin` に入れる）。
- GPU 世代が違えば、`setup_policy_env.sh` の `TORCH_BACKEND`（既定は cu128）を合わせる。例: `TORCH_BACKEND=cu126 bash setup_policy_env.sh`。
- 3B 級の VLA は RAM も要る。pi0.5 は RAM 15GB で落ちた（[FAILURES.md](../FAILURES.md)）。

## 1. Isaac-GR00T を用意する

```bash
git clone https://github.com/NVIDIA/Isaac-GR00T.git ~/Isaac-GR00T     # 場所は自由。GROOT_DIR で指定する
git -C ~/Isaac-GR00T checkout 7d5a455add459e870c2e4e4569006acace432d49  # 動作確認したコミット
```

以降のスクリプトは、`GROOT_DIR`（既定 `~/Isaac-GR00T`）の clone を使う。別の場所に置いたら、
各コマンドの前に `GROOT_DIR=/path/to/Isaac-GR00T` を付ける。他の設定値も同様に環境変数で変えられる（[env.sh](scripts/env.sh)）。

## 2. 環境を作る（sim を動かす PC で。初回のみ）

```bash
cd VLA/sim/scripts
bash install_git_lfs.sh        # git-lfs が無いとき（sudo 不要。~/.local/bin に入る。版と SHA-256 を固定。Linux x86_64 のみ）
bash setup_policy_env.sh       # 方策側の venv（torch は GPU に合わせた版へ入れ替え、GPU 計算まで確認する）
bash setup_wbc_sim.sh          # sim 側の venv（公式スクリプトを呼ぶ）
python3 -I patch_no_flash_attn.py   # flash-attn が無いときだけ。SDPA に切り替える（冪等）
```

注意:
- `setup_policy_env.sh` の後、方策側の venv で **`uv sync` を再実行しない**（torch が cu126 に戻る）。
- nvcc があり flash-attn を入れられる環境では、`patch_no_flash_attn.py` は不要。その場合の動作は未確認。

## 3. 動かす

```bash
cd VLA/sim/scripts
# 一括（サーバー起動 → 待機 → rollout → サーバー停止）。動画は /tmp/sim_eval_videos_*/ に出る
CKPT=cloudwalk-research/GR00T-N1.6-G1-PnPAppleToPlate N_EP=3 bash run_eval.sh

# 別々に動かすとき（2 つの端末で）
CKPT=nvidia/GR00T-N1.6-G1-PnPAppleToPlate bash run_server.sh
N_EP=1 MAX_STEPS=1440 bash run_client.sh
```

- サーバーは `POLICY_BIND_HOST`（既定 127.0.0.1）でだけ待ち受ける。認証は無いので、広げないこと。
  別の PC の sim から使うときだけ、サーバー側で `POLICY_BIND_HOST=0.0.0.0`、sim 側で `POLICY_HOST=<サーバーの IP>` を指定する（未確認）。
- 初回は checkpoint の取得で数分かかる（cloudwalk で約 355 秒）。
- 結果は、動画と、標準出力の成功率だけ。action / state は残らない。

## 4. 結果を見る（Rerun）

動画は、PC で Rerun の `.rrd` にして見る。可視化の依存は別に入れる（sim の PC には不要）。

```bash
python3 -m venv VLA/sim/.venv-viewer && VLA/sim/.venv-viewer/bin/pip install -r VLA/sim/requirements-viewer.txt

# 同梱の 4 本から作る（既定のパス）
VLA/sim/.venv-viewer/bin/python -I VLA/sim/scripts/make_rrd.py
VLA/sim/.venv-viewer/bin/rerun VLA/sim/results/g1_compare_tabs.rrd

# 自分で回した動画から作るとき: 動画を sim の PC から持ってきて（例: omen。uuid 名のディレクトリの下にある）、
# episodes.json を書いて --manifest / --videos-dir / --out を指定する
scp -r omen:/tmp/sim_eval_videos_gr00tlocomanip_g1_sim/ ./sim_videos/
```

- 画面中央のタブでエピソードを切り替える（✅ 成功 / ❌ 失敗 / 📋 まとめ）。時間軸は `sim_time`。
- `.rrd` は 68MB あり、リポジトリには入れていない（`.gitignore` 済み）。上のコマンドで作り直せる。
  作った 1 本は、omen の `~/vla_results/g1_compare_tabs.rrd` にもある（omen に入れる人だけ取れる）。
- すでに Rerun が起動していると、`rerun` コマンドは新しい窓を作らず、起動中の窓に流し込んで終わる。別窓で開くなら `--port auto` を付ける。
- 動画の一覧だけ見たいときは `make_contact_sheets.py --videos-dir DIR --out-dir DIR`。

## スクリプト一覧

| ファイル | 役割 |
|---|---|
| [env.sh](scripts/env.sh) | 共通設定（他のスクリプトが source）。パス・ポート・checkpoint の既定値 |
| [install_git_lfs.sh](scripts/install_git_lfs.sh) | git-lfs を sudo なしで導入 |
| [setup_policy_env.sh](scripts/setup_policy_env.sh) | 方策側の venv（torch の入れ替えと GPU 確認を含む） |
| [setup_wbc_sim.sh](scripts/setup_wbc_sim.sh) | sim 側（WBC）の venv |
| [patch_no_flash_attn.py](scripts/patch_no_flash_attn.py) | flash-attn なしで読めるようにするパッチ（冪等） |
| [run_server.sh](scripts/run_server.sh) / [run_client.sh](scripts/run_client.sh) / [run_eval.sh](scripts/run_eval.sh) | サーバー / rollout / 一括 |
| [make_rrd.py](scripts/make_rrd.py) | 動画 → Rerun の `.rrd`（タブ切り替え） |
| [make_contact_sheets.py](scripts/make_contact_sheets.py) | 動画 → 6 フレームの一覧 JPEG |

## 検証したこと

- `make_rrd.py` / `make_contact_sheets.py`: 同梱の 4 本で実行し、`.rrd` と JPEG ができることを確認した。
- `patch_no_flash_attn.py`: omen の Isaac-GR00T の未パッチのソース（`7d5a455`）に当て、omen で実際に当てたファイルと内容が一致し、2 回目は何も変わらないことを確認した。
- `run_eval.sh`: ダミーの `GROOT_DIR`（偽のサーバーと偽の rollout）で、起動 → 待機 → 実行 → サーバー停止の流れを確認した。
- **未実行**: `install_git_lfs.sh` / `setup_policy_env.sh` / `setup_wbc_sim.sh` と、本物のモデルでの `run_*.sh` の再実行。
  手元で動かした手順を、パスだけ引数にして移したもの。他の PC で初めて動かすときは、ここで詰まる可能性がある。
