# VLA 失敗ログ

VLA を動かす際に実際に起きた失敗と、その原因・反省・再発防止策を記録する。
手順書（[sim/README.md](sim/README.md)）とは別に、**同じ失敗を繰り返さないための記録**として維持する。
新しい失敗が起きたら、下のテンプレートに沿って追記する。原因が確認できていないものは「未特定」と書く。

## テンプレート

```
## YYYY-MM-DD: 失敗の一言タイトル

**何が起きたか**
（観測された事実。憶測ではなく実際に見えたことを書く）

**原因**
（コード上・手順上の根本原因。未確認なら「未特定」）

**反省**
（なぜ事前に気づけなかったか。見落としていた前提は何か）

**再発防止策**
（実際にコード/手順をどう変えたか）
```

---

## 2026-10-07: pi0.5（LIBERO 用）の評価が RAM 不足で強制終了（omen）

**何が起きたか**
`lerobot/pi05_libero_finetuned_v044`（約 3B）で LeRobot の LIBERO 評価を回すと、プロセスが終了コード 137
（SIGKILL）で落ちた。bf16 に切り替えて再実行しても同じだった。omen のカーネルログに `Out of memory` が出ていた。

**原因**
omen の RAM は 15GB（swap 4GB）。モデルの読み込み段階で RAM が足りなくなったと見ている。
VRAM（16GB）は足りる見込みだったが、推論時の使用量は測っていない。

**反省**
GPU の VRAM だけを見て、RAM を見ていなかった。3B 級のモデルは、読み込み時に RAM も大きく使う。

**再発防止策**
3B 級の VLA を omen で動かす前に、RAM（15GB）を前提に見積もる。この件で小さい VLA（X-VLA）に切り替えた。

## 2026-10-08: `uv sync` が flash-attn で失敗する（nvcc が無い）

**何が起きたか**
`uv sync --extra gpu` が flash-attn のビルドで `CUDA_HOME` が未設定というエラーで止まった。

**原因**
omen に CUDA Toolkit（nvcc）が無い。flash-attn はソースからビルドされる。

**反省**
omen に nvcc が無いことを、導入前に確認していなかった。

**再発防止策**
`--no-install-package flash-attn` で外し、attention は SDPA に切り替えた。
[setup_policy_env.sh](sim/scripts/setup_policy_env.sh) と [patch_no_flash_attn.py](sim/scripts/patch_no_flash_attn.py)。

## 2026-10-08: torch が sm_120（RTX 5060 Ti）に対応していない

**何が起きたか**
`UV_TORCH_BACKEND=cu128` を付けて `uv sync` したのに、入った torch は cu126 ビルドで、sm_120 に対応していなかった。

**原因**
`uv sync` が、指定した backend ではなく cu126 を選んだ。理由までは調べていない（未特定）。

**反省**
入った torch のビルドを確認せず、先へ進みかけた。

**再発防止策**
sync の後に torch 2.7.1 / torchvision 0.22.1 を cu128 の index から入れ直し、GPU 上で行列積まで実行して確認する。
同じ [setup_policy_env.sh](sim/scripts/setup_policy_env.sh)。**この venv では `uv sync` を再実行しない**（cu126 に戻る）。

## 2026-10-08: deepspeed が import で落ちる（CUDA_HOME）

**何が起きたか**
deepspeed の import が `MissingCUDAException`（`CUDA_HOME` 未設定）で失敗した。

**原因**
deepspeed が import 時に CUDA Toolkit を探す。omen には無い。

**反省**
推論だけなら deepspeed は要らない、と先に確認すればよかった。

**再発防止策**
venv から deepspeed を外した（gr00t のソースは直接 import していない）。同じ setup_policy_env.sh が外す。

## 2026-10-08: Eagle が FlashAttention2 を強制して読み込みに失敗する

**何が起きたか**
`AutoModel.from_config` が FlashAttention2 の ImportError で失敗した。

**原因**
`Eagle-Block2A-2B-v2` の `config.json` と `modeling_eagle3_vl.py` が、attention 実装に flash_attention_2 を指定し、
さらに assert で強制していた。`eagle_backbone.py` にも同様の assert がある。

**反省**
sync で flash-attn を外した時点で、読み込み側にも手が要ると気づくべきだった。

**再発防止策**
3 ファイルを SDPA に切り替えるパッチ [patch_no_flash_attn.py](sim/scripts/patch_no_flash_attn.py)。冪等で、
omen で実際に当てたファイルと同じ結果になることを確認している。

## 2026-10-08: omen に git-lfs が無かった

**何が起きたか**
omen に git-lfs が入っていなかった。公式の WBC セットアップは git submodule を使うため、LFS のファイルを取るには git-lfs が要ると判断し、
先に導入した（無い状態でセットアップを実行して失敗したかは、確認していない）。
導入後も、非対話の ssh では PATH に `~/.local/bin` が入らず、omen 上の `git diff` が git-lfs を見つけられなかった。

**原因**
omen に git-lfs が無く、sudo は使わない方針だった。非対話シェルは `~/.local/bin` を PATH に含めない。

**反省**
公式スクリプトの前提を、実行前に確認していなかった。

**再発防止策**
[install_git_lfs.sh](sim/scripts/install_git_lfs.sh) で `~/.local/bin` に入れる。他のスクリプトは [env.sh](sim/scripts/env.sh) で PATH に足している。

## 2026-10-08: `pkill -f` を ssh 越しに実行すると自分のシェルが落ちる

**何が起きたか**
`ssh omen "pkill -f run_gr00t_server.py"` が終了コード 255 で終わり、接続が切れた。

**原因**
`pkill -f` はコマンドライン全体に一致させるため、ssh が起動したシェル自身（コマンドラインに同じ文字列を含む）にも一致したと見ている。

**反省**
後片付けの 1 行を、動作確認なしで書いた。

**再発防止策**
サーバーは起動時に PID を控え、その PID で止める（[run_eval.sh](sim/scripts/run_eval.sh)。終了時は trap でも止める）。

## 2026-10-08: 公式 checkpoint が再現しなかった（確認不足のまま採用）

**何が起きたか**
`nvidia/GR00T-N1.6-G1-PnPAppleToPlate`（公式）を 1 回回して失敗した（両手を伸ばすが、リンゴをつかめない）。
後から調べると、NVIDIA/Isaac-GR00T の issue #574 に、公式 checkpoint で 0/11 という報告があった。

**原因**
未特定。issue の報告者は「公開された重みが公開パイプラインと違うのでは」と疑っている。こちらの SDPA 置き換えが原因かは、
切り分けていない。ただし cloudwalk の checkpoint は同じ環境で 1/3 成功したので、SDPA が一律に壊しているわけではなさそうだ。

**反省**
checkpoint を選ぶ前に、issue を見ていなかった。「公式だから動く」と思い込んでいた。

**再発防止策**
VLA の checkpoint を採用する前に、リポジトリの issue と Hugging Face のコミュニティ欄で再現報告を確認する。

## 2026-10-08: 評価スクリプトが action / state を保存しない

**何が起きたか**
rollout の出力は mp4 と成功率だけで、各ステップの action・state・画像の生データが残らない。
このため、失敗の原因（issue #574 にある「左腕が後ろに回る」「手が開閉しない」が起きているか）を、数値で確かめられなかった。

**原因**
Isaac-GR00T の `rollout_policy.py` の仕様。

**反省**
生データが無いと後から解析できない。実行前に気づけたはずだった。

**再発防止策**
未対応。次に VLA を回すときは、先に action / state / 画像を記録する rollout を用意する
（[README](README.md) の「次にやるなら」）。
