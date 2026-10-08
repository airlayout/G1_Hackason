# 2026-10-08 GR00T N1.6 で G1 に「リンゴを皿へ」を実行させた（sim・学習なし）

「VLA 体験会」に向けて、**学習せずに**、VLA を G1 の sim で動かして Rerun で見る実験。
Franka ではなく Unitree G1 で行うこと、というのが依頼だった。

**結論: 動かせた。ただし安定はしない。** cloudwalk の checkpoint で 3 回中 1 回成功、公式 checkpoint は 1 回中 0 回。

## 条件

| 項目 | 内容 |
|---|---|
| タスク | `pick up the apple, walk left and place the apple on the plate.`（リンゴを取り、左へ歩き、皿に置く） |
| sim | Isaac-GR00T 同梱の GR00T-WholeBodyControl（MuJoCo）。env: `gr00tlocomanip_g1_sim/LMPnPAppleToPlateDC_G1_gear_wbc`（robocasa 系のキッチン） |
| ロボット | G1 29DOF + ハンド（`g1_29dof_with_hand_rev_1_0`）。ハンドの機種は未確認（コード上は `InspireHands`） |
| VLA | GR00T N1.6（3B）。embodiment tag `UNITREE_G1`、`--use-sim-policy-wrapper` |
| Isaac-GR00T | `NVIDIA/Isaac-GR00T` コミット `7d5a455add459e870c2e4e4569006acace432d49` |
| rollout | `n_action_steps=20`、`max_episode_steps=1440`、`n_envs=1`（動画は 20fps なので最大 72 秒） |
| 計算機 | omen（Ubuntu 24.04、RTX 5060 Ti 16GB = sm_120、RAM 15GB、nvcc なし） |
| 環境の工夫 | torch 2.7.1+cu128、flash-attn なし（SDPA に置き換え）。経緯は [FAILURES.md](../../FAILURES.md) |

役割分担: VLA が上半身の動作と移動の指示を出し、下半身の歩行・バランスは WBC 側が担当する。

### 使った checkpoint

| 略称 | モデル | 備考 |
|---|---|---|
| nvidia 公式 | [`nvidia/GR00T-N1.6-G1-PnPAppleToPlate`](https://huggingface.co/nvidia/GR00T-N1.6-G1-PnPAppleToPlate) | 公式。issue #574 に再現できないとの報告あり |
| cloudwalk | [`cloudwalk-research/GR00T-N1.6-G1-PnPAppleToPlate`](https://huggingface.co/cloudwalk-research/GR00T-N1.6-G1-PnPAppleToPlate) | 第三者の微調整版。モデルカードの自己申告は 5/10 成功（映像なし） |

どちらも、**このタスク専用に微調整された checkpoint**（モデルカードの記述。こちらでは検証していない）。
したがって今回は**ゼロショットではない**。「学習なしで（推論だけ）動かした」が正確。
未知のタスクや、言い換えた指示文への汎化は試していない。

## 結果

| エピソード | checkpoint | 結果 | 長さ | 所見（等間隔 6 フレームを見た範囲） |
|---|---|---|---|---|
| `nvidia_official_ep0_FAIL` | nvidia 公式 | 失敗 | 36 秒 | 両手を伸ばすが、つかめない。歩き出さず時間切れ |
| `cloudwalk_ep_104b1afa_FAIL` | cloudwalk | 失敗 | 36 秒 | 両手を開いて近づけるが、触れない。後半は腕が横に大きく開く |
| `cloudwalk_ep_909f0c7a_FAIL` | cloudwalk | 失敗 | 36 秒 | つかんで左へ動き出すが、最後はリンゴが机の上にある。落とした可能性（推測） |
| `cloudwalk_ep_00b6b9e9_SUCCESS` | cloudwalk | **成功** | 22 秒 | つかんで持ち上げ、左へ歩き、皿に置く。完了して早く終わった |

- 成功の判定は sim 側の判定による。時間切れまでに判定が出なければ失敗。
- 動画ファイル名の末尾 `_s0` / `_s1` は、この 4 本では失敗 / 成功と一致している。命名規則は、コードでは確認していない。
- サーバーの起動は、cloudwalk で約 355 秒（checkpoint の取得を含む）。

映像は [results/videos/](../results/videos/)、等間隔 6 フレームの一覧は [results/contact_sheets/](../results/contact_sheets/)、
一覧表は [results/episodes.json](../results/episodes.json)。Rerun で見る手順は [sim/README.md](../README.md) の「結果を見る」。

## 分かったこと・言えないこと

**言えること**
- 学習済みの VLA を、学習なしで G1 の sim に載せ、人型が歩きながらリンゴを皿に運ぶところまで動かせた（1 回）。
- 同じ checkpoint・同じ指示でも、成功する回と失敗する回がある。失敗の様子も回ごとに違う（つかめない / 触れない / つかんだ後に運べない）。

**言えないこと**
- 成功率。3 回（公式は 1 回）では測れない。1/3 は「成功することがある」程度の意味しかない。
- 失敗の原因。開始状態の違いか、拡散モデルの出力のばらつきかは、切り分けていない。
- 公式 checkpoint の失敗が、issue #574 の症状（左腕が後ろに回る、手が開閉しない）と同じか。action / state を保存していないため、映像でしか見ていない。映像では「左腕が後ろ」は確認できなかった。
- SDPA への置き換えが結果に影響したか。cloudwalk が 1/3 成功したので一律には壊していないが、影響が無いとは言えない。

## 未確認のこと

- ハンドの機種（Dex3 か Inspire か）。
- リンゴの開始位置を sim がランダムにしているか。映像では、試行ごとに少し違って見えた。
- 乱数の固定（seed）の仕方。
- 映像の全編は見ていない。上の所見は、どれも等間隔 6 フレームのシートから読んだもの。

## 参考

- [NVIDIA/Isaac-GR00T issue #574](https://github.com/NVIDIA/Isaac-GR00T/issues/574): 公式 checkpoint で 0/11。
- [LeRobot の Unitree G1 ドキュメント](https://huggingface.co/docs/lerobot/main/en/unitree_g1): G1 の事前学習済みの例は `nepyope/sonic_walk`（pi0.5、歩行）のみで、操作の checkpoint や成功率の記載は無かった。

## 再現手順

[sim/README.md](../README.md) を参照。
