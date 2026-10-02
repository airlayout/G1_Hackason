# 評価環境の困りごとのログ

`Button_Press/Yada` の評価環境を使っていて、スムーズにいかなかったことを記録する場所（ブランチ `log/eval-env`）。
「質問した」「回り道した」「文書と違った」など、小さなことでも書いてほしい。改善の材料にする。

## 書き方

どのブランチで作業していても、次のコマンドで書ける（今のブランチは切り替わらない）。

```bash
P=~/miniconda3/envs/lerobot/bin/python
$P Button_Press/Yada/tools/trouble_log.py add      # 質問に答えて 1 件書く（コミットまで）
$P Button_Press/Yada/tools/trouble_log.py push     # GitHub に送る
$P Button_Press/Yada/tools/trouble_log.py list     # 一覧
```

`Button_Press/Yada/tools/trouble_log.py` が無いブランチでは、評価環境のブランチ（`Dev/ButtonPress_Yada`）から
取り込むか、GitHub のこのブランチに、`eval_env_log/` の下へ直接ファイルを足してもよい（1 件 = 1 ファイル、
ファイル名は `日時_書いた人_分類_ランダムな4文字.md` のように英数字だけにする。下の項目をそろえる）。

## 1 件のファイルの中身

先頭の項目（`key: value`）は、一覧（`trouble_log.py list`）で使う。

| 項目 | 内容 |
|---|---|
| `title` | 題名（1 行） |
| `date`, `author` | 書いた日時と人（自動） |
| `category` | setup, docs, mujoco, isaac, sim_dds, agent, report, real, other |
| `severity` | blocker（進めなくなった）/ major（回り道した）/ minor（少し迷った） |
| `status` | 未対応 / 対応中 / 対応済み / 対応しない（対応した人が書き換える） |
| `branch`, `commit` | 書いたときに作業していたブランチと、そのコミット（自動） |

本文: 何をしようとしたか / 何が起きたか / 期待していたこと / どう回避したか / 再現のコマンド / エラーの文 / 環境（自動）。

## ぶつからないための決まり

- 1 件 = 1 ファイル。ほかの人のファイルは、`status` と「対応」の欄を書き換えるときだけ触る
- 一覧のファイルは置かない（`list` で作る）
