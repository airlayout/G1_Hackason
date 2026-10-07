# 開発エージェント向け指示

このリポジトリで作業するすべてのチャットに適用するGit運用ルール。

## Git運用

- 各班は長命の機能ブランチ `Dev/Mapping2`・`Dev/Navigation`・`Dev/Perception` で作業する。担当が分からない場合は勝手に選ばない。
- `main` への取り込みは必ずPR経由。`main` への直接pushは禁止。
- **コミットとpushは別々の操作にする。** `git commit && git push` のように連続実行しない。コミットまでで止め、内容を確認してから明示的にpushする。コミットの依頼だけではpushしない。
- PRのマージ方法は **Create a merge commit** に統一する。squash・rebase mergeは使わない。
- マージ後も機能ブランチを削除せず、同じブランチを使い続ける。GitHubの **Delete branch** も押さない。
- **Update branchはmerge commit版のみ使用する。rebase版は禁止。** これはリポジトリ設定では防げないため、操作時に必ず確認する。
- レビューは各自で行う。自分のPRを自分でマージしてよい。Required approvalsは不要。
- 作業再開時は、先に自分の機能ブランチへ `origin/main` を取り込む。未コミットの変更がある場合は保護し、他の作業を上書きしない。

```bash
git checkout <自分のブランチ>
git fetch origin
git merge origin/main
```

### 長命ブランチでmerge commitを使う理由

squash・rebase mergeではmain側に元とは別のコミットが作られ、元のブランチのコミットがmainの祖先にならない。そのまま同じブランチを使うと、mainを取り込む際に取り込み済みの変更でコンフリクトが出ることがある。
Update branchのrebase版もコミットのSHAを書き換えるため、既にそのブランチをpullしている人と履歴が食い違う。共有ブランチをrebaseして履歴を書き換えない。
squash運用はマージ後にブランチを削除し、毎回新しく作る運用とセットにする。本プロジェクトでは長命ブランチとmerge commitを選ぶ。

## リポジトリ設定について

以下はユーザーが示した **2026-09-01時点** の状態であり、現在の設定を確認した結果ではない。ルールの大半はGitHub設定で強制されていないため、設定任せにせず運用で守る。

| 設定 | 2026-09-01時点の状態 |
| --- | --- |
| squash / rebase merge | 無効化済み。Allow merge commitsのみ有効 |
| PR必須 | 未設定。mainへの直接pushが可能だが、運用では禁止 |
| マージ前のmainへの追従必須 | 未設定 |
| マージ後のブランチ自動削除 | オフ。維持する |

まずは運用で守り、実際に事故が起きたら設定で制限する方針。設定変更は勝手に行わない。Branch rulesetを設定する場合は次を守る。

- Enforcement statusを **Active** にし、Target branchesに **main** を指定する。どちらか欠けると適用されない。「Applies to 0 targets」に注意する。
- Require a pull request before mergingを有効化し、**Required approvalsは0**にする。自分のPRを自分でapproveできないため、1以上だと各自マージできない。
- Require status checks to passはRulesetsでは空にできないため、status checkを1つ以上用意して指定する。その上でRequire branches to be up to date before mergingを有効化する。
- **Require linear historyは有効化しない。** merge commitを禁止してしまうため。
