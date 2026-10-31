# 知識の目次（docs/knowledge）

| ファイル | 内容 |
|---|---|
| [real_robot_notes.md](real_robot_notes.md) | このプロジェクト（G1・PC2・カメラ・エレベーター）で分かったことと数値 |
| [tech/shell_and_ssh.md](tech/shell_and_ssh.md) | `pkill -f` が自分のシェルを止める、ssh の設定で鍵が使われない、`!` でパスワードを入力できない、Claude が PC2 に ssh する手順（鍵の登録・オプション・起動したプロセスの扱い）、Claude にはターミナルの出力が見えない |
| [patterns/robot_arm_control.md](patterns/robot_arm_control.md) | 腕の制御・計画: 遅れは押す前の値を基準に、開始時から当たっている接触、斜めの机の箱、吊り下げと立った姿勢の違い、ボトルが滑ると遅れで当たったと分からない、腕を前へ出すと体が前傾する |
| [patterns/camera_checks.md](patterns/camera_checks.md) | 深度の床・机の面からカメラの向きを求めて別の方法で裏付ける、止まった画像の見つけ方 |
| [patterns/sim_real_config.md](patterns/sim_real_config.md) | 実機だけの補正を置く場所、実機のデータをシミュレーションのテストが読んでいた件 |
| [patterns/analysis_pitfalls.md](patterns/analysis_pitfalls.md) | AI が間違えた点: 仮定から出した結論を事実のように伝えた、原因を一つに決めつけた、人に貼ってもらう文字列を省略した |
| [domain/unitree_g1.md](domain/unitree_g1.md) | G1 の機体: 頭カメラの向きは首の傾きで変わる（通常 約 45°） |
| [mine/script_design.md](mine/script_design.md) | ユーザーの方針（未検証）: 毎回の処理と、その日に 1 回の前提を分ける |

`_wrap-log.md` は、知識をまとめた日と、そのとき記録した項目の一覧。
