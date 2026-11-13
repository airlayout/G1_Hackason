# 知識の目次（docs/knowledge）

| ファイル | 内容 |
|---|---|
| [real_robot_notes.md](real_robot_notes.md) | このプロジェクト（G1・PC2・カメラ・エレベーター）で分かったことと数値 |
| [tech/shell_and_ssh.md](tech/shell_and_ssh.md) | `pkill -f` が自分のシェルを止める、ssh の設定で鍵が使われない、`!` でパスワードを入力できない、Claude が PC2 に ssh する手順（鍵の登録・オプション・起動したプロセスの扱い）、Claude にはターミナルの出力が見えない、ssh 先の `pkill -f` が自分を止める（再発）、無線の ssh から PC2 の押す処理を切り離して動かす（`onboard.sh`） |
| [tech/rl_training.md](tech/rl_training.md) | 強化学習でボトルを押した工夫と難しかった点: 成功判定の抜け穴、強打は打ち切りで直す、範囲を広げる前に腕が届くか計算する、弱い配置を学習だけ多く引く、探す時間は罰や条件の緩和では縮まなかった |
| [tech/pc2_onboard.md](tech/pc2_onboard.md) | PC2 で動かすときの注意: YOLO26 の ONNX は `end2end=False` で書き出し元の `.pt` と比べる、lowstate をコールバックで全部受けると 1 コアの 7 割 |
| [patterns/robot_arm_control.md](patterns/robot_arm_control.md) | 腕の制御・計画: 遅れは押す前の値を基準に、開始時から当たっている接触、斜めの机の箱、吊り下げと立った姿勢の違い、ボトルが滑ると遅れで当たったと分からない、腕を前へ出すと体が前傾する、「動いたか」の確認は指令を実際の位置へ寄せる動きでは外す、dry-run では戻る動きを確かめられない、当たったと見る遅れのしきい値は実機で大きく・押す力は遅れ×腕の硬さで頭打ち、推定トルクから押した力を出すときは重力を引く、IK の初期姿勢を変えると体に当たる位置でも届く（手首を曲げる形）、推定の力は空中でも 6〜9 N 出る（実機）、「立っているか」の値は本体のモードで変わる |
| [patterns/camera_checks.md](patterns/camera_checks.md) | 深度の床・机の面からカメラの向きを求めて別の方法で裏付ける、止まった画像の見つけ方、下向きの頭カメラで柱までの距離・向きを出すときの落とし穴（床は壁を取り除いてから、狭い床では直さない、段になった面は奥行きのヒストグラムで） |
| [patterns/sim_real_config.md](patterns/sim_real_config.md) | 実機だけの補正を置く場所、実機のデータをシミュレーションのテストが読んでいた件、観測の口で IMU を入れ忘れても既定値で気づかなかった件、シミュの手は白く実機の手は黒い（手の色で画像の判定の前提を分ける）、腰固定の評価環境で歩くには乗り場を逆にずらす・真値にも同じずらしを |
| [patterns/analysis_pitfalls.md](patterns/analysis_pitfalls.md) | AI が間違えた点: 仮定から出した結論を事実のように伝えた、原因を一つに決めつけた、人に貼ってもらう文字列を省略した |
| [domain/unitree_g1.md](domain/unitree_g1.md) | G1 の機体: 頭カメラの向きは首の傾きで変わる（通常 約 45°）、純正の歩行は遅い指令では歩き出さない（vx 0.3 で 1 秒約 23 cm、約 1.9 秒歩き続ける） |
| [mine/script_design.md](mine/script_design.md) | ユーザーの方針（未検証）: 毎回の処理と、その日に 1 回の前提を分ける |

`_wrap-log.md` は、知識をまとめた日と、そのとき記録した項目の一覧。
