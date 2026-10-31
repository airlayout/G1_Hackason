# 知識をまとめた記録

## 2026-10-22 / G1 ボタン押し（Button_Press/J1-gen）

対象: 2026-10-21〜22 のセッション全体（ラボ PC で実機、頭カメラの向き、1 回目の押し込み、`auto_press.py`）

- tech/shell_and_ssh.md: `pkill -f` が、実行したシェル自身を止める
- tech/shell_and_ssh.md: `~/.ssh/config` の `PubkeyAuthentication no` で、登録した鍵が使われない
- tech/shell_and_ssh.md: パスワードを聞く ssh / scp は、Claude Code の `!` では動かない
- patterns/robot_arm_control.md: 接触を「遅れ」で見るときは、押す前からある遅れを基準にする（止める判定も、保持の位置も）
- patterns/robot_arm_control.md: 開始姿勢で、もう当たり判定に入っていることがある
- patterns/robot_arm_control.md: 斜めに置かれた机を 1 つの直方体にすると、腕の通り道で余計に厳しくなる
- patterns/robot_arm_control.md: 吊り下げ・脱力の状態で測った位置は、立った状態の位置と違う
- patterns/camera_checks.md: 深度の床・机の面から、カメラの向きを求める。別の方法でも裏付ける
- patterns/camera_checks.md: 画像が「止まっている」かは、2 枚の差がちょうど 0 かで分かる
- patterns/sim_real_config.md: 実機だけの補正は、実機用の設定ファイルに書く
- patterns/sim_real_config.md: 実機の日のデータを、シミュレーションのテストが読んでいた
- patterns/analysis_pitfalls.md: 仮定の値から出した結論を、事実のように伝えない
- patterns/analysis_pitfalls.md: 前回と値が違うとき、原因を一つに決めつけない
- real_robot_notes.md: 「2026-10-21〜22 に分かったこと」（腕の追従の遅れの実測、体の動き、当たり判定、吊り下げとの違い、
  YOLO の確信度、テストの `CYCLONEDDS_HOME`、リンク検査、`auto_press.py` の事前確認）と、押し込みの止め方の記述の更新
- domain/unitree_g1.md: 頭カメラの向きは、首の傾きで変わる（ユーザー承認）
- mine/script_design.md: 毎回の処理と、その日に 1 回の前提を分ける（ユーザー承認）
- 提案したが保存しなかった: 「次のステップの方針（Enter なしで最後まで流すやり方へ移る）」

## 2026-10-31 / G1 ボタン押し（Button_Press/J1-gen）

対象: 2026-10-31 のセッション全体（ラボ PC・有線。PC2 への ssh、カメラ配信の起動、既定・広げた設定でのボトル押し）

- tech/shell_and_ssh.md: Claude が G1 の PC2 に ssh する手順（鍵の登録・オプション・起動したプロセスの扱い）
- tech/shell_and_ssh.md: Claude には、ユーザーのターミナルの出力が見えない（保存されたログで裏を取る）
- patterns/robot_arm_control.md: ボトルが滑ると、遅れでは当たったと分からない（固定して測る）
- patterns/robot_arm_control.md: 腕を大きく前へ出すと、体が前へ傾くことがある（広げた設定の経由の姿勢）
- patterns/analysis_pitfalls.md: 人に貼ってもらう文字列を、省略して書いた
- real_robot_notes.md: 「2026-10-31 に分かったこと」（モード切り替えで PC2 が再起動しカメラ配信が消える、立ち直すと位置関係が変わる、
  骨盤の傾きの上限、追従の遅れ・押し込みの結果、広げた設定の初回、頭カメラの向き 50.2°、録画の大きさ）
- 提案したが保存しなかった: なし
