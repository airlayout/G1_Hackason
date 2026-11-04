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

## 2026-11-01 / G1 ボタン押し（Button_Press/J1-gen）

対象: 2026-10-31〜11-01 のセッション全体（ノート PC から Wi-Fi で PC2 に ssh。onboard の見積もり、PC2 の調査・計画の時間、lowstate の受け取り、ボタンと人の YOLO の ONNX）

- tech/pc2_onboard.md（新規）: YOLO26 の ONNX は `end2end=False` で書き出し、元の `.pt` と比べる
- tech/pc2_onboard.md（新規）: lowstate をコールバックで全部受けると 1 コアの 7 割（最新 1 件だけ持つ DataReader で 10%）
- tech/shell_and_ssh.md: `pkill -f` が自分を止める件の再発（ssh 先の `bash -c`）
- real_robot_notes.md: 「2026-10-31 に分かったこと（PC2 で動かす準備）」（PC2 の中なら DDS が届く、DDS のライブラリは lerobot にある、計画・YOLO の時間）
- 提案したが保存しなかった: なし

## 2026-11-01 / G1 ボタン押し（Button_Press/J1-gen）

対象: 10/31 の YOLO の撮り足し（点灯した ▲）、自動ラベルの直し、8 本での最終版、評価環境の 3 つ
- real_robot_notes.md: 点灯した ▲ の動画で確かめた結果、最終版 final8 の候補と `J1GEN_BUTTON_WEIGHTS`、点字を拾う注意
- 提案したが保存しなかった: なし

## 2026-11-01 / G1（Perception の人の検出、PC2 の Web カメラ）

対象: G1 の Web カメラで YOLO（CPU・TensorRT）、person_alert（写真を残して PC へ・映像の配信）、カメラの番号の入れ替わり、Wi-Fi の自動接続
- real_robot_notes.md: 「2026-10-31 に分かったこと（PC2 の Web カメラ・人の検出・Wi-Fi）」（カメラ 2 つと by-id、Mapping の配信の受け取り方、GPU の速さ、Fujitsu_free_Wi-Fi の自動接続を切った件）
- 提案したが保存しなかった: なし

## 2026-11-01 / G1 ボタン押し（Button_Press/J1-gen、10/31 の前日準備）

対象: 配置 A〜F の予想とシミュ、模擬ロボットで auto_press、判定基準、動画、reach_wide2.yaml、経由なしの地図
- real_robot_notes.md: 経由なしの地図（経由の姿勢は今も必要）、reach_wide2.yaml、「指先が実際に進んだ距離」の読み方（50 mm が上限、判定は当たって止まった＋動画）
- 提案したが保存しなかった: なし


## 2026-11-01 / G1 ボタン押し（Button_Press/J1-gen、onboard の実機日）

対象: 広げた設定を既定に、机の箱の余裕 1 cm、無線の ssh で PC2 の上で押す（onboard.sh）、NoMotionError の誤判定、置ける範囲の dry-run
- patterns/robot_arm_control.md: 「指令どおりに動いたか」の確認は、指令を実際の位置へ寄せるだけの動きでは外す
- patterns/robot_arm_control.md: dry-run では、中止して戻る動きは確かめられない（模擬ロボットで確かめる）
- tech/shell_and_ssh.md: 無線の ssh から、PC2 の上の押す処理を切り離して動かす（`real/onboard.sh`）
- real_robot_notes.md: 2026-11-01 に分かったこと（onboard、送る間隔、別の班の VLA、置ける範囲）
- 提案したが保存しなかった: （domain・mine は個人の知識の方で確認中）

## 2026-11-01 / G1 ボタン押し（Button_Press/J1-gen、エレベーターの実機日・午後）

対象: ノート PC から有線（WSL2 mirrored）、エレベーターの車いす用 ▼ を 5 回、ボタン用の補正、押す力の不足、テストと mirrored
- patterns/robot_arm_control.md: 当たったと見る遅れのしきい値は実機の空中の動きで超えない値に。押す力は遅れ × 腕の硬さで頭打ち
- real_robot_notes.md: 2026-11-01 午後に分かったこと（WSL2 の mirrored、`lo` のテスト、rgbd_server の Python、YOLO 15/15、補正、押す力の不足）
- 提案したが保存しなかった: （domain・mine は個人の知識の方で確認中）

- 2026-11-03 / Button_Press（J1-gen）
  対象: 11/1 に押せなかった理由の分析、指をまっすぐ向けて垂直に押す変更、立ち位置、ラボで空中のボタンを 40〜50 cm
  - real_robot_notes.md: 2026-11-03 に分かったこと（小指で押していた、まっすぐ押す・立ち位置、有線の口の名前、RealSense と Web カメラ、頭カメラの向き、空中のボタン）
  - patterns/robot_arm_control.md: 狙う点と最初に当たる部分は違う／腕を伸ばしきる近くの関節の速さの上限と戻りの速さ

- 2026-11-03（夜）/ Button_Press（J1-gen）
  対象: エレベーターでまっすぐ押した 17 回、押す力を弱める・手袋なし、点灯の判定、PC2 の配信
  - real_robot_notes.md: 2026-11-03 夜に分かったこと（15/17 点灯、力は既定で足りる、かごの到着で判定が狂う、配信の起動し直し、IP の重なり）
  - patterns/robot_arm_control.md: 力不足に見えても当たり方を疑う・柔らかい指は接触の判定を鈍らせる
