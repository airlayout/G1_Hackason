# G1 サッカー調査(2026-09-30)

## 結論
「重みをDLして実機でサッカー」まで一気にできる公開物は無い。VLA は不向き
(UnifoLM-VLA は上半身の操作用、全身の動的制御は 50Hz 以上の RL 方策が担う)。
公式 [RoboCup ページ](https://www.unitree.com/robocup)(G1-Comp)にも学習済みサッカー方策の記載は無い。

## 候補(重みの実在は README/API の記載ベース。clone して未確認)
| 名前 | 重み | 環境 | 実機導線 | ライセンス |
|---|---|---|---|---|
| [RoboNaldo](https://github.com/OpenDriveLab/RoboNaldo) + [Deploy](https://github.com/OpenDriveLab/RoboNaldo_Deploy) | なし(自分で学習) | Isaac Sim 5.1 / Lab 2.3.2 | あり(DDS 50Hz)。LiDAR・RealSense・再帰反射ボールが必要 | MIT |
| [HumanoidSoccer](https://github.com/TeleHuman/HumanoidSoccer) | ONNX 同梱の記載あり。PAiD 本体は未公開 | Lab 2.1.1 / MuJoCo | README に無し | CC BY-NC |
| [G1-mjlab-soccer](https://github.com/q1781756566/G1-mjlab-soccer) | キーパー側のみ | MuJoCo | 無し | Apache-2.0 |
| [rfl-engine](https://github.com/robot-football-league/rfl-engine) | 歩行のみ(蹴りはルールベース) | MuJoCo | 無し | MIT |

## 実機デプロイ
- 歩行: `unitree_rl_lab` deploy / `unitree_rl_gym` deploy_real
- 蹴り: BeyondMimic + LAFAN1 retarget の kick 系モーション → ONNX。学習なしなら GR00T SONIC
- 前提: 有線接続、吊り下げ、オンボード制御の停止

## 実機でサッカーを実現するステップ
※各リポジトリの手順は調査時の要約に基づく。着手前に一次情報(README)で確認すること。

| # | ステップ | やること | 完了条件 |
|---|---|---|---|
| 0 | 準備 | 外付けPC(Ubuntu、G1と有線)、吊り下げ用の架台、コントローラ、ボール。認識用に LiDAR(Livox MID-360)+RealSense、または G1-Comp | 機材が揃い、PC から G1 に有線で疎通する |
| 1 | 候補の動作確認 | G1-mjlab-soccer / HumanoidSoccer を clone し、重みの実在を確認、MuJoCo で再生 | 重みが実在し、シムで蹴る |
| 2 | 歩行を実機で | `unitree_rl_gym` deploy_real か `unitree_rl_lab` deploy で歩行方策を実機へ。PC 側は静的IP(192.168.123.x)、オンボード制御を停止(デバッグモード)。吊り下げ→接地の順 | 速度指令で歩ける |
| 3 | 認識 | ボール検出と位置推定(YOLO + 深度、または LiDAR)。RoboNaldo は再帰反射ボール+AprilTag(ゴール)+カルマンフィルタ | ボール位置を胴体座標で安定取得 |
| 4 | 蹴り方策 | (A) RoboNaldo を学習(別環境: Isaac Sim 5.1 / Lab 2.3.2)→ ONNX。(B) BeyondMimic + LAFAN1 の kick モーションで学習→ ONNX。(C) 学習なしなら GR00T SONIC | シムで静止球を蹴る |
| 5 | sim2sim | MuJoCo で実機用コードと同じ経路を通す(RoboNaldo_Deploy の `deploy_mujoco.py` 等) | デプロイコードがシムで動く |
| 6 | 実機で蹴る | 吊り下げで動作確認→接地→静止球→認識連動。DDS 50Hz。RoboNaldo_Deploy は Orin NX の G1 非対応と明記のため外付けPCで動かす | 実機で静止球を蹴れる |
| 7 | 判断層 | 「探す→近づく→蹴る」のステートマシン。rfl-engine の行動 API(go_to_ball / kick_toward)を参考に、歩行(2)・認識(3)・蹴り(4)を束ねる | ボールを見つけて自動で蹴る |

安全: 常に吊り下げから開始。非常停止手段を手元に置く。転倒時に人が近づかない。

## 注意
- 本プロジェクト環境(Isaac Sim 6.0 / IsaacLab 3.0 beta2)とは各候補の版が合わない。MuJoCo 経由が確実。
- モーション追従だけではボール位置に適応できない。
- 未確認: RoboCup SDK の公開有無、2025 年に G1 を使ったチームの有無。

## 次の一手
上記ステップ1(clone、重みの実在確認、MuJoCo 再生)。
