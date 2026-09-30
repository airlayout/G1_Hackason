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

## 注意
- 本プロジェクト環境(Isaac Sim 6.0 / IsaacLab 3.0 beta2)とは各候補の版が合わない。MuJoCo 経由が確実。
- モーション追従だけではボール位置に適応できない。
- 未確認: RoboCup SDK の公開有無、2025 年に G1 を使ったチームの有無。

## 次の一手
G1-mjlab-soccer と HumanoidSoccer を clone し、重みの実在確認と MuJoCo 再生。
