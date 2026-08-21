# G1 プロジェクト

Unitree G1 をデジタルツイン上で操作するプロジェクト。

## 構成

| ディレクトリ | 内容 | 状態 |
|---|---|---|
| [G1_HuggingFace/](G1_HuggingFace/README.md) | HuggingFace LeRobot + Unitree SDK (`unitree_sdk2py`) 経由の共通環境（操作PC/G1本体のPython環境構築） | 動作確認済み |
| [Common/](Common/) | 機能横断で使う共通スクリプト（ネットワーク設定・疎通確認など） | 運用中 |
| [SimpleWalk/](SimpleWalk/README.md) | 前進歩行 | 動作確認済み（シミュレーション・実機） |
| [Perception/](Perception/README.md) | 画像取得・認識 | 未着手 |
| [Mapping/](Mapping/README.md) | G1によるMap計測・作成 | 未着手 |
| [SLAM/](SLAM/README.md) | G1内SLAM機能 | 未着手 |
| [IsaacSim_Env/](IsaacSim_Env/README.md) | キーボード操作 + 2D LiDAR による地図作成・Nav2 自律走行 | 当面使用しない |
| [SimEnv3D/](SimEnv3D/README.md) | 3D LiDAR（Livox Mid-360 相当）+ octomap による 3D 化 | 当面使用しない（作りかけ） |

`SimpleWalk/`・`Perception/`・`Mapping/`・`SLAM/`は、それぞれ`sim/`（シミュレーションでの
検証）と`real/`（実機デプロイ）に分けて開発する。`SimpleWalk/`で実践した
「シムで作る→実機で動かす」の流れをテンプレート化したもの。いずれも
`G1_HuggingFace/`の環境（操作PC側venv・G1本体側conda環境）を共通で使う。
各フォルダの`FAILURES.md`に、実際に起きた失敗と反省を記録する。

環境設定・開発時の注意点は [CLAUDE.md](CLAUDE.md) を参照。

## 初期設定・環境構築

初めて動かす場合は [SETUP.md](SETUP.md) を読むこと。
操作PC側・G1本体側それぞれのPython環境構築手順を、ゼロから追える形でまとめてある。
検証済みの数値や既知のバグの詳細は[G1_HuggingFace/README.md](G1_HuggingFace/README.md)、
実際に作業した際の生ログは[G1_HuggingFace/Note](G1_HuggingFace/Note)を参照。

（`IsaacSim_Env/SETUP.md`はIsaac Sim版の手順だが、`IsaacSim_Env/`自体は当面使用しない）
