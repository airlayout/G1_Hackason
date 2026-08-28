# G1 プロジェクト

Unitree G1 をデジタルツイン上で操作するプロジェクト。

## 構成

| ディレクトリ | 内容 | 状態 |
|---|---|---|
| [G1_HuggingFace/](G1_HuggingFace/README.md) | HuggingFace LeRobot + Unitree SDK (`unitree_sdk2py`) 経由の共通環境（操作PC/G1本体のPython環境構築） | 動作確認済み |
| [Common/](Common/) | 機能横断で使う共通スクリプト（ネットワーク設定・疎通確認など） | 運用中 |
| [SimpleWalk/](SimpleWalk/README.md) | 前進歩行 | 動作確認済み（シミュレーション・実機） |
| [Perception/](Perception/README.md) | 画像取得・認識 | 未着手 |
| [Mapping/](Mapping/README.md) | G1内蔵LIO／FAST-LIO2による3D Mapping | 実機で初回試験済み（onboard系。raw系は未実走） |
| [Navigation/](Navigation/README.md) | 作成済み地図を使った自律移動・巡回（Unitree純正`slam_operate`に乗る） | 未着手 |
| [IsaacSim_Env/](IsaacSim_Env/README.md) | キーボード操作 + 2D LiDAR による地図作成・Nav2 自律走行 | 当面使用しない |
| [SimEnv3D/](SimEnv3D/README.md) | 3D LiDAR（Livox Mid-360 相当）+ octomap による 3D 化 | 当面使用しない（作りかけ） |

`SimpleWalk/`・`Perception/`・`Mapping/`・`Navigation/`は、それぞれ`sim/`（シミュレーションでの
検証）と`real/`（実機デプロイ）に分けて開発する。`SimpleWalk/`で実践した
「シムで作る→実機で動かす」の流れをテンプレート化したもの。Mappingの実機用ROS 2環境は
再現性を優先してDockerへ隔離し、`G1_HuggingFace/`のvenvとは依存関係を共有しない。
各フォルダの`FAILURES.md`に、実際に起きた失敗と反省を記録する。

環境設定・開発時の注意点は [CLAUDE.md](CLAUDE.md) を参照。

## 開発環境の使い分け（2026-08-28 決定）

**Dockerを使うのは`Mapping/`だけ。他のフォルダでは使わない。**

| フォルダ | 環境 | 理由 |
|---|---|---|
| `Mapping/` | **Docker**（ROS 2 Humble） | LiDAR点群の高レート購読・rosbag記録・FAST-LIO2にROS 2が要る |
| それ以外すべて | `G1_HuggingFace/venv/`（Python 3.12） | ROS 2が不要なため |

G1への指令はDDSで送る。歩行（`LocoClient`）も純正SLAM/ナビ（`slam_operate`のAPI-ID）も
`unitree_sdk2py`から直接叩けるので、**制御するだけならROS 2もDockerも要らない**。
ROS 2が要るのは、ROS 2エコシステムの既製ノード（rosbag、RViz、FAST-LIO2など）を
使いたいときだけである。

開発環境のDockerイメージを8人全員へ配る案もあったが、ROS 2が必要なのはMapping班だけ
なので、不要な重さを配らないほうを選んだ。他のフォルダでROS 2が必要になったら、
その時点で`Mapping/real/`の構成を流用して判断し直す。

## 初期設定・環境構築

初めて動かす場合は [SETUP.md](SETUP.md) を読むこと。
操作PC側・G1本体側それぞれのPython環境構築手順を、ゼロから追える形でまとめてある。
検証済みの数値や既知のバグの詳細は[G1_HuggingFace/README.md](G1_HuggingFace/README.md)、
実際に作業した際の生ログは[G1_HuggingFace/Note](G1_HuggingFace/Note)を参照。

（`IsaacSim_Env/SETUP.md`はIsaac Sim版の手順だが、`IsaacSim_Env/`自体は当面使用しない）
