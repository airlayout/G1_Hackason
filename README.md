# G1 プロジェクト

Unitree G1 をデジタルツイン上で操作するプロジェクト。以下の 3 つがある。

- **マニュアル操作**（キーボードで歩行）— `IsaacSim_Env/`
- **2D Nav**（SLAM で地図作成 + Nav2 による自律ナビゲーション）— `IsaacSim_Env/`
- **3D（作りかけ）**（3D LiDAR + octomap による自律ナビゲーション）— `SimEnv3D/`

## 構成

| ディレクトリ | 内容 | 状態 |
|---|---|---|
| [IsaacSim_Env/](IsaacSim_Env/README.md) | キーボード操作 + 2D LiDAR による地図作成・Nav2 自律走行 | 動作確認済み |
| [SimEnv3D/](SimEnv3D/README.md) | 3D LiDAR（Livox Mid-360 相当）+ octomap による 3D 化 | 作りかけ（`IsaacSim_Env/` の後継として開発中） |
| [G1_HuggingFace/](G1_HuggingFace/README.md) | HuggingFace LeRobot + Unitree SDK (`unitree_sdk2py`) 経由の操作環境 | 動作確認済み（シミュレーション） |

環境設定・開発時の注意点は [CLAUDE.md](CLAUDE.md) を参照。

## 初期設定・環境構築

初めて動かす場合は [IsaacSim_Env/SETUP.md](IsaacSim_Env/SETUP.md) を読むこと。
必要な環境・インストール手順・最初の動作確認までをまとめてある。

**Claude Code などの AI エージェントに `IsaacSim_Env/SETUP.md` を読み込ませ、
その内容に沿って環境構築を行わせることを推奨する。**
