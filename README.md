# G1 プロジェクト

Unitree G1 をデジタルツイン上で操作するプロジェクト。以下の 3 つがある。

- **マニュアル操作**（キーボードで歩行）— `SimEnvTest/`
- **2D Nav**（SLAM で地図作成 + Nav2 による自律ナビゲーション）— `SimEnvTest/`
- **3D（作りかけ）**（3D LiDAR + octomap による自律ナビゲーション）— `SimEnv3D/`

## 構成

| ディレクトリ | 内容 | 状態 |
|---|---|---|
| [SimEnvTest/](SimEnvTest/README.md) | キーボード操作 + 2D LiDAR による地図作成・Nav2 自律走行 | 動作確認済み |
| [SimEnv3D/](SimEnv3D/README.md) | 3D LiDAR（Livox Mid-360 相当）+ octomap による 3D 化 | 作りかけ（`SimEnvTest/` の後継として開発中） |

環境設定・開発時の注意点は [CLAUDE.md](CLAUDE.md) を参照。
