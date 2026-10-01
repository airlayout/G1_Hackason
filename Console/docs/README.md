# Console/docs 目次

Console（G1 開発コンソール）の資料。リンクがあるものは、リポジトリに入っている。

## 実機を触る前に読む
| 文書 | 内容 |
|---|---|
| [REAL_ROBOT_CHECKLIST.md](REAL_ROBOT_CHECKLIST.md) | 実機接続時の確認項目。PR #30 のマージ条件（A〜C）を含む |
| [NEXT_CHECKLIST.md](NEXT_CHECKLIST.md) | 前回取れなかった項目・未解決（D435i の RGB など）の確認手順。守ること（カメラサーバーは手動起動・停止のみ）を含む |

## 調査・分析（2026-09-30 取得のデータ）
| 文書 | 内容 |
|---|---|
| [G1_FINDINGS.md](G1_FINDINGS.md) | 実機調査の記録（確認済み／未確認を区別） |
| [g1_analysis.md](g1_analysis.md) | 取得データの分析（観測／推測／不明を区別） |
| [DEV_GUIDE_OFFLINE.md](DEV_GUIDE_OFFLINE.md) | ロボットに触れない間の開発ガイド、未解決課題 |

## Console の仕様
| 文書 | 内容 |
|---|---|
| [API.md](API.md) / [openapi.yaml](openapi.yaml) | API 定義（`api_spec.py` から生成。手で編集しない） |
| [REMOTE_CONTROLLER.md](REMOTE_CONTROLLER.md) | 物理リモコンの機能整理と、コンソールの位置づけ |
| [../README.md](../README.md) | Console の使い方 |

## 手元だけにある資料（git に入れない。他の人は見られない）
`.gitignore` で対象外、または未追加のため、上の文書からはリンクしていない。

- `g1_snapshot/` … Jetson から取った生データ一式（bash_history・ネットワーク情報を含む）。原本、変更禁止
- `g1_logs/` … DDS のライブログ（`live_20260930_234841.jsonl`、75MB）
- `g1_raw/` … 生データの台帳（`MANIFEST.md`）、原本コピー（`original/`）、伏せ字版（`redacted/`）。伏せ字は機械処理なので、共有前に目視確認する
- `unitree-g1-developer/` … Unitree 公式の開発者ドキュメント（00〜44）
- `../../Common/g1-safety/` … 安全・操作の資料（リモコン、起動・停止、制御レベル、ポップアップ、未確認事項）（PR #31。Common に置く）

取得データを再生成する方法は `../tools/snapshot.sh`（読み取り専用）を参照。
